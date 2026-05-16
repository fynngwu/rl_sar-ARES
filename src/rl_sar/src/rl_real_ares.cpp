/*
 * ARES RL Control ROS2 Node
 * Projected gravity from driver (Imu.linear_acceleration).
 * Motor topics use FL RL FR RR URDF order; driver node reorders.
 * Keyboard in 200Hz control loop: [0]=STOP [1]=himloco [2]=cts.
 * On policy switch: publishes new kp/kd via /motor_param_update.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "observations_struct.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "keyboard_helper.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

using Lock = std::lock_guard<std::mutex>;

static const std::map<char, std::string> kPolicyMap = {
    {'1', "ares_himloco/himloco"},
    {'2', "dogv2_cts/cts"}
};

enum class State { STOPPED, RUNNING };

class ARSNode : public rclcpp::Node
{
public:
    explicit ARSNode(const std::string& policy_name)
        : Node("ares_rl_node"),
          num_of_dofs_(12), dt_(0.005f), decimation_(4),
          lin_vel_scale_(2.0f), ang_vel_scale_(0.25f),
          dof_pos_scale_(1.0f), dof_vel_scale_(0.05f),
          gravity_vec_scale_(1.0f), clip_obs_(100.0f),
          policy_name_(policy_name)
    {
        using namespace std::placeholders;
        motor_command_pub_ = create_publisher<sensor_msgs::msg::JointState>("/motor_command", 10);
        motor_param_pub_   = create_publisher<sensor_msgs::msg::JointState>("/motor_param_update", 1);
        motor_feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>("/motor_feedback", 10, std::bind(&ARSNode::MotorFeedbackCallback, this, _1));
        imu_sub_  = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&ARSNode::ImuCallback, this, _1));
        xbox_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/xbox_vel", 10, std::bind(&ARSNode::XboxVelCallback, this, _1));

        if (!InitRL()) { RCLCPP_ERROR(get_logger(), "RL init failed!"); return; }

        loop_control_ = std::make_shared<LoopFunc>("loop_control",  dt_, std::bind(&ARSNode::RobotControl, this));
        loop_rl_ = std::make_shared<LoopFunc>("loop_rl", dt_*decimation_, std::bind(&ARSNode::RunModel, this));
        loop_control_->start();  loop_rl_->start();

        printf("\n=== ARES RL Controls ===\n  [0] STOP  [1] himloco  [2] cts\n  Switch policy from STOPPED\n\n");
    }

    ~ARSNode()
    {
        if (loop_control_) loop_control_->shutdown();
        if (loop_rl_) loop_rl_->shutdown();
        if (csv_file_.is_open()) csv_file_.close();
    }

private:
    // ---- Init ----------------------------------------------------------------
    bool InitRL()
    {
        std::string policy_dir(POLICY_DIR), cfg = policy_dir + "/" + policy_name_ + "/config.yaml", model_fn;
        RCLCPP_INFO(get_logger(), "Config: %s", cfg.c_str());
        try {
            YAML::Node root = YAML::LoadFile(cfg), rc = root[policy_name_];
#define GP(k,v) if (rc[#k]) v = rc[#k].as<decltype(v)>()
            GP(dt,dt_); GP(decimation,decimation_); GP(num_of_dofs,num_of_dofs_);
            GP(lin_vel_scale,lin_vel_scale_); GP(ang_vel_scale,ang_vel_scale_);
            GP(dof_pos_scale,dof_pos_scale_); GP(dof_vel_scale,dof_vel_scale_);
            GP(gravity_vec_scale,gravity_vec_scale_); GP(clip_obs,clip_obs_);

            if (rc["model_name"]) model_fn = rc["model_name"].as<std::string>();

            auto strs = [&](const char* k) { std::vector<std::string> v;
                if (rc[k]) for (const auto& e : rc[k]) v.push_back(e.as<std::string>()); return v; };
            observations_ = strs("observations");

            if (rc["observations_history"]) {
                observations_history_.clear();
                for (const auto& h : rc["observations_history"]) observations_history_.push_back(h.as<int>());
            }

            auto flt = [&](const char* k) { std::vector<float> v;
                if (rc[k]) for (const auto& e : rc[k]) v.push_back(e.as<float>()); return v; };
            clip_actions_upper_ = flt("clip_actions_upper");  clip_actions_lower_ = flt("clip_actions_lower");
            action_scale_ = flt("action_scale");  commands_scale_ = flt("commands_scale");  default_dof_pos_ = flt("default_dof_pos");

            if (rc["topic_to_driver"]) {
                auto t2d = rc["topic_to_driver"];
                std::array<int,12> a{};  for (int i=0;i<12;++i) a[i]=t2d[i].as<int>();
                for (int i=0;i<12;++i) driver_to_topic_[a[i]]=i;
            }

            auto farr = [&](const YAML::Node& n) -> std::vector<float> {
                if (n.IsSequence()) { std::vector<float> v; for (const auto& e : n) v.push_back(e.as<float>()); return v; }
                return std::vector<float>(num_of_dofs_, n.as<float>());
            };
            current_kp_ = farr(rc["fixed_kp"]);  current_kd_ = farr(rc["fixed_kd"]);  current_torque_limits_ = farr(rc["torque_limits"]);

            if (rc["record"] && rc["record"]["enabled"]) {
                if (csv_file_.is_open()) csv_file_.close();
                std::string fp = rc["record"]["filepath"].as<std::string>();
                csv_file_.open(fp, std::ios::out|std::ios::trunc);
                if (csv_file_.is_open()) {
                    record_enabled_ = true;  csv_file_ << "timestamp";
                    for (int i=0;i<12;++i) csv_file_<<","<<kJointNames[i]<<"_pos,"<<kJointNames[i]<<"_vel,"<<kJointNames[i]<<"_torque,"<<kJointNames[i]<<"_target";
                    csv_file_ << "\n";
                }
            } else record_enabled_ = false;
        } catch (const YAML::Exception& e) { RCLCPP_ERROR(get_logger(), "Bad config: %s", e.what()); return false; }

        EnsureVectorSizes();

        std::string mp = model_fn.empty() ? FindLatestOnnx(policy_dir+"/"+policy_name_+"/")
                                          : policy_dir+"/"+policy_name_+"/"+model_fn;
        if (mp.empty()) { RCLCPP_ERROR(get_logger(), "No .onnx"); return false; }

        RCLCPP_INFO(get_logger(), "Model: %s", mp.c_str());
        model_ = InferenceRuntime::ModelFactory::load_model(mp);
        if (!model_) { RCLCPP_ERROR(get_logger(), "Bad model"); return false; }

        ComputeObsDims();

        int hist = 1;
        if (!observations_history_.empty()) hist = *std::max_element(observations_history_.begin(),observations_history_.end())+1;
        history_obs_buf_ = ObservationBuffer(1, obs_dims_, hist, "time");

        int od = 0;  for (int d : obs_dims_) od += d;
        for (int i=0; i<hist; ++i) history_obs_buf_.insert(std::vector<float>(od,0));

        obs_.lin_vel={0,0,0}; obs_.ang_vel={0,0,0}; obs_.gravity_vec={0,0,-1}; obs_.commands={0,0,0};
        obs_.base_quat={1,0,0,0}; obs_.dof_pos=default_dof_pos_;
        obs_.dof_vel.assign(num_of_dofs_,0); obs_.actions.assign(num_of_dofs_,0);
        latest_target_pos_ = default_dof_pos_;

        rl_init_done_ = true;
        RCLCPP_INFO(get_logger(), "Init OK  dof=%d  hist=%d", num_of_dofs_, hist);
        return true;
    }

    // ---- Inference  (~50Hz) -------------------------------------------------
    void RunModel()
    {
        if (current_state_ == State::STOPPED || !rl_init_done_) return;

        {   Lock lk(data_mutex_);
            if (!all_sensors_ready_) return;
            obs_.ang_vel = {imu_gyro_[0], imu_gyro_[1], imu_gyro_[2]};
            obs_.gravity_vec = {imu_gravity_[0], imu_gravity_[1], imu_gravity_[2]};
            obs_.commands    = {commands_buffer_[0], commands_buffer_[1], commands_buffer_[2]};
            for (int i=0;i<num_of_dofs_;++i) { obs_.dof_pos[i]=joint_pos_[i]; obs_.dof_vel[i]=joint_vel_[i]; }
        }

        std::vector<float> actions = Forward(obs_);
        if (actions.empty()) return;
        obs_.actions = actions;

        {   Lock lk(output_mutex_);  latest_target_pos_ = ComputeTargetPositions(actions); }
        inference_count_++;

        if (record_enabled_) {
            Lock lk(data_mutex_);  csv_file_ << record_time_;
            for (int i=0;i<num_of_dofs_;++i) csv_file_<<","<<obs_.dof_pos[i]<<","<<obs_.dof_vel[i]<<","<<joint_torque_[i]<<","<<latest_target_pos_[i];
            csv_file_ << "\n";  record_time_ += dt_ * decimation_;
        }

        auto now = std::chrono::steady_clock::now();
        if (!last_print_time_ || std::chrono::duration<float>(now - *last_print_time_).count() >= 1.0f) {
            last_print_time_ = now;
            printf("\033[2J\033[H%-8s [%s] inf:%.1fms #%d  [0]=S [1]=H [2]=C\n",
                   current_state_==State::STOPPED?"STOPPED":"RUNNING", policy_name_.c_str(), inference_time_ms_, inference_count_);
        }
    }

    // ---- Control  (~200Hz, also checks keyboard) ----------------------------
    void RobotControl()
    {
        if (!rl_init_done_ || !all_sensors_ready_) return;

        int key = kbhit();

        if (key == '0' && current_state_ != State::STOPPED) {
            sensor_msgs::msg::JointState cmd;
            cmd.header.stamp = now();
            for (int i=0;i<num_of_dofs_;++i) cmd.position.push_back(default_dof_pos_[i]);
            motor_command_pub_->publish(cmd);
            current_state_ = State::STOPPED;
            printf("[RL] STOPPED\n");  return;
        }

        if ((key == '1' || key == '2') && current_state_ == State::STOPPED) {
            auto it = kPolicyMap.find(static_cast<char>(key));
            if (it != kPolicyMap.end()) {
                printf("[RL] Loading %s ...\n", it->second.c_str());
                rl_init_done_ = false;  policy_name_ = it->second;
                if (InitRL()) { all_sensors_ready_ = true;  PublishMotorParams();  current_state_ = State::RUNNING;
                    printf("[RL] RUNNING (%s)\n", policy_name_.c_str()); }
                else printf("[RL] Reinit FAILED\n");
            }  return;
        }

        if (current_state_ == State::STOPPED) return;

        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = now();
        {   Lock lk(output_mutex_);  for (int i=0;i<num_of_dofs_;++i) cmd.position.push_back(latest_target_pos_[i]); }
        motor_command_pub_->publish(cmd);
    }

    // ---- Forward ------------------------------------------------------------
    std::vector<float> Forward(const Observations<float>& obs_snap)
    {
        using Clock = std::chrono::high_resolution_clock;
        auto t0 = Clock::now();
        std::vector<float> clamped = ComputeObservation(obs_snap), actions;

        if (!observations_history_.empty()) {
            history_obs_buf_.insert(clamped);
            history_obs_ = history_obs_buf_.get_obs_vec(observations_history_);
            actions = model_->forward({history_obs_});
        } else actions = model_->forward({clamped});

        inference_time_ms_ = std::chrono::duration<double,std::milli>(Clock::now()-t0).count();

        if (actions.empty() || actions.size()!=static_cast<size_t>(num_of_dofs_)) { RCLCPP_ERROR(get_logger(),"Bad inf"); return {}; }
        for (float v : actions) if (std::isnan(v)||std::isinf(v)) { RCLCPP_ERROR(get_logger(),"NaN"); return {}; }

        for (size_t i=0;i<actions.size();++i) {
            float lo = i<clip_actions_lower_.size()?clip_actions_lower_[i]:-1.0f;
            float hi = i<clip_actions_upper_.size()?clip_actions_upper_[i]: 1.0f;
            actions[i] = std::max(lo, std::min(hi, actions[i]));
        }
        return actions;
    }

    // ---- Observation --------------------------------------------------------
    std::vector<float> ComputeObservation(const Observations<float>& os)
    {
        std::vector<float> ov;
        std::vector<float> gv = os.gravity_vec;  for (auto& v:gv) v*=gravity_vec_scale_;

        for (const std::string& n : observations_) {
            if (n=="commands") {
                std::vector<float> s = os.commands;
                for (size_t i=0;i<s.size()&&i<commands_scale_.size();++i) s[i]*=commands_scale_[i];
                ov.insert(ov.end(), s.begin(), s.end());
            } else if (n=="ang_vel") {
                for (float v:os.ang_vel) ov.push_back(v*ang_vel_scale_);
            } else if (n=="gravity_vec") {
                ov.insert(ov.end(), gv.begin(), gv.end());
            } else if (n=="dof_pos") {
                for (int i=0;i<num_of_dofs_;++i) ov.push_back((os.dof_pos[i]-default_dof_pos_[i])*dof_pos_scale_);
            } else if (n=="dof_vel") {
                for (int i=0;i<num_of_dofs_;++i) ov.push_back(os.dof_vel[i]*dof_vel_scale_);
            } else if (n=="actions") {
                ov.insert(ov.end(), os.actions.begin(), os.actions.end());
            }
        }
        for (auto& v:ov) v = std::max(-clip_obs_, std::min(clip_obs_, v));
        return ov;
    }

    // ---- ROS callbacks ------------------------------------------------------
    void MotorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Lock lk(data_mutex_);
        size_t n = std::min(msg->position.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i=0;i<n;++i) { joint_pos_[i]=msg->position[i]; joint_vel_[i]=msg->velocity[i]; joint_torque_[i]=msg->effort[i]; }
        if (!all_sensors_ready_ && imu_received_) all_sensors_ready_ = true;
    }

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Lock lk(data_mutex_);
        imu_gyro_[0]=msg->angular_velocity.x; imu_gyro_[1]=msg->angular_velocity.y; imu_gyro_[2]=msg->angular_velocity.z;
        imu_gravity_[0]=msg->linear_acceleration.x; imu_gravity_[1]=msg->linear_acceleration.y; imu_gravity_[2]=msg->linear_acceleration.z;
        if (!all_sensors_ready_ && motor_feedback_received_) all_sensors_ready_ = true;
    }

    void XboxVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        Lock lk(data_mutex_);
        commands_buffer_[0]=msg->linear.x; commands_buffer_[1]=msg->linear.y; commands_buffer_[2]=msg->angular.z;
    }

    // ---- Helpers ------------------------------------------------------------
    std::vector<float> ComputeTargetPositions(const std::vector<float>& acts) const
    {
        std::vector<float> out(num_of_dofs_);
        size_t n = std::min(acts.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i=0;i<n;++i) out[i]=acts[i]*action_scale_[i]+default_dof_pos_[i];
        for (size_t i=n;i<out.size();++i) out[i]=default_dof_pos_[i];
        return out;
    }

    void EnsureVectorSizes()
    {
        auto sz = static_cast<size_t>(num_of_dofs_);
        if (action_scale_.size()<sz) action_scale_.resize(sz,0.25f);
        if (commands_scale_.size()<3) commands_scale_.resize(3,1.0f);
        if (default_dof_pos_.size()<sz) default_dof_pos_.resize(sz,0.0f);
        if (clip_actions_upper_.size()<sz) clip_actions_upper_.resize(sz,1.0f);
        if (clip_actions_lower_.size()<sz) clip_actions_lower_.resize(sz,-1.0f);
    }

    void ComputeObsDims()
    {
        obs_dims_.clear();
        for (const std::string& n : observations_) {
            int d=0;
            if (n=="lin_vel"||n=="ang_vel"||n=="gravity_vec"||n=="commands") d=3;
            else if (n=="dof_pos"||n=="dof_vel"||n=="actions") d=num_of_dofs_;
            if (d>0) obs_dims_.push_back(d);
        }
    }

    std::string FindLatestOnnx(const std::string& dir)
    {
        namespace fs = std::filesystem;
        std::string best;  fs::file_time_type bt;
        try {
            if (!fs::exists(dir)) return "";
            for (const auto& e : fs::directory_iterator(dir)) {
                if (!e.is_regular_file()) continue;
                auto fn = e.path().filename().string();
                if (fn.size()>=5 && strcasecmp(fn.c_str()+fn.size()-5,".onnx")==0) {
                    auto mt = e.last_write_time();
                    if (best.empty()||mt>bt) { best=e.path().string(); bt=mt; }
                }
            }
        } catch (...) {}
        return best;
    }

    void PublishMotorParams()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        for (int i=0;i<num_of_dofs_;++i) {
            msg.position.push_back(i<(int)current_kp_.size()?current_kp_[i]:20.0f);
            msg.velocity.push_back(i<(int)current_kd_.size()?current_kd_[i]:1.0f);
            msg.effort.push_back(  i<(int)current_torque_limits_.size()?current_torque_limits_[i]:17.0f);
        }
        motor_param_pub_->publish(msg);
    }

    // ---- Members ------------------------------------------------------------
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_, motor_param_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_sub_;
    std::shared_ptr<LoopFunc> loop_control_, loop_rl_;
    std::mutex data_mutex_, output_mutex_;

    std::array<float,12> joint_pos_{}, joint_vel_{}, joint_torque_{};
    std::array<int,12> driver_to_topic_{};
    std::array<float,3> commands_buffer_{}, imu_gyro_{}, imu_gravity_{};
    bool imu_received_{false}, record_enabled_{false};
    std::ofstream csv_file_;
    double record_time_{0.0};

    static constexpr const char* kJointNames[12] = {
        "FL_HipA","RL_HipA","FR_HipA","RR_HipA",
        "FL_HipF","RL_HipF","FR_HipF","RR_HipF",
        "FL_Knee","RL_Knee","FR_Knee","RR_Knee"
    };

    Observations<float> obs_;
    ObservationBuffer history_obs_buf_;
    std::vector<float> history_obs_;
    std::unique_ptr<InferenceRuntime::Model> model_;
    std::vector<float> latest_target_pos_;
    std::vector<int> obs_dims_;
    std::vector<std::string> observations_;
    std::vector<int> observations_history_;
    std::vector<float> action_scale_, commands_scale_, default_dof_pos_;
    std::vector<float> clip_actions_upper_, clip_actions_lower_;
    int num_of_dofs_;
    float dt_;
    int decimation_;
    float lin_vel_scale_, ang_vel_scale_, dof_pos_scale_, dof_vel_scale_, gravity_vec_scale_, clip_obs_;
    std::vector<float> current_kp_, current_kd_, current_torque_limits_;

    bool rl_init_done_{false}, all_sensors_ready_{false};
    int inference_count_{0};
    double inference_time_ms_{0.0};
    std::string policy_name_;
    std::optional<std::chrono::steady_clock::time_point> last_print_time_;
    State current_state_{State::RUNNING};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ARSNode>(argc>1?argv[1]:"ares_himloco/himloco");
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);  exec.spin();
    rclcpp::shutdown();  return 0;
}

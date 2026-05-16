/*
 * ARES RL Control ROS2 Node
 *
 * Projected gravity from driver (Imu.linear_acceleration).
 * Motor topics use FL RL FR RR URDF order; driver node reorders.
 *
 * Keyboard state machine (checked inline in the 200Hz control loop):
 *   [0] STOP  — publish default joint positions, stop inference
 *   [1] himloco — switch policy (from STOPPED)
 *   [2] cts     — switch policy (from STOPPED)
 *
 * On policy switch: InitRL() loads the new ONNX model, publishes kp/kd
 * via /motor_param_update, then transitions to RUNNING.
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
        motor_feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/motor_feedback", 10, std::bind(&ARSNode::MotorFeedbackCallback, this, _1));
        imu_sub_  = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&ARSNode::ImuCallback, this, _1));
        xbox_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/xbox_vel", 10, std::bind(&ARSNode::XboxVelCallback, this, _1));

        if (!InitRL()) { RCLCPP_ERROR(get_logger(), "RL init failed!"); return; }

        loop_control_ = std::make_shared<LoopFunc>("loop_control",  dt_,
                                                     std::bind(&ARSNode::RobotControl, this));
        loop_rl_ = std::make_shared<LoopFunc>("loop_rl", dt_ * decimation_,
                                                std::bind(&ARSNode::RunModel, this));
        loop_control_->start();  loop_rl_->start();

        printf("\n=== ARES RL Controls ===\n"
               "  [0] STOP  [1] himloco  [2] cts\n"
               "  Switch policy from STOPPED\n\n");
    }

    ~ARSNode()
    {
        if (loop_control_) loop_control_->shutdown();
        if (loop_rl_) loop_rl_->shutdown();
        if (csv_file_.is_open()) csv_file_.close();
    }

private:
    // ---- Init (YAML config + ONNX model) -------------------------------------

    bool InitRL()
    {
        std::string policy_dir(POLICY_DIR);
        std::string cfg_path = policy_dir + "/" + policy_name_ + "/config.yaml";
        std::string model_fn;

        RCLCPP_INFO(get_logger(), "Config: %s", cfg_path.c_str());

        try {
            YAML::Node root = YAML::LoadFile(cfg_path);
            YAML::Node rc = root[policy_name_];

            // Read a scalar config field into a member variable.
            // YAML key matches the variable name (minus trailing underscore).
#define GET_CONFIG_VAR(key, var) if (rc[#key]) var = rc[#key].as<decltype(var)>()
            GET_CONFIG_VAR(dt,              dt_);
            GET_CONFIG_VAR(decimation,      decimation_);
            GET_CONFIG_VAR(num_of_dofs,     num_of_dofs_);
            GET_CONFIG_VAR(lin_vel_scale,   lin_vel_scale_);
            GET_CONFIG_VAR(ang_vel_scale,   ang_vel_scale_);
            GET_CONFIG_VAR(dof_pos_scale,   dof_pos_scale_);
            GET_CONFIG_VAR(dof_vel_scale,   dof_vel_scale_);
            GET_CONFIG_VAR(gravity_vec_scale, gravity_vec_scale_);
            GET_CONFIG_VAR(clip_obs,        clip_obs_);
#undef GET_CONFIG_VAR

            if (rc["model_name"])
                model_fn = rc["model_name"].as<std::string>();

            // Observation field names from config
            if (rc["observations"]) {
                const auto& list = rc["observations"];
                for (const auto& item : list)
                    observations_.push_back(item.as<std::string>());
            }

            // Optional observation history indices (for stacking frames)
            if (rc["observations_history"]) {
                obs_history_.clear();
                for (const auto& step : rc["observations_history"])
                    obs_history_.push_back(step.as<int>());
            }

            // Read float lists from YAML
            auto read_floats = [&](const char* key) {
                std::vector<float> result;
                if (rc[key])
                    for (const auto& item : rc[key])
                        result.push_back(item.as<float>());
                return result;
            };
            clip_actions_upper_ = read_floats("clip_actions_upper");
            clip_actions_lower_ = read_floats("clip_actions_lower");
            action_scale_       = read_floats("action_scale");
            commands_scale_     = read_floats("commands_scale");
            default_dof_pos_    = read_floats("default_dof_pos");

            // Topic reorder mapping: config joint order → driver topic order
            if (rc["topic_to_driver"]) {
                auto order = rc["topic_to_driver"];
                std::array<int, 12> driver_order{};
                for (int i = 0; i < 12; ++i)
                    driver_order[i] = order[i].as<int>();
                for (int i = 0; i < 12; ++i)
                    driver_to_topic_[driver_order[i]] = i;
            }

            // kp, kd, torque_limits: single float (apply to all DOFs) or per-DOF list
            auto read_scalar_or_list = [&](const YAML::Node& node) -> std::vector<float> {
                if (node.IsSequence()) {
                    std::vector<float> values;
                    for (const auto& item : node)
                        values.push_back(item.as<float>());
                    return values;
                }
                return std::vector<float>(num_of_dofs_, node.as<float>());
            };
            current_kp_            = read_scalar_or_list(rc["fixed_kp"]);
            current_kd_            = read_scalar_or_list(rc["fixed_kd"]);
            current_torque_limits_ = read_scalar_or_list(rc["torque_limits"]);

            // CSV data recording
            if (rc["record"] && rc["record"]["enabled"]) {
                if (csv_file_.is_open()) csv_file_.close();
                std::string filepath = rc["record"]["filepath"].as<std::string>();
                csv_file_.open(filepath, std::ios::out | std::ios::trunc);
                if (csv_file_.is_open()) {
                    record_enabled_ = true;
                    csv_file_ << "timestamp";
                    for (int i = 0; i < 12; ++i)
                        csv_file_ << "," << kJointNames[i] << "_pos,"
                                  << kJointNames[i] << "_vel,"
                                  << kJointNames[i] << "_torque,"
                                  << kJointNames[i] << "_target";
                    csv_file_ << "\n";
                }
            } else {
                record_enabled_ = false;
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Bad config: %s", e.what());
            return false;
        }

        EnsureVectorSizes();

        // Locate ONNX model file
        std::string model_path;
        if (model_fn.empty()) {
            model_path = FindLatestOnnx(policy_dir + "/" + policy_name_ + "/");
        } else {
            model_path = policy_dir + "/" + policy_name_ + "/" + model_fn;
        }
        if (model_path.empty()) {
            RCLCPP_ERROR(get_logger(), "No .onnx model found");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Model: %s", model_path.c_str());
        model_ = InferenceRuntime::ModelFactory::load_model(model_path);
        if (!model_) {
            RCLCPP_ERROR(get_logger(), "Failed to load model");
            return false;
        }

        ComputeObsDims();

        // Initialize observation history buffer
        int history_len = 1;
        if (!obs_history_.empty())
            history_len = *std::max_element(obs_history_.begin(), obs_history_.end()) + 1;
        history_obs_buf_ = ObservationBuffer(1, obs_dims_, history_len, "time");

        int total_obs_dim = 0;
        for (int dim : obs_dims_) total_obs_dim += dim;
        for (int i = 0; i < history_len; ++i)
            history_obs_buf_.insert(std::vector<float>(total_obs_dim, 0));

        // Reset observation state to defaults
        obs_.lin_vel     = {0, 0, 0};
        obs_.ang_vel     = {0, 0, 0};
        obs_.gravity_vec = {0, 0, -1};
        obs_.commands    = {0, 0, 0};
        obs_.base_quat   = {1, 0, 0, 0};
        obs_.dof_pos     = default_dof_pos_;
        obs_.dof_vel.assign(num_of_dofs_, 0);
        obs_.actions.assign(num_of_dofs_, 0);
        latest_target_pos_ = default_dof_pos_;

        rl_init_done_ = true;
        RCLCPP_INFO(get_logger(), "Init OK  dof=%d  history=%d", num_of_dofs_, history_len);
        return true;
    }

    // ---- Inference  (~50Hz) --------------------------------------------------

    void RunModel()
    {
        // STOPPED → model is not loaded / safe to use
        if (current_state_ == State::STOPPED || !rl_init_done_)
            return;

        // Snapshot sensor data under lock
        {
            Lock lock(data_mutex_);
            if (!all_sensors_ready_) return;
            obs_.ang_vel     = {imu_gyro_[0], imu_gyro_[1], imu_gyro_[2]};
            obs_.gravity_vec = {imu_gravity_[0], imu_gravity_[1], imu_gravity_[2]};
            obs_.commands    = {commands_buffer_[0], commands_buffer_[1], commands_buffer_[2]};
            for (int i = 0; i < num_of_dofs_; ++i) {
                obs_.dof_pos[i] = joint_pos_[i];
                obs_.dof_vel[i] = joint_vel_[i];
            }
        }

        std::vector<float> actions = Forward(obs_);
        if (actions.empty()) return;
        obs_.actions = actions;

        {
            Lock lock(output_mutex_);
            latest_target_pos_ = ComputeTargetPositions(actions);
        }
        inference_count_++;

        // CSV logging
        if (record_enabled_) {
            Lock lock(data_mutex_);
            csv_file_ << record_time_;
            for (int i = 0; i < num_of_dofs_; ++i)
                csv_file_ << "," << obs_.dof_pos[i]
                          << "," << obs_.dof_vel[i]
                          << "," << joint_torque_[i]
                          << "," << latest_target_pos_[i];
            csv_file_ << "\n";
            record_time_ += dt_ * decimation_;
        }

        // Status print once per second
        auto now = std::chrono::steady_clock::now();
        if (!last_print_time_ ||
            std::chrono::duration<float>(now - *last_print_time_).count() >= 1.0f) {
            last_print_time_ = now;
            printf("\033[2J\033[H%-8s [%s] inf:%.1fms #%d  [0]=S [1]=H [2]=C\n",
                   current_state_ == State::STOPPED ? "STOPPED" : "RUNNING",
                   policy_name_.c_str(), inference_time_ms_, inference_count_);
        }
    }

    // ---- Control loop  (~200Hz, also checks keyboard) ------------------------

    void RobotControl()
    {
        if (!rl_init_done_ || !all_sensors_ready_) return;

        int key = kbhit();

        // STOP: publish default positions, pause inference
        if (key == '0' && current_state_ != State::STOPPED) {
            sensor_msgs::msg::JointState cmd;
            cmd.header.stamp = now();
            for (int i = 0; i < num_of_dofs_; ++i)
                cmd.position.push_back(default_dof_pos_[i]);
            motor_command_pub_->publish(cmd);
            current_state_ = State::STOPPED;
            printf("[RL] STOPPED\n");
            return;
        }

        // START: reinit with new policy (only valid from STOPPED)
        if ((key == '1' || key == '2') && current_state_ == State::STOPPED) {
            auto it = kPolicyMap.find(static_cast<char>(key));
            if (it != kPolicyMap.end()) {
                printf("[RL] Loading %s ...\n", it->second.c_str());
                rl_init_done_ = false;
                policy_name_ = it->second;
                if (InitRL()) {
                    // Sensors already streaming; mark ready immediately
                    all_sensors_ready_ = true;
                    PublishMotorParams();
                    current_state_ = State::RUNNING;
                    printf("[RL] RUNNING (%s)\n", policy_name_.c_str());
                } else {
                    printf("[RL] Reinit FAILED\n");
                }
            }
            return;
        }

        if (current_state_ == State::STOPPED) return;

        // RUNNING: publish latest target positions
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = now();
        {
            Lock lock(output_mutex_);
            for (int i = 0; i < num_of_dofs_; ++i)
                cmd.position.push_back(latest_target_pos_[i]);
        }
        motor_command_pub_->publish(cmd);
    }

    // ---- Forward -------------------------------------------------------------

    std::vector<float> Forward(const Observations<float>& obs)
    {
        using Clock = std::chrono::high_resolution_clock;
        auto start = Clock::now();

        std::vector<float> obs_vec = ComputeObservation(obs);
        std::vector<float> actions;

        if (!obs_history_.empty()) {
            history_obs_buf_.insert(obs_vec);
            history_obs_ = history_obs_buf_.get_obs_vec(obs_history_);
            actions = model_->forward({history_obs_});
        } else {
            actions = model_->forward({obs_vec});
        }

        inference_time_ms_ = std::chrono::duration<double, std::milli>(
            Clock::now() - start).count();

        // Wrong size or NaN/Inf → model issue, don't propagate
        if (actions.empty() || actions.size() != static_cast<size_t>(num_of_dofs_)) {
            RCLCPP_ERROR(get_logger(), "Bad inference output");
            return {};
        }
        for (float v : actions) {
            if (std::isnan(v) || std::isinf(v)) {
                RCLCPP_ERROR(get_logger(), "NaN/Inf in inference output");
                return {};
            }
        }

        // Clip actions to configured bounds
        for (size_t i = 0; i < actions.size(); ++i) {
            float lo = i < clip_actions_lower_.size() ? clip_actions_lower_[i] : -1.0f;
            float hi = i < clip_actions_upper_.size() ? clip_actions_upper_[i] :  1.0f;
            actions[i] = std::max(lo, std::min(hi, actions[i]));
        }
        return actions;
    }

    // ---- Observation assembly -------------------------------------------------

    std::vector<float> ComputeObservation(const Observations<float>& obs)
    {
        std::vector<float> obs_vec;

        // Pre-scale gravity vector
        std::vector<float> gravity_vec = obs.gravity_vec;
        for (auto& v : gravity_vec)
            v *= gravity_vec_scale_;

        for (const std::string& key : observations_) {
            if (key == "commands") {
                std::vector<float> s = obs.commands;
                for (size_t i = 0; i < s.size() && i < commands_scale_.size(); ++i)
                    s[i] *= commands_scale_[i];
                obs_vec.insert(obs_vec.end(), s.begin(), s.end());
            } else if (key == "ang_vel") {
                for (float v : obs.ang_vel)
                    obs_vec.push_back(v * ang_vel_scale_);
            } else if (key == "gravity_vec") {
                obs_vec.insert(obs_vec.end(), gravity_vec.begin(), gravity_vec.end());
            } else if (key == "dof_pos") {
                for (int i = 0; i < num_of_dofs_; ++i)
                    obs_vec.push_back((obs.dof_pos[i] - default_dof_pos_[i]) * dof_pos_scale_);
            } else if (key == "dof_vel") {
                for (int i = 0; i < num_of_dofs_; ++i)
                    obs_vec.push_back(obs.dof_vel[i] * dof_vel_scale_);
            } else if (key == "actions") {
                obs_vec.insert(obs_vec.end(), obs.actions.begin(), obs.actions.end());
            }
        }

        // Global observation clipping
        for (auto& v : obs_vec)
            v = std::max(-clip_obs_, std::min(clip_obs_, v));
        return obs_vec;
    }

    // ---- ROS callbacks -------------------------------------------------------

    void MotorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Lock lock(data_mutex_);
        size_t n = std::min(msg->position.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < n; ++i) {
            joint_pos_[i]    = msg->position[i];
            joint_vel_[i]    = msg->velocity[i];
            joint_torque_[i] = msg->effort[i];
        }
        motor_feedback_received_ = true;
        // Inference starts only after both IMU and feedback have arrived
        if (!all_sensors_ready_ && imu_received_)
            all_sensors_ready_ = true;
    }

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        Lock lock(data_mutex_);
        imu_gyro_[0]    = msg->angular_velocity.x;
        imu_gyro_[1]    = msg->angular_velocity.y;
        imu_gyro_[2]    = msg->angular_velocity.z;
        imu_gravity_[0] = msg->linear_acceleration.x;
        imu_gravity_[1] = msg->linear_acceleration.y;
        imu_gravity_[2] = msg->linear_acceleration.z;
        imu_received_   = true;
        if (!all_sensors_ready_ && motor_feedback_received_)
            all_sensors_ready_ = true;
    }

    void XboxVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        Lock lock(data_mutex_);
        commands_buffer_[0] = msg->linear.x;
        commands_buffer_[1] = msg->linear.y;
        commands_buffer_[2] = msg->angular.z;
    }

    // ---- Helper functions ----------------------------------------------------

    std::vector<float> ComputeTargetPositions(const std::vector<float>& actions) const
    {
        std::vector<float> target(num_of_dofs_);
        size_t n = std::min(actions.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < n; ++i)
            target[i] = actions[i] * action_scale_[i] + default_dof_pos_[i];
        for (size_t i = n; i < target.size(); ++i)
            target[i] = default_dof_pos_[i];
        return target;
    }

    void EnsureVectorSizes()
    {
        auto sz = static_cast<size_t>(num_of_dofs_);
        if (action_scale_.size() < sz)       action_scale_.resize(sz, 0.25f);
        if (commands_scale_.size() < 3)      commands_scale_.resize(3, 1.0f);
        if (default_dof_pos_.size() < sz)    default_dof_pos_.resize(sz, 0.0f);
        if (clip_actions_upper_.size() < sz) clip_actions_upper_.resize(sz, 1.0f);
        if (clip_actions_lower_.size() < sz) clip_actions_lower_.resize(sz, -1.0f);
    }

    void ComputeObsDims()
    {
        obs_dims_.clear();
        for (const std::string& key : observations_) {
            int dim = 0;
            if (key == "lin_vel" || key == "ang_vel" ||
                key == "gravity_vec" || key == "commands")
                dim = 3;
            else if (key == "dof_pos" || key == "dof_vel" || key == "actions")
                dim = num_of_dofs_;
            if (dim > 0) obs_dims_.push_back(dim);
        }
    }

    std::string FindLatestOnnx(const std::string& dir)
    {
        namespace fs = std::filesystem;
        std::string best_path;
        fs::file_time_type best_time;
        try {
            if (!fs::exists(dir)) return "";
            for (const auto& entry : fs::directory_iterator(dir)) {
                if (!entry.is_regular_file()) continue;
                auto fn = entry.path().filename().string();
                if (fn.size() >= 5 &&
                    strcasecmp(fn.c_str() + fn.size() - 5, ".onnx") == 0) {
                    auto mtime = entry.last_write_time();
                    if (best_path.empty() || mtime > best_time) {
                        best_path = entry.path().string();
                        best_time = mtime;
                    }
                }
            }
        } catch (...) {}
        return best_path;
    }

    void PublishMotorParams()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        for (int i = 0; i < num_of_dofs_; ++i) {
            msg.position.push_back(
                i < (int)current_kp_.size() ? current_kp_[i] : 20.0f);
            msg.velocity.push_back(
                i < (int)current_kd_.size() ? current_kd_[i] : 1.0f);
            msg.effort.push_back(
                i < (int)current_torque_limits_.size() ? current_torque_limits_[i] : 17.0f);
        }
        motor_param_pub_->publish(msg);
    }

    // ---- Members -------------------------------------------------------------

    // ROS publishers / subscriptions
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_, motor_param_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_sub_;
    std::shared_ptr<LoopFunc> loop_control_, loop_rl_;

    // Thread safety: sensor data (callbacks ↔ inference), output positions (inference ↔ control)
    std::mutex data_mutex_, output_mutex_;

    // Sensor data buffers (written by callbacks, read by RunModel)
    std::array<float, 12> joint_pos_{}, joint_vel_{}, joint_torque_{};
    std::array<int, 12>   driver_to_topic_{};
    std::array<float, 3>  commands_buffer_{}, imu_gyro_{}, imu_gravity_{};

    bool imu_received_{false}, motor_feedback_received_{false};
    bool record_enabled_{false};
    std::ofstream csv_file_;
    double record_time_{0.0};

    static constexpr const char* kJointNames[12] = {
        "FL_HipA", "RL_HipA", "FR_HipA", "RR_HipA",
        "FL_HipF", "RL_HipF", "FR_HipF", "RR_HipF",
        "FL_Knee", "RL_Knee", "FR_Knee", "RR_Knee"
    };

    // RL / model state
    Observations<float> obs_;
    ObservationBuffer history_obs_buf_;
    std::vector<float> history_obs_;
    std::unique_ptr<InferenceRuntime::Model> model_;
    std::vector<float> latest_target_pos_;
    std::vector<int> obs_dims_;

    // Config (from YAML)
    std::vector<std::string> observations_;  // observation field names composing the input vector
    std::vector<int> obs_history_;           // history timesteps to stack
    std::vector<float> action_scale_, commands_scale_, default_dof_pos_;
    std::vector<float> clip_actions_upper_, clip_actions_lower_;
    int    num_of_dofs_;
    float  dt_;          // control timestep (s)
    int    decimation_;  // inference runs once per N control cycles
    float  lin_vel_scale_, ang_vel_scale_, dof_pos_scale_, dof_vel_scale_;
    float  gravity_vec_scale_, clip_obs_;
    std::vector<float> current_kp_, current_kd_, current_torque_limits_;

    // Runtime state
    bool   rl_init_done_{false}, all_sensors_ready_{false};
    int    inference_count_{0};
    double inference_time_ms_{0.0};
    std::string policy_name_;
    std::optional<std::chrono::steady_clock::time_point> last_print_time_;
    State current_state_{State::RUNNING};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ARSNode>(
        argc > 1 ? argv[1] : "ares_himloco/himloco");
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

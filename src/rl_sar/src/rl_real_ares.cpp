/*
 * ARES Robot RL Control ROS2 Node
 *
 * Adapted from rl_real_uika.cpp for ARES quadruped.
 * Uses standard ROS2 messages (sensor_msgs/JointState, sensor_msgs/Imu).
 * No custom interface package needed.
 *
 * Key differences from UIKA:
 *   - Projected gravity received directly from driver (in Imu.linear_acceleration),
 *     NOT computed from quaternion
 *   - No kp/kd sent in motor commands (set at driver init)
 *   - /motor_feedback and /motor_command use FL RL FR RR ordering (URDF order)
 *   - ares_driver_node handles reordering to/from driver internal order
 *   - 6-frame observation history
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "vector_math.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>



class ARSNode : public rclcpp::Node
{
public:
    ARSNode()
        : Node("ares_rl_node"),
          num_of_dofs_(12),
          dt_(0.005f),
          decimation_(4),
          lin_vel_scale_(2.0f),
          ang_vel_scale_(0.25f),
          dof_pos_scale_(1.0f),
          dof_vel_scale_(0.05f),
          gravity_vec_scale_(1.0f),
          clip_obs_(100.0f),
          rl_init_done_(false),
          all_sensors_ready_(false),
          inference_count_(0),
          inference_time_ms_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ARES RL Node...");

        motor_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/motor_command", 10);

        motor_feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor_feedback", 10,
            std::bind(&ARSNode::MotorFeedbackCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ARSNode::ImuCallback, this, std::placeholders::_1));

        xbox_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/xbox_vel", 10,
            std::bind(&ARSNode::XboxVelCallback, this, std::placeholders::_1));

        if (!InitRL())
        {
            RCLCPP_ERROR(this->get_logger(), "RL initialization failed!");
            return;
        }

        loop_control_ = std::make_shared<LoopFunc>(
            "loop_control", dt_, std::bind(&ARSNode::RobotControl, this));
        loop_rl_ = std::make_shared<LoopFunc>(
            "loop_rl", dt_ * decimation_, std::bind(&ARSNode::RunModel, this));

        loop_control_->start();
        loop_rl_->start();

        RCLCPP_INFO(this->get_logger(), "ARES RL Node started");
        RCLCPP_INFO(this->get_logger(), "  Control: %.1fHz, Inference: %.1fHz",
                    1.0 / dt_, 1.0 / (dt_ * decimation_));
    }

    ~ARSNode()
    {
        if (loop_control_)
            loop_control_->shutdown();
        if (loop_rl_)
            loop_rl_->shutdown();
        RCLCPP_INFO(this->get_logger(), "ARES RL Node stopped");
    }

private:
    bool InitRL()
    {
        std::string policy_dir = std::string(POLICY_DIR);
        std::string config_path = policy_dir + "/ares_himloco/himloco/config.yaml";
        std::string model_name_from_config;

        RCLCPP_INFO(this->get_logger(), "Loading config: %s", config_path.c_str());

        try
        {
            YAML::Node config = YAML::LoadFile(config_path);
            YAML::Node robot_config = config["ares_himloco/himloco"];
            #define GET_PARAM(key, var) \
                if (robot_config[#key]) var = robot_config[#key].as<decltype(var)>();

            GET_PARAM(dt, dt_)
            GET_PARAM(decimation, decimation_)
            GET_PARAM(num_of_dofs, num_of_dofs_)
            GET_PARAM(lin_vel_scale, lin_vel_scale_)
            GET_PARAM(ang_vel_scale, ang_vel_scale_)
            GET_PARAM(dof_pos_scale, dof_pos_scale_)
            GET_PARAM(dof_vel_scale, dof_vel_scale_)
            GET_PARAM(gravity_vec_scale, gravity_vec_scale_)
            GET_PARAM(clip_obs, clip_obs_)

            if (robot_config["model_name"])
                model_name_from_config = robot_config["model_name"].as<std::string>();

            if (robot_config["observations"])
            {
                observations_.clear();
                for (const auto &obs : robot_config["observations"])
                    observations_.push_back(obs.as<std::string>());
            }

            if (robot_config["observations_history"])
            {
                observations_history_.clear();
                for (const auto &h : robot_config["observations_history"])
                    observations_history_.push_back(h.as<int>());
            }

            if (robot_config["clip_actions_upper"])
            {
                clip_actions_upper_.clear();
                for (const auto &c : robot_config["clip_actions_upper"])
                    clip_actions_upper_.push_back(c.as<float>());
            }
            if (robot_config["clip_actions_lower"])
            {
                clip_actions_lower_.clear();
                for (const auto &c : robot_config["clip_actions_lower"])
                    clip_actions_lower_.push_back(c.as<float>());
            }

            if (robot_config["action_scale"])
            {
                action_scale_.clear();
                for (const auto &s : robot_config["action_scale"])
                    action_scale_.push_back(s.as<float>());
            }
            if (robot_config["commands_scale"])
            {
                commands_scale_.clear();
                for (const auto &s : robot_config["commands_scale"])
                    commands_scale_.push_back(s.as<float>());
            }
            if (robot_config["default_dof_pos"])
            {
                default_dof_pos_.clear();
                for (const auto &p : robot_config["default_dof_pos"])
                    default_dof_pos_.push_back(p.as<float>());
            }

            if (robot_config["fixed_kp"])
            {
                kp_.clear();
                for (const auto &s : robot_config["fixed_kp"])
                    kp_.push_back(s.as<float>());
            }
            if (robot_config["fixed_kd"])
            {
                kd_.clear();
                for (const auto &s : robot_config["fixed_kd"])
                    kd_.push_back(s.as<float>());
            }
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            return false;
        }

        EnsureVectorSizes();

        std::string model_path;
        if (!model_name_from_config.empty())
        {
            model_path = policy_dir + "/ares_himloco/himloco/" + model_name_from_config;
        }
        else
        {
            std::string search_dir = policy_dir + "/ares_himloco/himloco/";
            model_path = FindLatestOnnx(search_dir);
            if (model_path.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No .onnx model found");
                return false;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loading model: %s", model_path.c_str());
        model_ = InferenceRuntime::ModelFactory::load_model(model_path);
        if (!model_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model!");
            return false;
        }

        ComputeObsDims();

        int history_length = 1;
        if (!observations_history_.empty())
            history_length = *std::max_element(observations_history_.begin(), observations_history_.end()) + 1;
        history_obs_buf_ = ObservationBuffer(1, obs_dims_, history_length, "time");

        int obs_dim = 0;
        for (int dim : obs_dims_)
            obs_dim += dim;
        for (int i = 0; i < history_length; ++i)
            history_obs_buf_.insert(std::vector<float>(obs_dim, 0.0f));

        obs_.lin_vel = {0.0f, 0.0f, 0.0f};
        obs_.ang_vel = {0.0f, 0.0f, 0.0f};
        obs_.gravity_vec = {0.0f, 0.0f, -1.0f};
        obs_.commands = {0.0f, 0.0f, 0.0f};
        obs_.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
        obs_.dof_pos = default_dof_pos_;
        obs_.dof_vel.resize(num_of_dofs_, 0.0f);
        obs_.actions.resize(num_of_dofs_, 0.0f);
        last_action_.assign(num_of_dofs_, 0.0f);

        RCLCPP_INFO(this->get_logger(), "default_dof_pos: %s", FormatVector(default_dof_pos_).c_str());
        RCLCPP_INFO(this->get_logger(), "action_scale: %s", FormatVector(action_scale_).c_str());
        RCLCPP_INFO(this->get_logger(), "commands_scale: %s", FormatVector(commands_scale_).c_str());
        RCLCPP_INFO(this->get_logger(), "observations: %s", FormatStringVector(observations_).c_str());
        RCLCPP_INFO(this->get_logger(), "observations_history length: %d", history_length);

        rl_init_done_ = true;
        return true;
    }

    void RunModel()
    {
        if (!rl_init_done_)
            return;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!all_sensors_ready_)
                return;

            obs_.ang_vel = {imu_gyro_[0], imu_gyro_[1], imu_gyro_[2]};
            obs_.gravity_vec = {imu_gravity_[0], imu_gravity_[1], imu_gravity_[2]};
            obs_.commands = {commands_buffer_[0], commands_buffer_[1], commands_buffer_[2]};

            for (int i = 0; i < num_of_dofs_; ++i)
            {
                obs_.dof_pos[i] = joint_pos_[i];
                obs_.dof_vel[i] = joint_vel_[i];
            }
        }

        std::vector<float> actions = Forward(obs_);
        obs_.actions = actions;
        std::vector<float> output_dof_pos = ComputeTargetPositions(actions);

        inference_count_++;
        // Print every 1 second
        auto now = std::chrono::steady_clock::now();
        if (!last_print_time_.has_value() ||
            std::chrono::duration<float>(now - last_print_time_.value()).count() >= 1.0f)
        {
            last_print_time_ = now;
            RCLCPP_INFO(this->get_logger(),
                        "[Infer #%d] cmd=%s ang_vel=%s gravity=%s dof_pos=%s dof_vel=%s action=%s target=%s",
                        inference_count_,
                        FormatVector(obs_.commands).c_str(),
                        FormatVector(obs_.ang_vel).c_str(),
                        FormatVector(obs_.gravity_vec).c_str(),
                        FormatVector(obs_.dof_pos).c_str(),
                        FormatVector(obs_.dof_vel).c_str(),
                        FormatVector(actions).c_str(),
                        FormatVector(output_dof_pos).c_str());
        }

        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            last_action_ = actions;
            output_dof_pos_queue_.push(output_dof_pos);
        }
    }

    void RobotControl()
    {
        if (!rl_init_done_ || !all_sensors_ready_)
            return;

        std::vector<float> output_dof_pos;
        std::vector<float> fallback_action;
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            while (!output_dof_pos_queue_.empty())
            {
                output_dof_pos = output_dof_pos_queue_.front();
                output_dof_pos_queue_.pop();
            }
            fallback_action = last_action_;
        }

        if (output_dof_pos.empty())
        {
            if (fallback_action.size() == static_cast<size_t>(num_of_dofs_))
                output_dof_pos = ComputeTargetPositions(fallback_action);
            else
                output_dof_pos = default_dof_pos_;
        }

        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = this->now();
        for (int i = 0; i < num_of_dofs_; ++i)
        {
            cmd_msg.position.push_back(output_dof_pos[i]);
        }
        motor_command_pub_->publish(cmd_msg);
    }

    std::vector<float> Forward(const Observations<float> &obs_snapshot)
    {
        auto inference_start = std::chrono::high_resolution_clock::now();
        std::vector<float> clamped_obs = ComputeObservation(obs_snapshot);
        std::vector<float> actions;
        std::vector<float> fallback_action;

        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            fallback_action = last_action_;
        }

        if (!observations_history_.empty())
        {
            history_obs_buf_.insert(clamped_obs);
            history_obs_ = history_obs_buf_.get_obs_vec(observations_history_);
            actions = model_->forward({history_obs_});
        }
        else
        {
            actions = model_->forward({clamped_obs});
        }

        auto inference_end = std::chrono::high_resolution_clock::now();
        inference_time_ms_ = std::chrono::duration<double, std::milli>(inference_end - inference_start).count();

        if (actions.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Inference returned empty action, reusing last action");
            return fallback_action;
        }
        if (actions.size() != static_cast<size_t>(num_of_dofs_))
        {
            RCLCPP_ERROR(this->get_logger(), "Inference returned %zu actions, expected %d",
                         actions.size(), num_of_dofs_);
            return fallback_action;
        }

        for (float &value : actions)
        {
            if (std::isnan(value) || std::isinf(value))
            {
                RCLCPP_ERROR(this->get_logger(), "NaN/Inf detected in actions, reusing last action");
                return fallback_action;
            }
        }

        for (size_t i = 0; i < actions.size(); ++i)
        {
            float lower = (i < clip_actions_lower_.size()) ? clip_actions_lower_[i] : -1.0f;
            float upper = (i < clip_actions_upper_.size()) ? clip_actions_upper_[i] : 1.0f;
            actions[i] = std::max(lower, std::min(upper, actions[i]));
        }
        return actions;
    }

    std::vector<float> ComputeObservation(const Observations<float> &obs_snapshot)
    {
        std::vector<float> obs_vec;

        // Projected gravity is already in body frame from the driver.
        // Apply gravity_vec_scale_ (should be 1.0 for ARES).
        std::vector<float> scaled_gravity = obs_snapshot.gravity_vec;
        for (auto &v : scaled_gravity)
            v *= gravity_vec_scale_;

        for (const std::string &obs_name : observations_)
        {
            if (obs_name == "commands")
            {
                std::vector<float> scaled = obs_snapshot.commands;
                for (size_t i = 0; i < scaled.size() && i < commands_scale_.size(); ++i)
                    scaled[i] *= commands_scale_[i];
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "ang_vel")
            {
                std::vector<float> scaled = obs_snapshot.ang_vel;
                for (auto &v : scaled)
                    v *= ang_vel_scale_;
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "gravity_vec")
            {
                obs_vec.insert(obs_vec.end(), scaled_gravity.begin(), scaled_gravity.end());
            }
            else if (obs_name == "dof_pos")
            {
                std::vector<float> dof_pos_rel(num_of_dofs_, 0.0f);
                for (int i = 0; i < num_of_dofs_; ++i)
                    dof_pos_rel[i] = (obs_snapshot.dof_pos[i] - default_dof_pos_[i]) * dof_pos_scale_;
                obs_vec.insert(obs_vec.end(), dof_pos_rel.begin(), dof_pos_rel.end());
            }
            else if (obs_name == "dof_vel")
            {
                std::vector<float> scaled = obs_snapshot.dof_vel;
                for (auto &v : scaled)
                    v *= dof_vel_scale_;
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "actions")
            {
                obs_vec.insert(obs_vec.end(), obs_snapshot.actions.begin(), obs_snapshot.actions.end());
            }
        }

        for (auto &v : obs_vec)
            v = std::max(-clip_obs_, std::min(clip_obs_, v));
        return obs_vec;
    }

    void MotorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        size_t count = std::min(msg->position.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < count; ++i)
        {
            joint_pos_[i] = msg->position[i];
            joint_vel_[i] = msg->velocity[i];
        }

        motor_feedback_received_ = true;
        if (motor_feedback_received_ && imu_received_)
            all_sensors_ready_ = true;
    }

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Angular velocity from WIT IMU (body frame, rad/s)
        imu_gyro_[0] = msg->angular_velocity.x;
        imu_gyro_[1] = msg->angular_velocity.y;
        imu_gyro_[2] = msg->angular_velocity.z;

        // Projected gravity from driver (carried in linear_acceleration field)
        // This is already in body frame — no quaternion rotation needed
        imu_gravity_[0] = msg->linear_acceleration.x;
        imu_gravity_[1] = msg->linear_acceleration.y;
        imu_gravity_[2] = msg->linear_acceleration.z;

        imu_received_ = true;
        if (motor_feedback_received_ && imu_received_)
            all_sensors_ready_ = true;
    }

    void XboxVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        commands_buffer_[0] = msg->linear.x;
        commands_buffer_[1] = msg->linear.y;
        commands_buffer_[2] = msg->angular.z;
    }

    std::vector<float> ComputeTargetPositions(const std::vector<float> &actions) const
    {
        std::vector<float> output_dof_pos(num_of_dofs_, 0.0f);
        size_t count = std::min(actions.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < count; ++i)
            output_dof_pos[i] = actions[i] * action_scale_[i] + default_dof_pos_[i];
        for (size_t i = count; i < output_dof_pos.size(); ++i)
            output_dof_pos[i] = default_dof_pos_[i];
        return output_dof_pos;
    }

    void EnsureVectorSizes()
    {
        if (action_scale_.size() < static_cast<size_t>(num_of_dofs_))
            action_scale_.resize(num_of_dofs_, 0.25f);
        if (commands_scale_.size() < 3)
            commands_scale_.resize(3, 1.0f);
        if (default_dof_pos_.size() < static_cast<size_t>(num_of_dofs_))
            default_dof_pos_.resize(num_of_dofs_, 0.0f);
        if (clip_actions_upper_.size() < static_cast<size_t>(num_of_dofs_))
            clip_actions_upper_.resize(num_of_dofs_, 1.0f);
        if (clip_actions_lower_.size() < static_cast<size_t>(num_of_dofs_))
            clip_actions_lower_.resize(num_of_dofs_, -1.0f);
        if (kp_.size() < static_cast<size_t>(num_of_dofs_))
            kp_.resize(num_of_dofs_, 40.0f);
        if (kd_.size() < static_cast<size_t>(num_of_dofs_))
            kd_.resize(num_of_dofs_, 0.5f);
    }

    void ComputeObsDims()
    {
        obs_dims_.clear();
        for (const std::string &obs : observations_)
        {
            int dim = 0;
            if (obs == "lin_vel")
                dim = 3;
            else if (obs == "ang_vel")
                dim = 3;
            else if (obs == "gravity_vec")
                dim = 3;
            else if (obs == "commands")
                dim = 3;
            else if (obs == "dof_pos")
                dim = num_of_dofs_;
            else if (obs == "dof_vel")
                dim = num_of_dofs_;
            else if (obs == "actions")
                dim = num_of_dofs_;
            if (dim > 0)
                obs_dims_.push_back(dim);
        }
    }

    std::string FindLatestOnnx(const std::string &dir_path)
    {
        namespace fs = std::filesystem;
        std::string latest_path;
        std::filesystem::file_time_type latest_time;

        try
        {
            if (!fs::exists(dir_path) || !fs::is_directory(dir_path))
                return "";

            for (const auto &entry : fs::directory_iterator(dir_path))
            {
                if (!entry.is_regular_file())
                    continue;
                std::string filename = entry.path().filename().string();
                if (filename.size() >= 5 &&
                    strcasecmp(filename.c_str() + filename.size() - 5, ".onnx") == 0)
                {
                    auto mtime = entry.last_write_time();
                    if (latest_path.empty() || mtime > latest_time)
                    {
                        latest_path = entry.path().string();
                        latest_time = mtime;
                    }
                }
            }
        }
        catch (const std::exception &)
        {
        }

        return latest_path;
    }

    std::string FormatVector(const std::vector<float> &values) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4) << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
                oss << ", ";
            oss << values[i];
        }
        oss << "]";
        return oss.str();
    }

    std::string FormatStringVector(const std::vector<std::string> &values) const
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
                oss << ", ";
            oss << values[i];
        }
        oss << "]";
        return oss.str();
    }

    // ROS2 communication
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_sub_;

    // Control loops
    std::shared_ptr<LoopFunc> loop_control_;
    std::shared_ptr<LoopFunc> loop_rl_;

    // Thread safety
    std::mutex data_mutex_;
    std::mutex output_mutex_;

    // Sensor data buffers
    std::array<float, 12> joint_pos_{};
    std::array<float, 12> joint_vel_{};
    std::array<float, 3> commands_buffer_{};
    std::array<float, 3> imu_gyro_{};        // angular velocity (body frame, rad/s)
    std::array<float, 3> imu_gravity_{};     // projected gravity (body frame, unit vector)
    bool motor_feedback_received_{false};
    bool imu_received_{false};

    // Output queue
    std::queue<std::vector<float>> output_dof_pos_queue_;
    std::vector<float> latest_action_for_log_;

    // RL framework
    Observations<float> obs_;
    ObservationBuffer history_obs_buf_;
    std::vector<float> history_obs_;
    std::unique_ptr<InferenceRuntime::Model> model_;
    std::vector<float> last_action_;

    // Config-driven parameters
    std::vector<int> obs_dims_;
    std::vector<std::string> observations_;
    std::vector<int> observations_history_;
    std::vector<float> action_scale_;
    std::vector<float> commands_scale_;
    std::vector<float> default_dof_pos_;
    std::vector<float> clip_actions_upper_;
    std::vector<float> clip_actions_lower_;
    std::vector<float> kp_;  // Loaded for logging only — NOT sent per-command
    std::vector<float> kd_;  // Loaded for logging only — NOT sent per-command
    // Basic parameters
    int num_of_dofs_;
    float dt_;
    int decimation_;
    float lin_vel_scale_;
    float ang_vel_scale_;
    float dof_pos_scale_;
    float dof_vel_scale_;
    float gravity_vec_scale_;
    float clip_obs_;

    // State flags
    bool rl_init_done_;
    bool all_sensors_ready_;
    int inference_count_;
    double inference_time_ms_;
    std::optional<std::chrono::steady_clock::time_point> last_print_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ARES RL Node...");

    auto node = std::make_shared<ARSNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main"), "ARES RL Node shutting down...");
    rclcpp::shutdown();
    return 0;
}

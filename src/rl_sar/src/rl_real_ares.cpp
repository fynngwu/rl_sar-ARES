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
          num_of_dofs_(12), control_timestep_(0.005f), decimation_(4),
          lin_vel_scale_(2.0f), ang_vel_scale_(0.25f),
          dof_pos_scale_(1.0f), dof_vel_scale_(0.05f),
          gravity_vector_scale_(1.0f), clip_observations_(100.0f),
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

        loop_control_ = std::make_shared<LoopFunc>("loop_control",  control_timestep_,
                                                     std::bind(&ARSNode::RobotControl, this));
        loop_rl_ = std::make_shared<LoopFunc>("loop_rl", control_timestep_ * decimation_,
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
    // ---- Initialization (loads YAML config + ONNX model) ----------------------

    bool InitRL()
    {
        std::string policy_dir(POLICY_DIR);
        std::string config_path = policy_dir + "/" + policy_name_ + "/config.yaml";
        std::string model_filename;

        RCLCPP_INFO(get_logger(), "Config: %s", config_path.c_str());

        try {
            YAML::Node root = YAML::LoadFile(config_path);
            YAML::Node rc = root[policy_name_];

            // Macro to read a scalar config field into a member variable.
            // The YAML key name matches the variable name (minus trailing underscore).
#define GET_CONFIG_VAR(key, var) if (rc[#key]) var = rc[#key].as<decltype(var)>()
            GET_CONFIG_VAR(dt,              control_timestep_);
            GET_CONFIG_VAR(decimation,      decimation_);
            GET_CONFIG_VAR(num_of_dofs,     num_of_dofs_);
            GET_CONFIG_VAR(lin_vel_scale,   lin_vel_scale_);
            GET_CONFIG_VAR(ang_vel_scale,   ang_vel_scale_);
            GET_CONFIG_VAR(dof_pos_scale,   dof_pos_scale_);
            GET_CONFIG_VAR(dof_vel_scale,   dof_vel_scale_);
            GET_CONFIG_VAR(gravity_vec_scale, gravity_vector_scale_);
            GET_CONFIG_VAR(clip_obs,        clip_observations_);
#undef GET_CONFIG_VAR

            if (rc["model_name"])
                model_filename = rc["model_name"].as<std::string>();

            // Read list of observation field names from config
            auto load_string_list = [&](const char* key) {
                std::vector<std::string> result;
                if (rc[key])
                    for (const auto& item : rc[key])
                        result.push_back(item.as<std::string>());
                return result;
            };
            observations_ = load_string_list("observations");

            // Optional observation history indices
            if (rc["observations_history"]) {
                observations_history_.clear();
                for (const auto& h : rc["observations_history"])
                    observations_history_.push_back(h.as<int>());
            }

            // Read lists of float parameters
            auto load_float_list = [&](const char* key) {
                std::vector<float> result;
                if (rc[key])
                    for (const auto& item : rc[key])
                        result.push_back(item.as<float>());
                return result;
            };
            clip_actions_upper_ = load_float_list("clip_actions_upper");
            clip_actions_lower_ = load_float_list("clip_actions_lower");
            action_scale_       = load_float_list("action_scale");
            commands_scale_     = load_float_list("commands_scale");
            default_dof_pos_    = load_float_list("default_dof_pos");

            // Topic reorder mapping: joint order from config → driver topic order
            if (rc["topic_to_driver"]) {
                auto topic_to_driver = rc["topic_to_driver"];
                std::array<int, 12> driver_order{};
                for (int i = 0; i < 12; ++i)
                    driver_order[i] = topic_to_driver[i].as<int>();
                for (int i = 0; i < 12; ++i)
                    driver_to_topic_[driver_order[i]] = i;
            }

            // kp, kd, torque_limits: each can be a single float (applied to all DOFs)
            // or a per-DOF list
            auto load_parameter_array = [&](const YAML::Node& config_node)
                -> std::vector<float> {
                if (config_node.IsSequence()) {
                    std::vector<float> values;
                    for (const auto& item : config_node)
                        values.push_back(item.as<float>());
                    return values;
                }
                return std::vector<float>(num_of_dofs_, config_node.as<float>());
            };
            current_kp_            = load_parameter_array(rc["fixed_kp"]);
            current_kd_            = load_parameter_array(rc["fixed_kd"]);
            current_torque_limits_ = load_parameter_array(rc["torque_limits"]);

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
        if (model_filename.empty()) {
            model_path = FindLatestOnnx(policy_dir + "/" + policy_name_ + "/");
        } else {
            model_path = policy_dir + "/" + policy_name_ + "/" + model_filename;
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

        ComputeObservationDimensions();

        // Initialize observation history buffer
        int history_length = 1;
        if (!observations_history_.empty())
            history_length = *std::max_element(
                observations_history_.begin(), observations_history_.end()) + 1;
        history_observation_buffer_ = ObservationBuffer(
            1, observation_dimensions_, history_length, "time");

        int total_observation_dims = 0;
        for (int dim : observation_dimensions_)
            total_observation_dims += dim;
        for (int i = 0; i < history_length; ++i)
            history_observation_buffer_.insert(
                std::vector<float>(total_observation_dims, 0));

        // Reset observation state to defaults
        current_observation_.lin_vel     = {0, 0, 0};
        current_observation_.ang_vel     = {0, 0, 0};
        current_observation_.gravity_vec = {0, 0, -1};
        current_observation_.commands    = {0, 0, 0};
        current_observation_.base_quat   = {1, 0, 0, 0};
        current_observation_.dof_pos     = default_dof_pos_;
        current_observation_.dof_vel.assign(num_of_dofs_, 0);
        current_observation_.actions.assign(num_of_dofs_, 0);
        latest_target_pos_ = default_dof_pos_;

        rl_init_done_ = true;
        RCLCPP_INFO(get_logger(), "Init OK  dof=%d  history=%d", num_of_dofs_, history_length);
        return true;
    }

    // ---- Model Inference  (~50Hz) --------------------------------------------

    void RunModel()
    {
        // When STOPPED the model is not loaded or safe to use
        if (current_state_ == State::STOPPED || !rl_init_done_)
            return;

        // Snapshot sensor data under lock
        {
            Lock lock(data_mutex_);
            if (!all_sensors_ready_) return;
            current_observation_.ang_vel = {
                imu_gyro_[0], imu_gyro_[1], imu_gyro_[2]};
            current_observation_.gravity_vec = {
                imu_gravity_[0], imu_gravity_[1], imu_gravity_[2]};
            current_observation_.commands = {
                commands_buffer_[0], commands_buffer_[1], commands_buffer_[2]};
            for (int i = 0; i < num_of_dofs_; ++i) {
                current_observation_.dof_pos[i] = joint_pos_[i];
                current_observation_.dof_vel[i] = joint_vel_[i];
            }
        }

        std::vector<float> actions = Forward(current_observation_);
        if (actions.empty()) return;
        current_observation_.actions = actions;

        {
            Lock lock(output_mutex_);
            latest_target_pos_ = ComputeTargetPositions(actions);
        }
        inference_count_++;

        // CSV logging (every inference step)
        if (record_enabled_) {
            Lock lock(data_mutex_);
            csv_file_ << record_time_;
            for (int i = 0; i < num_of_dofs_; ++i)
                csv_file_ << "," << current_observation_.dof_pos[i]
                          << "," << current_observation_.dof_vel[i]
                          << "," << joint_torque_[i]
                          << "," << latest_target_pos_[i];
            csv_file_ << "\n";
            record_time_ += control_timestep_ * decimation_;
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

    // ---- Control Loop  (~200Hz, also checks keyboard) ------------------------

    void RobotControl()
    {
        if (!rl_init_done_ || !all_sensors_ready_) return;

        int key = kbhit();

        // STOP: publish default positions and pause inference
        if (key == '0' && current_state_ != State::STOPPED) {
            sensor_msgs::msg::JointState command_msg;
            command_msg.header.stamp = now();
            for (int i = 0; i < num_of_dofs_; ++i)
                command_msg.position.push_back(default_dof_pos_[i]);
            motor_command_pub_->publish(command_msg);
            current_state_ = State::STOPPED;
            printf("[RL] STOPPED\n");
            return;
        }

        // START: re-init with a new policy (only valid from STOPPED)
        if ((key == '1' || key == '2') && current_state_ == State::STOPPED) {
            auto it = kPolicyMap.find(static_cast<char>(key));
            if (it != kPolicyMap.end()) {
                printf("[RL] Loading %s ...\n", it->second.c_str());
                rl_init_done_ = false;
                policy_name_ = it->second;
                if (InitRL()) {
                    // Sensors are already streaming; mark them ready immediately
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

        // Normal RUNNING: publish latest target positions
        sensor_msgs::msg::JointState command_msg;
        command_msg.header.stamp = now();
        {
            Lock lock(output_mutex_);
            for (int i = 0; i < num_of_dofs_; ++i)
                command_msg.position.push_back(latest_target_pos_[i]);
        }
        motor_command_pub_->publish(command_msg);
    }

    // ---- Forward inference pass ----------------------------------------------

    std::vector<float> Forward(const Observations<float>& observation_snapshot)
    {
        using Clock = std::chrono::high_resolution_clock;
        auto start_time = Clock::now();

        std::vector<float> observation_vector = ComputeObservation(observation_snapshot);
        std::vector<float> actions;

        if (!observations_history_.empty()) {
            // Accumulate observations across history steps
            history_observation_buffer_.insert(observation_vector);
            history_observations_ =
                history_observation_buffer_.get_obs_vec(observations_history_);
            actions = model_->forward({history_observations_});
        } else {
            actions = model_->forward({observation_vector});
        }

        inference_time_ms_ = std::chrono::duration<double, std::milli>(
            Clock::now() - start_time).count();

        // Validate: wrong size or NaN/Inf indicates a model issue
        if (actions.empty() ||
            actions.size() != static_cast<size_t>(num_of_dofs_)) {
            RCLCPP_ERROR(get_logger(), "Bad inference output");
            return {};
        }
        for (float value : actions) {
            if (std::isnan(value) || std::isinf(value)) {
                RCLCPP_ERROR(get_logger(), "NaN/Inf in inference output");
                return {};
            }
        }

        // Clip actions to configured bounds
        for (size_t i = 0; i < actions.size(); ++i) {
            float lower_bound = i < clip_actions_lower_.size() ? clip_actions_lower_[i] : -1.0f;
            float upper_bound = i < clip_actions_upper_.size() ? clip_actions_upper_[i] :  1.0f;
            actions[i] = std::max(lower_bound, std::min(upper_bound, actions[i]));
        }
        return actions;
    }

    // ---- Observation vector assembly -----------------------------------------

    std::vector<float> ComputeObservation(const Observations<float>& observation_snapshot)
    {
        std::vector<float> observation_values;

        // Pre-scale gravity vector (used below)
        std::vector<float> gravity_vector = observation_snapshot.gravity_vec;
        for (auto& value : gravity_vector)
            value *= gravity_vector_scale_;

        for (const std::string& observation_name : observations_) {
            if (observation_name == "commands") {
                std::vector<float> scaled_commands = observation_snapshot.commands;
                for (size_t i = 0; i < scaled_commands.size() && i < commands_scale_.size(); ++i)
                    scaled_commands[i] *= commands_scale_[i];
                observation_values.insert(
                    observation_values.end(), scaled_commands.begin(), scaled_commands.end());
            } else if (observation_name == "ang_vel") {
                for (float value : observation_snapshot.ang_vel)
                    observation_values.push_back(value * ang_vel_scale_);
            } else if (observation_name == "gravity_vec") {
                observation_values.insert(
                    observation_values.end(), gravity_vector.begin(), gravity_vector.end());
            } else if (observation_name == "dof_pos") {
                for (int i = 0; i < num_of_dofs_; ++i)
                    observation_values.push_back(
                        (observation_snapshot.dof_pos[i] - default_dof_pos_[i]) * dof_pos_scale_);
            } else if (observation_name == "dof_vel") {
                for (int i = 0; i < num_of_dofs_; ++i)
                    observation_values.push_back(observation_snapshot.dof_vel[i] * dof_vel_scale_);
            } else if (observation_name == "actions") {
                observation_values.insert(
                    observation_values.end(), observation_snapshot.actions.begin(),
                    observation_snapshot.actions.end());
            }
        }

        // Global observation clipping
        for (auto& value : observation_values)
            value = std::max(-clip_observations_, std::min(clip_observations_, value));
        return observation_values;
    }

    // ---- ROS callbacks -------------------------------------------------------

    void MotorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Lock lock(data_mutex_);
        size_t num_read = std::min(msg->position.size(),
                                   static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < num_read; ++i) {
            joint_pos_[i]    = msg->position[i];
            joint_vel_[i]    = msg->velocity[i];
            joint_torque_[i] = msg->effort[i];
        }
        motor_feedback_received_ = true;
        // Both IMU and feedback must arrive before inference starts
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

    std::vector<float> ComputeTargetPositions(
        const std::vector<float>& actions) const
    {
        std::vector<float> target_positions(num_of_dofs_);
        size_t num_actions = std::min(actions.size(),
                                      static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < num_actions; ++i)
            target_positions[i] = actions[i] * action_scale_[i] + default_dof_pos_[i];
        for (size_t i = num_actions; i < target_positions.size(); ++i)
            target_positions[i] = default_dof_pos_[i];
        return target_positions;
    }

    void EnsureVectorSizes()
    {
        auto size = static_cast<size_t>(num_of_dofs_);
        if (action_scale_.size() < size)       action_scale_.resize(size, 0.25f);
        if (commands_scale_.size() < 3)        commands_scale_.resize(3, 1.0f);
        if (default_dof_pos_.size() < size)    default_dof_pos_.resize(size, 0.0f);
        if (clip_actions_upper_.size() < size) clip_actions_upper_.resize(size, 1.0f);
        if (clip_actions_lower_.size() < size) clip_actions_lower_.resize(size, -1.0f);
    }

    void ComputeObservationDimensions()
    {
        observation_dimensions_.clear();
        for (const std::string& observation_name : observations_) {
            int dimension = 0;
            if (observation_name == "lin_vel"  ||
                observation_name == "ang_vel"  ||
                observation_name == "gravity_vec" ||
                observation_name == "commands")
                dimension = 3;
            else if (observation_name == "dof_pos" ||
                     observation_name == "dof_vel" ||
                     observation_name == "actions")
                dimension = num_of_dofs_;
            if (dimension > 0)
                observation_dimensions_.push_back(dimension);
        }
    }

    std::string FindLatestOnnx(const std::string& directory)
    {
        namespace fs = std::filesystem;
        std::string best_filename;
        fs::file_time_type best_file_time;
        try {
            if (!fs::exists(directory)) return "";
            for (const auto& entry : fs::directory_iterator(directory)) {
                if (!entry.is_regular_file()) continue;
                std::string filename = entry.path().filename().string();
                if (filename.size() >= 5 &&
                    strcasecmp(filename.c_str() + filename.size() - 5, ".onnx") == 0) {
                    auto modification_time = entry.last_write_time();
                    if (best_filename.empty() || modification_time > best_file_time) {
                        best_filename = entry.path().string();
                        best_file_time = modification_time;
                    }
                }
            }
        } catch (...) {}
        return best_filename;
    }

    void PublishMotorParams()
    {
        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        for (int i = 0; i < num_of_dofs_; ++i) {
            message.position.push_back(
                i < static_cast<int>(current_kp_.size()) ? current_kp_[i] : 20.0f);
            message.velocity.push_back(
                i < static_cast<int>(current_kd_.size()) ? current_kd_[i] : 1.0f);
            message.effort.push_back(
                i < static_cast<int>(current_torque_limits_.size()) ? current_torque_limits_[i] : 17.0f);
        }
        motor_param_pub_->publish(message);
    }

    // ---- Member variables ----------------------------------------------------

    // ROS publishers / subscriptions
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_param_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_sub_;
    std::shared_ptr<LoopFunc> loop_control_, loop_rl_;

    // Thread safety (sensor data and output position shared between callbacks and loops)
    std::mutex data_mutex_, output_mutex_;

    // Sensor data buffers (written by callbacks, read by RunModel)
    std::array<float, 12> joint_pos_{}, joint_vel_{}, joint_torque_{};
    std::array<int, 12>   driver_to_topic_{};
    std::array<float, 3>  commands_buffer_{}, imu_gyro_{}, imu_gravity_{};

    // Sensor readiness flags — inference starts only after both arrive
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
    Observations<float> current_observation_;
    ObservationBuffer history_observation_buffer_;
    std::vector<float> history_observations_;
    std::unique_ptr<InferenceRuntime::Model> model_;
    std::vector<float> latest_target_pos_;
    std::vector<int>   observation_dimensions_;

    // Config (loaded from YAML)
    std::vector<std::string> observations_;       // which fields compose the observation vector
    std::vector<int>         observations_history_; // history timesteps to stack
    std::vector<float> action_scale_, commands_scale_, default_dof_pos_;
    std::vector<float> clip_actions_upper_, clip_actions_lower_;
    int    num_of_dofs_;
    float  control_timestep_;
    int    decimation_;  // inference runs once per N control cycles
    float  lin_vel_scale_, ang_vel_scale_, dof_pos_scale_, dof_vel_scale_;
    float  gravity_vector_scale_, clip_observations_;
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
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

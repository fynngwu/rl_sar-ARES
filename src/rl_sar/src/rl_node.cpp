#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"

#include "rl_core.hpp"
#include "loop.hpp"
#include "driver_mode.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

using Lock = std::lock_guard<std::mutex>;

class AresRLNode : public rclcpp::Node
{
public:
    explicit AresRLNode(const std::string& policy_name)
        : Node("ares_rl_node")
    {
        using namespace std::placeholders;

        motor_command_pub_ = create_publisher<sensor_msgs::msg::JointState>("/motor_command", 10);
        motor_param_pub_   = create_publisher<sensor_msgs::msg::JointState>("/motor_param_update", 1);

        driver_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
            "/driver_mode", 10, std::bind(&AresRLNode::DriverModeCallback, this, _1));
        motor_feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/motor_feedback", 10, std::bind(&AresRLNode::MotorFeedbackCallback, this, _1));
        imu_sub_  = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&AresRLNode::ImuCallback, this, _1));
        xbox_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/xbox_vel", 10, std::bind(&AresRLNode::XboxVelCallback, this, _1));
        policy_cycle_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/policy_cycle", 10, std::bind(&AresRLNode::PolicyCycleCallback, this, _1));

        LoadPoliciesList();
        selected_policy_ = policy_name.empty() ? "dogv2_cts/cts" : policy_name;
        current_policy_index_ = FindPolicyIndex(selected_policy_);

        if (!InitRL(selected_policy_)) {
            RCLCPP_FATAL(get_logger(), "RL init failed for %s — aborting", selected_policy_.c_str());
            rclcpp::shutdown();
            return;
        }

        loop_control_ = std::make_shared<LoopFunc>("loop_control", rl_.GetDt(),
                                                     std::bind(&AresRLNode::RobotControl, this));
        loop_rl_ = std::make_shared<LoopFunc>("loop_rl", rl_.GetDt() * rl_.GetDecimation(),
                                                std::bind(&AresRLNode::ModelLoop, this));
        loop_control_->start();
        loop_rl_->start();

        RCLCPP_INFO(get_logger(), "RL node ready (%s) — waiting for driver_mode=RL", selected_policy_.c_str());
    }

    ~AresRLNode()
    {
        if (loop_control_) loop_control_->shutdown();
        if (loop_rl_) loop_rl_->shutdown();
    }

private:
    // 从 policies.yaml 加载可用策略列表，支持 gamepad SELECT_LOCOMOTION 切换
    void LoadPoliciesList()
    {
        available_policies_.clear();
        // 拼接路径: POLICY_DIR/policies.yaml (编译时定义 POLICY_DIR)
        std::string path = std::string(POLICY_DIR) + "/policies.yaml";
        try {
            YAML::Node root = YAML::LoadFile(path);
            // 遍历 YAML key-value，value 是策略子目录名 (如 "dogv2_cts/cts")
            for (const auto& kv : root) {
                std::string name = kv.second.as<std::string>();
                available_policies_.push_back(name);
            }
        } catch (...) {
            RCLCPP_FATAL(get_logger(), "Failed to load %s, aborting", path.c_str());
            rclcpp::shutdown();
            std::exit(1);
        }
        if (available_policies_.empty()) {
            RCLCPP_FATAL(get_logger(), "policies.yaml is empty, aborting");
            rclcpp::shutdown();
            std::exit(1);
        }
        // 打印加载结果，便于调试
        RCLCPP_INFO(get_logger(), "Available policies (%zu):", available_policies_.size());
        for (size_t i = 0; i < available_policies_.size(); ++i)
            RCLCPP_INFO(get_logger(), "  [%zu] %s", i, available_policies_[i].c_str());
    }

    // 根据策略名查找其在 available_policies_ 中的索引，未找到返回 0（第一个策略）
    int FindPolicyIndex(const std::string& name)
    {
        for (size_t i = 0; i < available_policies_.size(); ++i)
            if (available_policies_[i] == name) return static_cast<int>(i);
        return 0;
    }

    bool InitRL(const std::string& policy_name)
    {
        if (!rl_.Init(std::string(POLICY_DIR), policy_name))
            return false;

        const auto& kp = rl_.GetKp();
        const auto& kd = rl_.GetKd();
        const auto& tl = rl_.GetTorqueLimits();
        int ndof = rl_.GetNumDofs();
        if ((int)kp.size() != ndof || (int)kd.size() != ndof || (int)tl.size() != ndof) {
            RCLCPP_FATAL(get_logger(),
                "Motor param size mismatch: ndof=%d kp=%zu kd=%zu torque=%zu",
                ndof, kp.size(), kd.size(), tl.size());
            return false;
        }

        PublishMotorParams();
        return true;
    }

    void DriverModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        uint8_t raw = msg->data;
        if (raw > static_cast<uint8_t>(DriverMode::CLIMB))
            return;
        DriverMode new_mode = static_cast<DriverMode>(raw);

        if (new_mode == DriverMode::RL && driver_mode_ != DriverMode::RL) {
            rl_.SetState(AresRL::State::RUNNING);
            RCLCPP_INFO(get_logger(), "Driver → RL: starting inference (%s)", selected_policy_.c_str());
        } else if (new_mode != DriverMode::RL && driver_mode_ == DriverMode::RL) {
            rl_.SetState(AresRL::State::STOPPED);
            RCLCPP_INFO(get_logger(), "Driver left RL mode");
        }

        driver_mode_ = new_mode;
    }

    void ModelLoop()
    {
        if (!rl_.IsInitialized())
            return;
        if (driver_mode_ != DriverMode::RL)
            return;

        std::array<float, 12> joint_pos, joint_vel, joint_torque;
        std::array<float, 3> imu_gyro, imu_gravity;
        std::vector<float> commands;
        bool ready;

        {
            Lock lock(data_mutex_);
            ready = all_sensors_ready_;
            if (ready) {
                imu_gyro    = imu_gyro_;
                imu_gravity = imu_gravity_;
                commands    = commands_buffer_;
                joint_pos   = joint_pos_;
                joint_vel   = joint_vel_;
                joint_torque = joint_torque_;
            }
        }

        if (!ready) return;

        rl_.RunModel(imu_gyro.data(), imu_gravity.data(), commands.data(),
                     commands.size(), joint_pos.data(), joint_vel.data(),
                     joint_torque.data());
    }

    void RobotControl()
    {
        if (driver_mode_ != DriverMode::RL)
            return;
        if (!rl_.IsInitialized())
            return;

        if (!all_sensors_ready_) {
            auto now = this->now();
            if (!last_sensor_warn_ || (now - *last_sensor_warn_).seconds() >= 2.0) {
                last_sensor_warn_ = now;
                if (!imu_received_ && !motor_feedback_received_)
                    RCLCPP_WARN(get_logger(), "Waiting for sensors: no /imu/data and no /motor_feedback");
                else if (!imu_received_)
                    RCLCPP_WARN(get_logger(), "Waiting for sensors: no /imu/data");
                else
                    RCLCPP_WARN(get_logger(), "Waiting for sensors: no /motor_feedback");
            }
            return;
        }

        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = now();
        {
            Lock lock(output_mutex_);
            const auto& target = rl_.GetTargetPositions();
            const auto& limits = rl_.GetPositionLimits();
            for (int i = 0; i < rl_.GetNumDofs(); ++i) {
                float pos = target[rl_.GetDriverToTopic()[i]];
                if (!limits.empty() && static_cast<size_t>(i) < limits.size())
                    pos = std::clamp(pos, limits[i].first, limits[i].second);
                cmd.position.push_back(pos);
            }
        }
        motor_command_pub_->publish(cmd);
    }

    void PublishMotorParams()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = now();
        const auto& kp = rl_.GetKp();
        const auto& kd = rl_.GetKd();
        const auto& tl = rl_.GetTorqueLimits();
        for (int i = 0; i < rl_.GetNumDofs(); ++i) {
            msg.position.push_back(kp[i]);
            msg.velocity.push_back(kd[i]);
            msg.effort.push_back(tl[i]);
        }
        motor_param_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published /motor_param_update: kp[0]=%.1f kd[0]=%.2f torque[0]=%.1f",
                     kp.empty() ? 0.f : kp[0], kd.empty() ? 0.f : kd[0],
                     tl.empty() ? 0.f : tl[0]);
    }

    void MotorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        Lock lock(data_mutex_);
        size_t n = std::min(msg->position.size(), static_cast<size_t>(rl_.GetNumDofs()));
        for (size_t i = 0; i < n; ++i) {
            int topic_idx = rl_.GetDriverToTopic()[i];
            joint_pos_[topic_idx]    = msg->position[i];
            joint_vel_[topic_idx]    = msg->velocity[i];
            joint_torque_[topic_idx] = msg->effort[i];
        }
        motor_feedback_received_ = true;
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
        const auto& limits = rl_.GetGamepadLimits();
        int n = rl_.GetNumCommands();
        if ((int)commands_buffer_.size() < n)
            commands_buffer_.resize(n, 0.0f);

        if (n > 0) commands_buffer_[0] = std::clamp(static_cast<float>(msg->linear.x), limits[0].first, limits[0].second);
        if (n > 1) commands_buffer_[1] = std::clamp(static_cast<float>(msg->linear.y), limits[1].first, limits[1].second);
        if (n > 2) commands_buffer_[2] = std::clamp(static_cast<float>(msg->angular.z), limits[2].first, limits[2].second);
        if (n > 3) commands_buffer_[3] = std::clamp(static_cast<float>(msg->linear.z), limits[3].first, limits[3].second);
    }

    void PolicyCycleCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        int direction = msg->data;
        if (direction == 0) return;

        int n = static_cast<int>(available_policies_.size());
        current_policy_index_ = (current_policy_index_ + direction + n) % n;
        std::string new_policy = available_policies_[current_policy_index_];

        if (new_policy == selected_policy_) return;

        RCLCPP_INFO(get_logger(), "Policy → %s (%d/%d)", new_policy.c_str(), current_policy_index_ + 1, n);

        if (!rl_.SwitchPolicy(std::string(POLICY_DIR), new_policy)) {
            RCLCPP_ERROR(get_logger(), "Policy switch failed for %s", new_policy.c_str());
            return;
        }

        selected_policy_ = new_policy;
        PublishMotorParams();

        const auto& kp = rl_.GetKp();
        const auto& kd = rl_.GetKd();
        const auto& tl = rl_.GetTorqueLimits();
        RCLCPP_INFO(get_logger(), "Policy switched to %s — kp[0]=%.1f kd[0]=%.2f torque[0]=%.1f",
                     selected_policy_.c_str(),
                     kp.empty() ? 0.f : kp[0], kd.empty() ? 0.f : kd[0],
                     tl.empty() ? 0.f : tl[0]);
    }

    AresRL rl_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_, motor_param_pub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr driver_mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr policy_cycle_sub_;
    std::shared_ptr<LoopFunc> loop_control_, loop_rl_;

    std::mutex data_mutex_, output_mutex_;

    std::array<float, 12> joint_pos_{}, joint_vel_{}, joint_torque_{};
    std::array<float, 3>  imu_gyro_{}, imu_gravity_{};
    std::vector<float>    commands_buffer_;

    bool imu_received_{false}, motor_feedback_received_{false};
    bool all_sensors_ready_{false};

    DriverMode driver_mode_{DriverMode::DISABLE};
    std::optional<rclcpp::Time> last_sensor_warn_;

    std::vector<std::string> available_policies_;
    int current_policy_index_{0};
    std::string selected_policy_{"dogv2_cts/cts"};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AresRLNode>(
        argc > 1 ? argv[1] : "dogv2_cts/cts");
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

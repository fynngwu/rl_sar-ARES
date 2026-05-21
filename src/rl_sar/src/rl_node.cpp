#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rl_core.hpp"
#include "loop.hpp"
#include "keyboard_helper.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <map>

using Lock = std::lock_guard<std::mutex>;

class ARSNode : public rclcpp::Node
{
public:
    explicit ARSNode(const std::string& policy_name)
        : Node("ares_rl_node")
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

        {
            std::string policies_path = std::string(POLICY_DIR) + "/policies.yaml";
            YAML::Node root = YAML::LoadFile(policies_path);
            for (const auto& kv : root)
                policy_map_[kv.first.as<std::string>()[0]] = kv.second.as<std::string>();
        }

        std::string selected = policy_name;
        bool policy_found = false;
        for (const auto& [key, name] : policy_map_)
            if (name == selected) { policy_found = true; break; }
        if (!policy_found && !policy_map_.empty()) {
            selected = policy_map_.begin()->second;
            RCLCPP_WARN(get_logger(), "Policy not found, using first: %s", selected.c_str());
        }

        if (!InitRL(selected)) { RCLCPP_ERROR(get_logger(), "RL init failed!"); return; }

        loop_control_ = std::make_shared<LoopFunc>("loop_control", rl_.GetDt(),
                                                     std::bind(&ARSNode::RobotControl, this));
        loop_rl_ = std::make_shared<LoopFunc>("loop_rl", rl_.GetDt() * rl_.GetDecimation(),
                                                std::bind(&ARSNode::ModelLoop, this));
        loop_control_->start();  loop_rl_->start();

        std::string help = "\n=== ARES RL Controls ===\n";
        for (const auto& [key, name] : policy_map_)
            help += "  [" + std::string(1, key) + "] " + name + "\n";
        help += "  [0] STOP\n  Switch policy from STOPPED\n\n";
        printf("%s", help.c_str());
    }

    ~ARSNode()
    {
        if (loop_control_) loop_control_->shutdown();
        if (loop_rl_) loop_rl_->shutdown();
    }

private:
    bool InitRL(const std::string& policy_name)
    {
        if (!rl_.Init(std::string(POLICY_DIR), policy_name)) {
            return false;
        }

        PublishMotorParams();
        all_sensors_ready_ = false;
        return true;
    }

    void ModelLoop()
    {
        if (!rl_.IsInitialized())
            return;

        std::array<float, 12> joint_pos, joint_vel, joint_torque;
        std::array<float, 3> imu_gyro, imu_gravity, commands;
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
                     joint_pos.data(), joint_vel.data(), joint_torque.data());
    }

    void RobotControl()
    {
        if (!rl_.IsInitialized()) return;

        int key = kbhit();

        if (key == '0' && rl_.GetState() != AresRL::State::STOPPED) {
            sensor_msgs::msg::JointState cmd;
            cmd.header.stamp = now();
            const auto& default_pos = rl_.GetDefaultDofPos();
            const auto& limits = rl_.GetPositionLimits();
            for (int i = 0; i < rl_.GetNumDofs(); ++i) {
                float pos = default_pos[rl_.GetDriverToTopic()[i]];
                if (!limits.empty() && static_cast<size_t>(i) < limits.size())
                    pos = std::clamp(pos, limits[i].first, limits[i].second);
                cmd.position.push_back(pos);
            }
            motor_command_pub_->publish(cmd);
            rl_.SetState(AresRL::State::STOPPED);
            printf("[RL] STOPPED\n");
            return;
        }

        if (policy_map_.count(static_cast<char>(key)) && rl_.GetState() == AresRL::State::STOPPED) {
            auto it = policy_map_.find(static_cast<char>(key));
            if (it != policy_map_.end()) {
                printf("[RL] Loading %s ...\n", it->second.c_str());
                if (InitRL(it->second)) {
                    all_sensors_ready_ = true;
                    rl_.SetState(AresRL::State::RUNNING);
                    printf("[RL] RUNNING (%s)\n", it->second.c_str());
                } else {
                    printf("[RL] Reinit FAILED\n");
                }
            }
            return;
        }

        if (!all_sensors_ready_) return;
        if (rl_.GetState() == AresRL::State::STOPPED) return;

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
            msg.position.push_back(
                i < (int)kp.size() ? kp[i] : 20.0f);
            msg.velocity.push_back(
                i < (int)kd.size() ? kd[i] : 1.0f);
            msg.effort.push_back(
                i < (int)tl.size() ? tl[i] : 17.0f);
        }
        motor_param_pub_->publish(msg);
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
        commands_buffer_[0] = msg->linear.x;
        commands_buffer_[1] = msg->linear.y;
        commands_buffer_[2] = msg->angular.z;
    }

    AresRL rl_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_, motor_param_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_sub_;
    std::shared_ptr<LoopFunc> loop_control_, loop_rl_;

    std::mutex data_mutex_, output_mutex_;

    std::array<float, 12> joint_pos_{}, joint_vel_{}, joint_torque_{};
    std::array<float, 3>  commands_buffer_{}, imu_gyro_{}, imu_gravity_{};

    bool imu_received_{false}, motor_feedback_received_{false};
    bool all_sensors_ready_{false};

    std::map<char, std::string> policy_map_;
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

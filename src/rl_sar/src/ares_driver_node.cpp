/*
 * ARES Driver Node — thin ROS2 wrapper around AresDriverCore.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "ares_driver_core.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

static constexpr const char* kJointNames[AresDriverCore::NUM_JOINTS] = {
    "fl_hipa", "fl_hipf", "fl_knee",
    "rl_hipa", "rl_hipf", "rl_knee",
    "fr_hipa", "fr_hipf", "fr_knee",
    "rr_hipa", "rr_hipf", "rr_knee"
};

class AresDriverNode : public rclcpp::Node
{
public:
    explicit AresDriverNode(const std::string& policy_name)
        : Node("ares_driver_node"),
          core_(std::make_unique<AresDriverCore>(std::string(POLICY_DIR), policy_name))
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ARES Driver Node...");
        RCLCPP_INFO(this->get_logger(), "  kp: %s", FmtFloatVec(core_->config_kp()).c_str());
        RCLCPP_INFO(this->get_logger(), "  kd: %s", FmtFloatVec(core_->config_kd()).c_str());
        RCLCPP_INFO(this->get_logger(), "  torque_limits: %s", FmtFloatVec(core_->config_torque()).c_str());
        RCLCPP_INFO(this->get_logger(), "  gamepad_scale: %.2f", core_->gamepad_scale());
        RCLCPP_INFO(this->get_logger(), "DogDriver ready. IMU: %s", core_->imu_connected() ? "yes" : "no");

        if (core_->gamepad_connected())
            RCLCPP_INFO(this->get_logger(), "Gamepad: %s", core_->gamepad_name().c_str());
        else
            RCLCPP_WARN(this->get_logger(), "No gamepad at /dev/input/js0");

        motor_feedback_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/motor_feedback", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        xbox_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/xbox_vel", 10);

        motor_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor_command", 10,
            std::bind(&AresDriverNode::MotorCommandCallback, this, std::placeholders::_1));

        motor_param_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor_param_update", 1,
            std::bind(&AresDriverNode::MotorParamCallback, this, std::placeholders::_1));

        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&AresDriverNode::FeedbackTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "ARES Driver Node started");
        core_->PrintModeHelp();
    }

private:
    std::string FmtFloatVec(const std::vector<float>& v)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << "[";
        for (size_t i = 0; i < v.size(); ++i)
            oss << (i ? "," : "") << v[i];
        oss << "]";
        return oss.str();
    }

    void MotorCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < AresDriverCore::NUM_JOINTS)
            return;

        std::array<float, AresDriverCore::NUM_JOINTS> target;
        for (int i = 0; i < AresDriverCore::NUM_JOINTS; ++i)
            target[i] = msg->position[i];
        core_->SetTopicCommand(target);
    }

    void MotorParamCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < AresDriverCore::NUM_JOINTS)
            return;
        std::vector<float> kp(msg->position.begin(), msg->position.begin() + AresDriverCore::NUM_JOINTS);
        std::vector<float> kd(AresDriverCore::NUM_JOINTS, 0.0f);
        std::vector<float> torque;
        if (msg->velocity.size() >= AresDriverCore::NUM_JOINTS)
            kd.assign(msg->velocity.begin(), msg->velocity.begin() + AresDriverCore::NUM_JOINTS);
        if (msg->effort.size() >= AresDriverCore::NUM_JOINTS)
            torque.assign(msg->effort.begin(), msg->effort.begin() + AresDriverCore::NUM_JOINTS);
        core_->SetMotorParams(kp, kd, torque);
        RCLCPP_INFO(this->get_logger(), "Motor params updated from /motor_param_update");
    }

    void FeedbackTimerCallback()
    {
        auto joint_states = core_->GetTopicFeedback();
        sensor_msgs::msg::JointState feedback_msg;
        feedback_msg.header.stamp = this->now();
        for (int i = 0; i < AresDriverCore::NUM_JOINTS; ++i) {
            feedback_msg.name.push_back(kJointNames[i]);
            feedback_msg.position.push_back(joint_states.position[i]);
            feedback_msg.velocity.push_back(joint_states.velocity[i]);
            feedback_msg.effort.push_back(joint_states.torque[i]);
        }
        motor_feedback_pub_->publish(feedback_msg);

        auto imu_data = core_->GetImuData();
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.w = 1.0;
        imu_msg.angular_velocity.x = imu_data.angular_velocity[0];
        imu_msg.angular_velocity.y = imu_data.angular_velocity[1];
        imu_msg.angular_velocity.z = imu_data.angular_velocity[2];
        imu_msg.linear_acceleration.x = imu_data.projected_gravity[0];
        imu_msg.linear_acceleration.y = imu_data.projected_gravity[1];
        imu_msg.linear_acceleration.z = imu_data.projected_gravity[2];
        imu_pub_->publish(imu_msg);

        auto gamepad = core_->PollGamepad();
        if (gamepad.connected) {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = gamepad.linear_x;
            twist.linear.y = gamepad.linear_y;
            twist.linear.z = gamepad.linear_z;
            twist.angular.z = gamepad.angular_z;
            xbox_vel_pub_->publish(twist);
        }
    }

    std::unique_ptr<AresDriverCore> core_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_param_sub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string policy_name = (argc > 1) ? argv[1] : "ares_himloco/himloco";
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ARES Driver Node (policy: %s)...", policy_name.c_str());

    auto node = std::make_shared<AresDriverNode>(policy_name);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

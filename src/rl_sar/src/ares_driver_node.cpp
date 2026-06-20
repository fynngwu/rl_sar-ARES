/*
 * ARES Driver Node — thin ROS2 wrapper around AresDriverCore.
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "ares_driver_core.hpp"
#include "joint_names.hpp"
#include "remote_command.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

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
        remote_cmd_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/remote_command", 10);

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
    static const char* RemoteCommandName(RemoteCommand cmd)
    {
        switch (cmd) {
        case RemoteCommand::NONE: return "NONE";
        case RemoteCommand::RECOVER_STAND: return "RECOVER_STAND";
        case RemoteCommand::SELECT_LOCOMOTION: return "SELECT_LOCOMOTION";
        case RemoteCommand::START_DREAMWAQ: return "START_DREAMWAQ";
        case RemoteCommand::DISABLE: return "DISABLE";
        case RemoteCommand::DAMPING: return "DAMPING";
        case RemoteCommand::TOGGLE_RECORD: return "TOGGLE_RECORD";
        }
        return "UNKNOWN";
    }

    static const char* DriverModeName(DriverMode mode)
    {
        switch (mode) {
        case DriverMode::DISABLE: return "DISABLE";
        case DriverMode::STAND: return "STAND";
        case DriverMode::RL: return "RL";
        case DriverMode::DAMPING: return "DAMPING";
        }
        return "UNKNOWN";
    }

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

        if (pending_rl_enable_ && core_->GetMode() == DriverMode::STAND) {
            bool ok = core_->RequestModeChange(DriverMode::RL);
            RCLCPP_INFO(
                this->get_logger(),
                "First prepared /motor_command received. RequestModeChange(RL) result=%s, driver mode after=%s",
                ok ? "ok" : "rejected",
                DriverModeName(core_->GetMode()));
            if (ok)
                pending_rl_enable_ = false;
        }

        auto now = this->now();
        if (!last_motor_cmd_log_time_ ||
            (now - *last_motor_cmd_log_time_).seconds() >= 1.0) {
            last_motor_cmd_log_time_ = now;
            RCLCPP_INFO(
                this->get_logger(),
                "Received /motor_command: [%.3f, %.3f, %.3f, ...] mode=%s",
                target[0], target[1], target[2],
                DriverModeName(core_->GetMode()));
        }
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
        PublishRemoteCommand();
        LogDriverState();

        auto joint_states = core_->GetTopicFeedback();
        sensor_msgs::msg::JointState feedback_msg;
        feedback_msg.header.stamp = this->now();
        for (int i = 0; i < AresDriverCore::NUM_JOINTS; ++i) {
            feedback_msg.name.push_back(kJointNamesByLeg[i]);
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

    void PublishRemoteCommand()
    {
        RemoteCommand cmd = DetectRemoteCommand();
        if (cmd == RemoteCommand::NONE) {
            last_remote_cmd_ = RemoteCommand::NONE;
            return;
        }

        if (cmd == last_remote_cmd_)
            return;

        last_remote_cmd_ = cmd;

        RCLCPP_INFO(
            this->get_logger(),
            "Remote command detected: %s (driver mode before=%s)",
            RemoteCommandName(cmd),
            DriverModeName(core_->GetMode()));

        std_msgs::msg::UInt8 msg;
        msg.data = static_cast<uint8_t>(cmd);
        remote_cmd_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published /remote_command: %s", RemoteCommandName(cmd));

        switch (cmd) {
        case RemoteCommand::RECOVER_STAND:
        {
            pending_rl_enable_ = false;
            bool ok = core_->RequestModeChange(DriverMode::STAND);
            RCLCPP_INFO(
                this->get_logger(),
                "RequestModeChange(STAND) result=%s, driver mode after=%s",
                ok ? "ok" : "rejected",
                DriverModeName(core_->GetMode()));
            break;
        }
        case RemoteCommand::DISABLE:
        {
            pending_rl_enable_ = false;
            bool ok = core_->RequestModeChange(DriverMode::DISABLE);
            RCLCPP_INFO(
                this->get_logger(),
                "RequestModeChange(DISABLE) result=%s, driver mode after=%s",
                ok ? "ok" : "rejected",
                DriverModeName(core_->GetMode()));
            break;
        }
        case RemoteCommand::DAMPING:
        {
            pending_rl_enable_ = false;
            bool ok = core_->RequestModeChange(DriverMode::DAMPING);
            RCLCPP_INFO(
                this->get_logger(),
                "RequestModeChange(DAMPING) result=%s, driver mode after=%s",
                ok ? "ok" : "rejected",
                DriverModeName(core_->GetMode()));
            RCLCPP_WARN(this->get_logger(), "Damping mode is reserved and may change in a later version.");
            break;
        }
        case RemoteCommand::SELECT_LOCOMOTION:
            RCLCPP_INFO(this->get_logger(), "Locomotion family selected.");
            break;
        case RemoteCommand::START_DREAMWAQ:
        {
            if (core_->GetMode() != DriverMode::STAND) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Ignoring RL start because driver mode is %s, expected STAND",
                    DriverModeName(core_->GetMode()));
            } else {
                pending_rl_enable_ = true;
                RCLCPP_INFO(
                    this->get_logger(),
                    "RL start requested. Waiting for first prepared /motor_command before switching driver to RL.");
            }
            break;
        }
        case RemoteCommand::TOGGLE_RECORD:
            break;
        case RemoteCommand::NONE:
            break;
        }
    }

    RemoteCommand DetectRemoteCommand()
    {
        if (!core_->gamepad_connected())
            return RemoteCommand::NONE;

        const bool a = core_->GetGamepadButton(0);
        // LOGIC USB gamepad on this machine reports physical X/Y opposite to the
        // initial assumption, so map X->button 2 and Y->button 3.
        const bool x = core_->GetGamepadButton(2);
        const bool y = core_->GetGamepadButton(3);
        const bool lb = core_->GetGamepadButton(4);
        const bool rb = core_->GetGamepadButton(5);
        const bool start = core_->GetGamepadButton(7);
        const bool lt = core_->GetGamepadAxis(2) > 0.5f;
        const bool rt = core_->GetGamepadAxis(5) > 0.5f;

        if (a || x || y || lb || rb || start || lt || rt) {
            std::ostringstream sig;
            sig << a << x << y << lb << rb << start << lt << rt;
            if (sig.str() != last_gamepad_signature_) {
                last_gamepad_signature_ = sig.str();
                RCLCPP_INFO(
                    this->get_logger(),
                    "Gamepad state changed: A=%d X=%d Y=%d LB=%d RB=%d START=%d LT=%.3f RT=%.3f mode=%s",
                    a, x, y, lb, rb, start,
                    core_->GetGamepadAxis(2),
                    core_->GetGamepadAxis(5),
                    DriverModeName(core_->GetMode()));
            }
        } else {
            last_gamepad_signature_.clear();
        }

        if (rb && x)
            return RemoteCommand::DISABLE;
        if (lb && rb)
            return RemoteCommand::DAMPING;
        if (lb && start)
            return RemoteCommand::TOGGLE_RECORD;
        if (lb && a)
            return RemoteCommand::RECOVER_STAND;
        if (lb && y)
            return RemoteCommand::SELECT_LOCOMOTION;
        if (lt && y)
            return RemoteCommand::START_DREAMWAQ;
        return RemoteCommand::NONE;
    }

    void LogDriverState()
    {
        auto now = this->now();
        DriverMode mode = core_->GetMode();
        if (mode != last_logged_mode_ ||
            !last_state_log_time_ ||
            (now - *last_state_log_time_).seconds() >= 1.0) {
            last_logged_mode_ = mode;
            last_state_log_time_ = now;
            RCLCPP_INFO(
                this->get_logger(),
                "Driver state: mode=%s",
                DriverModeName(core_->GetMode()));
        }
    }

    std::unique_ptr<AresDriverCore> core_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr remote_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_param_sub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    mutable RemoteCommand last_remote_cmd_{RemoteCommand::NONE};
    std::optional<rclcpp::Time> last_state_log_time_;
    std::optional<rclcpp::Time> last_motor_cmd_log_time_;
    DriverMode last_logged_mode_{DriverMode::DISABLE};
    std::string last_gamepad_signature_;
    bool pending_rl_enable_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string policy_name = (argc > 1) ? argv[1] : "dream_waq/dream_waq";
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ARES Driver Node (policy: %s)...", policy_name.c_str());

    auto node = std::make_shared<AresDriverNode>(policy_name);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

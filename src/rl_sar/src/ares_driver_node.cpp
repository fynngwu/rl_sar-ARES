/*
 * ARES Driver Node — ROS2 wrapper for libdog_driver.so
 *
 * Publishes:
 *   /motor_feedback  (sensor_msgs/JointState) — 12 joint pos/vel from CAN motors
 *   /imu/data        (sensor_msgs/Imu)        — gyro + projected_gravity from WIT IMU
 *   /xbox_vel        (geometry_msgs/Twist)    — gamepad velocity command
 *
 * Subscribes:
 *   /motor_command   (sensor_msgs/JointState) — 12 joint target positions
 *
 * Hardware:
 *   4 CAN buses (can0-can3), 12 Robstride motors, 1 WIT IMU
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "dog_driver.hpp"
#include "observations.hpp"

#include <chrono>
#include <cmath>
#include <thread>
#include <atomic>
#include <array>
#include <string>

static constexpr const char* kJointNames[DogDriver::NUM_JOINTS] = {
    "lf_hipa", "lr_hipa", "rf_hipa", "rr_hipa",
    "lf_hipf", "lr_hipf", "rf_hipf", "rr_hipf",
    "lf_knee", "lr_knee", "rf_knee", "rr_knee"
};

class AresDriverNode : public rclcpp::Node
{
public:
    AresDriverNode()
        : Node("ares_driver_node"),
          running_(true)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ARES Driver Node...");

        // --- DogDriver (opens CAN + IMU, enables motors, sets MIT params) ---
        try {
            driver_ = std::make_unique<DogDriver>();
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "DogDriver init failed: %s", e.what());
            throw;
        }

        RCLCPP_INFO(this->get_logger(), "DogDriver ready. IMU connected: %s",
                    driver_->IsIMUConnected() ? "yes" : "no");

        // --- Publishers ---
        motor_feedback_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/motor_feedback", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);
        xbox_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/xbox_vel", 10);

        // --- Subscribers ---
        motor_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor_command", 10,
            std::bind(&AresDriverNode::MotorCommandCallback, this, std::placeholders::_1));

        // --- Gamepad ---
        try {
            gamepad_ = std::make_unique<Gamepad>("/dev/input/js0");
            RCLCPP_INFO(this->get_logger(), "Gamepad connected: %s", gamepad_->GetName().c_str());
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "No gamepad found at /dev/input/js0");
        }

        // --- Control thread (200Hz) ---
        control_thread_ = std::thread(&AresDriverNode::ControlLoop, this);

        RCLCPP_INFO(this->get_logger(), "ARES Driver Node started");
    }

    ~AresDriverNode()
    {
        running_ = false;
        if (control_thread_.joinable())
            control_thread_.join();
        if (driver_)
            driver_->DisableAll();
        RCLCPP_INFO(this->get_logger(), "ARES Driver Node stopped");
    }

private:
    void MotorCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        // Store latest target positions from the RL node
        if (msg->position.size() >= DogDriver::NUM_JOINTS)
        {
            for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
                latest_target_[i] = msg->position[i];
            has_new_command_ = true;
        }
    }

    void ControlLoop()
    {
        constexpr double dt = 0.005; // 200Hz
        auto next_wakeup = std::chrono::steady_clock::now();

        while (running_ && rclcpp::ok())
        {
            // --- Publish motor feedback ---
            auto joint_states = driver_->GetJointStates();

            sensor_msgs::msg::JointState feedback_msg;
            feedback_msg.header.stamp = this->now();
            for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            {
                feedback_msg.name.push_back(kJointNames[i]);
                feedback_msg.position.push_back(joint_states.position[i]);
                feedback_msg.velocity.push_back(joint_states.velocity[i]);
            }
            motor_feedback_pub_->publish(feedback_msg);

            // --- Publish IMU data ---
            auto imu_data = driver_->GetIMUData();

            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";
            // Dummy orientation — unused by rl_real_ares.cpp
            imu_msg.orientation.w = 1.0;
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            // Angular velocity (body frame, rad/s) — from WIT IMU
            imu_msg.angular_velocity.x = imu_data.angular_velocity[0];
            imu_msg.angular_velocity.y = imu_data.angular_velocity[1];
            imu_msg.angular_velocity.z = imu_data.angular_velocity[2];
            // Projected gravity (body frame, unit vector) — carried in linear_acceleration
            // (No standard ROS2 field for projected gravity; rl_real_ares.cpp reads this directly)
            imu_msg.linear_acceleration.x = imu_data.projected_gravity[0];
            imu_msg.linear_acceleration.y = imu_data.projected_gravity[1];
            imu_msg.linear_acceleration.z = imu_data.projected_gravity[2];
            imu_pub_->publish(imu_msg);

            // --- Send motor commands ---
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                if (has_new_command_)
                {
                    driver_->SetAllJointPositions(latest_target_);
                    has_new_command_ = false;
                }
            }

            // --- Gamepad ---
            if (gamepad_ && gamepad_->IsConnected())
            {
                geometry_msgs::msg::Twist twist;
                // axis1 = left stick X (forward/back), axis3 = right stick X (yaw)
                // Match CommandComponent convention from driver's observations.cpp
                twist.linear.x  = gamepad_->GetAxis(1) * 0.5f;
                twist.linear.y  = 0.0f;
                twist.angular.z = gamepad_->GetAxis(3) * 0.5f;
                xbox_vel_pub_->publish(twist);
            }

            // --- Timing ---
            next_wakeup += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
            std::this_thread::sleep_until(next_wakeup);
        }
    }

    // Hardware
    std::unique_ptr<DogDriver> driver_;
    std::unique_ptr<Gamepad> gamepad_;

    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_command_sub_;

    // Motor command buffer
    std::mutex cmd_mutex_;
    std::array<float, DogDriver::NUM_JOINTS> latest_target_{};
    bool has_new_command_ = false;

    // Control thread
    std::thread control_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ARES Driver Node...");

    auto node = std::make_shared<AresDriverNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main"), "ARES Driver Node shutting down...");
    rclcpp::shutdown();
    return 0;
}

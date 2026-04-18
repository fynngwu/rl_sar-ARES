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
 *   /motor_feedback and /motor_command use FL RL FR RR ordering (URDF order)
 *   - ares_driver_node handles reordering to/from driver internal order
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
#include <utility>
#include <fstream>

#include "yaml-cpp/yaml.h"

static constexpr const char* kJointNames[DogDriver::NUM_JOINTS] = {
    "fl_hipa", "fl_hipf", "fl_knee",
    "rl_hipa", "rl_hipf", "rl_knee",
    "fr_hipa", "fr_hipf", "fr_knee",
    "rr_hipa", "rr_hipf", "rr_knee"
};

// Position limits in FL RL FR RR order (joint-space, residual without offset)
// Driver order: HipA(LF/LR/RF/RR) HipF(LF/LR/RF/RR) Knee(LF/LR/RF/RR)
static constexpr std::array<std::pair<float,float>, DogDriver::NUM_JOINTS> kPositionLimits = {{
    {-0.7853982f,  0.7853982f},  // 0  FL_HipA
    {-1.2217658f,  0.8726683f},  // 1  FL_HipF
    {-1.2217299f,  0.6f},        // 2  FL_Knee
    {-0.7853982f,  0.7853982f},  // 3  RL_HipA
    {-1.2217305f,  0.8726683f},  // 4  RL_HipF
    {-1.2217299f,  0.6f},        // 5  RL_Knee
    {-0.7853982f,  0.7853982f},  // 6  FR_HipA
    {-0.8726999f,  1.2217342f},  // 7  FR_HipF
    {-0.6f,        1.2217287f},  // 8  FR_Knee
    {-0.7853982f,  0.7853982f},  // 9  RR_HipA
    {-0.8726999f,  1.2217305f},  // 10 RR_HipF
    {-0.6f,        1.2217287f},  // 11 RR_Knee
}};

// Torque limit for MIT commands (Nm)
static constexpr float TORQUE_LIMIT = 17.0f;

// topic_index -> driver_index (FL RL FR RR -> driver order)
static constexpr int kTopicToDriver[DogDriver::NUM_JOINTS] = {
    0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11
};
// driver_index -> topic_index (driver order -> FL RL FR RR)
static constexpr int kDriverToTopic[DogDriver::NUM_JOINTS] = {
    0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11
};

class AresDriverNode : public rclcpp::Node
{
public:
    AresDriverNode()
        : Node("ares_driver_node"),
          running_(true),
          received_first_command_(false),
          gamepad_scale_(0.5f)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ARES Driver Node...");

        std::string config_path = std::string(POLICY_DIR) + "/ares_himloco/himloco/config.yaml";
        if (std::ifstream(config_path))
        {
            try
            {
                YAML::Node config = YAML::LoadFile(config_path);
                std::string config_key = "ares_himloco";
                if (config["ares_himloco/himloco"])
                    config_key = "ares_himloco/himloco";
                YAML::Node rc = config[config_key];

                if (rc["fixed_kp"])
                    for (const auto &v : rc["fixed_kp"]) config_kp_.push_back(v.as<float>());
                if (rc["fixed_kd"])
                    for (const auto &v : rc["fixed_kd"]) config_kd_.push_back(v.as<float>());
                if (rc["torque_limits"])
                    for (const auto &v : rc["torque_limits"]) config_torque_.push_back(v.as<float>());
                if (rc["gamepad_scale"])
                    gamepad_scale_ = rc["gamepad_scale"].as<float>();

                RCLCPP_INFO(this->get_logger(), "Config loaded: %s", config_path.c_str());
                RCLCPP_INFO(this->get_logger(), "  kp=%d kd=%d torque=%d gamepad_scale=%.2f",
                            (int)config_kp_.size(), (int)config_kd_.size(),
                            (int)config_torque_.size(), gamepad_scale_);
            }
            catch (const YAML::Exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to parse config: %s", e.what());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Config not found: %s, using defaults", config_path.c_str());
        }

        // --- DogDriver (opens CAN + IMU, enables motors, sets MIT params) ---
        try {
            driver_ = std::make_unique<DogDriver>();
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "DogDriver init failed: %s", e.what());
            throw;
        }

        RCLCPP_INFO(this->get_logger(), "DogDriver ready. IMU connected: %s",
                    driver_->IsIMUConnected() ? "yes" : "no");

        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
            float kp = (i < (int)config_kp_.size()) ? config_kp_[i] : DogDriver::DEFAULT_KP;
            float kd = (i < (int)config_kd_.size()) ? config_kd_[i] : DogDriver::DEFAULT_KD;
            float torque = (i < (int)config_torque_.size()) ? config_torque_[i] : TORQUE_LIMIT;
            driver_->SetMITParams(i, kp, kd);
            driver_->SetTorqueLimit(i, torque);
        }
        RCLCPP_INFO(this->get_logger(), "MIT params set for all joints (kp from config, torque from config)");

        // --- Publishers ---
        motor_feedback_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/motor_feedback", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);
        xbox_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/xbox_vel", 10);

        // --- Subscriber ---
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

        // --- Feedback timer (100Hz) ---
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&AresDriverNode::FeedbackTimerCallback, this));

        // --- Command worker thread (200Hz) ---
        worker_thread_ = std::thread(&AresDriverNode::CommandLoop, this);

        RCLCPP_INFO(this->get_logger(), "ARES Driver Node started");
    }

    ~AresDriverNode()
    {
        running_ = false;
        if (worker_thread_.joinable())
            worker_thread_.join();
        if (driver_) {
            driver_->DisableAll();
            driver_->ClearAllErrors();
        }
        RCLCPP_INFO(this->get_logger(), "ARES Driver Node stopped");
    }

private:
    void MotorCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < DogDriver::NUM_JOINTS)
            return;
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            latest_target_[i] = msg->position[i];
        if (!received_first_command_.exchange(true))
            RCLCPP_INFO(this->get_logger(), "First command received! Starting motor control.");
    }

    void FeedbackTimerCallback()
    {
        // --- Publish motor feedback ---
        auto joint_states = driver_->GetJointStates();

        sensor_msgs::msg::JointState feedback_msg;
        feedback_msg.header.stamp = this->now();
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
        {
            int di = kTopicToDriver[i];
            feedback_msg.name.push_back(kJointNames[i]);
            feedback_msg.position.push_back(joint_states.position[di]);
            feedback_msg.velocity.push_back(joint_states.velocity[di]);
        }
        motor_feedback_pub_->publish(feedback_msg);

        // --- Publish IMU data ---
        auto imu_data = driver_->GetIMUData();

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

        // --- Gamepad ---
        if (gamepad_ && gamepad_->IsConnected())
        {
            geometry_msgs::msg::Twist twist;
            twist.linear.x  = gamepad_->GetAxis(1) * gamepad_scale_;
            twist.linear.y  = -gamepad_->GetAxis(0) * gamepad_scale_;
            twist.angular.z = gamepad_->GetAxis(3) * gamepad_scale_;
            xbox_vel_pub_->publish(twist);
        }
    }

    void CommandLoop()
    {
        // --- First-command gate ---
        int wait_count = 0;
        while (running_ && rclcpp::ok() && !received_first_command_.load())
        {
            wait_count++;
            if (wait_count % 1000 == 0)
                RCLCPP_INFO(this->get_logger(), "Waiting for first command...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (!running_ || !rclcpp::ok())
            return;

        RCLCPP_INFO(this->get_logger(), "Start sending motion commands");

        auto next_tick = std::chrono::steady_clock::now();
        auto last_diag_log = next_tick;
        size_t send_loops = 0;
        auto max_send_duration = std::chrono::steady_clock::duration::zero();
        constexpr auto COMMAND_PERIOD = std::chrono::milliseconds(5);

        while (running_ && rclcpp::ok())
        {
            auto send_start = std::chrono::steady_clock::now();

            // Read latest target and apply limits
            std::array<float, DogDriver::NUM_JOINTS> clamped_target;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                clamped_target = latest_target_;
            }
            for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
                clamped_target[i] = std::clamp(clamped_target[i],
                                                kPositionLimits[i].first,
                                                kPositionLimits[i].second);

            std::array<float, DogDriver::NUM_JOINTS> driver_target;
            for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
                driver_target[i] = clamped_target[kDriverToTopic[i]];
            driver_->SetAllJointPositions(driver_target);

            auto send_duration = std::chrono::steady_clock::now() - send_start;
            max_send_duration = std::max(max_send_duration, send_duration);
            send_loops++;

            auto now = std::chrono::steady_clock::now();

            if (send_duration > COMMAND_PERIOD) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Motor send loop overrun: %.2f ms",
                                     std::chrono::duration<double, std::milli>(send_duration).count());
            }

            if (now - last_diag_log >= std::chrono::seconds(1)) {
                RCLCPP_INFO(this->get_logger(),
                            "Motor send diag: loops=%zu avg_hz=%.1f max_send_ms=%.2f",
                            send_loops,
                            static_cast<double>(send_loops) /
                                std::chrono::duration<double>(now - last_diag_log).count(),
                            std::chrono::duration<double, std::milli>(max_send_duration).count());
                send_loops = 0;
                max_send_duration = std::chrono::steady_clock::duration::zero();
                last_diag_log = now;
            }

            next_tick += COMMAND_PERIOD;
            std::this_thread::sleep_until(next_tick);
            if (std::chrono::steady_clock::now() > next_tick + COMMAND_PERIOD)
                next_tick = std::chrono::steady_clock::now();
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

    // Timers / Threads
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    std::thread worker_thread_;

    // Motor command buffer
    std::mutex cmd_mutex_;
    std::array<float, DogDriver::NUM_JOINTS> latest_target_{};
    std::atomic<bool> received_first_command_;

    // Thread control
    std::atomic<bool> running_;

    std::vector<float> config_kp_;
    std::vector<float> config_kd_;
    std::vector<float> config_torque_;
    float gamepad_scale_;
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

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
 *   /motor_feedback and /motor_command use topic order (defined by topic_to_driver in yaml)
 *   - ares_driver_node handles reordering to/from driver internal order
 *   - All config (mapping, PD gains, limits) loaded from config.yaml, no hardcoded defaults
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
#include <stdexcept>
#include <sstream>

#include "yaml-cpp/yaml.h"

// Joint names in topic order (index matches topic_to_driver array position)
static constexpr const char* kJointNames[DogDriver::NUM_JOINTS] = {
    "fl_hipa", "fl_hipf", "fl_knee",
    "rl_hipa", "rl_hipf", "rl_knee",
    "fr_hipa", "fr_hipf", "fr_knee",
    "rr_hipa", "rr_hipf", "rr_knee"
};

// Position limits in topic order (joint-space, rad)
static constexpr std::array<std::pair<float,float>, DogDriver::NUM_JOINTS> kPositionLimits = {{
    {-0.7853982f,  0.7853982f},  // FL_HipA
    {-1.2217658f,  0.8726683f},  // FL_HipF
    {-1.2217299f,  0.6f},        // FL_Knee
    {-0.7853982f,  0.7853982f},  // RL_HipA
    {-1.2217305f,  0.8726683f},  // RL_HipF
    {-1.2217299f,  0.6f},        // RL_Knee
    {-0.7853982f,  0.7853982f},  // FR_HipA
    {-0.8726999f,  1.2217342f},  // FR_HipF
    {-0.6f,        1.2217287f},  // FR_Knee
    {-0.7853982f,  0.7853982f},  // RR_HipA
    {-0.8726999f,  1.2217305f},  // RR_HipF
    {-0.6f,        1.2217287f},  // RR_Knee
}};

class AresDriverNode : public rclcpp::Node
{
public:
    AresDriverNode()
        : Node("ares_driver_node"),
          running_(true),
          received_first_command_(false)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ARES Driver Node...");

        // --- Load config ---
        std::string config_path = std::string(POLICY_DIR) + "/ares_himloco/himloco/config.yaml";
        YAML::Node rc = YAML::LoadFile(config_path)["ares_himloco/himloco"];
        if (!rc)
            throw std::runtime_error("Missing 'ares_himloco/himloco' in " + config_path);

        // Joint mapping: topic_to_driver[topic_idx] = driver_idx
        auto t2d = rc["topic_to_driver"];
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            topic_to_driver_[i] = t2d[i].as<int>();
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            driver_to_topic_[topic_to_driver_[i]] = i;

        for (const auto &v : rc["fixed_kp"])    config_kp_.push_back(v.as<float>());
        for (const auto &v : rc["fixed_kd"])    config_kd_.push_back(v.as<float>());
        for (const auto &v : rc["torque_limits"]) config_torque_.push_back(v.as<float>());
        gamepad_scale_ = rc["gamepad_scale"].as<float>();

        RCLCPP_INFO(this->get_logger(), "Config: %s", config_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  topic_to_driver: %s", FmtIntArr(topic_to_driver_).c_str());
        RCLCPP_INFO(this->get_logger(), "  driver_to_topic: %s", FmtIntArr(driver_to_topic_).c_str());
        RCLCPP_INFO(this->get_logger(), "  kp: %s", FmtFloatVec(config_kp_).c_str());
        RCLCPP_INFO(this->get_logger(), "  kd: %s", FmtFloatVec(config_kd_).c_str());
        RCLCPP_INFO(this->get_logger(), "  torque_limits: %s", FmtFloatVec(config_torque_).c_str());
        RCLCPP_INFO(this->get_logger(), "  gamepad_scale: %.2f", gamepad_scale_);

        // --- DogDriver (opens CAN + IMU, enables motors) ---
        driver_ = std::make_unique<DogDriver>();
        RCLCPP_INFO(this->get_logger(), "DogDriver ready. IMU: %s",
                    driver_->IsIMUConnected() ? "yes" : "no");

        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
            driver_->SetMITParams(i, config_kp_[i], config_kd_[i]);
            driver_->SetTorqueLimit(i, config_torque_[i]);
        }

        // --- Publishers ---
        motor_feedback_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/motor_feedback", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        xbox_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/xbox_vel", 10);

        // --- Subscriber ---
        motor_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor_command", 10,
            std::bind(&AresDriverNode::MotorCommandCallback, this, std::placeholders::_1));

        // --- Gamepad (optional) ---
        try {
            gamepad_ = std::make_unique<Gamepad>("/dev/input/js0");
            RCLCPP_INFO(this->get_logger(), "Gamepad: %s", gamepad_->GetName().c_str());
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "No gamepad at /dev/input/js0");
        }

        // --- Feedback timer (100Hz) ---
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&AresDriverNode::FeedbackTimerCallback, this));

        // --- Command worker thread (200Hz) ---
        worker_thread_ = std::thread(&AresDriverNode::CommandLoop, this);
        RCLCPP_INFO(this->get_logger(), "ARES Driver Node started");
    }

    void SetDampingMode() {
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            driver_->SetMITParams(i, 0.0f, 10.0f);
    }

    ~AresDriverNode()
    {
        running_ = false;
        if (worker_thread_.joinable()) worker_thread_.join();
        if (driver_) driver_->ClearAllErrors();
        RCLCPP_INFO(this->get_logger(), "ARES Driver Node stopped");
    }

private:
    std::string FmtIntArr(const std::array<int, DogDriver::NUM_JOINTS> &a) {
        std::ostringstream oss;
        oss << "[";
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            oss << (i ? "," : "") << a[i];
        oss << "]";
        return oss.str();
    }

    std::string FmtFloatVec(const std::vector<float> &v) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << "[";
        for (size_t i = 0; i < v.size(); ++i)
            oss << (i ? "," : "") << v[i];
        oss << "]";
        return oss.str();
    }

    void MotorCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < DogDriver::NUM_JOINTS) return;
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            latest_target_[i] = msg->position[i];
        if (!received_first_command_.exchange(true))
            RCLCPP_INFO(this->get_logger(), "First command received!");
    }

    void FeedbackTimerCallback()
    {
        // Publish motor feedback (reorder: driver -> topic)
        auto joint_states = driver_->GetJointStates();
        sensor_msgs::msg::JointState feedback_msg;
        feedback_msg.header.stamp = this->now();
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
            int di = topic_to_driver_[i];
            feedback_msg.name.push_back(kJointNames[i]);
            feedback_msg.position.push_back(joint_states.position[di]);
            feedback_msg.velocity.push_back(joint_states.velocity[di]);
            feedback_msg.effort.push_back(joint_states.torque[di]);
        }
        motor_feedback_pub_->publish(feedback_msg);

        // Publish IMU (angular_velocity = gyro, linear_acceleration = projected_gravity)
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

        // Publish gamepad velocity command
        if (gamepad_ && gamepad_->IsConnected()) {
            if (gamepad_->GetButton(7))
                height_value_ = std::min(HEIGHT_MAX, height_value_ + HEIGHT_STEP);
            if (gamepad_->GetButton(10))
                height_value_ = std::max(HEIGHT_MIN, height_value_ - HEIGHT_STEP);

            geometry_msgs::msg::Twist twist;
            twist.linear.x  = gamepad_->GetAxis(1) * gamepad_scale_;
            twist.linear.y  = -gamepad_->GetAxis(0) * gamepad_scale_;
            twist.linear.z  = height_value_;
            twist.angular.z = gamepad_->GetAxis(3) * gamepad_scale_;
            xbox_vel_pub_->publish(twist);
        }
    }

    void CommandLoop()
    {
        // Wait for first RL command before sending to motors
        int wait_count = 0;
        while (running_ && rclcpp::ok() && !received_first_command_.load()) {
            if (++wait_count % 1000 == 0)
                RCLCPP_INFO(this->get_logger(), "Waiting for first command...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (!running_ || !rclcpp::ok()) return;

        RCLCPP_INFO(this->get_logger(), "Start sending motion commands");

        auto next_tick = std::chrono::steady_clock::now();
        auto last_diag_log = next_tick;
        size_t send_loops = 0;
        auto max_send_duration = std::chrono::steady_clock::duration::zero();
        constexpr auto COMMAND_PERIOD = std::chrono::milliseconds(5);

        while (running_ && rclcpp::ok()) {
            auto send_start = std::chrono::steady_clock::now();

            // Clamp target positions and reorder: topic -> driver
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
                driver_target[i] = clamped_target[driver_to_topic_[i]];
            driver_->SetAllJointPositions(driver_target);

            auto send_duration = std::chrono::steady_clock::now() - send_start;
            max_send_duration = std::max(max_send_duration, send_duration);
            send_loops++;

            auto now = std::chrono::steady_clock::now();
            if (send_duration > COMMAND_PERIOD)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Motor send overrun: %.2f ms",
                                     std::chrono::duration<double, std::milli>(send_duration).count());

            if (now - last_diag_log >= std::chrono::seconds(1)) {
                RCLCPP_INFO(this->get_logger(),
                            "Motor diag: hz=%.1f max_ms=%.2f",
                            send_loops / std::chrono::duration<double>(now - last_diag_log).count(),
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

    rclcpp::TimerBase::SharedPtr feedback_timer_;
    std::thread worker_thread_;

    // Motor command buffer
    std::mutex cmd_mutex_;
    std::array<float, DogDriver::NUM_JOINTS> latest_target_{};
    std::atomic<bool> received_first_command_;
    std::atomic<bool> running_;

    // Config from yaml
    std::vector<float> config_kp_;
    std::vector<float> config_kd_;
    std::vector<float> config_torque_;
    float gamepad_scale_;
    std::array<int, DogDriver::NUM_JOINTS> topic_to_driver_{};
    std::array<int, DogDriver::NUM_JOINTS> driver_to_topic_{};
    static constexpr float HEIGHT_MIN = -0.05f;
    static constexpr float HEIGHT_MAX = 0.05f;
    static constexpr float HEIGHT_STEP = 0.005f;
    float height_value_ = 0.0f;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ARES Driver Node...");

    auto node = std::make_shared<AresDriverNode>();

    rclcpp::on_shutdown([node]() {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Setting damping mode...");
        node->SetDampingMode();
    });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

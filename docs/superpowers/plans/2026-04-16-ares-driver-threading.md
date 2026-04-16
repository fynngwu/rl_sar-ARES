# ARES Driver Threading Refactor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor `ares_driver_node.cpp` from a single 200Hz control loop to a wall_timer (100Hz feedback) + worker_thread (200Hz commands) pattern matching the vendor `robstride_ros_sample`, with position limits and diagnostics.

**Architecture:** Two concurrency paths — a ROS2 `wall_timer_` at 10ms reads motor state/IMU/gamepad and publishes; a `std::thread` at 5ms sends clamped joint positions. The worker thread gates on `received_first_command_` to avoid sending zeros. constexpr position limits per joint are applied before every command send.

**Tech Stack:** C++17, ROS2 (rclcpp, sensor_msgs, geometry_msgs), DogDriver shared library (libdog_driver.so)

**Design spec:** `docs/superpowers/specs/2026-04-16-ares-driver-threading-design.md`

---

## File Changed

| File | Action |
|------|--------|
| `src/rl_sar/src/ares_driver_node.cpp` | Rewrite (single file change per spec) |

No other files are touched. No CMakeLists changes needed — the build target already exists.

---

## Position Limits Derivation

Vendor `MOTOR_LIMITS` (in their "joint space", i.e. user-facing before transforms):

| Vendor Index | Joint | Min | Max |
|---|---|---|---|
| 0 | fl_hip | -1.50 | 0.00 |
| 1 | fl_thigh | -0.79 | 0.79 |
| 2 | fl_calf | 0.187 | 2.07 |
| 3 | fr_hip | 0.00 | 1.50 |
| 4 | fr_thigh | -0.79 | 0.79 |
| 5 | fr_calf | -2.07 | -0.187 |
| 6 | rl_hip | 0.00 | 1.50 |
| 7 | rl_thigh | -0.79 | 0.79 |
| 8 | rl_calf | 0.187 | 2.07 |
| 9 | rr_hip | -1.50 | 0.00 |
| 10 | rr_thigh | -0.79 | 0.79 |
| 11 | rr_calf | -2.07 | -0.187 |

Mapping to DogDriver joint order (fl→LF, fr→RF, rl→LR, rr→RR; hip→HipA, thigh→HipF, calf→Knee):

| DogDriver Index | Joint | Vendor Index | Min | Max |
|---|---|---|---|---|
| 0 | LF_HipA | 0 (fl_hip) | -1.50 | 0.00 |
| 1 | LR_HipA | 6 (rl_hip) | 0.00 | 1.50 |
| 2 | RF_HipA | 3 (fr_hip) | 0.00 | 1.50 |
| 3 | RR_HipA | 9 (rr_hip) | -1.50 | 0.00 |
| 4 | LF_HipF | 1 (fl_thigh) | -0.79 | 0.79 |
| 5 | LR_HipF | 7 (rl_thigh) | -0.79 | 0.79 |
| 6 | RF_HipF | 4 (fr_thigh) | -0.79 | 0.79 |
| 7 | RR_HipF | 10 (rr_thigh) | -0.79 | 0.79 |
| 8 | LF_Knee | 2 (fl_calf) | 0.187 | 2.07 |
| 9 | LR_Knee | 8 (rl_calf) | 0.187 | 2.07 |
| 10 | RF_Knee | 5 (fr_calf) | -2.07 | -0.187 |
| 11 | RR_Knee | 11 (rr_calf) | -2.07 | -0.187 |

---

### Task 1: Add position limits constexpr array

**Files:**
- Modify: `src/rl_sar/src/ares_driver_node.cpp:31-35` (after `kJointNames`)

- [ ] **Step 1: Add the `kPositionLimits` constexpr array**

After the `kJointNames` array (line 35), insert:

```cpp
// Joint-space position limits (rad), derived from vendor MOTOR_LIMITS
// mapped to DogDriver joint order.
// DogDriver joint order: LF_HipA(0) LR_HipA(1) RF_HipA(2) RR_HipA(3)
//                         LF_HipF(4) LR_HipF(5) RF_HipF(6) RR_HipF(7)
//                         LF_Knee(8) LR_Knee(9) RF_Knee(10) RR_Knee(11)
static constexpr std::array<std::pair<float,float>, DogDriver::NUM_JOINTS> kPositionLimits = {{
    {-1.50f,  0.00f},   // 0  LF_HipA
    { 0.00f,  1.50f},   // 1  LR_HipA
    { 0.00f,  1.50f},   // 2  RF_HipA
    {-1.50f,  0.00f},   // 3  RR_HipA
    {-0.79f,  0.79f},   // 4  LF_HipF
    {-0.79f,  0.79f},   // 5  LR_HipF
    {-0.79f,  0.79f},   // 6  RF_HipF
    {-0.79f,  0.79f},   // 7  RR_HipF
    { 0.187f, 2.07f},   // 8  LF_Knee
    { 0.187f, 2.07f},   // 9  LR_Knee
    {-2.07f, -0.187f},  // 10 RF_Knee
    {-2.07f, -0.187f},  // 11 RR_Knee
}};
```

Also add the `<utility>` include at the top (for `std::pair`):

```cpp
#include <utility>
```

- [ ] **Step 2: Build to verify compilation**

Run: `cd /home/wufy/projects/my_ares_himloco/rl_sar-ARES && colcon build --packages-select rl_sar 2>&1 | tail -5`
Expected: Build succeeds (the constexpr is unused so far but must compile).

---

### Task 2: Replace control loop with wall_timer + worker_thread

**Files:**
- Modify: `src/rl_sar/src/ares_driver_node.cpp` — full class rewrite

- [ ] **Step 1: Rewrite the node class**

Replace the entire `ares_driver_node.cpp` with the new implementation. The new file keeps the same includes (plus `<utility>`), same `kJointNames`, same `kPositionLimits`, and rewrites the `AresDriverNode` class.

Full replacement file content:

```cpp
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
 * Threading:
 *   wall_timer_  (10ms, 100Hz) — publish motor feedback, IMU, gamepad
 *   worker_thread_ (5ms, 200Hz) — send clamped position commands to motors
 *   Worker thread gates on first command to avoid sending zeros.
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

static constexpr const char* kJointNames[DogDriver::NUM_JOINTS] = {
    "lf_hipa", "lr_hipa", "rf_hipa", "rr_hipa",
    "lf_hipf", "lr_hipf", "rf_hipf", "rr_hipf",
    "lf_knee", "lr_knee", "rf_knee", "rr_knee"
};

// Joint-space position limits (rad), derived from vendor MOTOR_LIMITS
// mapped to DogDriver joint order.
// DogDriver joint order: LF_HipA(0) LR_HipA(1) RF_HipA(2) RR_HipA(3)
//                         LF_HipF(4) LR_HipF(5) RF_HipF(6) RR_HipF(7)
//                         LF_Knee(8) LR_Knee(9) RF_Knee(10) RR_Knee(11)
static constexpr std::array<std::pair<float,float>, DogDriver::NUM_JOINTS> kPositionLimits = {{
    {-1.50f,  0.00f},   // 0  LF_HipA
    { 0.00f,  1.50f},   // 1  LR_HipA
    { 0.00f,  1.50f},   // 2  RF_HipA
    {-1.50f,  0.00f},   // 3  RR_HipA
    {-0.79f,  0.79f},   // 4  LF_HipF
    {-0.79f,  0.79f},   // 5  LR_HipF
    {-0.79f,  0.79f},   // 6  RF_HipF
    {-0.79f,  0.79f},   // 7  RR_HipF
    { 0.187f, 2.07f},   // 8  LF_Knee
    { 0.187f, 2.07f},   // 9  LR_Knee
    {-2.07f, -0.187f},  // 10 RF_Knee
    {-2.07f, -0.187f},  // 11 RR_Knee
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
        if (driver_)
            driver_->DisableAll();
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
            twist.linear.x  = gamepad_->GetAxis(1) * 0.5f;
            twist.linear.y  = 0.0f;
            twist.angular.z = gamepad_->GetAxis(3) * 0.5f;
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

            driver_->SetAllJointPositions(clamped_target);

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
```

Key changes from the original:
- `control_thread_` → `feedback_timer_` (wall_timer, 10ms) + `worker_thread_` (std::thread, 5ms)
- `has_new_command_` → `received_first_command_` (atomic<bool>, one-way gate)
- FeedbackTimerCallback: publishes motor feedback + IMU + gamepad (no motor commands)
- CommandLoop: first-command gate → clamp limits → send positions → diagnostics
- `kPositionLimits` array with clamping via `std::clamp`
- Per-second diagnostic log + overrun warning
- Clock catchup on major drift (`next_tick` reset if >1 period behind)

- [ ] **Step 2: Build to verify compilation**

Run: `cd /home/wufy/projects/my_ares_himloco/rl_sar-ARES && colcon build --packages-select rl_sar 2>&1 | tail -10`
Expected: Build succeeds with no errors.

---

### Task 3: Commit the refactor

- [ ] **Step 1: Stage and commit**

Run:
```bash
cd /home/wufy/projects/my_ares_himloco/rl_sar-ARES
git add src/rl_sar/src/ares_driver_node.cpp
git commit -m "refactor(driver): split control loop into wall_timer + worker_thread

Align threading model with vendor robstride_ros_sample:
- wall_timer (100Hz): publish motor feedback, IMU, gamepad
- worker_thread (200Hz): send position commands with first-command gate
- Add constexpr joint position limits with std::clamp before send
- Add per-second diagnostics (avg_hz, max_send_ms) and overrun warnings

No DogDriver API changes. No custom messages. Single file change."
```

---

### Task 4: Verify on hardware (manual, no automated test)

> **Note:** This project has no unit test framework for the driver node. Verification is done by running on the robot.

- [ ] **Step 1: Deploy and run**

Run:
```bash
cd /home/wufy/projects/my_ares_himloco/rl_sar-ARES
colcon build --packages-select rl_sar
ros2 run rl_sar ares_driver_node
```

Expected observations:
1. Node prints "Waiting for first command..." periodically
2. No motor commands sent until RL node publishes to `/motor_command`
3. After first command: prints "First command received! Starting motor control."
4. Per-second log: `Motor send diag: loops=~200 avg_hz=~200.0 max_send_ms=<X>`
5. `/motor_feedback` publishes at ~100Hz, `/imu/data` at ~100Hz
6. Motor movement matches expected behavior — no jumps to zero

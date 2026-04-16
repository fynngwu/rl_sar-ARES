# ARES Driver Node Threading Refactor

**Date:** 2026-04-16
**Status:** Approved

## Goal

Refactor `ares_driver_node.cpp` to align its threading model and publishing patterns with the vendor-provided `robstride_ros_sample` reference implementation, while keeping DogDriver's CAN protocol and coordinate system unchanged.

## Constraints

- **No library changes.** DogDriver API remains untouched.
- **Single file change.** Only `ares_driver_node.cpp` is modified.
- **Standard ROS2 messages.** No custom message types.
- **DogDriver coordinate system preserved.** Joint-space positions, existing offsets/directions/gear ratios.

## Design

### Threading Model

Replace the single 200Hz `ControlLoop` thread with:

| Component | Mechanism | Rate | Responsibility |
|-----------|-----------|------|----------------|
| Feedback + IMU + Gamepad | `wall_timer_` | 10ms (100Hz) | Read motor state, publish `/motor_feedback`, `/imu/data`, `/xbox_vel` |
| Command send | `worker_thread_` | 5ms (200Hz) | Block until first command, then send clamped positions |

**First-command gate:** The worker thread spins in a 1ms sleep loop until `received_first_command_` becomes true. This prevents sending zero positions before the RL node publishes its first command. Matches vendor pattern (`robstride_ros_sample:296-305`).

### Motor Position Limits

Hardcoded `constexpr` array of min/max per joint, applied in the command loop before sending to DogDriver.

```cpp
// DogDriver joint order: LF_HipA(0) LR_HipA(1) RF_HipA(2) RR_HipA(3)
//                         LF_HipF(4) LR_HipF(5) RF_HipF(6) RR_HipF(7)
//                         LF_Knee(8) LR_Knee(9) RF_Knee(10) RR_Knee(11)
// Values in joint-space (rad), relative to zero offset
static constexpr std::array<std::pair<float,float>, 12> kPositionLimits = {
    // To be derived from vendor MOTOR_LIMITS converted to DogDriver coordinate system
};
```

Clamping: `pos = std::clamp(pos, kPositionLimits[i].first, kPositionLimits[i].second)`.

### Published Topics

| Topic | Message | Rate | Fields |
|-------|---------|------|--------|
| `/motor_feedback` | `sensor_msgs/JointState` | 100Hz | `name[12]`, `position[12]`, `velocity[12]` |
| `/imu/data` | `sensor_msgs/Imu` | 100Hz | Same as current |
| `/xbox_vel` | `geometry_msgs/Twist` | 100Hz | Same as current |

### Subscribed Topics

| Topic | Message | Fields used |
|-------|---------|-------------|
| `/motor_command` | `sensor_msgs/JointState` | `position[12]` only |

### Diagnostics

Per-second diagnostic log matching vendor pattern:

```
Motor send diag: loops=<N> avg_hz=<F> max_send_ms=<M>
```

Plus `RCLCPP_WARN_THROTTLE` if a single command iteration exceeds 5ms.

### Node Structure

```cpp
class AresDriverNode : public rclcpp::Node {
    // Publishers
    Publisher<JointState> motor_feedback_pub_;   // 100Hz
    Publisher<Imu> imu_pub_;                     // 100Hz
    Publisher<Twist> xbox_vel_pub_;              // 100Hz

    // Subscriber
    Subscription<JointState> motor_command_sub_;

    // Timers / Threads
    TimerBase::SharedPtr feedback_timer_;        // 10ms wall_timer
    std::thread worker_thread_;                  // 5ms command loop

    // Motor command buffer
    std::mutex cmd_mutex_;
    std::array<float, 12> latest_target_{};
    bool received_first_command_ = false;

    // Position limits (constexpr)
    static constexpr auto kPositionLimits = {...};

    // Hardware
    std::unique_ptr<DogDriver> driver_;
    std::unique_ptr<Gamepad> gamepad_;

    // Thread control
    std::atomic<bool> running_;

    // Methods
    void FeedbackTimerCallback();     // wall_timer callback
    void CommandLoop();               // worker thread entry
    void MotorCommandCallback(msg);   // subscription callback
    void ApplyPositionLimits(...);    // clamp helper
};
```

### Limit Values Derivation

Vendor's `MOTOR_LIMITS` (robstride_ros_sample) are in their own coordinate system with:
- Direction flips on hip/thigh (`-position` for send, `-feedback` for read)
- Gear ratio on calf (`position * 28/15`)
- Different motor-to-joint mapping

These must be converted to DogDriver's joint-space convention where:
- DogDriver already handles direction + offset + gear ratio internally
- `SetJointPosition(i, pos)` takes joint-space rad (relative to zero offset)
- `GetJointStates().position[i]` returns joint-space rad

The conversion will be done during implementation by applying the inverse of DogDriver's internal transforms to the vendor's limit values.

## Files Changed

| File | Change Type |
|------|-------------|
| `src/rl_sar/src/ares_driver_node.cpp` | Rewrite threading model, add limits + diagnostics |

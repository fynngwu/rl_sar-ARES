# FL RL FR RR Joint Ordering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move joint reordering from rl_real_ares.cpp to ares_driver_node.cpp so /motor_feedback and /motor_command use FL RL FR RR order, eliminating the joint_mapping config.

**Architecture:** DogDriver internal order stays unchanged (HipA×4, HipF×4, Knee×4). ares_driver_node applies reordering at publish/subscribe boundaries. rl_real_ares reads and writes topics directly in URDF order without any mapping.

**Tech Stack:** C++17, ROS2 (rclcpp, sensor_msgs), CMake

---

### Task 1: Add mapping arrays and reorder feedback in ares_driver_node.cpp

**Files:**
- Modify: `src/rl_sar/src/ares_driver_node.cpp`

- [ ] **Step 1: Add mapping constants and reorder kJointNames/kPositionLimits**

Replace the existing `kJointNames` and `kPositionLimits` arrays at lines 34-58 with FL RL FR RR ordering and add the driver↔topic mapping tables.

Replace `kJointNames` (line 34-38) and `kPositionLimits` (line 45-58) with:

```cpp
static constexpr const char* kJointNames[DogDriver::NUM_JOINTS] = {
    "fl_hipa", "fl_hipf", "fl_knee",
    "rl_hipa", "rl_hipf", "rl_knee",
    "fr_hipa", "fr_hipf", "fr_knee",
    "rr_hipa", "rr_hipf", "rr_knee"
};

static constexpr std::array<std::pair<float,float>, DogDriver::NUM_JOINTS> kPositionLimits = {{
    {-1.50f,  0.00f},    // 0  FL_HipA
    {-0.79f,  0.79f},    // 1  FL_HipF
    { 0.187f, 2.07f},    // 2  FL_Knee
    { 0.00f,  1.50f},    // 3  RL_HipA
    {-0.79f,  0.79f},    // 4  RL_HipF
    { 0.187f, 2.07f},    // 5  RL_Knee
    { 0.00f,  1.50f},    // 6  FR_HipA
    {-0.79f,  0.79f},    // 7  FR_HipF
    {-2.07f, -0.187f},   // 8  FR_Knee
    {-1.50f,  0.00f},    // 9  RR_HipA
    {-0.79f,  0.79f},    // 10 RR_HipF
    {-2.07f, -0.187f},   // 11 RR_Knee
}};

// topic_index -> driver_index (FL RL FR RR -> driver order)
static constexpr int kTopicToDriver[DogDriver::NUM_JOINTS] = {
    0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11
};
// driver_index -> topic_index (driver order -> FL RL FR RR)
static constexpr int kDriverToTopic[DogDriver::NUM_JOINTS] = {
    0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11
};
```

- [ ] **Step 2: Reorder FeedbackTimerCallback to publish in FL RL FR RR order**

In `FeedbackTimerCallback` (line 136-148), after `auto joint_states = driver_->GetJointStates();`, reorder before publishing:

Replace:
```cpp
        sensor_msgs::msg::JointState feedback_msg;
        feedback_msg.header.stamp = this->now();
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
        {
            feedback_msg.name.push_back(kJointNames[i]);
            feedback_msg.position.push_back(joint_states.position[i]);
            feedback_msg.velocity.push_back(joint_states.velocity[i]);
        }
        motor_feedback_pub_->publish(feedback_msg);
```

With:
```cpp
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
```

- [ ] **Step 3: Reorder CommandLoop to convert FL RL FR RR to driver order before sending**

In `CommandLoop` (line 176-244), after reading `latest_target_` and applying limits, reorder to driver order before calling `SetAllJointPositions`:

Replace:
```cpp
            driver_->SetAllJointPositions(clamped_target);
```

With:
```cpp
            std::array<float, DogDriver::NUM_JOINTS> driver_target;
            for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
                driver_target[i] = clamped_target[kDriverToTopic[i]];
            driver_->SetAllJointPositions(driver_target);
```

- [ ] **Step 4: Build the driver library and ROS2 package to verify compilation**

Run:
```bash
cmake -S /home/wufy/projects/my_ares_himloco/rl_sar-ARES/driver -B /home/wufy/projects/my_ares_himloco/rl_sar-ARES/driver/build && cmake --build /home/wufy/projects/my_ares_himloco/rl_sar-ARES/driver/build
```

Note: The ROS2 package build requires `colcon build` which may need a sourced ROS2 environment. Skip if unavailable, but verify the C++ logic is correct by inspection.

- [ ] **Step 5: Commit**

```bash
git add src/rl_sar/src/ares_driver_node.cpp
git commit -m "refactor: reorder driver node I/O to FL RL FR RR joint ordering"
```

---

### Task 2: Remove joint_mapping from rl_real_ares.cpp

**Files:**
- Modify: `src/rl_sar/src/rl_real_ares.cpp`

- [ ] **Step 1: Remove joint_mapping config loading**

Delete lines 215-240 (the block that loads `joint_mapping` and computes `driver_to_urdf_`):
```cpp
            if (robot_config["joint_mapping"])
            {
                joint_mapping_.clear();
                for (const auto &m : robot_config["joint_mapping"])
                    joint_mapping_.push_back(m.as<int>());
            }
            else if (base_config["joint_mapping"])
            {
                joint_mapping_.clear();
                for (const auto &m : base_config["joint_mapping"])
                    joint_mapping_.push_back(m.as<int>());
            }

            if (!joint_mapping_.empty())
            {
                driver_to_urdf_.resize(joint_mapping_.size());
                for (size_t urdf_i = 0; urdf_i < joint_mapping_.size(); ++urdf_i)
                {
                    int driver_i = joint_mapping_[urdf_i];
                    driver_to_urdf_[driver_i] = static_cast<int>(urdf_i);
                }
                RCLCPP_INFO(this->get_logger(), "joint_mapping (urdf_i -> driver_i): %s",
                            FormatIntVector(joint_mapping_).c_str());
                RCLCPP_INFO(this->get_logger(), "driver_to_urdf (driver_i -> urdf_i): %s",
                            FormatIntVector(driver_to_urdf_).c_str());
            }
```

- [ ] **Step 2: Simplify MotorFeedbackCallback — direct copy, no mapping**

Replace the mapping logic in `MotorFeedbackCallback` (lines 554-591).

Replace:
```cpp
    void MotorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        size_t count = std::min(msg->position.size(), static_cast<size_t>(num_of_dofs_));

        if (!joint_mapping_.empty())
        {
            std::array<float, 12> raw_pos{};
            std::array<float, 12> raw_vel{};
            for (size_t i = 0; i < count; ++i)
            {
                raw_pos[i] = msg->position[i];
                raw_vel[i] = msg->velocity[i];
            }
            for (size_t urdf_i = 0; urdf_i < joint_mapping_.size(); ++urdf_i)
            {
                int driver_i = joint_mapping_[urdf_i];
                if (driver_i < static_cast<int>(count))
                {
                    joint_pos_[urdf_i] = raw_pos[driver_i];
                    joint_vel_[urdf_i] = raw_vel[driver_i];
                }
            }
        }
        else
        {
            for (size_t i = 0; i < count; ++i)
            {
                joint_pos_[i] = msg->position[i];
                joint_vel_[i] = msg->velocity[i];
            }
        }

        motor_feedback_received_ = true;
        if (motor_feedback_received_ && imu_received_)
            all_sensors_ready_ = true;
    }
```

With:
```cpp
    void MotorFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        size_t count = std::min(msg->position.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < count; ++i)
        {
            joint_pos_[i] = msg->position[i];
            joint_vel_[i] = msg->velocity[i];
        }

        motor_feedback_received_ = true;
        if (motor_feedback_received_ && imu_received_)
            all_sensors_ready_ = true;
    }
```

- [ ] **Step 3: Simplify RobotControl — direct publish, no mapping**

In `RobotControl` (lines 385-441), replace the driver-order name array and mapping logic.

Replace:
```cpp
        // Publish motor command
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = this->now();

        static const char* names[12] = {
            "lf_hipa", "lr_hipa", "rf_hipa", "rr_hipa",
            "lf_hipf", "lr_hipf", "rf_hipf", "rr_hipf",
            "lf_knee", "lr_knee", "rf_knee", "rr_knee"
        };

        // driver_to_urdf_[driver_i] = urdf_i
        // 遍历driver索引，用映射找到对应的urdf索引取值
        if (!driver_to_urdf_.empty())
        {
            for (int driver_i = 0; driver_i < num_of_dofs_; ++driver_i)
            {
                int urdf_i = driver_to_urdf_[driver_i];
                cmd_msg.name.push_back(names[driver_i]);  // names按driver顺序定义
                cmd_msg.position.push_back(output_dof_pos[urdf_i]);  // 用urdf索引取值
            }
        }
        else
        {
            for (int i = 0; i < num_of_dofs_; ++i)
            {
                cmd_msg.name.push_back(names[i]);
                cmd_msg.position.push_back(output_dof_pos[i]);
            }
        }

        motor_command_pub_->publish(cmd_msg);
```

With:
```cpp
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = this->now();
        for (int i = 0; i < num_of_dofs_; ++i)
        {
            cmd_msg.position.push_back(output_dof_pos[i]);
        }
        motor_command_pub_->publish(cmd_msg);
```

- [ ] **Step 4: Remove member variables and FormatIntVector**

Remove the member variable declarations (lines 797-798):
```cpp
    std::vector<int> joint_mapping_;   // driver_index = joint_mapping_[urdf_index]
    std::vector<int> driver_to_urdf_;  // urdf_index = driver_to_urdf_[driver_index]
```

Remove the `FormatIntVector` method (lines 738-750):
```cpp
    std::string FormatIntVector(const std::vector<int> &values) const
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
                oss << ", ";
            oss << values[i];
        }
        oss << "]";
        return oss.str();
    }
```

- [ ] **Step 5: Update file header comment**

Replace lines 12-15:
```cpp
 *   - ARES driver joint ordering: HipA(LF/LR/RF/RR), HipF(LF/LR/RF/RR), Knee(LF/LR/RF/RR)
 *   - Policy uses URDF ordering: FL(0-2), RL(3-5), FR(6-8), RR(9-11)
 *   - joint_mapping reorders between driver and policy at I/O boundaries
```

With:
```cpp
 *   - /motor_feedback and /motor_command use FL RL FR RR ordering (URDF order)
 *   - ares_driver_node handles reordering to/from driver internal order
```

- [ ] **Step 6: Commit**

```bash
git add src/rl_sar/src/rl_real_ares.cpp
git commit -m "refactor: remove joint_mapping from RL node, topic now in URDF order"
```

---

### Task 3: Remove joint_mapping from base.yaml

**Files:**
- Modify: `policy/ares_himloco/base.yaml`

- [ ] **Step 1: Remove joint_mapping line and update joint_names**

Replace the end of `base.yaml` (lines 25-31):

Replace:
```yaml
  joint_names: ["lf_hipa", "lr_hipa", "rf_hipa", "rr_hipa",
                "lf_hipf", "lr_hipf", "rf_hipf", "rr_hipf",
                "lf_knee", "lr_knee", "rf_knee", "rr_knee"]
  # Policy index (URDF order) -> driver index mapping
  # URDF order: FL(0-2), RL(3-5), FR(6-8), RR(9-11)
  # Driver order: lf_hipa(0), lr_hipa(1), rf_hipa(2), rr_hipa(3), lf_hipf(4), ...
  joint_mapping: [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
```

With:
```yaml
  joint_names: ["fl_hipa", "fl_hipf", "fl_knee",
                "rl_hipa", "rl_hipf", "rl_knee",
                "fr_hipa", "fr_hipf", "fr_knee",
                "rr_hipa", "rr_hipf", "rr_knee"]
```

- [ ] **Step 2: Commit**

```bash
git add policy/ares_himloco/base.yaml
git commit -m "refactor: remove joint_mapping, update joint_names to FL RL FR RR order"
```

---

### Task 4: Build verification

- [ ] **Step 1: Build driver library**

```bash
cmake -S driver -B driver/build && cmake --build driver/build
```

- [ ] **Step 2: Build ROS2 package**

```bash
source /opt/ros/*/setup.bash 2>/dev/null || true
colcon build --packages-select rl_sar
```

If colcon is not available, verify by reading the final files to ensure consistency.

- [ ] **Step 3: Verify mapping consistency**

Run a quick sanity check that the two mapping arrays are consistent inverses:

For each i in 0..11: `kTopicToDriver[kDriverToTopic[i]] == i` and `kDriverToTopic[kTopicToDriver[i]] == i`

- [ ] **Step 4: Final commit (if any fixes needed)**

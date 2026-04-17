# YAML Config Consolidation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Merge ares_himloco YAML configs into single file, make driver node read kp/kd/gamepad_scale from YAML.

**Architecture:** Two nodes (`ares`, `ares_driver_node`) share one YAML file per robot variant. `ares` reads RL parameters (observations, scales, etc.). `ares_driver_node` reads driver parameters (kp, kd, torque_limits, gamepad_scale). Both resolve config via `robot_name` ROS2 param.

**Tech Stack:** C++17, yaml-cpp, CMake, ROS2 Humble

---

### Task 1: Merge YAML configs and delete base.yaml

**Files:**
- Modify: `policy/ares_himloco/config.yaml`
- Delete: `policy/ares_himloco/base.yaml`

- [ ] **Step 1: Pull latest remote changes first**

```bash
git stash
git pull origin master
git stash pop  # may need manual resolve
```

- [ ] **Step 2: Rewrite `policy/ares_himloco/config.yaml`**

Replace entire file with merged content. Section key remains `ares_himloco/himloco` to avoid breaking existing `config_key` resolution in `rl_real_ares.cpp`:

```yaml
ares_himloco/himloco:
  model_name: policy.onnx
  num_observations: 45
  observations: [commands, ang_vel, gravity_vec, dof_pos, dof_vel, actions]
  observations_history: [0, 1, 2, 3, 4, 5]
  observations_history_priority: time
  clip_obs: 100.0
  clip_actions_lower: [-10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10]
  clip_actions_upper: [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]
  dt: 0.005
  decimation: 4
  num_of_dofs: 12
  action_scale: [0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25]
  commands_scale: [2.0, 2.0, 0.25]
  lin_vel_scale: 2.0
  ang_vel_scale: 0.25
  dof_pos_scale: 1.0
  dof_vel_scale: 0.05
  fixed_kp: [30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30]
  fixed_kd: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  torque_limits: [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]
  default_dof_pos: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  gamepad_scale: 0.5
```

- [ ] **Step 3: Delete `policy/ares_himloco/base.yaml`**

```bash
rm policy/ares_himloco/base.yaml
```

- [ ] **Step 4: Commit**

```bash
git add policy/ares_himloco/config.yaml policy/ares_himloco/base.yaml
git commit -m "refactor: merge ares_himloco base.yaml into single config.yaml"
```

---

### Task 2: Simplify rl_real_ares.cpp — remove base.yaml loading

**Files:**
- Modify: `src/rl_sar/src/rl_real_ares.cpp:114-239`

- [ ] **Step 1: Remove base_config loading and GET_PARAM macro**

In `InitRL()`, remove lines 139-147 (base_config declaration, base_config_path, GET_PARAM macro). Change all `GET_PARAM(key, var)` calls (lines 149-157) to direct reads from `robot_config`:

Replace:
```cpp
            YAML::Node base_config;

            std::string base_config_path = policy_dir + "/" + robot_name_ + "/base.yaml";
            if (std::ifstream(base_config_path))
                base_config = YAML::LoadFile(base_config_path)[robot_name_];

            #define GET_PARAM(key, var) \
                if (robot_config[#key]) var = robot_config[#key].as<decltype(var)>(); \
                else if (base_config[#key]) var = base_config[#key].as<decltype(var)>();

            GET_PARAM(dt, dt_)
            GET_PARAM(decimation, decimation_)
            GET_PARAM(num_of_dofs, num_of_dofs_)
            GET_PARAM(lin_vel_scale, lin_vel_scale_)
            GET_PARAM(ang_vel_scale, ang_vel_scale_)
            GET_PARAM(dof_pos_scale, dof_pos_scale_)
            GET_PARAM(dof_vel_scale, dof_vel_scale_)
            GET_PARAM(gravity_vec_scale, gravity_vec_scale_)
            GET_PARAM(clip_obs, clip_obs_)
```

With:
```cpp
            #define GET_PARAM(key, var) \
                if (robot_config[#key]) var = robot_config[#key].as<decltype(var)>();

            GET_PARAM(dt, dt_)
            GET_PARAM(decimation, decimation_)
            GET_PARAM(num_of_dofs, num_of_dofs_)
            GET_PARAM(lin_vel_scale, lin_vel_scale_)
            GET_PARAM(ang_vel_scale, ang_vel_scale_)
            GET_PARAM(dof_pos_scale, dof_pos_scale_)
            GET_PARAM(dof_vel_scale, dof_vel_scale_)
            GET_PARAM(gravity_vec_scale, gravity_vec_scale_)
            GET_PARAM(clip_obs, clip_obs_)
```

- [ ] **Step 2: Remove base_config fallback from default_dof_pos**

Replace (lines 201-212):
```cpp
            if (robot_config["default_dof_pos"])
            {
                default_dof_pos_.clear();
                for (const auto &p : robot_config["default_dof_pos"])
                    default_dof_pos_.push_back(p.as<float>());
            }
            else if (base_config["default_dof_pos"])
            {
                default_dof_pos_.clear();
                for (const auto &p : base_config["default_dof_pos"])
                    default_dof_pos_.push_back(p.as<float>());
            }
```

With:
```cpp
            if (robot_config["default_dof_pos"])
            {
                default_dof_pos_.clear();
                for (const auto &p : robot_config["default_dof_pos"])
                    default_dof_pos_.push_back(p.as<float>());
            }
```

- [ ] **Step 3: Remove base_config fallback from fixed_kp and fixed_kd**

Replace (lines 214-239):
```cpp
            // PD gains are set at driver init (kp=40, kd=0.5).
            // We still load them for logging/compatibility, but do NOT send per-command.
            if (robot_config["fixed_kp"])
            {
                kp_.clear();
                for (const auto &s : robot_config["fixed_kp"])
                    kp_.push_back(s.as<float>());
            }
            else if (base_config["fixed_kp"])
            {
                kp_.clear();
                for (const auto &s : base_config["fixed_kp"])
                    kp_.push_back(s.as<float>());
            }
            if (robot_config["fixed_kd"])
            {
                kd_.clear();
                for (const auto &s : robot_config["fixed_kd"])
                    kd_.push_back(s.as<float>());
            }
            else if (base_config["fixed_kd"])
            {
                kd_.clear();
                for (const auto &s : base_config["fixed_kd"])
                    kd_.push_back(s.as<float>());
            }
```

With:
```cpp
            if (robot_config["fixed_kp"])
            {
                kp_.clear();
                for (const auto &s : robot_config["fixed_kp"])
                    kp_.push_back(s.as<float>());
            }
            if (robot_config["fixed_kd"])
            {
                kd_.clear();
                for (const auto &s : robot_config["fixed_kd"])
                    kd_.push_back(s.as<float>());
            }
```

- [ ] **Step 4: Build and verify**

```bash
source /opt/ros/humble/setup.bash
cmake --build src/rl_sar/build
```

Expected: Compiles without errors.

- [ ] **Step 5: Commit**

```bash
git add src/rl_sar/src/rl_real_ares.cpp
git commit -m "refactor: remove base.yaml loading from rl_real_ares, use single config"
```

---

### Task 3: Add YAML loading to ares_driver_node

**Files:**
- Modify: `src/rl_sar/src/ares_driver_node.cpp`
- Modify: `src/rl_sar/CMakeLists.txt`

- [ ] **Step 1: Add yaml-cpp include to ares_driver_node.cpp**

After `#include <utility>` (line 30), add:
```cpp
#include "yaml-cpp/yaml.h"
```

- [ ] **Step 2: Add ROS2 `config_path` param and YAML loading to constructor**

Add `config_path_` and `gamepad_scale_` member variables. In the constructor, before `DogDriver` init, load the YAML and read parameters.

Add these members to the class (private section, after `running_` and `received_first_command_`):

```cpp
std::string config_path_;
float gamepad_scale_;
```

In the constructor body, after `RCLCPP_INFO(this->get_logger(), "Initializing ARES Driver Node...");` and before `DogDriver` init, add:

```cpp
        this->declare_parameter("config_path", std::string(POLICY_DIR) + "/ares_himloco/config.yaml");
        this->get_parameter("config_path", config_path_);
        gamepad_scale_ = 0.5f;

        if (std::ifstream(config_path_))
        {
            try
            {
                YAML::Node config = YAML::LoadFile(config_path_);
                std::string config_key = "ares_himloco";
                if (config["ares_himloco/himloco"])
                    config_key = "ares_himloco/himloco";
                YAML::Node rc = config[config_key];

                std::vector<float> kp_vals, kd_vals, torque_vals;
                if (rc["fixed_kp"])
                    for (const auto &v : rc["fixed_kp"]) kp_vals.push_back(v.as<float>());
                if (rc["fixed_kd"])
                    for (const auto &v : rc["fixed_kd"]) kd_vals.push_back(v.as<float>());
                if (rc["torque_limits"])
                    for (const auto &v : rc["torque_limits"]) torque_vals.push_back(v.as<float>());
                if (rc["gamepad_scale"])
                    gamepad_scale_ = rc["gamepad_scale"].as<float>();

                RCLCPP_INFO(this->get_logger(), "Config loaded: %s (key=%s)", config_path_.c_str(), config_key.c_str());
                RCLCPP_INFO(this->get_logger(), "  gamepad_scale=%.2f", gamepad_scale_);
                RCLCPP_INFO(this->get_logger(), "  kp=%d kd=%d torque=%d",
                            (int)kp_vals.size(), (int)kd_vals.size(), (int)torque_vals.size());
            }
            catch (const YAML::Exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to parse config: %s", e.what());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Config not found: %s, using defaults", config_path_.c_str());
        }
```

- [ ] **Step 3: Apply kp/kd after DogDriver init**

After the DogDriver construction (after line 84) and before the torque limit loop (line 89), add kp/kd application. Replace the existing torque-only loop (lines 89-92) with combined kp/kd/torque:

Replace:
```cpp
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
            driver_->SetTorqueLimit(i, TORQUE_LIMIT);
        }
        RCLCPP_INFO(this->get_logger(), "Torque limit set to %.1f Nm for all joints", TORQUE_LIMIT);
```

With (reading kp/kd/torque from the parsed values — they need to be accessible in this scope). The parsed values are local to the try block above, so store them as member vectors:

Add members:
```cpp
std::vector<float> config_kp_;
std::vector<float> config_kd_;
std::vector<float> config_torque_;
```

Update the YAML loading block to assign to these members instead of local vars:
```cpp
                if (rc["fixed_kp"])
                    for (const auto &v : rc["fixed_kp"]) config_kp_.push_back(v.as<float>());
                if (rc["fixed_kd"])
                    for (const auto &v : rc["fixed_kd"]) config_kd_.push_back(v.as<float>());
                if (rc["torque_limits"])
                    for (const auto &v : rc["torque_limits"]) config_torque_.push_back(v.as<float>());
```

Then replace the torque loop with:
```cpp
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
            float kp = (i < (int)config_kp_.size()) ? config_kp_[i] : DogDriver::DEFAULT_KP;
            float kd = (i < (int)config_kd_.size()) ? config_kd_[i] : DogDriver::DEFAULT_KD;
            float torque = (i < (int)config_torque_.size()) ? config_torque_[i] : TORQUE_LIMIT;
            driver_->SetMITParams(i, kp, kd);
            driver_->SetTorqueLimit(i, torque);
        }
        RCLCPP_INFO(this->get_logger(), "MIT params set for all joints");
```

Note: keep `TORQUE_LIMIT` as fallback constant (it's already defined at line 57).

- [ ] **Step 4: Use gamepad_scale_ instead of hardcoded 0.5f**

Replace (lines 185-187):
```cpp
            twist.linear.x  = gamepad_->GetAxis(1) * 0.5f;
            twist.linear.y  = 0.0f;
            twist.angular.z = gamepad_->GetAxis(3) * 0.5f;
```

With:
```cpp
            twist.linear.x  = gamepad_->GetAxis(1) * gamepad_scale_;
            twist.linear.y  = 0.0f;
            twist.angular.z = gamepad_->GetAxis(3) * gamepad_scale_;
```

- [ ] **Step 5: Add yaml-cpp to CMakeLists.txt for ares_driver_node**

In `src/rl_sar/CMakeLists.txt`, find the `ares_driver_node` target_link_libraries block (around line 267-275) and add `${YAML_CPP_LIBRARIES}`:

Replace:
```cpp
    if(DOG_DRIVER_LIB)
        add_executable(ares_driver_node src/ares_driver_node.cpp)
        target_link_libraries(ares_driver_node
            dog_driver
            Threads::Threads
            rclcpp::rclcpp
            sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
            geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
        )
    endif()
```

With:
```cpp
    if(DOG_DRIVER_LIB)
        add_executable(ares_driver_node src/ares_driver_node.cpp)
        target_link_libraries(ares_driver_node
            dog_driver
            ${YAML_CPP_LIBRARIES}
            Threads::Threads
            rclcpp::rclcpp
            sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
            geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
        )
    endif()
```

- [ ] **Step 6: Build and verify**

```bash
source /opt/ros/humble/setup.bash
cmake --build src/rl_sar/build
```

Expected: Compiles without errors. `ares_driver_node` binary is produced.

- [ ] **Step 7: Commit**

```bash
git add src/rl_sar/src/ares_driver_node.cpp src/rl_sar/CMakeLists.txt
git commit -m "feat: ares_driver_node reads kp/kd/torque/gamepad_scale from YAML"
```

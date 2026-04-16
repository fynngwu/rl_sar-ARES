# ARES HIMloco Variant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add HIMloco policy support for ARES quadruped — fix config, export ONNX, create MuJoCo sim test script, and create a new `ares_himloco` C++ executable.

**Architecture:** Three independent deliverables (Python scripts + new C++ executable) with config fixes. All new files except a minor CMakeLists.txt addition. No existing source files modified.

**Tech Stack:** Python 3 (torch, mujoco, onnxruntime), ROS2 Humble (C++17), ONNX Runtime

---

## Task 1: Fix Config Files

**Files:**
- Modify: `policy/ares_himloco/base.yaml:33`
- Modify: `policy/ares_himloco/himloco/config.yaml:45`

- [ ] **Step 1: Fix joint_mapping in base.yaml**

Change line 33 in `policy/ares_himloco/base.yaml` from:
```yaml
  joint_mapping: [0, 4, 8, 2, 6, 10, 1, 5, 9, 3, 7, 11]
```
to:
```yaml
  joint_mapping: [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
```

The correct mapping is FL=driver[0,4,8], RL=driver[1,5,9], FR=driver[2,6,10], RR=driver[3,7,11], where `policy[i] = driver[mapping[i]]`.

- [ ] **Step 2: Fix commands_scale in config.yaml**

Change line 45 in `policy/ares_himloco/himloco/config.yaml` from:
```yaml
  commands_scale: [1.2, 0.5, 0.5]
```
to:
```yaml
  commands_scale: [3.5, 2.0, 1.0]
```

This matches the training config (htdw_4438).

- [ ] **Step 3: Commit**

```bash
git add policy/ares_himloco/base.yaml policy/ares_himloco/himloco/config.yaml
git commit -m "fix: correct HIMloco config joint_mapping and commands_scale"
```

---

## Task 2: ONNX Export Script

**Files:**
- Create: `scripts/export_onnx.py`

- [ ] **Step 1: Create the ONNX export script**

```python
#!/usr/bin/env python3
"""Export HIMloco policy.pt to policy.onnx for deployment."""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser(description="Export policy.pt to ONNX")
    parser.add_argument("--input", type=str, default=os.path.join(SCRIPT_DIR, "..", "policy.pt"),
                        help="Path to policy.pt (default: ../policy.pt)")
    parser.add_argument("--output", type=str, default=os.path.join(SCRIPT_DIR, "..", "policy", "ares_himloco", "himloco", "policy.onnx"),
                        help="Path to output policy.onnx")
    args = parser.parse_args()

    import torch

    input_path = os.path.abspath(args.input)
    output_path = os.path.abspath(args.output)

    if not os.path.exists(input_path):
        print(f"Error: {input_path} not found")
        sys.exit(1)

    print(f"Loading model: {input_path}")
    model = torch.jit.load(input_path, map_location="cpu")
    model.eval()

    dummy_input = torch.randn(1, 270, dtype=torch.float32)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    print(f"Exporting to: {output_path}")

    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        opset_version=17,
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
    )

    # Verify
    import onnx
    onnx_model = onnx.load(output_path)
    inp = onnx_model.graph.input[0]
    out = onnx_model.graph.output[0]
    print(f"Input:  name={inp.name}, dims={[d.dim_value for d in inp.type.tensor_type.shape.dim]}")
    print(f"Output: name={out.name}, dims={[d.dim_value for d in out.type.tensor_type.shape.dim]}")

    # Test inference
    import onnxruntime as ort
    session = ort.InferenceSession(output_path)
    result = session.run(None, {"obs": dummy_input.numpy()})
    print(f"Test output shape: {result[0].shape}, values: {result[0][0, :4]}")
    print("Export complete.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run export**

```bash
conda run -n env_isaaclab python /home/wufy/projects/UIKA_HIMloco/rl_sar-ARES/scripts/export_onnx.py \
    --input /home/wufy/projects/UIKA_HIMloco/policy.pt \
    --output /home/wufy/projects/UIKA_HIMloco/rl_sar-ARES/policy/ares_himloco/himloco/policy.onnx
```

Expected: Input name=obs, dims=[1, 270], Output name=actions, dims=[1, 12]

- [ ] **Step 3: Commit**

```bash
git add scripts/export_onnx.py policy/ares_himloco/himloco/policy.onnx
git commit -m "feat: add ONNX export script and exported HIMloco policy"
```

---

## Task 3: MuJoCo Sim Test Script

**Files:**
- Create: `scripts/test_himloco_mujoco.py`

- [ ] **Step 1: Create the MuJoCo sim test script**

This script is adapted from the reference deploy script at `/home/wufy/projects/UIKA_HIMloco/deploy/deploy_mujoco/deploy_4438.py`, adjusted for ARES dogv2 model and direct paths.

```python
#!/usr/bin/env python3
"""Test HIMloco policy on ARES dogv2 in MuJoCo simulation.

Usage:
    conda run -n gym python scripts/test_himloco_mujoco.py --onnx policy/ares_himloco/himloco/policy.onnx
    conda run -n gym python scripts/test_himloco_mujoco.py --policy policy/ares_himloco/himloco/policy.onnx --mode torch
"""

import argparse
import os
import sys
import time
from collections import deque

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

# Default paths
DEFAULT_ONNX = os.path.join(PROJECT_ROOT, "policy", "ares_himloco", "himloco", "policy.onnx")
DEFAULT_PT = os.path.join(os.path.dirname(PROJECT_ROOT), "policy.pt")
DEFAULT_XML = os.path.join(os.path.dirname(PROJECT_ROOT), "dogv2", "mjcf", "dogv2_view.xml")


def parse_args():
    parser = argparse.ArgumentParser(description="Test HIMloco policy in MuJoCo (dogv2)")
    parser.add_argument("--policy", type=str, default=None,
                        help="Path to .pt or .onnx policy file")
    parser.add_argument("--xml", type=str, default=DEFAULT_XML,
                        help="Path to MJCF XML model")
    parser.add_argument("--mode", type=str, choices=["onnx", "torch"], default="onnx",
                        help="Inference backend (default: onnx)")
    return parser.parse_args()


def quat_rotate_inverse(q, v):
    """Rotate vector v by inverse of quaternion q (MuJoCo convention: w,x,y,z)."""
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w**2 - 1.0)
    b = __import__('numpy').cross(q_vec, v) * q_w * 2.0
    c = q_vec * __import__('numpy').dot(q_vec, v) * 2.0
    return a - b + c


def main():
    args = parse_args()

    import numpy as np

    # --- Determine policy path and mode ---
    if args.policy:
        policy_path = os.path.abspath(args.policy)
    else:
        policy_path = DEFAULT_ONNX

    if not os.path.exists(policy_path):
        print(f"Error: policy not found at {policy_path}")
        sys.exit(1)

    if policy_path.endswith(".pt"):
        mode = "torch"
    elif policy_path.endswith(".onnx"):
        mode = "onnx"
    else:
        mode = args.mode

    print(f"Policy: {policy_path} (mode={mode})")
    print(f"XML:    {args.xml}")

    # --- Load inference backend ---
    if mode == "torch":
        import torch
        model = torch.jit.load(policy_path, map_location="cpu")
        model.eval()

        def infer(obs_history):
            with torch.no_grad():
                tensor_in = torch.from_numpy(obs_history.astype(np.float32))
                tensor_out = model(tensor_in)
                return tensor_out.numpy()
    else:
        import onnxruntime as ort
        session = ort.InferenceSession(policy_path)
        input_name = session.get_inputs()[0].name

        def infer(obs_history):
            result = session.run(None, {input_name: obs_history.astype(np.float32)})
            return result[0]

    # --- Load MuJoCo model ---
    import mujoco

    if not os.path.exists(args.xml):
        print(f"Error: MJCF not found at {args.xml}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)

    # dogv2_view.xml has 0 actuators — we apply torques directly via data.ctrl
    # First, add 12 general actuators to the model
    # MuJoCo model has nu=0, so we need to set it up
    # We'll apply torques via mj_applyFT instead since adding actuators after load is complex
    # Actually, we can modify model.nu and use data.ctrl
    # Simplest: apply forces directly via data.qfrc_applied

    # --- Training parameters ---
    sim_dt = 0.005
    control_decimation = 2
    kp = 10.0
    kd = 0.3
    action_scale = 0.25
    cmd_scale = np.array([3.5, 2.0, 1.0], dtype=np.float32)
    ang_vel_scale = 0.25
    dof_pos_scale = 1.0
    dof_vel_scale = 0.05
    default_dof_pos = np.array([0, 0.9, -1.5] * 4, dtype=np.float32)
    num_obs = 45
    history_length = 6
    num_total_obs = num_obs * history_length  # 270

    # dogv2_view.xml: qpos = [7 floating base] + [12 joint dofs]
    # Joint order: LF_HipA, LF_HipF, LF_Knee, LH_HipA, LH_HipF, LH_Knee,
    #              RF_HipA, RF_HipF, RF_Knee, RH_HipA, RH_HipF, RH_Knee
    # LH = LR, RH = RR — same per-leg order as policy (FL, RL, FR, RR)
    # No remapping needed between MJCF and policy

    # --- Initialize robot state ---
    data.qpos[7:] = default_dof_pos
    data.qpos[2] = 0.15  # base height
    mujoco.mj_forward(model, data)

    target_dof_pos = default_dof_pos.copy()
    action = np.zeros(12, dtype=np.float32)

    # History buffer (newest first)
    obs_history_buffer = deque(
        [np.zeros(num_obs, dtype=np.float32) for _ in range(history_length)],
        maxlen=history_length,
    )

    # --- Keyboard control ---
    cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    paused = False

    from pynput import keyboard

    def on_press(key):
        nonlocal cmd
        try:
            if key == keyboard.Key.up:
                cmd[0] = 0.6
            elif key == keyboard.Key.down:
                cmd[0] = -0.4
            elif key == keyboard.Key.left:
                cmd[2] = 0.8
            elif key == keyboard.Key.right:
                cmd[2] = -0.8
        except AttributeError:
            pass

    def on_release(key):
        nonlocal cmd
        try:
            if key in (keyboard.Key.up, keyboard.Key.down):
                cmd[0] = 0.0
            elif key in (keyboard.Key.left, keyboard.Key.right):
                cmd[2] = 0.0
        except AttributeError:
            pass

    def key_callback(keycode):
        nonlocal paused
        if chr(keycode) == ' ':
            paused = not paused
            print(f"Paused: {paused}")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("Simulation started. Arrow keys: move, Space: pause.")

    # --- Simulation loop ---
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        step_counter = 0
        while viewer.is_running():
            step_start = time.time()

            if not paused:
                # Policy control at 100Hz (sim_dt=0.005, decimation=2)
                if step_counter % control_decimation == 0:
                    # Read state
                    qj = data.qpos[7:].astype(np.float32)   # 12 joint positions
                    dqj = data.qvel[6:].astype(np.float32)   # 12 joint velocities
                    quat = data.qpos[3:7].astype(np.float32) # [w, x, y, z]

                    # Angular velocity from sensor
                    try:
                        omega = data.sensor("imu_ang_vel").data.astype(np.float32)
                    except KeyError:
                        omega = data.qvel[3:6].astype(np.float32)

                    # Projected gravity
                    gravity_vec = np.array([0.0, 0.0, -1.0], dtype=np.float32)
                    proj_gravity = quat_rotate_inverse(quat, gravity_vec)

                    # Build observation (45 dims, commands-first)
                    obs_raw = np.concatenate([
                        cmd * cmd_scale,                          # 0-2: velocity command
                        omega * ang_vel_scale,                     # 3-5: angular velocity
                        proj_gravity,                              # 6-8: projected gravity
                        (qj - default_dof_pos) * dof_pos_scale,   # 9-20: joint positions
                        dqj * dof_vel_scale,                      # 21-32: joint velocities
                        action,                                   # 33-44: previous actions
                    ]).astype(np.float32)

                    # Update history (newest first)
                    obs_history_buffer.appendleft(obs_raw.copy())
                    obs_history = np.concatenate(list(obs_history_buffer)).reshape(1, -1)

                    # Inference
                    raw_action = infer(obs_history)[0].astype(np.float32)
                    raw_action = np.clip(raw_action, -10.0, 10.0)
                    action = raw_action
                    target_dof_pos = raw_action * action_scale + default_dof_pos

                # PD control (applied every step)
                qj = data.qpos[7:]
                dqj = data.qvel[6:]
                tau = kp * (target_dof_pos - qj) - kd * dqj

                # Apply torques via qfrc_applied (dogv2_view.xml has 0 actuators)
                # Map torques to generalised force indices for DOFs 7-18
                # In MuJoCo, qfrc_applied has same dim as nv (generalized velocities)
                # For a freejoint robot: nv = 6 (freejoint) + 12 (joint dofs) = 18
                # Indices 6:18 correspond to our 12 joints
                data.qfrc_applied[6:] = tau

                mujoco.mj_step(model, data)
                step_counter += 1

            viewer.sync()

            # Frame rate sync
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    listener.stop()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Test with torch mode**

```bash
conda run -n gym python /home/wufy/projects/UIKA_HIMloco/rl_sar-ARES/scripts/test_himloco_mujoco.py \
    --policy /home/wufy/projects/UIKA_HIMloco/policy.pt --mode torch
```

Expected: MuJoCo viewer launches, robot stands with default pose [0, 0.9, -1.5] per leg. Arrow keys control movement. No crash.

- [ ] **Step 3: Test with onnx mode**

```bash
conda run -n gym python /home/wufy/projects/UIKA_HIMloco/rl_sar-ARES/scripts/test_himloco_mujoco.py \
    --policy /home/wufy/projects/UIKA_HIMloco/rl_sar-ARES/policy/ares_himloco/himloco/policy.onnx
```

Note: `gym` env needs `pip install onnxruntime` if not already installed.

Expected: Same behavior as torch mode — robot stands, responds to arrow keys.

- [ ] **Step 4: Commit**

```bash
git add scripts/test_himloco_mujoco.py
git commit -m "feat: add MuJoCo sim test script for HIMloco policy"
```

---

## Task 4: C++ Executable — ares_himloco Node

**Files:**
- Create: `src/rl_sar/src/rl_real_ares_himloco.cpp`

This is a copy-and-adapt of `rl_real_ares.cpp` (739 lines) with the following changes:

1. **Class name**: `ARSNodeHimloco` (was `ARSNode`)
2. **Default decimation**: 2 (was 4) — 100Hz policy at 200Hz control
3. **Node name**: `"ares_himloco_rl_node"` (was `"ares_rl_node"`)
4. **Joint mapping**: loads `joint_mapping` from config, applies in `RunModel()` and `RobotControl()`
5. **No other structural changes** — same observation construction, same ROS2 interface, same pattern

- [ ] **Step 1: Create the C++ source file**

```cpp
/*
 * ARES HIMloco RL Control ROS2 Node
 *
 * Separate executable from ares node. Loads config from policy/ares_himloco/.
 * Key differences from rl_real_ares.cpp:
 *   - Joint mapping: driver->policy reorder for observations, policy->driver reorder for commands
 *   - Default decimation: 2 (100Hz policy at 200Hz control)
 *   - PD gains: kp=10, kd=0.3 (from config)
 *   - Commands-first observation ordering (from config)
 *   - 6-frame history (from config)
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "vector_math.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

class ARSNodeHimloco : public rclcpp::Node
{
public:
    ARSNodeHimloco()
        : Node("ares_himloco_rl_node"),
          robot_name_("ares_himloco"),
          num_of_dofs_(12),
          dt_(0.005f),
          decimation_(2),
          lin_vel_scale_(2.0f),
          ang_vel_scale_(0.25f),
          dof_pos_scale_(1.0f),
          dof_vel_scale_(0.05f),
          gravity_vec_scale_(1.0f),
          clip_obs_(100.0f),
          rl_init_done_(false),
          all_sensors_ready_(false),
          inference_count_(0),
          inference_time_ms_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ARES HIMloco RL Node...");

        this->declare_parameter("robot_name", robot_name_);
        this->get_parameter("robot_name", robot_name_);

        motor_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/motor_command", 10);

        motor_feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motor_feedback", 10,
            std::bind(&ARSNodeHimloco::MotorFeedbackCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ARSNodeHimloco::ImuCallback, this, std::placeholders::_1));

        xbox_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/xbox_vel", 10,
            std::bind(&ARSNodeHimloco::XboxVelCallback, this, std::placeholders::_1));

        if (!InitRL())
        {
            RCLCPP_ERROR(this->get_logger(), "RL initialization failed!");
            return;
        }

        loop_control_ = std::make_shared<LoopFunc>(
            "loop_control", dt_, std::bind(&ARSNodeHimloco::RobotControl, this));
        loop_rl_ = std::make_shared<LoopFunc>(
            "loop_rl", dt_ * decimation_, std::bind(&ARSNodeHimloco::RunModel, this));

        loop_control_->start();
        loop_rl_->start();

        RCLCPP_INFO(this->get_logger(), "ARES HIMloco RL Node started");
        RCLCPP_INFO(this->get_logger(), "  Control: %.1fHz, Inference: %.1fHz",
                    1.0 / dt_, 1.0 / (dt_ * decimation_));
        RCLCPP_INFO(this->get_logger(), "  Joint mapping: %s", FormatIntVector(joint_mapping_).c_str());
    }

    ~ARSNodeHimloco()
    {
        if (loop_control_)
            loop_control_->shutdown();
        if (loop_rl_)
            loop_rl_->shutdown();
        RCLCPP_INFO(this->get_logger(), "ARES HIMloco RL Node stopped");
    }

private:
    bool InitRL()
    {
        std::string policy_dir = std::string(POLICY_DIR);
        std::string config_path = policy_dir + "/" + robot_name_ + "/himloco/config.yaml";
        std::string model_subdir = "himloco/";
        std::string model_name_from_config;

        if (!std::ifstream(config_path))
        {
            config_path = policy_dir + "/" + robot_name_ + "/config.yaml";
            model_subdir.clear();
        }

        RCLCPP_INFO(this->get_logger(), "Loading config: %s", config_path.c_str());

        try
        {
            YAML::Node config = YAML::LoadFile(config_path);

            std::string config_key = robot_name_;
            if (config[robot_name_ + "/himloco"])
                config_key = robot_name_ + "/himloco";

            YAML::Node robot_config = config[config_key];
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

            if (robot_config["model_name"])
                model_name_from_config = robot_config["model_name"].as<std::string>();

            if (robot_config["observations"])
            {
                observations_.clear();
                for (const auto &obs : robot_config["observations"])
                    observations_.push_back(obs.as<std::string>());
            }

            if (robot_config["observations_history"])
            {
                observations_history_.clear();
                for (const auto &h : robot_config["observations_history"])
                    observations_history_.push_back(h.as<int>());
            }

            if (robot_config["clip_actions_upper"])
            {
                clip_actions_upper_.clear();
                for (const auto &c : robot_config["clip_actions_upper"])
                    clip_actions_upper_.push_back(c.as<float>());
            }
            if (robot_config["clip_actions_lower"])
            {
                clip_actions_lower_.clear();
                for (const auto &c : robot_config["clip_actions_lower"])
                    clip_actions_lower_.push_back(c.as<float>());
            }

            if (robot_config["action_scale"])
            {
                action_scale_.clear();
                for (const auto &s : robot_config["action_scale"])
                    action_scale_.push_back(s.as<float>());
            }
            if (robot_config["commands_scale"])
            {
                commands_scale_.clear();
                for (const auto &s : robot_config["commands_scale"])
                    commands_scale_.push_back(s.as<float>());
            }
            if (robot_config["default_dof_pos"])
            {
                default_dof_pos_.clear();
                for (const auto &p : robot_config["default_dof_pos"])
                    default_dof_pos_.push_back(p.as<float>());
            }

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

            // Load joint mapping from config
            // Check both config and base_config for joint_mapping
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
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            return false;
        }

        // Build inverse mapping (policy -> driver)
        // joint_mapping_[i] = driver index for policy index i
        // inverse_mapping_[driver_idx] = policy index
        inverse_mapping_.resize(num_of_dofs_, 0);
        for (int i = 0; i < num_of_dofs_; ++i)
        {
            if (i < static_cast<int>(joint_mapping_.size()))
                inverse_mapping_[joint_mapping_[i]] = i;
        }

        EnsureVectorSizes();

        std::string model_path;
        if (!model_name_from_config.empty())
        {
            model_path = policy_dir + "/" + robot_name_ + "/" + model_subdir + model_name_from_config;
        }
        else
        {
            std::string search_dir = policy_dir + "/" + robot_name_ + "/" + model_subdir;
            model_path = FindLatestOnnx(search_dir);
            if (model_path.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No .onnx model found");
                return false;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loading model: %s", model_path.c_str());
        model_ = InferenceRuntime::ModelFactory::load_model(model_path);
        if (!model_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model!");
            return false;
        }

        ComputeObsDims();

        int history_length = 1;
        if (!observations_history_.empty())
            history_length = *std::max_element(observations_history_.begin(), observations_history_.end()) + 1;
        history_obs_buf_ = ObservationBuffer(1, obs_dims_, history_length, "time");

        int obs_dim = 0;
        for (int dim : obs_dims_)
            obs_dim += dim;
        for (int i = 0; i < history_length; ++i)
            history_obs_buf_.insert(std::vector<float>(obs_dim, 0.0f));

        obs_.lin_vel = {0.0f, 0.0f, 0.0f};
        obs_.ang_vel = {0.0f, 0.0f, 0.0f};
        obs_.gravity_vec = {0.0f, 0.0f, -1.0f};
        obs_.commands = {0.0f, 0.0f, 0.0f};
        obs_.base_quat = {1.0f, 0.0f, 0.0f, 0.0f};
        obs_.dof_pos = default_dof_pos_;
        obs_.dof_vel.resize(num_of_dofs_, 0.0f);
        obs_.actions.resize(num_of_dofs_, 0.0f);
        last_action_.assign(num_of_dofs_, 0.0f);

        RCLCPP_INFO(this->get_logger(), "default_dof_pos: %s", FormatVector(default_dof_pos_).c_str());
        RCLCPP_INFO(this->get_logger(), "action_scale: %s", FormatVector(action_scale_).c_str());
        RCLCPP_INFO(this->get_logger(), "commands_scale: %s", FormatVector(commands_scale_).c_str());
        RCLCPP_INFO(this->get_logger(), "observations: %s", FormatStringVector(observations_).c_str());
        RCLCPP_INFO(this->get_logger(), "observations_history length: %d", history_length);
        RCLCPP_INFO(this->get_logger(), "joint_mapping: %s", FormatIntVector(joint_mapping_).c_str());
        RCLCPP_INFO(this->get_logger(), "inverse_mapping: %s", FormatIntVector(inverse_mapping_).c_str());

        rl_init_done_ = true;
        return true;
    }

    void RunModel()
    {
        if (!rl_init_done_)
            return;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (!all_sensors_ready_)
                return;

            obs_.ang_vel = {imu_gyro_[0], imu_gyro_[1], imu_gyro_[2]};
            obs_.gravity_vec = {imu_gravity_[0], imu_gravity_[1], imu_gravity_[2]};
            obs_.commands = {commands_buffer_[0], commands_buffer_[1], commands_buffer_[2]};

            // Remap joint data from driver order to policy order
            for (int i = 0; i < num_of_dofs_; ++i)
            {
                int driver_idx = joint_mapping_[i];
                obs_.dof_pos[i] = joint_pos_[driver_idx];
                obs_.dof_vel[i] = joint_vel_[driver_idx];
            }
        }

        std::vector<float> actions = Forward(obs_);
        obs_.actions = actions;
        std::vector<float> output_dof_pos = ComputeTargetPositions(actions);

        inference_count_++;
        RCLCPP_INFO(this->get_logger(),
                    "[Infer #%d] cmd=%s ang_vel=%s gravity=%s dof_pos4=%s dof_vel4=%s action=%s target=%s",
                    inference_count_,
                    FormatVector(obs_.commands).c_str(),
                    FormatVector(obs_.ang_vel).c_str(),
                    FormatVector(obs_.gravity_vec).c_str(),
                    FormatVector(std::vector<float>(obs_.dof_pos.begin(), obs_.dof_pos.begin() + std::min(4, num_of_dofs_))).c_str(),
                    FormatVector(std::vector<float>(obs_.dof_vel.begin(), obs_.dof_vel.begin() + std::min(4, num_of_dofs_))).c_str(),
                    FormatVector(actions).c_str(),
                    FormatVector(output_dof_pos).c_str());

        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            last_action_ = actions;
            output_dof_pos_queue_.push(output_dof_pos);
        }
    }

    void RobotControl()
    {
        if (!rl_init_done_ || !all_sensors_ready_)
            return;

        std::vector<float> policy_dof_pos;
        std::vector<float> fallback_action;
        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            while (!output_dof_pos_queue_.empty())
            {
                policy_dof_pos = output_dof_pos_queue_.front();
                output_dof_pos_queue_.pop();
            }
            fallback_action = last_action_;
        }

        if (policy_dof_pos.empty())
        {
            if (fallback_action.size() == static_cast<size_t>(num_of_dofs_))
                policy_dof_pos = ComputeTargetPositions(fallback_action);
            else
                policy_dof_pos = default_dof_pos_;
        }

        // Remap from policy order back to driver order for motor commands
        std::vector<float> driver_dof_pos(num_of_dofs_, 0.0f);
        for (int i = 0; i < num_of_dofs_; ++i)
        {
            int driver_idx = joint_mapping_[i];
            driver_dof_pos[driver_idx] = policy_dof_pos[i];
        }

        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = this->now();

        static const char* names[12] = {
            "lf_hipa", "lr_hipa", "rf_hipa", "rr_hipa",
            "lf_hipf", "lr_hipf", "rf_hipf", "rr_hipf",
            "lf_knee", "lr_knee", "rf_knee", "rr_knee"
        };

        for (int i = 0; i < num_of_dofs_; ++i)
        {
            cmd_msg.name.push_back(names[i]);
            cmd_msg.position.push_back(driver_dof_pos[i]);
        }

        motor_command_pub_->publish(cmd_msg);
    }

    std::vector<float> Forward(const Observations<float> &obs_snapshot)
    {
        auto inference_start = std::chrono::high_resolution_clock::now();
        std::vector<float> clamped_obs = ComputeObservation(obs_snapshot);
        std::vector<float> actions;
        std::vector<float> fallback_action;

        {
            std::lock_guard<std::mutex> lock(output_mutex_);
            fallback_action = last_action_;
        }

        if (!observations_history_.empty())
        {
            history_obs_buf_.insert(clamped_obs);
            history_obs_ = history_obs_buf_.get_obs_vec(observations_history_);
            actions = model_->forward({history_obs_});
        }
        else
        {
            actions = model_->forward({clamped_obs});
        }

        auto inference_end = std::chrono::high_resolution_clock::now();
        inference_time_ms_ = std::chrono::duration<double, std::milli>(inference_end - inference_start).count();

        if (actions.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Inference returned empty action, reusing last action");
            return fallback_action;
        }
        if (actions.size() != static_cast<size_t>(num_of_dofs_))
        {
            RCLCPP_ERROR(this->get_logger(), "Inference returned %zu actions, expected %d",
                         actions.size(), num_of_dofs_);
            return fallback_action;
        }

        for (float &value : actions)
        {
            if (std::isnan(value) || std::isinf(value))
            {
                RCLCPP_ERROR(this->get_logger(), "NaN/Inf detected in actions, reusing last action");
                return fallback_action;
            }
        }

        for (size_t i = 0; i < actions.size(); ++i)
        {
            float lower = (i < clip_actions_lower_.size()) ? clip_actions_lower_[i] : -1.0f;
            float upper = (i < clip_actions_upper_.size()) ? clip_actions_upper_[i] : 1.0f;
            actions[i] = std::max(lower, std::min(upper, actions[i]));
        }
        return actions;
    }

    std::vector<float> ComputeObservation(const Observations<float> &obs_snapshot)
    {
        std::vector<float> obs_vec;

        std::vector<float> scaled_gravity = obs_snapshot.gravity_vec;
        for (auto &v : scaled_gravity)
            v *= gravity_vec_scale_;

        for (const std::string &obs_name : observations_)
        {
            if (obs_name == "commands")
            {
                std::vector<float> scaled = obs_snapshot.commands;
                for (size_t i = 0; i < scaled.size() && i < commands_scale_.size(); ++i)
                    scaled[i] *= commands_scale_[i];
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "ang_vel")
            {
                std::vector<float> scaled = obs_snapshot.ang_vel;
                for (auto &v : scaled)
                    v *= ang_vel_scale_;
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "gravity_vec")
            {
                obs_vec.insert(obs_vec.end(), scaled_gravity.begin(), scaled_gravity.end());
            }
            else if (obs_name == "dof_pos")
            {
                std::vector<float> dof_pos_rel(num_of_dofs_, 0.0f);
                for (int i = 0; i < num_of_dofs_; ++i)
                    dof_pos_rel[i] = (obs_snapshot.dof_pos[i] - default_dof_pos_[i]) * dof_pos_scale_;
                obs_vec.insert(obs_vec.end(), dof_pos_rel.begin(), dof_pos_rel.end());
            }
            else if (obs_name == "dof_vel")
            {
                std::vector<float> scaled = obs_snapshot.dof_vel;
                for (auto &v : scaled)
                    v *= dof_vel_scale_;
                obs_vec.insert(obs_vec.end(), scaled.begin(), scaled.end());
            }
            else if (obs_name == "actions")
            {
                obs_vec.insert(obs_vec.end(), obs_snapshot.actions.begin(), obs_snapshot.actions.end());
            }
        }

        for (auto &v : obs_vec)
            v = std::max(-clip_obs_, std::min(clip_obs_, v));
        return obs_vec;
    }

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

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        imu_gyro_[0] = msg->angular_velocity.x;
        imu_gyro_[1] = msg->angular_velocity.y;
        imu_gyro_[2] = msg->angular_velocity.z;

        imu_gravity_[0] = msg->linear_acceleration.x;
        imu_gravity_[1] = msg->linear_acceleration.y;
        imu_gravity_[2] = msg->linear_acceleration.z;

        imu_received_ = true;
        if (motor_feedback_received_ && imu_received_)
            all_sensors_ready_ = true;
    }

    void XboxVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        commands_buffer_[0] = msg->linear.x;
        commands_buffer_[1] = msg->linear.y;
        commands_buffer_[2] = msg->angular.z;
    }

    std::vector<float> ComputeTargetPositions(const std::vector<float> &actions) const
    {
        std::vector<float> output_dof_pos(num_of_dofs_, 0.0f);
        size_t count = std::min(actions.size(), static_cast<size_t>(num_of_dofs_));
        for (size_t i = 0; i < count; ++i)
            output_dof_pos[i] = actions[i] * action_scale_[i] + default_dof_pos_[i];
        for (size_t i = count; i < output_dof_pos.size(); ++i)
            output_dof_pos[i] = default_dof_pos_[i];
        return output_dof_pos;
    }

    void EnsureVectorSizes()
    {
        if (action_scale_.size() < static_cast<size_t>(num_of_dofs_))
            action_scale_.resize(num_of_dofs_, 0.25f);
        if (commands_scale_.size() < 3)
            commands_scale_.resize(3, 1.0f);
        if (default_dof_pos_.size() < static_cast<size_t>(num_of_dofs_))
            default_dof_pos_.resize(num_of_dofs_, 0.0f);
        if (clip_actions_upper_.size() < static_cast<size_t>(num_of_dofs_))
            clip_actions_upper_.resize(num_of_dofs_, 1.0f);
        if (clip_actions_lower_.size() < static_cast<size_t>(num_of_dofs_))
            clip_actions_lower_.resize(num_of_dofs_, -1.0f);
        if (kp_.size() < static_cast<size_t>(num_of_dofs_))
            kp_.resize(num_of_dofs_, 10.0f);
        if (kd_.size() < static_cast<size_t>(num_of_dofs_))
            kd_.resize(num_of_dofs_, 0.3f);
        if (joint_mapping_.size() < static_cast<size_t>(num_of_dofs_))
            joint_mapping_.resize(num_of_dofs_, 0);
    }

    void ComputeObsDims()
    {
        obs_dims_.clear();
        for (const std::string &obs : observations_)
        {
            int dim = 0;
            if (obs == "lin_vel")
                dim = 3;
            else if (obs == "ang_vel")
                dim = 3;
            else if (obs == "gravity_vec")
                dim = 3;
            else if (obs == "commands")
                dim = 3;
            else if (obs == "dof_pos")
                dim = num_of_dofs_;
            else if (obs == "dof_vel")
                dim = num_of_dofs_;
            else if (obs == "actions")
                dim = num_of_dofs_;
            if (dim > 0)
                obs_dims_.push_back(dim);
        }
    }

    std::string FindLatestOnnx(const std::string &dir_path)
    {
        namespace fs = std::filesystem;
        std::string latest_path;
        std::filesystem::file_time_type latest_time;

        try
        {
            if (!fs::exists(dir_path) || !fs::is_directory(dir_path))
                return "";

            for (const auto &entry : fs::directory_iterator(dir_path))
            {
                if (!entry.is_regular_file())
                    continue;
                std::string filename = entry.path().filename().string();
                if (filename.size() >= 5 &&
                    strcasecmp(filename.c_str() + filename.size() - 5, ".onnx") == 0)
                {
                    auto mtime = entry.last_write_time();
                    if (latest_path.empty() || mtime > latest_time)
                    {
                        latest_path = entry.path().string();
                        latest_time = mtime;
                    }
                }
            }
        }
        catch (const std::exception &)
        {
        }

        return latest_path;
    }

    std::string FormatVector(const std::vector<float> &values) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4) << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
                oss << ", ";
            oss << values[i];
        }
        oss << "]";
        return oss.str();
    }

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

    std::string FormatStringVector(const std::vector<std::string> &values) const
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

    // ROS2 communication
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr xbox_vel_sub_;

    // Control loops
    std::shared_ptr<LoopFunc> loop_control_;
    std::shared_ptr<LoopFunc> loop_rl_;

    // Thread safety
    std::mutex data_mutex_;
    std::mutex output_mutex_;

    // Sensor data buffers (driver order)
    std::array<float, 12> joint_pos_{};
    std::array<float, 12> joint_vel_{};
    std::array<float, 3> commands_buffer_{};
    std::array<float, 3> imu_gyro_{};
    std::array<float, 3> imu_gravity_{};
    bool motor_feedback_received_{false};
    bool imu_received_{false};

    // Output queue
    std::queue<std::vector<float>> output_dof_pos_queue_;
    std::vector<float> latest_action_for_log_;

    // RL framework
    Observations<float> obs_;
    ObservationBuffer history_obs_buf_;
    std::vector<float> history_obs_;
    std::unique_ptr<InferenceRuntime::Model> model_;
    std::vector<float> last_action_;

    // Config-driven parameters
    std::vector<int> obs_dims_;
    std::vector<std::string> observations_;
    std::vector<int> observations_history_;
    std::vector<float> action_scale_;
    std::vector<float> commands_scale_;
    std::vector<float> default_dof_pos_;
    std::vector<float> clip_actions_upper_;
    std::vector<float> clip_actions_lower_;
    std::vector<float> kp_;
    std::vector<float> kd_;

    // Joint mapping: joint_mapping_[i] = driver index for policy index i
    std::vector<int> joint_mapping_;
    std::vector<int> inverse_mapping_;

    // Basic parameters
    std::string robot_name_;
    int num_of_dofs_;
    float dt_;
    int decimation_;
    float lin_vel_scale_;
    float ang_vel_scale_;
    float dof_pos_scale_;
    float dof_vel_scale_;
    float gravity_vec_scale_;
    float clip_obs_;

    // State flags
    bool rl_init_done_;
    bool all_sensors_ready_;
    int inference_count_;
    double inference_time_ms_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting ARES HIMloco RL Node...");

    auto node = std::make_shared<ARSNodeHimloco>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    RCLCPP_INFO(rclcpp::get_logger("main"), "ARES HIMloco RL Node shutting down...");
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/rl_sar/src/rl_real_ares_himloco.cpp
git commit -m "feat: add ares_himloco C++ executable with joint mapping support"
```

---

## Task 5: CMakeLists.txt — Add Build Target

**Files:**
- Modify: `src/rl_sar/CMakeLists.txt:277` (after `ares` install target)

- [ ] **Step 1: Add ares_himloco target**

After the existing `install(TARGETS ares DESTINATION lib/${PROJECT_NAME})` line (line 277), and before the `ares_driver_node` block, add:

```cmake
    # executable for ARES HIMloco RL inference node
    add_executable(ares_himloco src/rl_real_ares_himloco.cpp)
    target_link_libraries(ares_himloco
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        Threads::Threads
    )
    ament_target_dependencies(ares_himloco
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
    )
    install(TARGETS ares_himloco DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 2: Verify build compiles**

```bash
cd /home/wufy/projects/UIKA_HIMloco/rl_sar-ARES && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select rl_sar 2>&1 | tail -20
```

Expected: Build succeeds, `build/rl_sar/bin/ares_himloco` binary exists.

- [ ] **Step 3: Commit**

```bash
git add src/rl_sar/CMakeLists.txt
git commit -m "build: add ares_himloco executable target to CMakeLists"
```

---

## Task 6: Build Verification

**Files:** None (verification only)

- [ ] **Step 1: Clean build both executables**

```bash
cd /home/wufy/projects/UIKA_HIMloco/rl_sar-ARES
source /opt/ros/humble/setup.bash
rm -rf build/rl_sar install/rl_sar
colcon build --symlink-install --packages-select rl_sar 2>&1 | tail -30
```

Expected: Both `build/rl_sar/bin/ares` and `build/rl_sar/bin/ares_himloco` exist.

- [ ] **Step 2: Verify existing ares binary is unaffected**

```bash
ls -la build/rl_sar/bin/ares build/rl_sar/bin/ares_himloco build/rl_sar/bin/ares_driver_node
```

Expected: All three binaries exist with recent timestamps.

- [ ] **Step 3: Verify no modifications to original source**

```bash
git diff src/rl_sar/src/rl_real_ares.cpp
```

Expected: No output (file unchanged).

---

## Self-Review Checklist

**1. Spec coverage:**
- [x] Config fixes (joint_mapping + commands_scale) → Task 1
- [x] ONNX export script → Task 2
- [x] MuJoCo sim test script → Task 3
- [x] New C++ executable → Task 4
- [x] CMakeLists.txt addition → Task 5
- [x] Build verification → Task 6

**2. Placeholder scan:**
- No TBDs, TODOs, or vague requirements found
- All code is complete in each step
- All commands have expected output documented

**3. Type consistency:**
- `joint_mapping_` is `std::vector<int>` everywhere in C++ code
- `inverse_mapping_` is `std::vector<int>` — used for logging only, not for remapping
- Remapping uses `joint_mapping_[policy_idx] = driver_idx` consistently in RunModel() and RobotControl()
- Config YAML joint_mapping is a list of ints — matches
- Python script uses numpy arrays consistently

**4. Architecture verification:**
- No existing source files modified (only CMakeLists.txt gets target appended)
- New .cpp file is self-contained (no shared-base-class refactor)
- Same ROS2 topics and message types as existing node
- Joint mapping applied at observation construction and command output boundaries

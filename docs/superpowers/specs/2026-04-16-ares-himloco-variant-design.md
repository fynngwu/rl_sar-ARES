# ARES HIMloco Variant Design Spec

**Goal:** Add HIMloco policy support for the ARES quadruped — MuJoCo sim testing, ONNX export, and a new C++ real-deployment executable — without modifying any existing code.

**Architecture:** Three independent deliverables: (1) standalone Python MuJoCo sim test, (2) ONNX export script, (3) new `ares_himloco` C++ executable. All new files, zero modifications to existing `rl_real_ares.cpp`, `ares_driver_node.cpp`, or any shared libraries.

**Tech Stack:** Python 3 (torch, mujoco, onnxruntime), ROS2 Humble (C++17), ONNX Runtime

---

## 1. Policy Model (policy.pt)

TorchScript `RecursiveScriptModule` (HIMActorCritic):

```
estimator(obs_history[270]) -> parts[19]
  vel = parts[:3]
  z = parts[3:19]
  z0 = normalize(z)

actor(cat(obs_current[45], vel[3], z0[16])[64]) -> actions[12]
```

- Estimator: Linear(270,128) → ReLU → Linear(128,64) → ReLU → Linear(64,19)
- Actor: Linear(64,512) → ReLU → Linear(512,256) → ReLU → Linear(256,128) → ReLU → Linear(128,12)
- Input: `[batch, 270]` (6 frames × 45 dims)
- Output: `[batch, 12]` (joint position targets)

### Observation Vector (45 dims, commands-first)

| Indices | Dim | Content | Scale |
|---------|-----|---------|-------|
| 0–2 | 3 | Velocity command [vx, vy, vyaw] | [3.5, 2.0, 1.0] (cmd_scale) |
| 3–5 | 3 | Angular velocity (body frame, rad/s) | 0.25 |
| 6–8 | 3 | Projected gravity (body frame) | 1.0 |
| 9–20 | 12 | Joint positions (relative to default) | 1.0 |
| 21–32 | 12 | Joint velocities | 0.05 |
| 33–44 | 12 | Previous actions | 1.0 |

### Policy Training Parameters

| Parameter | Value |
|-----------|-------|
| PD gains | kp=10, kd=0.3 |
| Decimation | 2 (100Hz policy at 200Hz sim) |
| History | 6 frames (270 dims total) |
| Default pose | [0, 0.9, -1.5] per leg |
| Action scale | 0.25 |
| cmd_scale | [3.5, 2.0, 1.0] |
| Base height | 0.15m |

## 2. Joint Ordering

### Three orderings, one mapping

| Index | Policy (IsaacGym URDF) | Driver (CAN bus) | MJCF (dogv2_view.xml) |
|-------|------------------------|-------------------|-----------------------|
| 0 | LF_HipA | LF_HipA (0) | LF_HipA |
| 1 | LF_HipF | LF_HipF (4) | LF_HipF |
| 2 | LF_Knee | LF_Knee (8) | LF_Knee |
| 3 | LR_HipA | LR_HipA (1) | LH_HipA |
| 4 | LR_HipF | LR_HipF (5) | LH_HipF |
| 5 | LR_Knee | LR_Knee (9) | LH_Knee |
| 6 | RF_HipA | RF_HipA (2) | RF_HipA |
| 7 | RF_HipF | RF_HipF (6) | RF_HipF |
| 8 | RF_Knee | RF_Knee (10) | RF_Knee |
| 9 | RR_HipA | RR_HipA (3) | RH_HipA |
| 10 | RR_HipF | RR_HipF (7) | RH_HipF |
| 11 | RR_Knee | RR_Knee (11) | RH_Knee |

- **Policy ↔ MJCF**: Same per-leg ordering (FL, RL, FR, RR). LH=LR, RH=RR naming only. No remapping needed for MuJoCo sim.
- **Driver ↔ Policy**: Different. Driver uses joint-type grouping (HipA×4, HipF×4, Knee×4). Mapping: `driver_to_policy = [0,4,8, 3,7,11, 1,5,9, 2,6,10]` (i.e., `policy[i] = driver[mapping[i]]`)

## 3. Deliverable 1: MuJoCo Sim Test Script

**File:** `scripts/test_himloco_mujoco.py`

**Purpose:** Standalone Python script to test the HIMloco policy in MuJoCo simulation with the dogv2 model. Validates that the policy produces reasonable locomotion before deploying on hardware.

### Behavior

1. Parse args: `--policy` (path to .pt or .onnx), `--xml` (MJCF path), `--mode` (torch/onnx)
2. Load config from `policy/ares_himloco/base.yaml` and `policy/ares_himloco/himloco/config.yaml`
3. Load MuJoCo model, add position actuators if none exist (dogv2_view.xml has 0 actuators)
4. Initialize robot at default pose, height 0.15m
5. Run 200Hz sim loop with 100Hz policy control (decimation=2)
6. Keyboard control: arrow keys for vx/vyaw
7. MuJoCo viewer with passive launch

### Observation Construction (per step, 45 dims)

```python
obs[0:3]   = cmd * cmd_scale           # velocity command
obs[3:6]   = omega * ang_vel_scale     # angular velocity
obs[6:9]   = projected_gravity         # quat_rotate_inverse(quat, [0,0,-1])
obs[9:21]  = (qpos - default) * dof_pos_scale  # joint positions
obs[21:33] = qvel * dof_vel_scale      # joint velocities
obs[33:45] = prev_action               # previous actions
```

History buffer: `deque(maxlen=6)`, append left (newest first). Concatenate to 270 dims.

### PD Control

```python
tau = kp * (target_pos - qpos) - kd * qvel
```

### MuJoCo Actuator Setup

dogv2_view.xml has 0 actuators. The script must add 12 position actuators programmatically or use a modified XML. Approach: use `mujoco.mj_setConst` to add actuators, or create a temporary XML wrapper that adds actuators to the existing model.

**Environment:** Uses `env_isaaclab` conda env (has torch 2.7.0). Will need `pip install mujoco onnxruntime` there, OR detect environment and fall back.

## 4. Deliverable 2: ONNX Export Script

**File:** `scripts/export_onnx.py`

**Purpose:** Convert policy.pt to policy.onnx for deployment.

### Behavior

1. Accept `--input` (policy.pt path) and `--output` (policy.onnx path) args
2. Load TorchScript model
3. Export with `torch.onnx.export()`, opset 17, dynamic batch axis
4. Verify with `onnx.load()` + print input/output shapes

## 5. Deliverable 3: New C++ Executable (ares_himloco)

**File:** `src/rl_sar/src/rl_real_ares_himloco.cpp`

**Purpose:** ROS2 node for real-robot HIMloco policy inference. Separate executable from `ares`, does not modify `rl_real_ares.cpp`.

### ROS2 Interface

Same topics as existing `ares` node:
- Subscribe: `/motor_feedback` (sensor_msgs/JointState), `/imu/data` (sensor_msgs/Imu), `/xbox_vel` (geometry_msgs/Twist)
- Publish: `/motor_command` (sensor_msgs/JointState)

### Key Differences from `ares` (rl_real_ares.cpp)

| Feature | ares | ares_himloco |
|---------|------|-------------|
| PD gains | kp=40, kd=0.5 | kp=10, kd=0.3 |
| History | 10 frames | 6 frames |
| Obs order | ang_vel, gravity, cmd, pos, vel, actions | cmd, ang_vel, gravity, pos, vel, actions |
| cmd_scale | [1.2, 0.5, 0.5] | [3.5, 2.0, 1.0] |
| Default pose | [0]×12 | [0, 0.9, -1.5]×4 |
| Joint mapping | None (same order) | driver→policy remap |
| Config namespace | `ares` | `ares_himloco` |

### Implementation Approach

The new `.cpp` file contains its own class (`ARSNodeHimloco` or similar) that:
- Inherits from or reimplements the RL SDK interface
- Loads config from `policy/ares_himloco/` namespace
- Applies `joint_mapping` when converting between driver and policy joint orderings
- Follows the same ROS2 subscriber/publisher pattern as the existing `ares` node

This is a **copy-and-adapt** of `rl_real_ares.cpp` with the HIMloco-specific changes, not a shared-base-class refactor.

### CMakeLists.txt Addition

Add new executable target:

```cmake
add_executable(ares_himloco src/rl_real_ares_himloco.cpp)
target_link_libraries(ares_himloco
    rl_sdk
    observation_buffer
    ${YAML_CPP_LIBRARIES}
    Threads::Threads
)
ament_target_dependencies(ares_himloco
    rclcpp std_msgs geometry_msgs sensor_msgs
)
install(TARGETS ares_himloco DESTINATION lib/${PROJECT_NAME})
```

## 6. Config Files (already created, minor fixes needed)

### policy/ares_himloco/base.yaml

Fix `joint_mapping` from `[0,4,8, 2,6,10, 1,5,9, 3,7,11]` to `[0,4,8, 3,7,11, 1,5,9, 2,6,10]` (FL, RL, FR, RR per-leg order).

### policy/ares_himloco/himloco/config.yaml

Fix `commands_scale` from `[1.2, 0.5, 0.5]` to `[3.5, 2.0, 1.0]` to match training config.

## 7. File Summary

| File | Action | Description |
|------|--------|-------------|
| `scripts/export_onnx.py` | Create | policy.pt → ONNX converter |
| `scripts/test_himloco_mujoco.py` | Create | MuJoCo sim test script |
| `src/rl_sar/src/rl_real_ares_himloco.cpp` | Create | New C++ executable |
| `src/rl_sar/CMakeLists.txt` | Modify (add target only) | Add ares_himloco build target |
| `policy/ares_himloco/base.yaml` | Fix | Correct joint_mapping |
| `policy/ares_himloco/himloco/config.yaml` | Fix | Correct commands_scale |
| `policy/ares_himloco/himloco/policy.onnx` | Create (exported) | ONNX policy file |

**No existing source files are modified** (only CMakeLists.txt gets a new target appended).

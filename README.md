# rl_sar-ARES

RL policy inference for the ARES quadruped robot. Based on the [rl_sar](https://github.com/fan-ziqi/rl_sar) framework, adapted for ARES hardware (Robstride motors via CAN, WIT IMU via serial).

## Architecture

Two ROS2 Humble nodes communicating via standard ROS2 topics:

```
ares_driver_node  ──/motor_feedback──>  ares_rl_node
(sensor_msgs/JointState)            (ONNX inference)

ares_driver_node  ──/imu/data────────>  ares_rl_node
(sensor_msgs/Imu)                   (obs construction)

ares_rl_node     ──/motor_command──>  ares_driver_node
(sensor_msgs/JointState)            (CAN MIT commands)

ares_driver_node  ──/xbox_vel────────>  ares_rl_node
(geometry_msgs/Twist)               (gamepad velocity command)
```

- **ares_driver_node** (`src/ares_driver_node.cpp`) — Wraps `libdog_driver.so`. Opens 4 CAN buses (can0–can3) + WIT IMU, runs a 200Hz control loop that reads motor feedback/IMU and sends MIT position commands.
- **ares_rl_node** (`src/rl_real_ares.cpp`) — Runs ONNX policy inference with observation history buffer and FSM state machine (Passive → GetUp → RLLocomotion).

No custom ROS2 message types needed — all topics use `sensor_msgs` and `geometry_msgs`.

## Topic Interface

| Topic | Direction | Message Type | Content |
|-------|-----------|--------------|---------|
| `/motor_feedback` | driver → rl | `sensor_msgs/JointState` | name[12] + position[12] + velocity[12] |
| `/imu/data` | driver → rl | `sensor_msgs/Imu` | orientation (quaternion) + angular_velocity + linear_acceleration* |
| `/motor_command` | rl → driver | `sensor_msgs/JointState` | name[12] + position[12] (target positions) |
| `/xbox_vel` | driver → rl | `geometry_msgs/Twist` | linear.x, linear.y, angular.z (gamepad) |

*Projected gravity is carried in `linear_acceleration` (no standard ROS2 field for projected gravity). The driver computes it from the IMU quaternion internally.

## Joint Ordering

The project is transitioning from **joint-type grouped** to **per-leg grouped** ordering to match the URDF/training joint order. The current state:

### `ares` (default) — still uses driver order (joint-type grouped)

| Index | Joint | Leg |
|-------|-------|-----|
| 0–3 | HipA (yaw) | LF, LR, RF, RR |
| 4–7 | HipF (pitch) | LF, LR, RF, RR |
| 8–11 | Knee | LF, LR, RF, RR |

### `ares_himloco` — uses URDF order (per-leg grouped) with joint_mapping

Training uses URDF order (FL, RL, FR, RR):

| Index | Joint | Leg |
|-------|-------|-----|
| 0–2 | HipA, HipF, Knee | FL (left-front) |
| 3–5 | HipA, HipF, Knee | RL (right-left, i.e. rear-left) |
| 6–8 | HipA, HipF, Knee | FR (right-front) |
| 9–11 | HipA, HipF, Knee | RR (right-rear) |

The driver still reads/writes in joint-type order. A `joint_mapping` array remaps at the RL node boundary:

```
joint_mapping: [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
# Policy idx 0 (FL_HipA)  → Driver idx 0
# Policy idx 1 (FL_HipF)  → Driver idx 4
# Policy idx 2 (FL_Knee)  → Driver idx 8
# ...
```

**TODO**: Reorder the driver layer (DogDriver + ares_driver_node) to use per-leg ordering natively, then remove `joint_mapping`.

### Knee Gear Ratio

Knee joints have a gear ratio of **1.667**, handled entirely within `DogDriver`:
- `GetJointStates()` already divides knee positions/velocities by 1.667
- `SetAllJointPositions()` already multiplies knee targets by 1.667 before sending to motors

The RL node does **not** need to handle gear ratio.

## Policy Variants

### `ares` (default)

Original ARES configuration with ang_vel-first observation ordering and 10-frame history.

| Parameter | Value |
|-----------|-------|
| PD gains | kp=40, kd=0.5 |
| Decimation | 4 (50Hz policy) |
| Obs order | ang_vel, gravity_vec, commands, dof_pos, dof_vel, actions |
| History | 10 frames (450 dims) |
| Default pose | [0]×12 |
| Action scale | 0.25 |
| Commands scale | [1.2, 0.5, 0.5] |

Config: `policy/ares/base.yaml` + `policy/ares/himloco/config.yaml`

### `ares_himloco`

HIMloco-compatible variant matching `dog_v2_2_4` training. Parameters adapted from `legged_gym/envs/dog_v2_2_4/dog_v2_2_4_config.py`.

| Parameter | Value | Training source |
|-----------|-------|-----------------|
| PD gains | kp=30, kd=1.0 | `control_config.stiffness=30, damping=1` |
| Decimation | 4 (50Hz policy) | `decimation=4, dt=0.005` |
| Obs order | commands, ang_vel, gravity_vec, dof_pos, dof_vel, actions | `observations` in config |
| History | 6 frames (270 dims) | `observations_history` in config |
| Default pose | [0, 0.55, -0.95, 0, 0.55, -0.95, 0, -0.55, 0.95, 0, -0.55, 0.95] | `default_joint_angles` |
| Action scale | 0.25 | `action_scale` |
| Commands scale | [2.0, 2.0, 0.25] | `obs_scales.lin_vel=2.0, obs_scales.ang_vel=0.25` |
| Obs scales | dof_pos=1.0, dof_vel=0.05, ang_vel=0.25 | `obs_scales` in config |
| Clip obs/actions | 100.0 / ±10 | `normalization.clip_observations/actions` |

Config: `policy/ares_himloco/base.yaml` + `policy/ares_himloco/himloco/config.yaml`

**Status**: YAML configs updated. Driver reorder + joint_mapping removal pending.

## Observation Vector (ares_himloco)

Single-frame layout (45 dims):

| Indices | Dim | Content | Scale |
|---------|-----|---------|-------|
| 0–2 | 3 | Velocity command [vx, vy, vyaw] | [2.0, 2.0, 0.25] |
| 3–5 | 3 | Angular velocity (body frame, rad/s) | 0.25 |
| 6–8 | 3 | Projected gravity (body frame, unit vector) | 1.0 |
| 9–20 | 12 | Joint positions (relative to default) | 1.0 |
| 21–32 | 12 | Joint velocities | 0.05 |
| 33–44 | 12 | Previous actions | 1.0 |

Total model input = 45 × 6 (history) = **270 dims**.

## Data Flow Trace (ares_himloco)

Step-by-step trace of how raw sensor data becomes an observation and then an action:

### 1. Motor Feedback Path (driver → rl node)

```
CAN motors (physical)
  → DogDriver::GetJointStates()        # reads CAN, applies gear ratio (÷1.667 for knees)
    → positions[12], velocities[12]    # driver order (joint-type grouped)
      → ares_driver_node publishes /motor_feedback
        → rl_real_ares::MotorFeedbackCallback()
          → joint_mapping_[i] remaps to policy order (URDF: FL/RL/FR/RR)
            → dof_pos_[12], dof_vel_[12] stored for observation
```

### 2. IMU Path (driver → rl node)

```
WIT IMU (serial)
  → DogDriver::GetIMUMsg()             # reads serial, extracts gyro + quaternion
    → DogDriver computes projected gravity from quaternion
      → ares_driver_node publishes /imu/data
        → rl_real_ares::IMUMsgCallback()
          → imu_msg_.angular_velocity[3]  → obs ang_vel
          → imu_msg_.linear_acceleration  → obs projected_gravity
            (NOTE: linear_acceleration is repurposed for projected gravity)
```

### 3. Command Path (gamepad → rl node)

```
Xbox gamepad
  → DogDriver::GetXboxMsg()
    → ares_driver_node publishes /xbox_vel
      → rl_real_ares::XboxVelCallback()
        → cmd_vel_[3] = {linear.x, linear.y, angular.z}
```

### 4. Observation Construction (RunModel)

```
cmd_vel_[3]      × commands_scale [2.0, 2.0, 0.25]
imu angular_vel  × ang_vel_scale 0.25
imu grav_proj    × 1.0
dof_pos_ - default_dof_pos  × dof_pos_scale 1.0
dof_vel_         × dof_vel_scale 0.05
last_action_     × 1.0
  → obs_buffer_.push(obs[45])          # newest-first insertion
    → obs_history_[270] = stack 6 frames
      → model_->Run(obs_history_)      # ONNX inference
        → output action_raw_[12]
```

### 5. Action Path (rl node → driver → motors)

```
action_raw_[12]   (policy order, URDF: FL/RL/FR/RR)
  × action_scale 0.25
  + default_dof_pos
  → joint_mapping_ inverse remaps to driver order
    → publish /motor_command
      → ares_driver_node subscribes
        → DogDriver::SetAllJointPositions()
          × gear ratio (×1.667 for knees)
            → CAN MIT position commands → motors
```

### Key Verification Points

| Step | What to verify | How |
|------|----------------|-----|
| Motor feedback | Positions ≈ standing pose, velocities ≈ 0 at rest | `test_driver_feedback` node |
| IMU data | angular_velocity near 0 at rest, gravity ≈ [0, -9.81, 0] | `test_driver_feedback` node |
| Observation | All values in reasonable range, no NaN | `test_onnx_smoke` (synthetic) |
| ONNX inference | Output non-zero, changes with command | `test_onnx_smoke` (synthetic) |
| Mapping correctness | Joint indices match between driver and policy | Check `joint_mapping` in YAML |

## Directory Structure

```
rl_sar-ARES/
├── src/rl_sar/
│   ├── CMakeLists.txt
│   ├── package.ros2.xml → package.xml
│   ├── src/
│   │   ├── rl_real_ares.cpp          # RL inference node
│   │   └── ares_driver_node.cpp      # Hardware driver node
│   ├── test/
│   │   ├── test_onnx_smoke.cpp       # ONNX model smoke test (standalone, no ROS2)
│   │   └── test_driver_feedback.cpp  # Motor feedback + IMU data reader (needs ROS2)
│   ├── fsm_robot/
│   │   ├── fsm.hpp                   # FSM base classes
│   │   ├── fsm_ares.hpp              # ARES FSM states (Passive/GetUp/GetDown/RLLocomotion)
│   │   └── fsm_all.hpp
│   ├── include/                      # rl_sdk, observation_buffer headers
│   └── library/core/                 # inference_runtime, rl_sdk, observation_buffer, fsm, etc.
├── driver/
│   ├── include/dog_driver.hpp        # DogDriver API (TODO: reorder to per-leg)
│   └── libdog_driver.so              # Pre-built CAN + IMU driver (TODO: rebuild)
├── policy/
│   ├── ares/                         # Default ARES variant
│   │   ├── base.yaml
│   │   └── himloco/
│   │       ├── config.yaml
│   │       └── policy.onnx
│   └── ares_himloco/                 # HIMloco-compatible variant (YAML updated)
│       ├── base.yaml                 # Updated: kp/kd, default_dof_pos, joint_mapping
│       └── himloco/
│           ├── config.yaml           # Updated: commands_scale, obs scales, kp/kd
│           └── policy.onnx
├── library/inference_runtime/        # ONNX Runtime (system-wide or vendored)
└── dogv2/mjcf/                       # MuJoCo MJCF model for simulation
```

## Building

### Prerequisites

- ROS2 Humble
- C++17 compiler
- yaml-cpp, TBB
- Python3 with NumPy development headers
- ONNX Runtime (v1.17.1+) — system-wide or in `library/inference_runtime/onnxruntime/`
- (Optional) LibTorch — for PyTorch model support

### Build

```bash
# From the rl_sar-ARES directory
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Build Outputs

- `build/rl_sar/bin/ares` — RL inference node
- `build/rl_sar/bin/ares_driver_node` — Hardware driver node
- `build/rl_sar/bin/test_onnx_smoke` — ONNX model smoke test (no ROS2 needed)
- `build/rl_sar/bin/test_driver_feedback` — Motor feedback reader (needs ROS2 + hardware)

## Running

```bash
# Terminal 1: Driver node (requires hardware)
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rl_sar ares_driver_node

# Terminal 2: RL node — default ARES variant
ros2 run rl_sar ares --ros-args -p robot_name:=ares

# Terminal 2 (alt): RL node — HIMloco variant
ros2 run rl_sar ares --ros-args -p robot_name:=ares_himloco
```

### Testing

```bash
# Step 1: ONNX smoke test (no hardware needed)
# Verifies: model loads, default observation produces valid output, output responds to commands
cd build/rl_sar
./bin/test_onnx_smoke

# Step 2: Driver feedback test (needs hardware + running driver)
# In terminal 1: ros2 run rl_sar ares_driver_node
# In terminal 2:
source install/setup.bash
ros2 run rl_sar test_driver_feedback
# Prints 10 samples of /motor_feedback + /imu/data, then exits

# Step 3 (after driver reorder): Full integration test
# With driver reorder complete, joint_mapping is removed.
# Verify joints match by comparing test_driver_feedback output with expected URDF order.
```

### FSM Controls

| Key | Gamepad | State Transition |
|-----|---------|-----------------|
| 0 | A | Passive → GetUp |
| 1 | RB+DPadUp | GetUp → RLLocomotion |
| 9 | B | GetUp/RLLocomotion → GetDown |
| P | LB+X | Any → Passive |

## Pending Work (Driver Reorder)

The driver layer still uses joint-type ordering. The planned change:

1. **`dog_driver.hpp`**: Reorder `kMotorIds`, `kOffsets`, `kJointDirection` arrays from joint-type order to per-leg order (FL/RL/FR/RR)
2. **`dog_driver.cpp`**: Fix Knee gear ratio check from `i >= 8` to per-joint check (knees at indices 2,5,8,11)
3. **`ares_driver_node.cpp`**: Reorder `kJointNames` to match new driver order
4. **`rl_real_ares.cpp`**: Remove `joint_mapping_` logic, simplify I/O
5. **YAML configs**: Remove `joint_mapping`, update `joint_names`
6. **Rebuild** `libdog_driver.so` (requires C++ build environment for the driver)

After this change, the data flow simplifies — no index remapping at the RL node boundary.

## Key Differences from UIKA

| Parameter | UIKA | ARES (ares_himloco) |
|-----------|------|---------------------|
| PD gains (kp, kd) | 30, 1 | 30, 1.0 |
| History length | 6 | 6 |
| Obs order | cmd, ang_vel, gravity, pos, vel, actions | cmd, ang_vel, gravity, pos, vel, actions |
| Joint order | FL/FR/RL/RR (leg-grouped) | FL/RL/FR/RR (URDF, with mapping) |
| Commands scale | [1.2, 0.5, 0.5] | [2.0, 2.0, 0.25] |
| Knee gear ratio | N/A | 1.667 |
| IMU | ROS2 topic | WIT sensor via serial (from driver) |
| Default angles | Non-zero offsets | [0, 0.55, -0.95] per leg (training values) |
| Torque limits | [6, 6, 11.2] per leg | [17.0] all joints |

## Unchanged from rl_sar Framework

The following shared components are not modified:
- `library/core/rl_sdk.hpp` — RL SDK base class
- `library/core/observation_buffer.hpp` — History buffer (newest-first insertion)
- `library/core/inference_runtime/` — ONNX Runtime wrapper
- `library/core/loop.hpp` — Loop timing
- `library/core/fsm.hpp` — FSM framework
- `library/core/vector_math.hpp` — Math utilities

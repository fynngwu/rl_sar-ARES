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

- **ares_driver_node** — Wraps `libdog_driver.so`. Opens 4 CAN buses (can0–can3) + WIT IMU, runs a 200Hz control loop. Reads kp/kd/torque_limits/gamepad_scale from YAML config.
- **ares_rl_node** — Runs ONNX policy inference with observation history buffer.

No custom ROS2 message types needed — all topics use `sensor_msgs` and `geometry_msgs`.

## Topic Interface

| Topic | Direction | Message Type | Content |
|-------|-----------|--------------|---------|
| `/motor_feedback` | driver → rl | `sensor_msgs/JointState` | name[12] + position[12] + velocity[12] |
| `/imu/data` | driver → rl | `sensor_msgs/Imu` | angular_velocity + linear_acceleration* |
| `/motor_command` | rl → driver | `sensor_msgs/JointState` | position[12] (target positions) |
| `/xbox_vel` | driver → rl | `geometry_msgs/Twist` | gamepad velocity command |

*Projected gravity is carried in `linear_acceleration`.

## Joint Ordering

All topics use **FL RL FR RR** (URDF order):

| Index | Joint | Leg |
|-------|-------|-----|
| 0–2 | HipA, HipF, Knee | FL |
| 3–5 | HipA, HipF, Knee | RL |
| 6–8 | HipA, HipF, Knee | FR |
| 9–11 | HipA, HipF, Knee | RR |

The driver node handles reordering internally between URDF order and driver internal order.

### Knee Gear Ratio

Knee joints have a gear ratio of **1.667**, handled entirely within `DogDriver`. The RL node does not need to handle gear ratio.

## Config

All parameters in a single YAML file: `policy/ares_himloco/himloco/config.yaml`

Both nodes share this file. Key parameters:

| Parameter | Description |
|-----------|-------------|
| `fixed_kp` / `fixed_kd` | PD gains (driver reads at init) |
| `torque_limits` | Per-joint torque limit (Nm) |
| `gamepad_scale` | Gamepad axis scaling |
| `commands_scale` | Velocity command scaling [vx, vy, vyaw] |
| `action_scale` | Action output scaling |
| `default_dof_pos` | Default standing pose |
| `observations` | Observation order |
| `observations_history` | History frame indices |

Modify this file and restart nodes — no recompilation needed.

## Building

### Prerequisites

- ROS2 Humble (`source /opt/ros/humble/setup.bash`)
- C++17 compiler, CMake 3.10+, yaml-cpp, TBB
- Python3 with NumPy development headers
- ONNX Runtime (v1.17.1+) in `library/inference_runtime/onnxruntime/`

### One-click build

```bash
./build.sh
```

This builds the driver library and all ROS2 nodes.

### Manual build

```bash
# 1. Build driver library
cmake -S driver -B driver/build
cmake --build driver/build

# 2. Build ROS2 nodes
source /opt/ros/humble/setup.bash
cmake -S src/rl_sar -B src/rl_sar/build
cmake --build src/rl_sar/build
```

To rebuild after changes:
```bash
./build.sh                    # rebuilds everything
cmake --build driver/build    # only driver
cmake --build src/rl_sar/build # only nodes
```

### Build Outputs

| Binary | Description |
|--------|-------------|
| `src/rl_sar/build/bin/ares` | RL inference node |
| `src/rl_sar/build/bin/ares_driver_node` | Hardware driver node |
| `src/rl_sar/build/bin/test_onnx_smoke` | ONNX smoke test (no hardware) |
| `src/rl_sar/build/bin/test_driver_feedback` | Motor feedback reader (needs hardware) |
| `driver/libdog_driver.so` | CAN + IMU driver library |

## Running

```bash
source /opt/ros/humble/setup.bash

# Terminal 1: Driver node (requires hardware)
./src/rl_sar/build/bin/ares_driver_node

# Terminal 2: RL node
./src/rl_sar/build/bin/ares
```

### Testing

```bash
# ONNX smoke test (no hardware)
./src/rl_sar/build/bin/test_onnx_smoke

# Driver feedback test (needs hardware + running driver)
source /opt/ros/humble/setup.bash
./src/rl_sar/build/bin/test_driver_feedback
```

### FSM Controls

| Key | Gamepad | State Transition |
|-----|---------|-----------------|
| 0 | A | Passive → GetUp |
| 1 | RB+DPadUp | GetUp → RLLocomotion |
| 9 | B | GetUp/RLLocomotion → GetDown |
| P | LB+X | Any → Passive |

## Directory Structure

```
rl_sar-ARES/
├── build.sh                          # One-click build script
├── src/rl_sar/
│   ├── CMakeLists.txt                # Pure cmake build (no colcon)
│   ├── src/
│   │   ├── rl_real_ares.cpp          # RL inference node
│   │   └── ares_driver_node.cpp      # Hardware driver node
│   ├── test/                         # Test binaries
│   ├── fsm_robot/                    # FSM states
│   ├── include/                      # rl_sdk, observation_buffer headers
│   └── library/core/                 # inference_runtime, rl_sdk, observation_buffer, etc.
├── driver/
│   ├── CMakeLists.txt
│   ├── include/dog_driver.hpp
│   ├── src/                          # CAN, serial, IMU, motor control sources
│   └── libdog_driver.so
├── policy/
│   └── ares_himloco/
│       └── himloco/
│           ├── config.yaml           # Unified config (RL + driver params)
│           └── policy.onnx
└── library/inference_runtime/        # ONNX Runtime
```

## Key Differences from UIKA

| Parameter | UIKA | ARES |
|-----------|------|------|
| PD gains (kp, kd) | 30, 1 | 30, 1.0 |
| History length | 6 | 6 |
| Obs order | cmd, ang_vel, gravity, pos, vel, actions | cmd, ang_vel, gravity, pos, vel, actions |
| Joint order | FL/FR/RL/RR | FL/RL/FR/RR (URDF) |
| Commands scale | [1.2, 0.5, 0.5] | [-4.0, 2.0, 0.25] |
| Knee gear ratio | N/A | 1.667 |
| IMU | ROS2 topic | WIT sensor via serial |
| Torque limits | [6, 6, 11.2] per leg | [17.0] all joints |

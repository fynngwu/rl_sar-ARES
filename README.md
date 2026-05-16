# rl_sar-ARES

RL policy inference for the ARES quadruped robot. Based on the [rl_sar](https://github.com/fan-ziqi/rl_sar) framework, adapted for ARES hardware (Robstride motors via CAN, WIT IMU via serial).

## Quick Start

```bash
# 1. Download inference runtime (ONNX Runtime)
./download_inference_runtime.sh

# 2. Build
./build.sh

# 3. Run (launches both nodes with default policy: dogv2_cts/cts)
./run.sh

# Or specify a different policy:
./run.sh ares_himloco/himloco

# Or run nodes separately (useful for debugging):
ares_driver_node dogv2_cts/cts   # terminal 1
ares dogv2_cts/cts               # terminal 2
```

Available policies are subdirectories under `policy/` (e.g. `dogv2_cts/cts`, `ares_himloco/himloco`). Each contains `config.yaml` + `policy.onnx`.

Ctrl+C to stop. On shutdown, motors enter **damping mode** (kp=0, kd=10) for safety.

## Architecture

Two ROS2 nodes communicating via standard ROS2 topics:

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
| `fixed_kp` / `fixed_kd` | PD gains, scalar or 12-element list |
| `torque_limits` | Torque limit (Nm), scalar or 12-element list |
| `gamepad_scale` | Gamepad axis scaling |
| `commands_scale` | Velocity command scaling [vx, vy, vyaw] |
| `action_scale` | Action output scaling |
| `default_dof_pos` | Default standing pose |
| `observations` | Observation order |
| `observations_history` | History frame indices |

`fixed_kp`, `fixed_kd`, `torque_limits` accept either a single value (applied to all 12 joints) or a list of 12 values. Modify this file and restart nodes — no recompilation needed.

## Building

### Prerequisites

- ROS2 (Humble or Jazzy, sourced in your shell)
- C++17 compiler, CMake 3.10+, yaml-cpp, TBB
- Python3 with NumPy development headers
- ONNX Runtime (via `download_inference_runtime.sh`)

### Download inference runtime

```bash
./download_inference_runtime.sh          # ONNX Runtime + LibTorch
./download_inference_runtime.sh onnx     # ONNX Runtime only
./download_inference_runtime.sh libtorch # LibTorch only
```

### Build

```bash
./build.sh          # full: cmake configure + build
./build.sh make     # incremental build only
```

Or manually:

```bash
cmake -S driver -B driver/build && cmake --build driver/build
cmake -S src/rl_sar -B src/rl_sar/build && cmake --build src/rl_sar/build
```

### Build Outputs

| Binary | Description |
|--------|-------------|
| `src/rl_sar/build/bin/ares` | RL inference node |
| `src/rl_sar/build/bin/ares_driver_node` | Hardware driver node |
| `driver/libdog_driver.so` | CAN + IMU driver library |

## Running

```bash
./run.sh
```

This launches both nodes in one terminal. Ctrl+C stops both and motors enter damping mode.

Or run separately:

```bash
./src/rl_sar/build/bin/ares_driver_node   # Terminal 1
./src/rl_sar/build/bin/ares                # Terminal 2
```

## Shutdown Behavior

On Ctrl+C, both nodes exit cleanly. The driver node's worker thread joins immediately (non-blocking keyboard input ensures no hang on shutdown). Motors are **not** commanded during shutdown — they retain whatever state the hardware was in.

## State Machine (Keyboard Mode Switching)

`ares_driver_node` now includes a three-mode state machine controllable via keyboard.

### Launch

```bash
./run.sh                    # both nodes
# or separately:
ares_driver_node            # Terminal 1 — keyboard input goes here
```

### Modes

| Key | Command | Description |
|-----|---------|-------------|
| `s` | STAND | Enable motors, slowly stand up to zero pose over 2s, then hold |
| `r` | RL | Run RL policy (normal operation, receives `/motor_command` topic) |
| `d` | DISABLE | Disable all motors (power off) |

### Transition Diagram

```
DISABLE ──s──> STAND ──r──> RL
                ^            │
                │            ├──s──> STAND
                │            └──d──> DISABLE
```

- **DISABLE → STAND**: `s` key — motors enable and stand up
- **STAND → RL**: `r` key — start RL policy control
- **RL → STAND**: `s` key — interpolate back to zero pose
- **RL → DISABLE**: `d` key — shut down motors
- All other transitions are ignored (e.g. DISABLE → RL directly is not allowed)

### Initial Mode

On startup, the system detects:
- If all joints are online and near position 0 → **RL mode** (ready to run)
- Otherwise → **DISABLE mode** (safe default, user must press `s` then `r`)

## Directory Structure

```
rl_sar-ARES/
├── build.sh                          # Build script (full / incremental)
├── run.sh                            # One-click run both nodes
├── download_inference_runtime.sh     # Download ONNX Runtime / LibTorch
├── src/rl_sar/
│   ├── CMakeLists.txt
│   ├── src/
│   │   ├── rl_real_ares.cpp          # RL inference node
│   │   └── ares_driver_node.cpp      # Hardware driver node
│   ├── test/                         # Test sources
│   └── library/core/                 # inference_runtime, observation_buffer, loop, etc.
├── driver/
│   ├── include/dog_driver.hpp
│   ├── src/                          # CAN, serial, IMU, motor control
│   └── libdog_driver.so
├── policy/
│   └── ares_himloco/
│       └── himloco/
│           ├── config.yaml           # Unified config
│           └── policy.onnx
└── library/inference_runtime/        # Downloaded by download_inference_runtime.sh
```

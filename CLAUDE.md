# ARES Project Guide for Claude

## Build & Test

### Remote Build Test (MANDATORY)

Every code change MUST be build-tested on the remote machine before marking complete. Workflow:

1. Push local commits to the dev branch on GitHub (`git push origin dev/<feature>`)
2. Pull on remote, then build:
   ```bash
   ssh wufy@100.66.202.29 "source /opt/ros/jazzy/setup.bash && cd ~/projects/rl_sar-ARES && git fetch origin && git checkout dev/<feature> && git pull origin dev/<feature> && bash build.sh make"
   ```

Key details:
- Remote host: `wufy@100.66.202.29` (hostname `chimi`)
- Remote path: `~/projects/rl_sar-ARES`
- ROS2 must be sourced before build: `source /opt/ros/jazzy/setup.bash`
- Use `bash build.sh make` for incremental build
- Local macOS build will fail (missing `<linux/can.h>` for driver). Always test on remote.

### Source Files (`src/rl_sar/src/`)

| File | Description |
|------|-------------|
| `ares_driver_core.cpp/.hpp` | CAN driver core — joint limit safety, mode transitions (DISABLE/STAND/RL), motor command interpolation, YAML config parsing |
| `ares_driver_node.cpp` | ROS2 node wrapping `AresDriverCore` — joint state/IMU publishers, cmd_vel subscriber, keyboard state switching via `kbhit()` |
| `rl_real_ares.cpp` | RL control node — observation buffer, ONNX inference loop at 200Hz, kp/kd param updates on policy switch |
| `keyboard_helper.cpp/.hpp` | Reusable non-blocking stdin reader — `kbhit()` returns ASCII or -1, termios raw mode with `atexit()` restore |




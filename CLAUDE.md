# ARES Project Guide for Claude

## Build & Test

### Remote Build Test (MANDATORY)

Every code change MUST be build-tested on the remote machine before marking complete:

```bash
ssh wufy@100.66.202.29 "source /opt/ros/jazzy/setup.bash && cd ~/projects/rl_sar-ARES && bash build.sh make"
```

Key details:
- Remote host: `wufy@100.66.202.29` (hostname `chimi`)
- Remote path: `~/projects/rl_sar-ARES`
- ROS2 must be sourced before build: `source /opt/ros/jazzy/setup.bash`
- Use `bash build.sh make` for incremental build
- Local macOS build will fail (missing `<linux/can.h>` for driver). Always test on remote.

### keyboard_helper

A reusable non-blocking stdin reader. Use in any node that needs keyboard input:

- Header: `src/rl_sar/include/keyboard_helper.hpp`
- Source: `src/rl_sar/src/keyboard_helper.cpp`
- API: `int kbhit()` — returns ASCII char or -1 if no key
- Termios raw mode set on first call, restored via `atexit()`
- Add `.cpp` to the target's `add_executable()` or `add_library()` in CMakeLists.txt

### State Machine

`AresDriverCore` has three modes, switched by keyboard:

| Key | Mode | Valid From | Behavior |
|-----|------|-----------|----------|
| `s` | STAND | DISABLE, RL | Enable motors, interpolate to zero pose over 2s, hold |
| `r` | RL | STAND | Forward topic commands at 200Hz (existing behavior) |
| `d` | DISABLE | RL | Disable all motors (power off) |

Transitions:
```
DISABLE ──s──> STAND ──r──> RL
                ^            │
                │            ├──s──> STAND
                │            └──d──> DISABLE
```

Initial state auto-detected: RL if all joints online and near 0, else DISABLE.

### Dev Branch Workflow

- Work on `master`, or create `dev/<feature>` branches
- Push to `origin` (GitHub), then pull + build-test on remote

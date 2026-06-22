# AGENTS.md

## Build Commands

- **编译**: `./build.sh` (incremental, default) 或 `./build.sh full` (clean configure + build)
- **禁止使用 `colcon build`**，本项目使用纯 CMake 构建
- **运行**: `./run.sh` (同时启动 ares_driver_node + ares)
- **每次改完代码后必须运行 `./build.sh`，编译通过才算工作完成**
- build.sh 自动检测并 source ROS2 humble，无需手动 source

## Project Structure

- `src/rl_sar/src/rl_node.cpp` — RL 推理节点 (ares)，纯推理引擎，无状态机
- `src/rl_sar/src/rl_core.cpp` — RL 核心逻辑 (AresRL 类)，ONNX 推理
- `src/rl_sar/src/ares_driver_node.cpp` — 硬件驱动节点 (ares_driver_node)，**唯一状态机 owner**
- `src/rl_sar/src/ares_driver_core.cpp` — 驱动核心逻辑 (AresDriverCore 类)
- `driver/` — DogDriver 硬件抽象层 (CAN + IMU + Gamepad)
- `policy/` — RL 策略配置 + ONNX 模型
- `library/inference_runtime/onnxruntime/` — ONNX Runtime 运行时

## Architecture: State Machine

**状态机只在 driver node 维护**，RL node 是无状态的推理引擎。

### Driver Node (ares_driver_node)
- 状态: DISABLE → STAND → RL (→ DAMPING)
- 发布 `/driver_mode` (UInt8) 广播当前状态
- 游戏手柄: 检测组合键 → 发布 `/remote_command` + 自行切换状态

### RL Node (ares)
- 订阅 `/driver_mode`，收到 RL 时开始推理，其他时跳过
- 发布 `/motor_command` (仅在 RL 模式下)
- 无状态机逻辑，不订阅 `/remote_command`

### Topic 流向
```
Driver ──/driver_mode──→ RL Node
Driver ──/motor_feedback──→ RL Node
Driver ──/imu/data──→ RL Node
Driver ──/xbox_vel──→ RL Node
RL Node ──/motor_command──→ Driver (仅 RL 模式)
Driver ──/remote_command──→ (仅用于 TOGGLE_RECORD 等，未来可移除)
```

## Key Conventions

- 关节顺序: FL RL FR RR (URDF order)，driver 内部按 HipA/HipF/Knee 分组，driver node 负责映射
- `fixed_kp`, `fixed_kd`, `torque_limits` 支持 YAML 标量或 12 元素数组
- IMU 的 `linear_acceleration` 字段传递的是 projected_gravity (非真实加速度)
- 关闭时进入阻尼模式 (kp=0, kd=10)
- `DriverMode` 枚举定义在 `include/driver_mode.hpp`，两个节点共用

## Current Development Status

- 重构完成：状态机从双节点 (driver + RL) 简化为单节点 (driver only)
- RL node 已移除所有状态机逻辑和键盘控制，完全由 `/driver_mode` 驱动
- Driver node 新增 `/driver_mode` topic，直接控制 RL node 的推理启停
- 已移除所有 keyboard 依赖（`keyboard_helper.cpp` 从构建中移除），控制完全由 gamepad 经 topic 传递
- 已清理死代码：UIKA 变体、orphaned FSM、rl_sdk、vector_math、motion_loader、matplotlibcpp、mujoco/robot_sdk
- Motor params 无默认值：Init 时校验 kp/kd/torque 长度，不匹配直接 FATAL 退出
- 传感器未就绪时持续打印警告（每 2 秒），明确指示缺少 IMU 或 motor feedback

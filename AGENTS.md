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
- `policy/dogv2_cts/cts/config.yaml` — **当前开发策略 (CTS) 的配置文件**，包含 gamepad_limits、PD gains、关节映射等
- `library/inference_runtime/onnxruntime/` — ONNX Runtime 运行时

## Architecture: State Machine

**状态机只在 driver node 维护**，RL node 是无状态的推理引擎。

### Driver Node (ares_driver_node)
- 状态: DISABLE → STAND → RL (→ DAMPING)
- 发布 `/driver_mode` (UInt8) 广播当前状态
- 游戏手柄: 检测组合键 → 直接切换状态 + 打印 `[Gamepad]` 日志

### RL Node (ares)
- 订阅 `/driver_mode`，收到 RL 时开始推理，其他时跳过
- 发布 `/motor_command` (仅在 RL 模式下)
- 无状态机逻辑

### Topic 流向
```
Driver ──/driver_mode──→ RL Node
Driver ──/motor_feedback──→ RL Node
Driver ──/imu/data──→ RL Node
Driver ──/xbox_vel──→ RL Node
RL Node ──/motor_command──→ Driver (仅 RL 模式)
```

### 游戏手柄组合键
| 组合键 | 动作 |
|--------|------|
| LB+A | → STAND |
| LT+Y | → RL (需先在 STAND) |
| RB+X | → DISABLE |
| LB+RB | → DAMPING |
| LB+Start | TOGGLE_RECORD |
| LB+Y | SELECT_LOCOMOTION |

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
- 已移除 `/remote_command` topic（driver 发布但无人订阅的遗留产物）
- Motor params 无默认值：Init 时校验 kp/kd/torque 长度，不匹配直接 FATAL 退出
- 传感器未就绪时持续打印警告（每 2 秒），明确指示缺少 IMU 或 motor feedback

### 断线重连机制 (2026-06-22)

**健康检查**：`IsHealthy()` 要求 **IMU 连接 + 全部 12 电机在线**，任何一个不满足就触发重建。

**重建流程**：不重建 DogDriver 对象，而是调用 `ReconnectAll()` 在同一实例上销毁旧 CAN/motor/IMU 并重新初始化。

**重连触发**：`MaintainConnections()` 每秒检查一次，打印明确的组件级日志：
```
[DRIVER] Health check FAILED: IMU=no, Motors=6/12 online. Reconnecting CAN+motors+IMU...
[DRIVER] Reconnected successfully (IMU=yes, Motors=12/12)
[GAMEPAD] Not connected. Retrying at /dev/input/js0...
```

**CAN 保护**：
- CAN 不存在时跳过电机初始化（`CANInterface::IsOpen()` 检查），不浪费时间
- CAN 断电时读线程检测 `ENODEV`/`ENOTCONN` 自动退出，不空转
- `ReconnectAll()` 和 `LogAndReconnectDriverLocked()` 均有 try-catch 异常保护

**Robstride 错误处理**：
- `HandleFault` / `HandleErrorFeedback`：任何错误（不仅是 phase-current）→ `Disable → Clear → Enable`
- Control thread 只更新 online/offline 状态，不做重连

# rl_sar-ARES

ARES 四足机器人 RL 策略推理。基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) 框架，适配 ARES 硬件（Robstride 电机通过 CAN，WIT IMU 通过串口）。

## 快速开始

```bash
# 1. 首次运行：下载 ONNX Runtime
./download_inference_runtime.sh

# 2. 编译
./build.sh

# 3. 运行（默认策略 dogv2_cts/cts）
./run.sh                          # 或 ./run.sh dogv2_cts/cts
```

编译后二进制文件安装到 `~/.local/bin/`。

当前默认接入 `dogv2_cts/cts` 作为 locomotion policy。开机后默认处于 `DISABLE`，通过 LOGIC 手柄进入站立、启动策略、切换 GAIT、停止策略、录制数据等。Ctrl+C 停止。

## systemd 自启

```bash
# 安装（默认策略 dogv2_cts/cts）
sudo ./install_service.sh dogv2_cts/cts

# 启动
sudo systemctl start ares_rl.service

# 查看状态
sudo systemctl status ares_rl.service

# 查看日志
journalctl -u ares_rl.service -f

# 停止
sudo systemctl stop ares_rl.service

# 移除（同时清除 bashrc aliases）
sudo ./install_service.sh --remove
```

安装脚本会自动在 `~/.bashrc` 中添加管理 aliases（带幂等检查，不会重复添加）：

| Alias | 功能 |
|-------|------|
| `ares-start` | 启动服务 |
| `ares-stop` | 停止服务 |
| `ares-restart` | 重启服务 |
| `ares-status` | 查看状态 |
| `ares-logs` | 实时日志 |
| `ares-enable` | 设为开机自启 |
| `ares-disable` | 禁用开机自启 |

安装后执行 `source ~/.bashrc` 或开新终端即可使用 aliases。

服务启动前会自动执行 `~/.local/bin/start`（`ExecStartPre`），用于初始化 CAN 通信等硬件前置操作。

## 遥控指令

当前使用 LOGIC USB 手柄（`/dev/input/js0`）遥控机器人，按键定义参考 `Loco_Intern_SDK`。

### 遥控指令速查

| 组合键 | 功能 | 说明 |
|--------|------|------|
| `LB + A` | 站立 / 恢复站立 | 从任意模式进入 `STAND`；如果当前正在运行策略，则先停策略再回站立 |
| `LB + Y` | 启动 RL 策略 | 仅在 `STAND` 模式下有效，切换到 `RL` 模式运行当前 policy |
| `LB + B` | 启动 GAIT 步态 | 仅在 `STAND` 模式下有效，切换到 `GAIT` 模式运行内置步态控制器 |
| `RB + X` | Disable | 全局失能，直接进入 `DISABLE` |
| `LB + RB` | Damping | 进入阻尼模式（kd=4, kp=0） |
| `LB + START` | 录制数据开关 | 运行中切换 CSV 录制；未运行策略时按下只会提示，不执行危险动作 |

### 推荐操作顺序

```text
1. 上电 / 启动服务后，默认处于 DISABLE
2. 按 LB + A，让机器人进入 STAND
3. 按 LB + Y，启动 RL 策略（或按 LB + B 启动 GAIT 步态）
4. 运行中如需停止并回站立，再按一次 LB + A
5. 如需紧急失能，按 RB + X
```

## 架构

两个 ROS2 节点通过标准 ROS2 topic 通信：

```
ares_driver_node  ──/motor_feedback──>  ares_rl_node
(sensor_msgs/JointState)              (ONNX 推理)

ares_driver_node  ──/imu/data────────>  ares_rl_node
(sensor_msgs/Imu)                     (观测构造)

ares_rl_node     ──/motor_command──>  ares_driver_node
(sensor_msgs/JointState)              (CAN MIT 指令)

ares_driver_node  ──/xbox_vel────────>  ares_rl_node
(geometry_msgs/Twist)                 (手柄速度指令)

ares_driver_node  ──/driver_mode────>  ares_rl_node
(std_msgs/UInt8)                      (模式广播)
```

- **ares_driver_node** — 封装 `libdog_driver.so`。打开 4 路 CAN + WIT IMU。运行状态机（DISABLE/STAND/RL/DAMPING/GAIT）。所有手柄按键在 driver 内部处理。
- **ares_rl_node** — 运行 ONNX 策略推理，带观测历史缓冲区。根据 `/driver_mode` 自动启停推理。

无需自定义 ROS2 消息类型 — 全部使用 `sensor_msgs` 和 `geometry_msgs`。

## Topic 接口

| Topic | 方向 | 消息类型 | 内容 |
|-------|------|----------|------|
| `/motor_feedback` | driver → rl | `sensor_msgs/JointState` | name[12] + position[12] + velocity[12] + effort[12] |
| `/imu/data` | driver → rl | `sensor_msgs/Imu` | angular_velocity + linear_acceleration* |
| `/motor_command` | rl → driver | `sensor_msgs/JointState` | position[12]（目标位置） |
| `/motor_param_update` | rl → driver | `sensor_msgs/JointState` | position=kp, velocity=kd, effort=torque |
| `/xbox_vel` | driver → rl | `geometry_msgs/Twist` | 手柄速度指令 |
| `/driver_mode` | driver → rl | `std_msgs/UInt8` | 当前模式枚举 (DISABLE=0, STAND=1, RL=2, DAMPING=3, GAIT=4) |

*`linear_acceleration` 字段传递的是 projected gravity（非真实加速度）。

## 关节顺序

Driver 和 topic 使用相同顺序 — 按关节类型分组：

| 索引 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 |
|------|---|---|---|---|---|---|---|---|---|---|----|----|
| 关节 | HipA | HipA | HipA | HipA | HipF | HipF | HipF | HipF | Knee | Knee | Knee | Knee |
| 腿 | LF | LR | RF | RR | LF | LR | RF | RR | LF | LR | RF | RR |

RL 模型可通过 `config.yaml` 中的 `topic_to_driver` 指定不同的期望顺序：

```yaml
# topic_to_driver[topic_idx] = driver_idx
topic_to_driver: [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]
```

RL 节点内部使用 `driver_to_topic[]`（逆映射）在模型顺序和 driver 顺序之间转换。

## 配置

每个策略一个 YAML 文件，例如 `policy/dogv2_cts/cts/config.yaml`。

两个节点共用此文件。关键参数：

| 参数 | 说明 |
|------|------|
| `fixed_kp` / `fixed_kd` | PD 增益，标量或 12 元素数组 |
| `torque_limits` | 力矩限制 (Nm)，标量或 12 元素数组 |
| `gamepad_scale` | 手柄轴缩放 |
| `commands_scale` | 速度指令缩放 [vx, vy, vyaw] |
| `action_scale` | 动作输出缩放 |
| `default_dof_pos` | 默认站立姿态 |
| `observations` | 观测顺序 |
| `observations_history` | 历史帧索引 |

`fixed_kp`、`fixed_kd`、`torque_limits` 支持单值（应用到全部 12 个关节）或 12 元素数组。

## 编译

```bash
./build.sh          # 增量编译（默认）
./build.sh full     # 完全编译：cmake 配置 + 构建
```

或手动编译：

```bash
cmake -S driver -B driver/build && cmake --build driver/build
cmake -S src/rl_sar -B src/rl_sar/build && cmake --build src/rl_sar/build
```

### 编译产物

| 二进制文件 | 说明 |
|-----------|------|
| `ares` | RL 推理节点 |
| `ares_driver_node` | 硬件驱动节点 |
| `driver/libdog_driver.so` | CAN + IMU 驱动库 |

## 状态机

`ares_driver_node` 管理五模式状态机。所有手柄按键事件在 driver 内部处理，不通过 topic 传递。

### 模式

| 模式 | 说明 |
|------|------|
| `DISABLE` | 全部电机关闭。安全默认值。 |
| `STAND` | 电机使能，插值到零位姿态（2 秒），然后保持。 |
| `RL` | 正常运行 — 接收 RL 节点的 `/motor_command`。 |
| `DAMPING` | 阻尼模式（kp=0, kd=4），电机使能但无力矩输出。 |
| `GAIT` | 内置步态控制器 — 根据手柄摇杆直接驱动腿部运动学，无需 RL 推理。 |

### 状态转换

```
DISABLE ──> STAND ──> RL
   │          │   │
   │          │   └──> GAIT
   │          └──────> DAMPING
   └──────────────────> DAMPING
```

仅允许以上转换，其他转换会被拒绝。

### 初始模式

- 启动后始终进入 **DISABLE 模式**
- 需要通过手柄 `LB + A` 明确触发站立

### LOGIC 手柄按键

完整遥控指令请直接参考前面的 [遥控指令](#遥控指令) 小节。

这里和状态机相关的要点再强调一次：

- `LB + A`：从任意模式进入 `STAND`，或从运行中的策略回到 `STAND`
- `LB + Y`：仅在 `STAND` 后用于启动 RL 策略
- `LB + B`：仅在 `STAND` 后用于启动 GAIT 步态
- `RB + X`：任意时刻优先执行 `DISABLE`
- `LB + RB`：进入 `DAMPING`

### Driver Node 内部手柄处理

`ares_driver_node` 中的手柄回调直接调用 `AresDriverCore` 内部方法切换模式：

| 按键 | 模式切换 | 限制 |
|------|----------|------|
| `LB + A` | → STAND | 无限制 |
| `LB + Y` | → RL | 仅从 STAND |
| `LB + B` | → GAIT | 仅从 STAND |
| `RB + X` | → DISABLE | 无限制 |
| `LB + RB` | → DAMPING | 无限制 |

### DogDriver API 使用

`AresDriverCore` 调用以下 `DogDriver` 方法（来自 `driver/include/dog_driver.hpp`）：

| DogDriver API | 使用位置 | 用途 |
|---------------|---------|------|
| `DogDriver()` | 构造函数 | 打开 CAN + IMU，使能全部电机，设置默认 MIT 参数 |
| `GetJointStates()` | `GetTopicFeedback()`、站立起始位置、初始模式检测 | 从后台 CAN RX 线程读取最新关节位置/速度/力矩 |
| `GetIMUData()` | `GetImuData()` | 从 IMU 串口线程读取陀螺仪 + projected gravity |
| `SetMITParams(idx, kp, kd)` | 构造函数、`SetMotorParams()`、STAND 转换 | 设置单个关节 PD 增益（通过 CAN 发送） |
| `SetTorqueLimit(idx, torque)` | 构造函数、`SetMotorParams()` | 设置单个关节力矩限制 (Nm) |
| `SetAllJointPositions(pos)` | RL 循环（5ms）、STAND 插值、DISABLE（置零） | 向全部 12 个关节发送 MIT 位置指令 |
| `EnableAll()` | STAND 转换 | 使能全部 12 个电机（MIT 模式） |
| `DisableAll()` | DISABLE 转换 | 关闭全部 12 个电机 |
| `IsIMUConnected()` | 启动信息 | 检查 IMU 是否初始化成功 |

`AresDriverCore` 未使用但可用的 API：`SetJointPosition()`、`EnableJoint()`、`DisableJoint()`、`ClearAllErrors()`、`SetZero()`、`EnableAutoReport()`、`IsJointOnline()`、`IsJointInitialized()`。

## AresDriverCore API

`AresDriverCore`（`src/rl_sar/include/ares_driver_core.hpp`）封装 `DogDriver` 并管理状态机。无 ROS 依赖，可独立使用。

### 构造

```cpp
#include "ares_driver_core.hpp"

AresDriverCore core(POLICY_DIR, "dogv2_cts/cts");
```

读取 `config.yaml`，初始化 DogDriver（CAN + IMU），设置 kp/kd/torque，启动内部命令循环线程。

### 模式控制

```cpp
DriverMode mode = core.GetMode();  // DISABLE、STAND、RL、DAMPING 或 GAIT
```

### 电机指令（RL 模式）

```cpp
std::array<float, 12> target = { /* 按 driver 顺序排列 */ };
core.SetTopicCommand(target);
```

RL 模式下，内部循环每 5ms 调用 `SetAllJointPositions()`。

### 传感器反馈

```cpp
auto feedback = core.GetTopicFeedback();  // JointState {position, velocity, torque}
auto imu = core.GetImuData();             // {angular_velocity[3], projected_gravity[3]}
```

### 手柄

```cpp
auto gp = core.PollGamepad();
// gp.connected, gp.linear_x, gp.linear_y, gp.linear_z, gp.angular_z
```

### 动态参数更新

```cpp
core.SetMotorParams(kp, kd, torque);  // float vector
```

### 配置访问

```cpp
core.config_kp();            // vector<float>，12 元素
core.config_kd();
core.config_torque();
core.gamepad_scale();
core.imu_connected();
core.gamepad_connected();
```

## AresRL API

`AresRL`（`src/rl_sar/include/rl_core.hpp`）处理 ONNX 策略推理。无 ROS 依赖。

### 构造

```cpp
#include "rl_core.hpp"

AresRL rl;
bool ok = rl.Init(POLICY_DIR, "dogv2_cts/cts");
```

### 状态控制

```cpp
AresRL::State state = rl.GetState();   // STOPPED 或 RUNNING
rl.SetState(AresRL::State::RUNNING);
bool ready = rl.IsInitialized();
```

### 运行推理

```cpp
rl.RunModel(imu_gyro, imu_gravity, commands, joint_pos, joint_vel, joint_torque);
// float[3] 用于 IMU/commands，float[12] 用于关节
```

按 RL 循环频率调用（`dt * decimation`，例如 50Hz）。调用后 `GetTargetPositions()` 返回新目标。

### 读取输出

```cpp
const auto& target = rl.GetTargetPositions();  // vector<float>，12 元素
const auto& kp = rl.GetKp();
const auto& kd = rl.GetKd();
const auto& torque = rl.GetTorqueLimits();
const auto& limits = rl.GetPositionLimits();
```

### 坐标重映射

```cpp
const auto& d2t = rl.GetDriverToTopic();  // driver 索引 → topic 索引
int topic_idx = d2t[driver_idx];
```

### 策略启动

RL 节点启动时根据命令行参数初始化指定策略（默认 `dogv2_cts/cts`），通过 `/motor_param_update` 发布电机参数到 driver。

推理启停由 `/driver_mode` 驱动：

- Driver 广播 `RL` 模式 → RL 节点自动开始推理
- Driver 离开 `RL` 模式 → RL 节点自动停止推理

实际运行入口由 LOGIC 手柄触发：

- `LB + Y`：从 STAND 进入 RL（driver 内部处理）

### 录制

```cpp
rl.ToggleRecording();                       // 切换 CSV 录制开关
bool recording = rl.IsRecordEnabled();
std::string path = rl.GetRecordFilepath();
```

### 计时信息

```cpp
float dt = rl.GetDt();                      // 例如 0.005
int dec = rl.GetDecimation();               // 例如 4
double ms = rl.GetInferenceTimeMs();
int count = rl.GetInferenceCount();
```

## 遥控实现说明

所有手柄按键事件由 `ares_driver_node` 内部直接处理，不通过 topic 传递。

### Driver 侧

`ares_driver_node` 读取 LOGIC 手柄，直接在 `OnGamepadUpdate()` 回调中处理模式切换：

- `LB + A` → `STAND`
- `LB + Y` → `RL`（仅从 STAND）
- `LB + B` → `GAIT`（仅从 STAND）
- `RB + X` → `DISABLE`
- `LB + RB` → `DAMPING`
- `LB + START` → TOGGLE_RECORD（发布到 RL 节点）

Driver 通过 `/driver_mode` topic 广播当前模式。

### RL 侧

`ares` 节点订阅 `/driver_mode`，根据模式自动启停推理：

- 收到 `RL` → 启动 ONNX 推理
- 收到其他模式 → 停止推理

RL 节点不再处理任何手柄按键事件。

### 循环结构

`rl_node.cpp` 中两个 `LoopFunc` 线程：

```cpp
// loop_control: dt (5ms) — 读传感器 + 发电机指令
// loop_rl:      dt * decimation (20ms) — ONNX 推理
auto loop_control = LoopFunc("loop_control", rl.GetDt(),
                              std::bind(&AresRLNode::RobotControl, this));
auto loop_rl = LoopFunc("loop_rl", rl.GetDt() * rl.GetDecimation(),
                          std::bind(&AresRLNode::ModelLoop, this));
```

## QuadrupedLegController (QLC)

`QuadrupedLegController` 是一个纯运动学库（`src/rl_sar/include/quadruped_leg_controller.hpp`），提供四足步态生成和逆运动学求解，无 ROS 依赖。

### 功能

| 功能 | 说明 |
|------|------|
| **3-DOF 逆运动学** | 足端位置 → [abad, hip, knee] 关节角 |
| **Trot 步态生成** | 可配置周期、占空比、步幅、抬腿高度 |
| **足端轨迹规划** | 支撑相/摆线相轨迹，自动工作空间限制 |
| **关节角映射** | IK 输出 → 电机角度（符号 + 偏移修正） |

### 腿索引约定

| 索引 | 0 | 1 | 2 | 3 |
|------|---|---|---|---|
| 腿 | LF | RF | LH | RH |

### 使用方式

```cpp
#include "quadruped_leg_controller.hpp"

QuadrupedLegController qlc;
qlc.set_dt(0.02);  // 50Hz

GaitCommand cmd{0.5, 0.0, 0.0};  // vx=0.5 m/s
auto states = qlc.update(cmd);     // 4 条腿的关节角

// IK 求解（无硬件）
Vec3 foot = {0.0, 0.0, -0.3};
Eigen::Vector3d q_ik = qlc.solve_ik(foot);
Eigen::Vector3d q_motor = qlc.to_motor_angles(leg_idx, q_ik);
```

### 集成

- **GAIT 模式**：`ares_driver_core` 内部持有 `QuadrupedLegController`，`LB + B` 从 STAND 进入 GAIT 模式时直接驱动运动学，无需 RL 推理
- **leg_test 工具**：`tools/leg_test/` 可独立编译运行，用于 IK 验证和单腿运动测试

### GAIT 模式参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `period` | 0.8s | 步态周期 |
| `duty_factor` | 0.5 | 支撑相比例 |
| `step_height` | 0.05m | 摆腿抬升高度 |
| `max_stride` | 0.12m | 最大半步幅 |
| `stand_height` | 0.3m | 站立高度 |

### 构建 leg_test

```bash
cmake -S tools/leg_test -B tools/leg_test/build && cmake --build tools/leg_test/build
```

```bash
# IK 验证（无硬件）
./tools/leg_test/build/leg_test ik 0 0 -0.3

# 单腿运动测试
./tools/leg_test/build/leg_test move_foot 0 0 -0.3

# 步态测试（需要电机在线）
./tools/leg_test/build/leg_test gait 0.5 0 0
```

## 目录结构

```
rl_sar-ARES/
├── build.sh                          # 编译脚本
├── download_inference_runtime.sh     # 下载 ONNX Runtime
├── src/rl_sar/
│   ├── include/
│   │   ├── ares_driver_core.hpp      # Driver core API
│   │   ├── rl_core.hpp               # RL 推理 API
│   │   ├── quadruped_leg_controller.hpp  # QLC 运动学库
│   │   └── ...
│   ├── src/
│   │   ├── ares_driver_core.cpp      # 状态机 + 电机控制 + GAIT 模式
│   │   ├── ares_driver_node.cpp      # Driver 的 ROS2 封装
│   │   ├── rl_core.cpp               # ONNX 推理逻辑
│   │   └── rl_node.cpp               # RL 的 ROS2 封装
│   └── library/core/                 # inference_runtime、observation_buffer、loop
├── driver/
│   ├── include/dog_driver.hpp        # CAN + IMU 驱动 API
│   ├── src/
│   └── libdog_driver.so
├── policy/
│   ├── policies.yaml                 # 策略注册表
│   ├── dogv2_cts/cts/
│   │   ├── config.yaml
│   │   └── policy.onnx
│   └── dream_waq/dream_waq/
│       ├── config.yaml
│       └── policy.onnx
├── tools/
│   ├── leg_test/                     # QLC 独立测试工具（IK + 步态）
│   ├── motor_tool/                   # 电机调试工具
│   ├── virtual_gamepad/              # 虚拟手柄
│   └── torque_monitor.py             # 力矩监控
└── library/inference_runtime/        # ONNX Runtime（通过 download_inference_runtime.sh）
```

# rl_sar-ARES

ARES 四足机器人 RL 策略推理。基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) 框架，适配 ARES 硬件（Robstride 电机通过 CAN，WIT IMU 通过串口）。

## 快速开始

```bash
# 1. 首次运行：下载 ONNX Runtime
./download_inference_runtime.sh

# 2. 编译
./build.sh

# 3. 分别在两个终端运行
ares_driver_node          # 终端 1 — 驱动 + 状态机
ares                      # 终端 2 — RL 推理
```

编译后二进制文件安装到 `~/.local/bin/`。

通过键盘（或手柄，见下文）选择策略和模式。Ctrl+C 停止。

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
```

- **ares_driver_node** — 封装 `libdog_driver.so`。打开 4 路 CAN + WIT IMU。运行状态机（DISABLE/STAND/RL）。
- **ares_rl_node** — 运行 ONNX 策略推理，带观测历史缓冲区。

无需自定义 ROS2 消息类型 — 全部使用 `sensor_msgs` 和 `geometry_msgs`。

## Topic 接口

| Topic | 方向 | 消息类型 | 内容 |
|-------|------|----------|------|
| `/motor_feedback` | driver → rl | `sensor_msgs/JointState` | name[12] + position[12] + velocity[12] + effort[12] |
| `/imu/data` | driver → rl | `sensor_msgs/Imu` | angular_velocity + linear_acceleration* |
| `/motor_command` | rl → driver | `sensor_msgs/JointState` | position[12]（目标位置） |
| `/motor_param_update` | rl → driver | `sensor_msgs/JointState` | position=kp, velocity=kd, effort=torque |
| `/xbox_vel` | driver → rl | `geometry_msgs/Twist` | 手柄速度指令 |

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

每个策略一个 YAML 文件，例如 `policy/ares_himloco/himloco/config.yaml`。

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

`ares_driver_node` 管理三模式状态机。

### 模式

| 模式 | 说明 |
|------|------|
| `DISABLE` | 全部电机关闭。安全默认值。 |
| `STAND` | 电机使能，插值到零位姿态（2 秒），然后保持。 |
| `RL` | 正常运行 — 接收 RL 节点的 `/motor_command`。 |

### 状态转换

```
DISABLE ──> STAND ──> RL
             ^          │
             │          ├──> STAND
             │          └──> DISABLE
```

仅允许以上转换，其他转换会被拒绝。

### 初始模式

- 全部关节接近位置 0 → **RL 模式**
- 否则 → **STAND 模式**

### 当前键盘调用的 API

`ares_driver_node` 中的键盘处理调用以下 `AresDriverCore` 方法：

| 输入 | API 调用 |
|------|----------|
| `s`（DISABLE/RL → STAND） | `driver_->EnableAll()`，设置 kp/kd，记录站立起始位置 |
| `r`（STAND → RL） | `stand_active_ = false`（开始接收 `/motor_command` topic） |
| `d`（任意 → DISABLE） | `driver_->DisableAll()` |
| 查询模式 | `core.GetMode()` 返回 `DriverMode` 枚举 |

状态机逻辑在 `AresDriverCore::Impl::handle_key_command()` 和 `transition_allowed()` 中。要用 gamepad 替换键盘，需将这些暴露为公共 API — 见 [扩展 Gamepad](#扩展-gamepad)。

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

AresDriverCore core(POLICY_DIR, "ares_himloco/himloco");
```

读取 `config.yaml`，初始化 DogDriver（CAN + IMU），设置 kp/kd/torque，启动内部命令循环线程。

### 模式控制

```cpp
DriverMode mode = core.GetMode();  // DISABLE、STAND 或 RL
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
bool ok = rl.Init(POLICY_DIR, "ares_himloco/himloco");
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

### 策略切换

策略注册在 `policy/policies.yaml` 中：

```yaml
"1": ares_himloco/himloco
"2": dogv2_cts/cts
"3": dream_waq/dream_waq
```

运行时切换：

```cpp
rl.SetState(AresRL::State::STOPPED);
rl.Init(POLICY_DIR, "dogv2_cts/cts");
// 发布新电机参数到 /motor_param_update ...
rl.SetState(AresRL::State::RUNNING);
```

**注意**：每个策略可能有不同的 kp/kd/torque/action_scale/default_dof_pos。`Init()` 后需通过 `/motor_param_update` 发布新参数。

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

## 扩展 Gamepad

### 替换键盘控制状态机（Driver 侧）

当前键盘内部调用 `handle_key_command()`。用 gamepad 驱动：

1. 给 `AresDriverCore` 添加公共方法：
   ```cpp
   void RequestModeChange(DriverMode target);
   ```
2. 将 `transition_allowed()` + 模式设置逻辑从 `Impl` 移入此方法。
3. gamepad 按键时调用 `core.RequestModeChange(DriverMode::STAND)`。

### 替换键盘切换策略（RL 侧）

当前键盘调用 `Init()` + `SetState()`。用 gamepad 驱动：

1. 加载 `policies.yaml` 获取 key→policy 映射。
2. 按键时：
   ```cpp
   rl.SetState(AresRL::State::STOPPED);
   rl.Init(POLICY_DIR, selected_policy);
   // 发布新电机参数...
   rl.SetState(AresRL::State::RUNNING);
   ```
3. 与 driver 协调 — 切换前确保处于 STAND 模式，然后切回 RL。

### 循环结构

`rl_node.cpp` 中两个 `LoopFunc` 线程：

```cpp
// loop_control: dt (5ms) — 读传感器 + 发电机指令
// loop_rl:      dt * decimation (20ms) — ONNX 推理
auto loop_control = LoopFunc("loop_control", rl.GetDt(),
                              std::bind(&ARSNode::RobotControl, this));
auto loop_rl = LoopFunc("loop_rl", rl.GetDt() * rl.GetDecimation(),
                          std::bind(&ARSNode::ModelLoop, this));
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
│   │   └── ...
│   ├── src/
│   │   ├── ares_driver_core.cpp      # 状态机 + 电机控制
│   │   ├── ares_driver_node.cpp      # Driver 的 ROS2 封装
│   │   ├── rl_core.cpp               # ONNX 推理逻辑
│   │   ├── rl_node.cpp               # RL 的 ROS2 封装
│   │   └── keyboard_helper.cpp       # 非阻塞键盘输入
│   └── library/core/                 # inference_runtime、observation_buffer、loop
├── driver/
│   ├── include/dog_driver.hpp        # CAN + IMU 驱动 API
│   ├── src/
│   └── libdog_driver.so
├── policy/
│   ├── policies.yaml                 # 策略注册表
│   └── ares_himloco/himloco/
│       ├── config.yaml
│       └── policy.onnx
└── library/inference_runtime/        # ONNX Runtime（通过 download_inference_runtime.sh）
```

# ARES Joint Mapping & New Network Configuration Guide

## 1. 三层 Joint 顺序

系统存在三种 joint 顺序，数据流经两层映射转换：

```
RL 网络 (yaml config 顺序)
    ↕  直接透传，无重排
ROS Topic (/motor_feedback, /motor_command) — FL RL FR RR 顺序
    ↕  kTopicToDriver / kDriverToTopic 映射
DogDriver 内部 — LF LR RF RR 按关节类型分组
```

### 1.1 ROS Topic 顺序 (FL RL FR RR / URDF order)

`ares_driver_node.cpp:35-40` 定义了 12 个 joint 名称：

```
Index 0-2:   FL_HipA, FL_HipF, FL_Knee     (前左)
Index 3-5:   RL_HipA, RL_HipF, RL_Knee     (后左)
Index 6-8:   FR_HipA, FR_HipF, FR_Knee     (前右)
Index 9-11:  RR_HipA, RR_HipF, RR_Knee     (后右)
```

这是 `/motor_feedback` 和 `/motor_command` topic 使用的顺序。
**RL 节点 (rl_real_ares.cpp) 直接使用此顺序，与 YAML 配置一一对应。**

### 1.2 DogDriver 内部顺序 (按关节类型分组)

`dog_driver.hpp:167-171` 定义了 driver 的内部索引：

```
Index 0-3:  LF_HipA, LR_HipA, RF_HipA, RR_HipA   (4个 HipA)
Index 4-7:  LF_HipF, LR_HipF, RF_HipF, RR_HipF   (4个 HipF)
Index 8-11: LF_Knee, LR_Knee, RF_Knee, RR_Knee    (4个 Knee)
```

分组原因：`kMotorIds` 按 `leg + joint_type * NUM_LEGS` 编排，物理上同一关节类型的电机共享 CAN 总线拓扑。

> 注意：driver 用 LF/LR/RF/RR (LeftFront...)，topic 用 FL/RL/FR/RR (FrontLeft...)。同一腿，命名惯例不同。

### 1.3 映射表

`ares_driver_node.cpp:62-69`：

```cpp
// topic_index -> driver_index
kTopicToDriver[12] = {0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11};

// driver_index -> topic_index
kDriverToTopic[12] = {0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11};
```

| Topic idx | Joint name | Driver idx |
|-----------|-----------|------------|
| 0         | FL_HipA   | 0          |
| 1         | FL_HipF   | 4          |
| 2         | FL_Knee   | 8          |
| 3         | RL_HipA   | 1          |
| 4         | RL_HipF   | 5          |
| 5         | RL_Knee   | 9          |
| 6         | FR_HipA   | 2          |
| 7         | FR_HipF   | 6          |
| 8         | FR_Knee   | 10         |
| 9         | RR_HipA   | 3          |
| 10        | RR_HipF   | 7          |
| 11        | RR_Knee   | 11         |

### 1.4 数据流详细路径

**Feedback 方向 (硬件 → RL)：**

1. `DogDriver::GetJointStates()` 返回 driver 顺序的 position/velocity
2. `AresDriverNode::FeedbackTimerCallback()` 用 `kTopicToDriver[i]` 重排为 topic 顺序发布到 `/motor_feedback`
3. `ARSNode::MotorFeedbackCallback()` 直接按索引存入 `joint_pos_[i]`、`joint_vel_[i]`，**无重排**

**Command 方向 (RL → 硬件)：**

1. `ARSNode::RobotControl()` 从 `/motor_command` 发布 topic 顺序的 target positions
2. `AresDriverNode::CommandLoop()` 接收后先 clamp，再用 `kDriverToTopic[i]` 重排为 driver 顺序
3. `DogDriver::SetAllJointPositions()` 按 driver 索引发送 MIT 指令

**关键结论：RL 节点与 YAML 之间不存在任何重排。YAML 中 action/default_dof_pos 的顺序就是 ROS Topic 的顺序 (FL RL FR RR)。**

## 2. YAML 配置如何影响系统

配置文件路径：`policy/ares_himloco/himloco/config.yaml`

### 2.1 两个节点读取同一份 YAML

| 参数 | 读取节点 | 用途 |
|------|---------|------|
| `model_name` | RL 节点 | 加载 ONNX 模型文件 |
| `observations` | RL 节点 | 构建 observation 向量的字段和顺序 |
| `observations_history` | RL 节点 | 历史帧索引 (如 [0,1,2,3,4,5] = 6帧) |
| `num_of_dofs` | RL 节点 | action 维度 (12) |
| `dt` / `decimation` | RL 节点 | 控制频率 = 1/dt, 推理频率 = 1/(dt*decimation) |
| `action_scale` (×12) | RL 节点 | `target = action * scale + default_dof_pos` |
| `default_dof_pos` (×12) | RL 节点 | 默认站立姿态，也是 action 的偏移基准 |
| `clip_actions_upper/lower` (×12) | RL 节点 | action 裁剪范围 |
| `commands_scale` (×3) | RL 节点 | 指令缩放 [vx, vy, yaw] |
| `*_scale` 系列 | RL 节点 | observation 各字段缩放因子 |
| `fixed_kp` (×12 or scalar) | **Driver 节点** | MIT PD 控制的 kp |
| `fixed_kd` (×12 or scalar) | **Driver 节点** | MIT PD 控制的 kd |
| `torque_limits` (×12) | **Driver 节点** | 各关节力矩限制 |
| `gamepad_scale` | **Driver 节点** | 手柄速度指令缩放 |

### 2.2 YAML 中 12 维参数的顺序

YAML 中所有 12 维数组 (action_scale, default_dof_pos, clip_actions 等) 使用 **ROS Topic 顺序 (FL RL FR RR)**，即：

```
[FL_HipA, FL_HipF, FL_Knee, RL_HipA, RL_HipF, RL_Knee,
 FR_HipA, FR_HipF, FR_Knee, RR_HipA, RR_HipF, RR_Knee]
```

kp/kd 在 Driver 节点中按索引 0-11 设置给 `driver_->SetMITParams(i, kp, kd)`，而 driver 的 `SetMITParams` 使用 **driver 内部顺序**。这意味着 kp/kd 数组在 YAML 中的顺序实际上也是 driver 顺序（因为 driver 节点按 0-11 索引直接传给 driver）。

> **⚠️ 潜在问题**：kp/kd 的 12 维数组按 driver 顺序理解，但 action_scale/default_dof_pos 按 topic 顺序理解。同一个 YAML 的 12 维数组在不同节点有不同语义。当 fixed_kp 为标量时（当前配置 `fixed_kp: 30`）无影响，但如果写成 12 维数组需注意。

## 3. 换新网络时如何配置和测试

### 3.1 识别新网络的 Action 顺序

训练框架（如 Isaac Gym / Legged Gym）的 action 顺序通常由 URDF 或训练代码定义。常见顺序：

| 框架默认 | 顺序 |
|---------|------|
| Legged Gym (default) | FL, FR, RL, RR 每腿 HipA HipF Knee |
| RSL (default) | FL, FR, RL, RR |
| 本项目当前 | FL, RL, FR, RR |

### 3.2 配置步骤

假设新网络的 action 顺序为 `[FL, FR, RL, RR]`（与当前 `[FL, RL, FR, RR]` 不同），需要：

**Step 1 — 准备文件**

```bash
# 放置新模型
cp new_policy.onnx policy/ares_himloco/himloco/policy.onnx
```

**Step 2 — 修改 config.yaml**

需要根据新网络的 action 顺序，将所有 12 维参数重排到 `[FL, RL, FR, RR]` (topic 顺序)。

```
新网络顺序: [FL_HipA, FL_HipF, FL_Knee, FR_HipA, FR_HipF, FR_Knee,
             RL_HipA, RL_HipF, RL_Knee, RR_HipA, RR_HipF, RR_Knee]
         即: [0,1,2,   6,7,8,   3,4,5,   9,10,11]  (topic 索引)

Topic 顺序: [FL_HipA, FL_HipF, FL_Knee, RL_HipA, RL_HipF, RL_Knee,
             FR_HipA, FR_HipF, FR_Knee, RR_HipA, RR_HipF, RR_Knee]
```

重排映射：新网络 index `[0,1,2,3,4,5,6,7,8,9,10,11]` → topic index `[0,1,2,6,7,8,3,4,5,9,10,11]`

```yaml
# 假设新网络的 default_dof_pos (按 FL FR RL RR 顺序) 为:
# [a0,a1,a2, a3,a4,a5, a6,a7,a8, a9,a10,a11]
# 则 config.yaml 中应写为 (FL RL FR RR):
default_dof_pos: [a0,a1,a2, a6,a7,a8, a3,a4,a5, a9,a10,a11]
```

同理处理 `action_scale`, `clip_actions_upper`, `clip_actions_lower`。

**Step 3 — 确认 observation 结构**

检查新网络的 observation 结构是否匹配 `observations` 字段：

```yaml
# 当前配置:
observations: ["commands", "ang_vel", "gravity_vec", "dof_pos", "dof_vel", "actions"]
```

如果新网络使用了不同的 observation 结构（如多了 `lin_vel`），需要：
1. 修改此列表
2. 确认 `rl_real_ares.cpp` 的 `ComputeObservation` 支持该字段（当前支持 `lin_vel/ang_vel/gravity_vec/commands/dof_pos/dof_vel/actions`）

**Step 4 — 调整缩放参数**

对比训练代码中的缩放因子，确保 `config.yaml` 中的 `*_scale` 参数匹配：

```yaml
ang_vel_scale: 0.25      # 对应训练代码的 obs_scales.ang_vel
dof_pos_scale: 1.0       # 对应 obs_scales.dof_pos
dof_vel_scale: 0.05      # 对应 obs_scales.dof_vel
commands_scale: [-2.0, 2.0, -0.25]  # [vx, vy, yaw] 缩放
```

**Step 5 — 调整 PD 参数**

根据新网络的动作幅度和机器人的响应特性调整：

```yaml
fixed_kp: 30        # 位置增益 (标量或 12 维)
fixed_kd: 1.0       # 速度增益 (标量或 12 维)
torque_limits: [17.0, ...]  # 力矩限制
action_scale: [0.25, ...]   # 动作缩放 (控制实际角度变化幅度)
```

### 3.3 测试流程

**Phase 1 — 架机验证 (必须先做)**

1. 把机器人架起来，四脚悬空
2. 启动 driver 节点：`ros2 run rl_sar ares_driver_node`
3. 观察电机 feedback 是否正常（`ros2 topic echo /motor_feedback`）
4. 启动 RL 节点：`ros2 run rl_sar ares_rl_node`
5. 观察关节运动是否合理：
   - 关节是否在正确方向运动
   - 幅度是否过大/过小
   - 左右腿是否对称

**Phase 2 — 对比验证**

```bash
# 录制 topic 数据进行离线分析
ros2 bag record /motor_feedback /motor_command /imu/data

# 检查 action → target position 映射
ros2 topic echo /motor_command
```

检查要点：
- `default_dof_pos` 是否让机器人保持合理站立姿态
- 推理日志中的 `dof_pos` 是否在合理范围
- `action` 输出值是否在 clip 范围内

**Phase 3 — 落地测试**

1. 从低速指令开始 (`gamepad_scale: 0.3`)
2. 逐步增加速度
3. 注意观察机器人稳定性

### 3.4 常见 Action 顺序映射速查

从训练框架顺序到本项目 Topic 顺序的映射：

| 源顺序 | 映射公式 |
|--------|---------|
| FL FR RL RR (Legged Gym 默认) | 重排索引: `[0,1,2, 6,7,8, 3,4,5, 9,10,11]` |
| FL RL FR RR (本项目当前) | 无需重排 |
| LF RF LH RH (IsaacGym 某些配置) | 同 FL FR RL RR |

**Python 辅助脚本**（在训练环境中使用）：

```python
# 将训练网络的参数从 [FL, FR, RL, RR] 重排为 [FL, RL, FR, RR]
REORDER = [0,1,2, 6,7,8, 3,4,5, 9,10,11]  # source_idx -> target position

def reorder_to_topic(src_12):
    """src: [FL,FR,RL,RR] -> topic: [FL,RL,FR,RR]"""
    dst = [0.0] * 12
    for target_idx, src_idx in enumerate(REORDER):
        dst[target_idx] = src_12[src_idx]
    return dst
```

## 4. 关键文件索引

| 文件 | 关键行 | 内容 |
|------|--------|------|
| `src/rl_sar/src/ares_driver_node.cpp:35-69` | Joint 名称、映射表、位置限制 |
| `src/rl_sar/src/rl_real_ares.cpp:111-261` | YAML 加载、observation 构建 |
| `src/rl_sar/src/rl_real_ares.cpp:404-457` | ComputeObservation — observation 拼接 |
| `src/rl_sar/src/rl_real_ares.cpp:503-512` | ComputeTargetPositions — action → 关节角度 |
| `driver/include/dog_driver.hpp:167-185` | Driver 内部 motor ID、offset、方向 |
| `driver/src/dog_driver.cpp:79-94` | GetJointStates — motor → joint 转换 |
| `driver/src/dog_driver.cpp:114-125` | SetAllJointPositions — joint → motor 转换 |
| `policy/ares_himloco/himloco/config.yaml` | 全局配置 |

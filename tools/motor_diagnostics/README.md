# ARES Motor Diagnostics

集中管理 ARES 电机故障排查工具。所有新实机电机测试优先使用 `motor_diag`，旧的 `tools/motor_tool` 和 `collect_pace_data` 仅保留兼容。

## Safety

- 测试前让机器人悬空或可靠支撑，确认足端不会突然落地。
- 从单关节、低幅值、低 kp/kd 开始，不要直接运行 RL 复现故障。
- 诊断工具不修改底层 Robstride 恢复策略；完整 32-bit 故障帧仍由现有 driver 代码打印到终端。
- `--torque-limit` 只写入现有 `MIT_params.torque_limit` 字段；当前 driver 尚未把它写到 RS02 的 `0x700B limit_torque` 参数。

## Joint Layout

内部诊断顺序与 `DogDriver` 一致：

```text
0:LF_HipA  1:LR_HipA  2:RF_HipA  3:RR_HipA
4:LF_HipF  5:LR_HipF  6:RF_HipF  7:RR_HipF
8:LF_Knee  9:LR_Knee 10:RF_Knee 11:RR_Knee
```

CAN 分配：`can0=LF`，`can1=LR`，`can2=RF`，`can3=RR`。

## Build

从仓库根目录运行：

```bash
./build.sh
```

生成目标在 `driver/build/motor_diag`。`build.sh` 会同时安装主 ROS 节点到 `~/.local/bin`。

## Recommended Workflow

1. 静态状态检查：

```bash
driver/build/motor_diag status
```

2. 记录故障现场：

```bash
driver/build/motor_diag fault-log --joint 8 --duration 20
```

日志写入 `diagnostics_logs/fault_log_*.jsonl`，包含当前可通过现有接口读取的 `error_code`、`pattern`、online 状态、位置、速度和力矩。若需要 32-bit `fault/warning`，同时保存 `motor_diag` 运行终端输出，现有 driver 会打印故障帧解析。

3. 单关节低风险正弦测试：

```bash
driver/build/motor_diag single --joint 8 --amp 0.03 --freq 0.3 --kp 8 --kd 0.4 --duration 10
```

4. 疑似电机和健康电机对照：

```bash
driver/build/motor_diag compare --joint 8 --ref 10 --amp 0.03 --freq 0.3 --kp 8 --kd 0.4 --duration 10
```

5. 单关节 chirp：

```bash
driver/build/motor_diag chirp --joint 8 --amp 0.03 --min-freq 0.1 --max-freq 1.5 --kp 8 --kd 0.4 --duration 20
```

6. 常用安全命令：

```bash
driver/build/motor_diag damping --joint 8
driver/build/motor_diag disable --joint 8
driver/build/motor_diag errors --joint 8
driver/build/motor_diag errors --joint 8 --clear
```

## ROS Online Monitor

RL 或 driver node 已经运行时，可以继续用 ROS topic 监控力矩：

```bash
python3 tools/torque_monitor.py 10
```

它只订阅 `/motor_feedback`，不会直接控制电机。

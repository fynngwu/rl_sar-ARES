# AGENTS.md

## Build Commands

- **编译**: `source /opt/ros/jazzy/setup.bash && ./build.sh` (full) 或 `./build.sh make` (incremental)
- **禁止使用 `colcon build`**，本项目使用纯 CMake 构建
- **运行**: `./run.sh` (同时启动 ares_driver_node + ares)
- **每次改完代码后必须运行 `./build.sh`，编译通过才算工作完成**

## Project Structure

- `src/rl_sar/src/rl_real_ares.cpp` — RL 推理节点 (ares)
- `src/rl_sar/src/ares_driver_node.cpp` — 硬件驱动节点 (ares_driver_node)
- `driver/` — DogDriver 硬件抽象层 (CAN + IMU)
- `policy/ares_himloco/himloco/config.yaml` — 统一配置文件 (RL + driver 共用)
- `library/inference_runtime/onnxruntime/` — 软链接指向 `/usr/local`，由 `download_inference_runtime.sh` 安装

## Key Conventions

- 关节顺序: FL RL FR RR (URDF order)，driver 内部按 HipA/HipF/Knee 分组，driver node 负责映射
- `fixed_kp`, `fixed_kd`, `torque_limits` 支持 YAML 标量或 12 元素数组
- IMU 的 `linear_acceleration` 字段传递的是 projected_gravity (非真实加速度)
- 关闭时进入阻尼模式 (kp=0, kd=10)

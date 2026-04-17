# YAML Config Consolidation Design

## Goal

1. Merge `ares_himloco` two YAML files (`base.yaml` + `himloco/config.yaml`) into a single `config.yaml`
2. Driver node reads kp/kd from YAML (calls `SetMITParams()` at init)
3. Gamepad scale from YAML (replaces hardcoded `0.5f`)

## Scope

Only `ares_himloco` variant. `ares` variant is unchanged.

## Changes

### 1. Merge YAML — `policy/ares_himloco/`

Delete `base.yaml`. Merge all keys into `policy/ares_himloco/config.yaml` under `ares_himloco:` section:

```yaml
ares_himloco:
  # Model
  model_name: policy.onnx
  num_observations: 45
  observations: [commands, ang_vel, gravity_vec, dof_pos, dof_vel, actions]
  observations_history: [0, 1, 2, 3, 4, 5]
  observations_history_priority: time
  clip_obs: 100.0
  clip_actions_lower: [-10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10, -10]
  clip_actions_upper: [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]

  # Control
  dt: 0.005
  decimation: 4
  num_of_dofs: 12
  action_scale: [0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25]
  commands_scale: [2.0, 2.0, 0.25]
  lin_vel_scale: 2.0
  ang_vel_scale: 0.25
  dof_pos_scale: 1.0
  dof_vel_scale: 0.05

  # PD gains (used by driver node)
  fixed_kp: [30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30]
  fixed_kd: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
  torque_limits: [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]

  # Pose
  default_dof_pos: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

  # Driver config (used by ares_driver_node)
  gamepad_scale: 0.5
```

### 2. `rl_real_ares.cpp` — Remove base.yaml loading

- Remove `base_config` loading (`base.yaml` path + `YAML::LoadFile`)
- Remove `GET_PARAM` fallback to `base_config`
- Direct reads from single `robot_config` only
- Config path logic: try `robot_name_/himloco/config.yaml` first, then `robot_name_/config.yaml` (unchanged for `ares`; for `ares_himloco` the `himloco/config.yaml` path still works)
- Remove `joint_mapping_` if still present (it was removed from config but check code)

### 3. `ares_driver_node.cpp` — Read YAML

- Add `#include "yaml-cpp/yaml.h"`
- Accept ROS2 param `config_path` (default: `POLICY_DIR/ares_himloco/config.yaml`)
- At init, load YAML, read `fixed_kp` / `fixed_kd`, call `driver_->SetMITParams(i, kp[i], kd[i])` for each joint
- Read `torque_limits` → `driver_->SetTorqueLimit()`
- Read `gamepad_scale` → replace hardcoded `0.5f` in FeedbackTimerCallback
- Read `gamepad_scale` for both linear.x and angular.z axes

### 4. `CMakeLists.txt` — Link yaml-cpp for driver node

- `ares_driver_node` target: add `${YAML_CPP_LIBRARIES}` to `target_link_libraries`

## Files Changed

| File | Action |
|---|---|
| `policy/ares_himloco/config.yaml` | Rewrite: merge base.yaml into this |
| `policy/ares_himloco/base.yaml` | Delete |
| `src/rl_sar/src/rl_real_ares.cpp` | Remove base.yaml loading, simplify config reads |
| `src/rl_sar/src/ares_driver_node.cpp` | Add YAML loading for kp/kd/torque/gamepad_scale |
| `src/rl_sar/CMakeLists.txt` | yaml-cpp for driver node |

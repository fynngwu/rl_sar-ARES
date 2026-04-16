# FL RL FR RR Joint Ordering Alignment

## Problem

The current system has two joint orderings with an awkward mapping layer:

- **Driver order** (DogDriver internal): `[HipA×4, HipF×4, Knee×4]` grouped by joint type
- **URDF/Policy order**: `[FL×3, RL×3, FR×3, RR×3]` grouped by leg
- A `joint_mapping` array in `base.yaml` bridges them in `rl_real_ares.cpp`

uika's robstride_ros uses per-leg ordering natively in their ROS topics. We want ARES to match this convention.

## Decision

Keep DogDriver internal order unchanged. Move the reordering to `ares_driver_node.cpp` so that `/motor_feedback` and `/motor_command` topics use FL RL FR RR order. This eliminates the `joint_mapping` from the RL node entirely.

## Target Order

```
FL_HipA(0) FL_HipF(1) FL_Knee(2)
RL_HipA(3) RL_HipF(4) RL_Knee(5)
FR_HipA(6) FR_HipF(7) FR_Knee(8)
RR_HipA(9) RR_HipF(10) RR_Knee(11)
```

## Changes

### 1. `ares_driver_node.cpp`

Add mapping arrays:
```cpp
// leg_order[i] = driver_index for topic index i (FL RL FR RR -> driver order)
static constexpr int kLegOrder[12] = {0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11};
// driver_order[i] = topic_index for driver index i (inverse of kLegOrder)
static constexpr int kDriverOrder[12] = {0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11};
```

- **FeedbackTimerCallback**: Read driver states in driver order, reorder to FL RL FR RR before publishing
- **CommandLoop**: Read latest_target_ in FL RL FR RR order, reorder back to driver order before calling SetAllJointPositions
- **kJointNames**: Update to FL RL FR RR ordering
- **kPositionLimits**: Reorder to match FL RL FR RR

### 2. `rl_real_ares.cpp`

- Remove `joint_mapping_` and `driver_to_urdf_` loading from config
- Remove mapping logic in `MotorFeedbackCallback` (topic is already in URDF order)
- Remove mapping logic in `RobotControl` (topic is already in URDF order)
- Remove driver-order `names[]` array in RobotControl

### 3. `base.yaml`

- Remove `joint_mapping: [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11]`
- Remove `joint_names` (no longer needed, or update to match topic order)

### 4. Thread structure — No changes

Current structure already matches uika:
- `wall_timer` 100Hz feedback publishing
- `worker_thread` 200Hz command sending
- `MultiThreadedExecutor`
- first-command gate, overrun detection, diagnostic logging

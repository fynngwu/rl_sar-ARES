# ARES rl_sar Deployment + Testing + README Design

**Date**: 2026-04-16
**Target**: ARES onboard PC at 10.20.127.185
**Policy variant**: ares_himloco (kp=30, kd=1.0)
**Command input**: Qt virtual joystick via ROS2 topic

---

## 1. Deployment

### Transfer to Onboard PC

```bash
# From development machine
rsync -avz --exclude 'build/' --exclude 'install/' --exclude 'log/' \
    rl_sar-ARES/ user@10.20.127.185:~/rl_sar-ARES/
```

### Prerequisites (Onboard PC)

- ROS2 Humble installed
- CAN devices: can0, can1, can2, can3 available
- IMU serial: `/dev/ttyCH341USB0` accessible
- User permissions for CAN and serial devices (udev rules or groups)

---

## 2. Build + Sensor Verification

### Build

```bash
ssh user@10.20.127.185
cd ~/rl_sar-ARES
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Sensor Verification (Before RL)

Run driver node only, verify motor and IMU feedback:

```bash
# Terminal 1
source install/setup.bash
ros2 run rl_sar ares_driver_node

# Terminal 2
ros2 topic echo /motor_feedback

# Terminal 3
ros2 topic echo /imu/data
```

**Expected output:**
- `/motor_feedback`: 12 joint positions/velocities (sensor_msgs/JointState)
- `/imu/data`: Valid quaternion + angular_velocity + linear_acceleration

**Success criteria:**
- All 12 motors report online (positions/velocities updating)
- IMU quaternion is valid (not NaN, magnitude ≈ 1.0)

---

## 3. Qt Virtual Joystick Integration

### Architecture

- **qt_ros2_client.py**: New file in project root
- Uses existing `Joystick` widget from `joystick.py`
- Publishes to ROS2 topic `/xbox_vel` (geometry_msgs/Twist)
- Runs on laptop, connects to onboard PC via ROS2 DDS networking

### Implementation

```python
# qt_ros2_client.py
# - rclpy node with Qt threading integration (MultithreadedExecutor)
# - Joystick widget + yaw slider
# - Publishes (vx, vy, vyaw) to /xbox_vel
# - Optional: subscribe to /motor_feedback and /imu/data for display
```

### ROS2 Network Configuration

Laptop and onboard PC must discover each other via DDS:
- Ensure multicast enabled on network
- Or set `ROS_DOMAIN_ID` same on both machines
- If issues, use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` with config

### Running

```bash
# On laptop
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0  # match onboard PC
python qt_ros2_client.py
```

---

## 4. README Updates

### Changes Required

1. **Fix ares_himloco PD gains**: Update table from `kp=10, kd=0.3` → `kp=30, kd=1.0`

2. **Add Deployment section**:
   - rsync command
   - Prerequisites checklist

3. **Add Sensor Verification section**:
   - Driver-only test procedure
   - Expected outputs

4. **Add Qt Virtual Joystick section**:
   - How to run qt_ros2_client.py
   - ROS2 network setup tips

5. **Add Testing Checklist**:
   - Step-by-step procedure
   - Safety notes (lift robot before first RL run)

---

## 5. Testing Checklist

### Step 1: Sensor Verification
- Run `ares_driver_node` only
- Verify all motors online
- Verify IMU data valid

### Step 2: Motor Position Test (Optional)
- With robot on stand/lifted
- Send position commands via `ros2 topic pub /motor_command`

### Step 3: Full RL System
- Run `ares_driver_node`
- Run `ares_rl_node` with `robot_name:=ares_himloco`
- Run Qt joystick on laptop
- FSM: Passive → GetUp (press A) → RLLocomotion (press RB+DPadUp)
- Test velocity commands via joystick

### Safety Notes
- **Always lift/secure robot before first RL run**
- Test on stand before ground locomotion
- Have emergency stop ready (B button = GetDown)

---

## 6. Files to Create/Modify

| File | Action |
|------|--------|
| `rl_sar-ARES/README.md` | Update PD gains + add deployment/testing sections |
| `qt_ros2_client.py` | Create - Qt joystick ROS2 client |
| `rl_sar-ARES/policy/ares_himloco/himloco/config.yaml` | No change (already correct) |

---

## Summary

- **Deployment**: rsync to 10.20.127.185, colcon build
- **Sensor test**: Driver-only, verify motors + IMU
- **Command input**: Qt virtual joystick → ROS2 /xbox_vel
- **README**: Fix PD gains + comprehensive deployment/testing guide
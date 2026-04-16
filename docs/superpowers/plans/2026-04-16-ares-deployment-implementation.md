# ARES Deployment + Qt ROS2 Client Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deploy rl_sar-ARES to onboard PC, update README with correct PD gains and deployment guide, create Qt ROS2 joystick client for velocity commands.

**Architecture:** rsync deployment to onboard PC, README updates for ares_himloco variant (kp=30 kd=1.0), new Qt-ROS2 client using existing Joystick widget publishing to /xbox_vel topic.

**Tech Stack:** Python3, PySide6, rclpy (ROS2), geometry_msgs

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `rl_sar-ARES/README.md` | Modify | Fix PD gains, add deployment/sensor verification/Qt joystick sections |
| `qt_ros2_client.py` | Create | Qt virtual joystick with ROS2 integration, publishes to /xbox_vel |

---

### Task 1: Update README.md - Fix ares_himloco PD Gains

**Files:**
- Modify: `/home/wufy/projects/my_ares_himloco/rl_sar-ARES/README.md`

- [ ] **Step 1: Read current README to locate PD gains section**

Run: Read file `/home/wufy/projects/my_ares_himloco/rl_sar-ARES/README.md`
Locate the `ares_himloco` variant table showing `PD gains | kp=10, kd=0.3`

- [ ] **Step 2: Edit README to fix PD gains**

Replace:
```markdown
| PD gains | kp=10, kd=0.3 |
```

With:
```markdown
| PD gains | kp=30, kd=1.0 |
```

- [ ] **Step 3: Commit the fix**

```bash
git add rl_sar-ARES/README.md
git commit -m "fix(readme): correct ares_himloco PD gains to kp=30 kd=1.0"
```

---

### Task 2: Update README.md - Add Deployment Section

**Files:**
- Modify: `/home/wufy/projects/my_ares_himloco/rl_sar-ARES/README.md`

- [ ] **Step 1: Add Deployment section after Building section**

Insert after the `## Building` section, before `## Running`:

```markdown
## Deployment

### Transfer to Onboard PC

```bash
# From development machine
rsync -avz --exclude 'build/' --exclude 'install/' --exclude 'log/' \
    rl_sar-ARES/ user@10.20.127.185:~/rl_sar-ARES/
```

### Prerequisites (Onboard PC)

- ROS2 Humble installed (`source /opt/ros/humble/setup.bash`)
- CAN devices: `can0`, `can1`, `can2`, `can3` available
- IMU serial: `/dev/ttyCH341USB0` accessible
- User permissions for CAN and serial devices:
  ```bash
  sudo usermod -aG dialout $USER  # serial access
  ```
- yaml-cpp, TBB installed:
  ```bash
  sudo apt install libyaml-cpp-dev libtbb-dev
  ```
```

- [ ] **Step 2: Commit**

```bash
git add rl_sar-ARES/README.md
git commit -m "docs(readme): add deployment section with rsync and prerequisites"
```

---

### Task 3: Update README.md - Add Sensor Verification Section

**Files:**
- Modify: `/home/wufy/projects/my_ares_himloco/rl_sar-ARES/README.md`

- [ ] **Step 1: Add Sensor Verification section after Deployment**

Insert after `## Deployment`:

```markdown
## Sensor Verification

Test hardware driver before running RL policy.

### Run Driver Node Only

```bash
ssh user@10.20.127.185
cd ~/rl_sar-ARES
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rl_sar ares_driver_node
```

### Verify Motor Feedback

In another terminal:
```bash
ros2 topic echo /motor_feedback
```

Expected output:
- `name`: 12 joint names (lf_hipa, lr_hipa, ..., lf_knee, lr_knee, ...)
- `position`: 12 float values (rad)
- `velocity`: 12 float values (rad/s)

### Verify IMU Data

```bash
ros2 topic echo /imu/data
```

Expected output:
- `orientation`: quaternion (x, y, z, w), magnitude ≈ 1.0
- `angular_velocity`: 3 floats (rad/s, body frame)
- `linear_acceleration`: projected gravity vector (body frame)

### Success Criteria

- All 12 motors report positions/velocities updating
- IMU quaternion is valid (not NaN)
- If motors offline: check CAN device permissions, verify motor power
```

- [ ] **Step 2: Commit**

```bash
git add rl_sar-ARES/README.md
git commit -m "docs(readme): add sensor verification section"
```

---

### Task 4: Update README.md - Add Qt Virtual Joystick Section

**Files:**
- Modify: `/home/wufy/projects/my_ares_himloco/rl_sar-ARES/README.md`

- [ ] **Step 1: Add Qt Joystick section after Sensor Verification**

Insert after `## Sensor Verification`:

```markdown
## Qt Virtual Joystick

Alternative to physical gamepad - use Qt GUI to send velocity commands.

### ROS2 Network Setup

Laptop and onboard PC must discover each other via DDS:

```bash
# On both machines
export ROS_DOMAIN_ID=0  # must match
# If multicast issues:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Run Qt Client

```bash
# On laptop
cd /home/wufy/projects/my_ares_himloco
source /opt/ros/humble/setup.bash
python qt_ros2_client.py
```

### Usage

- Drag joystick circle for `vx` (forward) / `vy` (strafe)
- Slider controls `vyaw` (yaw rate)
- Values auto-center when released
- Press "Send CMD" to publish to `/xbox_vel`

### Topic

Publishes `geometry_msgs/Twist` to `/xbox_vel`:
- `linear.x`: forward velocity (scaled)
- `linear.y`: strafe velocity (scaled)
- `angular.z`: yaw rate (scaled)
```

- [ ] **Step 2: Commit**

```bash
git add rl_sar-ARES/README.md
git commit -m "docs(readme): add Qt virtual joystick section"
```

---

### Task 5: Update README.md - Add Testing Checklist

**Files:**
- Modify: `/home/wufy/projects/my_ares_himloco/rl_sar-ARES/README.md`

- [ ] **Step 1: Add Testing Checklist section after Qt Joystick**

Insert after `## Qt Virtual Joystick`:

```markdown
## Testing Checklist

### Step 1: Sensor Verification
- [ ] Run `ares_driver_node` only
- [ ] Verify all 12 motors online (`ros2 topic echo /motor_feedback`)
- [ ] Verify IMU valid (`ros2 topic echo /imu/data`)

### Step 2: Motor Position Test (Optional - Robot Lifted)
- [ ] Lift robot or place on stand
- [ ] Send test position commands:
  ```bash
  ros2 topic pub /motor_command sensor_msgs/JointState \
    "{name: ['lf_hipa', 'lr_hipa', 'rf_hipa', 'rr_hipa',
             'lf_hipf', 'lr_hipf', 'rf_hipf', 'rr_hipf',
             'lf_knee', 'lr_knee', 'rf_knee', 'rr_knee'],
      position: [0, 0, 0, 0, 0.3, 0.3, -0.3, -0.3, -0.6, -0.6, 0.6, 0.6]}" --once
  ```
- [ ] Verify motors respond

### Step 3: Full RL System
- [ ] Run `ares_driver_node`
- [ ] Run `ares_rl_node`:
  ```bash
  ros2 run rl_sar ares --ros-args -p robot_name:=ares_himloco
  ```
- [ ] Run Qt joystick on laptop
- [ ] FSM transitions:
  - Press `A` (gamepad) or key `0`: Passive → GetUp
  - Press `RB+DPadUp` or key `1`: GetUp → RLLocomotion
  - Press `B` or key `9`: GetDown (safely lower)
- [ ] Test velocity commands via joystick

### Safety Notes
- **Lift/secure robot before first RL run**
- Test on stand before ground locomotion
- Have emergency stop ready (`B` = GetDown)
```

- [ ] **Step 2: Commit**

```bash
git add rl_sar-ARES/README.md
git commit -m "docs(readme): add testing checklist with safety notes"
```

---

### Task 6: Create qt_ros2_client.py

**Files:**
- Create: `/home/wufy/projects/my_ares_himloco/qt_ros2_client.py`

- [ ] **Step 1: Write the Qt ROS2 client implementation**

```python
#!/usr/bin/env python3
"""Qt Virtual Joystick with ROS2 integration for ARES robot control."""
import sys
import threading
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox,
)
from PySide6.QtCore import Qt, Signal, QObject

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist

# Import existing Joystick widget
from joystick import Joystick

MAX_VX = 1.2
MAX_VY = 0.5
MAX_YAW = 0.5


class Ros2Bridge(QObject):
    """Marshals ROS2 callbacks into Qt signals."""
    connected = Signal()


class VelocityPublisher(Node):
    """ROS2 node publishing velocity commands to /xbox_vel."""
    
    def __init__(self):
        super().__init__('qt_velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/xbox_vel', 10)
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_yaw = 0.0
    
    def publish_velocity(self, vx: float, vy: float, yaw: float):
        msg = Twist()
        msg.linear.x = vx * MAX_VX
        msg.linear.y = vy * MAX_VY
        msg.angular.z = yaw * MAX_YAW
        self.publisher.publish(msg)
        self.get_logger().debug(f'Published: vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} yaw={msg.angular.z:.2f}')


class MainWindow(QMainWindow):
    def __init__(self, ros2_node: VelocityPublisher):
        super().__init__()
        self.setWindowTitle("ARES Qt Joystick (ROS2)")
        self.setMinimumSize(400, 350)
        self.ros2_node = ros2_node
        self._setup_ui()
        self._setup_connections()
    
    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # Joystick group
        joy_group = QGroupBox("Velocity Joystick")
        joy_layout = QVBoxLayout()
        
        self.joystick = Joystick()
        joy_layout.addWidget(self.joystick, alignment=Qt.AlignmentFlag.AlignCenter)
        
        self.joy_label = QLabel("vx=0.00  vy=0.00")
        self.joy_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        joy_layout.addWidget(self.joy_label)
        
        # Yaw slider
        yaw_row = QHBoxLayout()
        yaw_row.addWidget(QLabel("Yaw rate:"))
        self.yaw_slider = QSlider(Qt.Orientation.Horizontal)
        self.yaw_slider.setRange(-100, 100)
        self.yaw_slider.setValue(0)
        self.yaw_label = QLabel("0.00")
        yaw_row.addWidget(self.yaw_slider)
        yaw_row.addWidget(self.yaw_label)
        joy_layout.addLayout(yaw_row)
        
        # Send button
        self.send_btn = QPushButton("Send Velocity Command")
        joy_layout.addWidget(self.send_btn)
        
        # Status label
        self.status_label = QLabel("Ready - connected to ROS2 /xbox_vel")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        joy_layout.addWidget(self.status_label)
        
        joy_group.setLayout(joy_layout)
        layout.addWidget(joy_group)
    
    def _setup_connections(self):
        self.joystick.value_changed.connect(self._on_joystick_changed)
        self.yaw_slider.valueChanged.connect(self._on_yaw_changed)
        self.send_btn.clicked.connect(self._send_velocity)
    
    def _on_joystick_changed(self, vx: float, vy: float):
        self.ros2_node.current_vx = vx
        self.ros2_node.current_vy = vy
        self.joy_label.setText(f"vx={vx:.2f}  vy={vy:.2f}")
    
    def _on_yaw_changed(self, value: int):
        yaw = value / 100.0
        self.ros2_node.current_yaw = yaw
        self.yaw_label.setText(f"{yaw:.2f}")
    
    def _send_velocity(self):
        vx = self.ros2_node.current_vx
        vy = self.ros2_node.current_vy
        yaw = self.ros2_node.current_yaw
        self.ros2_node.publish_velocity(vx, vy, yaw)
        self.status_label.setText(f"Sent: vx={vx*MAX_VX:.2f} vy={vy*MAX_VY:.2f} yaw={yaw*MAX_YAW:.2f}")


def main():
    # Initialize ROS2
    rclpy.init()
    ros2_node = VelocityPublisher()
    
    # Create executor for background spinning
    executor = MultiThreadedExecutor()
    executor.add_node(ros2_node)
    
    # Start ROS2 spinning in background thread
    ros2_thread = threading.Thread(target=executor.spin, daemon=True)
    ros2_thread.start()
    
    # Create Qt application
    app = QApplication(sys.argv)
    window = MainWindow(ros2_node)
    window.show()
    
    # Run Qt event loop
    ret = app.exec()
    
    # Cleanup
    executor.shutdown()
    ros2_node.destroy_node()
    rclpy.shutdown()
    
    return ret


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: Verify Python syntax**

Run: `python -m py_compile qt_ros2_client.py`
Expected: No errors

- [ ] **Step 3: Add pyproject.toml dependency for PySide6 (if needed)**

Check if PySide6 already in dependencies:
```bash
grep -q "PySide6" pyproject.toml && echo "already present" || echo "need to add"
```

If not present, add:
```bash
uv add PySide6
```

- [ ] **Step 4: Commit**

```bash
git add qt_ros2_client.py pyproject.toml uv.lock
git commit -m "feat: add qt_ros2_client.py for virtual joystick ROS2 control"
```

---

## Self-Review Checklist

- **Spec coverage**: All sections covered (PD fix, deployment, sensor verify, Qt joystick, testing checklist)
- **Placeholders**: None - all steps have actual code/commands
- **Type consistency**: ROS2 types consistent (geometry_msgs/Twist), Joystick widget reused correctly

---

## Execution Options

After saving plan, choose execution approach:
1. **Subagent-Driven (recommended)** - Fresh subagent per task, review between tasks
2. **Inline Execution** - Execute in this session with checkpoints
# ROS package for RobStride motor control
This routine was reposted by RobStride Dynamics from DR.MuShibo. Sincere gratitude goes to DR.MuShibo for their development and sharing.

### USB2CAN Hardware:Canable
- canable (cantact clone): http://canable.io/ (STM32F042C6)
- 灵足的串口转CAN模块只适用于灵足的上位机，Ubuntu上使用需要额外的canable模块。

## Dependency:
- 注意自己的ros2版本号，自行修改
```shell
sudo apt-get install net-tools
sudo apt-get install can-utils
sudo apt-get install ros-humble-can-msgs
sudo apt-get install ros-humble-socketcan-bridge
```

### Ubuntu
```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 1000000 

sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100
```

### Launch the launch file for the demo
- 在工作空间中运行如下命令:
```shell
chmod +x ./setup_can.sh
colcon build
source ./install/setup.bash
ros2 run rs_motor_ros2 rs_motor_ros2
```
/home/nvidia/UIKA/Quadruped_Uika/setup_can.sh
## 编译注意事项

### 重要：清理旧版本
如果遇到节点运行但话题不显示的问题，可能是使用了旧版本二进制文件。

**编译前先删除旧版本：**
```shell
rm -rf ~/VMC_Quadruped/src/robstride_ros_sample/install
colcon build --packages-select rs_motor_ros2
```

**验证编译结果：**
```shell
# 检查二进制文件修改时间（应该是最新编译的时间）
ls -la ~/VMC_Quadruped/install/rs_motor_ros2/lib/rs_motor_ros2/rs_motor_ros2

# 检查进程中实际运行的路径
pgrep -f rs_motor_ros2 | while read pid; do
  echo "PID $pid:"
  cat /proc/$pid/cmdline | tr '\0' ' '
  echo
done
```

**确保使用的是 `~/VMC_Quadruped/install/` 下的版本，而非 `~/VMC_Quadruped/src/robstride_ros_sample/install/`**

## 话题说明

### 发布的话题
- `/motor_feedback` - 电机反馈数据（10ms周期）
  - 类型：`interfaces::msg::MotorFeedback12`
  - 内容：12个电机的扭矩、位置、速度、温度

### 订阅的话题
- `/motor_command` - 电机控制命令
  - 类型：`interfaces::msg/MotorCommand12`
  - 内容：12个电机的扭矩、位置、速度、kp、kd
  - **注意：收到第一个命令前电机不会转动**

### 测试命令
```shell
# 查看话题
ros2 topic list

# 查看电机反馈数据
ros2 topic echo /motor_feedback

# 发布控制命令（12个电机全0）
ros2 topic pub /motor_command interfaces/msg/MotorCommand12 "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'cmd'}, fl_hip: {torque: 0.0, position: -0.7, velocity: 0.0, kp: 30.0, kd: 10}, fl_thigh: {torque: 0.0, position: 0.2, velocity: 0.0, kp: 30.0, kd: 10}, fl_calf: {torque: 0.0, position: 0.7, velocity: 0.0, kp: 35.0, kd: 10}, fr_hip: {torque: 0.0, position: 0.7, velocity: 0.0, kp: 30.0, kd: 10}, fr_thigh: {torque: 0.0, position: -0.2, velocity: 0.0, kp: 30.0, kd: 10}, fr_calf: {torque: 0.0, position: -0.7, velocity: 0.0, kp: 35.0, kd: 10}, rl_hip: {torque: 0.0, position: 0.7, velocity: 0.0, kp: 30.0, kd: 10}, rl_thigh: {torque: 0.0, position: 0.2, velocity: 0.0, kp: 30.0, kd: 10}, rl_calf: {torque: 0.0, position: 0.7, velocity: 0.0, kp: 35.0, kd: 10}, rr_hip: {torque: 0.0, position: -0.7, velocity: 0.0, kp: 30.0, kd: 10}, rr_thigh: {torque: 0.0, position: -0.2, velocity: 0.0, kp: 30.0, kd: 10}, rr_calf: {torque: 0.0, position: -0.7, velocity: 0.0, kp: 35.0, kd: 10}}" --once
```
ros2 topic pub /motor_command interfaces/msg/MotorCommand12 \
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'cmd'
fl_hip:
  torque: 0.0
  position: -0.80
  velocity: 0.0
  kp: 15.0
  kd: 8.0
fl_thigh:
  torque: 0.0
  position: 0.05
  velocity: 0.0
  kp: 15.0
  kd: 8.0
fl_calf:
  torque: 0.0
  position: 0.70
  velocity: 0.0
  kp: 15.0
  kd: 8.0
fr_hip:
  torque: 0.0
  position: 0.80
  velocity: 0.0
  kp: 15.0
  kd: 8.0
fr_thigh:
  torque: 0.0
  position: -0.05
  velocity: 0.0
  kp: 15.0
  kd: 8.0
fr_calf:
  torque: 0.0
  position: -0.70
  velocity: 0.0
  kp: 15.0
  kd: 8.0
rl_hip:
  torque: 0.0
  position: 0.75
  velocity: 0.0
  kp: 15.0
  kd: 8.0
rl_thigh:
  torque: 0.0
  position: 0.05
  velocity: 0.0
  kp: 15.0
  kd: 8.0
rl_calf:
  torque: 0.0
  position: 0.70
  velocity: 0.0
  kp: 15.0
  kd: 8.0
rr_hip:
  torque: 0.0
  position: -0.75
  velocity: 0.0
  kp: 15.0
  kd: 8.0
rr_thigh:
  torque: 0.0
  position: -0.05
  velocity: 0.0
  kp: 15.0
  kd: 8.0
rr_calf:
  torque: 0.0
  position: -0.70
  velocity: 0.0
  kp: 15.0
  kd: 8.0" \
--once
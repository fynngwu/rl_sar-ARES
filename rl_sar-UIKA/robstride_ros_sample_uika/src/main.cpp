#include "motor_ros2/motor_cfg.h"
#include "interfaces/msg/motor_command12.hpp"
#include "interfaces/msg/motor_feedback12.hpp"
#include "stdint.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <unistd.h>
#include <vector>

// 电机配置: CAN通道, 主站ID, 电机ID, 电机类型
struct MotorConfig {
  std::string can_iface;
  uint8_t master_id;
  uint8_t motor_id;
  int actuator_type;
};

// 12个电机配置: CAN0-3各3个电机, 全部使用ROBSTRIDE_02类型
static const std::vector<MotorConfig> MOTOR_CONFIGS = {
    // CAN1
    {"can1", 0xFF, 1,  2},
    {"can1", 0xFF, 2,  2},
    {"can1", 0xFF, 3,  2},
    // CAN4
    {"can4", 0xFF, 10,  2},
    {"can4", 0xFF, 11,  2},
    {"can4", 0xFF, 12,  2},
    // CAN2
    {"can2", 0xFF, 4,  2},
    {"can2", 0xFF, 5,  2},
    {"can2", 0xFF, 6,  2},
    // CAN3
    {"can3", 0xFF, 7, 2},
    {"can3", 0xFF, 8, 2},
    {"can3", 0xFF, 9, 2},
};

// ========================== 电子限位配置 ==========================
// 减速比 电机:关节 = 28:15
const float GEAR_RATIO_CALF = 28.0f / 15.0f;

// 顺序 0~11：
// fl_hip, fl_thigh, fl_calf,
// fr_hip, fr_thigh, fr_calf,
// rl_hip, rl_thigh, rl_calf,
// rr_hip, rr_thigh, rr_calf

// 👉 所有小腿限位 = 图片关节值 × 减速比 1.87
static const std::vector<std::vector<float>> MOTOR_LIMITS = {
    // 0 左前髋
    {-1.50f,  0.00f,  0.1f},
    // 1 左前大腿
    {-0.79f,  0.79f,  0.1f},
    // 2 左前小腿 
    {0.187f,  2.07f,  0.15f},

    // 3 右前髋
    {0.00f,  1.50f,  0.1f},
    // 4 右前大腿
    {-0.79f,  0.79f,  0.1f},
    // 5 右前小腿 
    {-2.07f, -0.187f, 0.15f},

    // 6 左后髋
    {0.00f,  1.50f,  0.1f},
    // 7 左后大腿
    {-0.79f,  0.79f,  0.1f},
    // 8 左后小腿 
    {0.187f,  2.07f,  0.15f},

    // 9 右后髋
    {-1.50f,  0.00f,  0.1f},
    // 10 右后大腿
    {-0.79f,  0.79f,  0.1f},
    // 11 右后小腿 
    {-2.07f, -0.187f, 0.15f},
};

const float KD_MULTIPLIER = 3.0f;
const auto COMMAND_PERIOD = std::chrono::milliseconds(5);
const auto FEEDBACK_PERIOD = std::chrono::milliseconds(10);
const auto DIAG_LOG_PERIOD = std::chrono::seconds(1);
// ==================================================================


class MotorControlSample : public rclcpp::Node {
public:
  MotorControlSample()
      : rclcpp::Node("motor_control_set_node"),
        received_first_command_(false) {

    // 初始化12个电机
    motors_.reserve(MOTOR_CONFIGS.size());
    motor_initialized_.assign(MOTOR_CONFIGS.size(), false);
    for (const auto &cfg : MOTOR_CONFIGS) {
      motors_.push_back(std::make_unique<RobStrideMotor>(
          cfg.can_iface, cfg.master_id, cfg.motor_id, cfg.actuator_type));
    }

    // 依次初始化每个电机，跳过失败的
    for (size_t i = 0; i < motors_.size(); ++i) {
      if (!rclcpp::ok() || !running_) {
        break;
      }
      try {
        std::lock_guard<std::mutex> lock(motors_mutex_);
        motors_[i]->Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        motors_[i]->enable_motor();
        usleep(1000);
        motor_initialized_[i] = true;
        RCLCPP_INFO(this->get_logger(), "Motor %zu initialized on %s", i,
                    MOTOR_CONFIGS[i].can_iface.c_str());
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Motor %zu init failed: %s", i,
                    e.what());
      }
    }

    // 创建电机反馈发布者
    feedback_pub_ = this->create_publisher<interfaces::msg::MotorFeedback12>("/motor_feedback", 10);

    // 创建定时器，每10ms发布一次反馈
    feedback_timer_ = this->create_wall_timer(
        FEEDBACK_PERIOD,
        [this]() {
          this->publish_feedback();
        });

    RCLCPP_INFO(this->get_logger(), "Motor feedback publisher started on /motor_feedback");

    // 创建电机命令订阅
    command_sub_ = this->create_subscription<interfaces::msg::MotorCommand12>(
        "/motor_command", 10,
        [this](const interfaces::msg::MotorCommand12::SharedPtr msg) {
          this->command_callback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "Subscribed to /motor_command, waiting for commands...");

    worker_thread_ = std::thread(&MotorControlSample::excute_loop, this);
  }

  ~MotorControlSample() {
    running_ = false;
    if (worker_thread_.joinable())
      worker_thread_.join();
    std::lock_guard<std::mutex> lock(motors_mutex_);
    for (size_t i = 0; i < motors_.size(); ++i) {
      if (!motor_initialized_[i]) {
        continue;
      }
      try {
        motors_[i]->Disenable_Motor(0);
      } catch (const std::exception &) {
      }
    }
  }

private:
  // 限位保护函数
  void apply_motor_limit(int motor_idx, float &pos, float &torque, float &kp, float &kd) {
    float min_pos = MOTOR_LIMITS[motor_idx][0];
    float max_pos = MOTOR_LIMITS[motor_idx][1];
    float margin = MOTOR_LIMITS[motor_idx][2];

    // // 超限位 → 直接停机保护
    // if (pos < min_pos || pos > max_pos) {
    //   torque = 0.0f;
    //   kp = 0.0f;
    //   kd = 0.0f;
    //   return;
    // }

    // // 接近限位 → 增大阻尼减速
    // if (pos < min_pos + margin || pos > max_pos - margin) {
    //   kd *= KD_MULTIPLIER;
    // }

    // ====================== 力矩限制 ±6Nm（仅添加这里）======================
    if (torque > 6.0f)
        torque = 6.0f;
    if (torque < -6.0f)
        torque = -6.0f;
  }

  void command_callback(const interfaces::msg::MotorCommand12::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_ = *msg;
    received_first_command_ = true;
    RCLCPP_INFO_ONCE(this->get_logger(), "First command received! Starting motor control.");
  }

  void publish_feedback() {
    auto msg = interfaces::msg::MotorFeedback12();
    msg.header.stamp = this->now();
    msg.header.frame_id = "motor_feedback";

    {
      std::lock_guard<std::mutex> lock(motors_mutex_);
      if (motor_initialized_[0]) {
        msg.fl_hip.torque = -motors_[0]->torque_;
        msg.fl_hip.position = -motors_[0]->position_;
        msg.fl_hip.velocity = -motors_[0]->velocity_;
        msg.fl_hip.temperature = motors_[0]->temperature_;
      }

      if (motor_initialized_[1]) {
        msg.fl_thigh.torque = -motors_[1]->torque_;
        msg.fl_thigh.position = -motors_[1]->position_;
        msg.fl_thigh.velocity = -motors_[1]->velocity_;
        msg.fl_thigh.temperature = motors_[1]->temperature_;
      }

      if (motor_initialized_[2]) {
        msg.fl_calf.torque = motors_[2]->torque_ / GEAR_RATIO_CALF;
        msg.fl_calf.position = motors_[2]->position_ / GEAR_RATIO_CALF;
        msg.fl_calf.velocity = motors_[2]->velocity_ / GEAR_RATIO_CALF;
        msg.fl_calf.temperature = motors_[2]->temperature_;
      }

      if (motor_initialized_[3]) {
        msg.fr_hip.torque = -motors_[3]->torque_;
        msg.fr_hip.position = -motors_[3]->position_;
        msg.fr_hip.velocity = -motors_[3]->velocity_;
        msg.fr_hip.temperature = motors_[3]->temperature_;
      }

      if (motor_initialized_[4]) {
        msg.fr_thigh.torque = -motors_[4]->torque_;
        msg.fr_thigh.position = -motors_[4]->position_;
        msg.fr_thigh.velocity = -motors_[4]->velocity_;
        msg.fr_thigh.temperature = motors_[4]->temperature_;
      }

      if (motor_initialized_[5]) {
        msg.fr_calf.torque = motors_[5]->torque_ / GEAR_RATIO_CALF;
        msg.fr_calf.position = motors_[5]->position_ / GEAR_RATIO_CALF;
        msg.fr_calf.velocity = motors_[5]->velocity_ / GEAR_RATIO_CALF;
        msg.fr_calf.temperature = motors_[5]->temperature_;
      }

      if (motor_initialized_[6]) {
        msg.rl_hip.torque = -motors_[6]->torque_;
        msg.rl_hip.position = -motors_[6]->position_;
        msg.rl_hip.velocity = -motors_[6]->velocity_;
        msg.rl_hip.temperature = motors_[6]->temperature_;
      }

      if (motor_initialized_[7]) {
        msg.rl_thigh.torque = -motors_[7]->torque_;
        msg.rl_thigh.position = -motors_[7]->position_;
        msg.rl_thigh.velocity = -motors_[7]->velocity_;
        msg.rl_thigh.temperature = motors_[7]->temperature_;
      }

      if (motor_initialized_[8]) {
        msg.rl_calf.torque = motors_[8]->torque_ / GEAR_RATIO_CALF;
        msg.rl_calf.position = motors_[8]->position_ / GEAR_RATIO_CALF;
        msg.rl_calf.velocity = motors_[8]->velocity_ / GEAR_RATIO_CALF;
        msg.rl_calf.temperature = motors_[8]->temperature_;
      }

      if (motor_initialized_[9]) {
        msg.rr_hip.torque = -motors_[9]->torque_;
        msg.rr_hip.position = -motors_[9]->position_;
        msg.rr_hip.velocity = -motors_[9]->velocity_;
        msg.rr_hip.temperature = motors_[9]->temperature_;
      }

      if (motor_initialized_[10]) {
        msg.rr_thigh.torque = -motors_[10]->torque_;
        msg.rr_thigh.position = -motors_[10]->position_;
        msg.rr_thigh.velocity = -motors_[10]->velocity_;
        msg.rr_thigh.temperature = motors_[10]->temperature_;
      }

      if (motor_initialized_[11]) {
        msg.rr_calf.torque = motors_[11]->torque_ / GEAR_RATIO_CALF;
        msg.rr_calf.position = motors_[11]->position_ / GEAR_RATIO_CALF;
        msg.rr_calf.velocity = motors_[11]->velocity_ / GEAR_RATIO_CALF;
        msg.rr_calf.temperature = motors_[11]->temperature_;
      }
    }

    feedback_pub_->publish(msg);
  }

  void excute_loop() {
    // 等待接收第一个命令
    int wait_count = 0;
    while (rclcpp::ok() && running_ && !received_first_command_) {
      wait_count++;
      if (wait_count % 1000 == 0) {
        RCLCPP_INFO(this->get_logger(), "Waiting for first command...");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!rclcpp::ok() || !running_) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Start sending motion commands");
    auto next_tick = std::chrono::steady_clock::now();
    auto last_diag_log = next_tick;
    size_t send_loops = 0;
    auto max_send_duration = std::chrono::steady_clock::duration::zero();

    while (rclcpp::ok() && running_) {
      interfaces::msg::MotorCommand12 cmd;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        cmd = latest_command_;
      }

      auto send_start = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(motors_mutex_);

      // 0 左前髋
      float t0 = cmd.fl_hip.torque;
      float p0 = -cmd.fl_hip.position;  //加负号
      float kp0 = cmd.fl_hip.kp;
      float kd0 = cmd.fl_hip.kd;
      apply_motor_limit(0, p0, t0, kp0, kd0);
      if (motor_initialized_[0]) motors_[0]->send_motion_command(t0, p0, 0, kp0, kd0);  // 速度加负号

      // 1 左前大腿
      float t1 = cmd.fl_thigh.torque;
      float p1 = -cmd.fl_thigh.position;  // 加负号
      float kp1 = cmd.fl_thigh.kp;
      float kd1 = cmd.fl_thigh.kd;
      apply_motor_limit(1, p1, t1, kp1, kd1);
      if (motor_initialized_[1]) motors_[1]->send_motion_command(t1, p1, 0, kp1, kd1);

      // 2 左前小腿
      float t2 = cmd.fl_calf.torque;
      float p2 = cmd.fl_calf.position * GEAR_RATIO_CALF;
      float kp2 = cmd.fl_calf.kp;
      float kd2 = cmd.fl_calf.kd;
      apply_motor_limit(2, p2, t2, kp2, kd2);
      if (motor_initialized_[2]) motors_[2]->send_motion_command(t2, p2, 0, kp2, kd2);

      // 3 右前髋
      float t3 = cmd.fr_hip.torque;
      float p3 = -cmd.fr_hip.position;
      float kp3 = cmd.fr_hip.kp;
      float kd3 = cmd.fr_hip.kd;
      apply_motor_limit(3, p3, t3, kp3, kd3);
      if (motor_initialized_[3]) motors_[3]->send_motion_command(t3, p3, 0, kp3, kd3);

      // 4 右前大腿
      float t4 = cmd.fr_thigh.torque;
      float p4 = -cmd.fr_thigh.position;
      float kp4 = cmd.fr_thigh.kp;
      float kd4 = cmd.fr_thigh.kd;
      apply_motor_limit(4, p4, t4, kp4, kd4);
      if (motor_initialized_[4]) motors_[4]->send_motion_command(t4, p4, 0, kp4, kd4);

      // 5 右前小腿
      float t5 = cmd.fr_calf.torque;
      float p5 = cmd.fr_calf.position * GEAR_RATIO_CALF;
      float kp5 = cmd.fr_calf.kp;
      float kd5 = cmd.fr_calf.kd;
      apply_motor_limit(5, p5, t5, kp5, kd5);
      if (motor_initialized_[5]) motors_[5]->send_motion_command(t5, p5, 0, kp5, kd5);

      // 6 左后髋
      float t6 = cmd.rl_hip.torque;
      float p6 = -cmd.rl_hip.position;
      float kp6 = cmd.rl_hip.kp;
      float kd6 = cmd.rl_hip.kd;
      apply_motor_limit(6, p6, t6, kp6, kd6);
      if (motor_initialized_[6]) motors_[6]->send_motion_command(t6, p6, 0, kp6, kd6);

      // 7 左后大腿
      float t7 = cmd.rl_thigh.torque;
      float p7 = -cmd.rl_thigh.position;
      float kp7 = cmd.rl_thigh.kp;
      float kd7 = cmd.rl_thigh.kd;
      apply_motor_limit(7, p7, t7, kp7, kd7);
      if (motor_initialized_[7]) motors_[7]->send_motion_command(t7, p7, 0, kp7, kd7);

      // 8 左后小腿
      float t8 = cmd.rl_calf.torque;
      float p8 = cmd.rl_calf.position * GEAR_RATIO_CALF;
      float kp8 = cmd.rl_calf.kp;
      float kd8 = cmd.rl_calf.kd;
      apply_motor_limit(8, p8, t8, kp8, kd8);
      if (motor_initialized_[8]) motors_[8]->send_motion_command(t8, p8, 0, kp8, kd8);

      // 9 右后髋
      float t9 = cmd.rr_hip.torque;
      float p9 = -cmd.rr_hip.position;
      float kp9 = cmd.rr_hip.kp;
      float kd9 = cmd.rr_hip.kd;
      apply_motor_limit(9, p9, t9, kp9, kd9);
      if (motor_initialized_[9]) motors_[9]->send_motion_command(t9, p9, 0, kp9, kd9);

      // 10 右后大腿
      float t10 = cmd.rr_thigh.torque;
      float p10 = -cmd.rr_thigh.position;
      float kp10 = cmd.rr_thigh.kp;
      float kd10 = cmd.rr_thigh.kd;
      apply_motor_limit(10, p10, t10, kp10, kd10);
      if (motor_initialized_[10]) motors_[10]->send_motion_command(t10, p10, 0, kp10, kd10);

      // 11 右后小腿
      float t11 = cmd.rr_calf.torque;
      float p11 = cmd.rr_calf.position * GEAR_RATIO_CALF;
      float kp11 = cmd.rr_calf.kp;
      float kd11 = cmd.rr_calf.kd;
      apply_motor_limit(11, p11, t11, kp11, kd11);
      if (motor_initialized_[11]) motors_[11]->send_motion_command(t11, p11, 0, kp11, kd11);
      }

      auto send_duration = std::chrono::steady_clock::now() - send_start;
      max_send_duration = std::max(max_send_duration, send_duration);
      send_loops++;

      auto now = std::chrono::steady_clock::now();
      if (send_duration > COMMAND_PERIOD) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Motor send loop overrun: %.2f ms",
                             std::chrono::duration<double, std::milli>(send_duration).count());
      }
      if (now - last_diag_log >= DIAG_LOG_PERIOD) {
        RCLCPP_INFO(this->get_logger(),
                    "Motor send diag: loops=%zu avg_hz=%.1f max_send_ms=%.2f",
                    send_loops,
                    static_cast<double>(send_loops) /
                        std::chrono::duration<double>(now - last_diag_log).count(),
                    std::chrono::duration<double, std::milli>(max_send_duration).count());
        send_loops = 0;
        max_send_duration = std::chrono::steady_clock::duration::zero();
        last_diag_log = now;
      }

      next_tick += COMMAND_PERIOD;
      std::this_thread::sleep_until(next_tick);
      if (std::chrono::steady_clock::now() > next_tick + COMMAND_PERIOD) {
        next_tick = std::chrono::steady_clock::now();
      }
    }
  }

  rclcpp::Publisher<interfaces::msg::MotorFeedback12>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::Subscription<interfaces::msg::MotorCommand12>::SharedPtr command_sub_;
  std::thread worker_thread_;
  std::atomic<bool> running_ = true;
  std::atomic<bool> received_first_command_;
  std::mutex command_mutex_;
  std::mutex motors_mutex_;
  interfaces::msg::MotorCommand12 latest_command_;
  std::vector<bool> motor_initialized_;
  std::vector<std::unique_ptr<RobStrideMotor>> motors_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<MotorControlSample>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
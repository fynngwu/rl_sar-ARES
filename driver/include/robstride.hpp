#pragma once
#include <memory>
#include <vector>
#include <cstring>
#include "can_interface.hpp"
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

struct motor_state {
    float position;
    float velocity;
    float torque;
};

struct MIT_params {
    float kp;
    float kd;

    float vel_limit;
    float torque_limit;
};

class RobstrideController {
public:
    RobstrideController();
    ~RobstrideController();
    int BindCAN(std::shared_ptr<CANInterface> can_interface);

    struct MotorInfo {
        int motor_id;
        int host_id;

        float max_torque;
        float max_speed;
    };
    int BindMotor(const char* can_if, std::unique_ptr<struct MotorInfo> motor_info);
    struct motor_state GetMotorState(int motor_idx);
    int SetMITParams(int motor_idx, struct MIT_params mit_params);
    struct MIT_params GetMITParams(int motor_idx);
    int SendMITCommand(int motor_idx, float pos);
    int EnableMotor(int motor_idx);
    int EnableMotorOnly(int motor_idx);
    int DisableMotor(int motor_idx);
    int ClearMotor(int motor_idx);
    bool IsMotorOnline(int motor_idx);
    int EnableAutoReport(int motor_idx);
    int DisableAutoReport(int motor_idx);
    int SetZero(int motor_idx);
    struct MotorError {
        uint8_t error_code;
        uint8_t pattern;
    };
    MotorError GetMotorError(int motor_idx);

    // Callback for CAN RX
    void HandleCANMessage(const struct device *dev, struct can_frame *frame);

    // Enable/disable automatic fault recovery (Disable→Clear→Enable) in CAN RX thread.
    // When false, motors that fault will NOT be auto-re-enabled.
    void SetAutoRecovery(bool enabled);

private:
    float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    
    struct MotorData {
        struct MotorInfo motor_info;
        std::shared_ptr<CANInterface> can_iface;
        int motor_id;
        int host_id;
        bool enabled;
        bool online;

        struct MIT_params mit_params;
        struct motor_state state;

        float target_pos = 0.0f;
        float target_radps = 0.0f;
        float target_torque = 0.0f;
        std::chrono::steady_clock::time_point last_response_time;
        uint8_t error_code = 0;
        uint8_t pattern = 0;
    };
    std::vector<std::shared_ptr<CANInterface>> can_interfaces;
    std::vector<struct MotorData> motor_data;

    CANInterface::can_rx_callback_t can_rx_callback;

    std::thread control_thread;
    std::atomic<bool> running;
    std::atomic<bool> auto_recovery_{true};
    std::recursive_mutex motor_data_mutex;
};

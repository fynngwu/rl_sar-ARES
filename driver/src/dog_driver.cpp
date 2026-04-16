#include "dog_driver.hpp"
#include "robstride.hpp"
#include "observations.hpp"
#include "can_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

static constexpr const char* kCanNames[4] = {"can0", "can1", "can2", "can3"};
static constexpr const char* kIMUDev = "/dev/ttyCH341USB0";

DogDriver::DogDriver() {
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        can_interfaces_[leg] = std::make_shared<CANInterface>(kCanNames[leg]);
    }

    motor_controller_ = std::make_shared<RobstrideController>();

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        motor_controller_->BindCAN(can_interfaces_[leg]);
    }

    std::cout << "[DogDriver] Binding motors..." << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        for (int j = 0; j < JOINTS_PER_LEG; ++j) {
            int global_idx = leg + j * NUM_LEGS;
            auto motor_info = std::make_unique<RobstrideController::MotorInfo>();
            motor_info->motor_id = kMotorIds[global_idx];
            motor_info->host_id = HOST_ID;
            motor_info->max_torque = MAX_TORQUE;
            motor_info->max_speed = MAX_SPEED;
            motor_info->max_kp = 500.0f;
            motor_info->max_kd = 5.0f;
            int idx = motor_controller_->BindMotor(kCanNames[leg], std::move(motor_info));
            motor_indices_[global_idx] = idx;

            motor_controller_->EnableMotor(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            motor_controller_->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            motor_controller_->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            MIT_params params;
            params.kp = DEFAULT_KP;
            params.kd = DEFAULT_KD;
            params.vel_limit = MAX_SPEED;
            params.torque_limit = MAX_TORQUE;
            motor_controller_->SetMITParams(idx, params);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    imu_ = std::make_unique<IMUComponent>(kIMUDev);
    imu_connected_ = true;

    std::cout << "[DogDriver] Waiting for motors online..." << std::endl;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        bool all_online = true;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (!motor_controller_->IsMotorOnline(motor_indices_[i])) {
                all_online = false;
                break;
            }
        }
        if (all_online) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    int online_count = 0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (motor_controller_->IsMotorOnline(motor_indices_[i])) ++online_count;
    }
    std::cout << "[DogDriver] Ready: " << online_count << "/" << NUM_JOINTS
              << " motors online" << std::endl;
}

DogDriver::~DogDriver() = default;

DogDriver::JointState DogDriver::GetJointStates() const {
    JointState states;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        auto ms = motor_controller_->GetMotorState(motor_indices_[i]);
        float joint_pos = kJointDirection[i] * (ms.position - kOffsets[i]);
        float joint_vel = kJointDirection[i] * ms.velocity;
        if (i >= 8) {
            joint_pos /= KNEE_GEAR_RATIO;
            joint_vel /= KNEE_GEAR_RATIO;
        }
        states.position[i] = joint_pos;
        states.velocity[i] = joint_vel;
    }
    return states;
}

DogDriver::IMUData DogDriver::GetIMUData() const {
    IMUData data;
    auto obs = imu_->GetObs();
    for (int i = 0; i < 3; ++i) {
        data.angular_velocity[i] = obs[i];
        data.projected_gravity[i] = obs[i + 3];
    }
    return data;
}

int DogDriver::SetJointPosition(int joint_idx, float pos) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return -1;
    float motor_delta = pos;
    if (joint_idx >= 8) motor_delta *= KNEE_GEAR_RATIO;
    float motor_pos = kJointDirection[joint_idx] * motor_delta + kOffsets[joint_idx];
    return motor_controller_->SendMITCommand(motor_indices_[joint_idx], motor_pos);
}

int DogDriver::SetAllJointPositions(const std::array<float, NUM_JOINTS>& pos) {
    int ret = 0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        float motor_delta = pos[i];
        if (i >= 8) motor_delta *= KNEE_GEAR_RATIO;
        float motor_pos = kJointDirection[i] * motor_delta + kOffsets[i];
        int r = motor_controller_->SendMITCommand(motor_indices_[i], motor_pos);
        if (r != 0 && ret == 0) ret = r;
    }
    return ret;
}

int DogDriver::EnableAll() {
    int ret = 0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        int r = motor_controller_->EnableMotor(motor_indices_[i]);
        if (r != 0 && ret == 0) ret = r;
    }
    return ret;
}

int DogDriver::DisableAll() {
    int ret = 0;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        int r = motor_controller_->DisableMotor(motor_indices_[i]);
        if (r != 0 && ret == 0) ret = r;
    }
    return ret;
}

int DogDriver::EnableJoint(int joint_idx) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return -1;
    return motor_controller_->EnableMotor(motor_indices_[joint_idx]);
}

int DogDriver::DisableJoint(int joint_idx) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return -1;
    return motor_controller_->DisableMotor(motor_indices_[joint_idx]);
}

int DogDriver::SetZero(int joint_idx) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return -1;
    return motor_controller_->SetZero(motor_indices_[joint_idx]);
}

int DogDriver::EnableAutoReport(int joint_idx) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return -1;
    return motor_controller_->EnableAutoReport(motor_indices_[joint_idx]);
}

int DogDriver::DisableAutoReport(int joint_idx) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return -1;
    return motor_controller_->DisableAutoReport(motor_indices_[joint_idx]);
}

int DogDriver::SetMITParams(int joint_idx, float kp, float kd) {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return -1;
    MIT_params params;
    params.kp = kp;
    params.kd = kd;
    params.vel_limit = MAX_SPEED;
    params.torque_limit = MAX_TORQUE;
    return motor_controller_->SetMITParams(motor_indices_[joint_idx], params);
}

bool DogDriver::IsJointOnline(int joint_idx) const {
    if (joint_idx < 0 || joint_idx >= NUM_JOINTS) return false;
    return motor_controller_->IsMotorOnline(motor_indices_[joint_idx]);
}

bool DogDriver::IsIMUConnected() const {
    return imu_connected_;
}

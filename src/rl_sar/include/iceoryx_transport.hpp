#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace iox_msg {

static constexpr int NUM_JOINTS = 12;

struct MotorFeedback {
    float position[NUM_JOINTS];
    float velocity[NUM_JOINTS];
    float torque[NUM_JOINTS];
};

struct MotorCommand {
    float position[NUM_JOINTS];
};

struct MotorParam {
    float kp[NUM_JOINTS];
    float kd[NUM_JOINTS];
    float torque[NUM_JOINTS];
};

struct Imu {
    float angular_velocity[3];
    float projected_gravity[3];
};

struct XboxVel {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_z;
};

struct RemoteCmd {
    uint8_t cmd;
};

} // namespace iox_msg

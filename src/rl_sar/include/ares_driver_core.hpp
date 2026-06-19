#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "dog_driver.hpp"

enum class DriverMode : uint8_t {
    DISABLE,
    STAND,
    RL
};

class AresDriverCore {
public:
    static constexpr int NUM_JOINTS = DogDriver::NUM_JOINTS;

    using JointFeedback = DogDriver::JointState;
    using ImuData = DogDriver::IMUData;

    struct GamepadCommand {
        bool connected = false;
        float linear_x = 0.0f;
        float linear_y = 0.0f;
        float linear_z = 0.0f;
        float angular_z = 0.0f;
    };

    AresDriverCore(const std::string& policy_dir, const std::string& policy_name);
    ~AresDriverCore();

    AresDriverCore(const AresDriverCore&) = delete;
    AresDriverCore& operator=(const AresDriverCore&) = delete;

    void SetTopicCommand(const std::array<float, NUM_JOINTS>& topic_target);
    JointFeedback GetTopicFeedback() const;
    ImuData GetImuData() const;
    GamepadCommand PollGamepad();

    void SetMotorParams(const std::vector<float>& kp, const std::vector<float>& kd,
                        const std::vector<float>& torque);

    DriverMode GetMode() const;
    void PrintModeHelp() const;

    const std::vector<float>& config_kp() const;
    const std::vector<float>& config_kd() const;
    const std::vector<float>& config_torque() const;
    float gamepad_scale() const;
    bool imu_connected() const;
    bool gamepad_connected() const;
    const std::string& gamepad_name() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

class AresDriverCore {
public:
    static constexpr int NUM_JOINTS = 12;

    struct JointFeedback {
        std::array<float, NUM_JOINTS> position{};
        std::array<float, NUM_JOINTS> velocity{};
        std::array<float, NUM_JOINTS> torque{};
    };

    struct ImuData {
        std::array<float, 3> angular_velocity{};
        std::array<float, 3> projected_gravity{};
    };

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
    void SetDampingMode();

    const std::array<int, NUM_JOINTS>& topic_to_driver() const;
    const std::array<int, NUM_JOINTS>& driver_to_topic() const;
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

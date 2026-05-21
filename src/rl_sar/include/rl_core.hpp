#pragma once

#include <vector>
#include <array>
#include <string>
#include <memory>
#include <optional>
#include <chrono>
#include <fstream>
#include <utility>

#include "observations_struct.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"

class AresRL
{
public:
    AresRL();
    ~AresRL();

    bool Init(const std::string& policy_dir, const std::string& policy_name);

    void RunModel(const float imu_gyro[3], const float imu_gravity[3],
                  const float commands[3], const float joint_pos[12],
                  const float joint_vel[12], const float joint_torque[12]);

    const std::vector<float>& GetTargetPositions() const { return latest_target_pos_; }
    const std::vector<float>& GetDefaultDofPos() const { return default_dof_pos_; }
    const std::vector<float>& GetKp() const { return current_kp_; }
    const std::vector<float>& GetKd() const { return current_kd_; }
    const std::vector<float>& GetTorqueLimits() const { return current_torque_limits_; }
    const std::vector<std::pair<float, float>>& GetPositionLimits() const { return position_limits_; }
    const std::array<int, 12>& GetTopicToDriver() const { return topic_to_driver_; }
    const std::array<int, 12>& GetDriverToTopic() const { return driver_to_topic_; }

    int    GetNumDofs() const { return num_of_dofs_; }
    float  GetDt() const { return dt_; }
    int    GetDecimation() const { return decimation_; }

    enum class State { STOPPED, RUNNING };

    State  GetState() const { return current_state_; }
    void   SetState(State s) { current_state_ = s; }
    bool   IsInitialized() const { return rl_init_done_; }

    double GetInferenceTimeMs() const { return inference_time_ms_; }
    int    GetInferenceCount() const { return inference_count_; }
    const std::string& GetPolicyName() const { return policy_name_; }

    bool        IsRecordEnabled() const { return record_enabled_; }
    const std::string& GetRecordFilepath() const { return record_filepath_; }

private:
    std::vector<float> Forward(const Observations<float>& obs);
    std::vector<float> ComputeObservation(const Observations<float>& obs);
    std::vector<float> ComputeTargetPositions(const std::vector<float>& actions) const;
    void EnsureVectorSizes();
    void ComputeObsDims();
    std::string FindLatestOnnx(const std::string& dir);
    std::string FormatVector(const std::vector<float>& values) const;
    std::string FormatStringVector(const std::vector<std::string>& values) const;
    void PrintStatus();

    Observations<float> obs_;
    ObservationBuffer history_obs_buf_;
    std::vector<float> history_obs_;
    std::unique_ptr<InferenceRuntime::Model> model_;
    std::vector<float> latest_target_pos_;
    std::vector<int> obs_dims_;

    std::vector<std::string> observations_;
    std::vector<int> obs_history_;
    std::vector<float> action_scale_, commands_scale_, default_dof_pos_;
    std::vector<float> clip_actions_upper_, clip_actions_lower_;
    int    num_of_dofs_{12};
    float  dt_{0.005f};
    int    decimation_{4};
    float  lin_vel_scale_{2.0f}, ang_vel_scale_{0.25f};
    float  dof_pos_scale_{1.0f}, dof_vel_scale_{0.05f};
    float  gravity_vec_scale_{1.0f}, clip_obs_{100.0f};

    std::vector<float> current_kp_, current_kd_, current_torque_limits_;

    std::vector<std::pair<float, float>> position_limits_;

    std::array<int, 12> topic_to_driver_{}, driver_to_topic_{};

    State  current_state_{State::RUNNING};
    bool   rl_init_done_{false};
    int    inference_count_{0};
    double inference_time_ms_{0.0};
    std::string policy_name_;
    std::optional<std::chrono::steady_clock::time_point> last_print_time_;

    bool   record_enabled_{false};
    std::string record_filepath_;
    std::ofstream csv_file_;
    double record_time_{0.0};

    std::vector<float> snap_joint_pos_, snap_joint_vel_, snap_joint_torque_;
};

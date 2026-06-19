/*
 * ARES RL Node — iceoryx-based RL inference node.
 */

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

#include "rl_core.hpp"
#include "loop.hpp"
#include "keyboard_helper.hpp"
#include "iceoryx_transport.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <map>
#include <thread>

using Lock = std::lock_guard<std::mutex>;

static std::atomic<bool> g_running{true};
static void sigHandler(int) { g_running = false; }

int main(int argc, char** argv)
{
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    std::string policy_name = (argc > 1) ? argv[1] : "";
    printf("[RL] Starting ARES RL Node (iceoryx)...\n");

    iox::runtime::PoshRuntime::initRuntime("ares_rl");

    AresRL rl;

    iox::popo::PublisherOptions pubOpts;
    pubOpts.m_historyCapacity = 10;
    iox::popo::Publisher<iox_msg::MotorCommand> motorCmdPub(
        iox::capro::ServiceDescription{"rl_sar", "motor_command", ""}, pubOpts);
    iox::popo::Publisher<iox_msg::MotorParam> motorParamPub(
        iox::capro::ServiceDescription{"rl_sar", "motor_param", ""}, pubOpts);

    iox::popo::SubscriberOptions subOpts;
    subOpts.m_historyCapacity = 10;
    iox::popo::Subscriber<iox_msg::MotorFeedback> motorFbSub(
        iox::capro::ServiceDescription{"rl_sar", "motor_feedback", ""}, subOpts);
    iox::popo::Subscriber<iox_msg::Imu> imuSub(
        iox::capro::ServiceDescription{"rl_sar", "imu", ""}, subOpts);
    iox::popo::Subscriber<iox_msg::XboxVel> xboxSub(
        iox::capro::ServiceDescription{"rl_sar", "xbox_vel", ""}, subOpts);

    std::map<char, std::string> policy_map;
    {
        std::string policies_path = std::string(POLICY_DIR) + "/policies.yaml";
        YAML::Node root = YAML::LoadFile(policies_path);
        for (const auto& kv : root)
            policy_map[kv.first.as<std::string>()[0]] = kv.second.as<std::string>();
    }

    std::mutex data_mutex_, output_mutex_;
    std::array<float, 12> joint_pos_{}, joint_vel_{}, joint_torque_{};
    std::array<float, 3> commands_buffer_{}, imu_gyro_{}, imu_gravity_{};
    bool imu_received_{false}, motor_feedback_received_{false};
    bool all_sensors_ready_{false};

    auto pollSubscribers = [&]() {
        auto fbResult = motorFbSub.take();
        if (fbResult.has_value()) {
            auto& sample = fbResult.value();
            const auto* fb = sample.get();
            if (fb) {
                Lock lock(data_mutex_);
                for (int i = 0; i < rl.GetNumDofs(); ++i) {
                    int topic_idx = rl.GetDriverToTopic()[i];
                    joint_pos_[topic_idx]    = fb->position[i];
                    joint_vel_[topic_idx]    = fb->velocity[i];
                    joint_torque_[topic_idx] = fb->torque[i];
                }
                motor_feedback_received_ = true;
                if (!all_sensors_ready_ && imu_received_)
                    all_sensors_ready_ = true;
            }
        }

        auto imuResult = imuSub.take();
        if (imuResult.has_value()) {
            auto& sample = imuResult.value();
            const auto* imu = sample.get();
            if (imu) {
                Lock lock(data_mutex_);
                for (int i = 0; i < 3; ++i) {
                    imu_gyro_[i]    = imu->angular_velocity[i];
                    imu_gravity_[i] = imu->projected_gravity[i];
                }
                imu_received_ = true;
                if (!all_sensors_ready_ && motor_feedback_received_)
                    all_sensors_ready_ = true;
            }
        }

        auto xboxResult = xboxSub.take();
        if (xboxResult.has_value()) {
            auto& sample = xboxResult.value();
            const auto* xbox = sample.get();
            if (xbox) {
                Lock lock(data_mutex_);
                const auto& limits = rl.GetGamepadLimits();
                commands_buffer_[0] = std::clamp(xbox->linear_x, limits[0].first, limits[0].second);
                commands_buffer_[1] = std::clamp(xbox->linear_y, limits[1].first, limits[1].second);
                commands_buffer_[2] = std::clamp(xbox->angular_z, limits[2].first, limits[2].second);
            }
        }
    };

    auto publishJointCommand = [&](const std::vector<float>& positions) {
        iox_msg::MotorCommand cmd;
        const auto& limits = rl.GetPositionLimits();
        for (int i = 0; i < rl.GetNumDofs(); ++i) {
            float pos = positions[rl.GetDriverToTopic()[i]];
            if (!limits.empty() && static_cast<size_t>(i) < limits.size())
                pos = std::clamp(pos, limits[i].first, limits[i].second);
            cmd.position[i] = pos;
        }
        auto loan = motorCmdPub.loan();
        if (loan.has_value()) {
            auto& l = loan.value();
            std::memcpy(l.get(), &cmd, sizeof(iox_msg::MotorCommand));
            l.publish();
        }
    };

    auto publishMotorParams = [&]() {
        iox_msg::MotorParam msg;
        const auto& kp = rl.GetKp();
        const auto& kd = rl.GetKd();
        const auto& tl = rl.GetTorqueLimits();
        for (int i = 0; i < rl.GetNumDofs(); ++i) {
            msg.kp[i]     = i < (int)kp.size() ? kp[i] : 20.0f;
            msg.kd[i]     = i < (int)kd.size() ? kd[i] : 1.0f;
            msg.torque[i] = i < (int)tl.size() ? tl[i] : 17.0f;
        }
        auto loan = motorParamPub.loan();
        if (loan.has_value()) {
            auto& l = loan.value();
            std::memcpy(l.get(), &msg, sizeof(iox_msg::MotorParam));
            l.publish();
        }
    };

    auto initRL = [&](const std::string& name) -> bool {
        if (!rl.Init(std::string(POLICY_DIR), name))
            return false;
        publishMotorParams();
        all_sensors_ready_ = false;
        return true;
    };

    if (!policy_name.empty()) {
        std::string selected = policy_name;
        bool policy_found = false;
        for (const auto& [key, name] : policy_map)
            if (name == selected) { policy_found = true; break; }
        if (!policy_found && !policy_map.empty()) {
            selected = policy_map.begin()->second;
            printf("[RL] WARN: Policy not found, using first: %s\n", selected.c_str());
        }
        if (!initRL(selected)) {
            printf("[RL] ERROR: RL init failed!\n");
            return 1;
        }
        all_sensors_ready_ = true;
        rl.SetState(AresRL::State::RUNNING);
        printf("[RL] RUNNING (%s)\n", selected.c_str());
    }

    auto modelLoopFn = [&]() {
        if (!rl.IsInitialized())
            return;

        std::array<float, 12> joint_pos, joint_vel, joint_torque;
        std::array<float, 3> imu_gyro, imu_gravity, commands;
        bool ready;

        {
            Lock lock(data_mutex_);
            ready = all_sensors_ready_;
            if (ready) {
                imu_gyro     = imu_gyro_;
                imu_gravity  = imu_gravity_;
                commands     = commands_buffer_;
                joint_pos    = joint_pos_;
                joint_vel    = joint_vel_;
                joint_torque = joint_torque_;
            }
        }

        if (!ready) return;

        rl.RunModel(imu_gyro.data(), imu_gravity.data(), commands.data(),
                     joint_pos.data(), joint_vel.data(), joint_torque.data());
    };

    auto controlLoopFn = [&]() {
        pollSubscribers();

        int key = kbhit();

        if (policy_map.count(static_cast<char>(key)) &&
            (!rl.IsInitialized() || rl.GetState() == AresRL::State::STOPPED)) {
            auto it = policy_map.find(static_cast<char>(key));
            if (it != policy_map.end()) {
                printf("[RL] Loading %s ...\n", it->second.c_str());
                if (initRL(it->second)) {
                    all_sensors_ready_ = true;
                    rl.SetState(AresRL::State::RUNNING);
                    printf("[RL] RUNNING (%s)\n", it->second.c_str());
                } else {
                    printf("[RL] Init FAILED\n");
                }
            }
            return;
        }

        if (!rl.IsInitialized()) return;

        if (key == '0' && rl.GetState() != AresRL::State::STOPPED) {
            publishJointCommand(rl.GetDefaultDofPos());
            rl.SetState(AresRL::State::STOPPED);
            printf("[RL] STOPPED\n");
            return;
        }

        if ((key == 'L' || key == 'l') && rl.IsInitialized())
            rl.ToggleRecording();

        if (!all_sensors_ready_) return;
        if (rl.GetState() == AresRL::State::STOPPED) return;

        {
            Lock lock(output_mutex_);
            publishJointCommand(rl.GetTargetPositions());
        }
    };

    std::string help = "\n=== ARES RL Controls (iceoryx) ===\n";
    for (const auto& [key, name] : policy_map)
        help += "  [" + std::string(1, key) + "] " + name + "\n";
    help += "  [0] STOP\n  Switch policy from STOPPED\n\n";
    printf("%s", help.c_str());

    auto loop_control_ = std::make_shared<LoopFunc>("loop_control", rl.GetDt(), controlLoopFn);
    auto loop_rl_      = std::make_shared<LoopFunc>("loop_rl", rl.GetDt() * rl.GetDecimation(), modelLoopFn);
    loop_control_->start();
    loop_rl_->start();

    printf("[RL] iceoryx node started (Ctrl+C to quit)\n");

    while (g_running)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    loop_control_->shutdown();
    loop_rl_->shutdown();
    printf("[RL] Stopped\n");
    return 0;
}

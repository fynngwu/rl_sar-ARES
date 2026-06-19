/*
 * ARES Driver Node — thin iceoryx wrapper around AresDriverCore.
 */

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

#include "ares_driver_core.hpp"
#include "joint_names.hpp"
#include "iceoryx_transport.hpp"
#include "loop.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::atomic<bool> g_running{true};
static void sigHandler(int) { g_running = false; }

static std::string FmtFloatVec(const std::vector<float>& v)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << "[";
    for (size_t i = 0; i < v.size(); ++i)
        oss << (i ? "," : "") << v[i];
    oss << "]";
    return oss.str();
}

int main(int argc, char** argv)
{
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    std::string policy_name = (argc > 1) ? argv[1] : "ares_himloco/himloco";
    printf("[ARES Driver] Starting (policy: %s)...\n", policy_name.c_str());

    iox::runtime::PoshRuntime::initRuntime("ares_driver");

    auto core = std::make_unique<AresDriverCore>(std::string(POLICY_DIR), policy_name);

    printf("[ARES Driver] kp: %s\n", FmtFloatVec(core->config_kp()).c_str());
    printf("[ARES Driver] kd: %s\n", FmtFloatVec(core->config_kd()).c_str());
    printf("[ARES Driver] torque_limits: %s\n", FmtFloatVec(core->config_torque()).c_str());
    printf("[ARES Driver] gamepad_scale: %.2f\n", core->gamepad_scale());
    printf("[ARES Driver] IMU: %s\n", core->imu_connected() ? "yes" : "no");

    if (core->gamepad_connected())
        printf("[ARES Driver] Gamepad: %s\n", core->gamepad_name().c_str());
    else
        printf("[ARES Driver] WARN: No gamepad at /dev/input/js0\n");

    iox::popo::PublisherOptions pubOpts;
    pubOpts.m_historyCapacity = 10;
    iox::popo::Publisher<iox_msg::MotorFeedback> motorFbPub(
        iox::capro::ServiceDescription{"rl_sar", "motor_feedback", ""}, pubOpts);
    iox::popo::Publisher<iox_msg::Imu> imuPub(
        iox::capro::ServiceDescription{"rl_sar", "imu", ""}, pubOpts);
    iox::popo::Publisher<iox_msg::XboxVel> xboxPub(
        iox::capro::ServiceDescription{"rl_sar", "xbox_vel", ""}, pubOpts);

    iox::popo::SubscriberOptions subOpts;
    subOpts.m_historyCapacity = 10;
    iox::popo::Subscriber<iox_msg::MotorCommand> motorCmdSub(
        iox::capro::ServiceDescription{"rl_sar", "motor_command", ""}, subOpts);
    iox::popo::Subscriber<iox_msg::MotorParam> motorParamSub(
        iox::capro::ServiceDescription{"rl_sar", "motor_param", ""}, subOpts);

    printf("[ARES Driver] iceoryx transport ready\n");

    auto feedbackLoop = std::make_shared<LoopFunc>("feedback", 0.01f, [&]() {
        auto cmdResult = motorCmdSub.take();
        if (cmdResult.has_value()) {
            auto& sample = cmdResult.value();
            const auto* cmd = sample.get();
            if (cmd) {
                std::array<float, AresDriverCore::NUM_JOINTS> target;
                for (int i = 0; i < AresDriverCore::NUM_JOINTS; ++i)
                    target[i] = cmd->position[i];
                core->SetTopicCommand(target);
            }
        }

        auto paramResult = motorParamSub.take();
        if (paramResult.has_value()) {
            auto& sample = paramResult.value();
            const auto* param = sample.get();
            if (param) {
                std::vector<float> kp(param->kp, param->kp + AresDriverCore::NUM_JOINTS);
                std::vector<float> kd(param->kd, param->kd + AresDriverCore::NUM_JOINTS);
                std::vector<float> torque(param->torque, param->torque + AresDriverCore::NUM_JOINTS);
                core->SetMotorParams(kp, kd, torque);
                printf("[DRIVER] Motor params updated from iceoryx\n");
            }
        }

        auto joint_states = core->GetTopicFeedback();
        auto fbLoan = motorFbPub.loan();
        if (fbLoan.has_value()) {
            auto& loan = fbLoan.value();
            for (int i = 0; i < AresDriverCore::NUM_JOINTS; ++i) {
                loan->position[i] = joint_states.position[i];
                loan->velocity[i] = joint_states.velocity[i];
                loan->torque[i] = joint_states.torque[i];
            }
            loan.publish();
        }

        auto imu_data = core->GetImuData();
        auto imuLoan = imuPub.loan();
        if (imuLoan.has_value()) {
            auto& loan = imuLoan.value();
            for (int i = 0; i < 3; ++i) {
                loan->angular_velocity[i] = imu_data.angular_velocity[i];
                loan->projected_gravity[i] = imu_data.projected_gravity[i];
            }
            loan.publish();
        }

        auto gamepad = core->PollGamepad();
        if (gamepad.connected) {
            auto xboxLoan = xboxPub.loan();
            if (xboxLoan.has_value()) {
                auto& loan = xboxLoan.value();
                loan->linear_x = gamepad.linear_x;
                loan->linear_y = gamepad.linear_y;
                loan->linear_z = gamepad.linear_z;
                loan->angular_z = gamepad.angular_z;
                loan.publish();
            }
        }
    });

    core->PrintModeHelp();
    printf("[ARES Driver] iceoryx node started (Ctrl+C to quit)\n");

    feedbackLoop->start();

    while (g_running)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    feedbackLoop->shutdown();
    printf("[ARES Driver] Stopped\n");
    return 0;
}

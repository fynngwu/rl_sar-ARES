/*
 * ARES Driver Node — thin iceoryx wrapper around AresDriverCore.
 * Replaces ROS2 transport with iceoryx zero-copy IPC.
 */

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

#include "ares_driver_core.hpp"
#include "joint_names.hpp"
#include "iceoryx_transport.hpp"
#include "remote_command.hpp"
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

static const char* RemoteCommandName(RemoteCommand cmd)
{
    switch (cmd) {
    case RemoteCommand::NONE: return "NONE";
    case RemoteCommand::RECOVER_STAND: return "RECOVER_STAND";
    case RemoteCommand::SELECT_LOCOMOTION: return "SELECT_LOCOMOTION";
    case RemoteCommand::START_DREAMWAQ: return "START_DREAMWAQ";
    case RemoteCommand::DISABLE: return "DISABLE";
    case RemoteCommand::DAMPING: return "DAMPING";
    case RemoteCommand::TOGGLE_RECORD: return "TOGGLE_RECORD";
    }
    return "UNKNOWN";
}

static const char* DriverModeName(DriverMode mode)
{
    switch (mode) {
    case DriverMode::DISABLE: return "DISABLE";
    case DriverMode::STAND: return "STAND";
    case DriverMode::RL: return "RL";
    case DriverMode::DAMPING: return "DAMPING";
    }
    return "UNKNOWN";
}

static RemoteCommand DetectRemoteCommand(AresDriverCore* core)
{
    if (!core->gamepad_connected())
        return RemoteCommand::NONE;

    const bool a = core->GetGamepadButton(0);
    // LOGIC USB gamepad on this machine reports physical X/Y opposite to the
    // initial assumption, so map X->button 2 and Y->button 3.
    const bool x = core->GetGamepadButton(2);
    const bool y = core->GetGamepadButton(3);
    const bool lb = core->GetGamepadButton(4);
    const bool rb = core->GetGamepadButton(5);
    const bool start = core->GetGamepadButton(7);
    const bool lt = core->GetGamepadAxis(2) > 0.5f;
    const bool rt = core->GetGamepadAxis(5) > 0.5f;

    if (a || x || y || lb || rb || start || lt || rt) {
        printf("[ARES Driver] Gamepad state: A=%d X=%d Y=%d LB=%d RB=%d START=%d LT=%.3f RT=%.3f mode=%s\n",
               a, x, y, lb, rb, start,
               core->GetGamepadAxis(2),
               core->GetGamepadAxis(5),
               DriverModeName(core->GetMode()));
    }

    if (rb && x)
        return RemoteCommand::DISABLE;
    if (lb && rb)
        return RemoteCommand::DAMPING;
    if (lb && start)
        return RemoteCommand::TOGGLE_RECORD;
    if (lb && a)
        return RemoteCommand::RECOVER_STAND;
    if (lb && y)
        return RemoteCommand::SELECT_LOCOMOTION;
    if (lt && y)
        return RemoteCommand::START_DREAMWAQ;
    return RemoteCommand::NONE;
}

int main(int argc, char** argv)
{
    signal(SIGINT, sigHandler);
    signal(SIGTERM, sigHandler);

    std::string policy_name = (argc > 1) ? argv[1] : "dream_waq/dream_waq";
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
    pubOpts.historyCapacity = 10;
    iox::popo::Publisher<iox_msg::MotorFeedback> motorFbPub(
        iox::capro::ServiceDescription{"rl_sar", "motor_feedback", ""}, pubOpts);
    iox::popo::Publisher<iox_msg::Imu> imuPub(
        iox::capro::ServiceDescription{"rl_sar", "imu", ""}, pubOpts);
    iox::popo::Publisher<iox_msg::XboxVel> xboxPub(
        iox::capro::ServiceDescription{"rl_sar", "xbox_vel", ""}, pubOpts);
    iox::popo::Publisher<iox_msg::RemoteCmd> remoteCmdPub(
        iox::capro::ServiceDescription{"rl_sar", "remote_cmd", ""}, pubOpts);

    iox::popo::SubscriberOptions subOpts;
    subOpts.queueCapacity = 10;
    iox::popo::Subscriber<iox_msg::MotorCommand> motorCmdSub(
        iox::capro::ServiceDescription{"rl_sar", "motor_command", ""}, subOpts);
    iox::popo::Subscriber<iox_msg::MotorParam> motorParamSub(
        iox::capro::ServiceDescription{"rl_sar", "motor_param", ""}, subOpts);

    printf("[ARES Driver] iceoryx transport ready\n");

    RemoteCommand last_remote_cmd_{RemoteCommand::NONE};

    auto publishRemoteCommand = [&]() {
        RemoteCommand cmd = DetectRemoteCommand(core.get());
        if (cmd == RemoteCommand::NONE) {
            last_remote_cmd_ = RemoteCommand::NONE;
            return;
        }

        if (cmd == last_remote_cmd_)
            return;

        last_remote_cmd_ = cmd;

        printf("[ARES Driver] Remote command detected: %s (driver mode before=%s)\n",
               RemoteCommandName(cmd),
               DriverModeName(core->GetMode()));

        iox_msg::RemoteCmd msg;
        msg.cmd = static_cast<uint8_t>(cmd);
        auto loan = remoteCmdPub.loan();
        if (loan.has_value()) {
            auto& l = loan.value();
            std::memcpy(l.get(), &msg, sizeof(iox_msg::RemoteCmd));
            l.publish();
        }
        printf("[ARES Driver] Published /remote_cmd: %s\n", RemoteCommandName(cmd));

        switch (cmd) {
        case RemoteCommand::RECOVER_STAND:
        {
            bool ok = core->RequestModeChange(DriverMode::STAND);
            printf("[ARES Driver] RequestModeChange(STAND) result=%s, driver mode after=%s\n",
                   ok ? "ok" : "rejected",
                   DriverModeName(core->GetMode()));
            break;
        }
        case RemoteCommand::DISABLE:
        {
            bool ok = core->RequestModeChange(DriverMode::DISABLE);
            printf("[ARES Driver] RequestModeChange(DISABLE) result=%s, driver mode after=%s\n",
                   ok ? "ok" : "rejected",
                   DriverModeName(core->GetMode()));
            break;
        }
        case RemoteCommand::DAMPING:
        {
            bool ok = core->RequestModeChange(DriverMode::DAMPING);
            printf("[ARES Driver] RequestModeChange(DAMPING) result=%s, driver mode after=%s\n",
                   ok ? "ok" : "rejected",
                   DriverModeName(core->GetMode()));
            break;
        }
        case RemoteCommand::SELECT_LOCOMOTION:
            printf("[ARES Driver] Locomotion family selected\n");
            break;
        case RemoteCommand::START_DREAMWAQ:
            break;
        case RemoteCommand::TOGGLE_RECORD:
            break;
        case RemoteCommand::NONE:
            break;
        }
    };

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
                printf("[DRIVER] Motor params updated: kp=[");
                for (size_t i = 0; i < kp.size(); ++i)
                    printf("%s%.1f", i ? "," : "", kp[i]);
                printf("] kd=[");
                for (size_t i = 0; i < kd.size(); ++i)
                    printf("%s%.2f", i ? "," : "", kd[i]);
                printf("] torque=[");
                for (size_t i = 0; i < torque.size(); ++i)
                    printf("%s%.2f", i ? "," : "", torque[i]);
                printf("]\n");
            }
        }

        publishRemoteCommand();

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

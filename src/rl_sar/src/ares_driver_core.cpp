#include "ares_driver_core.hpp"
#include "keyboard_helper.hpp"

#include "dog_driver.hpp"
#include "observations.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <yaml-cpp/yaml.h>

static constexpr std::array<std::pair<float, float>, AresDriverCore::NUM_JOINTS> kPositionLimits = {{
    // HipA: LF, LR, RF, RR
    {-0.7853982f,  0.7853982f},
    {-0.7853982f,  0.7853982f},
    {-0.7853982f,  0.7853982f},
    {-0.7853982f,  0.7853982f},
    // HipF: LF, LR, RF, RR
    {-1.2217658f,  0.8726683f},
    {-1.2217305f,  0.8726683f},
    {-0.8726999f,  1.2217342f},
    {-0.8726999f,  1.2217305f},
    // Knee: LF, LR, RF, RR
    {-1.0217299f,  0.8f},
    {-1.0217299f,  0.8f},
    {-0.8f,        1.0217287f},
    {-0.8f,        1.0217287f},
}};

class AresDriverCore::Impl {
public:
    explicit Impl(const std::string& policy_dir, const std::string& policy_name)
        : running_(true),
          received_first_command_(false)
    {
        std::string config_path = policy_dir + "/" + policy_name + "/config.yaml";
        YAML::Node rc = YAML::LoadFile(config_path)[policy_name];
        if (!rc)
            throw std::runtime_error("Missing '" + policy_name + "' in " + config_path);

        auto t2d = rc["topic_to_driver"];
        for (int i = 0; i < NUM_JOINTS; ++i)
            topic_to_driver_[i] = t2d[i].as<int>();
        for (int i = 0; i < NUM_JOINTS; ++i)
            driver_to_topic_[topic_to_driver_[i]] = i;

        auto load_float_array = [&](const YAML::Node& n) -> std::vector<float> {
            if (n.IsSequence()) {
                std::vector<float> v;
                for (const auto& e : n)
                    v.push_back(e.as<float>());
                return v;
            }
            return std::vector<float>(NUM_JOINTS, n.as<float>());
        };
        config_kp_ = load_float_array(rc["fixed_kp"]);
        config_kd_ = load_float_array(rc["fixed_kd"]);
        config_torque_ = load_float_array(rc["torque_limits"]);
        gamepad_scale_ = rc["gamepad_scale"].as<float>();

        driver_ = std::make_unique<DogDriver>();
        for (int i = 0; i < NUM_JOINTS; ++i) {
            driver_->SetMITParams(i, config_kp_[i], config_kd_[i]);
            driver_->SetTorqueLimit(i, config_torque_[i]);
        }

        try {
            gamepad_ = std::make_unique<Gamepad>("/dev/input/js0");
            gamepad_name_ = gamepad_->GetName();
        } catch (...) {
            gamepad_.reset();
        }

        worker_thread_ = std::thread(&Impl::CommandLoop, this);
    }

    ~Impl()
    {
        running_ = false;
        if (worker_thread_.joinable())
            worker_thread_.join();
    }

    void SetTopicCommand(const std::array<float, NUM_JOINTS>& topic_target)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_target_ = topic_target;
        received_first_command_ = true;
    }

    JointFeedback GetTopicFeedback() const
    {
        auto joint_states = driver_->GetJointStates();
        JointFeedback feedback;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            int di = topic_to_driver_[i];
            feedback.position[i] = joint_states.position[di];
            feedback.velocity[i] = joint_states.velocity[di];
            feedback.torque[i] = joint_states.torque[di];
        }
        return feedback;
    }

    ImuData GetImuData() const
    {
        auto imu = driver_->GetIMUData();
        return {imu.angular_velocity, imu.projected_gravity};
    }

    GamepadCommand PollGamepad()
    {
        GamepadCommand cmd;
        if (!gamepad_ || !gamepad_->IsConnected())
            return cmd;

        if (gamepad_->GetButton(7))
            height_value_ = std::min(HEIGHT_MAX, height_value_ + HEIGHT_STEP);
        if (gamepad_->GetButton(10))
            height_value_ = std::max(HEIGHT_MIN, height_value_ - HEIGHT_STEP);

        cmd.connected = true;
        cmd.linear_x = gamepad_->GetAxis(1) * gamepad_scale_;
        cmd.linear_y = -gamepad_->GetAxis(0) * gamepad_scale_;
        cmd.linear_z = height_value_;
        cmd.angular_z = gamepad_->GetAxis(3) * gamepad_scale_;
        return cmd;
    }

    void SetDampingMode()
    {
        running_ = false;
        if (worker_thread_.joinable())
            worker_thread_.join();

        for (int i = 0; i < NUM_JOINTS; ++i)
            driver_->SetMITParams(i, 0.0f, 10.0f);
        // driver_->SetAllJointPositions({});
    }

    void SetMotorParams(const std::vector<float>& kp, const std::vector<float>& kd,
                         const std::vector<float>& torque)
    {
        size_t n = std::min({kp.size(), kd.size(), static_cast<size_t>(NUM_JOINTS)});
        for (size_t i = 0; i < n; ++i) {
            driver_->SetMITParams(i, kp[i], kd[i]);
        }
        if (!torque.empty()) {
            size_t tn = std::min(torque.size(), static_cast<size_t>(NUM_JOINTS));
            for (size_t i = 0; i < tn; ++i)
                driver_->SetTorqueLimit(i, torque[i]);
        }
        config_kp_.assign(kp.begin(), kp.begin() + n);
        config_kd_.assign(kd.begin(), kd.begin() + n);
        if (!torque.empty())
            config_torque_.assign(torque.begin(), torque.begin() + std::min(torque.size(), static_cast<size_t>(NUM_JOINTS)));
        printf("[DRIVER] Motor params updated: kp=%.1f..%.1f kd=%.2f..%.2f\n",
               kp.front(), kp.back(), kd.front(), kd.back());
    }

    DriverMode GetMode() const { return mode_.load(); }

    void PrintModeHelp() const
    {
        printf("\n=== ARES Driver Mode Help ===\n");
        printf("Current mode: %s\n", mode_name(mode_.load()));
        printf("Keys:\n");
        printf("  [s] STAND   — stand up (from DISABLE or RL)\n");
        printf("  [r] RL      — run RL policy (from STAND only)\n");
        printf("  [d] DISABLE — disable motors (from RL only)\n");
        printf("=============================\n\n");
    }

    void CommandLoop()
    {
        // --- Detect initial mode ---
        auto joint_states = driver_->GetJointStates();
        bool all_near_zero = true;
        bool any_online = false;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            if (std::abs(joint_states.position[i]) > 0.05f)
                all_near_zero = false;
            if (driver_->IsJointOnline(i))
                any_online = true;
        }

        DriverMode initial = DriverMode::DISABLE;
        if (all_near_zero && any_online)
            initial = DriverMode::RL;
        mode_ = initial;

        printf("\n[MODE] Initial state: %s\n", mode_name(initial));
        printf("Keys: [s] Stand  [r] RL  [d] Disable\n\n");

        if (initial == DriverMode::RL) {
            while (running_ && !received_first_command_.load())
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (!running_) return;
        } else {
            driver_->DisableAll();
        }

        // --- Main loop ---
        constexpr auto RL_PERIOD = std::chrono::milliseconds(5);
        constexpr auto STAND_DT  = std::chrono::milliseconds(20);
        constexpr auto DISABLE_DT = std::chrono::milliseconds(50);
        constexpr int STAND_STEPS = 100;

        auto next_tick = std::chrono::steady_clock::now();

        while (running_) {
            // --- Keyboard input ---
            int key = kbhit();
            if (key > 0) {
                key = std::tolower(key);
                handle_key_command(key);
            }

            // --- Mode-specific tick ---
            switch (mode_.load()) {
            case DriverMode::RL: {
                std::array<float, NUM_JOINTS> topic_target;
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    topic_target = latest_target_;
                }

                std::array<float, NUM_JOINTS> driver_target;
                for (int i = 0; i < NUM_JOINTS; ++i)
                    driver_target[i] = topic_target[driver_to_topic_[i]];
                for (int i = 0; i < NUM_JOINTS; ++i)
                    driver_target[i] = std::clamp(driver_target[i],
                                                   kPositionLimits[i].first,
                                                   kPositionLimits[i].second);
                driver_->SetAllJointPositions(driver_target);

                next_tick += RL_PERIOD;
                std::this_thread::sleep_until(next_tick);
                if (std::chrono::steady_clock::now() > next_tick + RL_PERIOD)
                    next_tick = std::chrono::steady_clock::now();
                break;
            }

            case DriverMode::STAND: {
                if (stand_step_ <= STAND_STEPS) {
                    float alpha = (float)stand_step_ / STAND_STEPS;
                    std::array<float, NUM_JOINTS> target;
                    for (int i = 0; i < NUM_JOINTS; ++i) {
                        float pos = stand_start_pos_[i] * (1.0f - alpha);
                        target[i] = std::clamp(pos,
                                               kPositionLimits[i].first,
                                               kPositionLimits[i].second);
                    }
                    driver_->SetAllJointPositions(target);
                    stand_step_++;
                } else {
                    std::array<float, NUM_JOINTS> target{};
                    driver_->SetAllJointPositions(target);
                }
                next_tick = std::chrono::steady_clock::now();
                std::this_thread::sleep_for(STAND_DT);
                break;
            }

            case DriverMode::DISABLE:
                next_tick = std::chrono::steady_clock::now();
                std::this_thread::sleep_for(DISABLE_DT);
                break;
            }
        }
    }

    void handle_key_command(int key)
    {
        DriverMode cur = mode_.load();
        DriverMode tgt = cur;

        if (key == 's' && cur != DriverMode::STAND)
            tgt = DriverMode::STAND;
        else if (key == 'r')
            tgt = DriverMode::RL;
        else if (key == 'd')
            tgt = DriverMode::DISABLE;

        if (tgt == cur)
            return;

        if (!transition_allowed(cur, tgt)) {
            printf("[MODE] Cannot transition from %s to %s\n",
                   mode_name(cur), mode_name(tgt));
            return;
        }

        // Perform transition
        printf("[MODE] %s → %s\n", mode_name(cur), mode_name(tgt));
        mode_ = tgt;

        switch (tgt) {
        case DriverMode::STAND:
            for (int i = 0; i < NUM_JOINTS; ++i)
                driver_->SetMITParams(i, config_kp_[i], config_kd_[i]);
            driver_->EnableAll();
            {
                auto states = driver_->GetJointStates();
                stand_start_pos_ = states.position;
            }
            stand_step_ = 0;
            stand_active_ = true;
            break;
        case DriverMode::RL:
            stand_active_ = false;
            break;
        case DriverMode::DISABLE:
            driver_->DisableAll();
            stand_active_ = false;
            break;
        }
    }

    static bool transition_allowed(DriverMode from, DriverMode to)
    {
        if (from == DriverMode::DISABLE && to == DriverMode::STAND) return true;
        if (from == DriverMode::STAND   && to == DriverMode::RL)     return true;
        if (from == DriverMode::RL      && to == DriverMode::STAND)  return true;
        if (from == DriverMode::RL      && to == DriverMode::DISABLE) return true;
        return false;
    }

    static const char* mode_name(DriverMode m)
    {
        switch (m) {
        case DriverMode::DISABLE: return "DISABLE";
        case DriverMode::STAND:   return "STAND";
        case DriverMode::RL:      return "RL";
        }
        return "???";
    }

    std::unique_ptr<DogDriver> driver_;
    std::unique_ptr<Gamepad> gamepad_;
    std::string gamepad_name_;

    std::thread worker_thread_;
    mutable std::mutex cmd_mutex_;
    std::array<float, NUM_JOINTS> latest_target_{};
    std::atomic<bool> received_first_command_;
    std::atomic<DriverMode> mode_{DriverMode::DISABLE};
    std::atomic<bool> running_;

    bool stand_active_ = false;
    int stand_step_ = 0;
    std::array<float, NUM_JOINTS> stand_start_pos_{};

    std::vector<float> config_kp_;
    std::vector<float> config_kd_;
    std::vector<float> config_torque_;
    float gamepad_scale_ = 0.0f;
    std::array<int, NUM_JOINTS> topic_to_driver_{};
    std::array<int, NUM_JOINTS> driver_to_topic_{};
    static constexpr float HEIGHT_MIN = -0.05f;
    static constexpr float HEIGHT_MAX = 0.05f;
    static constexpr float HEIGHT_STEP = 0.005f;
    float height_value_ = 0.0f;
};

AresDriverCore::AresDriverCore(const std::string& policy_dir, const std::string& policy_name)
    : impl_(std::make_unique<Impl>(policy_dir, policy_name))
{
}

AresDriverCore::~AresDriverCore() = default;

void AresDriverCore::SetTopicCommand(const std::array<float, NUM_JOINTS>& topic_target)
{
    impl_->SetTopicCommand(topic_target);
}

AresDriverCore::JointFeedback AresDriverCore::GetTopicFeedback() const
{
    return impl_->GetTopicFeedback();
}

AresDriverCore::ImuData AresDriverCore::GetImuData() const
{
    return impl_->GetImuData();
}

AresDriverCore::GamepadCommand AresDriverCore::PollGamepad()
{
    return impl_->PollGamepad();
}

void AresDriverCore::SetDampingMode()
{
    impl_->SetDampingMode();
}

void AresDriverCore::SetMotorParams(const std::vector<float>& kp, const std::vector<float>& kd,
                                     const std::vector<float>& torque)
{
    impl_->SetMotorParams(kp, kd, torque);
}

DriverMode AresDriverCore::GetMode() const
{
    return impl_->GetMode();
}

void AresDriverCore::PrintModeHelp() const
{
    impl_->PrintModeHelp();
}

const std::array<int, AresDriverCore::NUM_JOINTS>& AresDriverCore::topic_to_driver() const
{
    return impl_->topic_to_driver_;
}

const std::array<int, AresDriverCore::NUM_JOINTS>& AresDriverCore::driver_to_topic() const
{
    return impl_->driver_to_topic_;
}

const std::vector<float>& AresDriverCore::config_kp() const
{
    return impl_->config_kp_;
}

const std::vector<float>& AresDriverCore::config_kd() const
{
    return impl_->config_kd_;
}

const std::vector<float>& AresDriverCore::config_torque() const
{
    return impl_->config_torque_;
}

float AresDriverCore::gamepad_scale() const
{
    return impl_->gamepad_scale_;
}

bool AresDriverCore::imu_connected() const
{
    return impl_->driver_->IsIMUConnected();
}

bool AresDriverCore::gamepad_connected() const
{
    return static_cast<bool>(impl_->gamepad_);
}

const std::string& AresDriverCore::gamepad_name() const
{
    return impl_->gamepad_name_;
}

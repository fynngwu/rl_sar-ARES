#include "ares_driver_core.hpp"
#include "yaml_utils.hpp"

#include "dog_driver.hpp"
#include "observations.hpp"
#include "quadruped_leg_controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <yaml-cpp/yaml.h>

class AresDriverCore::Impl {
public:
    explicit Impl(const std::string& policy_dir, const std::string& policy_name)
        : policy_dir_(policy_dir), running_(true)
    {
        std::string config_path = policy_dir + "/" + policy_name + "/config.yaml";
        YAML::Node rc = YAML::LoadFile(config_path)[policy_name];
        if (!rc)
            throw std::runtime_error("Missing '" + policy_name + "' in " + config_path);

        config_kp_ = yaml_utils::LoadScalarOrArray(rc["fixed_kp"], NUM_JOINTS);
        config_kd_ = yaml_utils::LoadScalarOrArray(rc["fixed_kd"], NUM_JOINTS);
        config_torque_ = yaml_utils::LoadScalarOrArray(rc["torque_limits"], NUM_JOINTS);
        gamepad_scale_ = rc["gamepad_scale"].as<float>();

        TryCreateDriver();
        CheckGamepadConnection();

        worker_thread_ = std::thread(&Impl::CommandLoop, this);
    }

    ~Impl()
    {
        running_ = false;
        if (worker_thread_.joinable())
            worker_thread_.join();
    }

    // --- Thread-safe getters (called from ROS2 thread) ---

    void SetTopicCommand(const std::array<float, NUM_JOINTS>& target)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_target_ = target;
    }

    JointFeedback GetTopicFeedback() const
    {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (!driver_) return {};
        auto s = driver_->GetJointStates();
        JointFeedback fb;
        for (int i = 0; i < NUM_JOINTS; ++i) {
            fb.position[i] = s.position[i];
            fb.velocity[i] = s.velocity[i];
            fb.torque[i] = s.torque[i];
        }
        return fb;
    }

    ImuData GetImuData() const
    {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (!driver_) return {};
        auto imu = driver_->GetIMUData();
        return {imu.angular_velocity, imu.projected_gravity};
    }

    GamepadCommand PollGamepad()
    {
        GamepadCommand cmd;
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        if (!gamepad_ || !gamepad_->IsConnected()) return cmd;

        auto now = std::chrono::steady_clock::now();
        float dpad_y = gamepad_->GetAxis(7);
        if ((now - last_height_change_) >= std::chrono::milliseconds(200)) {
            if (dpad_y < -0.5f) {
                float old = height_value_;
                height_value_ = std::min(HEIGHT_MAX, height_value_ + HEIGHT_STEP);
                if (height_value_ != old) {
                    printf("[GAMEPAD] DPad↑ Height: %.3f → %.3f\n", old, height_value_);
                    last_height_change_ = now;
                }
            }
            if (dpad_y > 0.5f) {
                float old = height_value_;
                height_value_ = std::max(HEIGHT_MIN, height_value_ - HEIGHT_STEP);
                if (height_value_ != old) {
                    printf("[GAMEPAD] DPad↓ Height: %.3f → %.3f\n", old, height_value_);
                    last_height_change_ = now;
                }
            }
        }

        auto apply_deadzone = [](float value, float deadzone = 0.05f) -> float {
            return std::abs(value) < deadzone ? 0.0f : value;
        };

        cmd.connected = true;
        cmd.linear_x = apply_deadzone(-gamepad_->GetAxis(1)) * gamepad_scale_;
        cmd.linear_y = apply_deadzone(-gamepad_->GetAxis(0)) * gamepad_scale_;
        cmd.linear_z = height_value_;
        cmd.angular_z = apply_deadzone(-gamepad_->GetAxis(3)) * gamepad_scale_;
        return cmd;
    }

    void SetMotorParams(const std::vector<float>& kp, const std::vector<float>& kd,
                         const std::vector<float>& torque)
    {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (!driver_) return;
        size_t n = std::min({kp.size(), kd.size(), static_cast<size_t>(NUM_JOINTS)});
        for (size_t i = 0; i < n; ++i)
            driver_->SetMITParams(i, kp[i], kd[i]);
        if (!torque.empty()) {
            size_t tn = std::min(torque.size(), static_cast<size_t>(NUM_JOINTS));
            for (size_t i = 0; i < tn; ++i)
                driver_->SetTorqueLimit(i, torque[i]);
        }
        config_kp_.assign(kp.begin(), kp.begin() + n);
        config_kd_.assign(kd.begin(), kd.begin() + n);
        if (!torque.empty())
            config_torque_.assign(torque.begin(), torque.begin() + std::min(torque.size(), static_cast<size_t>(NUM_JOINTS)));
        printf("[DRIVER] Motor params updated\n");
    }

    DriverMode GetMode() const { return mode_.load(); }

    void PrintModeHelp() const
    {
        printf("\n=== ARES Driver Mode Help ===\n");
        printf("Current mode: %s\n", mode_name(mode_.load()));
        printf("=============================\n\n");
    }

    float gamepad_scale_val() const { return gamepad_scale_; }
    const std::vector<float>& config_kp_val() const { return config_kp_; }
    const std::vector<float>& config_kd_val() const { return config_kd_; }
    const std::vector<float>& config_torque_val() const { return config_torque_; }

    bool imu_connected() const
    {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        return driver_ && driver_->IsIMUConnected();
    }

    bool gamepad_connected() const
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        return gamepad_ && gamepad_->IsConnected();
    }

    std::string gamepad_name() const
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        return gamepad_name_;
    }

    // --- Worker thread ---

    void CommandLoop()
    {
        DriverMode initial = DriverMode::DISABLE;
        mode_ = initial;
        prev_mode_ = initial;
        printf("[MODE] Initial state: %s\n", mode_name(initial));

        constexpr auto RL_PERIOD = std::chrono::milliseconds(5);
        constexpr auto LOOP_DT   = std::chrono::milliseconds(20);
        constexpr int STAND_STEPS = 100;
        auto next_tick = std::chrono::steady_clock::now();

        while (running_) {
            CheckDriverConnection();
            CheckGamepadConnection();

            DriverMode cur = mode_.load();
            if (cur != prev_mode_) {
                ApplyMode(prev_mode_, cur);
                prev_mode_ = cur;
            }

            switch (cur) {
            case DriverMode::RL: {
                std::array<float, NUM_JOINTS> target;
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    target = latest_target_;
                }
                {
                    std::lock_guard<std::mutex> lock(driver_mutex_);
                    if (driver_) driver_->SetAllJointPositions(target);
                }
                next_tick += RL_PERIOD;
                std::this_thread::sleep_until(next_tick);
                if (std::chrono::steady_clock::now() > next_tick + RL_PERIOD)
                    next_tick = std::chrono::steady_clock::now();
                break;
            }
            case DriverMode::GAIT: {
                GamepadCommand gp = PollGamepad();

                GaitCommand gait_cmd{gp.linear_x, gp.linear_y, gp.angular_z};
                if (gp.linear_z != 0.0f)
                    gait_controller_.set_stand_height(0.3 + gp.linear_z);
                auto states = gait_controller_.update(gait_cmd);

                constexpr int LEG_JOINTS[4][3] = {
                    {0, 4, 8},
                    {2, 6, 10},
                    {1, 5, 9},
                    {3, 7, 11},
                };
                std::array<float, NUM_JOINTS> cmd{};
                for (int leg = 0; leg < 4; ++leg)
                    for (int ji = 0; ji < 3; ++ji)
                        cmd[LEG_JOINTS[leg][ji]] = (float)states[leg].q(ji);

                {
                    std::lock_guard<std::mutex> lock(driver_mutex_);
                    if (driver_) driver_->SetAllJointPositions(cmd);
                }
                next_tick += LOOP_DT;
                std::this_thread::sleep_until(next_tick);
                if (std::chrono::steady_clock::now() > next_tick + LOOP_DT)
                    next_tick = std::chrono::steady_clock::now();
                break;
            }
            case DriverMode::STAND: {
                if (stand_step_ <= STAND_STEPS) {
                    float alpha = (float)stand_step_ / STAND_STEPS;
                    std::array<float, NUM_JOINTS> target;
                    for (int i = 0; i < NUM_JOINTS; ++i)
                        target[i] = stand_start_pos_[i] * (1.0f - alpha);
                    {
                        std::lock_guard<std::mutex> lock(driver_mutex_);
                        if (driver_) driver_->SetAllJointPositions(target);
                    }
                    stand_step_++;
                } 
                next_tick = std::chrono::steady_clock::now();
                std::this_thread::sleep_for(LOOP_DT);
                break;
            }
            case DriverMode::DAMPING:
            case DriverMode::DISABLE:
                next_tick = std::chrono::steady_clock::now();
                std::this_thread::sleep_for(LOOP_DT);
                break;
            }
        }
    }

private:
    static const char* mode_name(DriverMode m)
    {
        switch (m) {
        case DriverMode::DISABLE: return "DISABLE";
        case DriverMode::STAND:   return "STAND";
        case DriverMode::RL:      return "RL";
        case DriverMode::DAMPING: return "DAMPING";
        case DriverMode::GAIT:    return "GAIT";
        }
        return "???";
    }

    void SetDampingGains()
    {
        for (int i = 0; i < NUM_JOINTS; ++i)
            driver_->SetMITParams(i, 0.0f, 4.0f);
    }

    struct JointPid {
        float kp = 0.0f;
        float kd = 0.0f;
    };

    static JointPid LoadJointPid(const YAML::Node& joints, const char* key)
    {
        YAML::Node node = joints[key];
        if (!node)
            throw std::runtime_error(std::string("Missing position_control.joints.") + key);
        JointPid pid;
        pid.kp = node["kp"].as<float>();
        pid.kd = node["kd"].as<float>();
        return pid;
    }

    void ApplyMotorParams(bool use_position_control_pid)
    {
        std::vector<float> kp = config_kp_;
        std::vector<float> kd = config_kd_;

        if (use_position_control_pid) {
            std::string config_path = policy_dir_ + "/position_control/config.yaml";
            try {
                YAML::Node root = YAML::LoadFile(config_path);
                YAML::Node rc = root["position_control"];
                if (!rc || !rc["joints"])
                    throw std::runtime_error("Missing position_control.joints");

                JointPid hip = LoadJointPid(rc["joints"], "hip");
                JointPid thigh = LoadJointPid(rc["joints"], "thigh");
                JointPid knee = LoadJointPid(rc["joints"], "knee");

                kp.assign(NUM_JOINTS, 0.0f);
                kd.assign(NUM_JOINTS, 0.0f);
                for (int i = 0; i < NUM_JOINTS; ++i) {
                    const JointPid& pid = (i < 4) ? hip : (i < 8 ? thigh : knee);
                    kp[i] = pid.kp;
                    kd[i] = pid.kd;
                }
                printf("[DRIVER] Position control PID loaded: %s\n", config_path.c_str());
            } catch (const std::exception& e) {
                printf("[DRIVER] Position control PID unavailable (%s). Using policy motor params.\n",
                       e.what());
            }
        }

        for (int i = 0; i < NUM_JOINTS; ++i)
            driver_->SetMITParams(i, kp[i], kd[i]);
        for (int i = 0; i < NUM_JOINTS && i < static_cast<int>(config_torque_.size()); ++i)
            driver_->SetTorqueLimit(i, config_torque_[i]);
    }

    void TryCreateDriver()
    {
        try {
            driver_ = std::make_unique<DogDriver>();
            printf("[DRIVER] DogDriver initialized\n");
        } catch (const std::exception& e) {
            printf("[DRIVER] Init failed: %s\n", e.what());
            driver_.reset();
        }
    }

    void CheckDriverConnection()
    {
        DriverMode mode = mode_.load();
        if (mode != DriverMode::DISABLE && mode != DriverMode::DAMPING) return;

        auto now = std::chrono::steady_clock::now();
        if (now - last_reconnect_ < std::chrono::milliseconds(2000)) return;
        last_reconnect_ = now;

        bool imu = imu_connected();
        const char* gp = GamepadStatusStr();
        int motors = driver_ ? driver_->OnlineMotorCount() : 0;

        printf("[STATUS] IMU=%s, Motors=%d/%d, Gamepad=%s, Height=%.3f\n",
               imu ? "yes" : "no", motors, NUM_JOINTS, gp, height_value_);

        if (!imu) {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (driver_) {
                printf("[DRIVER] IMU offline, reconnecting...\n");
                driver_->ReconnectIMU();
            }
        }

        if (!(gamepad_ && gamepad_->IsConnected())) {
            CheckGamepadConnection();
        }
    }

    void CheckGamepadConnection()
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        if (gamepad_ && gamepad_->IsConnected()) {
            if (!gamepad_connected_logged_) {
                gamepad_name_ = gamepad_->GetName();
                printf("[GAMEPAD] Connected: %s\n", gamepad_name_.c_str());
                gamepad_connected_logged_ = true;
            }
            return;
        }
        if (gamepad_connected_logged_) {
            gamepad_connected_logged_ = false;
        }
        gamepad_.reset();
        gamepad_name_.clear();
        try {
            auto gp = std::make_unique<Gamepad>("/dev/input/js0");
            if (gp->IsConnected()) {
                gamepad_name_ = gp->GetName();
                gp->SetOnUpdate([this]() { OnGamepadUpdate(); });
                gamepad_ = std::move(gp);
                printf("[GAMEPAD] Connected: %s\n", gamepad_name_.c_str());
                gamepad_connected_logged_ = true;
            }
        } catch (...) {}
    }

    const char* GamepadStatusStr()
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        return (gamepad_ && gamepad_->IsConnected()) ? "yes" : "no";
    }

    void ApplyMode(DriverMode /*prev*/, DriverMode next)
    {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (!driver_) {
            printf("[MODE] %s (driver offline)\n", mode_name(next));
            return;
        }
        switch (next) {
        case DriverMode::DISABLE:
            for (int i = 0; i < NUM_JOINTS; ++i)
                driver_->SetMITParams(i, 0.0f, 0.0f);
            driver_->DisableAll();
            printf("[MODE] → DISABLED\n");
            break;
        case DriverMode::STAND: {
            SetDampingGains();
            driver_->EnableAll();
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            auto s = driver_->GetJointStates();
            stand_start_pos_ = s.position;
            driver_->SetAllJointPositions(stand_start_pos_);
            ApplyMotorParams(false);
            stand_step_ = 0;
            printf("[MODE] → STAND\n");
            break;
        }
        case DriverMode::RL:
            ApplyMotorParams(false);
            driver_->EnableAll();
            printf("[MODE] → RL\n");
            break;
        case DriverMode::DAMPING:
            SetDampingGains();
            driver_->EnableAll();
            printf("[MODE] → DAMPING\n");
            break;
        case DriverMode::GAIT:
            ApplyMotorParams(true);
            driver_->EnableAll();
            gait_controller_.reset();
            printf("[MODE] → GAIT\n");
            break;
        }
    }

    void OnGamepadUpdate()
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        if (!gamepad_ || !gamepad_->IsConnected()) return;

        bool a     = gamepad_->GetButton(0);
        bool b     = gamepad_->GetButton(1);
        bool x     = gamepad_->GetButton(2);
        bool y     = gamepad_->GetButton(3);
        bool lb    = gamepad_->GetButton(4);
        bool rb    = gamepad_->GetButton(5);
        bool start = gamepad_->GetButton(7);

        bool rb_x     = rb && x;
        bool lb_rb    = lb && rb;
        bool lb_start = lb && start;
        bool lb_a     = lb && a;
        bool lb_b     = lb && b;
        bool lb_y     = lb && y;

        bool rb_x_edge     = rb_x     && !prev_rb_x_;
        bool lb_rb_edge    = lb_rb    && !prev_lb_rb_;
        bool lb_start_edge = lb_start && !prev_lb_start_;
        bool lb_a_edge     = lb_a     && !prev_lb_a_;
        bool lb_b_edge     = lb_b     && !prev_lb_b_;
        bool lb_y_edge     = lb_y     && !prev_lb_y_;

        prev_rb_x_     = rb_x;
        prev_lb_rb_    = lb_rb;
        prev_lb_start_ = lb_start;
        prev_lb_a_     = lb_a;
        prev_lb_b_     = lb_b;
        prev_lb_y_     = lb_y;

        if (rb_x_edge) {
            printf("[GAMEPAD] RB+X → DISABLE\n");
            mode_ = DriverMode::DISABLE;
        } else if (lb_rb_edge) {
            printf("[GAMEPAD] LB+RB → DAMPING\n");
            mode_ = DriverMode::DAMPING;
        } else if (lb_a_edge) {
            printf("[GAMEPAD] LB+A → STAND\n");
            mode_ = DriverMode::STAND;
        } else if (lb_y_edge) {
            if (mode_.load() == DriverMode::STAND) {
                printf("[GAMEPAD] LB+Y → RL\n");
                mode_ = DriverMode::RL;
            }
        } else if (lb_b_edge) {
            if (mode_.load() == DriverMode::STAND) {
                printf("[GAMEPAD] LB+B → GAIT\n");
                mode_ = DriverMode::GAIT;
            }
        } else if (lb_start_edge) {
            printf("[GAMEPAD] LB+Start → TOGGLE_RECORD\n");
        }
    }

    // --- State ---
    std::unique_ptr<DogDriver> driver_;
    std::unique_ptr<Gamepad> gamepad_;
    std::string gamepad_name_;
    std::string policy_dir_;

    std::thread worker_thread_;
    mutable std::mutex cmd_mutex_;
    mutable std::mutex driver_mutex_;
    mutable std::mutex gamepad_mutex_;
    std::array<float, NUM_JOINTS> latest_target_{};
    std::atomic<DriverMode> mode_{DriverMode::DISABLE};
    DriverMode prev_mode_{DriverMode::DISABLE};
    std::atomic<bool> running_;

    int stand_step_ = 0;
    std::array<float, NUM_JOINTS> stand_start_pos_{};

    std::vector<float> config_kp_;
    std::vector<float> config_kd_;
    std::vector<float> config_torque_;
    float gamepad_scale_ = 0.0f;
    static constexpr float HEIGHT_MIN = -0.15f;
    static constexpr float HEIGHT_MAX = 0.05f;
    static constexpr float HEIGHT_STEP = 0.03f;
    float height_value_ = 0.0f;
    std::chrono::steady_clock::time_point last_height_change_{};
    bool gamepad_connected_logged_ = false;
    bool driver_connected_logged_ = false;
    std::chrono::steady_clock::time_point last_reconnect_{};
    bool prev_rb_x_ = false, prev_lb_rb_ = false, prev_lb_start_ = false;
    bool prev_lb_a_ = false, prev_lb_b_ = false, prev_lb_y_ = false;

    QuadrupedLegController gait_controller_;
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

const std::vector<float>& AresDriverCore::config_kp() const
{
    return impl_->config_kp_val();
}

const std::vector<float>& AresDriverCore::config_kd() const
{
    return impl_->config_kd_val();
}

const std::vector<float>& AresDriverCore::config_torque() const
{
    return impl_->config_torque_val();
}

float AresDriverCore::gamepad_scale() const
{
    return impl_->gamepad_scale_val();
}

bool AresDriverCore::imu_connected() const
{
    return impl_->imu_connected();
}

bool AresDriverCore::gamepad_connected() const
{
    return impl_->gamepad_connected();
}

std::string AresDriverCore::gamepad_name() const
{
    return impl_->gamepad_name();
}

#include "ares_driver_core.hpp"
#include "yaml_utils.hpp"

#include "climb_controller.hpp"
#include "dog_driver.hpp"
#include "jump_controller.hpp"
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
    explicit Impl(const std::string& policy_dir)
        : policy_dir_(policy_dir), running_(true)
    {
        LoadPositionControlPid();

        gamepad_scale_ = 1.0f;

        jump_controller_.LoadFromYaml(policy_dir, "position_control");
        climb_controller_.LoadFromYaml(policy_dir, "position_control");

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

    void UpdateGamepadCache()
    {
        GamepadCommand cmd;
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        if (!gamepad_ || !gamepad_->IsConnected()) {
            cached_gamepad_ = cmd;
            return;
        }

        auto now = std::chrono::steady_clock::now();
        float dpad_y = gamepad_->GetAxis(7);
        float dpad_x = gamepad_->GetAxis(6);
        bool lb = gamepad_->GetButton(4);
        if (lb) {
            if ((now - last_step_height_change_) >= std::chrono::milliseconds(200)) {
                if (dpad_y < -0.5f) {
                    float old = step_height_value_;
                    step_height_value_ = std::min(STEP_HEIGHT_MAX, step_height_value_ + STEP_HEIGHT_STEP);
                    if (step_height_value_ != old) {
                        printf("[GAMEPAD] LB+DPad↑ StepHeight: %.3f → %.3f\n", old, step_height_value_);
                        last_step_height_change_ = now;
                    }
                }
                if (dpad_y > 0.5f) {
                    float old = step_height_value_;
                    step_height_value_ = std::max(STEP_HEIGHT_MIN, step_height_value_ - STEP_HEIGHT_STEP);
                    if (step_height_value_ != old) {
                        printf("[GAMEPAD] LB+DPad↓ StepHeight: %.3f → %.3f\n", old, step_height_value_);
                        last_step_height_change_ = now;
                    }
                }
            }
        } else {
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
        }
        if ((now - last_speed_scale_change_) >= std::chrono::milliseconds(200)) {
            if (dpad_x > 0.5f) {
                speed_scale_x_ += 0.2f;
                printf("[GAMEPAD] DPad→ Speed Scale X: %.2f\n", speed_scale_x_);
                last_speed_scale_change_ = now;
            }
            if (dpad_x < -0.5f) {
                speed_scale_x_ = std::max(0.2f, speed_scale_x_ - 0.2f);
                printf("[GAMEPAD] DPad← Speed Scale X: %.2f\n", speed_scale_x_);
                last_speed_scale_change_ = now;
            }
        }

        auto apply_deadzone = [](float value, float deadzone = 0.05f) -> float {
            return std::abs(value) < deadzone ? 0.0f : value;
        };

        cmd.connected = true;
        float raw_x = -gamepad_->GetAxis(1);
        float raw_y = -gamepad_->GetAxis(0);
        float raw_yaw = -gamepad_->GetAxis(3);
        float lateral_deadzone = (mode_.load() == DriverMode::GAIT) ? GAIT_LINEAR_Y_DEADZONE : 0.05f;
        cmd.linear_x = apply_deadzone(raw_x) * gamepad_scale_ * speed_scale_x_;
        cmd.linear_y = apply_deadzone(raw_y, lateral_deadzone) * gamepad_scale_;
        cmd.angular_z = apply_deadzone(raw_yaw) * gamepad_scale_;
        cmd.linear_z = height_value_;
        cmd.stride_scale = static_cast<float>(stride_scale_value_);
        cmd.step_height = step_height_value_;
        cached_gamepad_ = cmd;
    }

    GamepadCommand PollGamepad() const
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        if (auto_mode_.load()) {
            return cached_auto_cmd_;
        }
        return cached_gamepad_;
    }

    bool IsAutoMode() const
    {
        return auto_mode_.load();
    }

    void SetAutoCommand(float linear_x, float linear_y, float linear_z, float angular_z)
    {
        std::lock_guard<std::mutex> lock(gamepad_mutex_);
        cached_auto_cmd_.connected = true;
        cached_auto_cmd_.linear_x = linear_x;
        cached_auto_cmd_.linear_y = linear_y;
        cached_auto_cmd_.linear_z = linear_z;
        cached_auto_cmd_.angular_z = angular_z;
    }

    void SetMotorParams(const std::vector<float>& kp, const std::vector<float>& kd,
                         const std::vector<float>& torque)
    {
        {
            std::lock_guard<std::mutex> lock(rl_params_mutex_);
            size_t n = std::min({kp.size(), kd.size(), static_cast<size_t>(NUM_JOINTS)});
            rl_kp_.assign(kp.begin(), kp.begin() + n);
            rl_kd_.assign(kd.begin(), kd.begin() + n);
            if (!torque.empty())
                rl_torque_.assign(torque.begin(), torque.begin() + std::min(torque.size(), static_cast<size_t>(NUM_JOINTS)));
        }
        printf("[DRIVER] RL motor params cached: kp=");
        for (size_t i = 0; i < rl_kp_.size(); ++i) printf("%.1f ", rl_kp_[i]);
        printf("kd=");
        for (size_t i = 0; i < rl_kd_.size(); ++i) printf("%.2f ", rl_kd_[i]);
        printf("torque=");
        for (size_t i = 0; i < rl_torque_.size(); ++i) printf("%.1f ", rl_torque_[i]);
        printf("\n");
    }

    DriverMode GetMode() const { return mode_.load(); }

    void PrintModeHelp() const
    {
        printf("\n=== ARES Driver Mode Help ===\n");
        printf("Current mode: %s\n", mode_name(mode_.load()));
        printf("Auto mode: %s\n", auto_mode_.load() ? "ON" : "OFF");
        printf("Gamepad combos:\n");
        printf("  LB+A      → STAND\n");
        printf("  LB+Y      → RL\n");
        printf("  LB+B      → GAIT\n");
        printf("  RB+X      → DISABLE\n");
        printf("  LB+RB     → DAMPING\n");
        printf("  RB+A      → JUMP (from STAND)\n");
        printf("  RB+Y      → CLIMB (from STAND)\n");
        printf("  RB+B      → Cycle policy\n");
        printf("  LB+Start  → Toggle AUTO mode\n");
        printf("  Start     → Reconnect CAN\n");
        printf("=============================\n\n");
    }

    float gamepad_scale_val() const { return gamepad_scale_; }
    const std::vector<float>& config_kp_val() const { return config_kp_; }
    const std::vector<float>& config_kd_val() const { return config_kd_; }
    const std::vector<float>& config_torque_val() const { return config_torque_; }

    int ConsumeCycleDirection()
    {
        return policy_cycle_pending_.exchange(0);
    }

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

    int GetLastSendError(int joint_idx) const
    {
        std::lock_guard<std::mutex> lock(driver_mutex_);
        if (!driver_) return -1;
        return driver_->GetLastSendError(joint_idx);
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
            UpdateGamepadCache();
            CheckGamepadConnection();

            if (reconnect_pending_.exchange(false)) {
                ReconnectCAN();
            }

            CheckDriverConnection();

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
                float lx = gp.linear_x;
                float ly = gp.linear_y;
                float az = gp.angular_z;
                float height_cmd = gp.linear_z;
                float stride_cmd = gp.stride_scale;
                gait_params_.step_height = gp.step_height;

                GaitCommand gait_cmd{lx, ly, az};
                if (height_cmd != 0.0f)
                    gait_controller_.set_stand_height(0.3 + height_cmd);
                gait_params_.stride_scale = stride_cmd;
                gait_controller_.set_gait(gait_params_);
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
            case DriverMode::JUMP: {
                if (jump_controller_.IsActive()) {
                    auto target = jump_controller_.Update();
                    {
                        std::lock_guard<std::mutex> lock(driver_mutex_);
                        if (driver_) driver_->SetAllJointPositions(target);
                    }
                    if (jump_controller_.IsDone()) {
                        printf("[JUMP] Finished → STAND\n");
                        jump_controller_.Reset();
                        mode_ = DriverMode::STAND;
                    }
                }
                next_tick += LOOP_DT;
                std::this_thread::sleep_until(next_tick);
                if (std::chrono::steady_clock::now() > next_tick + LOOP_DT)
                    next_tick = std::chrono::steady_clock::now();
                break;
            }
            case DriverMode::CLIMB: {
                if (climb_controller_.IsActive()) {
                    auto target = climb_controller_.Update();
                    {
                        std::lock_guard<std::mutex> lock(driver_mutex_);
                        if (driver_) driver_->SetAllJointPositions(target);
                    }
                    if (climb_controller_.IsDone()) {
                        printf("[CLIMB] Finished → STAND\n");
                        climb_controller_.Stop();
                        mode_ = DriverMode::STAND;
                    }
                } else {
                    mode_ = DriverMode::STAND;
                }
                next_tick += LOOP_DT;
                std::this_thread::sleep_until(next_tick);
                if (std::chrono::steady_clock::now() > next_tick + LOOP_DT)
                    next_tick = std::chrono::steady_clock::now();
                break;
            }
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
        case DriverMode::JUMP:    return "JUMP";
        case DriverMode::CLIMB:   return "CLIMB";
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

    static double LoadDoubleOrDefault(const YAML::Node& node, const char* key, double fallback)
    {
        return (node && node[key]) ? node[key].as<double>() : fallback;
    }

    void LoadPositionControlMotion()
    {
        gait_params_ = GaitParams{};
        double stride_min = 0.4;
        double stride_max = 1.6;
        double stride_step = 0.1;
        double stride_default = 1.0;

        std::string config_path = policy_dir_ + "/position_control/config.yaml";
        try {
            YAML::Node root = YAML::LoadFile(config_path);
            YAML::Node pc = root["position_control"];
            YAML::Node motion = pc ? pc["motion"] : YAML::Node();
            if (!motion) {
                {
                    std::lock_guard<std::mutex> lock(gamepad_mutex_);
                    stride_scale_min_ = stride_min;
                    stride_scale_max_ = stride_max;
                    stride_scale_step_ = stride_step;
                    stride_scale_value_ = stride_default;
                }
                gait_params_.stride_scale = stride_default;
                gait_controller_.set_gait(gait_params_);
                printf("[DRIVER] Position control motion unavailable. Using default gait params.\n");
                return;
            }

            gait_params_.period = LoadDoubleOrDefault(motion, "period", gait_params_.period);
            gait_params_.duty_factor = LoadDoubleOrDefault(motion, "duty_factor", gait_params_.duty_factor);
            gait_params_.step_height = LoadDoubleOrDefault(motion, "step_height", gait_params_.step_height);
            gait_params_.max_stride = LoadDoubleOrDefault(motion, "max_stride", gait_params_.max_stride);
            gait_params_.foot_center_x = LoadDoubleOrDefault(motion, "foot_center_x", gait_params_.foot_center_x);
            stride_min = LoadDoubleOrDefault(motion, "stride_scale_min", stride_min);
            stride_max = LoadDoubleOrDefault(motion, "stride_scale_max", stride_max);
            stride_step = LoadDoubleOrDefault(motion, "stride_scale_step", stride_step);
            stride_default = LoadDoubleOrDefault(motion, "stride_scale_default", stride_default);
            stride_default = std::max(stride_min, std::min(stride_default, stride_max));
            {
                std::lock_guard<std::mutex> lock(gamepad_mutex_);
                stride_scale_min_ = stride_min;
                stride_scale_max_ = stride_max;
                stride_scale_step_ = stride_step;
                stride_scale_value_ = stride_default;
            }
            gait_params_.stride_scale = stride_default;
            gait_controller_.set_gait(gait_params_);
            printf("[DRIVER] Position control motion loaded: %s\n", config_path.c_str());
        } catch (const std::exception& e) {
            {
                std::lock_guard<std::mutex> lock(gamepad_mutex_);
                stride_scale_min_ = stride_min;
                stride_scale_max_ = stride_max;
                stride_scale_step_ = stride_step;
                stride_scale_value_ = stride_default;
            }
            gait_params_.stride_scale = stride_default;
            gait_controller_.set_gait(gait_params_);
            printf("[DRIVER] Position control motion unavailable (%s). Using default gait params.\n",
                   e.what());
        }
    }

    void ApplyMotorParams(bool use_position_control_pid)
    {
        std::vector<float> kp, kd, torque;

        if (use_position_control_pid) {
            kp.assign(NUM_JOINTS, 40.0f);
            kd.assign(NUM_JOINTS, 2.0f);
            std::string config_path = policy_dir_ + "/position_control/config.yaml";
            try {
                YAML::Node root = YAML::LoadFile(config_path);
                YAML::Node rc = root["position_control"];
                if (!rc || !rc["joints"])
                    throw std::runtime_error("Missing position_control.joints");

                JointPid hip = LoadJointPid(rc["joints"], "hip");
                JointPid thigh = LoadJointPid(rc["joints"], "thigh");
                JointPid knee = LoadJointPid(rc["joints"], "knee");

                for (int i = 0; i < NUM_JOINTS; ++i) {
                    const JointPid& pid = (i < 4) ? hip : (i < 8 ? thigh : knee);
                    kp[i] = pid.kp;
                    kd[i] = pid.kd;
                }
                printf("[DRIVER] Position control PID loaded: %s\n", config_path.c_str());
            } catch (const std::exception& e) {
                printf("[DRIVER] Position control PID unavailable (%s). Using defaults.\n",
                       e.what());
            }
            torque = config_torque_;
        } else {
            {
                std::lock_guard<std::mutex> lock(rl_params_mutex_);
                kp = rl_kp_;
                kd = rl_kd_;
                torque = rl_torque_;
            }
            printf("[DRIVER] Apply RL motor params: kp=");
            for (size_t i = 0; i < kp.size(); ++i) printf("%.1f ", kp[i]);
            printf("kd=");
            for (size_t i = 0; i < kd.size(); ++i) printf("%.2f ", kd[i]);
            printf("torque=");
            for (size_t i = 0; i < torque.size(); ++i) printf("%.1f ", torque[i]);
            printf("\n");
        }

        if (static_cast<int>(kp.size()) < NUM_JOINTS || static_cast<int>(kd.size()) < NUM_JOINTS) {
            printf("[DRIVER] ERROR: Motor params not loaded (kp=%zu kd=%zu torque=%zu). "
                   "Falling back to damping.\n", kp.size(), kd.size(), torque.size());
            for (int i = 0; i < NUM_JOINTS; ++i)
                driver_->SetMITParams(i, 0.0f, 4.0f);
            return;
        }
        for (int i = 0; i < NUM_JOINTS; ++i)
            driver_->SetMITParams(i, kp[i], kd[i]);
        for (int i = 0; i < NUM_JOINTS && i < static_cast<int>(torque.size()); ++i)
            driver_->SetTorqueLimit(i, torque[i]);
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

        bool imu;
        int motors;
        {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            imu = driver_ && driver_->IsIMUConnected();
            motors = driver_ ? driver_->OnlineMotorCount() : 0;
        }
        const char* gp = GamepadStatusStr();

        {
            std::lock_guard<std::mutex> lock(gamepad_mutex_);
            printf("[STATUS] IMU=%s, Motors=%d/%d, Gamepad=%s, Height=%.3f\n",
                   imu ? "yes" : "no", motors, NUM_JOINTS, gp, height_value_);
        }

        if (!imu) {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (driver_) {
                printf("[DRIVER] IMU offline, reconnecting...\n");
                auto t0 = std::chrono::steady_clock::now();
                driver_->ReconnectIMU();
                auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count();
                bool ok = driver_->IsIMUConnected();
                printf("[DRIVER] IMU reconnect done (%ldms): %s\n",
                       dt, ok ? "OK" : "FAILED");
            }
        }

        if (!(gamepad_ && gamepad_->IsConnected())) {
            CheckGamepadConnection();
        }
    }

    void ReconnectCAN()
    {
        printf("[DRIVER] Running start.sh to re-up CAN...\n");
        int ret = system("/home/ares/rl_sar-ARES/start.sh");
        if (ret != 0) {
            printf("[DRIVER] start.sh failed with code %d\n", ret);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        std::lock_guard<std::mutex> lock(driver_mutex_);
        printf("[DRIVER] Destroying old DogDriver and creating new one...\n");
        driver_.reset();
        TryCreateDriver();
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
        switch (next) {
        case DriverMode::DISABLE: {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (!driver_) { printf("[MODE] DISABLE (driver offline)\n"); return; }
            for (int i = 0; i < NUM_JOINTS; ++i)
                driver_->SetMITParams(i, 0.0f, 0.0f);
            driver_->DisableAll();
            printf("[MODE] → DISABLED\n");
            break;
        }
        case DriverMode::STAND: {
            speed_scale_x_ = 1.0f;
            {
                std::lock_guard<std::mutex> lock(driver_mutex_);
                if (!driver_) { printf("[MODE] STAND (driver offline)\n"); return; }
                SetDampingGains();
                driver_->EnableAll();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            {
                std::lock_guard<std::mutex> lock(driver_mutex_);
                if (!driver_) { printf("[MODE] STAND (driver offline)\n"); return; }
                auto s = driver_->GetJointStates();
                stand_start_pos_ = s.position;
                driver_->SetAllJointPositions(stand_start_pos_);
                ApplyMotorParams(true);
            }
            stand_step_ = 0;
            printf("[MODE] → STAND\n");
            break;
        }
        case DriverMode::RL: {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (!driver_) { printf("[MODE] RL (driver offline)\n"); return; }
            ApplyMotorParams(false);
            driver_->EnableAll();
            printf("[MODE] → RL\n");
            break;
        }
        case DriverMode::DAMPING: {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (!driver_) { printf("[MODE] DAMPING (driver offline)\n"); return; }
            SetDampingGains();
            driver_->EnableAll();
            printf("[MODE] → DAMPING\n");
            break;
        }
        case DriverMode::GAIT: {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (!driver_) { printf("[MODE] GAIT (driver offline)\n"); return; }
            LoadPositionControlMotion();
            ApplyMotorParams(true);
            driver_->EnableAll();
            gait_controller_.reset();
            printf("[MODE] → GAIT\n");
            break;
        }
        case DriverMode::JUMP: {
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (!driver_) { printf("[MODE] JUMP (driver offline)\n"); return; }
            ApplyMotorParams(true);
            driver_->EnableAll();
            jump_controller_.Reset();
            printf("[MODE] → JUMP\n");
            break;
        }
        case DriverMode::CLIMB: {
            if (!climb_controller_.Start()) {
                printf("[MODE] CLIMB unavailable → STAND\n");
                mode_ = DriverMode::STAND;
                break;
            }
            std::lock_guard<std::mutex> lock(driver_mutex_);
            if (!driver_) { printf("[MODE] CLIMB (driver offline)\n"); return; }
            ApplyMotorParams(true);
            driver_->EnableAll();
            printf("[MODE] → CLIMB\n");
            break;
        }
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
        bool lb_a     = lb && a;
        bool lb_b     = lb && b;
        bool lb_y     = lb && y;
        bool lb_x     = lb && x;
        bool lb_start = lb && start;
        bool rb_a     = rb && a;
        bool rb_b     = rb && b;
        bool rb_y     = rb && y;

        bool rb_x_edge     = rb_x     && !prev_rb_x_;
        bool lb_rb_edge    = lb_rb    && !prev_lb_rb_;
        bool lb_a_edge     = lb_a     && !prev_lb_a_;
        bool lb_b_edge     = lb_b     && !prev_lb_b_;
        bool lb_y_edge     = lb_y     && !prev_lb_y_;
        bool lb_x_edge     = lb_x     && !prev_lb_x_;
        bool lb_start_edge = lb_start && !prev_lb_start_;
        bool rb_a_edge     = rb_a     && !prev_rb_a_;
        bool rb_b_edge     = rb_b     && !prev_rb_b_;
        bool rb_y_edge     = rb_y     && !prev_rb_y_;
        bool start_edge    = start    && !prev_start_;

        prev_rb_x_     = rb_x;
        prev_lb_rb_    = lb_rb;
        prev_lb_a_     = lb_a;
        prev_lb_b_     = lb_b;
        prev_lb_y_     = lb_y;
        prev_lb_x_     = lb_x;
        prev_lb_start_ = lb_start;
        prev_rb_a_     = rb_a;
        prev_rb_b_     = rb_b;
        prev_rb_y_     = rb_y;
        prev_start_    = start;

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
            printf("[GAMEPAD] LB+Y → RL\n");
            mode_ = DriverMode::RL;
        } else if (lb_b_edge) {
            printf("[GAMEPAD] LB+B → GAIT\n");
            mode_ = DriverMode::GAIT;
        } else if (lb_start_edge) {
            bool was_auto = auto_mode_.load();
            auto_mode_.store(!was_auto);
            if (!was_auto) {
                printf("[GAMEPAD] LB+Start → AUTO MODE ON\n");
                printf("  Velocity source: /auto_cmd topic (gamepad摇杆/方向键被忽略)\n");
                printf("  手柄组合键仍然有效 (LB+RB→DAMPING 紧急停止等)\n");
            } else {
                printf("[GAMEPAD] LB+Start → AUTO MODE OFF\n");
                printf("  Velocity source: gamepad\n");
            }
        } else if (rb_a_edge) {
            if (mode_.load() == DriverMode::STAND) {
                printf("[GAMEPAD] RB+A → JUMP\n");
                mode_ = DriverMode::JUMP;
            }
        } else if (rb_b_edge) {
            CyclePolicy(+1);
        } else if (rb_y_edge) {
            if (mode_.load() == DriverMode::STAND) {
                printf("[GAMEPAD] RB+Y → CLIMB\n");
                mode_ = DriverMode::CLIMB;
            }
        } else if (start_edge) {
            printf("[GAMEPAD] Start → RECONNECT CAN (pending)\n");
            reconnect_pending_.store(true);
        }

    }

    void LoadPositionControlPid()
    {
        std::string path = policy_dir_ + "/position_control/config.yaml";
        try {
            YAML::Node root = YAML::LoadFile(path);
            YAML::Node pc = root["position_control"];
            YAML::Node joints = pc["joints"];
            JointPid hip  = LoadJointPid(joints, "hip");
            JointPid thigh = LoadJointPid(joints, "thigh");
            JointPid knee  = LoadJointPid(joints, "knee");
            config_kp_.resize(NUM_JOINTS);
            config_kd_.resize(NUM_JOINTS);
            for (int i = 0; i < 4; ++i) { config_kp_[i] = hip.kp;    config_kd_[i] = hip.kd; }
            for (int i = 4; i < 8; ++i) { config_kp_[i] = thigh.kp;  config_kd_[i] = thigh.kd; }
            for (int i = 8; i < 12; ++i) { config_kp_[i] = knee.kp;  config_kd_[i] = knee.kd; }

            if (pc["torque_limits"]) {
                auto tl = yaml_utils::LoadScalarOrArray(pc["torque_limits"], NUM_JOINTS);
                config_torque_.assign(tl.begin(), tl.begin() + std::min(tl.size(), static_cast<size_t>(NUM_JOINTS)));
                printf("[DRIVER] Loaded torque_limits from position_control\n");
            }

            printf("[DRIVER] Loaded position_control PID\n");
        } catch (const std::exception& e) {
            printf("[DRIVER] Failed to load position_control PID: %s\n", e.what());
            config_kp_.assign(NUM_JOINTS, 40.0f);
            config_kd_.assign(NUM_JOINTS, 2.0f);
        }
    }

    void CyclePolicy(int direction)
    {
        policy_cycle_pending_.store(direction);
        printf("[GAMEPAD] Policy cycle: %s\n", direction > 0 ? "next" : "prev");
    }

    // --- State ---
    std::unique_ptr<DogDriver> driver_;
    std::unique_ptr<Gamepad> gamepad_;
    std::string gamepad_name_;
    std::string policy_dir_;

    std::thread worker_thread_;
    mutable std::mutex cmd_mutex_;
    mutable std::mutex rl_params_mutex_;
    mutable std::mutex gamepad_mutex_;
    std::array<float, NUM_JOINTS> latest_target_{};
    std::atomic<DriverMode> mode_{DriverMode::DISABLE};
    DriverMode prev_mode_{DriverMode::DISABLE};
    std::atomic<bool> running_;

    int stand_step_ = 0;
    std::array<float, NUM_JOINTS> stand_start_pos_{};

    std::vector<float> config_kp_;       // position_control kp
    std::vector<float> config_kd_;       // position_control kd
    std::vector<float> config_torque_;   // position_control torque

    std::vector<float> rl_kp_;           // RL strategy kp
    std::vector<float> rl_kd_;           // RL strategy kd
    std::vector<float> rl_torque_;       // RL strategy torque
    float gamepad_scale_ = 0.0f;
    static constexpr float HEIGHT_MIN = -0.15f;
    static constexpr float HEIGHT_MAX = 0.05f;
    static constexpr float HEIGHT_STEP = 0.03f;
    static constexpr float STEP_HEIGHT_MIN = 0.05f;
    static constexpr float STEP_HEIGHT_MAX = 0.18f;
    static constexpr float STEP_HEIGHT_STEP = 0.05f;
    static constexpr float GAIT_LINEAR_Y_DEADZONE = 0.20f;
    float height_value_ = 0.0f;
    float step_height_value_ = 0.10f;
    float speed_scale_x_ = 1.0f;
    std::chrono::steady_clock::time_point last_height_change_{};
    std::chrono::steady_clock::time_point last_step_height_change_{};
    std::chrono::steady_clock::time_point last_speed_scale_change_{};
    GaitParams gait_params_;
    double stride_scale_value_ = 1.0;
    double stride_scale_min_ = 0.4;
    double stride_scale_max_ = 1.6;
    double stride_scale_step_ = 0.1;
    bool gamepad_connected_logged_ = false;
    bool driver_connected_logged_ = false;
    std::chrono::steady_clock::time_point last_reconnect_{};
    bool prev_rb_x_ = false, prev_lb_rb_ = false;
    bool prev_lb_a_ = false, prev_lb_b_ = false, prev_lb_y_ = false, prev_lb_x_ = false;
    bool prev_lb_start_ = false;
    bool prev_rb_a_ = false;
    bool prev_rb_b_ = false, prev_rb_y_ = false;
    bool prev_start_ = false;

    mutable std::mutex driver_mutex_;
    std::atomic<bool> reconnect_pending_{false};
    GamepadCommand cached_gamepad_{};

    std::atomic<bool> auto_mode_{false};
    GamepadCommand cached_auto_cmd_{};

    std::atomic<int> policy_cycle_pending_{0};

    std::chrono::steady_clock::time_point last_gait_print_{};

    QuadrupedLegController gait_controller_;
    JumpController jump_controller_;
    ClimbController climb_controller_;
};

AresDriverCore::AresDriverCore(const std::string& policy_dir)
    : impl_(std::make_unique<Impl>(policy_dir))
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

AresDriverCore::GamepadCommand AresDriverCore::PollGamepad() const
{
    return impl_->PollGamepad();
}

void AresDriverCore::SetAutoCommand(float linear_x, float linear_y, float linear_z, float angular_z)
{
    impl_->SetAutoCommand(linear_x, linear_y, linear_z, angular_z);
}

bool AresDriverCore::IsAutoMode() const
{
    return impl_->IsAutoMode();
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

int AresDriverCore::GetLastSendError(int joint_idx) const
{
    return impl_->GetLastSendError(joint_idx);
}

int AresDriverCore::ConsumeCycleDirection()
{
    return impl_->ConsumeCycleDirection();
}

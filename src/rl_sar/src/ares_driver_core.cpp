#include "ares_driver_core.hpp"

#include "dog_driver.hpp"
#include "observations.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <yaml-cpp/yaml.h>

static constexpr std::array<std::pair<float, float>, AresDriverCore::NUM_JOINTS> kPositionLimits = {{
    {-0.7853982f,  0.7853982f}, 
    {-1.2217658f,  0.8726683f},
    {-1.2217299f,  0.6f},
    {-0.7853982f,  0.7853982f},
    {-1.2217305f,  0.8726683f},
    {-1.2217299f,  0.6f},
    {-0.7853982f,  0.7853982f},
    {-0.8726999f,  1.2217342f},
    {-0.6f,        1.2217287f},
    {-0.7853982f,  0.7853982f},
    {-0.8726999f,  1.2217305f},
    {-0.6f,        1.2217287f},
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
        driver_->SetAllJointPositions({});
    }

    void CommandLoop()
    {
        while (running_ && !received_first_command_.load())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!running_)
            return;

        auto next_tick = std::chrono::steady_clock::now();
        constexpr auto COMMAND_PERIOD = std::chrono::milliseconds(5);

        while (running_) {
            std::array<float, NUM_JOINTS> clamped_target;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                clamped_target = latest_target_;
            }
            for (int i = 0; i < NUM_JOINTS; ++i)
                clamped_target[i] = std::clamp(clamped_target[i],
                                               kPositionLimits[i].first,
                                               kPositionLimits[i].second);

            std::array<float, NUM_JOINTS> driver_target;
            for (int i = 0; i < NUM_JOINTS; ++i)
                driver_target[i] = clamped_target[driver_to_topic_[i]];
            driver_->SetAllJointPositions(driver_target);

            next_tick += COMMAND_PERIOD;
            std::this_thread::sleep_until(next_tick);
            if (std::chrono::steady_clock::now() > next_tick + COMMAND_PERIOD)
                next_tick = std::chrono::steady_clock::now();
        }
    }

    std::unique_ptr<DogDriver> driver_;
    std::unique_ptr<Gamepad> gamepad_;
    std::string gamepad_name_;

    std::thread worker_thread_;
    mutable std::mutex cmd_mutex_;
    std::array<float, NUM_JOINTS> latest_target_{};
    std::atomic<bool> received_first_command_;
    std::atomic<bool> running_;

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

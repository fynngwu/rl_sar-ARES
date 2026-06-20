#include "rl_core.hpp"
#include <yaml-cpp/yaml.h>

#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

static constexpr const char* kJointNames[12] = {
    "FL_HipA", "RL_HipA", "FR_HipA", "RR_HipA",
    "FL_HipF", "RL_HipF", "FR_HipF", "RR_HipF",
    "FL_Knee", "RL_Knee", "FR_Knee", "RR_Knee"
};

AresRL::AresRL() {}

AresRL::~AresRL()
{
    if (csv_file_.is_open())
        csv_file_.close();
}

bool AresRL::Init(const std::string& policy_dir, const std::string& policy_name)
{
    std::string cfg_path = policy_dir + "/" + policy_name + "/config.yaml";
    std::string model_fn;
    rl_init_done_ = false;

    printf("[RL] Config: %s\n", cfg_path.c_str());

    try {
        YAML::Node root = YAML::LoadFile(cfg_path);
        YAML::Node rc = root[policy_name];

#define GET_CONFIG_VAR(key, var) if (rc[#key]) var = rc[#key].as<decltype(var)>()
        GET_CONFIG_VAR(dt,              dt_);
        GET_CONFIG_VAR(decimation,      decimation_);
        GET_CONFIG_VAR(num_of_dofs,     num_of_dofs_);
        GET_CONFIG_VAR(lin_vel_scale,   lin_vel_scale_);
        GET_CONFIG_VAR(ang_vel_scale,   ang_vel_scale_);
        GET_CONFIG_VAR(dof_pos_scale,   dof_pos_scale_);
        GET_CONFIG_VAR(dof_vel_scale,   dof_vel_scale_);
        GET_CONFIG_VAR(gravity_vec_scale, gravity_vec_scale_);
        GET_CONFIG_VAR(clip_obs,        clip_obs_);
#undef GET_CONFIG_VAR

        if (rc["model_name"])
            model_fn = rc["model_name"].as<std::string>();

        if (rc["observations"]) {
            const auto& list = rc["observations"];
            observations_.clear();
            for (const auto& item : list)
                observations_.push_back(item.as<std::string>());
        }

        if (rc["observations_history"]) {
            obs_history_.clear();
            for (const auto& step : rc["observations_history"])
                obs_history_.push_back(step.as<int>());
        }

        auto read_floats = [&](const char* key) {
            std::vector<float> result;
            if (rc[key])
                for (const auto& item : rc[key])
                    result.push_back(item.as<float>());
            return result;
        };
        clip_actions_upper_ = read_floats("clip_actions_upper");
        clip_actions_lower_ = read_floats("clip_actions_lower");
        commands_scale_     = read_floats("commands_scale");
        default_dof_pos_    = read_floats("default_dof_pos");

        if (rc["topic_to_driver"]) {
            auto order = rc["topic_to_driver"];
            for (int i = 0; i < 12; ++i)
                topic_to_driver_[i] = order[i].as<int>();
            for (int i = 0; i < 12; ++i)
                driver_to_topic_[topic_to_driver_[i]] = i;
        }

        auto read_scalar_or_list = [&](const YAML::Node& node) -> std::vector<float> {
            if (node.IsSequence()) {
                std::vector<float> values;
                for (const auto& item : node)
                    values.push_back(item.as<float>());
                return values;
            }
            return std::vector<float>(num_of_dofs_, node.as<float>());
        };
        current_kp_            = read_scalar_or_list(rc["fixed_kp"]);
        current_kd_            = read_scalar_or_list(rc["fixed_kd"]);
        current_torque_limits_ = read_scalar_or_list(rc["torque_limits"]);
        action_scale_          = read_scalar_or_list(rc["action_scale"]);

        // Load position limits from YAML
        position_limits_.clear();
        if (rc["position_limits"]) {
            for (const auto& item : rc["position_limits"]) {
                float lower = item[0].as<float>();
                float upper = item[1].as<float>();
                position_limits_.emplace_back(lower, upper);
            }
        }

        // Load gamepad limits from YAML (required)
        if (rc["gamepad_limits"]) {
            auto gl = rc["gamepad_limits"];
            gamepad_limits_[0] = {gl["linear_x"][0].as<float>(), gl["linear_x"][1].as<float>()};
            gamepad_limits_[1] = {gl["linear_y"][0].as<float>(), gl["linear_y"][1].as<float>()};
            gamepad_limits_[2] = {gl["angular_z"][0].as<float>(), gl["angular_z"][1].as<float>()};
        } else {
            fprintf(stderr, "[RL] ERROR: gamepad_limits not found in config\n");
            return false;
        }

        // CSV data recording
        record_dir_ = "records";
        if (rc["record"] && rc["record"]["filepath"])
            record_dir_ = rc["record"]["filepath"].as<std::string>();

        if (rc["record"] && rc["record"]["enabled"] && rc["record"]["enabled"].as<bool>()) {
            OpenRecordFile();
        } else {
            record_enabled_ = false;
        }
    } catch (const YAML::Exception& e) {
        fprintf(stderr, "[RL] ERROR: Bad config: %s\n", e.what());
        return false;
    }

    EnsureVectorSizes();

    std::string model_path;
    if (model_fn.empty()) {
        model_path = FindLatestOnnx(policy_dir + "/" + policy_name + "/");
    } else {
        model_path = policy_dir + "/" + policy_name + "/" + model_fn;
    }
    if (model_path.empty()) {
        fprintf(stderr, "[RL] ERROR: No .onnx model found\n");
        return false;
    }

    printf("[RL] Model: %s\n", model_path.c_str());
    model_ = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!model_) {
        fprintf(stderr, "[RL] ERROR: Failed to load model\n");
        return false;
    }

    ComputeObsDims();

    int history_len = 1;
    if (!obs_history_.empty())
        history_len = *std::max_element(obs_history_.begin(), obs_history_.end()) + 1;
    history_obs_buf_ = ObservationBuffer(1, obs_dims_, history_len, "time");

    int total_obs_dim = 0;
    for (int dim : obs_dims_) total_obs_dim += dim;
    for (int i = 0; i < history_len; ++i)
        history_obs_buf_.insert(std::vector<float>(total_obs_dim, 0));

    obs_.lin_vel     = {0, 0, 0};
    obs_.ang_vel     = {0, 0, 0};
    obs_.gravity_vec = {0, 0, -1};
    obs_.commands    = {0, 0, 0};
    obs_.base_quat   = {1, 0, 0, 0};
    obs_.dof_pos     = default_dof_pos_;
    obs_.dof_vel.assign(num_of_dofs_, 0);
    obs_.actions.assign(num_of_dofs_, 0);
    latest_target_pos_ = default_dof_pos_;

    snap_joint_pos_.assign(num_of_dofs_, 0);
    snap_joint_vel_.assign(num_of_dofs_, 0);
    snap_joint_torque_.assign(num_of_dofs_, 0);

    policy_name_ = policy_name;
    rl_init_done_ = true;

    printf("[RL] Init OK  dof=%d  history=%d\n", num_of_dofs_, history_len);
    printf("[RL]   default_dof_pos: %s\n", FormatVector(default_dof_pos_).c_str());
    printf("[RL]   action_scale:    %s\n", FormatVector(action_scale_).c_str());
    printf("[RL]   commands_scale:  %s\n", FormatVector(commands_scale_).c_str());
    printf("[RL]   observations:    %s\n", FormatStringVector(observations_).c_str());

    if (!position_limits_.empty()) {
        printf("[RL]   position_limits (%zu joints):\n", position_limits_.size());
        for (size_t i = 0; i < position_limits_.size() && i < 12; ++i)
            printf("[RL]     [%zu] %-12s  lower=%.6f  upper=%.6f\n",
                   i, kJointNames[i], position_limits_[i].first, position_limits_[i].second);
    }

    return true;
}

void AresRL::RunModel(const float imu_gyro[3], const float imu_gravity[3],
                       const float commands[3], const float joint_pos[12],
                       const float joint_vel[12], const float joint_torque[12])
{
    if (current_state_ == State::STOPPED || !rl_init_done_)
        return;

    obs_.ang_vel     = {imu_gyro[0], imu_gyro[1], imu_gyro[2]};
    obs_.gravity_vec = {imu_gravity[0], imu_gravity[1], imu_gravity[2]};
    obs_.commands    = {commands[0], commands[1], commands[2]};
    for (int i = 0; i < num_of_dofs_; ++i) {
        obs_.dof_pos[i] = joint_pos[i];
        obs_.dof_vel[i] = joint_vel[i];
    }

    snap_joint_pos_.assign(joint_pos, joint_pos + num_of_dofs_);
    snap_joint_vel_.assign(joint_vel, joint_vel + num_of_dofs_);
    snap_joint_torque_.assign(joint_torque, joint_torque + num_of_dofs_);

    std::vector<float> actions = Forward(obs_);
    if (actions.empty()) return;
    obs_.actions = actions;

    latest_target_pos_ = ComputeTargetPositions(actions);
    inference_count_++;

    if (record_enabled_ && csv_file_.is_open()) {
        csv_file_ << record_step_;
        for (int i = 0; i < num_of_dofs_; ++i)
            csv_file_ << "," << snap_joint_pos_[i]
                      << "," << snap_joint_vel_[i]
                      << "," << snap_joint_torque_[i]
                      << "," << latest_target_pos_[i];
        csv_file_ << "\n";
        record_step_++;
    }

    PrintStatus();
}

std::vector<float> AresRL::Forward(const Observations<float>& obs)
{
    using Clock = std::chrono::high_resolution_clock;
    auto start = Clock::now();

    std::vector<float> obs_vec = ComputeObservation(obs);
    std::vector<float> actions;

    if (!obs_history_.empty()) {
        history_obs_buf_.insert(obs_vec);
        history_obs_ = history_obs_buf_.get_obs_vec(obs_history_);
        actions = model_->forward({history_obs_});
    } else {
        actions = model_->forward({obs_vec});
    }

    inference_time_ms_ = std::chrono::duration<double, std::milli>(
        Clock::now() - start).count();

    if (actions.empty() || actions.size() != static_cast<size_t>(num_of_dofs_)) {
        fprintf(stderr, "[RL] ERROR: Bad inference output\n");
        return {};
    }
    for (float v : actions) {
        if (std::isnan(v) || std::isinf(v)) {
            fprintf(stderr, "[RL] ERROR: NaN/Inf in inference output\n");
            return {};
        }
    }

    for (size_t i = 0; i < actions.size(); ++i) {
        float lo = i < clip_actions_lower_.size() ? clip_actions_lower_[i] : -1.0f;
        float hi = i < clip_actions_upper_.size() ? clip_actions_upper_[i] :  1.0f;
        actions[i] = std::max(lo, std::min(hi, actions[i]));
    }
    return actions;
}

std::vector<float> AresRL::ComputeObservation(const Observations<float>& obs)
{
    std::vector<float> obs_vec;

    std::vector<float> gravity_vec = obs.gravity_vec;
    for (auto& v : gravity_vec)
        v *= gravity_vec_scale_;

    for (const std::string& key : observations_) {
        if (key == "commands") {
            std::vector<float> s = obs.commands;
            for (size_t i = 0; i < s.size() && i < commands_scale_.size(); ++i)
                s[i] *= commands_scale_[i];
            obs_vec.insert(obs_vec.end(), s.begin(), s.end());
        } else if (key == "ang_vel") {
            for (float v : obs.ang_vel)
                obs_vec.push_back(v * ang_vel_scale_);
        } else if (key == "gravity_vec") {
            obs_vec.insert(obs_vec.end(), gravity_vec.begin(), gravity_vec.end());
        } else if (key == "dof_pos") {
            for (int i = 0; i < num_of_dofs_; ++i)
                obs_vec.push_back((obs.dof_pos[i] - default_dof_pos_[i]) * dof_pos_scale_);
        } else if (key == "dof_vel") {
            for (int i = 0; i < num_of_dofs_; ++i)
                obs_vec.push_back(obs.dof_vel[i] * dof_vel_scale_);
        } else if (key == "actions") {
            obs_vec.insert(obs_vec.end(), obs.actions.begin(), obs.actions.end());
        }
    }

    for (auto& v : obs_vec)
        v = std::max(-clip_obs_, std::min(clip_obs_, v));
    return obs_vec;
}

std::vector<float> AresRL::ComputeTargetPositions(const std::vector<float>& actions) const
{
    std::vector<float> target(num_of_dofs_);
    size_t n = std::min(actions.size(), static_cast<size_t>(num_of_dofs_));
    for (size_t i = 0; i < n; ++i)
        target[i] = actions[i] * action_scale_[i] + default_dof_pos_[i];
    for (size_t i = n; i < target.size(); ++i)
        target[i] = default_dof_pos_[i];
    return target;
}

void AresRL::EnsureVectorSizes()
{
    auto sz = static_cast<size_t>(num_of_dofs_);
    if (action_scale_.size() < sz)       action_scale_.resize(sz, 0.25f);
    if (commands_scale_.size() < 3)      commands_scale_.resize(3, 1.0f);
    if (default_dof_pos_.size() < sz)    default_dof_pos_.resize(sz, 0.0f);
    if (clip_actions_upper_.size() < sz) clip_actions_upper_.resize(sz, 1.0f);
    if (clip_actions_lower_.size() < sz) clip_actions_lower_.resize(sz, -1.0f);
}

void AresRL::ComputeObsDims()
{
    obs_dims_.clear();
    for (const std::string& key : observations_) {
        int dim = 0;
        if (key == "lin_vel" || key == "ang_vel" ||
            key == "gravity_vec" || key == "commands")
            dim = 3;
        else if (key == "dof_pos" || key == "dof_vel" || key == "actions")
            dim = num_of_dofs_;
        if (dim > 0) obs_dims_.push_back(dim);
    }
}

std::string AresRL::FindLatestOnnx(const std::string& dir)
{
    namespace fs = std::filesystem;
    std::string best_path;
    fs::file_time_type best_time;
    try {
        if (!fs::exists(dir)) return "";
        for (const auto& entry : fs::directory_iterator(dir)) {
            if (!entry.is_regular_file()) continue;
            auto fn = entry.path().filename().string();
            if (fn.size() >= 5 &&
                strcasecmp(fn.c_str() + fn.size() - 5, ".onnx") == 0) {
                auto mtime = entry.last_write_time();
                if (best_path.empty() || mtime > best_time) {
                    best_path = entry.path().string();
                    best_time = mtime;
                }
            }
        }
    } catch (...) {}
    return best_path;
}

std::string AresRL::FormatVector(const std::vector<float>& values) const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}

std::string AresRL::FormatStringVector(const std::vector<std::string>& values) const
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << values[i];
    }
    oss << "]";
    return oss.str();
}

void AresRL::WriteCsvHeader()
{
    csv_file_ << "step";
    for (int i = 0; i < num_of_dofs_; ++i)
        csv_file_ << "," << kJointNames[i] << "_pos,"
                  << kJointNames[i] << "_vel,"
                  << kJointNames[i] << "_torque,"
                  << kJointNames[i] << "_target";
    csv_file_ << "\n";
}

void AresRL::OpenRecordFile()
{
    if (csv_file_.is_open()) csv_file_.close();

    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << record_dir_ << "/record_" << std::put_time(std::localtime(&tt), "%Y%m%d_%H%M%S") << ".csv";

    auto dir = std::filesystem::path(record_dir_);
    if (!std::filesystem::exists(dir))
        std::filesystem::create_directory(dir);

    csv_file_.open(ss.str());
    if (csv_file_.is_open()) {
        record_enabled_ = true;
        record_filepath_ = ss.str();
        record_step_ = 0;
        WriteCsvHeader();
        printf("[RL] Recording started: %s\n", record_filepath_.c_str());
    }
}

void AresRL::ToggleRecording()
{
    if (record_enabled_ && csv_file_.is_open()) {
        csv_file_.close();
        record_enabled_ = false;
        printf("[RL] Recording stopped (%d steps): %s\n", record_step_, record_filepath_.c_str());
    } else {
        OpenRecordFile();
    }
}

void AresRL::PrintStatus()
{
    auto now = std::chrono::steady_clock::now();
    if (!last_print_time_ ||
        std::chrono::duration<float>(now - *last_print_time_).count() >= 1.0f) {
        last_print_time_ = now;
        printf("\033[2J\033[H");
        printf("%-8s [%s] inf:%.1fms #%d  [0]=S [1]=H [2]=C\n",
               current_state_ == State::STOPPED ? "STOPPED" : "RUNNING",
               policy_name_.c_str(), inference_time_ms_, inference_count_);
        printf("Joint          DofPos       DofVel       DofTrq       Target\n");
        printf("-------------------------------------------------------------------\n");
        for (int i = 0; i < num_of_dofs_; ++i) {
            int t = driver_to_topic_[i];
            printf("%-12s %10.4f %10.4f %10.4f %10.4f\n",
                   kJointNames[i],
                   snap_joint_pos_[t], snap_joint_vel_[t],
                   snap_joint_torque_[t], latest_target_pos_[t]);
        }
        fflush(stdout);
    }
}

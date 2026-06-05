#include "dog_driver.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

std::atomic<bool> g_stop_requested{false};

constexpr std::array<const char*, DogDriver::NUM_JOINTS> kJointNames = {
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
};

constexpr std::array<std::pair<float, float>, DogDriver::NUM_JOINTS> kPositionLimits = {{
    {-0.7853982f,  0.7853982f},
    {-0.7853982f,  0.7853982f},
    {-0.7853982f,  0.7853982f},
    {-0.7853982f,  0.7853982f},
    {-1.2217658f,  0.8726683f},
    {-1.2217305f,  0.8726683f},
    {-0.8726999f,  1.2217342f},
    {-0.8726999f,  1.2217305f},
    {-1.0217299f,  0.8f},
    {-1.0217299f,  0.8f},
    {-0.8f,        1.0217287f},
    {-0.8f,        1.0217287f},
}};

constexpr std::array<float, DogDriver::NUM_JOINTS> kDefaultBias = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
};

constexpr std::array<float, DogDriver::NUM_JOINTS> kDefaultDirection = {
     1.0f, -1.0f,  1.0f, -1.0f,
     1.0f,  1.0f, -1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,  1.0f,
};

constexpr std::array<float, DogDriver::NUM_JOINTS> kDefaultAmplitude = {
    0.20f, 0.20f, 0.20f, 0.20f,
    0.30f, 0.30f, 0.30f, 0.30f,
    0.40f, 0.40f, 0.40f, 0.40f,
};

constexpr float kSafetyMargin = 0.05f;
constexpr float kPi = 3.14159265358979323846f;

struct Config {
    std::string output_csv = "/tmp/ares_pace_chirp.csv";
    double loop_hz = 50.0;
    double duration = 20.0;
    double settle_duration = 0.5;
    double min_frequency = 0.1;
    double max_frequency = 3.0;
    float kp = 15.0f;
    float kd = 0.8f;
    float torque_limit = 10.0f;
    bool disable_on_exit = false;
    std::array<float, DogDriver::NUM_JOINTS> bias = kDefaultBias;
    std::array<float, DogDriver::NUM_JOINTS> direction = kDefaultDirection;
    std::array<float, DogDriver::NUM_JOINTS> amplitude = kDefaultAmplitude;
};

void SignalHandler(int) {
    g_stop_requested.store(true);
}

void PrintUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "Options:\n"
        << "  --output <path>          CSV output path (default: /tmp/ares_pace_chirp.csv)\n"
        << "  --loop_hz <Hz>           Control / logging rate (default: 50)\n"
        << "  --duration <s>           Chirp duration (default: 20)\n"
        << "  --settle <s>             Hold-start duration before chirp (default: 0.5)\n"
        << "  --min_freq <Hz>          Chirp start frequency (default: 0.1)\n"
        << "  --max_freq <Hz>          Chirp end frequency (default: 3.0)\n"
        << "  --kp <value>             MIT Kp for all joints (default: 15)\n"
        << "  --kd <value>             MIT Kd for all joints (default: 0.8)\n"
        << "  --torque_limit <Nm>      Torque limit for all joints (default: 10)\n"
        << "  --disable_on_exit        Disable motors after recording\n"
        << "  --help                   Show this message\n";
}

bool ParseDoubleArg(int& i, int argc, char** argv, double& value) {
    if (i + 1 >= argc) {
        return false;
    }
    value = std::atof(argv[++i]);
    return true;
}

bool ParseFloatArg(int& i, int argc, char** argv, float& value) {
    if (i + 1 >= argc) {
        return false;
    }
    value = std::atof(argv[++i]);
    return true;
}

bool ParseStringArg(int& i, int argc, char** argv, std::string& value) {
    if (i + 1 >= argc) {
        return false;
    }
    value = argv[++i];
    return true;
}

bool ParseArgs(int argc, char** argv, Config& cfg) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--output") {
            if (!ParseStringArg(i, argc, argv, cfg.output_csv)) return false;
        } else if (arg == "--loop_hz") {
            if (!ParseDoubleArg(i, argc, argv, cfg.loop_hz)) return false;
        } else if (arg == "--duration") {
            if (!ParseDoubleArg(i, argc, argv, cfg.duration)) return false;
        } else if (arg == "--settle") {
            if (!ParseDoubleArg(i, argc, argv, cfg.settle_duration)) return false;
        } else if (arg == "--min_freq") {
            if (!ParseDoubleArg(i, argc, argv, cfg.min_frequency)) return false;
        } else if (arg == "--max_freq") {
            if (!ParseDoubleArg(i, argc, argv, cfg.max_frequency)) return false;
        } else if (arg == "--kp") {
            if (!ParseFloatArg(i, argc, argv, cfg.kp)) return false;
        } else if (arg == "--kd") {
            if (!ParseFloatArg(i, argc, argv, cfg.kd)) return false;
        } else if (arg == "--torque_limit") {
            if (!ParseFloatArg(i, argc, argv, cfg.torque_limit)) return false;
        } else if (arg == "--disable_on_exit") {
            cfg.disable_on_exit = true;
        } else if (arg == "--help") {
            PrintUsage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            return false;
        }
    }
    return true;
}

float ClampWithMargin(float value, const std::pair<float, float>& limits) {
    return std::clamp(value, limits.first + kSafetyMargin, limits.second - kSafetyMargin);
}

std::array<float, DogDriver::NUM_JOINTS> ClampPose(const std::array<float, DogDriver::NUM_JOINTS>& pose) {
    std::array<float, DogDriver::NUM_JOINTS> clamped = pose;
    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
        clamped[i] = ClampWithMargin(clamped[i], kPositionLimits[i]);
    }
    return clamped;
}

void CheckConfig(const Config& cfg) {
    if (cfg.loop_hz <= 0.0) {
        throw std::runtime_error("loop_hz must be positive");
    }
    if (cfg.duration <= 0.0) {
        throw std::runtime_error("duration must be positive");
    }
    if (cfg.min_frequency < 0.0 || cfg.max_frequency <= 0.0 || cfg.max_frequency < cfg.min_frequency) {
        throw std::runtime_error("invalid chirp frequency range");
    }
    if (cfg.settle_duration < 0.0) {
        throw std::runtime_error("settle duration must be non-negative");
    }
}

void EnsureHardwareReady(DogDriver& driver) {
    int online_count = 0;
    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
        if (driver.IsJointInitialized(i) && driver.IsJointOnline(i)) {
            ++online_count;
        }
    }
    std::cout << "[PACE] Online joints: " << online_count << "/" << DogDriver::NUM_JOINTS << std::endl;
    if (online_count != DogDriver::NUM_JOINTS) {
        throw std::runtime_error("not all joints are online; aborting collection");
    }
}

void ConfigureMIT(DogDriver& driver, const Config& cfg) {
    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
        const int kp_result = driver.SetMITParams(i, cfg.kp, cfg.kd);
        const int torque_result = driver.SetTorqueLimit(i, cfg.torque_limit);
        if (kp_result != 0 || torque_result != 0) {
            std::ostringstream oss;
            oss << "failed to configure joint " << i;
            throw std::runtime_error(oss.str());
        }
    }
}

void HoldPose(
    DogDriver& driver,
    const std::array<float, DogDriver::NUM_JOINTS>& pose,
    double loop_hz,
    double duration_sec)
{
    const auto clamped_pose = ClampPose(pose);
    const int steps = std::max(1, static_cast<int>(std::round(duration_sec * loop_hz)));
    const auto tick = std::chrono::duration<double>(1.0 / loop_hz);
    auto next_tick = Clock::now();

    for (int step = 0; step < steps && !g_stop_requested.load(); ++step) {
        driver.SetAllJointPositions(clamped_pose);
        next_tick += std::chrono::duration_cast<Clock::duration>(tick);
        std::this_thread::sleep_until(next_tick);
    }
}

std::array<float, DogDriver::NUM_JOINTS> ComputeTarget(const Config& cfg, double t_sec) {
    const double chirp_phase =
        2.0 * kPi * (cfg.min_frequency * t_sec +
                     ((cfg.max_frequency - cfg.min_frequency) / (2.0 * cfg.duration)) * t_sec * t_sec);
    const float chirp_value = static_cast<float>(std::sin(chirp_phase));

    std::array<float, DogDriver::NUM_JOINTS> pose{};
    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
        pose[i] = cfg.bias[i] + cfg.direction[i] * cfg.amplitude[i] * chirp_value;
    }
    return ClampPose(pose);
}

void WriteHeader(std::ofstream& out) {
    out << "time";
    for (const char* joint_name : kJointNames) out << ",des_" << joint_name;
    for (const char* joint_name : kJointNames) out << ",meas_" << joint_name;
    for (const char* joint_name : kJointNames) out << ",vel_" << joint_name;
    for (const char* joint_name : kJointNames) out << ",torque_" << joint_name;
    out << '\n';
}

void WriteRow(
    std::ofstream& out,
    double t_sec,
    const std::array<float, DogDriver::NUM_JOINTS>& target,
    const DogDriver::JointState& state)
{
    out << std::fixed << std::setprecision(9) << t_sec;
    for (float value : target) out << ',' << value;
    for (float value : state.position) out << ',' << value;
    for (float value : state.velocity) out << ',' << value;
    for (float value : state.torque) out << ',' << value;
    out << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    Config cfg;
    if (!ParseArgs(argc, argv, cfg)) {
        return 1;
    }

    try {
        CheckConfig(cfg);

        std::cout << "[PACE] Starting real-robot chirp collection" << std::endl;
        std::cout << "[PACE] Joint order:";
        for (const char* name : kJointNames) {
            std::cout << ' ' << name;
        }
        std::cout << std::endl;
        std::cout << "[PACE] Output CSV: " << cfg.output_csv << std::endl;
        std::cout << "[PACE] Loop rate: " << cfg.loop_hz << " Hz, chirp: "
                  << cfg.min_frequency << " -> " << cfg.max_frequency << " Hz" << std::endl;

        DogDriver driver;
        EnsureHardwareReady(driver);
        ConfigureMIT(driver, cfg);

        const auto start_pose = ComputeTarget(cfg, 0.0);
        std::cout << "[PACE] Setting start pose directly, following PACE data_collection behavior..." << std::endl;
        driver.SetAllJointPositions(start_pose);
        if (!g_stop_requested.load() && cfg.settle_duration > 0.0) {
            std::cout << "[PACE] Settling before chirp..." << std::endl;
            HoldPose(driver, start_pose, cfg.loop_hz, cfg.settle_duration);
        }

        std::ofstream out(cfg.output_csv);
        if (!out.is_open()) {
            throw std::runtime_error("failed to open output CSV");
        }
        WriteHeader(out);

        const int num_steps = static_cast<int>(std::round(cfg.duration * cfg.loop_hz));
        const auto tick = std::chrono::duration<double>(1.0 / cfg.loop_hz);
        auto next_tick = Clock::now();

        std::cout << "[PACE] Recording chirp..." << std::endl;
        for (int step = 0; step < num_steps && !g_stop_requested.load(); ++step) {
            const double t_sec = static_cast<double>(step) / cfg.loop_hz;
            const auto target = ComputeTarget(cfg, t_sec);
            driver.SetAllJointPositions(target);
            const auto state = driver.GetJointStates();
            WriteRow(out, t_sec, target, state);

            if (step % static_cast<int>(cfg.loop_hz) == 0) {
                std::cout << "[PACE] t=" << std::fixed << std::setprecision(1)
                          << t_sec << " / " << cfg.duration << " s" << std::endl;
            }

            next_tick += std::chrono::duration_cast<Clock::duration>(tick);
            std::this_thread::sleep_until(next_tick);
            if (Clock::now() > next_tick + std::chrono::duration_cast<Clock::duration>(tick)) {
                next_tick = Clock::now();
            }
        }

        out.close();

        if (cfg.disable_on_exit) {
            std::cout << "[PACE] Disabling motors on exit..." << std::endl;
            driver.DisableAll();
        }

        if (g_stop_requested.load()) {
            std::cout << "[PACE] Stopped early by signal. Partial CSV saved to "
                      << cfg.output_csv << std::endl;
            return 2;
        }

        std::cout << "[PACE] Finished. CSV saved to " << cfg.output_csv << std::endl;
        std::cout << "[PACE] Convert with: python3 tools/pace/convert_pace_csv.py "
                  << cfg.output_csv << " /path/to/chirp_data.pt" << std::endl;
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "[PACE] ERROR: " << ex.what() << std::endl;
        return 1;
    }
}

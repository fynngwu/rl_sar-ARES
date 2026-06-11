#include "dog_driver.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

constexpr int NUM_JOINTS = DogDriver::NUM_JOINTS;
constexpr int NUM_LEGS = DogDriver::NUM_LEGS;
constexpr float PI = 3.14159265358979323846f;
constexpr const char* VERSION = "20260611_squat";

std::atomic<bool> g_running{true};

constexpr std::array<const char*, NUM_JOINTS> kJointNames = {
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee",
};

constexpr std::array<std::pair<float, float>, NUM_JOINTS> kPositionLimits = {{
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

// Driver order is LF, LR, RF, RR inside each joint group. HipF/Knee position
// limits are mirrored between left and right sides, so the default squat shape
// mirrors left and right rather than sending the same sign to all four legs.
constexpr std::array<float, NUM_JOINTS> kSquatShape = {
     0.0f,  0.0f,  0.0f,  0.0f,
     0.7f,  0.7f, -0.7f, -0.7f,
    -1.0f, -1.0f,  1.0f,  1.0f,
};

constexpr std::array<float, NUM_JOINTS> kReverseSquatShape = {
     0.0f,  0.0f,  0.0f,  0.0f,
    -0.7f, -0.7f,  0.7f,  0.7f,
     1.0f,  1.0f, -1.0f, -1.0f,
};

// Same command sign on all four legs. Kept because it is a useful diagnostic,
// but it is not the default squat shape on this mirrored joint convention.
constexpr std::array<float, NUM_JOINTS> kSameSignShape = {
     0.0f,  0.0f,  0.0f,  0.0f,
     0.7f,  0.7f,  0.7f,  0.7f,
    -1.0f, -1.0f, -1.0f, -1.0f,
};

// Old diagnostic shape retained for comparison only; it can look gait-like.
constexpr std::array<float, NUM_JOINTS> kLegacyShape = {
     0.0f,  0.0f,  0.0f,  0.0f,
     0.7f, -0.7f,  0.7f, -0.7f,
    -1.0f,  1.0f, -1.0f,  1.0f,
};

enum class SquatShape {
    Squat,
    Reverse,
    Same,
    Legacy,
    Custom,
};

struct Options {
    float amp = 0.25f;
    float freq = 0.5f;
    float kp = 20.0f;
    float kd = 1.0f;
    float torque_limit = 17.0f;
    double duration = 10.0;
    double stand_duration = 2.0;
    double hold_duration = 0.5;
    double loop_hz = 200.0;
    bool disable_on_exit = false;
    bool step_profile = false;
    bool abort_on_offline = true;
    SquatShape shape = SquatShape::Squat;
    std::array<float, NUM_LEGS> hipf_signs = {1.0f, 1.0f, -1.0f, -1.0f};
    std::array<float, NUM_LEGS> knee_signs = {-1.0f, -1.0f, 1.0f, 1.0f};
    std::string log_dir = "diagnostics_logs";
};

struct JointStats {
    int samples = 0;
    int offline_samples = 0;
    int offline_events = 0;
    bool was_online = true;
    float max_abs_torque = 0.0f;
    float max_abs_tracking_error = 0.0f;
};

void SignalHandler(int) {
    g_running.store(false);
}

float ClampJoint(int joint, float value) {
    return std::max(kPositionLimits[joint].first, std::min(kPositionLimits[joint].second, value));
}

std::array<float, NUM_JOINTS> ClampPose(const std::array<float, NUM_JOINTS>& pose) {
    std::array<float, NUM_JOINTS> out = pose;
    for (int i = 0; i < NUM_JOINTS; ++i) out[i] = ClampJoint(i, out[i]);
    return out;
}

std::string Timestamp() {
    auto now = std::chrono::system_clock::now();
    auto tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

void Usage(const char* prog) {
    std::cout
        << "motor_squat_diag " << VERSION << "\n"
        << "Usage: " << prog << " [options]\n\n"
        << "Runs a whole-body stand-up + squat diagnostic through DogDriver.\n\n"
        << "Options:\n"
        << "  --amp A              Squat pose scale rad (default 0.25)\n"
        << "  --freq F             Squat frequency Hz (default 0.5)\n"
        << "  --kp K --kd D        MIT gains, deployment default 20/1.0\n"
        << "  --torque-limit T     Stored torque limit field (default 17)\n"
        << "  --duration SEC       Squat duration seconds (default 10)\n"
        << "  --stand-duration SEC Stand interpolation duration (default 2)\n"
        << "  --hold-duration SEC  Hold zero pose before/after squat (default 0.5)\n"
        << "  --loop-hz HZ         Command/log rate, deployment-like default 200\n"
        << "  --profile sine|step  Smooth squat or abrupt up/down target (default sine)\n"
        << "  --shape squat|reverse|same|legacy  Squat joint signs (default squat)\n"
        << "  --hipf-signs a,b,c,d Override LF,LR,RF,RR HipF signs\n"
        << "  --knee-signs a,b,c,d Override LF,LR,RF,RR Knee signs\n"
        << "  --no-abort-on-offline Continue even if a joint goes offline\n"
        << "  --disable-on-exit    Disable all motors at exit; use only when supported\n"
        << "  --log-dir DIR        Log directory (default diagnostics_logs)\n";
}

bool ParseFloat(const char* text, float& out) {
    try {
        out = std::stof(text);
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseDouble(const char* text, double& out) {
    try {
        out = std::stod(text);
        return true;
    } catch (...) {
        return false;
    }
}

bool ParseSignList(const char* text, std::array<float, NUM_LEGS>& out) {
    std::string s(text);
    std::replace(s.begin(), s.end(), ',', ' ');
    std::istringstream iss(s);
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!(iss >> out[i])) return false;
        if (out[i] > 0.0f) out[i] = 1.0f;
        else if (out[i] < 0.0f) out[i] = -1.0f;
        else return false;
    }
    std::string extra;
    return !(iss >> extra);
}

bool ParseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto need_value = [&](const char* name) {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << std::endl;
                return false;
            }
            return true;
        };

        if (arg == "--amp") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.amp)) return false;
        } else if (arg == "--freq") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.freq)) return false;
        } else if (arg == "--kp") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.kp)) return false;
        } else if (arg == "--kd") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.kd)) return false;
        } else if (arg == "--torque-limit") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.torque_limit)) return false;
        } else if (arg == "--duration") {
            if (!need_value(arg.c_str()) || !ParseDouble(argv[++i], opt.duration)) return false;
        } else if (arg == "--stand-duration") {
            if (!need_value(arg.c_str()) || !ParseDouble(argv[++i], opt.stand_duration)) return false;
        } else if (arg == "--hold-duration") {
            if (!need_value(arg.c_str()) || !ParseDouble(argv[++i], opt.hold_duration)) return false;
        } else if (arg == "--loop-hz") {
            if (!need_value(arg.c_str()) || !ParseDouble(argv[++i], opt.loop_hz)) return false;
        } else if (arg == "--profile") {
            if (!need_value(arg.c_str())) return false;
            std::string profile = argv[++i];
            if (profile == "sine") opt.step_profile = false;
            else if (profile == "step") opt.step_profile = true;
            else {
                std::cerr << "--profile must be sine or step" << std::endl;
                return false;
            }
        } else if (arg == "--shape") {
            if (!need_value(arg.c_str())) return false;
            std::string shape = argv[++i];
            if (shape == "squat") opt.shape = SquatShape::Squat;
            else if (shape == "reverse") opt.shape = SquatShape::Reverse;
            else if (shape == "same") opt.shape = SquatShape::Same;
            else if (shape == "legacy") opt.shape = SquatShape::Legacy;
            else {
                std::cerr << "--shape must be squat, reverse, same, or legacy" << std::endl;
                return false;
            }
        } else if (arg == "--hipf-signs") {
            if (!need_value(arg.c_str())) return false;
            if (!ParseSignList(argv[++i], opt.hipf_signs)) {
                std::cerr << "--hipf-signs must be four nonzero values, e.g. 1,1,-1,-1" << std::endl;
                return false;
            }
            opt.shape = SquatShape::Custom;
        } else if (arg == "--knee-signs") {
            if (!need_value(arg.c_str())) return false;
            if (!ParseSignList(argv[++i], opt.knee_signs)) {
                std::cerr << "--knee-signs must be four nonzero values, e.g. -1,-1,1,1" << std::endl;
                return false;
            }
            opt.shape = SquatShape::Custom;
        } else if (arg == "--disable-on-exit") {
            opt.disable_on_exit = true;
        } else if (arg == "--no-abort-on-offline") {
            opt.abort_on_offline = false;
        } else if (arg == "--log-dir") {
            if (!need_value(arg.c_str())) return false;
            opt.log_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            return false;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            return false;
        }
    }

    if (opt.amp < 0 || opt.freq <= 0 || opt.kp < 0 || opt.kd < 0 ||
        opt.torque_limit < 0 || opt.duration <= 0 || opt.stand_duration < 0 ||
        opt.hold_duration < 0 || opt.loop_hz <= 0) {
        std::cerr << "Invalid numeric option" << std::endl;
        return false;
    }
    return true;
}

std::array<float, NUM_JOINTS> ShapeVector(const Options& opt) {
    if (opt.shape == SquatShape::Reverse) return kReverseSquatShape;
    if (opt.shape == SquatShape::Same) return kSameSignShape;
    if (opt.shape == SquatShape::Legacy) return kLegacyShape;
    if (opt.shape == SquatShape::Custom) {
        std::array<float, NUM_JOINTS> custom{};
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            custom[4 + leg] = 0.7f * opt.hipf_signs[leg];
            custom[8 + leg] = opt.knee_signs[leg];
        }
        return custom;
    }
    return kSquatShape;
}

const char* ShapeName(SquatShape shape) {
    if (shape == SquatShape::Reverse) return "reverse";
    if (shape == SquatShape::Same) return "same";
    if (shape == SquatShape::Legacy) return "legacy";
    if (shape == SquatShape::Custom) return "custom";
    return "squat";
}

std::string FormatSigns(const std::array<float, NUM_LEGS>& signs) {
    std::ostringstream oss;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (i > 0) oss << ",";
        oss << (signs[i] > 0.0f ? "+1" : "-1");
    }
    return oss.str();
}

std::ofstream OpenLog(const Options& opt) {
    std::filesystem::create_directories(opt.log_dir);
    std::filesystem::path path = std::filesystem::path(opt.log_dir) /
        ("squat_" + Timestamp() + ".csv");
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("failed to open log file: " + path.string());
    }
    std::cout << "Logging to " << path.string() << std::endl;
    out << "phase,t,joint,joint_name,desired,position,velocity,torque,online\n";
    return out;
}

void ConfigureDriver(DogDriver& driver, const Options& opt) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        driver.SetMITParams(i, opt.kp, opt.kd);
        driver.SetTorqueLimit(i, opt.torque_limit);
        driver.EnableJoint(i);
        driver.EnableAutoReport(i);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

bool WriteLogRows(std::ofstream& out, DogDriver& driver, const std::string& phase,
                  double t_sec, const std::array<float, NUM_JOINTS>& desired,
                  std::array<JointStats, NUM_JOINTS>& stats) {
    auto state = driver.GetJointStates();
    bool all_online = true;
    out << std::fixed << std::setprecision(6);
    for (int i = 0; i < NUM_JOINTS; ++i) {
        bool online = driver.IsJointOnline(i);
        if (!online) all_online = false;
        auto& s = stats[i];
        s.samples++;
        if (!online) {
            s.offline_samples++;
            if (s.was_online) s.offline_events++;
        }
        s.was_online = online;
        s.max_abs_torque = std::max(s.max_abs_torque, std::abs(state.torque[i]));
        s.max_abs_tracking_error = std::max(
            s.max_abs_tracking_error, std::abs(desired[i] - state.position[i]));

        out << phase << "," << t_sec << "," << i << "," << kJointNames[i] << ","
            << desired[i] << "," << state.position[i] << ","
            << state.velocity[i] << "," << state.torque[i] << ","
            << (online ? 1 : 0) << "\n";
    }
    return all_online;
}

bool SendAndLog(DogDriver& driver, std::ofstream& out, const std::string& phase,
                double t_sec, const std::array<float, NUM_JOINTS>& desired,
                std::array<JointStats, NUM_JOINTS>& stats) {
    driver.SetAllJointPositions(desired);
    return WriteLogRows(out, driver, phase, t_sec, desired, stats);
}

bool RunForDuration(DogDriver& driver, std::ofstream& out, const Options& opt,
                    const std::string& phase, double duration,
                    const std::array<float, NUM_JOINTS>& desired,
                    std::array<JointStats, NUM_JOINTS>& stats) {
    const auto tick = std::chrono::duration<double>(1.0 / opt.loop_hz);
    auto start = Clock::now();
    auto next_tick = Clock::now();
    while (g_running.load()) {
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        if (t >= duration) break;
        bool all_online = SendAndLog(driver, out, phase, t, desired, stats);
        if (opt.abort_on_offline && !all_online) {
            std::cerr << "ERROR: joint offline during " << phase << ", aborting motion." << std::endl;
            return false;
        }
        next_tick += std::chrono::duration_cast<Clock::duration>(tick);
        std::this_thread::sleep_until(next_tick);
    }
    return true;
}

bool RunStand(DogDriver& driver, std::ofstream& out, const Options& opt,
              std::array<JointStats, NUM_JOINTS>& stats) {
    auto start_state = driver.GetJointStates();
    std::array<float, NUM_JOINTS> zero{};
    const auto tick = std::chrono::duration<double>(1.0 / opt.loop_hz);
    auto start = Clock::now();
    auto next_tick = Clock::now();

    while (g_running.load()) {
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        if (t >= opt.stand_duration) break;
        float alpha = opt.stand_duration > 0.0 ? static_cast<float>(t / opt.stand_duration) : 1.0f;
        alpha = std::max(0.0f, std::min(1.0f, alpha));
        std::array<float, NUM_JOINTS> target{};
        for (int i = 0; i < NUM_JOINTS; ++i)
            target[i] = start_state.position[i] * (1.0f - alpha);
        bool all_online = SendAndLog(driver, out, "stand", t, ClampPose(target), stats);
        if (opt.abort_on_offline && !all_online) {
            std::cerr << "ERROR: joint offline during stand, aborting motion." << std::endl;
            return false;
        }
        next_tick += std::chrono::duration_cast<Clock::duration>(tick);
        std::this_thread::sleep_until(next_tick);
    }
    return SendAndLog(driver, out, "stand", opt.stand_duration, zero, stats) || !opt.abort_on_offline;
}

std::array<float, NUM_JOINTS> SquatTarget(const Options& opt, double t_sec) {
    float phase;
    if (opt.step_profile) {
        double half_period = 0.5 / opt.freq;
        int phase_index = static_cast<int>(std::floor(t_sec / half_period));
        phase = (phase_index % 2 == 0) ? 1.0f : 0.0f;
    } else {
        phase = 0.5f * (1.0f - std::cos(2.0f * PI * opt.freq * static_cast<float>(t_sec)));
    }

    std::array<float, NUM_JOINTS> target{};
    const auto shape = ShapeVector(opt);
    for (int i = 0; i < NUM_JOINTS; ++i)
        target[i] = opt.amp * phase * shape[i];
    return ClampPose(target);
}

bool RunSquat(DogDriver& driver, std::ofstream& out, const Options& opt,
              std::array<JointStats, NUM_JOINTS>& stats) {
    const auto tick = std::chrono::duration<double>(1.0 / opt.loop_hz);
    auto start = Clock::now();
    auto next_tick = Clock::now();
    while (g_running.load()) {
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        if (t >= opt.duration) break;
        auto target = SquatTarget(opt, t);
        bool all_online = SendAndLog(driver, out, "squat", t, target, stats);
        if (opt.abort_on_offline && !all_online) {
            std::cerr << "ERROR: joint offline during squat, aborting motion." << std::endl;
            return false;
        }
        next_tick += std::chrono::duration_cast<Clock::duration>(tick);
        std::this_thread::sleep_until(next_tick);
    }
    return true;
}

void PrintSummary(const std::array<JointStats, NUM_JOINTS>& stats) {
    std::cout << "Summary:\n";
    std::cout << " J " << std::setw(8) << "Joint"
              << " " << std::setw(7) << "samples"
              << " " << std::setw(7) << "off_evt"
              << " " << std::setw(8) << "off_samp"
              << " " << std::setw(10) << "max_tau"
              << " " << std::setw(12) << "max_err"
              << "\n";
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::cout << std::setw(2) << i << " "
                  << std::setw(8) << kJointNames[i] << " "
                  << std::setw(7) << stats[i].samples << " "
                  << std::setw(7) << stats[i].offline_events << " "
                  << std::setw(8) << stats[i].offline_samples << " "
                  << std::setw(10) << std::fixed << std::setprecision(4) << stats[i].max_abs_torque << " "
                  << std::setw(12) << std::setprecision(5) << stats[i].max_abs_tracking_error
                  << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    Options opt;
    if (!ParseArgs(argc, argv, opt)) {
        Usage(argv[0]);
        return 1;
    }

    try {
        std::cout << "motor_squat_diag " << VERSION << std::endl;
        std::cout << "kp=" << opt.kp << " kd=" << opt.kd
                  << " amp=" << opt.amp << " freq=" << opt.freq
                  << " profile=" << (opt.step_profile ? "step" : "sine")
                  << " shape=" << ShapeName(opt.shape)
                  << " loop_hz=" << opt.loop_hz << std::endl;
        if (opt.shape == SquatShape::Custom) {
            std::cout << "custom hipf_signs(LF,LR,RF,RR)=" << FormatSigns(opt.hipf_signs)
                      << " knee_signs=" << FormatSigns(opt.knee_signs) << std::endl;
        }

        DogDriver driver;
        ConfigureDriver(driver, opt);

        auto out = OpenLog(opt);
        std::array<JointStats, NUM_JOINTS> stats{};

        bool ok = RunStand(driver, out, opt, stats);
        std::array<float, NUM_JOINTS> zero{};
        if (ok) ok = RunForDuration(driver, out, opt, "hold_before", opt.hold_duration, zero, stats);
        if (ok) ok = RunSquat(driver, out, opt, stats);
        if (ok) ok = RunForDuration(driver, out, opt, "hold_after", opt.hold_duration, zero, stats);

        PrintSummary(stats);

        if (opt.disable_on_exit) {
            driver.DisableAll();
            std::cout << "Motors disabled on exit." << std::endl;
        } else {
            driver.SetAllJointPositions(zero);
            std::cout << "Holding zero pose on exit. Use --disable-on-exit only when supported." << std::endl;
        }
        return (g_running.load() && ok) ? 0 : 2;
    } catch (const std::exception& ex) {
        std::cerr << "[motor_squat_diag] ERROR: " << ex.what() << std::endl;
        return 1;
    }
}

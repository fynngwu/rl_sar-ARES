#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "can_interface.hpp"
#include "robstride.hpp"

namespace {

using Clock = std::chrono::steady_clock;

constexpr int NUM_JOINTS = 12;
constexpr int NUM_LEGS = 4;
constexpr uint8_t HOST_ID = 0xFD;
constexpr float MAX_TORQUE = 17.0f;
constexpr float MAX_SPEED = 44.0f;
constexpr float DAMPING_KP = 0.0f;
constexpr float DAMPING_KD = 10.0f;
constexpr float KNEE_GEAR_RATIO = 1.667f;
constexpr float PI = 3.14159265358979323846f;

constexpr std::array<int, NUM_JOINTS> kMotorIds = {
    1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15
};

constexpr std::array<float, NUM_JOINTS> kOffsets = {
    0.37f, -0.37f, -0.37f, 0.37f,
    0.13f,  0.13f, -0.13f, -0.13f,
    1.76702f, 1.76702f, -1.76702f, -1.76702f
};

constexpr std::array<float, NUM_JOINTS> kJointDirection = {
    -1, -1, -1, -1, -1, -1, -1, -1, +1, +1, +1, +1
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

constexpr std::array<const char*, NUM_JOINTS> kJointNames = {
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee"
};

constexpr std::array<const char*, NUM_LEGS> kCanNames = {"can0", "can1", "can2", "can3"};

volatile sig_atomic_t g_running = 1;
void SignalHandler(int) { g_running = 0; }

int CanIndex(int joint) { return joint % NUM_LEGS; }

float MotorToJointPos(int joint, float motor_pos) {
    float pos = kJointDirection[joint] * (motor_pos - kOffsets[joint]);
    if (joint >= 8) pos /= KNEE_GEAR_RATIO;
    return pos;
}

float MotorToJointVel(int joint, float motor_vel) {
    float vel = kJointDirection[joint] * motor_vel;
    if (joint >= 8) vel /= KNEE_GEAR_RATIO;
    return vel;
}

float MotorToJointTorque(int joint, float motor_torque) {
    return kJointDirection[joint] * motor_torque;
}

float JointToMotorPos(int joint, float joint_pos) {
    float motor_delta = joint_pos;
    if (joint >= 8) motor_delta *= KNEE_GEAR_RATIO;
    return kJointDirection[joint] * motor_delta + kOffsets[joint];
}

float ClampJointPos(int joint, float pos) {
    return std::max(kPositionLimits[joint].first, std::min(kPositionLimits[joint].second, pos));
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

std::string Hex8(uint8_t value) {
    std::ostringstream oss;
    oss << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(value);
    return oss.str();
}

struct Options {
    std::vector<int> joints;
    int ref_joint = -1;
    float amp = 0.05f;
    float freq = 0.5f;
    float min_freq = 0.1f;
    float max_freq = 2.0f;
    float kp = 10.0f;
    float kd = 0.5f;
    float torque_limit = 8.0f;
    double duration = 5.0;
    double loop_hz = 100.0;
    bool clear = false;
    std::string log_dir = "diagnostics_logs";
};

void Usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " <command> [options]\n\n"
        << "Commands:\n"
        << "  status                         Print online/state/fault table\n"
        << "  fault-log [--duration SEC]     Log state and current error code to JSONL\n"
        << "  single --joint J [...]         Low-risk single-joint sine test\n"
        << "  chirp --joint J [...]          Low-risk single-joint chirp test\n"
        << "  compare --joint BAD --ref GOOD Run the same sine test on two joints\n"
        << "  damping [--joint J]            Send damping command (kp=0,kd=10)\n"
        << "  disable [--joint J]            Disable selected motors\n"
        << "  errors [--joint J] [--clear]   Print motor error code, optionally clear\n\n"
        << "Options:\n"
        << "  --joint, -j N      Joint index 0-11; repeat or omit for all where valid\n"
        << "  --ref N            Reference joint for compare\n"
        << "  --amp A            Joint-space sine/chirp amplitude rad (default 0.05)\n"
        << "  --freq F           Single-test frequency Hz (default 0.5)\n"
        << "  --min-freq F       Chirp start frequency Hz (default 0.1)\n"
        << "  --max-freq F       Chirp end frequency Hz (default 2.0)\n"
        << "  --kp K --kd D      MIT gains (default 10, 0.5)\n"
        << "  --torque-limit T   Stored MIT torque_limit field (default 8; current driver does not write 0x700B)\n"
        << "  --duration SEC     Test duration seconds (default 5)\n"
        << "  --loop-hz HZ       Command/log rate (default 100)\n"
        << "  --log-dir DIR      Log directory (default diagnostics_logs)\n"
        << "  --clear, -c        Clear errors for errors command\n\n"
        << "Joint layout:\n"
        << "  0:LF_HipA  1:LR_HipA  2:RF_HipA  3:RR_HipA\n"
        << "  4:LF_HipF  5:LR_HipF  6:RF_HipF  7:RR_HipF\n"
        << "  8:LF_Knee  9:LR_Knee 10:RF_Knee 11:RR_Knee\n";
}

bool ParseInt(const char* text, int& out) {
    try {
        out = std::stoi(text);
        return true;
    } catch (...) {
        return false;
    }
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

bool ParseArgs(int argc, char** argv, Options& opt) {
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        auto need_value = [&](const char* name) {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << std::endl;
                return false;
            }
            return true;
        };

        if (arg == "--joint" || arg == "-j") {
            if (!need_value(arg.c_str())) return false;
            int joint = -1;
            if (!ParseInt(argv[++i], joint) || joint < 0 || joint >= NUM_JOINTS) {
                std::cerr << "Joint index must be 0-11" << std::endl;
                return false;
            }
            opt.joints.push_back(joint);
        } else if (arg == "--ref") {
            if (!need_value(arg.c_str())) return false;
            if (!ParseInt(argv[++i], opt.ref_joint) || opt.ref_joint < 0 || opt.ref_joint >= NUM_JOINTS) {
                std::cerr << "--ref must be 0-11" << std::endl;
                return false;
            }
        } else if (arg == "--amp") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.amp)) return false;
        } else if (arg == "--freq") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.freq)) return false;
        } else if (arg == "--min-freq") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.min_freq)) return false;
        } else if (arg == "--max-freq") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.max_freq)) return false;
        } else if (arg == "--kp") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.kp)) return false;
        } else if (arg == "--kd") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.kd)) return false;
        } else if (arg == "--torque-limit") {
            if (!need_value(arg.c_str()) || !ParseFloat(argv[++i], opt.torque_limit)) return false;
        } else if (arg == "--duration") {
            if (!need_value(arg.c_str()) || !ParseDouble(argv[++i], opt.duration)) return false;
        } else if (arg == "--loop-hz") {
            if (!need_value(arg.c_str()) || !ParseDouble(argv[++i], opt.loop_hz)) return false;
        } else if (arg == "--log-dir") {
            if (!need_value(arg.c_str())) return false;
            opt.log_dir = argv[++i];
        } else if (arg == "--clear" || arg == "-c") {
            opt.clear = true;
        } else if (arg == "--help" || arg == "-h") {
            return false;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            return false;
        }
    }
    if (opt.duration <= 0 || opt.loop_hz <= 0 || opt.freq <= 0 ||
        opt.min_freq <= 0 || opt.max_freq < opt.min_freq || opt.amp < 0 ||
        opt.kp < 0 || opt.kd < 0 || opt.torque_limit < 0) {
        std::cerr << "Invalid numeric option" << std::endl;
        return false;
    }
    return true;
}

std::vector<int> AllJointsIfEmpty(std::vector<int> joints) {
    if (!joints.empty()) return joints;
    for (int i = 0; i < NUM_JOINTS; ++i) joints.push_back(i);
    return joints;
}

int RequireSingleJoint(const Options& opt) {
    if (opt.joints.size() != 1) {
        std::cerr << "This command requires exactly one --joint" << std::endl;
        return -1;
    }
    return opt.joints[0];
}

class MotorContext {
public:
    void Init(const std::vector<int>& joints) {
        ctrl_ = std::make_shared<RobstrideController>();
        for (int joint : joints) {
            int can = CanIndex(joint);
            if (!can_opened_[can]) {
                can_ifaces_[can] = std::make_shared<CANInterface>(kCanNames[can]);
                ctrl_->BindCAN(can_ifaces_[can]);
                can_opened_[can] = true;
            }
            auto info = std::make_unique<RobstrideController::MotorInfo>();
            info->motor_id = kMotorIds[joint];
            info->host_id = HOST_ID;
            info->max_torque = MAX_TORQUE;
            info->max_speed = MAX_SPEED;
            info->max_kp = 500.0f;
            info->max_kd = 5.0f;
            motor_idx_[joint] = ctrl_->BindMotor(kCanNames[can], std::move(info));
            bound_[joint] = true;
        }
    }

    std::shared_ptr<RobstrideController> ctrl() { return ctrl_; }
    int motor_idx(int joint) const { return motor_idx_[joint]; }

private:
    std::array<std::shared_ptr<CANInterface>, NUM_LEGS> can_ifaces_{};
    std::shared_ptr<RobstrideController> ctrl_;
    std::array<int, NUM_JOINTS> motor_idx_{};
    std::array<bool, NUM_JOINTS> bound_{};
    std::array<bool, NUM_LEGS> can_opened_{};
};

void EnableReports(MotorContext& ctx, const std::vector<int>& joints) {
    for (int joint : joints) ctx.ctrl()->EnableAutoReport(ctx.motor_idx(joint));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void DisableReports(MotorContext& ctx, const std::vector<int>& joints) {
    for (int joint : joints) ctx.ctrl()->DisableAutoReport(ctx.motor_idx(joint));
}

MIT_params MakeParams(const Options& opt) {
    return MIT_params{opt.kp, opt.kd, MAX_SPEED, opt.torque_limit};
}

void PrintFaultRow(MotorContext& ctx, int joint) {
    auto st = ctx.ctrl()->GetMotorState(ctx.motor_idx(joint));
    auto err = ctx.ctrl()->GetMotorError(ctx.motor_idx(joint));
    bool online = ctx.ctrl()->IsMotorOnline(ctx.motor_idx(joint));
    std::cout << std::setw(2) << joint << " "
              << std::setw(8) << kJointNames[joint] << " "
              << std::setw(5) << kMotorIds[joint] << " "
              << std::setw(4) << kCanNames[CanIndex(joint)] << " "
              << std::setw(6) << (online ? "YES" : "NO") << " "
              << std::setw(10) << std::fixed << std::setprecision(4) << MotorToJointPos(joint, st.position) << " "
              << std::setw(10) << MotorToJointVel(joint, st.velocity) << " "
              << std::setw(10) << MotorToJointTorque(joint, st.torque) << " "
              << std::setw(8) << Hex8(err.error_code) << " "
              << std::setw(8) << static_cast<int>(err.pattern)
              << std::endl;
}

void PrintFaultHeader() {
    std::cout << " J " << std::setw(8) << "Joint" << " "
              << std::setw(5) << "MID" << " "
              << std::setw(4) << "CAN" << " "
              << std::setw(6) << "On" << " "
              << std::setw(10) << "Pos" << " "
              << std::setw(10) << "Vel" << " "
              << std::setw(10) << "Trq" << " "
              << std::setw(8) << "Err" << " "
              << std::setw(8) << "Pattern"
              << std::endl;
}

std::ofstream OpenLog(const Options& opt, const std::string& prefix, const std::string& ext) {
    std::filesystem::create_directories(opt.log_dir);
    std::filesystem::path path = std::filesystem::path(opt.log_dir) / (prefix + "_" + Timestamp() + ext);
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("failed to open log file: " + path.string());
    }
    std::cout << "Logging to " << path.string() << std::endl;
    return out;
}

void WriteJsonLine(std::ofstream& out, MotorContext& ctx, int joint, double t_sec) {
    auto st = ctx.ctrl()->GetMotorState(ctx.motor_idx(joint));
    auto err = ctx.ctrl()->GetMotorError(ctx.motor_idx(joint));
    bool online = ctx.ctrl()->IsMotorOnline(ctx.motor_idx(joint));
    out << std::fixed << std::setprecision(6)
        << "{\"t\":" << t_sec
        << ",\"joint\":" << joint
        << ",\"joint_name\":\"" << kJointNames[joint] << "\""
        << ",\"motor_id\":" << kMotorIds[joint]
        << ",\"can\":\"" << kCanNames[CanIndex(joint)] << "\""
        << ",\"online\":" << (online ? "true" : "false")
        << ",\"pos\":" << MotorToJointPos(joint, st.position)
        << ",\"vel\":" << MotorToJointVel(joint, st.velocity)
        << ",\"torque\":" << MotorToJointTorque(joint, st.torque)
        << ",\"error_code\":" << static_cast<int>(err.error_code)
        << ",\"pattern\":" << static_cast<int>(err.pattern)
        << "}\n";
}

void WriteCsvHeader(std::ofstream& out) {
    out << "test,t,joint,joint_name,motor_id,can,desired,position,velocity,torque,"
           "online,error_code,pattern\n";
}

void WriteCsvRow(std::ofstream& out, MotorContext& ctx, const std::string& test,
                 int joint, double t_sec, float desired) {
    auto st = ctx.ctrl()->GetMotorState(ctx.motor_idx(joint));
    auto err = ctx.ctrl()->GetMotorError(ctx.motor_idx(joint));
    bool online = ctx.ctrl()->IsMotorOnline(ctx.motor_idx(joint));
    out << std::fixed << std::setprecision(6)
        << test << "," << t_sec << "," << joint << "," << kJointNames[joint] << ","
        << kMotorIds[joint] << "," << kCanNames[CanIndex(joint)] << ","
        << desired << "," << MotorToJointPos(joint, st.position) << ","
        << MotorToJointVel(joint, st.velocity) << ","
        << MotorToJointTorque(joint, st.torque) << ","
        << (online ? 1 : 0) << ","
        << static_cast<int>(err.error_code) << ","
        << static_cast<int>(err.pattern) << "\n";
}

int CmdStatus(const Options& opt) {
    auto joints = AllJointsIfEmpty(opt.joints);
    MotorContext ctx;
    ctx.Init(joints);
    EnableReports(ctx, joints);
    PrintFaultHeader();
    for (int joint : joints) PrintFaultRow(ctx, joint);
    DisableReports(ctx, joints);
    return 0;
}

int CmdErrors(const Options& opt) {
    auto joints = AllJointsIfEmpty(opt.joints);
    MotorContext ctx;
    ctx.Init(joints);
    EnableReports(ctx, joints);
    PrintFaultHeader();
    for (int joint : joints) PrintFaultRow(ctx, joint);
    if (opt.clear) {
        std::cout << "Clearing selected motor errors..." << std::endl;
        for (int joint : joints) {
            ctx.ctrl()->ClearMotor(ctx.motor_idx(joint));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    DisableReports(ctx, joints);
    return 0;
}

int CmdFaultLog(const Options& opt) {
    auto joints = AllJointsIfEmpty(opt.joints);
    MotorContext ctx;
    ctx.Init(joints);
    EnableReports(ctx, joints);
    auto out = OpenLog(opt, "fault_log", ".jsonl");
    auto start = Clock::now();
    const auto tick = std::chrono::duration<double>(1.0 / opt.loop_hz);
    auto next_tick = Clock::now();
    while (g_running) {
        double t = std::chrono::duration<double>(Clock::now() - start).count();
        if (t >= opt.duration) break;
        for (int joint : joints) WriteJsonLine(out, ctx, joint, t);
        next_tick += std::chrono::duration_cast<Clock::duration>(tick);
        std::this_thread::sleep_until(next_tick);
    }
    DisableReports(ctx, joints);
    return g_running ? 0 : 2;
}

bool WaitForOnline(MotorContext& ctx, int joint, std::chrono::milliseconds timeout) {
    auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        if (ctx.ctrl()->IsMotorOnline(ctx.motor_idx(joint))) return true;
        ctx.ctrl()->EnableAutoReport(ctx.motor_idx(joint));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return ctx.ctrl()->IsMotorOnline(ctx.motor_idx(joint));
}

bool ConfigureJoint(MotorContext& ctx, int joint, const Options& opt) {
    ctx.ctrl()->SetMITParams(ctx.motor_idx(joint), MakeParams(opt));
    ctx.ctrl()->EnableMotor(ctx.motor_idx(joint));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ctx.ctrl()->EnableAutoReport(ctx.motor_idx(joint));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ctx.ctrl()->EnableAutoReport(ctx.motor_idx(joint));
    if (!WaitForOnline(ctx, joint, std::chrono::milliseconds(800))) {
        std::cerr << "ERROR: " << kJointNames[joint]
                  << " motor_id=" << kMotorIds[joint]
                  << " on " << kCanNames[CanIndex(joint)]
                  << " is offline after enable/auto-report; aborting test."
                  << std::endl;
        return false;
    }
    return true;
}

int RunWaveTest(const Options& opt, const std::vector<std::pair<std::string, int>>& tests, bool chirp) {
    std::vector<int> joints;
    for (const auto& test : tests) joints.push_back(test.second);
    MotorContext ctx;
    ctx.Init(joints);
    auto out = OpenLog(opt, chirp ? "chirp" : "single", ".csv");
    WriteCsvHeader(out);

    for (const auto& test : tests) {
        const std::string& label = test.first;
        int joint = test.second;
        if (!ConfigureJoint(ctx, joint, opt)) {
            ctx.ctrl()->DisableMotor(ctx.motor_idx(joint));
            ctx.ctrl()->DisableAutoReport(ctx.motor_idx(joint));
            return 1;
        }
        auto st = ctx.ctrl()->GetMotorState(ctx.motor_idx(joint));
        float center = ClampJointPos(joint, MotorToJointPos(joint, st.position));
        std::cout << label << ": " << kJointNames[joint] << " center="
                  << center << " amp=" << opt.amp << " kp=" << opt.kp
                  << " kd=" << opt.kd << std::endl;

        auto start = Clock::now();
        const auto tick = std::chrono::duration<double>(1.0 / opt.loop_hz);
        auto next_tick = Clock::now();
        while (g_running) {
            double t = std::chrono::duration<double>(Clock::now() - start).count();
            if (t >= opt.duration) break;
            double phase;
            if (chirp) {
                phase = 2.0 * PI * (opt.min_freq * t +
                    ((opt.max_freq - opt.min_freq) / (2.0 * opt.duration)) * t * t);
            } else {
                phase = 2.0 * PI * opt.freq * t;
            }
            if (!ctx.ctrl()->IsMotorOnline(ctx.motor_idx(joint))) {
                std::cerr << "ERROR: " << kJointNames[joint]
                          << " motor_id=" << kMotorIds[joint]
                          << " went offline during " << label
                          << "; stopping commands." << std::endl;
                ctx.ctrl()->DisableMotor(ctx.motor_idx(joint));
                ctx.ctrl()->DisableAutoReport(ctx.motor_idx(joint));
                return 1;
            }
            float desired = ClampJointPos(joint, center + opt.amp * static_cast<float>(std::sin(phase)));
            ctx.ctrl()->SendMITCommand(ctx.motor_idx(joint), JointToMotorPos(joint, desired));
            WriteCsvRow(out, ctx, label, joint, t, desired);
            next_tick += std::chrono::duration_cast<Clock::duration>(tick);
            std::this_thread::sleep_until(next_tick);
        }
        ctx.ctrl()->DisableMotor(ctx.motor_idx(joint));
        ctx.ctrl()->DisableAutoReport(ctx.motor_idx(joint));
        if (!g_running) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    return g_running ? 0 : 2;
}

int CmdSingle(const Options& opt) {
    int joint = RequireSingleJoint(opt);
    if (joint < 0) return 1;
    return RunWaveTest(opt, {{"single", joint}}, false);
}

int CmdChirp(const Options& opt) {
    int joint = RequireSingleJoint(opt);
    if (joint < 0) return 1;
    return RunWaveTest(opt, {{"chirp", joint}}, true);
}

int CmdCompare(const Options& opt) {
    int joint = RequireSingleJoint(opt);
    if (joint < 0 || opt.ref_joint < 0) {
        std::cerr << "compare requires --joint BAD --ref GOOD" << std::endl;
        return 1;
    }
    return RunWaveTest(opt, {{"suspect", joint}, {"reference", opt.ref_joint}}, false);
}

int CmdDamping(const Options& opt) {
    auto joints = AllJointsIfEmpty(opt.joints);
    MotorContext ctx;
    ctx.Init(joints);
    EnableReports(ctx, joints);
    MIT_params params{DAMPING_KP, DAMPING_KD, MAX_SPEED, MAX_TORQUE};
    for (int joint : joints) {
        auto st = ctx.ctrl()->GetMotorState(ctx.motor_idx(joint));
        ctx.ctrl()->SetMITParams(ctx.motor_idx(joint), params);
        ctx.ctrl()->EnableMotor(ctx.motor_idx(joint));
        ctx.ctrl()->SendMITCommand(ctx.motor_idx(joint), st.position);
        std::cout << "Damping " << kJointNames[joint] << " kp=0 kd=10" << std::endl;
    }
    DisableReports(ctx, joints);
    return 0;
}

int CmdDisable(const Options& opt) {
    auto joints = AllJointsIfEmpty(opt.joints);
    MotorContext ctx;
    ctx.Init(joints);
    for (int joint : joints) {
        ctx.ctrl()->DisableMotor(ctx.motor_idx(joint));
        std::cout << "Disabled " << kJointNames[joint]
                  << " motor_id=" << kMotorIds[joint] << std::endl;
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    if (argc < 2) {
        Usage(argv[0]);
        return 1;
    }

    std::string cmd = argv[1];
    if (cmd == "--help" || cmd == "-h" || cmd == "help") {
        Usage(argv[0]);
        return 0;
    }
    Options opt;
    if (!ParseArgs(argc, argv, opt)) {
        Usage(argv[0]);
        return 1;
    }

    try {
        if (cmd == "status") return CmdStatus(opt);
        if (cmd == "fault-log") return CmdFaultLog(opt);
        if (cmd == "single") return CmdSingle(opt);
        if (cmd == "chirp") return CmdChirp(opt);
        if (cmd == "compare") return CmdCompare(opt);
        if (cmd == "damping") return CmdDamping(opt);
        if (cmd == "disable") return CmdDisable(opt);
        if (cmd == "errors") return CmdErrors(opt);
    } catch (const std::exception& ex) {
        std::cerr << "[motor_diag] ERROR: " << ex.what() << std::endl;
        return 1;
    }

    std::cerr << "Unknown command: " << cmd << std::endl;
    Usage(argv[0]);
    return 1;
}

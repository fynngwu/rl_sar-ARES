#include "dog_driver.hpp"
#include "observations.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

constexpr int kNumJoints = DogDriver::NUM_JOINTS;
constexpr double kStepRad = 0.3;
constexpr double kRecordSec = 5.0;
constexpr double kRateHz = 100.0;
constexpr double kStandSec = 2.0;
constexpr const char* kConfigPath = "policy/dream_waq/dream_waq/config.yaml";
constexpr const char* kPolicyKey = "dream_waq/dream_waq";
constexpr const char* kOutputCsv = "tools/kpkd_response_test/logs/dream_waq_step_response.csv";

std::atomic<bool> g_running{true};

void SignalHandler(int)
{
    g_running = false;
}

std::vector<float> LoadScalarOrArray(const YAML::Node& node, size_t n)
{
    if (!node)
        throw std::runtime_error("missing YAML node");
    if (node.IsSequence()) {
        std::vector<float> values;
        for (const auto& item : node)
            values.push_back(item.as<float>());
        if (values.size() != n)
            throw std::runtime_error("YAML array length is not 12");
        return values;
    }
    return std::vector<float>(n, node.as<float>());
}

struct MotorParams {
    std::vector<float> kp;
    std::vector<float> kd;
    std::vector<float> torque;
};

MotorParams LoadMotorParams()
{
    YAML::Node root = YAML::LoadFile(kConfigPath);
    YAML::Node rc = root[kPolicyKey];
    if (!rc)
        throw std::runtime_error(std::string("missing key: ") + kPolicyKey);

    MotorParams p;
    p.kp = LoadScalarOrArray(rc["fixed_kp"], kNumJoints);
    p.kd = LoadScalarOrArray(rc["fixed_kd"], kNumJoints);
    p.torque = LoadScalarOrArray(rc["torque_limits"], kNumJoints);
    return p;
}

void WriteHeader(std::ofstream& out)
{
    out << "time_s";
    for (int i = 0; i < kNumJoints; ++i) out << ",target_j" << i;
    for (int i = 0; i < kNumJoints; ++i) out << ",pos_j" << i;
    for (int i = 0; i < kNumJoints; ++i) out << ",vel_j" << i;
    for (int i = 0; i < kNumJoints; ++i) out << ",torque_j" << i;
    out << '\n';
}

void WriteSample(std::ofstream& out, double t,
                 const std::array<float, kNumJoints>& target,
                 const DogDriver::JointState& state)
{
    out << std::fixed << std::setprecision(6) << t;
    for (float v : target) out << ',' << v;
    for (float v : state.position) out << ',' << v;
    for (float v : state.velocity) out << ',' << v;
    for (float v : state.torque) out << ',' << v;
    out << '\n';
}

void PrintVector(const char* name, const std::vector<float>& values)
{
    std::printf("%s: [", name);
    for (size_t i = 0; i < values.size(); ++i)
        std::printf("%s%.3f", i ? ", " : "", values[i]);
    std::printf("]\n");
}

void PrintJointTable(const DogDriver::JointState& initial,
                     const std::array<float, kNumJoints>& target)
{
    std::printf("\n%-8s %12s %12s %12s\n", "Joint", "Initial", "Target", "Delta");
    std::printf("----------------------------------------------\n");
    for (int i = 0; i < kNumJoints; ++i) {
        std::printf("%-8d %12.4f %12.4f %12.4f\n",
                    i, initial.position[i], target[i], target[i] - initial.position[i]);
    }
}

void ApplyMotorParams(DogDriver& driver, const MotorParams& params)
{
    for (int i = 0; i < kNumJoints; ++i) {
        driver.EnableAutoReport(i);
        driver.SetMITParams(i, params.kp[i], params.kd[i]);
        driver.SetTorqueLimit(i, params.torque[i]);
    }
}

void ApplyDamping(DogDriver& driver)
{
    for (int i = 0; i < kNumJoints; ++i)
        driver.SetMITParams(i, 0.0f, 4.0f);
}

void MoveToStand(DogDriver& driver, const MotorParams& params)
{
    ApplyDamping(driver);
    driver.EnableAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    auto start_state = driver.GetJointStates();
    std::array<float, kNumJoints> stand_target{};

    const auto dt = std::chrono::duration<double>(1.0 / kRateHz);
    int steps = static_cast<int>(kStandSec * kRateHz);
    auto next_tick = std::chrono::steady_clock::now();

    std::printf("Entering STAND over %.1f s...\n", kStandSec);
    for (int step = 0; g_running && step <= steps; ++step) {
        float alpha = static_cast<float>(step) / steps;
        std::array<float, kNumJoints> cmd{};
        for (int i = 0; i < kNumJoints; ++i)
            cmd[i] = start_state.position[i] * (1.0f - alpha) + stand_target[i] * alpha;
        driver.SetAllJointPositions(cmd);
        std::this_thread::sleep_until(next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt));
    }

    ApplyMotorParams(driver, params);
    driver.SetAllJointPositions(stand_target);
}

bool WaitForBButton(DogDriver& driver)
{
    Gamepad gamepad("/dev/input/js0");
    const auto dt = std::chrono::duration<double>(1.0 / kRateHz);
    auto next_tick = std::chrono::steady_clock::now();
    std::array<float, kNumJoints> stand_target{};

    if (!gamepad.IsConnected())
        std::printf("No gamepad at /dev/input/js0. Waiting anyway; connect gamepad or Ctrl-C.\n");
    else
        std::printf("Gamepad connected: %s\n", gamepad.GetName().c_str());
    std::printf("Press B to start 0.3 rad step response recording.\n");

    bool prev_b = false;
    while (g_running) {
        driver.SetAllJointPositions(stand_target);
        bool b = gamepad.IsConnected() && gamepad.GetButton(1);
        if (b && !prev_b)
            return true;
        prev_b = b;
        std::this_thread::sleep_until(next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt));
    }
    return false;
}

}  // namespace

int main()
{
    struct sigaction sa{};
    sa.sa_handler = SignalHandler;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    MotorParams params;
    try {
        params = LoadMotorParams();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load %s: %s\n", kConfigPath, e.what());
        return 1;
    }

    std::filesystem::create_directories(std::filesystem::path(kOutputCsv).parent_path());
    std::ofstream csv(kOutputCsv);
    if (!csv.is_open()) {
        std::fprintf(stderr, "Failed to open output CSV: %s\n", kOutputCsv);
        return 1;
    }
    WriteHeader(csv);

    std::printf("KP/KD response test\n");
    std::printf("Config: %s [%s]\n", kConfigPath, kPolicyKey);
    PrintVector("fixed_kp", params.kp);
    PrintVector("fixed_kd", params.kd);
    PrintVector("torque_limits", params.torque);
    std::printf("Command: target = initial_position + %.3f rad, record %.1f s at %.1f Hz\n",
                kStepRad, kRecordSec, kRateHz);
    std::printf("Output: %s\n\n", kOutputCsv);

    std::printf("Initializing DogDriver...\n");
    DogDriver driver;
    driver.EnableAll();
    ApplyMotorParams(driver, params);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int online = driver.OnlineMotorCount();
    std::printf("Motors online: %d/%d\n", online, kNumJoints);
    if (online == 0) {
        std::fprintf(stderr, "No motors online. Aborting.\n");
        driver.DisableAll();
        return 1;
    }

    MoveToStand(driver, params);
    if (!WaitForBButton(driver)) {
        std::printf("Aborted before test start.\n");
        driver.DisableAll();
        return 1;
    }

    DogDriver::JointState initial = driver.GetJointStates();
    std::array<float, kNumJoints> step_target{};
    for (int i = 0; i < kNumJoints; ++i)
        step_target[i] = initial.position[i] + static_cast<float>(kStepRad);

    PrintJointTable(initial, step_target);
    std::printf("\nB pressed. Sending step now. Press Ctrl-C to abort.\n");

    const auto dt = std::chrono::duration<double>(1.0 / kRateHz);
    auto start = std::chrono::steady_clock::now();
    auto end = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(kRecordSec));
    auto next_tick = start;

    while (g_running && std::chrono::steady_clock::now() <= end) {
        driver.SetAllJointPositions(step_target);
        auto state = driver.GetJointStates();
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        WriteSample(csv, t, step_target, state);
        std::this_thread::sleep_until(next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt));
    }

    csv.flush();
    std::printf("Disabling motors...\n");
    driver.DisableAll();
    std::printf("Done. CSV saved to: %s\n", kOutputCsv);
    return 0;
}

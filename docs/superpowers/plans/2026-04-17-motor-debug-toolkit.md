# Motor Debug CLI Toolkit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a CLI tool `motor_tool` with 5 subcommands for debugging RobStride motors and WitMotion IMU.

**Architecture:** Standalone C++ executable that directly uses low-level driver components (CANInterface + RobstrideController + IMUComponent), bypassing DogDriver's heavy init. Minimal modifications to driver files for error code exposure.

**Tech Stack:** C++17, Linux SocketCAN, POSIX serial, pthreads

---

## Task 1: Add error code storage to RobstrideController

**Files:**
- Modify: `driver/include/robstride.hpp`
- Modify: `driver/src/robstride.cpp`

This adds error_code and pattern fields to MotorData, parses them from CAN responses, and exposes a getter.

- [ ] **Step 1: Add fields to MotorData in robstride.hpp**

In `driver/include/robstride.hpp`, add `error_code` and `pattern` to `MotorData` struct (after `offline_count`):

```cpp
    struct MotorData {
        struct MotorInfo motor_info;
        std::shared_ptr<CANInterface> can_iface;
        int motor_id;
        int host_id;
        bool enabled;
        bool online;
        std::chrono::steady_clock::time_point last_online_time;

        struct MIT_params mit_params;
        struct motor_state state;
        int offline_count;
        uint8_t error_code = 0;
        uint8_t pattern = 0;
    };
```

- [ ] **Step 2: Add MotorError struct and getter declaration to RobstrideController**

In `driver/include/robstride.hpp`, add after `SetZero` declaration:

```cpp
    struct MotorError {
        uint8_t error_code;
        uint8_t pattern;
    };
    MotorError GetMotorError(int motor_idx);
```

- [ ] **Step 3: Parse and store error_code/pattern in HandleCANMessage**

In `driver/src/robstride.cpp`, in `HandleCANMessage`, uncomment and fix the error parsing. After `motor.online = true;` and before `// Parse Data`, add:

```cpp
                motor.error_code = reserved & 0x3F;
                motor.pattern = (reserved >> 6) & 0x03;
```

- [ ] **Step 4: Implement GetMotorError in robstride.cpp**

Add at the end of `driver/src/robstride.cpp`:

```cpp
RobstrideController::MotorError RobstrideController::GetMotorError(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        return {motor_data[motor_idx].error_code, motor_data[motor_idx].pattern};
    }
    return {0, 0};
}
```

- [ ] **Step 5: Verify driver still compiles**

```bash
mkdir -p driver/build && cd driver/build && cmake .. && make -j$(nproc)
```

Expected: compiles without errors.

- [ ] **Step 6: Commit**

```bash
git add driver/include/robstride.hpp driver/src/robstride.cpp
git commit -m "feat(driver): expose motor error_code and pattern from CAN responses"
```

---

## Task 2: Create build system for motor_tool

**Files:**
- Create: `tools/motor_tool/CMakeLists.txt`

- [ ] **Step 1: Create directory**

```bash
mkdir -p tools/motor_tool
```

- [ ] **Step 2: Write CMakeLists.txt**

Create `tools/motor_tool/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(motor_tool CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../driver)

add_executable(motor_tool
    main.cpp
    ${DRIVER_DIR}/src/robstride.cpp
    ${DRIVER_DIR}/src/can_interface.cpp
    ${DRIVER_DIR}/src/observations.cpp
    ${DRIVER_DIR}/src/wit_c_sdk.c
    ${DRIVER_DIR}/src/serial.c
)

target_include_directories(motor_tool PRIVATE ${DRIVER_DIR}/include)
target_link_libraries(motor_tool Threads::Threads)
```

---

## Task 3: Create main.cpp with all subcommands

**Files:**
- Create: `tools/motor_tool/main.cpp`

- [ ] **Step 1: Create main.cpp**

Create `tools/motor_tool/main.cpp` with the complete implementation:

```cpp
#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>
#include <memory>
#include <array>

#include "robstride.hpp"
#include "can_interface.hpp"
#include "observations.hpp"

static constexpr int NUM_JOINTS = 12;
static constexpr int NUM_LEGS = 4;
static constexpr uint8_t HOST_ID = 0xFD;
static constexpr float MAX_TORQUE = 17.0f;
static constexpr float MAX_SPEED = 44.0f;
static constexpr float DEFAULT_KP = 40.0f;
static constexpr float DEFAULT_KD = 0.5f;

static constexpr int kMotorIds[NUM_JOINTS] = {
    1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15
};

static constexpr float kOffsets[NUM_JOINTS] = {
    0.37f, -0.37f, -0.37f, 0.37f,
    0.13f,  0.13f, -0.13f, -0.13f,
    1.76702f, 1.76702f, -1.76702f, -1.76702f
};

static const char* kJointNames[NUM_JOINTS] = {
    "LF_HipA", "LR_HipA", "RF_HipA", "RR_HipA",
    "LF_HipF", "LR_HipF", "RF_HipF", "RR_HipF",
    "LF_Knee", "LR_Knee", "RF_Knee", "RR_Knee"
};

static const char* kCanNames[NUM_LEGS] = {"can0", "can1", "can2", "can3"};

static int can_index(int joint) { return joint % NUM_LEGS; }

static volatile sig_atomic_t g_running = 1;
static void signal_handler(int) { g_running = 0; }

struct MotorCtx {
    std::array<std::shared_ptr<CANInterface>, NUM_LEGS> can_ifaces{};
    std::shared_ptr<RobstrideController> ctrl;
    int motor_idx[NUM_JOINTS]{};
    bool bound[NUM_JOINTS]{};
    bool can_opened[NUM_LEGS]{};

    void init(const std::vector<int>& joints) {
        ctrl = std::make_shared<RobstrideController>();
        for (int j : joints) {
            int ci = can_index(j);
            if (!can_opened[ci]) {
                can_ifaces[ci] = std::make_shared<CANInterface>(kCanNames[ci]);
                ctrl->BindCAN(can_ifaces[ci]);
                can_opened[ci] = true;
            }
            auto info = std::make_unique<RobstrideController::MotorInfo>();
            info->motor_id = kMotorIds[j];
            info->host_id = HOST_ID;
            info->max_torque = MAX_TORQUE;
            info->max_speed = MAX_SPEED;
            info->max_kp = 500.0f;
            info->max_kd = 5.0f;
            motor_idx[j] = ctrl->BindMotor(kCanNames[ci], std::move(info));
            bound[j] = true;
        }
    }

    std::vector<int> bound_joints() const {
        std::vector<int> js;
        for (int i = 0; i < NUM_JOINTS; ++i)
            if (bound[i]) js.push_back(i);
        return js;
    }
};

static int cmd_autoreport(const std::vector<int>& joints) {
    MotorCtx ctx;
    ctx.init(joints);
    auto all = ctx.bound_joints();

    for (int j : all)
        ctx.ctrl->EnableAutoReport(ctx.motor_idx[j]);

    printf("Auto-report enabled. Press Ctrl-C to stop.\n");
    while (g_running) {
        printf("\033[2J\033[H");
        printf("%-10s %10s %10s %10s %7s\n", "Joint", "Pos(rad)", "Vel(rad/s)", "Torque(Nm)", "Online");
        printf("-----------------------------------------------------------\n");
        for (int j : all) {
            auto st = ctx.ctrl->GetMotorState(ctx.motor_idx[j]);
            bool on = ctx.ctrl->IsMotorOnline(ctx.motor_idx[j]);
            printf("%-10s %10.4f %10.4f %10.4f %7s\n",
                   kJointNames[j], st.position, st.velocity, st.torque,
                   on ? "YES" : "NO");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    for (int j : all)
        ctx.ctrl->DisableAutoReport(ctx.motor_idx[j]);
    return 0;
}

static int cmd_setzero(const std::vector<int>& joints) {
    MotorCtx ctx;
    ctx.init(joints);

    for (int j : ctx.bound_joints()) {
        ctx.ctrl->SetZero(ctx.motor_idx[j]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        printf("SetZero sent to %-10s (motor ID %d)\n", kJointNames[j], kMotorIds[j]);
    }
    printf("Done.\n");
    return 0;
}

static int cmd_to_offset(const std::vector<int>& joints) {
    MotorCtx ctx;
    ctx.init(joints);
    auto all = ctx.bound_joints();

    for (int j : all) {
        MIT_params params;
        params.kp = DEFAULT_KP;
        params.kd = DEFAULT_KD;
        params.vel_limit = MAX_SPEED;
        params.torque_limit = MAX_TORQUE;
        ctx.ctrl->SetMITParams(ctx.motor_idx[j], params);
        ctx.ctrl->EnableMotor(ctx.motor_idx[j]);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (int j : all)
        ctx.ctrl->EnableAutoReport(ctx.motor_idx[j]);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    float start_pos[NUM_JOINTS]{};
    float end_pos[NUM_JOINTS]{};
    printf("%-10s %10s -> %10s\n", "Joint", "Current", "Target");
    printf("----------------------------------------\n");
    for (int j : all) {
        auto st = ctx.ctrl->GetMotorState(ctx.motor_idx[j]);
        start_pos[j] = st.position;
        end_pos[j] = kOffsets[j];
        printf("%-10s %10.4f -> %10.4f\n", kJointNames[j], start_pos[j], end_pos[j]);
    }

    printf("\nInterpolating over 2s... Press Ctrl-C to abort.\n");
    constexpr int STEPS = 100;
    constexpr auto STEP_DT = std::chrono::milliseconds(20);

    for (int step = 0; step <= STEPS && g_running; ++step) {
        float alpha = (float)step / STEPS;
        for (int j : all) {
            float pos = start_pos[j] + (end_pos[j] - start_pos[j]) * alpha;
            ctx.ctrl->SendMITCommand(ctx.motor_idx[j], pos);
        }
        std::this_thread::sleep_for(STEP_DT);
    }

    printf(g_running ? "Done. Disabling motors.\n" : "Aborted. Disabling motors.\n");
    for (int j : all) {
        ctx.ctrl->DisableMotor(ctx.motor_idx[j]);
        ctx.ctrl->DisableAutoReport(ctx.motor_idx[j]);
    }
    return 0;
}

static int cmd_imu() {
    auto imu = std::make_unique<IMUComponent>("/dev/ttyCH341USB0");

    printf("Reading IMU. Press Ctrl-C to stop.\n");
    while (g_running) {
        auto obs = imu->GetObs();
        printf("\033[2J\033[H");
        printf("IMU Data\n");
        printf("Gyro:  X=%8.4f  Y=%8.4f  Z=%8.4f  (rad/s)\n", obs[0], obs[1], obs[2]);
        printf("Grav:  X=%8.4f  Y=%8.4f  Z=%8.4f  (unit)\n", obs[3], obs[4], obs[5]);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}

static int cmd_errors(const std::vector<int>& joints, bool clear) {
    MotorCtx ctx;
    ctx.init(joints);
    auto all = ctx.bound_joints();

    for (int j : all)
        ctx.ctrl->EnableAutoReport(ctx.motor_idx[j]);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    printf("%-10s %8s %8s %8s\n", "Joint", "ErrCode", "Pattern", "Status");
    printf("------------------------------------------\n");
    bool any_fault = false;
    for (int j : all) {
        auto err = ctx.ctrl->GetMotorError(ctx.motor_idx[j]);
        bool fault = (err.pattern == 2);
        if (fault) any_fault = true;
        printf("%-10s  0x%02X      %2d    %s\n",
               kJointNames[j], err.error_code, err.pattern,
               fault ? "** FAULT **" : "OK");
    }

    if (clear) {
        printf("\nClearing errors...\n");
        for (int j : all) {
            ctx.ctrl->ClearMotor(ctx.motor_idx[j]);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        printf("Re-enabling motors...\n");
        for (int j : all) {
            ctx.ctrl->EnableMotor(ctx.motor_idx[j]);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("Done.\n");
    } else if (any_fault) {
        printf("\nFaults detected. Use -c flag to clear errors.\n");
    }

    for (int j : all)
        ctx.ctrl->DisableAutoReport(ctx.motor_idx[j]);
    return 0;
}

static void usage(const char* prog) {
    printf("Usage: %s <command> [options]\n\n", prog);
    printf("Commands:\n");
    printf("  autoreport [-j N]    Enable auto-report, print motor pos in real-time\n");
    printf("  setzero [-j N]       Set current position as zero point\n");
    printf("  to_offset [-j N]     Enable motors + interpolate to offset positions\n");
    printf("  imu                  Read IMU gyro + projected gravity\n");
    printf("  errors [-j N] [-c]   Read motor error codes; -c to clear\n");
    printf("\nOptions:\n");
    printf("  -j N   Joint index (0-11), can repeat. Omit for all joints.\n");
    printf("  -c     Clear errors (only for 'errors' command)\n");
    printf("\nJoint layout:\n");
    printf("  0:LF_HipA  1:LR_HipA  2:RF_HipA  3:RR_HipA\n");
    printf("  4:LF_HipF  5:LR_HipF  6:RF_HipF  7:RR_HipF\n");
    printf("  8:LF_Knee  9:LR_Knee 10:RF_Knee 11:RR_Knee\n");
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);

    std::string cmd = argv[1];
    std::vector<int> joints;
    bool clear = false;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-j" && i + 1 < argc) {
            int j = std::stoi(argv[++i]);
            if (j < 0 || j >= NUM_JOINTS) {
                printf("Joint index must be 0-11, got %d\n", j);
                return 1;
            }
            joints.push_back(j);
        } else if (arg == "-c" || arg == "--clear") {
            clear = true;
        }
    }

    if (joints.empty())
        for (int i = 0; i < NUM_JOINTS; ++i) joints.push_back(i);

    if (cmd == "autoreport") return cmd_autoreport(joints);
    if (cmd == "setzero")    return cmd_setzero(joints);
    if (cmd == "to_offset")  return cmd_to_offset(joints);
    if (cmd == "imu")        return cmd_imu();
    if (cmd == "errors")     return cmd_errors(joints, clear);

    printf("Unknown command: %s\n", cmd.c_str());
    usage(argv[0]);
    return 1;
}
```

---

## Task 4: Build and verify

- [ ] **Step 1: Build motor_tool**

```bash
mkdir -p tools/motor_tool/build && cd tools/motor_tool/build && cmake .. && make -j$(nproc)
```

Expected: produces `tools/motor_tool/build/motor_tool` executable.

- [ ] **Step 2: Verify help output**

```bash
./tools/motor_tool/build/motor_tool
```

Expected: prints usage text with all 5 commands.

- [ ] **Step 3: Commit all new files**

```bash
git add tools/motor_tool/ docs/
git commit -m "feat: add motor_tool CLI for motor and IMU debugging"
```

---

## Self-Review Checklist

- [x] Spec coverage: autoreport, setzero, to_offset, imu, errors — all covered
- [x] No placeholders: all code is complete
- [x] Type consistency: MotorCtx.motor_idx matches RobstrideController API
- [x] Error protocol matches UIKA reference: error_code = reserved & 0x3F, pattern = (reserved >> 6) & 0x03

#include <iostream>
#include <string>
#include <vector>
#include <csignal>
#include <chrono>
#include <thread>
#include <cstdio>
#include <array>

#include "dog_driver.hpp"
#include "quadruped_leg_controller.hpp"

static volatile sig_atomic_t g_running = 1;
static void signal_handler(int) { g_running = 0; }

static int cmd_move_foot(DogDriver& driver, double fx, double fy, double fz) {
    QuadrupedLegController qlc;
    Vec3 foot_hip = {fx, fy, fz};

    printf("Foot target in hip frame: (%.4f, %.4f, %.4f)\n", fx, fy, fz);

    const char* leg_names[] = {"LF", "RF", "LH", "RH"};
    int leg_joints[4][3] = {
        {0, 4, 8},   // LF: HipA, HipF, Knee
        {2, 6, 10},  // RF: HipA, HipF, Knee
        {1, 5, 9},   // LH: HipA, HipF, Knee
        {3, 7, 11},  // RH: HipA, HipF, Knee
    };

    printf("\nIK -> motor angles (rad):\n");
    printf("%-6s %10s %10s %10s\n", "Leg", "abad", "hip", "knee");
    printf("--------------------------------------\n");

    std::array<float, DogDriver::NUM_JOINTS> targets{};
    for (int leg = 0; leg < 4; ++leg) {
        Eigen::Vector3d q_ik = qlc.solve_ik(foot_hip);
        Eigen::Vector3d q_m = qlc.to_motor_angles(leg, q_ik);
        printf("%-6s %10.4f %10.4f %10.4f\n", leg_names[leg], q_m[0], q_m[1], q_m[2]);
        for (int ji = 0; ji < 3; ++ji)
            targets[leg_joints[leg][ji]] = (float)q_m(ji);
    }

    printf("\nEnabling motors...\n");
    driver.EnableAll();
    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
        driver.SetMITParams(i, 20.0f, 0.5f);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int online = driver.OnlineMotorCount();
    printf("Motors online: %d/%d\n", online, DogDriver::NUM_JOINTS);
    if (online == 0) {
        printf("ERROR: No motors online. Aborting.\n");
        return 1;
    }

    auto cur = driver.GetJointStates();
    printf("\n%-10s %10s %10s %10s\n", "Joint", "Current", "Target", "Online");
    printf("------------------------------------------------\n");
    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i) {
        bool on = driver.IsJointOnline(i);
        printf("%-10d %10.4f %10.4f %7s\n", i, cur.position[i], targets[i],
               on ? "YES" : "NO");
    }

    printf("\nInterpolating over 2s... Press Ctrl-C to abort.\n");
    constexpr int STEPS = 100;
    constexpr auto STEP_DT = std::chrono::milliseconds(20);

    for (int step = 0; step <= STEPS && g_running; ++step) {
        float alpha = (float)step / STEPS;
        std::array<float, DogDriver::NUM_JOINTS> cmd{};
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            cmd[i] = cur.position[i] + (targets[i] - cur.position[i]) * alpha;
        driver.SetAllJointPositions(cmd);
        std::this_thread::sleep_for(STEP_DT);
    }

    printf(g_running ? "\nDone. Ctrl-C to damping.\n"
                      : "\nAborted.\n");

    while (g_running)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    printf("Damping...\n");
    driver.DisableAll();
    printf("Done.\n");
    return 0;
}

static int cmd_ik_only(double fx, double fy, double fz) {
    QuadrupedLegController qlc;
    Vec3 foot_hip = {fx, fy, fz};
    const char* leg_names[] = {"LF", "RF", "LH", "RH"};

    printf("Foot: (%.4f, %.4f, %.4f)\n\n", fx, fy, fz);
    printf("%-6s %10s %10s %10s   (after to_motor_angles)\n", "Leg", "abad", "hip", "knee");
    printf("------------------------------------------------\n");
    for (int leg = 0; leg < 4; ++leg) {
        Eigen::Vector3d q_ik = qlc.solve_ik(foot_hip);
        Eigen::Vector3d q_m = qlc.to_motor_angles(leg, q_ik);
        printf("%-6s %10.4f %10.4f %10.4f\n", leg_names[leg], q_m[0], q_m[1], q_m[2]);
    }
    return 0;
}

static int cmd_gait(DogDriver& driver, double vx, double vy, double wz) {
    QuadrupedLegController qlc;
    qlc.set_dt(0.02);
    GaitCommand gait_cmd{vx, vy, wz};

    printf("Gait: vx=%.2f vy=%.2f wz=%.2f  dt=%.3f  |  Ctrl-C = damping\n", vx, vy, wz, qlc.dt());

    int leg_joints[4][3] = {
        {0, 4, 8},   // LF
        {2, 6, 10},  // RF
        {1, 5, 9},   // LH
        {3, 7, 11},  // RH
    };

    printf("Enabling motors...\n");
    driver.EnableAll();
    for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
        driver.SetMITParams(i, 20.0f, 0.5f);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int online = driver.OnlineMotorCount();
    printf("Motors online: %d/%d\n\n", online, DogDriver::NUM_JOINTS);
    if (online == 0) {
        printf("ERROR: No motors online. Aborting.\n");
        return 1;
    }

    auto t_start = std::chrono::steady_clock::now();
    constexpr auto DT = std::chrono::milliseconds(20);

    printf("%-10s %10s %10s\n", "Joint", "Pos", "Target");
    printf("--------------------------------------\n");

    while (g_running) {
        auto states = qlc.update(gait_cmd);

        std::array<float, DogDriver::NUM_JOINTS> cmd{};
        for (int leg = 0; leg < 4; ++leg)
            for (int ji = 0; ji < 3; ++ji)
                cmd[leg_joints[leg][ji]] = (float)states[leg].q(ji);

        driver.SetAllJointPositions(cmd);

        printf("\033[2J\033[H");
        printf("Gait: vx=%.2f vy=%.2f wz=%.2f  t=%.2fs  |  Ctrl-C = damping\n", vx, vy, wz, qlc.time());
        printf("%-10s %10s %10s\n", "Joint", "Pos", "Target");
        printf("--------------------------------------\n");
        auto st = driver.GetJointStates();
        for (int i = 0; i < DogDriver::NUM_JOINTS; ++i)
            printf("%-10d %10.4f %10.4f\n", i, st.position[i], cmd[i]);

        std::this_thread::sleep_for(DT);
    }

    printf("Damping...\n");
    driver.DisableAll();
    printf("Done.\n");
    return 0;
}

static void usage(const char* prog) {
    printf("Usage: %s <command> [args]\n\n", prog);
    printf("Commands:\n");
    printf("  move_foot X Y Z       Move all legs to foot position (hip frame)\n");
    printf("  ik X Y Z              Show IK only (no hardware)\n");
    printf("  gait VX VY WZ         Run gait trajectory (m/s, rad/s)\n");
    printf("\nExamples:\n");
    printf("  %s ik 0 0 -0.3\n", prog);
    printf("  %s move_foot 0 0 -0.3\n", prog);
    printf("  %s gait 0.5 0 0\n", prog);
}

int main(int argc, char* argv[]) {
    if (argc < 2) { usage(argv[0]); return 1; }

    struct sigaction sa{};
    sa.sa_handler = signal_handler;
    sigaction(SIGINT, &sa, nullptr);

    std::string cmd = argv[1];

    if (cmd == "ik") {
        if (argc < 5) { usage(argv[0]); return 1; }
        return cmd_ik_only(std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
    }

    if (cmd == "move_foot") {
        if (argc < 5) { usage(argv[0]); return 1; }
        printf("Initializing DogDriver...\n");
        DogDriver driver;
        printf("Done. Online: %d/%d\n\n",
               driver.OnlineMotorCount(), DogDriver::NUM_JOINTS);
        return cmd_move_foot(driver, std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
    }

    if (cmd == "gait") {
        if (argc < 5) { usage(argv[0]); return 1; }
        printf("Initializing DogDriver...\n");
        DogDriver driver;
        printf("Done. Online: %d/%d\n\n",
               driver.OnlineMotorCount(), DogDriver::NUM_JOINTS);
        return cmd_gait(driver, std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
    }

    printf("Unknown command: %s\n", cmd.c_str());
    usage(argv[0]);
    return 1;
}

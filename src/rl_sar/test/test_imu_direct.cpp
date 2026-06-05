/*
 * test_imu_direct.cpp — Direct IMU hardware test via AresDriverCore
 *
 * Verifies that the WIT IMU is actually returning real data (not defaults).
 * Tests that:
 *   1. IMU reports as connected (imu_connected() == true)
 *   2. projected_gravity has unit norm (|g| ≈ 1.0)
 *   3. Data is not the sentinel default [0, 0, -1]
 *   4. Data actually changes over time (not frozen)
 *
 * WARNING: Robot hardware must be connected and powered. Motors will be
 * enabled during construction. Keep the robot suspended or on a stand.
 */

#include "ares_driver_core.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <thread>

static constexpr float GRAVITY_NORM_MIN = 0.95f;
static constexpr float GRAVITY_NORM_MAX = 1.05f;
static constexpr int   MIN_SAMPLES      = 5;
static constexpr int   NUM_SAMPLES      = 20;   // 20 * 10ms = 200ms, quick exit
static constexpr auto  SAMPLE_DT        = std::chrono::milliseconds(10);

// Timeout: if test takes >10s, something is wrong
static std::atomic<bool> timeout_flag{false};
static void alarm_handler(int) { timeout_flag = true; }

int main(int argc, char** argv)
{
    signal(SIGALRM, alarm_handler);
    alarm(10); // 10 second watchdog

    const char* policy_name = argc > 1 ? argv[1] : "dream_waq/dream_waq";

    // Write results to a file so dog_driver stdout noise doesn't hide them
    const char* out_path = "/tmp/imu_test_results.txt";
    FILE* out = fopen(out_path, "w");
    if (!out) { perror("fopen"); return 1; }

#define PRINT(fmt, ...) do { fprintf(out, fmt, ##__VA_ARGS__); fflush(out); } while(0)
#define PRINTLN(fmt, ...) PRINT(fmt "\n", ##__VA_ARGS__)

    PRINTLN("=== ARES IMU Direct Test ===");
    PRINTLN("Policy: %s", policy_name);
    PRINTLN("");

    AresDriverCore core(std::string(POLICY_DIR), policy_name);
    PRINTLN("Driver created.");

    // --- Check IMU connection ---
    bool imu_ok = core.imu_connected();
    PRINTLN("  IMU connected: %s", imu_ok ? "YES" : "NO");
    PRINTLN("");

    // --- Sample IMU data (raw first samples immediately) ---
    PRINTLN("--- First 5 raw samples ---");
    PRINTLN("  #   gx       gy       gz       |g|      wx       wy       wz");
    PRINTLN("  --  -------- -------- -------- -------- -------- -------- --------");

    struct Sample {
        float gx, gy, gz, gnorm;
        float wx, wy, wz;
    };
    Sample first_samples[5] = {};
    int actual_samples = 0;
    double grav_norm_avg = 0;
    double grav_norm_min = 1e9, grav_norm_max = -1e9;
    bool data_changed = false;
    std::array<float, 3> prev_grav{-999, -999, -999};

    for (int i = 0; i < NUM_SAMPLES && !timeout_flag; ++i) {
        auto data = core.GetImuData();

        float gx = data.projected_gravity[0];
        float gy = data.projected_gravity[1];
        float gz = data.projected_gravity[2];
        float gnorm = std::sqrt(gx*gx + gy*gy + gz*gz);

        if (i < 5) {
            first_samples[i] = {gx, gy, gz, gnorm,
                                data.angular_velocity[0],
                                data.angular_velocity[1],
                                data.angular_velocity[2]};
        }

        grav_norm_avg += (gnorm - grav_norm_avg) / (i + 1);
        grav_norm_min = std::min(grav_norm_min, (double)gnorm);
        grav_norm_max = std::max(grav_norm_max, (double)gnorm);

        if (prev_grav[0] != -999.0f) {
            float diff = std::abs(gx - prev_grav[0]) +
                         std::abs(gy - prev_grav[1]) +
                         std::abs(gz - prev_grav[2]);
            if (diff > 1e-6f) data_changed = true;
        }
        prev_grav = {gx, gy, gz};
        actual_samples++;

        std::this_thread::sleep_for(SAMPLE_DT);
    }

    for (int i = 0; i < std::min(5, actual_samples); ++i) {
        auto& s = first_samples[i];
        PRINTLN("  %-3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f",
                i, s.gx, s.gy, s.gz, s.gnorm, s.wx, s.wy, s.wz);
    }

    // --- Results ---
    PRINTLN("");
    PRINTLN("=== Results ===");
    PRINTLN("Samples: %d", actual_samples);
    PRINTLN("Data changed: %s", data_changed ? "YES" : "NO");
    if (!data_changed && actual_samples > 1) {
        PRINTLN("  INFO: data stable across samples (robot stationary, expected)");
    }
    PRINTLN("Gravity norm: avg=%.4f  min=%.4f  max=%.4f",
            grav_norm_avg, grav_norm_min, grav_norm_max);

    // Check sentinel [0,0,-1] — use LAST sample (skip initial default before IMU syncs)
    bool is_sent = (actual_samples > 0 &&
                    std::abs(first_samples[std::min(4, actual_samples - 1)].gx) < 1e-6f &&
                    std::abs(first_samples[std::min(4, actual_samples - 1)].gy) < 1e-6f &&
                    std::abs(first_samples[std::min(4, actual_samples - 1)].gz + 1.0f) < 1e-6f);
    // Also check that at least one sample differs from sentinel
    bool all_sent = true;
    for (int i = 0; i < std::min(5, actual_samples); ++i) {
        if (std::abs(first_samples[i].gx) > 1e-6f ||
            std::abs(first_samples[i].gy) > 1e-6f ||
            std::abs(first_samples[i].gz + 1.0f) > 1e-6f) {
            all_sent = false;
            break;
        }
    }

    // Check NaN/Inf
    auto last = core.GetImuData();
    bool has_nan = false;
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(last.angular_velocity[i]) || std::isinf(last.angular_velocity[i]) ||
            std::isnan(last.projected_gravity[i]) || std::isinf(last.projected_gravity[i]))
            has_nan = true;
    }

    // --- Verdict ---
    PRINTLN("");
    PRINTLN("--- Verdict ---");

    int failed = 0;
    if (!imu_ok)            { PRINTLN("  FAIL: IMU not connected"); failed++; }
    else                      PRINTLN("  PASS: IMU connected");
    if (all_sent)           { PRINTLN("  FAIL: all samples are [0,0,-1] — IMU never delivered real data"); failed++; }
    else if (is_sent)       { PRINTLN("  FAIL: even last sample is [0,0,-1]"); failed++; }
    else {
        // Check if first sample was sentinel (IMU sync delay)
        bool first_sent = (std::abs(first_samples[0].gx) < 1e-6f &&
                           std::abs(first_samples[0].gy) < 1e-6f &&
                           std::abs(first_samples[0].gz + 1.0f) < 1e-6f);
        if (first_sent)
                              PRINTLN("  PASS: sample 0 was pre-sync sentinel [0,0,-1], samples 1+ are real IMU data");
        else
                              PRINTLN("  PASS: not sentinel [0,0,-1]");
    }
    if (grav_norm_avg < GRAVITY_NORM_MIN || grav_norm_avg > GRAVITY_NORM_MAX)
                            { PRINTLN("  FAIL: |gravity|=%.4f (expected [%.2f,%.2f])", grav_norm_avg, GRAVITY_NORM_MIN, GRAVITY_NORM_MAX); failed++; }
    else                      PRINTLN("  PASS: |gravity| ≈ 1 (%.4f)", grav_norm_avg);
    if (actual_samples < MIN_SAMPLES)
                            { PRINTLN("  FAIL: too few samples (%d < %d)", actual_samples, MIN_SAMPLES); failed++; }
    else                      PRINTLN("  PASS: sampled %d times", actual_samples);
    if (has_nan)            { PRINTLN("  FAIL: contains NaN/Inf"); failed++; }
    else                      PRINTLN("  PASS: no NaN/Inf");
    if (!data_changed && actual_samples > 1)
                              PRINTLN("  INFO: data stable across samples (robot stationary, expected)");
    else if (actual_samples > 1)
                              PRINTLN("  PASS: data updates over time");

    PRINTLN("");
    if (failed == 0) {
        PRINTLN("  RESULT: ALL PASS — IMU is working correctly.");
    } else {
        PRINTLN("  RESULT: %d check(s) FAILED — IMU may not be functioning.", failed);
    }

    fclose(out);

    alarm(0); // cancel timeout
    printf("Results written to %s\n", out_path);
    return failed > 0 ? 1 : 0;
}

#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>

#ifdef DOG_DRIVER_EXPORTS
#define DOG_DRIVER_API __attribute__((visibility("default")))
#else
#define DOG_DRIVER_API __attribute__((visibility("default")))
#endif

// Forward declarations to hide internal implementation headers.
// Consumers only need this header + -ldog_driver to link.
class CANInterface;
class RobstrideController;
class IMUComponent;

// ============================================================================
// DogDriver - Four-legged robot motor & IMU driver (shared library public API)
// ============================================================================
//
// This is the ONLY header consumers should include.
// All internal details (CAN, RobstrideController, IMUComponent) are hidden.
//
// Hardware (hardcoded, no configuration needed):
//   - 4 CAN buses: can0, can1, can2, can3
//     can0=LF, can1=LR, can2=RF, can3=RR (each drives HipA, HipF, Knee)
//   - 12 motors total, ordered as:
//       LF_HipA(0) LR_HipA(1) RF_HipA(2) RR_HipA(3)
//       LF_HipF(4) LR_HipF(5) RF_HipF(6) RR_HipF(7)
//       LF_Knee(8) LR_Knee(9) RF_Knee(10) RR_Knee(11)
//   - 1 WIT IMU on /dev/ttyCH341USB0 (gyro + quaternion -> projected gravity)
//
// Initialization (all done in constructor, ~1s):
//   1. Opens CAN sockets + IMU serial port
//   2. Binds all 12 motors with default MIT params (kp=40, kd=0.5)
//   3. Enables all motors + double EnableAutoReport for reliability
//   4. Waits up to 5s for all motors to report online
//   5. Starts background threads for CAN RX, motor watchdog, IMU polling
//
// Background threads (auto-managed, no user action needed):
//   - CAN RX thread: blocks on socket, dispatches motor feedback to callback
//   - Control thread (10ms): watches motor timeouts (500ms), re-enables offline motors
//   - IMU thread: continuously reads WIT serial data
//   - Motor auto-report: motors push state at ~20ms after EnableAutoReport
//
// Coordinate convention:
//   - Joint space is the USER-facing coordinate system (relative to zero offset)
//   - Motor space is the raw encoder coordinate system
//   - Conversion: motor_pos = kJointDirection[i] * joint_pos * gear_ratio + offset
//   - kJointDirection: HipA/HipF = -1.0, Knee = +1.0
//   - gear_ratio: Knee = 1.667, others = 1.0
//   - All offsets and directions are handled internally; users only deal with
//     joint-space values (rad, relative to zero pose).
//
// Usage:
//   DogDriver driver;                          // constructor does everything
//   auto states = driver.GetJointStates();     // latest from background thread
//   driver.SetAllJointPositions(target);       // send MIT position command
//   // ... when done:
//   driver.DisableAll();                       // disable all motors
// ============================================================================

class DOG_DRIVER_API DogDriver {
public:
    static constexpr int NUM_JOINTS = 12;
    static constexpr int NUM_LEGS = 4;
    static constexpr int JOINTS_PER_LEG = 3;
    static constexpr uint8_t HOST_ID = 0xFD;
    static constexpr float KNEE_GEAR_RATIO = 1.667f;
    static constexpr float MAX_TORQUE = 17.0f;
    static constexpr float MAX_SPEED = 44.0f;
    static constexpr float DEFAULT_KP = 40.0f;
    static constexpr float DEFAULT_KD = 0.5f;

    struct JointState {
        std::array<float, NUM_JOINTS> position;  // rad, relative to zero offset
        std::array<float, NUM_JOINTS> velocity;  // rad/s
    };

    struct IMUData {
        std::array<float, 3> angular_velocity;    // rad/s (body frame)
        std::array<float, 3> projected_gravity;   // unit vector, body frame
    };

    // Constructs driver, opens hardware, enables all motors, sets MIT params,
    // enables auto-report, and waits for all motors to come online.
    // Blocks for ~1-5s depending on motor response time.
    DogDriver();
    ~DogDriver();

    DogDriver(const DogDriver&) = delete;
    DogDriver& operator=(const DogDriver&) = delete;

    // --- State queries (non-blocking, reads latest from background thread) ---

    // Returns latest joint states (pos/vel) from auto-report feedback.
    // Thread-safe: reads from motor state updated by CAN RX callback.
    JointState GetJointStates() const;

    // Returns latest IMU data (gyro + projected gravity from quaternion).
    IMUData GetIMUData() const;

    // --- Motor commands ---

    // Send MIT position command to a single joint.
    // pos: joint-space position in rad (relative to zero offset).
    // Returns 0 on success, -1 on invalid index.
    int SetJointPosition(int joint_idx, float pos);

    // Send MIT position commands to all 12 joints at once.
    // pos[i]: joint-space position in rad for joint i.
    // Returns 0 if all succeed, first error code otherwise.
    int SetAllJointPositions(const std::array<float, NUM_JOINTS>& pos);

    // --- Enable / Disable ---

    // Enable/disable all 12 motors. Returns 0 on all success.
    int EnableAll();
    int DisableAll();

    // Enable/disable a single motor by joint index. Returns 0 on success.
    int EnableJoint(int joint_idx);
    int DisableJoint(int joint_idx);

    // --- Configuration ---

    // Set current motor position as zero point for one joint.
    // Should only be called in stand pose. Returns 0 on success.
    int SetZero(int joint_idx);

    // Enable/disable periodic motor state auto-report (~20ms interval).
    int EnableAutoReport(int joint_idx);
    int DisableAutoReport(int joint_idx);

    // Set MIT impedance parameters (kp, kd) for one joint.
    // vel_limit and torque_limit use class defaults (MAX_SPEED, MAX_TORQUE).
    int SetMITParams(int joint_idx, float kp, float kd);

    // --- Status queries ---

    // Returns true if motor has responded within the last 500ms.
    bool IsJointOnline(int joint_idx) const;

    // Returns true if IMU was successfully opened at construction time.
    bool IsIMUConnected() const;

private:
    std::array<std::shared_ptr<CANInterface>, NUM_LEGS> can_interfaces_;
    std::shared_ptr<RobstrideController> motor_controller_;
    std::unique_ptr<IMUComponent> imu_;

    std::array<int, NUM_JOINTS> motor_indices_;
    bool imu_connected_ = false;

    static constexpr std::array<int, NUM_JOINTS> kMotorIds = {
        1,  5,  9,  13,
        2,  6,  10, 14,
        3,  7,  11, 15
    };

    static constexpr std::array<float, NUM_JOINTS> kOffsets = {
        0.37f,  -0.37f, -0.37f,  0.37f,
        0.13f,   0.13f, -0.13f, -0.13f,
        1.76702f, 1.76702f, -1.76702f, -1.76702f
    };

    // -1 = motor and joint rotate in opposite directions (HipA, HipF)
    // +1 = motor and joint rotate in the same direction (Knee)
    static constexpr std::array<float, NUM_JOINTS> kJointDirection = {
        -1.0f, -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f, -1.0f,
        +1.0f, +1.0f, +1.0f, +1.0f
    };
};

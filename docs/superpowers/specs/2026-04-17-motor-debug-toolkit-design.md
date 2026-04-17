# Motor Debug CLI Toolkit Design

## Goal

A lightweight command-line tool for debugging RobStride motors and WitMotion IMU, using the low-level driver components directly (not DogDriver).

## Architecture

### File Structure

```
tools/motor_tool/
  main.cpp          -- CLI entry point, subcommand dispatch
  debug_driver.hpp  -- Lightweight wrapper for on-demand init
  debug_driver.cpp
  CMakeLists.txt    -- Compiles driver sources, links pthread
```

### DebugDriver

Directly composes `CANInterface` + `RobstrideController` + `IMUComponent`:

- `InitMotors(vector<int> joints)` — opens only needed CAN buses, binds specified joints, no auto-enable
- `InitIMU()` — opens IMU serial port
- Exposes `RobstrideController*` and `IMUComponent*` for direct use

No wait loops, no auto-enable, no MIT params setting.

### Driver Modifications (minimal)

**robstride.hpp**: Add `error_code` and `pattern` fields to `MotorData`.

**robstride.cpp**:
- `HandleCANMessage`: uncomment line 72-73, store error_code and pattern
- Add `GetMotorError(int motor_idx)` method returning `{error_code, pattern}`

## Subcommands

### `autoreport [-j N]`

Enable auto-report on specified joints (default: all 12), print motor position in real-time loop. Ctrl-C to exit (disables autoreport on exit).

### `setzero [-j N]`

Send SetZero command to specified joints.

### `to_offset [-j N]`

1. Enable motors + set MIT params (kp=40, kd=0.5)
2. Enable autoreport
3. Read current motor positions
4. Linear interpolate from current pos to kOffsets (motor space) over 2 seconds at 50Hz
5. Disable motors on completion or Ctrl-C

### `imu`

Open IMU, print gyro[xyz] and projected_gravity[xyz] in real-time loop. Ctrl-C to exit.

### `errors [-j N] [--clear]`

Read error_code and pattern from motors. Print status. With `--clear`, send clear error command (COMM_STOP with data[0]=0x01).

## Key Details

- `-j N`: joint index 0-11. Omit for all joints.
- Joint layout: LF_HipA(0) LR_HipA(1) RF_HipA(2) RR_HipA(3) LF_HipF(4) LR_HipF(5) RF_HipF(6) RR_HipF(7) LF_Knee(8) LR_Knee(9) RF_Knee(10) RR_Knee(11)
- Motor IDs: 1,5,9,13, 2,6,10,14, 3,7,11,15
- CAN buses: can0=LF, can1=LR, can2=RF, can3=RR
- HOST_ID: 0xFD
- kOffsets: {0.37, -0.37, -0.37, 0.37, 0.13, 0.13, -0.13, -0.13, 1.76702, 1.76702, -1.76702, -1.76702}
- Error protocol: error_code = (can_id >> 16) & 0x3F, pattern = (can_id >> 22) & 0x03. pattern==2 means fault.
- All subcommands handle Ctrl-C gracefully.

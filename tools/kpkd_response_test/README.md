# KP/KD Response Test

This tool reads `fixed_kp`, `fixed_kd`, and `torque_limits` from
`policy/dream_waq/dream_waq/config.yaml`, enters a STAND initialization pose,
waits for the gamepad `B` button, then sends a fixed `+0.3 rad` relative step
command to all 12 joints and records 5 seconds of joint feedback.

It is an independent tool and is not part of `./build.sh`.

## What It Does

1. Initializes `DogDriver`.
2. Loads `fixed_kp`, `fixed_kd`, and `torque_limits` from:

   ```text
   policy/dream_waq/dream_waq/config.yaml
   ```

3. Enables all motors and enters a STAND initialization pose.
4. Waits for gamepad `B`.
5. Sends:

   ```text
   target[i] = initial_position[i] + 0.3 rad
   ```

6. Records 5 seconds of feedback at 100 Hz.
7. Saves a CSV and plots the 12 joint responses.

## Build

```bash
cmake -S tools/kpkd_response_test -B tools/kpkd_response_test/build
cmake --build tools/kpkd_response_test/build -j$(nproc)
```

## Run

Make sure no other driver node is controlling the motors.

```bash
./tools/kpkd_response_test/build/kpkd_response_test
```

After startup, wait for the robot to enter STAND, then press `B` on the gamepad
to start the command and data capture.

The CSV is written to `tools/kpkd_response_test/logs/dream_waq_step_response.csv`.

Press `Ctrl-C` to abort before or during the test.

The CSV contains:

- `target_j0..target_j11`
- `pos_j0..pos_j11`
- `vel_j0..vel_j11`
- `torque_j0..torque_j11`

## Plot

```bash
python3 tools/kpkd_response_test/plot_response.py
```

This writes `dream_waq_step_response.png` next to the CSV.

## Output Files

```text
tools/kpkd_response_test/logs/dream_waq_step_response.csv
tools/kpkd_response_test/logs/dream_waq_step_response.png
```

## Notes

- The step command is relative to the measured initial STAND position, not an
  absolute `0.3 rad` target.
- The test uses the same DogDriver joint order:

  ```text
  HipA: LF LR RF RR
  HipF: LF LR RF RR
  Knee: LF LR RF RR
  ```

- Use a test stand or support fixture first. A simultaneous `+0.3 rad` step on
  all joints can move the robot abruptly.

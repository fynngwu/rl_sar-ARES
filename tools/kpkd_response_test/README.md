# KP/KD Response Test

This tool reads `fixed_kp`, `fixed_kd`, and `torque_limits` from
`policy/dream_waq/dream_waq/config.yaml`, enters a STAND initialization pose,
waits for the gamepad `B` button, then sends a fixed `+0.3 rad` relative step
command to all 12 joints and records 5 seconds of joint feedback.

## Build

```bash
cmake -S tools/kpkd_response_test -B tools/kpkd_response_test/build
cmake --build tools/kpkd_response_test/build -j$(nproc)
```

## Run

```bash
./tools/kpkd_response_test/build/kpkd_response_test
```

After startup, wait for the robot to enter STAND, then press `B` on the gamepad
to start the command and data capture.

The CSV is written to `tools/kpkd_response_test/logs/dream_waq_step_response.csv`.

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

# Step Response Test

Simple step response test for all 12 joints with hardcoded kp/kd.

## Parameters

Edit `main.cpp` to change:

- `kStepRad` — step target (rad), default 0.3
- `kTestKp` — kp, default 40.0
- `kTestKd` — kd, default 2.0
- `kRecordSec` — recording duration (s), default 5.0

## Build

```bash
cmake -S tools/kpkd_response_test -B tools/kpkd_response_test/build
cmake --build tools/kpkd_response_test/build -j$(nproc)
```

## Run

```bash
./tools/kpkd_response_test/build/kpkd_response_test
```

Wait for STAND, then press `B` to start. Press `Ctrl-C` to abort.

## Plot

```bash
python3 tools/kpkd_response_test/plot_response.py
```

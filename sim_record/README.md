# sim_record

Standalone MuJoCo policy recorder derived from the provided `auto_test.py`. It runs an ONNX locomotion policy in simulation, executes a scheduled command sequence, records policy observations/actions/targets into CSV, and optionally produces a diagnostic plot.

Its main purpose is to create replayable traces for the real-robot daemon in `dog_cli_tool`.

## What it does

- loads a MuJoCo XML and ONNX policy
- runs the same policy observation stack structure as the reference tooling
- supports a YAML schedule or a one-line inline command
- records policy-rate samples to CSV with explicit columns for replay and analysis
- optionally writes an `.npz` archive and a plot PNG

## Quick Start

```bash
# Using uv (recommended)
uv run python sim_record.py \
  --onnx policy.onnx \
  --xml leggedrobot_flat.xml \
  --schedule examples/walk_forward.yaml \
  --output output/recording.csv \
  --plot-output output/recording.png \
  --headless

# Or using the project virtual environment directly
.venv/bin/python sim_record.py \
  --onnx policy.onnx \
  --xml leggedrobot_flat.xml \
  --schedule examples/walk_forward.yaml \
  --output output/recording.csv \
  --headless
```

## YAML schedule format

A schedule is a YAML file with a top-level `schedule:` list. Each phase contains:

- `cmd`: `[cmd_x, cmd_y, cmd_yaw]`
- `duration`: seconds
- `name`: optional label used in the plot and CSV

Example:

```yaml
schedule:
  - name: Stand
    cmd: [0.0, 0.0, 0.0]
    duration: 3.0
  - name: Forward
    cmd: [0.5, 0.0, 0.0]
    duration: 5.0
  - name: TurnL
    cmd: [0.5, 0.0, 0.3]
    duration: 3.0
```

Example files are included in `examples/`.

## Usage examples

YAML schedule:

```bash
uv run python sim_record.py \
  --onnx policy.onnx \
  --xml leggedrobot_flat.xml \
  --schedule examples/walk_turn.yaml \
  --output output/recording.csv \
  --plot-output output/recording.png
```

Inline one-phase command:

```bash
uv run python sim_record.py \
  --onnx policy.onnx \
  --xml leggedrobot_flat.xml \
  --cmd 0.5 0.0 0.0 \
  --duration 10.0 \
  --output output/recording.csv
```

Headless mode (no GUI):

```bash
uv run python sim_record.py --onnx policy.onnx --xml leggedrobot_flat.xml \
  --schedule examples/walk_forward.yaml --output output/recording.csv --headless
```

With interactive viewer (omit `--headless`):

```bash
uv run python sim_record.py \
  --onnx policy.onnx \
  --xml leggedrobot_flat.xml \
  --schedule examples/walk_forward.yaml \
  --output output/recording.csv
```

## Visualization

After recording, visualize the results using the dog_cli_tool visualizer:

```bash
cd ../dog_cli_tool/python

# Visualize with joint limit analysis
uv run python visualize.py \
  ../../sim_record/output/recording.csv \
  --highlight-bounds \
  --output ../../sim_record/output/visualization.png
```

This produces:
- A 3×4 joint tracking plot (position vs target for each joint)
- Joint limit boundaries marked as horizontal dotted lines
- Phase background coloring to show command segments
- Optional highlighting of near-limit and violation points

## CSV format

Each row is recorded at the policy update rate (default 50 Hz, with a 0.005 s sim step and decimation 4). The CSV columns are:

- `timestamp_ms`, `step`, `sim_time`, `phase`, `phase_name`
- `cmd_x`, `cmd_y`, `cmd_yaw`
- `obs_0..obs_44`
- `raw_action_0..raw_action_11`
- `scaled_action_0..scaled_action_11`
- `target_q_0..target_q_11`
- `joint_pos_0..joint_pos_11`
- `joint_vel_0..joint_vel_11`

Important conventions:

- `scaled_action_*` is in **policy order** and is the exact column group used by daemon replay.
- `target_q_*` is the post-remap target pose in policy order after adding the sim default pose.
- `joint_pos_*` and `joint_vel_*` are policy-order simulated joint states.

## How the CSV feeds into `dog_cli.py replay`

The daemon command

```bash
python dog_cli.py --host 192.168.1.100 replay /path/to/recording.csv 1.0
```

reads `scaled_action_0..11`, reconstructs each sample timestamp from `timestamp_ms` or `sim_time`, and sends those relative joint targets to the real robot through the same clamped joint-space path used by `set_joint`.

That means the replay interface is intentionally simple:

- `sim_record` owns **recording**
- the daemon owns **real-robot replay**
- the shared contract is the `scaled_action_*` policy-order columns

### Replay speed control

The daemon accepts an optional speed factor:

```bash
# Normal speed (1.0x)
python dog_cli.py --host 192.168.1.100 replay /path/to/recording.csv 1.0

# Fast forward (2.0x)
python dog_cli.py --host 192.168.1.100 replay /path/to/recording.csv 2.0

# Slow motion (0.5x)
python dog_cli.py --host 192.168.1.100 replay /path/to/recording.csv 0.5
```

### Skip feedback check (open-loop replay)

By default, the daemon checks motor feedback freshness (500ms timeout) before each replay frame. If feedback is lost, replay is aborted. For open-loop testing scenarios, you can skip this check:

```bash
# Replay without feedback check
python dog_cli.py --host 192.168.1.100 replay /path/to/recording.csv 1.0 --no-feedback-check

# Fast replay without feedback check
python dog_cli.py --host 192.168.1.100 replay /path/to/recording.csv 2.0 --no-feedback-check
```

This is useful when:
- Testing the replay pipeline without real motors connected
- Running headless simulations
- Debugging recorded trajectories without hardware dependencies

### CSV compatibility

The generated CSV is fully compatible with the daemon replay interface. Required columns:

- `timestamp_ms` or `sim_time` ✅
- `scaled_action_0` through `scaled_action_11` ✅

## Offline limit analysis

Use the included `analyze_limits.py` script to check if `scaled_action` values exceed joint limits:

```bash
# Basic analysis
uv run python analyze_limits.py output/recording.csv

# Save violations to CSV
uv run python analyze_limits.py output/recording.csv --output output/violations.csv

# Show all violations with per-joint summary
uv run python analyze_limits.py output/recording.csv --verbose --by-joint
```

Example output:

```
JOINT LIMIT ANALYSIS SUMMARY
====================================================================================================
Joint       Idx    Limit Low   Limit High          Min          Max       Mean        Std   Violations   Near Limit   Max Exceed
----------------------------------------------------------------------------------------------------
LF_Knee       8    -1.221730     0.359928    -0.257664     0.615312   0.169907   0.218613           85          107     0.255384
LR_Knee       9    -1.221730     0.359928    -0.240354     0.665396   0.076277   0.229744           75           85     0.305468
RF_Knee      10    -0.359928     1.221729    -0.594578     0.315915  -0.105936   0.244241           87           98     0.234650
RR_Knee      11    -0.359928     1.221729    -0.648171     0.350452  -0.100653   0.239419           79          101     0.288243
====================================================================================================
```

The script reports:
- **Violations**: Number of samples exceeding joint limits
- **Near Limit**: Samples within 5% of the limit boundary
- **Max Exceed**: Maximum exceedance amount in radians
- Per-joint breakdown showing which joints (HipA/HipF/Knee) and which legs (LF/LR/RF/RR) are affected

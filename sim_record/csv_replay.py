"""CSV replay tool for MuJoCo - slow playback of recorded joint targets."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path
from typing import List, Optional

import mujoco
import mujoco.viewer
import numpy as np

KP = 25.0
KD = 0.5
TAU_LIMIT = np.array([17.0, 17.0, 25.0] * 4, dtype=np.float64)

# Policy joint order -> Sim joint order
POLICY_TO_SIM = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)


def load_csv(path: Path) -> List[dict]:
    """Load CSV file into list of dicts."""
    rows = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def extract_target_q(row: dict) -> np.ndarray:
    """Extract target_q from CSV row (policy order)."""
    target = np.zeros(12, dtype=np.float64)
    for i in range(12):
        target[i] = float(row[f"target_q_{i}"])
    return target


def extract_sim_time(row: dict) -> float:
    """Extract sim_time from CSV row."""
    return float(row["sim_time"])


def main() -> int:
    ap = argparse.ArgumentParser(description="Replay CSV in MuJoCo with slow speed")
    ap.add_argument("--xml", required=True, type=Path, help="MuJoCo XML path")
    ap.add_argument("--csv", required=True, type=Path, help="Recorded CSV path")
    ap.add_argument("--sim-dt", type=float, default=0.001, help="Simulation timestep (smaller = slower physics)")
    ap.add_argument("--play-speed", type=float, default=1.0, help="Playback speed multiplier (1.0 = normal, 0.1 = 10x slower visual)")
    ap.add_argument("--headless", action="store_true", help="Run without viewer")
    args = ap.parse_args()

    rows = load_csv(args.csv)
    if not rows:
        print("CSV is empty")
        return 1

    model = mujoco.MjModel.from_xml_path(str(args.xml))
    model.opt.timestep = args.sim_dt
    data = mujoco.MjData(model)

    # Get default joint positions
    default_joint_sim = np.array(model.qpos0[7:19], dtype=np.float64)

    # Reset to initial state
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    mujoco.mj_step(model, data)

    # Prepare target sequence
    target_times = [extract_sim_time(r) for r in rows]
    target_qs_policy = [extract_target_q(r) for r in rows]
    target_qs_sim = [t[POLICY_TO_SIM] + default_joint_sim for t in target_qs_policy]

    print(f"Loaded {len(rows)} samples, sim_time range: {target_times[0]:.4f} - {target_times[-1]:.4f}")
    print(f"sim_dt={args.sim_dt}, play_speed={args.play_speed}")

    current_target_idx = 0
    start_wall_time = time.time()

    if args.headless:
        while data.time < target_times[-1] + 1.0:
            # Find current target
            while current_target_idx < len(target_times) - 1 and target_times[current_target_idx + 1] <= data.time:
                current_target_idx += 1

            target_q_sim = target_qs_sim[current_target_idx]
            q_sim = np.array(data.qpos[7:19], dtype=np.float64)
            dq_sim = np.array(data.qvel[6:18], dtype=np.float64)

            tau = (target_q_sim - q_sim) * KP - dq_sim * KD
            tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.distance = 3.0
            viewer.cam.azimuth = 90.0
            viewer.cam.elevation = -45.0

            while viewer.is_running() and data.time < target_times[-1] + 1.0:
                # Find current target
                while current_target_idx < len(target_times) - 1 and target_times[current_target_idx + 1] <= data.time:
                    current_target_idx += 1

                target_q_sim = target_qs_sim[current_target_idx]
                q_sim = np.array(data.qpos[7:19], dtype=np.float64)
                dq_sim = np.array(data.qvel[6:18], dtype=np.float64)

                tau = (target_q_sim - q_sim) * KP - dq_sim * KD
                tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
                data.ctrl[:] = tau
                mujoco.mj_step(model, data)

                # Slow down visual playback
                elapsed_wall = time.time() - start_wall_time
                elapsed_sim = data.time
                desired_wall = elapsed_sim / args.play_speed
                if elapsed_wall < desired_wall:
                    time.sleep(desired_wall - elapsed_wall)

                viewer.sync()

    print(f"Replay finished at sim_time={data.time:.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
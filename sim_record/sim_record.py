"""Standalone MuJoCo policy recorder derived from the reference auto_test.py.

The tool runs the locomotion policy in simulation, executes a schedule of
velocity commands, records policy observations/actions/targets into a CSV, and
produces a diagnostic plot that includes phase backgrounds and joint-limit
annotations.
"""

from __future__ import annotations

import argparse
import csv
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort

from schedule import Schedule, SchedulePhase, build_inline_schedule, load_schedule

SIM_TO_POLICY = np.array([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)
POLICY_TO_SIM = np.array([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)
JOINT_NAMES = [
    "LF_HipA",
    "LR_HipA",
    "RF_HipA",
    "RR_HipA",
    "LF_HipF",
    "LR_HipF",
    "RF_HipF",
    "RR_HipF",
    "LF_Knee",
    "LR_Knee",
    "RF_Knee",
    "RR_Knee",
]
JOINT_LIMITS = [
    (-0.7853982, 0.7853982),
    (-0.7853982, 0.7853982),
    (-0.7853982, 0.7853982),
    (-0.7853982, 0.7853982),
    (-1.2217658, 0.8726683),
    (-1.2217305, 0.8726683),
    (-0.8726999, 1.2217342),
    (-0.8726999, 1.2217305),
    (-1.2217299, 0.6),
    (-1.2217299, 0.6),
    (-0.6, 1.2217287),
    (-0.6, 1.2217287),
]
PHASE_COLORS = ["gray", "green", "red", "blue", "cyan", "orange", "purple", "gray"]
KP = 25.0
KD = 0.5
TAU_LIMIT = np.array([17.0, 17.0, 25.0] * 4, dtype=np.float64)
ACTION_SCALE = 0.25
FRAME_STACK = 10
OBS_DIM = 45
DECIMATION = 4


@dataclass
class RecorderConfig:
    """Run configuration for the simulator recorder."""

    xml_path: Path
    onnx_path: Path
    output_csv: Path
    plot_output: Optional[Path]
    schedule: Schedule
    sim_dt: float = 0.005
    action_scale: float = ACTION_SCALE
    knee_ratio: float = 1.0
    headless: bool = False
    save_npz: Optional[Path] = None


@dataclass
class RecordRow:
    """One CSV row captured at policy rate."""

    timestamp_ms: int
    step: int
    sim_time: float
    phase: int
    phase_name: str
    cmd: np.ndarray
    obs: np.ndarray
    raw_action: np.ndarray
    scaled_action: np.ndarray
    target_q: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray


def quat_wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert a MuJoCo quaternion from ``wxyz`` to ``xyzw`` order."""
    return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)


def quat_rotate_inverse_xyzw(quat_xyzw: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate a vector by the inverse of a unit quaternion in ``xyzw`` order."""
    x, y, z, w = quat_xyzw
    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return rot.T @ vec


def phase_color(name: str, index: int) -> str:
    """Choose a display color for one phase."""
    if "forward" in name.lower():
        return "green"
    if "back" in name.lower():
        return "red"
    if "left" in name.lower():
        return "blue"
    if "right" in name.lower():
        return "cyan"
    if "turnl" in name.lower():
        return "orange"
    if "turnr" in name.lower():
        return "purple"
    return PHASE_COLORS[index % len(PHASE_COLORS)]


class ScheduleCursor:
    """Resolve the active command phase from simulation time."""

    def __init__(self, schedule: Schedule):
        """Create a cursor for a validated schedule.

        Args:
            schedule: Validated schedule.
        """
        self.schedule = schedule
        self.boundaries: List[Tuple[float, SchedulePhase, int]] = []
        elapsed = 0.0
        for idx, phase in enumerate(schedule.phases):
            self.boundaries.append((elapsed, phase, idx))
            elapsed += phase.duration
        self.total_time = elapsed

    def command_at(self, sim_time: float) -> Tuple[np.ndarray, int, str]:
        """Return the active command and phase info for one simulation time."""
        current = self.boundaries[-1]
        for start, phase, idx in self.boundaries:
            if sim_time < start + phase.duration:
                current = (start, phase, idx)
                break
        _, phase, idx = current
        return np.asarray(phase.cmd, dtype=np.float64), idx, phase.name

    def segments(self) -> List[Tuple[float, float, str]]:
        """Return phase segments for plotting."""
        out: List[Tuple[float, float, str]] = []
        for start, phase, _ in self.boundaries:
            out.append((start, start + phase.duration, phase.name))
        return out


class SimRecorder:
    """Policy-in-the-loop MuJoCo recorder."""

    def __init__(self, cfg: RecorderConfig):
        """Initialize the recorder.

        Args:
            cfg: Run configuration.
        """
        self.cfg = cfg
        self.cursor = ScheduleCursor(cfg.schedule)
        self.records: List[RecordRow] = []
        self.default_joint_sim: np.ndarray = np.zeros(12, dtype=np.float64)
        self.default_joint_policy: np.ndarray = np.zeros(12, dtype=np.float64)
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.session: Optional[ort.InferenceSession] = None
        self.hist_obs: deque[np.ndarray] = deque(maxlen=FRAME_STACK)
        self.action_policy = np.zeros(12, dtype=np.float64)
        self.target_q_sim = np.zeros(12, dtype=np.float64)

    def load(self) -> None:
        """Load MuJoCo and ONNX assets and reset recorder state."""
        self.model = mujoco.MjModel.from_xml_path(str(self.cfg.xml_path))
        self.model.opt.timestep = self.cfg.sim_dt
        self.data = mujoco.MjData(self.model)
        self.session = ort.InferenceSession(str(self.cfg.onnx_path))
        self.default_joint_sim = np.array(self.model.qpos0[7:19], dtype=np.float64)
        self.default_joint_policy = self.default_joint_sim[SIM_TO_POLICY].copy()
        self.target_q_sim = self.default_joint_sim.copy()
        self.action_policy = np.zeros(12, dtype=np.float64)
        self.hist_obs.clear()
        for _ in range(FRAME_STACK):
            self.hist_obs.append(np.zeros(OBS_DIM, dtype=np.float32))
        self.records.clear()
        self.data.qpos[:] = self.model.qpos0
        self.data.qvel[:] = 0.0
        mujoco.mj_step(self.model, self.data)

    def run(self) -> None:
        """Run the schedule in MuJoCo and store all policy-rate samples."""
        self.load()
        assert self.model is not None and self.data is not None
        if self.cfg.headless:
            while self.data.time < self.cursor.total_time:
                self._step_once()
        else:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                viewer.cam.distance = 3.0
                viewer.cam.azimuth = 90.0
                viewer.cam.elevation = -45.0
                while viewer.is_running() and self.data.time < self.cursor.total_time:
                    self._step_once()
                    viewer.sync()
        self.write_csv(self.cfg.output_csv)
        if self.cfg.save_npz is not None:
            self.write_npz(self.cfg.save_npz)
        if self.cfg.plot_output is not None:
            self.plot(self.cfg.plot_output)

    def _step_once(self) -> None:
        """Advance the simulation by one low-level timestep."""
        assert self.model is not None and self.data is not None and self.session is not None
        sim_time = float(self.data.time)
        cmd, phase_idx, phase_name = self.cursor.command_at(sim_time)

        q_sim_abs = np.array(self.data.qpos[7:19], dtype=np.float64)
        dq_sim = np.array(self.data.qvel[6:18], dtype=np.float64)
        q_policy_abs = q_sim_abs[SIM_TO_POLICY]
        dq_policy = dq_sim[SIM_TO_POLICY]
        q_policy_rel = q_policy_abs - self.default_joint_policy

        step_count = int(round(sim_time / self.cfg.sim_dt))
        if step_count % DECIMATION == 0:
            mj_quat = np.array(self.data.qpos[3:7], dtype=np.float64)
            quat_xyzw = quat_wxyz_to_xyzw(mj_quat)
            omega = self._extract_angular_velocity()
            gravity = quat_rotate_inverse_xyzw(quat_xyzw, np.array([0.0, 0.0, -1.0], dtype=np.float64))

            q_obs = q_policy_rel.copy()
            dq_obs = dq_policy.copy()
            if self.cfg.knee_ratio != 0.0:
                q_obs[8:12] /= self.cfg.knee_ratio
                dq_obs[8:12] /= self.cfg.knee_ratio

            obs = np.concatenate([omega, gravity, cmd, q_obs, dq_obs, self.action_policy]).astype(np.float32)
            obs = np.clip(obs, -100.0, 100.0)
            self.hist_obs.append(obs)

            input_name = self.session.get_inputs()[0].name
            policy_input = np.concatenate(list(self.hist_obs), axis=0)[None, :]
            raw_action = self.session.run(None, {input_name: policy_input})[0][0]
            raw_action = np.asarray(raw_action, dtype=np.float64).reshape(-1)
            raw_action = np.clip(raw_action, -100.0, 100.0)
            scaled_action = raw_action * self.cfg.action_scale

            self.action_policy = raw_action.copy()
            action_for_target = scaled_action.copy()
            action_for_target[8:12] *= self.cfg.knee_ratio
            target_q_sim = action_for_target[POLICY_TO_SIM] + self.default_joint_sim
            self.target_q_sim = target_q_sim

            self.records.append(
                RecordRow(
                    timestamp_ms=int(round(sim_time * 1000.0)),
                    step=len(self.records),
                    sim_time=sim_time,
                    phase=phase_idx,
                    phase_name=phase_name,
                    cmd=cmd.copy(),
                    obs=obs.copy(),
                    raw_action=raw_action.copy(),
                    scaled_action=scaled_action.copy(),
                    target_q=target_q_sim[SIM_TO_POLICY].copy(),
                    joint_pos=q_policy_rel.copy(),
                    joint_vel=dq_policy.copy(),
                )
            )

        tau = (self.target_q_sim - q_sim_abs) * KP - dq_sim * KD
        tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)

    def _extract_angular_velocity(self) -> np.ndarray:
        """Read the angular-velocity sensor if available, otherwise use qvel."""
        assert self.model is not None and self.data is not None
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
        if sensor_id != -1:
            adr = self.model.sensor_adr[sensor_id]
            dim = self.model.sensor_dim[sensor_id]
            if dim >= 3:
                return np.array(self.data.sensordata[adr : adr + 3], dtype=np.float64)
        return np.array(self.data.qvel[3:6], dtype=np.float64)

    def write_csv(self, path: Path) -> None:
        """Write recorded policy-rate samples to CSV.

        Args:
            path: Output CSV path.
        """
        path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "timestamp_ms",
            "step",
            "sim_time",
            "phase",
            "phase_name",
            "cmd_x",
            "cmd_y",
            "cmd_yaw",
        ]
        fieldnames += [f"obs_{i}" for i in range(OBS_DIM)]
        fieldnames += [f"raw_action_{i}" for i in range(12)]
        fieldnames += [f"scaled_action_{i}" for i in range(12)]
        fieldnames += [f"target_q_{i}" for i in range(12)]
        fieldnames += [f"joint_pos_{i}" for i in range(12)]
        fieldnames += [f"joint_vel_{i}" for i in range(12)]
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.records:
                out: Dict[str, object] = {
                    "timestamp_ms": row.timestamp_ms,
                    "step": row.step,
                    "sim_time": f"{row.sim_time:.6f}",
                    "phase": row.phase,
                    "phase_name": row.phase_name,
                    "cmd_x": f"{row.cmd[0]:.6f}",
                    "cmd_y": f"{row.cmd[1]:.6f}",
                    "cmd_yaw": f"{row.cmd[2]:.6f}",
                }
                out.update({f"obs_{i}": f"{float(v):.6f}" for i, v in enumerate(row.obs)})
                out.update({f"raw_action_{i}": f"{float(v):.6f}" for i, v in enumerate(row.raw_action)})
                out.update({f"scaled_action_{i}": f"{float(v):.6f}" for i, v in enumerate(row.scaled_action)})
                out.update({f"target_q_{i}": f"{float(v):.6f}" for i, v in enumerate(row.target_q)})
                out.update({f"joint_pos_{i}": f"{float(v):.6f}" for i, v in enumerate(row.joint_pos)})
                out.update({f"joint_vel_{i}": f"{float(v):.6f}" for i, v in enumerate(row.joint_vel)})
                writer.writerow(out)

    def write_npz(self, path: Path) -> None:
        """Optionally export the same records as a NumPy archive."""
        arrays: Dict[str, np.ndarray] = {
            "timestamp_ms": np.asarray([r.timestamp_ms for r in self.records], dtype=np.int64),
            "step": np.asarray([r.step for r in self.records], dtype=np.int64),
            "sim_time": np.asarray([r.sim_time for r in self.records], dtype=np.float64),
            "phase": np.asarray([r.phase for r in self.records], dtype=np.int64),
            "phase_name": np.asarray([r.phase_name for r in self.records], dtype=object),
            "cmd": np.asarray([r.cmd for r in self.records], dtype=np.float64),
            "obs": np.asarray([r.obs for r in self.records], dtype=np.float32),
            "raw_action": np.asarray([r.raw_action for r in self.records], dtype=np.float64),
            "scaled_action": np.asarray([r.scaled_action for r in self.records], dtype=np.float64),
            "target_q": np.asarray([r.target_q for r in self.records], dtype=np.float64),
            "joint_pos": np.asarray([r.joint_pos for r in self.records], dtype=np.float64),
            "joint_vel": np.asarray([r.joint_vel for r in self.records], dtype=np.float64),
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, **arrays)

    def plot(self, path: Path) -> None:
        """Render the diagnostic 6x3 figure with limit annotations."""
        t = np.asarray([r.sim_time for r in self.records], dtype=np.float64)
        obs = np.asarray([r.obs for r in self.records], dtype=np.float64)
        raw_action = np.asarray([r.raw_action for r in self.records], dtype=np.float64)
        scaled_action = np.asarray([r.scaled_action for r in self.records], dtype=np.float64)
        target_q = np.asarray([r.target_q for r in self.records], dtype=np.float64)
        joint_pos = np.asarray([r.joint_pos for r in self.records], dtype=np.float64)
        joint_vel = np.asarray([r.joint_vel for r in self.records], dtype=np.float64)

        fig = plt.figure(figsize=(18, 18))
        gs = GridSpec(6, 3, figure=fig, hspace=0.35, wspace=0.25)
        segments = self.cursor.segments()

        def add_phase_bg(ax: plt.Axes) -> None:
            for idx, (start, end, name) in enumerate(segments):
                ax.axvspan(start, end, alpha=0.15, color=phase_color(name, idx))

        def add_limit_annotations(ax: plt.Axes, indices: Sequence[int], values: np.ndarray) -> None:
            for local_idx, joint_idx in enumerate(indices):
                low, high = JOINT_LIMITS[joint_idx]
                ax.axhline(low, color="red", linestyle=":", linewidth=0.8)
                ax.axhline(high, color="red", linestyle=":", linewidth=0.8)
                val = values[:, local_idx]
                viol = (val < low) | (val > high)
                near = ((val - low) <= (high - low) * 0.05) | ((high - val) <= (high - low) * 0.05)
                if np.any(viol):
                    ax.scatter(t[viol], val[viol], s=12, color="red", alpha=0.8)
                if np.any(near):
                    ax.scatter(t[near], val[near], s=10, color="darkred", alpha=0.5)

        fig.text(0.5, 0.97, "Network Input (Observation) - 45 dims", ha="center", fontsize=12, fontweight="bold")

        ax = fig.add_subplot(gs[0, 0]); add_phase_bg(ax)
        ax.plot(t, obs[:, 0], label="wx"); ax.plot(t, obs[:, 1], label="wy"); ax.plot(t, obs[:, 2], label="wz")
        ax.set_title("[Input] Angular Velocity (3)"); ax.legend(); ax.grid(True, alpha=0.3)

        ax = fig.add_subplot(gs[0, 1]); add_phase_bg(ax)
        ax.plot(t, obs[:, 3], label="gx"); ax.plot(t, obs[:, 4], label="gy"); ax.plot(t, obs[:, 5], label="gz")
        ax.set_title("[Input] Projected Gravity (3)"); ax.legend(); ax.grid(True, alpha=0.3)

        ax = fig.add_subplot(gs[0, 2]); add_phase_bg(ax)
        ax.plot(t, obs[:, 6], label="cmd_x"); ax.plot(t, obs[:, 7], label="cmd_y"); ax.plot(t, obs[:, 8], label="cmd_yaw")
        ax.set_title("[Input] Velocity Command (3)"); ax.legend(); ax.grid(True, alpha=0.3)

        family_indices = [(0, 4), (4, 8), (8, 12)]
        family_titles = ["HipA", "HipF", "Knee"]

        for col, (start, end) in enumerate(family_indices):
            ax = fig.add_subplot(gs[1, col]); add_phase_bg(ax)
            vals = joint_pos[:, start:end]
            for j in range(start, end):
                ax.plot(t, vals[:, j - start], label=JOINT_NAMES[j])
            add_limit_annotations(ax, list(range(start, end)), vals)
            ax.set_title(f"[Input] Joint Position - {family_titles[col]} (4)")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

        for col, (start, end) in enumerate(family_indices):
            ax = fig.add_subplot(gs[2, col]); add_phase_bg(ax)
            vals = joint_vel[:, start:end]
            for j in range(start, end):
                ax.plot(t, vals[:, j - start], label=JOINT_NAMES[j])
            ax.set_title(f"[Input] Joint Velocity - {family_titles[col]} (4)")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

        for col, (start, end) in enumerate(family_indices):
            ax = fig.add_subplot(gs[3, col]); add_phase_bg(ax)
            vals = obs[:, 33 + start : 33 + end]
            for j in range(start, end):
                ax.plot(t, vals[:, j - start], label=JOINT_NAMES[j])
            ax.set_title(f"[Input] Last Action - {family_titles[col]} (4)")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

        for col, (start, end) in enumerate(family_indices):
            ax = fig.add_subplot(gs[4, col]); add_phase_bg(ax)
            vals = raw_action[:, start:end]
            for j in range(start, end):
                ax.plot(t, vals[:, j - start], label=JOINT_NAMES[j])
            ax.set_title(f"[Output] Raw Action - {family_titles[col]} (4)")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

        for col, (start, end) in enumerate(family_indices):
            ax = fig.add_subplot(gs[5, col]); add_phase_bg(ax)
            vals = target_q[:, start:end]
            for j in range(start, end):
                ax.plot(t, vals[:, j - start], label=JOINT_NAMES[j])
            add_limit_annotations(ax, list(range(start, end)), vals)
            ax.set_title(f"[Target] Target Joint Pos - {family_titles[col]} (4)")
            ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
            ax.set_xlabel("time (s)")

        legend_handles = [plt.Line2D([], [], color=phase_color(name, idx), linewidth=8, alpha=0.3, label=name) for idx, (_, _, name) in enumerate(segments)]
        fig.legend(handles=legend_handles, loc="upper center", ncol=max(1, len(legend_handles)), bbox_to_anchor=(0.5, 0.94))
        path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(path, dpi=160)
        plt.close(fig)


def build_argparser() -> argparse.ArgumentParser:
    """Create the command-line parser."""
    ap = argparse.ArgumentParser(description="Record ONNX policy actions in MuJoCo")
    ap.add_argument("--onnx", required=True, type=Path, help="Policy ONNX path")
    ap.add_argument("--xml", required=True, type=Path, help="MuJoCo XML path")
    ap.add_argument("--schedule", type=Path, help="YAML command schedule")
    ap.add_argument("--cmd", type=float, nargs=3, metavar=("CMD_X", "CMD_Y", "CMD_YAW"), help="Inline command triplet")
    ap.add_argument("--duration", type=float, help="Inline command duration in seconds")
    ap.add_argument("--output", required=True, type=Path, help="Output CSV path")
    ap.add_argument("--plot-output", type=Path, help="Optional output PNG path")
    ap.add_argument("--npz-output", type=Path, help="Optional output NPZ path")
    ap.add_argument("--headless", action="store_true", help="Run without the interactive MuJoCo viewer")
    ap.add_argument("--sim-dt", type=float, default=0.005, help="Simulation time step")
    ap.add_argument("--action-scale", type=float, default=ACTION_SCALE, help="Policy action scale")
    ap.add_argument("--knee-ratio", type=float, default=1.0, help="Observation/output knee ratio correction")
    return ap


def build_schedule_from_args(args: argparse.Namespace) -> Schedule:
    """Resolve schedule selection from CLI arguments."""
    if args.schedule is not None:
        return load_schedule(args.schedule)
    if args.cmd is not None and args.duration is not None:
        return build_inline_schedule(args.cmd, args.duration, name="inline")
    raise ValueError("Either --schedule or (--cmd and --duration) must be provided")


def main(argv: Optional[Iterable[str]] = None) -> int:
    """Program entry point."""
    args = build_argparser().parse_args(list(argv) if argv is not None else None)
    schedule = build_schedule_from_args(args)
    recorder = SimRecorder(
        RecorderConfig(
            xml_path=args.xml,
            onnx_path=args.onnx,
            output_csv=args.output,
            plot_output=args.plot_output,
            schedule=schedule,
            sim_dt=float(args.sim_dt),
            action_scale=float(args.action_scale),
            knee_ratio=float(args.knee_ratio),
            headless=bool(args.headless),
            save_npz=args.npz_output,
        )
    )
    recorder.run()
    print(f"CSV written to: {args.output}")
    if args.plot_output is not None:
        print(f"Plot written to: {args.plot_output}")
    if args.npz_output is not None:
        print(f"NPZ written to: {args.npz_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

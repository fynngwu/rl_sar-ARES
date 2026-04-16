"""Utilities for loading and validating YAML command schedules."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence

import yaml


@dataclass(frozen=True)
class SchedulePhase:
    """One command phase used by :mod:`sim_record`.

    Attributes:
        cmd: Command tuple ``(cmd_x, cmd_y, cmd_yaw)``.
        duration: Phase duration in seconds.
        name: Human-readable phase label.
    """

    cmd: Sequence[float]
    duration: float
    name: str


@dataclass(frozen=True)
class Schedule:
    """Validated schedule container."""

    phases: List[SchedulePhase]

    @property
    def total_time(self) -> float:
        """Return the total schedule duration in seconds."""
        return sum(phase.duration for phase in self.phases)


DEFAULT_NAME_PREFIX = "phase"


def normalize_schedule(phases: Iterable[dict]) -> Schedule:
    """Validate and normalize raw phase dictionaries.

    Args:
        phases: Iterable of dictionaries with ``cmd`` and ``duration`` entries.

    Returns:
        Validated :class:`Schedule`.
    """
    normalized: List[SchedulePhase] = []
    for index, item in enumerate(phases):
        if "cmd" not in item or "duration" not in item:
            raise ValueError(f"phase {index} must contain 'cmd' and 'duration'")
        cmd = list(item["cmd"])
        if len(cmd) != 3:
            raise ValueError(f"phase {index} cmd must contain exactly 3 floats")
        duration = float(item["duration"])
        if duration <= 0.0:
            raise ValueError(f"phase {index} duration must be positive")
        name = str(item.get("name", f"{DEFAULT_NAME_PREFIX}_{index}"))
        normalized.append(SchedulePhase(cmd=tuple(float(v) for v in cmd), duration=duration, name=name))
    if not normalized:
        raise ValueError("schedule must contain at least one phase")
    return Schedule(phases=normalized)


def load_schedule(path: Path) -> Schedule:
    """Load a YAML schedule file.

    Args:
        path: YAML file path.

    Returns:
        Validated :class:`Schedule`.
    """
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "schedule" not in data:
        raise ValueError("schedule YAML must be a mapping with a top-level 'schedule' key")
    if not isinstance(data["schedule"], list):
        raise ValueError("schedule YAML 'schedule' entry must be a list")
    return normalize_schedule(data["schedule"])


def build_inline_schedule(cmd: Sequence[float], duration: float, name: str = "inline") -> Schedule:
    """Build a one-phase schedule from command-line parameters.

    Args:
        cmd: ``(cmd_x, cmd_y, cmd_yaw)``.
        duration: Phase duration in seconds.
        name: Optional phase label.

    Returns:
        One-phase :class:`Schedule`.
    """
    return normalize_schedule([{"cmd": list(cmd), "duration": float(duration), "name": name}])

"""Offline analysis of scaled_action limit violations from sim_record CSV files."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

# Policy-order joint names and limits (from motor_config.py)
JOINT_NAMES: List[str] = [
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

KNEE_RATIO: float = 1.667

_XML_MIN: List[float] = [
    -0.7853982,
    -0.7853982,
    -0.7853982,
    -0.7853982,
    -1.2217658,
    -1.2217305,
    -0.8726999,
    -0.8726999,
    -1.2217299 * KNEE_RATIO,
    -1.2217299 * KNEE_RATIO,
    -0.6,
    -0.6,
]

_XML_MAX: List[float] = [
    0.7853982,
    0.7853982,
    0.7853982,
    0.7853982,
    0.8726683,
    0.8726683,
    1.2217342,
    1.2217305,
    0.6,
    0.6,
    1.2217287 * KNEE_RATIO,
    1.2217287 * KNEE_RATIO,
]

JOINT_LIMITS: List[Tuple[float, float]] = []
for idx, (low, high) in enumerate(zip(_XML_MIN, _XML_MAX)):
    if idx >= 8:
        JOINT_LIMITS.append((low / KNEE_RATIO, high / KNEE_RATIO))
    else:
        JOINT_LIMITS.append((low, high))


@dataclass
class LimitViolation:
    """Records a single limit violation event."""

    joint_name: str
    joint_index: int
    timestamp_ms: float
    sim_time: float
    value: float
    limit_low: float
    limit_high: float
    violation_type: str  # "low" or "high"
    exceed_amount: float


@dataclass
class JointSummary:
    """Summary statistics for one joint."""

    joint_name: str
    joint_index: int
    limit_low: float
    limit_high: float
    min_value: float
    max_value: float
    mean_value: float
    std_value: float
    violation_count: int
    max_violation_low: float  # Maximum exceed amount at low limit
    max_violation_high: float  # Maximum exceed amount at high limit
    near_limit_count: int  # Within 5% of limit


def read_csv(path: Path) -> Dict[str, np.ndarray]:
    """Load a sim_record CSV into numpy arrays keyed by column name.

    Args:
        path: CSV file path.

    Returns:
        Dictionary mapping column name to a numpy array.
    """
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"CSV is empty: {path}")
    
    # Determine numeric columns (skip non-numeric like phase_name)
    fieldnames = list(reader.fieldnames or [])
    numeric_columns: set[str] = set()
    for key in fieldnames:
        try:
            float(rows[0][key])
            numeric_columns.add(key)
        except (ValueError, TypeError):
            pass
    
    columns: Dict[str, List[float]] = {name: [] for name in numeric_columns}
    for row in rows:
        for key in numeric_columns:
            columns[key].append(float(row[key]))
    
    out: Dict[str, np.ndarray] = {}
    for key, values in columns.items():
        out[key] = np.asarray(values, dtype=float)
    return out


def extract_scaled_actions(data: Dict[str, np.ndarray]) -> np.ndarray:
    """Extract scaled_action columns as a 2D array (samples x 12 joints).

    Args:
        data: Loaded CSV dictionary.

    Returns:
        Numpy array of shape (N, 12) containing scaled_action values.
    """
    actions = []
    for j in range(12):
        col_name = f"scaled_action_{j}"
        if col_name not in data:
            raise ValueError(f"Missing column: {col_name}")
        actions.append(data[col_name])
    return np.column_stack(actions)


def analyze_limits(data: Dict[str, np.ndarray]) -> Tuple[List[JointSummary], List[LimitViolation]]:
    """Analyze scaled_action values against joint limits.

    Args:
        data: Loaded CSV dictionary.

    Returns:
        Tuple of (joint_summaries, all_violations).
    """
    actions = extract_scaled_actions(data)
    timestamps = data.get("timestamp_ms", np.arange(actions.shape[0]) * 20)
    sim_times = data.get("sim_time", timestamps / 1000.0)

    all_violations: List[LimitViolation] = []
    summaries: List[JointSummary] = []

    for j in range(12):
        joint_name = JOINT_NAMES[j]
        low, high = JOINT_LIMITS[j]
        values = actions[:, j]

        min_val = float(np.min(values))
        max_val = float(np.max(values))
        mean_val = float(np.mean(values))
        std_val = float(np.std(values))

        # Find violations
        below_low = values < low
        above_high = values > high
        violation_count = int(np.sum(below_low) + np.sum(above_high))

        # Calculate exceed amounts
        exceed_low = np.where(below_low, low - values, 0.0)
        exceed_high = np.where(above_high, values - high, 0.0)
        max_exceed_low = float(np.max(exceed_low)) if violation_count > 0 else 0.0
        max_exceed_high = float(np.max(exceed_high)) if violation_count > 0 else 0.0

        # Near-limit check (within 5% of range)
        range_size = high - low
        threshold = range_size * 0.05
        near_low = values - low <= threshold
        near_high = high - values <= threshold
        near_limit_count = int(np.sum(near_low | near_high))

        # Record individual violations
        for i in range(len(values)):
            if below_low[i]:
                all_violations.append(LimitViolation(
                    joint_name=joint_name,
                    joint_index=j,
                    timestamp_ms=float(timestamps[i]),
                    sim_time=float(sim_times[i]),
                    value=float(values[i]),
                    limit_low=low,
                    limit_high=high,
                    violation_type="low",
                    exceed_amount=float(low - values[i]),
                ))
            elif above_high[i]:
                all_violations.append(LimitViolation(
                    joint_name=joint_name,
                    joint_index=j,
                    timestamp_ms=float(timestamps[i]),
                    sim_time=float(sim_times[i]),
                    value=float(values[i]),
                    limit_low=low,
                    limit_high=high,
                    violation_type="high",
                    exceed_amount=float(values[i] - high),
                ))

        summaries.append(JointSummary(
            joint_name=joint_name,
            joint_index=j,
            limit_low=low,
            limit_high=high,
            min_value=min_val,
            max_value=max_val,
            mean_value=mean_val,
            std_value=std_val,
            violation_count=violation_count,
            max_violation_low=max_exceed_low,
            max_violation_high=max_exceed_high,
            near_limit_count=near_limit_count,
        ))

    return summaries, all_violations


def print_summary(summaries: List[JointSummary]) -> None:
    """Print a summary table of all joints."""
    print("\n" + "=" * 100)
    print("JOINT LIMIT ANALYSIS SUMMARY")
    print("=" * 100)
    print(f"{'Joint':<10} {'Idx':>4} {'Limit Low':>12} {'Limit High':>12} "
          f"{'Min':>12} {'Max':>12} {'Mean':>10} {'Std':>10} "
          f"{'Violations':>12} {'Near Limit':>12} {'Max Exceed':>12}")
    print("-" * 100)

    for s in summaries:
        max_exceed = max(s.max_violation_low, s.max_violation_high)
        print(f"{s.joint_name:<10} {s.joint_index:>4} "
              f"{s.limit_low:>12.6f} {s.limit_high:>12.6f} "
              f"{s.min_value:>12.6f} {s.max_value:>12.6f} "
              f"{s.mean_value:>10.6f} {s.std_value:>10.6f} "
              f"{s.violation_count:>12d} {s.near_limit_count:>12d} "
              f"{max_exceed:>12.6f}")

    print("=" * 100)
    total_violations = sum(s.violation_count for s in summaries)
    total_near_limit = sum(s.near_limit_count for s in summaries)
    print(f"{'TOTAL':<10} {'':>4} {'':>12} {'':>12} {'':>12} {'':>12} "
          f"{'':>10} {'':>10} {total_violations:>12d} {total_near_limit:>12d} {'':>12}")
    print("=" * 100 + "\n")


def print_violations(violations: List[LimitViolation], max_display: int = 50) -> None:
    """Print detailed violation events."""
    if not violations:
        print("No limit violations detected.\n")
        return

    print(f"\nDETAILED VIOLATIONS ({len(violations)} total events)")
    print("=" * 100)
    print(f"{'Joint':<10} {'Time (ms)':>12} {'Sim Time':>10} {'Value':>12} "
          f"{'Limit':>12} {'Type':>8} {'Exceed':>10}")
    print("-" * 100)

    display_count = min(len(violations), max_display)
    for v in violations[:display_count]:
        limit_val = v.limit_low if v.violation_type == "low" else v.limit_high
        print(f"{v.joint_name:<10} {v.timestamp_ms:>12.1f} {v.sim_time:>10.4f} "
              f"{v.value:>12.6f} {limit_val:>12.6f} {v.violation_type:>8} {v.exceed_amount:>10.6f}")

    if len(violations) > max_display:
        print(f"... and {len(violations) - max_display} more violations")

    print("=" * 100 + "\n")


def group_violations_by_joint(violations: List[LimitViolation]) -> Dict[str, List[LimitViolation]]:
    """Group violations by joint name."""
    result: Dict[str, List[LimitViolation]] = {}
    for v in violations:
        if v.joint_name not in result:
            result[v.joint_name] = []
        result[v.joint_name].append(v)
    return result


def print_violations_by_joint(violations: List[LimitViolation]) -> None:
    """Print violations grouped by joint."""
    if not violations:
        return

    grouped = group_violations_by_joint(violations)
    print("\nVIOLATIONS BY JOINT")
    print("=" * 100)

    for joint_name in JOINT_NAMES:
        if joint_name not in grouped:
            continue
        joint_violations = grouped[joint_name]
        low_viols = [v for v in joint_violations if v.violation_type == "low"]
        high_viols = [v for v in joint_violations if v.violation_type == "high"]

        print(f"\n{joint_name}:")
        if low_viols:
            print(f"  Low limit violations: {len(low_viols)}")
            print(f"    Max exceed: {max(v.exceed_amount for v in low_viols):.6f} rad")
            print(f"    First at: t={low_viols[0].timestamp_ms:.1f}ms, value={low_viols[0].value:.6f} rad")
        if high_viols:
            print(f"  High limit violations: {len(high_viols)}")
            print(f"    Max exceed: {max(v.exceed_amount for v in high_viols):.6f} rad")
            print(f"    First at: t={high_viols[0].timestamp_ms:.1f}ms, value={high_viols[0].value:.6f} rad")

    print("\n" + "=" * 100 + "\n")


def save_violations_csv(violations: List[LimitViolation], output_path: Path) -> None:
    """Save violations to a CSV file."""
    if not violations:
        print("No violations to save.")
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "joint_name", "joint_index", "timestamp_ms", "sim_time",
            "value", "limit_low", "limit_high", "violation_type", "exceed_amount"
        ])
        for v in violations:
            writer.writerow([
                v.joint_name, v.joint_index, v.timestamp_ms, v.sim_time,
                v.value, v.limit_low, v.limit_high, v.violation_type, v.exceed_amount
            ])

    print(f"Violations CSV saved to: {output_path}")


def build_argparser() -> argparse.ArgumentParser:
    """Create the CLI argument parser."""
    ap = argparse.ArgumentParser(description="Analyze sim_record CSV for scaled_action limit violations")
    ap.add_argument("csv", type=Path, help="Input CSV file path")
    ap.add_argument("--output", type=Path, help="Optional output path for violations CSV")
    ap.add_argument("--verbose", action="store_true", help="Print all violation events")
    ap.add_argument("--by-joint", action="store_true", help="Print violations grouped by joint")
    return ap


def main(argv=None) -> int:
    """Program entry point."""
    args = build_argparser().parse_args(argv)

    try:
        data = read_csv(args.csv)
        summaries, violations = analyze_limits(data)

        print_summary(summaries)

        if violations:
            print_violations(violations, max_display=50 if not args.verbose else len(violations))
            if args.by_joint:
                print_violations_by_joint(violations)

            if args.output:
                save_violations_csv(violations, args.output)
        else:
            print("No limit violations detected.\n")

        # Return non-zero if violations found
        return 1 if violations else 0

    except Exception as e:
        print(f"Error: {e}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

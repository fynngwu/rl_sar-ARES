#!/usr/bin/env python3
"""Plot motor torque curves from motor diagnostic CSV logs."""

import argparse
import csv
import math
from pathlib import Path


def parse_joint_filter(values):
    if not values:
        return None
    result = set()
    for item in values:
        for part in item.split(","):
            part = part.strip()
            if part:
                result.add(part)
    return result


def joint_matches(row, joint_filter):
    if joint_filter is None:
        return True
    return row["joint"] in joint_filter or row["joint_name"] in joint_filter


def load_rows(path, joint_filter, global_time):
    series = {}
    offline_points = {}
    time_state = {}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"t", "joint", "joint_name", "torque"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path} missing required columns: {sorted(missing)}")

        for row in reader:
            if not joint_matches(row, joint_filter):
                continue
            try:
                t = float(row["t"])
                torque = float(row["torque"])
            except (TypeError, ValueError):
                continue
            if not math.isfinite(t) or not math.isfinite(torque):
                continue

            key = f'{row["joint"]}:{row["joint_name"]}'
            if global_time:
                state = time_state.setdefault(key, {"offset": 0.0, "last_raw": None, "last_global": None})
                last_raw = state["last_raw"]
                if last_raw is not None and t < last_raw - 1e-6:
                    state["offset"] = (state["last_global"] or 0.0) + 1e-3
                    series.setdefault(key, {"t": [], "torque": []})
                    series[key]["t"].append(float("nan"))
                    series[key]["torque"].append(float("nan"))
                state["last_raw"] = t
                t = state["offset"] + t
                state["last_global"] = t

            series.setdefault(key, {"t": [], "torque": []})
            series[key]["t"].append(t)
            series[key]["torque"].append(torque)

            online = row.get("online", "1")
            if online in {"0", "false", "False"}:
                offline_points.setdefault(key, {"t": [], "torque": []})
                offline_points[key]["t"].append(t)
                offline_points[key]["torque"].append(torque)
    return series, offline_points


def default_output_path(input_path):
    return input_path.with_name(input_path.stem + "_torque.png")


def plot(series, offline_points, output, title, show, abs_torque):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required. Install it on the platform computer, "
            "for example: python3 -m pip install matplotlib"
        ) from exc

    if not series:
        raise SystemExit("No rows matched the requested joint filter.")

    fig, ax = plt.subplots(figsize=(12, 6))
    for key in sorted(series.keys(), key=lambda x: int(x.split(":", 1)[0])):
        t = series[key]["t"]
        torque = series[key]["torque"]
        if abs_torque:
            torque = [abs(v) for v in torque]
        ax.plot(t, torque, linewidth=1.4, label=key)

        if key in offline_points:
            ot = offline_points[key]["t"]
            ov = offline_points[key]["torque"]
            if abs_torque:
                ov = [abs(v) for v in ov]
            ax.scatter(ot, ov, s=14, marker="x", color="red", zorder=5)

    ax.set_xlabel("time (s)")
    ax.set_ylabel("|torque| (Nm)" if abs_torque else "torque (Nm)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    print(f"Saved plot to {output}")
    if show:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Plot time-torque curves from diagnostics_logs/*.csv"
    )
    parser.add_argument("csv", type=Path, help="diagnostic CSV path")
    parser.add_argument(
        "-j", "--joint", action="append",
        help="joint index or name; repeat or comma-separate, e.g. -j 9 -j 10 or -j LR_Knee"
    )
    parser.add_argument("-o", "--output", type=Path, help="output PNG path")
    parser.add_argument("--title", help="plot title")
    parser.add_argument("--abs", action="store_true", help="plot absolute torque")
    parser.add_argument(
        "--phase-time", action="store_true",
        help="plot raw phase-local time; default stitches phase-local time into global time"
    )
    parser.add_argument("--show", action="store_true", help="also open an interactive window")
    args = parser.parse_args()

    csv_path = args.csv
    if not csv_path.exists():
        raise SystemExit(f"CSV file not found: {csv_path}")

    joint_filter = parse_joint_filter(args.joint)
    output = args.output or default_output_path(csv_path)
    title = args.title or csv_path.name
    series, offline_points = load_rows(csv_path, joint_filter, not args.phase_time)
    plot(series, offline_points, output, title, args.show, args.abs)


if __name__ == "__main__":
    main()

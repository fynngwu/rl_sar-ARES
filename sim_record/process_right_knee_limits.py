#!/usr/bin/env python3
"""Process recording.csv to clamp right leg knee actions within their limits.

Right leg knee limits: RF_Knee(10), RR_Knee(11) -> (-0.36, 1.22)
Simply clamp values that exceed these limits.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

# Joint indices in policy order
RF_KNEE_IDX = 10  # Right Front Knee
RR_KNEE_IDX = 11  # Right Rear Knee

# Right leg knee limits from motor_config
RIGHT_KNEE_MIN = -0.36
RIGHT_KNEE_MAX = 1.22


def clamp_right_knee(value: float) -> float:
    """Clamp right knee action value to its limits."""
    return max(RIGHT_KNEE_MIN, min(RIGHT_KNEE_MAX, value))


def process_csv(input_path: Path, output_path: Path) -> int:
    """Process CSV file, clamping right knee actions."""
    clamped_count = 0

    with input_path.open("r", newline="") as infile:
        reader = csv.DictReader(infile)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)

    if not rows:
        print(f"Error: CSV file is empty: {input_path}")
        return -1

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", newline="") as outfile:
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()

        for row in rows:
            for idx in [RF_KNEE_IDX, RR_KNEE_IDX]:
                # Clamp scaled_action columns
                col_name = f"scaled_action_{idx}"
                if col_name in row:
                    original = float(row[col_name])
                    clamped = clamp_right_knee(original)
                    if abs(clamped - original) > 1e-6:
                        clamped_count += 1
                    row[col_name] = f"{clamped:.6f}"

                # Clamp target_q columns
                target_col = f"target_q_{idx}"
                if target_col in row:
                    original = float(row[target_col])
                    clamped = clamp_right_knee(original)
                    row[target_col] = f"{clamped:.6f}"

            writer.writerow(row)

    return clamped_count


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Clamp right leg knee actions within limits"
    )
    parser.add_argument(
        "input",
        type=Path,
        nargs="?",
        default=Path("output/recording.csv"),
        help="Input CSV file"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output CSV file (default: input_processed.csv)"
    )
    args = parser.parse_args(argv)

    input_path = args.input
    if not input_path.exists():
        print(f"Error: Input file does not exist: {input_path}")
        return 1

    output_path = args.output or input_path.parent / f"{input_path.stem}_processed.csv"

    print(f"Input: {input_path}")
    print(f"Output: {output_path}")
    print(f"Right knee limits: ({RIGHT_KNEE_MIN}, {RIGHT_KNEE_MAX})")

    clamped = process_csv(input_path, output_path)
    if clamped < 0:
        return 1

    print(f"Clamped {clamped} values")
    return 0


if __name__ == "__main__":
    sys.exit(main())
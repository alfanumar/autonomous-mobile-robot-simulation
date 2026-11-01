#!/usr/bin/env python3
"""Plot ground truth, quantized odometry, commands, and tracking error."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(handle)]


def plot(rows: list[dict[str, float]], output: Path) -> None:
    time = [row["time_s"] for row in rows]
    gt_x = [row["ground_truth_x_m"] for row in rows]
    gt_y = [row["ground_truth_y_m"] for row in rows]
    odom_x = [row["odometry_x_m"] for row in rows]
    odom_y = [row["odometry_y_m"] for row in rows]
    distance_error = [row["distance_error_m"] for row in rows]
    left = [row["left_command_mps"] for row in rows]
    right = [row["right_command_mps"] for row in rows]

    figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    axes[0].plot(gt_x, gt_y, label="Ground truth", linewidth=2.2)
    axes[0].plot(odom_x, odom_y, "--", label="Encoder odometry", linewidth=1.5)
    axes[0].plot([1, 1, 0, 0], [0, 1, 1, 0], "o", label="Waypoints")
    axes[0].set_title("Differential-drive route")
    axes[0].set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[0].axis("equal")
    axes[0].grid(alpha=0.3)
    axes[0].legend()

    axes[1].plot(time, distance_error, label="Distance error (m)", linewidth=2)
    axes[1].plot(time, left, label="Left command (m/s)", alpha=0.8)
    axes[1].plot(time, right, label="Right command (m/s)", alpha=0.8)
    axes[1].set_title("Feedback and wheel commands")
    axes[1].set_xlabel("time (s)")
    axes[1].grid(alpha=0.3)
    axes[1].legend()

    figure.suptitle("Autonomous Mobile Robot Simulation", fontsize=14)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plot(read_rows(args.csv_file), args.output)
    print(f"Saved plot to {args.output}")


if __name__ == "__main__":
    main()

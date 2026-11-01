#!/usr/bin/env python3
"""Run the autonomous differential-drive waypoint simulation."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from mobile_robot_sim import DifferentialDrive, Odometry, WaypointController


def default_waypoints() -> list[tuple[float, float]]:
    """A rectangular route that exercises straight motion and turns."""

    return [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]


def run_simulation(
    duration: float = 24.0,
    dt: float = 0.02,
    wheel_base: float = 0.30,
    max_wheel_speed: float = 0.80,
) -> list[dict[str, float]]:
    """Simulate ground truth, encoder odometry, and waypoint feedback."""

    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("duration and dt must be positive")
    waypoints = default_waypoints()
    robot = DifferentialDrive(wheel_base=wheel_base, max_wheel_speed=max_wheel_speed)
    odometry = Odometry(wheel_base=wheel_base)
    controller = WaypointController(wheel_base=wheel_base)
    rows: list[dict[str, float]] = []
    waypoint_index = 0
    completed_at_step: int | None = None
    steps = int(duration / dt)

    for step in range(steps + 1):
        time_s = step * dt
        target = waypoints[min(waypoint_index, len(waypoints) - 1)]
        output = controller.compute(odometry.pose, target)
        ground_truth, left_distance, right_distance = robot.step(output.command, dt)
        estimate = odometry.update(left_distance, right_distance)

        reached = controller.reached(estimate, target)
        if reached and waypoint_index < len(waypoints):
            waypoint_index += 1
            if waypoint_index >= len(waypoints) and completed_at_step is None:
                completed_at_step = step

        rows.append(
            {
                "time_s": time_s,
                "ground_truth_x_m": ground_truth.x,
                "ground_truth_y_m": ground_truth.y,
                "ground_truth_heading_rad": ground_truth.theta,
                "odometry_x_m": estimate.x,
                "odometry_y_m": estimate.y,
                "odometry_heading_rad": estimate.theta,
                "target_x_m": target[0],
                "target_y_m": target[1],
                "distance_error_m": output.distance_error,
                "heading_error_rad": output.heading_error,
                "left_command_mps": output.command.left_mps,
                "right_command_mps": output.command.right_mps,
                "waypoint_index": float(min(waypoint_index, len(waypoints))),
                "waypoint_reached": float(reached),
            }
        )
        if completed_at_step is not None and step - completed_at_step > int(0.75 / dt):
            break

    return rows


def write_csv(rows: list[dict[str, float]], output: Path) -> None:
    if not rows:
        raise ValueError("cannot write an empty result")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=24.0)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--output", type=Path, default=Path("results/trajectory.csv"))
    args = parser.parse_args()
    rows = run_simulation(duration=args.duration, dt=args.dt)
    write_csv(rows, args.output)
    final = rows[-1]
    print(f"Wrote {len(rows)} samples to {args.output}")
    print(
        "Final estimate: "
        f"x={final['odometry_x_m']:.3f} m, "
        f"y={final['odometry_y_m']:.3f} m, "
        f"heading={final['odometry_heading_rad']:.3f} rad"
    )
    print(f"Waypoints reached: {int(final['waypoint_index'])}/4")


if __name__ == "__main__":
    main()

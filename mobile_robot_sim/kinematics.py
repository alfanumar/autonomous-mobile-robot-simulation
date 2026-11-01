"""Differential-drive kinematics and a quantized wheel-odometry model."""

from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_angle(angle: float) -> float:
    """Return an angle in the interval [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Pose:
    """Planar robot pose in metres and radians."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass(frozen=True)
class WheelCommand:
    """Left and right wheel linear speeds in metres per second."""

    left_mps: float
    right_mps: float


class DifferentialDrive:
    """Integrate a differential-drive robot with bounded wheel speeds."""

    def __init__(self, wheel_base: float = 0.30, max_wheel_speed: float = 0.80) -> None:
        if wheel_base <= 0.0:
            raise ValueError("wheel_base must be positive")
        if max_wheel_speed <= 0.0:
            raise ValueError("max_wheel_speed must be positive")
        self.wheel_base = wheel_base
        self.max_wheel_speed = max_wheel_speed
        self.pose = Pose()

    def step(self, command: WheelCommand, dt: float) -> tuple[Pose, float, float]:
        """Advance the model and return pose plus actual wheel travel.

        The midpoint integration is accurate for the constant wheel command
        used during one simulation step. Returning wheel travel lets the
        odometry estimator use the same interface as encoder measurements.
        """

        if dt <= 0.0:
            raise ValueError("dt must be positive")

        left = max(-self.max_wheel_speed, min(self.max_wheel_speed, command.left_mps))
        right = max(-self.max_wheel_speed, min(self.max_wheel_speed, command.right_mps))
        linear = 0.5 * (left + right)
        angular = (right - left) / self.wheel_base

        midpoint_heading = self.pose.theta + 0.5 * angular * dt
        self.pose.x += linear * math.cos(midpoint_heading) * dt
        self.pose.y += linear * math.sin(midpoint_heading) * dt
        self.pose.theta = wrap_angle(self.pose.theta + angular * dt)
        return Pose(self.pose.x, self.pose.y, self.pose.theta), left * dt, right * dt


class Odometry:
    """Estimate pose by integrating quantized wheel travel."""

    def __init__(
        self,
        wheel_base: float = 0.30,
        wheel_radius: float = 0.05,
        ticks_per_revolution: int = 2048,
    ) -> None:
        if wheel_base <= 0.0 or wheel_radius <= 0.0:
            raise ValueError("wheel_base and wheel_radius must be positive")
        if ticks_per_revolution <= 0:
            raise ValueError("ticks_per_revolution must be positive")
        self.wheel_base = wheel_base
        self.tick_distance = 2.0 * math.pi * wheel_radius / ticks_per_revolution
        self.pose = Pose()

    def update(self, left_distance: float, right_distance: float) -> Pose:
        """Update the estimate from one pair of encoder wheel distances."""

        left_ticks = round(left_distance / self.tick_distance)
        right_ticks = round(right_distance / self.tick_distance)
        left = left_ticks * self.tick_distance
        right = right_ticks * self.tick_distance
        linear = 0.5 * (left + right)
        angular = (right - left) / self.wheel_base
        midpoint_heading = self.pose.theta + 0.5 * angular
        self.pose.x += linear * math.cos(midpoint_heading)
        self.pose.y += linear * math.sin(midpoint_heading)
        self.pose.theta = wrap_angle(self.pose.theta + angular)
        return Pose(self.pose.x, self.pose.y, self.pose.theta)

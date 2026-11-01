"""Simple feedback controller for waypoint tracking."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .kinematics import Pose, WheelCommand, wrap_angle


@dataclass(frozen=True)
class ControllerOutput:
    command: WheelCommand
    distance_error: float
    heading_error: float


class WaypointController:
    """Drive toward one waypoint using proportional feedback.

    This is intentionally a simple, explainable controller. It is not claimed
    to be a full navigation stack or a production-grade planner.
    """

    def __init__(
        self,
        wheel_base: float = 0.30,
        linear_gain: float = 0.9,
        angular_gain: float = 2.4,
        max_linear_speed: float = 0.55,
        max_angular_speed: float = 2.6,
        waypoint_tolerance: float = 0.08,
    ) -> None:
        if wheel_base <= 0.0 or linear_gain < 0.0 or angular_gain < 0.0:
            raise ValueError("gains and wheel_base must be non-negative")
        self.wheel_base = wheel_base
        self.linear_gain = linear_gain
        self.angular_gain = angular_gain
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.waypoint_tolerance = waypoint_tolerance

    def compute(self, pose: Pose, target: tuple[float, float]) -> ControllerOutput:
        """Compute wheel speeds and tracking errors for one target point."""

        dx = target[0] - pose.x
        dy = target[1] - pose.y
        distance = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_heading - pose.theta)

        if distance <= self.waypoint_tolerance:
            return ControllerOutput(WheelCommand(0.0, 0.0), distance, heading_error)

        linear = min(self.max_linear_speed, self.linear_gain * distance)
        # Do not drive forward quickly while facing away from the target.
        linear *= max(0.0, math.cos(heading_error))
        angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, self.angular_gain * heading_error),
        )
        left = linear - 0.5 * self.wheel_base * angular
        right = linear + 0.5 * self.wheel_base * angular

        peak = max(abs(left), abs(right), 1e-12)
        if peak > self.max_linear_speed:
            scale = self.max_linear_speed / peak
            left *= scale
            right *= scale
        return ControllerOutput(WheelCommand(left, right), distance, heading_error)

    def reached(self, pose: Pose, target: tuple[float, float]) -> bool:
        """Return whether the robot is within the configured tolerance."""

        return math.hypot(target[0] - pose.x, target[1] - pose.y) <= self.waypoint_tolerance

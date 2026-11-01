#!/usr/bin/env python3
"""ROS 2 waypoint controller using Gazebo odometry and laser scan feedback."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GazeboWaypointController(Node):
    """Drive a Gazebo differential-drive model around four waypoints."""

    def __init__(self) -> None:
        super().__init__("gazebo_waypoint_controller")
        self.declare_parameter("waypoint_tolerance", 0.10)
        self.declare_parameter("max_linear_speed", 0.35)
        self.declare_parameter("max_angular_speed", 1.4)
        self.declare_parameter("obstacle_stop_distance", 0.24)

        self.waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
        self.waypoint_index = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.min_scan = float("inf")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.control_callback)

        self.get_logger().info(
            "Waiting for Gazebo /odom and /scan; route has "
            f"{len(self.waypoints)} waypoints."
        )

    def odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.have_odom = True

    def scan_callback(self, message: LaserScan) -> None:
        valid = [
            value for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        self.min_scan = min(valid) if valid else float("inf")

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def control_callback(self) -> None:
        if not self.have_odom:
            self.stop()
            return

        if self.waypoint_index >= len(self.waypoints):
            self.stop()
            return

        target_x, target_y = self.waypoints[self.waypoint_index]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)
        tolerance = float(self.get_parameter("waypoint_tolerance").value)

        if distance <= tolerance:
            self.get_logger().info(
                f"Reached waypoint {self.waypoint_index + 1}/{len(self.waypoints)} "
                f"at ({self.x:.2f}, {self.y:.2f}) m"
            )
            self.waypoint_index += 1
            self.stop()
            return

        desired_yaw = math.atan2(dy, dx)
        heading_error = wrap_angle(desired_yaw - self.yaw)
        max_linear = float(self.get_parameter("max_linear_speed").value)
        max_angular = float(self.get_parameter("max_angular_speed").value)
        obstacle_stop = float(self.get_parameter("obstacle_stop_distance").value)

        command = Twist()
        if self.min_scan < obstacle_stop:
            command.angular.z = 0.45
            self.get_logger().warn(
                f"Laser stop condition active: minimum range {self.min_scan:.2f} m",
                throttle_duration_sec=2.0,
            )
        else:
            command.linear.x = min(max_linear, 0.8 * distance)
            command.linear.x *= max(0.0, math.cos(heading_error))
            command.angular.z = max(
                -max_angular,
                min(max_angular, 2.0 * heading_error),
            )

        self.cmd_pub.publish(command)

    def destroy_node(self) -> bool:
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GazeboWaypointController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

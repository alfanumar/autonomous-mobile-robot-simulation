#!/usr/bin/env python3
"""Launch Gazebo Sim, ROS-Gazebo bridges, RViz, and waypoint control."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("mobile_robot_gazebo")
    world = os.path.join(package_share, "worlds", "mobile_robot_world.sdf")
    bridge_config = os.path.join(package_share, "config", "bridge.yaml")
    rviz_config = os.path.join(package_share, "rviz", "mobile_robot.rviz")
    gz_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args",
                default_value=f"{world} -r",
                description="Arguments passed to Gazebo Sim. Add -s for headless mode.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    "gz_args": LaunchConfiguration("gz_args"),
                    "on_exit_shutdown": "true",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": bridge_config,
                        "expand_gz_topic_names": True,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="mobile_robot_gazebo",
                        executable="waypoint_controller.py",
                        parameters=[{"use_sim_time": True}],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                output="screen",
            ),
        ]
    )

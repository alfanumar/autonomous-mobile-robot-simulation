# ROS 2 and Gazebo Sim Validation

## Environment

- Ubuntu environment.
- ROS 2 Jazzy.
- Gazebo Sim 8.15.0.
- Package: mobile_robot_gazebo.
- Integration: ros_gz_bridge.

## Architecture

~~~text
Gazebo differential-drive model
  /odom ------------------------> waypoint_controller.py
  /scan ------------------------> waypoint_controller.py
  /cmd_vel <--------------------- waypoint_controller.py
  /clock -----------------------> ROS 2 time
~~~

## Checks

The package is built with colcon and the launch file starts Gazebo Sim, the
topic bridge, and the ROS 2 waypoint controller. Runtime checks should confirm
these topics:

- /cmd_vel: ROS 2 controller command to Gazebo.
- /odom: Gazebo differential-drive odometry bridged into ROS 2.
- /scan: simulated laser scan bridged into ROS 2.
- /tf: odometry transform bridged into ROS 2.

The verified server-only run produced these runtime observations:

~~~text
ROS2/Gazebo build: passed
ROS topics: /cmd_vel, /odom, /scan, /clock
LaserScan rate: approximately 10 Hz
Waypoint events: 4/4 reached
Waypoint positions: approximately (0.91, 0.00), (1.01, 0.90),
  (0.10, 1.01), and (-0.01, 0.10) m
~~~

This is software-in-the-loop simulation evidence. It is not a test of a
physical motor, encoder, LiDAR, or robot chassis.

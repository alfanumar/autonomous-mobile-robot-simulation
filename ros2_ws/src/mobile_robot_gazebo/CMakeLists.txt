cmake_minimum_required(VERSION 3.8)
project(mobile_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/waypoint_controller.py
  DESTINATION lib/mobile_robot_gazebo
)

install(
  DIRECTORY
    config
    launch
    rviz
    worlds
  DESTINATION share/mobile_robot_gazebo
)

ament_package()

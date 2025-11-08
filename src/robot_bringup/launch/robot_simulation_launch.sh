#!/bin/bash
gnome-terminal -- bash -c "ros2 run robot_simulator_cpp odometry_node; exec bash"
gnome-terminal -- bash -c "ros2 run robot_simulator_py controller_node; exec bash"
#gnome-terminal -- bash -c "ros2 run robot_state_publisher robot_state_publisher <temp>; exec bash"
gnome-terminal -- bash -c "ros2 run rviz2 rviz2 -d /home/john/Downloads/src/robot_bringup/rviz/basic_robot.rviz; exec bash"



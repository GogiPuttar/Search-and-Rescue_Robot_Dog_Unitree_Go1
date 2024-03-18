# ROS2 setup
ROS2 was set up on the Unitree Go1 by cloning and following the instructions for [this](https://github.com/unitreerobotics/unitree_ros2) repository. 

# ROS2 Packages

## `go1_description`
Holds all the models for visualizing the Unitree Go1 in RViz

## `ros2_unitree_legged_msgs`
Messages and services for controlling Unitree Go1

## `unitree_exploration`
Frontier Exploration with the Unitree Go1. Takes in the map, finds frontiers and commands 2-D goal poses to the Nav Stack accordingly.

## `unitree_kinematics`
Library for kinematic calculations for controlling the Go1.

## `unitree_legged_real`
Various nodes for controlling the Go1 and launchfiles for Zed 2i camera nodes.


Author: Aditya Nair

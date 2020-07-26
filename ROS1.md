# Luos with ROS 1

The `ROS 1 Bridge` allows ROS 2 packages such as `luos_ros2` to communicate with a ROS 1 ecosystem. However, both ROS 1 and 2 need to be installed and the bridge takes care of the translation. 
This procedure has been tested with ROS 1 Noetic + ROS 2 Foxy and Python 3.8.2 in Ubuntu. It might work with older distributions although it hasn't been tested.

## 1. Install ROS 2 and Luos

Make sure you have first [installed ROS 2](README.md) and managed to run the broker in ROS 2 with the command `ros2 run luos_interface broker`.
We assume your ROS 2 workspace is `~/ros2_ws`.

## 2. Install ROS 1 and the ROS 1 bridge

Then install [ROS 1 Noetic on Ubuntu 20.04](http://wiki.ros.org/noetic/Installation/Ubuntu).
We assume your ROS 1 workspace is `~/ros_ws`.

## 3. Initialize the bridge

The bridge has it own workspace:

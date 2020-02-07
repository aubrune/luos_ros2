# Luos with ROS 1 and ROS 2
## Install

First install [ROS 2 Dashing](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/) with FastRTPS and `ros1_bridge` and `colcon`.

```
cd dev_ws/src/
git clone https://github.com/aubrune/luos_ros2.git
cd luos_ros2
pip3 install .
cd .. && colcon build --symlink-install
```


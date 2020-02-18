# Luos with ROS1 and ROS2

Luos comes with a ROS1 and ROS2 interface.

## Install Luos in ROS 2

First install [ROS 2 Dashing](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/) with FastRTPS and `ros1_bridge` and `colcon`.

```
cd dev_ws/src/
git clone https://github.com/aubrune/luos_ros2.git
cd luos_ros2
pip3 install .
cd .. && colcon build --symlink-install
```

## Get started with Luos in ROS2
The Luos broker is a node that will automatically publish the detected plugged in Luos modules into ROS.
Just plug in your Luos gate and other modules, such as Imu, Color or State, and run the broker: 
```
~$ ros2 run luos_interface broker 
[INFO] [luos_broker]: Connecting to /dev/ttyUSB0...
[INFO] [luos_broker]: Found modules:
-------------------------------------------------
Type                Alias               ID   
-------------------------------------------------
Gate                gate                1    
Imu                 gps                 2    
State               lock                3    
Color               alarm               5    
```

According the modules you've plugged-in, the broker will automatically publish the revelant topics in the namespace of your modules' aliases. For isntance, here we've plugged a `State` module (alias `lock`), a `Imu` module (alias `gps`) and a `Color` module (alias `alarm`) to the gate ; thus the broker publishes the following topics:
```
~$ ros2 topic list
/alarm/variables/color/read
/alarm/variables/time/read
/gps/acceleration
/gps/compass
/gps/imu
/gps/variables/gravity_vector/read
/gps/variables/heading/read
/gps/variables/pedometer/read
/gps/variables/walk_time/read
/lock/events/changed
/lock/events/falling
/lock/events/rising
/parameter_events
/rosout
```

In order to echo messages from the terminal, use the regular ros2topic command line. For instance here's the current IMU data:
```
~$ ros2 topic echo /gps/imu 
header:
  stamp:
    sec: 1581267954
    nanosec: 5449
  frame_id: gps
orientation:
  x: 0.97814
  y: 0.052695
  z: -0.042831
  w: 0.196548
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 0.177171
  y: 0.04968
  z: 0.176722
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

## Get started with my own ROS2 package using Luos

```
cd ~/ros2_ws/src
ros2 pkg create luos_bike_alarm_example --build-type ament_python --dependencies luos_interface
```

## Package TODO List

- [ ] Clock() : timestamp messages with a sub type of clock
- [x] Hardcoded mesh filepath of bike alarm example: ament resources?
- [x] Launch files
- [ ] ROS param for publishing rate and...?
- [ ] (Auto?) Starting Luos modules data e.g. imu.orientation
- [ ] Get all the ttys
- [ ] Several imu modules with the same alias -> Publish to /gps1
- [ ] Subscribers (write access to Luos modules)
- [ ] Runtime update of pubs/subs when modules are connected and disconnected
- [ ] Windows and MacOS autoconnect
- [ ] ROS1 bridge
- [ ] Integrate doc into https://luos-robotics.github.io/ 

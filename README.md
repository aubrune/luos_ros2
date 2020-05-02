# Luos with ROS1 and ROS2

Luos comes with a ROS1 and ROS2 interface in this repository. You can get an example of an application using Luos modules in ROS2 with the [bike sharing example](https://github.com/aubrune/luos_bike_alarm_example).

## Install ROS 2 and Luos

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
Imu                 Imu_mod             2    
State               button_mod          3    
Color               rgb_led_mod         5    
```

According the modules you've plugged-in, the broker will automatically publish the revelant topics in the namespace of your modules' aliases.
Here we've plugged a `State` module (alias `button_mod`), a `Imu` module (alias `Imu_mod`) and a `Color` module (alias `rgb_led_mod`) to the gate ; thus the broker publishes the following topics:
```
~$ ros2 topic list
/Imu_mod/acceleration
/Imu_mod/compass
/Imu_mod/imu
/Imu_mod/variables/gravity_vector/read
/Imu_mod/variables/gravity_vector/write
/Imu_mod/variables/heading/read
/Imu_mod/variables/heading/write
/Imu_mod/variables/pedometer/read
/Imu_mod/variables/pedometer/write
/Imu_mod/variables/walk_time/read
/Imu_mod/variables/walk_time/write
/button_mod/events/changed
/button_mod/events/falling
/button_mod/events/rising
/button_mod/variables/state/read
/button_mod/variables/state/write
/parameter_events
/rgb_led_mod/variables/color/read
/rgb_led_mod/variables/color/write
/rgb_led_mod/variables/time/read
/rgb_led_mod/variables/time/write
/rosout
```

Most variables advertise both `/read` and `/write` topics, to get data, write data, or (de)activate this data source. Other types of data might be aggregates of Luos variables (such as the imu) or Luos events.

In order to echo messages from the terminal, use a regular ROS subscriber. For instance here's the current IMU data:
```
~$ ros2 topic echo /Imu_mod/imu
header:
  stamp:
    sec: 1581267954
    nanosec: 5449
  frame_id: Imu_mod
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

In order to publish messages to the Luos modules, use a regular ROS publisher. For instance, here's how to light up the Luos RGB module, in pink color:
```
ros2 topic pub /rgb_led_mod/variables/color/write std_msgs/msg/ColorRGBA "{r: 64, g: 0, b: 64}" --once
```

## Get started with my own ROS2 package using Luos in Python

```
cd ~/ros2_ws/src
ros2 pkg create my_luos_ros2_package --build-type ament_python --dependencies luos_interface
```

## Package TODO List

- [x] Clock() : timestamp messages with a sub type of clock
- [x] Hardcoded mesh filepath of bike alarm example: ament resources?
- [x] Launch files
- [ ] ROS param for publishing rate and...?
- [ ] (Auto?) Starting Luos modules data e.g. imu.orientation
- [ ] Get all the ttys
- [ ] Several imu modules with the same alias -> Publish to /gps1
- [x] Subscribers (write access to Luos modules)
- [ ] Runtime update of pubs/subs when modules are connected and disconnected
- [ ] Windows and MacOS autoconnect
- [ ] ROS1 bridge
- [ ] Integrate doc into https://luos-robotics.github.io/ 

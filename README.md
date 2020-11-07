 [![](http://certified.luos.io)](https://luos.io)

# Luos with ROS1 and ROS2

Luos comes with a ROS 2 interface in this repository. You can get an example of an application using Luos modules in ROS 2 with the [bike sharing example](https://github.com/aubrune/luos_bike_alarm_example). We will assume you're using ROS 2. If you want to communicate with a ROS 1 ecosystem, follow this quickstart anyway since ROS 2 needs to be installed and then refer to this [ROS 1 quickstart](./ROS1.md).

## Install ROS 2 and Luos

First install [ROS 2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/) with FastRTPS, also install [`colcon`](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon). Then clone this package and compile:

```
cd ros2_ws/src/    # Or replace by your ROS 2 workspace name, maybe also dev_ws/src
git clone https://github.com/aubrune/luos_ros2.git
cd luos_ros2/luos_interface
pip3 install --no-warn-script-location .
cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Get started with Luos in ROS2
The Luos broker is a node that will automatically publish the detected plugged in Luos modules into ROS.
Just plug in your Luos gate and other modules, such as Imu, Color or State, and run the broker: 
```
~$ ros2 launch luos_interface broker.launch.py
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

**Note:** If you have several Luos gates, you need several brokers. Specify a port and unique name for each of them:
```
ros2 launch luos_interface broker.launch.py device:=/dev/ttyUSB1 name:=brokerUSB1
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
/button_mod/events/pressed
/button_mod/events/released
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

This command lines will create a new package `my_luos_ros2_package` relying on `luos_interface`:
```
cd ~/ros2_ws/src
ros2 pkg create my_luos_ros2_package --build-type ament_python --dependencies luos_interface
```
You can then add your ROS Python scripts, by taking example on the [bike sharing example](https://github.com/aubrune/luos_bike_alarm_example).

## Roadmap

- [x] Clock() : timestamp messages with a sub type of clock
- [x] Hardcoded mesh filepath of bike alarm example: ament resources?
- [x] Launch files
- [x] ROS param for publishing rate and device name?
- [x] Subscribers (write access to Luos modules)
- [x] Get all the ttys
- [x] ROS1 bridge
- [~] Integration of all Luos types (2)
- [ ] Upgrade to the last pyluos, above 1.2.3 (modules renamed to containers) (1)
- [ ] (Auto?) Starting Luos modules data e.g. imu.orientation
- [ ] Several imu modules with the same alias -> Publish to /gps1
- [ ] Runtime update of pubs/subs when modules are connected and disconnected
- [ ] check Windows and MacOS autoconnect
- [~] Integrate doc into https://luos-robotics.github.io/ 
- [ ] ROS1-only version

**(1)** Pyluos > 1.2.3 Update > modules moved to containers > requires gate udpate > requires Luos 0.7.0 > Robus compiling issue 

### (2) Integration of all Luos types
**Done/Tested:**
3.2.1. Angle : OK / TESTED
3.2.2. Color : OK / TESTED
3.2.3. Controlled motor : OK / TESTED
3.2.6. Dynamixel motor : OK / TESTED
3.2.7. Gate : OK / TESTED
3.2.8. IMU : OK / TESTED
3.2.9. Light : OK / TESTED
3.2.10. Servo motor : OK / TESTED 
3.2.11. State : OK / TESTED

**In progress:**
3.2.4. DC motor : OK / UNTESTED value = Nan?
3.2.5. Distance : OK / UNTESTED defective module
3.2.12. Stepper motor : OK / UNTESTED https://github.com/Luos-io/Documentation/issues/86 + https://github.com/aubrune/Pyluos/commit/befabfa603a781f6381932d98ff5303332366b4c
3.2.13. Voltage : OK / UNTESTED No PlatformIO project



[![](https://img.shields.io/discourse/topics?server=https%3A%2F%2Fcommunity.luos.io&logo=Discourse)](https://community.luos.io)
[![](https://img.shields.io/badge/Luos-Documentation-34A3B4)](https://docs.luos.io)

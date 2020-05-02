import launch
import launch.actions
import launch_ros.actions

"""
This launchfile lets you automatize the connection to specific devices/ports and manage several Luos gates.
"""

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='luos_interface', node_executable='broker', output='screen',
            parameters=[
                # Replace parameter "auto" by /ttyUSB0, COM9, 192.168.0.6, my_device.local, /dev/cu.usbserial-DN2YEFLN, ...
                {"device": "auto"},
                # Set the publishing rate of Luos variables in Hertz
                {"rate": 30},
      ])
    ])
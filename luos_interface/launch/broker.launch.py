import launch
import launch.actions
import launch_ros.actions

"""
This launchfile lets you automatize the connection to specific devices/ports and manage several Luos gates.
"""

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('rate', default_value='30',
          description="Publishing rate of all Luos variables from the Luos gate the broker is connected to"),
        launch.actions.DeclareLaunchArgument('device', default_value='auto',
          description="The serial device to connect to: /ttyUSB0, COM9, 192.168.0.6, my_device.local, /dev/cu.usbserial-DN2YEFLN, ..."),
        launch.actions.DeclareLaunchArgument('name', default_value='luos_broker',
          description="Unique name of this broker in case several ports are used by several brokers"),
        launch_ros.actions.Node(
            package='luos_interface', executable='broker', output='screen',
            name=launch.substitutions.LaunchConfiguration('name'),
            parameters=[
                {"device": launch.substitutions.LaunchConfiguration('device')},
                {"rate": launch.substitutions.LaunchConfiguration('rate')},
      ])
    ])
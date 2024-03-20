from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    log_level = LaunchConfiguration("log-level")

    return LaunchDescription([
        DeclareLaunchArgument(
            'log-level',
            default_value='info'
        ),
        Node(
            package='aqlarp_motors',
            executable='aqlarp_motors',
            name='aqlarp_motors',
            arguments= [
                "--ros-args",
                "--log-level",
                ["aqlarp_motors:=", log_level],
            ]
        ),
        Node(
            package='aqlarp_sensors',
            executable='gyro',
            name='gyro',
            arguments= [
                "--ros-args",
                "--log-level",
                ["gyro:=", log_level],
            ]
        ),
        Node(
            package='aqlarp_input',
            executable='controller_input',
            name='controller_input',
            arguments= [
                "--ros-args",
                "--log-level",
                ["controller_input:=", log_level],
            ]
        ),
        Node(
            package='aqlarp_main',
            executable='aqlarp_main',
            name='aqlarp_main',
            arguments= [
                "--ros-args",
                "--log-level",
                ["aqlarp_main:=", log_level],
            ]
        )
    ])
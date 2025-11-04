from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rc_genicam_driver",
                executable="rc_genicam_driver",
                name="rc_genicam_driver",
                output="screen",
            ),
            Node(
                package="vision",
                executable="vision.py",
                name="vision",
                output="screen",
            ),
        ]
    )

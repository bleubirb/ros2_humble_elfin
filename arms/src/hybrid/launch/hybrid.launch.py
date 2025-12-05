from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hybrid",
                executable="cmd_move.py",
                name="cmd_move",
                output="screen",
            ),
            Node(
                package="hybrid",
                executable="move_solver.py",
                name="move_solver",
                output="screen",
            ),
            Node(
                package="hybrid",
                executable="hybrid.py",
                name="hybrid",
                output="screen",
            ),
        ]
    )

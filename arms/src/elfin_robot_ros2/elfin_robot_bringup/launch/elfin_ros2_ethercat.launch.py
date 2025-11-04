#!/usr/bin/python3

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None


# ========== **GENERATE LAUNCH DESCRIPTION** ========== #


def generate_launch_description():
    elfin_drivers_yaml = os.path.join(
        get_package_share_directory("elfin_robot_bringup"),
        "config",
        "elfin_drivers.yaml",
    )
    elfin_ethercat_node = Node(
        name="elfin_ethercat_driver_node",
        package="elfin_ethercat_driver",
        executable="elfin_ethercat_driver",
        output="screen",
        parameters=[elfin_drivers_yaml],
    )

    return LaunchDescription([elfin_ethercat_node])

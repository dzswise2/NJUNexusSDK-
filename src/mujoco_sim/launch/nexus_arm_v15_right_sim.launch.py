#!/usr/bin/env python3
"""
Launch file for Nexus-Arm V15 Right robot simulation.

This launch file starts MuJoCo simulation with the Nexus-Arm V15 Right robot.
All parameters are loaded from config/nexus_arm_v15_right_mujoco_sim.yaml.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Nexus-Arm V15 Right robot simulation."""

    pkg_share = get_package_share_directory('mujoco_sim')

    config_file = os.path.join(
        pkg_share,
        'config',
        'nexus_arm_v15_right_mujoco_sim.yaml'
    )

    simulator_node = Node(
        package='mujoco_sim',
        executable='simulator',
        name='nexus_arm_simulator',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        simulator_node,
    ])

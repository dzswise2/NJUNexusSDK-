#!/usr/bin/env python3
"""
Launch file for AR5 robot arm simulation with suction cup.

This launch file starts MuJoCo simulation with the AR5-5_07L-W4C4A2 robot arm
equipped with a rigid suction cup. All parameters are loaded from
config/ar5_suction_cup_mujoco_sim.yaml.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for AR5 with suction cup simulation."""

    # Get package share directory
    pkg_share = get_package_share_directory('mujoco_sim')

    # Config file path - all parameters are defined here
    ar5_suction_cup_config_file = os.path.join(
        pkg_share,
        'config',
        'ar5_suction_cup_mujoco_sim.yaml'
    )

    # MuJoCo simulator node
    simulator_node = Node(
        package='mujoco_sim',
        executable='simulator',
        name='ar5_suction_cup_simulator',
        output='screen',
        parameters=[ar5_suction_cup_config_file]
    )

    return LaunchDescription([
        simulator_node,
    ])

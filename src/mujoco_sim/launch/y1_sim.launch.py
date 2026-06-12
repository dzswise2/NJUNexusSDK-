#!/usr/bin/env python3
"""
Launch file for Y1 robot arm simulation.

This launch file starts MuJoCo simulation with the Y1 robot arm.
All parameters are loaded from config/mujoco_sim.yaml.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Y1 robot simulation."""
    
    # Get package share directory
    pkg_share = get_package_share_directory('mujoco_sim')
    
    # Config file path - all parameters are defined here
    config_file = os.path.join(
        pkg_share,
        'config',
        'mujoco_sim.yaml'
    )
    
    # MuJoCo simulator node - only load config file, no parameter overrides
    simulator_node = Node(
        package='mujoco_sim',
        executable='simulator',
        name='mujoco_simulator',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        simulator_node,
    ])

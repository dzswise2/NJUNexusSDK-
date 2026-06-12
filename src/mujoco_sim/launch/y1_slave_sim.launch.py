#!/usr/bin/env python3
"""
Launch file for dual Y1 robot arm simulation.

This launch file starts two MuJoCo simulation instances with Y1 robot arms:
- Master robot (using config/y1_master_mujoco_sim.yaml)
- Slave robot (using config/y1_slave_mujoco_sim.yaml)

All parameters are loaded from configuration files.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for dual Y1 robot simulation."""
    
    # Get package share directory
    pkg_share = get_package_share_directory('mujoco_sim')
    
    # Config file paths - all parameters are defined in these files   
    slave_config_file = os.path.join(
        pkg_share,
        'config',
        'y1_slave_mujoco_sim.yaml'
    )
    
    # Slave robot simulator node - only load config file, no parameter overrides
    slave_simulator_node = Node(
        package='mujoco_sim',
        executable='simulator',
        name='y1_slave_simulator',
        output='screen',
        parameters=[slave_config_file]
    )
    
    return LaunchDescription([
        slave_simulator_node,
    ])

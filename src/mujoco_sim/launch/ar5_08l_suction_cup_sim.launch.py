#!/usr/bin/env python3
""""""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    base_name = 'ar5_08l_suction_cup_simulator'
    node_name = f'{robot_id}_{base_name}' if robot_id else base_name

    pkg_share = get_package_share_directory('mujoco_sim')
    config_file = os.path.join(pkg_share, 'config', 'ar5_08l_suction_cup_mujoco_sim.yaml')

    params = [config_file]
    if robot_id:
        params.append({'robot_name': robot_id})

    simulator_node = Node(
        package='mujoco_sim',
        executable='simulator',
        name=node_name,
        output='screen',
        parameters=params,
    )
    return [simulator_node]


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='',
            description='Optional prefix for node name to avoid conflicts'
        ),
        OpaqueFunction(function=launch_setup),
    ])

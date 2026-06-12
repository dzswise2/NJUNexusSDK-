#!/usr/bin/env python3
"""Launch slave arm_control_node for Franka FR3v2.1 + Hand."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context).strip()
    base_name = 'slave_arm_control'
    node_name = f'{robot_id}_{base_name}' if robot_id else base_name
    log_level = LaunchConfiguration('log_level').perform(context)
    debug = LaunchConfiguration('debug').perform(context).strip()
    debug_prefix = ['gdb -ex run --args'] if debug == 'true' else []

    pkg_share = get_package_share_directory('robot_controller')
    params = [
        os.path.join(pkg_share, 'config', 'slave_single_fr3v2_1_hand_config.yaml'),
        os.path.join(pkg_share, 'config', 'arm_control_fr3v2_1_hand.yaml'),
        os.path.join(pkg_share, 'config', 'gripper_config_fr3v2_1_hand.yaml'),
    ]
    if robot_id:
        params.append({'master_robot_cfg.robot_name': robot_id})

    return [Node(
        package='robot_controller',
        executable='arm_control_node',
        name=node_name,
        output='screen',
        parameters=params,
        arguments=['--ros-args', '--log-level', log_level],
        prefix=debug_prefix,
        respawn=False,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('debug', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])

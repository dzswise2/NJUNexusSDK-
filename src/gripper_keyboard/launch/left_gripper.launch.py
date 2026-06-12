from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only left gripper keyboard (QWERT keys)."""
    return LaunchDescription([
        Node(
            package='gripper_keyboard',
            executable='qwer_keyboard_node',
            name='qwer_keyboard_node',
            output='screen',
            parameters=[{
                'robot_name': 'y1_master',
                'gripper_key_topic': 'teleop/gripper_key_state',
                'enabled_grippers': ['left_gripper'],
            }]
        )
    ])

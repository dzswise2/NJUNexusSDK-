from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch both left and right gripper keyboards.
    
    Key mappings (shared by all grippers):
        - Q: teleop_key
        - W: data_collect_key
        - E: marker_key
        - R: safety_key
    """
    return LaunchDescription([
        Node(
            package='gripper_keyboard',
            executable='qwer_keyboard_node',
            name='qwer_keyboard_node',
            output='screen',
            parameters=[{
                'robot_name': 'y1_master',
                'gripper_key_topic': 'infra/teleop_gripper_key_state',
                'enabled_grippers': ['left_gripper', 'right_gripper'],
            }]
        )
    ])

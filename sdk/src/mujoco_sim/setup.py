from setuptools import setup, find_packages
import os
from glob import glob


def get_data_files():
    """Generate data_files list for package resources."""
    data_files = []
    
    # Package index
    data_files.append((
        'share/ament_index/resource_index/packages',
        ['resource/mujoco_sim']
    ))
    
    # Package XML
    data_files.append(('share/mujoco_sim', ['package.xml']))
    
    # Launch files
    launch_files = glob('launch/*.py')
    if launch_files:
        data_files.append((
            'share/mujoco_sim/launch',
            launch_files
        ))
    
    # Config files
    config_files = glob('config/*.yaml')
    if config_files:
        data_files.append((
            'share/mujoco_sim/config',
            config_files
        ))
    
    # Robot description files (recursively)
    if os.path.exists('robot_description'):
        for root, dirs, files in os.walk('robot_description'):
            if files:
                rel_dir = os.path.relpath(root, '.')
                data_files.append((
                    os.path.join('share/mujoco_sim', rel_dir),
                    [os.path.join(root, f) for f in files]
                ))
    
    return data_files


package_name = 'mujoco_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description=(
        'MuJoCo simulation environment for robot and '
        'robotic arm simulation with ROS2 interface'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator = mujoco_sim.ros2_bridge:main',
            'joint_test = '
            'mujoco_sim.tools.joint_control_interactive:main',
        ],
    },
)

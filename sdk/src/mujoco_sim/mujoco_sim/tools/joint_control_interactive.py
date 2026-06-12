#!/usr/bin/env python3
"""
Interactive Joint Control Tool

Combined tool for sending control commands and monitoring joint states.
Provides real-time feedback of robot behavior during testing.

Usage:
    ros2 run mujoco_sim joint_control_interactive [--ros-args -p mode:=monitor]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from mujoco_sim_msgs.msg import JointMitControl
import numpy as np
import threading


class JointControlInteractive(Node):
    """
    Interactive joint control and monitoring node.
    
    Combines control command publishing with real-time state monitoring.
    """
    
    def __init__(self):
        super().__init__('joint_control_interactive')
        
        # Declare parameters
        self.declare_parameter('num_joints', 8)
        self.declare_parameter('mode', 'interactive')
        # interactive, monitor, position, torque, sweep
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('display_rate', 2.0)
        self.declare_parameter('display_mode', 'compact')
        self.declare_parameter('precision', 3)
        
        # Get parameters
        self.num_joints = self.get_parameter('num_joints').value
        self.mode = self.get_parameter('mode').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.display_rate = self.get_parameter('display_rate').value
        self.display_mode = self.get_parameter('display_mode').value
        self.precision = self.get_parameter('precision').value
        
        # Publisher
        self.cmd_publisher = self.create_publisher(
            JointMitControl, 'mujoco/joint_commands', 10
        )
        
        # Subscriber
        self.state_subscriber = self.create_subscription(
            JointState,
            'mujoco/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Store latest state
        self.latest_state = None
        self.state_lock = threading.Lock()
        
        # Statistics
        self.msg_count = 0
        self.first_msg_time = None
        
        # Test state for sweep mode
        self.sweep_counter = 0
        
        # Timers
        if self.mode in ['position', 'torque', 'sweep', 'interactive']:
            self.cmd_timer = self.create_timer(
                1.0 / self.publish_rate, self.publish_command
            )
        
        self.display_timer = self.create_timer(
            1.0 / self.display_rate, self.display_state
        )
        
        self.get_logger().info('Joint Control Interactive Tool started')
        self.get_logger().info(f'  Mode: {self.mode}')
        self.get_logger().info(f'  Number of joints: {self.num_joints}')
        self.get_logger().info(
            f'  Command rate: {self.publish_rate} Hz'
        )
        self.get_logger().info(
            f'  Display rate: {self.display_rate} Hz'
        )
        
        # Interactive mode instruction
        if self.mode == 'interactive':
            self.get_logger().info('')
            self.get_logger().info('Interactive mode commands:')
            self.get_logger().info('  Press Ctrl+C to exit')
            self.get_logger().info(
                '  Modify parameters with --ros-args to change behavior'
            )
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state."""
        with self.state_lock:
            self.latest_state = msg
            self.msg_count += 1
            
            if self.first_msg_time is None:
                self.first_msg_time = self.get_clock().now()
    
    def publish_command(self):
        """Publish control command based on mode."""
        msg = JointMitControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        if self.mode == 'position' or self.mode == 'interactive':
            # Position control with PD gains
            msg.kp = [30.0] * self.num_joints
            msg.joint_position = (
                [0.5, 0.5, 0.5] + [0.0] * (self.num_joints - 3)
            )
            msg.kd = [1.0] * self.num_joints
            msg.joint_velocity = [0.0] * self.num_joints
            msg.torque = [0.0] * self.num_joints
            
        elif self.mode == 'torque':
            # Pure torque control
            msg.kp = [0.0] * self.num_joints
            msg.joint_position = [0.0] * self.num_joints
            msg.kd = [0.0] * self.num_joints
            msg.joint_velocity = [0.0] * self.num_joints
            msg.torque = [1.0] * self.num_joints
            
        elif self.mode == 'sweep':
            # Sweep through positions
            import math
            phase = self.sweep_counter * 2.0 * math.pi / 20.0
            amplitude = 0.5
            
            msg.kp = [30.0] * self.num_joints
            msg.joint_position = [
                amplitude * math.sin(phase),
                amplitude * math.sin(phase + math.pi/4),
                amplitude * math.sin(phase + math.pi/2)
            ] + [0.0] * (self.num_joints - 3)
            msg.kd = [1.0] * self.num_joints
            msg.joint_velocity = [0.0] * self.num_joints
            msg.torque = [0.0] * self.num_joints
            
            self.sweep_counter += 1
        
        self.cmd_publisher.publish(msg)
    
    def display_state(self):
        """Display joint state based on mode."""
        with self.state_lock:
            state = self.latest_state
        
        if state is None:
            if self.msg_count == 0:
                print('\rWaiting for joint states...', end='', flush=True)
            return
        
        if self.display_mode == 'compact':
            self.display_compact(state)
        elif self.display_mode == 'detailed':
            self.display_detailed(state)
        elif self.display_mode == 'stats':
            self.display_statistics(state)
    
    def display_compact(self, state: JointState):
        """Display in compact format."""
        print('\n' + '='*80)
        print(
            f'Joint States (msg #{self.msg_count}) | '
            f'Mode: {self.mode}'
        )
        print('-'*80)
        
        # Position
        if state.position:
            pos_str = ' '.join(
                f'{p:>{self.precision + 4}.{self.precision}f}'
                for p in state.position
            )
            print(f'Position: [{pos_str}]')
        
        # Velocity
        if state.velocity:
            vel_str = ' '.join(
                f'{v:>{self.precision + 4}.{self.precision}f}'
                for v in state.velocity
            )
            print(f'Velocity: [{vel_str}]')
        
        # Effort
        if state.effort:
            eff_str = ' '.join(
                f'{e:>{self.precision + 4}.{self.precision}f}'
                for e in state.effort
            )
            print(f'Effort:   [{eff_str}]')
        
        print('='*80)
    
    def display_detailed(self, state: JointState):
        """Display in detailed format."""
        print('\n' + '='*80)
        print(
            f'Joint States (msg #{self.msg_count}) | '
            f'Mode: {self.mode}'
        )
        print('-'*80)
        print(
            f"{'Joint':<15} {'Position':>12} "
            f"{'Velocity':>12} {'Effort':>12}"
        )
        print('-'*80)
        
        for i, name in enumerate(state.name):
            pos = state.position[i] if i < len(state.position) else 0.0
            vel = state.velocity[i] if i < len(state.velocity) else 0.0
            eff = state.effort[i] if i < len(state.effort) else 0.0
            
            print(f"{name:<15} "
                  f"{pos:>12.{self.precision}f} "
                  f"{vel:>12.{self.precision}f} "
                  f"{eff:>12.{self.precision}f}")
        
        print('='*80)
    
    def display_statistics(self, state: JointState):
        """Display statistics."""
        print('\n' + '='*80)
        print(
            f'Joint State Statistics (msg #{self.msg_count}) | '
            f'Mode: {self.mode}'
        )
        print('-'*80)
        
        if state.position:
            pos_array = np.array(state.position)
            print('Position:')
            print(f'  Min:  {np.min(pos_array):.{self.precision}f}')
            print(f'  Max:  {np.max(pos_array):.{self.precision}f}')
            print(f'  Mean: {np.mean(pos_array):.{self.precision}f}')
            print(f'  Std:  {np.std(pos_array):.{self.precision}f}')
        
        if state.velocity:
            vel_array = np.array(state.velocity)
            print('Velocity:')
            print(f'  Min:  {np.min(vel_array):.{self.precision}f}')
            print(f'  Max:  {np.max(vel_array):.{self.precision}f}')
            print(f'  Mean: {np.mean(vel_array):.{self.precision}f}')
            print(f'  Std:  {np.std(vel_array):.{self.precision}f}')
        
        if state.effort:
            eff_array = np.array(state.effort)
            print('Effort:')
            print(f'  Min:  {np.min(eff_array):.{self.precision}f}')
            print(f'  Max:  {np.max(eff_array):.{self.precision}f}')
            print(f'  Mean: {np.mean(eff_array):.{self.precision}f}')
            print(f'  Std:  {np.std(eff_array):.{self.precision}f}')
        
        # Message rate
        if self.first_msg_time is not None:
            elapsed = (
                self.get_clock().now() - self.first_msg_time
            ).nanoseconds / 1e9
            if elapsed > 0:
                rate = self.msg_count / elapsed
                print(f'\nMessage rate: {rate:.2f} Hz')
        
        print('='*80)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = JointControlInteractive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\nShutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Generic MuJoCo Simulator Node

A ROS2 node that runs MuJoCo simulation for any robot model.
Loads robot description and scene from launch parameters.
Publishes joint states and subscribes to joint commands.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from infra_msg.msg import JointMitControl, TeleOpArmStatus, MotorStatus, EndPoseData
import numpy as np
import math
import mujoco
import mujoco.viewer
import os
import threading
from abc import ABC, abstractmethod
from typing import Optional
from ament_index_python.packages import get_package_share_directory


# =============================================================================
# Gripper Mapper Abstract Base Class and Implementations
# =============================================================================

class GripperMapperBase(ABC):
    """
    Abstract base class for gripper joint mapping.

    Different robots may have different gripper mechanisms:
    - Some use prismatic joints in simulation but revolute in real robot
    - Some use the same joint type in both
    - Some may need custom conversion logic

    Subclasses must implement:
    - stroke_to_angle / angle_to_stroke  (low-level, named by value type)

    The framework calls sim_to_real_pos / real_to_sim_pos (direction-aware).
    Default implementations map to stroke_to_angle / angle_to_stroke assuming
    sim=stroke, real=angle (Y1 convention).  Override for robots where
    sim=angle, real=stroke (e.g., AR5, Nexus).
    """

    # =========================================================================
    # Position conversion (abstract - must be implemented by subclass)
    # =========================================================================

    @abstractmethod
    def stroke_to_angle(self, stroke: float) -> float:
        """Convert a stroke value (distance) to an angle value (radians)."""
        pass

    @abstractmethod
    def angle_to_stroke(self, angle: float) -> float:
        """Convert an angle value (radians) to a stroke value (distance)."""
        pass

    # =========================================================================
    # Direction-aware position conversion (framework-facing)
    # =========================================================================

    def sim_to_real_pos(self, sim_pos: float) -> float:
        """
        Convert position: simulation -> real robot (state publishing).

        Default: sim=stroke, real=angle (Y1 convention).
        Override if sim=angle, real=stroke (AR5/Nexus).
        """
        return self.stroke_to_angle(sim_pos)

    def real_to_sim_pos(self, real_pos: float) -> float:
        """
        Convert position: real robot -> simulation (control command).

        Default: real=angle, sim=stroke (Y1 convention).
        Override if real=stroke, sim=angle (AR5/Nexus).
        """
        return self.angle_to_stroke(real_pos)
    
    # =========================================================================
    # Velocity conversion (default: identity, override for custom behavior)
    # =========================================================================
    
    def velocity_sim_to_real(self, velocity: float) -> float:
        """
        Convert velocity: simulation -> real robot.
        
        Used when publishing joint states.
        Default implementation returns input unchanged.
        Override for robot-specific conversion.
        
        Args:
            velocity: Velocity value from simulation
            
        Returns:
            Converted velocity value for real robot
        """
        return velocity
    
    def velocity_real_to_sim(self, velocity: float) -> float:
        """
        Convert velocity: real robot -> simulation.
        
        Used when computing control commands.
        Default implementation returns input unchanged.
        Override for robot-specific conversion.
        
        Args:
            velocity: Velocity value from real robot command
            
        Returns:
            Converted velocity value for simulation
        """
        return velocity
    
    # =========================================================================
    # Effort/Torque conversion (default: identity, override for custom behavior)
    # =========================================================================
    
    def effort_sim_to_real(self, effort: float) -> float:
        """
        Convert effort/torque: simulation -> real robot.
        
        Used when publishing joint states.
        Default implementation returns input unchanged.
        Override for robot-specific conversion.
        
        Args:
            effort: Effort/torque value from simulation
            
        Returns:
            Converted effort/torque value for real robot
        """
        return effort
    
    def effort_real_to_sim(self, effort: float) -> float:
        """
        Convert effort/torque: real robot -> simulation.
        
        Used when computing control commands (torque feedforward).
        Default implementation returns input unchanged.
        Override for robot-specific conversion.
        
        Args:
            effort: Effort/torque value from real robot command
            
        Returns:
            Converted effort/torque value for simulation
        """
        return effort


class IdentityGripperMapper(GripperMapperBase):
    """
    Identity mapper that performs no conversion.
    
    Use this for robots where the simulation and real robot
    use the same joint type and units for the gripper.
    
    All conversion methods return input unchanged (identity mapping).
    """
    
    def stroke_to_angle(self, stroke: float) -> float:
        """Return input unchanged."""
        return stroke
    
    def angle_to_stroke(self, angle: float) -> float:
        """Return input unchanged."""
        return angle
    
    # Velocity and effort methods use base class defaults (identity mapping)


class NexusArmGripperMapper(GripperMapperBase):
    """
    Gripper mapper for Nexus-Arm robot.
    
    Position mapping:
    - Stroke range: (0, 0.03) meters (simulation prismatic joint)
    - Angle range: (0, 1.467) radians (real robot)
    - Linear relationship between stroke and angle
    
    Velocity: Set to 0 (no conversion available)
    Effort/Torque: Direct passthrough (no conversion needed)
    """
    
    # Stroke range in simulation (meters)
    STROKE_MIN = 0.0
    STROKE_MAX = 0.03
    
    # Angle range for real robot (radians)
    ANGLE_MIN = 0.0
    ANGLE_MAX = 1.467
    
    # =========================================================================
    # Position conversion (linear mapping)
    # =========================================================================
    
    def stroke_to_angle(self, stroke: float) -> float:
        """
        Convert stroke (simulation) to angle (real robot).
        Linear mapping: stroke (0, 0.03) -> angle (0, 1.467)
        """
        # Clamp stroke to valid range
        stroke = max(self.STROKE_MIN, min(self.STROKE_MAX, stroke))
        
        # Linear interpolation
        if self.STROKE_MAX == self.STROKE_MIN:
            return self.ANGLE_MIN
        
        ratio = (stroke - self.STROKE_MIN) / (self.STROKE_MAX - self.STROKE_MIN)
        angle = self.ANGLE_MIN + ratio * (self.ANGLE_MAX - self.ANGLE_MIN)
        return angle
    
    def angle_to_stroke(self, angle: float) -> float:
        """
        Convert angle (real robot) to stroke (simulation).
        Linear mapping: angle (0, 1.467) -> stroke (0, 0.03)
        """
        # Clamp angle to valid range
        angle = max(self.ANGLE_MIN, min(self.ANGLE_MAX, angle))
        
        # Linear interpolation
        if self.ANGLE_MAX == self.ANGLE_MIN:
            return self.STROKE_MIN
        
        ratio = (angle - self.ANGLE_MIN) / (self.ANGLE_MAX - self.ANGLE_MIN)
        stroke = self.STROKE_MIN + ratio * (self.STROKE_MAX - self.STROKE_MIN)
        return stroke
    
    # =========================================================================
    # Velocity conversion (set to 0)
    # =========================================================================
    
    def velocity_sim_to_real(self, velocity: float) -> float:
        """Nexus-Arm gripper: No velocity conversion, return 0."""
        return 0.0
    
    def velocity_real_to_sim(self, velocity: float) -> float:
        """Nexus-Arm gripper: No velocity conversion, return 0."""
        return 0.0
    
    # =========================================================================
    # Effort/Torque conversion (direct passthrough)
    # Use base class defaults - no override needed
    # =========================================================================


class Y1GripperMapper(GripperMapperBase):
    """
    Gripper mapper for Y1 robot.
    
    The Y1 real robot has a revolute joint7 (gripper),
    but the simulation uses a prismatic joint. This class provides
    bidirectional mapping using a lookup table.
    
    Note: Velocity and torque/force conversions are not available,
    so they are set to 0.
    """
    
    # Hardcoded stroke-to-angle mapping table for Y1 robot
    # Format: (stroke_in_meters, angle_in_radians)
    MAPPING_TABLE = [
        (-0.0, 0.002), (-0.0005, 0.01407), (-0.001, 0.0368934),
        (-0.0015, 0.08634), (-0.002, 0.115854), (-0.0025, 0.161084),
        (-0.003, 0.176609), (-0.0035, 0.194816), (-0.004, 0.22893),
        (-0.0045, 0.262277), (-0.005, 0.295), (-0.0055, 0.314215),
        (-0.006, 0.344305), (-0.0065, 0.362704), (-0.007, 0.393177),
        (-0.0075, 0.411575), (-0.008, 0.441473), (-0.0085, 0.46773),
        (-0.009, 0.482871), (-0.0095, 0.508743), (-0.01, 0.527718),
        (-0.0105, 0.550333), (-0.011, 0.576397), (-0.0115, 0.60227),
        (-0.012, 0.621053), (-0.0125, 0.639452), (-0.013, 0.662833),
        (-0.0135, 0.683915), (-0.014, 0.707105), (-0.0145, 0.722054),
        (-0.015, 0.748693), (-0.0155, 0.763451), (-0.016, 0.782233),
        (-0.0165, 0.805039), (-0.017, 0.815964), (-0.0175, 0.841837),
        (-0.018, 0.856786), (-0.0185, 0.879593), (-0.019, 0.894158),
        (-0.0195, 0.91639), (-0.02, 0.92789), (-0.0205, 0.94648),
        (-0.021, 0.969287), (-0.0215, 0.987494), (-0.022, 0.998418),
        (-0.0225, 1.02084), (-0.023, 1.03617), (-0.0235, 1.05112),
        (-0.024, 1.0701), (-0.0245, 1.08505), (-0.025, 1.10383),
        (-0.0255, 1.11897), (-0.026, 1.13411), (-0.0265, 1.15557),
        (-0.027, 1.16382), (-0.0275, 1.18221), (-0.028, 1.20117),
        (-0.0285, 1.21594), (-0.029, 1.23779), (-0.0295, 1.24986),
        (-0.03, 1.26424), (-0.0305, 1.28264), (-0.031, 1.29452),
        (-0.0315, 1.31273), (-0.032, 1.3317), (-0.0325, 1.34359),
        (-0.033, 1.36505), (-0.0335, 1.38019), (-0.034, 1.39169),
        (-0.0345, 1.40683), (-0.035, 1.42542), (-0.0355, 1.44401),
        (-0.036, 1.45609), (-0.0365, 1.47468), (-0.037, 1.48943),
        (-0.0375, 1.50419), (-0.038, 1.52259), (-0.0385, 1.53735),
        (-0.039, 1.55575), (-0.0395, 1.57453), (-0.04, 1.582),
        (-0.0405, 1.60462), (-0.041, 1.62302), (-0.0415, 1.64218),
        (-0.042, 1.65694), (-0.0425, 1.66825), (-0.043, 1.68722),
        (-0.0435, 1.70984), (-0.044, 1.729), (-0.0445, 1.73973),
        (-0.045, 1.76216), (-0.0455, 1.78477), (-0.046, 1.79991),
        (-0.0465, 1.82214), (-0.047, 1.84399), (-0.0475, 1.86297),
        (-0.048, 1.88577), (-0.0485, 1.9036), (-0.049, 1.92717),
        (-0.0495, 1.94844), (-0.05, 1.97527)
    ]
    
    def __init__(self):
        """Initialize gripper mapper with hardcoded mapping table."""
        self.stroke_to_angle_data = list(self.MAPPING_TABLE)
    
    # =========================================================================
    # Velocity conversion for Y1 gripper
    # No valid conversion between prismatic (sim) and revolute (real) joints
    # =========================================================================
    
    def velocity_sim_to_real(self, velocity: float) -> float:
        """
        Y1 gripper: No velocity conversion available.
        Return 0 since prismatic and revolute velocities are incompatible.
        """
        return 0.0
    
    def velocity_real_to_sim(self, velocity: float) -> float:
        """
        Y1 gripper: No velocity conversion available.
        Return 0 since prismatic and revolute velocities are incompatible.
        """
        return 0.0
    
    # =========================================================================
    # Effort/Torque conversion for Y1 gripper
    # =========================================================================
    
    def effort_sim_to_real(self, effort: float) -> float:
        """
        Y1 gripper: No effort conversion available.
        Return 0 since force (prismatic) and torque (revolute) are incompatible.
        """
        return 0.0
    
    def effort_real_to_sim(self, effort: float) -> float:
        """
        Y1 gripper: Invert torque direction.
        The gripper joint direction is inverted between simulation and real robot.
        """
        return -effort
    
    # =========================================================================
    # Position conversion for Y1 gripper (stroke <-> angle mapping)
    # =========================================================================
    
    def stroke_to_angle(self, stroke: float) -> float:
        """
        Convert stroke (prismatic position) to angle (revolute position).
        
        Args:
            stroke: Stroke value in meters (in simulation, 0 to -0.05)
            
        Returns:
            Angle value in radians
        """
        if len(self.stroke_to_angle_data) == 0:
            return 0.0
        
        # Mirror the stroke: simulation 0 (closed) to -0.05 (open)
        # maps to table -0.05 (closed/large angle) to 0 (open/small angle)
        # Formula: inverted = -0.05 - stroke
        # When sim_stroke=0 (closed): inverted=-0.05 -> angle=1.97527
        # When sim_stroke=-0.05 (open): inverted=0 -> angle=0.002
        
        # Clamp to valid range [-0.05, 0]
        inverted_stroke = max(-0.05, min(0.0, stroke))
        
        # Linear interpolation
        if inverted_stroke >= self.stroke_to_angle_data[0][0]:
            return self.stroke_to_angle_data[0][1]
        if inverted_stroke <= self.stroke_to_angle_data[-1][0]:
            return self.stroke_to_angle_data[-1][1]
        
        # Find interpolation range
        for i in range(len(self.stroke_to_angle_data) - 1):
            s1, a1 = self.stroke_to_angle_data[i]
            s2, a2 = self.stroke_to_angle_data[i + 1]
            
            if s2 <= inverted_stroke <= s1:
                # Linear interpolation
                t = (inverted_stroke - s2) / (s1 - s2) if s2 != s1 else 0
                return a2 + t * (a1 - a2)
        
        return 0.0
    
    def angle_to_stroke(self, angle: float) -> float:
        """
        Convert angle (revolute position) to stroke (prismatic position).
        
        Args:
            angle: Angle value in radians
            
        Returns:
            Stroke value in meters (in simulation, 0 to -0.05)
        """
        if len(self.stroke_to_angle_data) == 0:
            return 0.0
        
        # Clamp to valid angle range
        angle = max(self.stroke_to_angle_data[0][1], 
                   min(self.stroke_to_angle_data[-1][1], angle))
        
        # Linear interpolation (inverse mapping)
        if angle >= self.stroke_to_angle_data[-1][1]:
            return self.stroke_to_angle_data[-1][0]
        if angle <= self.stroke_to_angle_data[0][1]:
            return self.stroke_to_angle_data[0][0]
        
        # Find interpolation range
        for i in range(len(self.stroke_to_angle_data) - 1):
            s1, a1 = self.stroke_to_angle_data[i]
            s2, a2 = self.stroke_to_angle_data[i + 1]
            
            if a1 <= angle <= a2:
                # Linear interpolation
                t = (angle - a1) / (a2 - a1) if a2 != a1 else 0
                return s1 + t * (s2 - s1)
        
        return 0.0


# =============================================================================
# Gripper Mapper Factory
# =============================================================================

class AR5GripperMapper(GripperMapperBase):
    """
    Gripper mapper for AR5 robot with parallel jaw 4-bar linkage gripper.

    Kinematics (closed state, all joints at 0):
      - Bottom base (joint_1 ↔ joint_3): 32mm
      - Top base (joint_2 ↔ joint_4): 22.72mm
      - Leg length: 55mm
      - theta_0 = arcsin((32 - 22.72) / (2 * 55))  (linkage bias angle at closed)

    Half opening distance (mm) = 55 * sin(theta_joint1 - theta_0) + 22.72/2
    Total opening stroke = 2 * (half(theta) - half(0))  [meters]
    """

    # 4-bar linkage geometry constants (meters)
    _LEG = 0.055        # linkage length
    _TOP_HALF = 0.01136  # top_base / 2 (22.72mm / 2)
    _BOTTOM_HALF = 0.016  # bottom_base / 2 (32mm / 2)
    _THETA_0 = math.asin((0.032 - 0.02272) / (2 * 0.055))  # ≈ 0.084464 rad

    def __init__(self):
        pass

    def velocity_sim_to_real(self, velocity: float) -> float:
        return 0.0

    def velocity_real_to_sim(self, velocity: float) -> float:
        return 0.0

    def effort_sim_to_real(self, effort: float) -> float:
        return effort

    def effort_real_to_sim(self, effort: float) -> float:
        return effort

    def stroke_to_angle(self, stroke: float) -> float:
        """Low-level: stroke(m) → angle(rad)."""
        x = stroke / (2 * self._LEG) - math.sin(self._THETA_0)
        x = max(-1.0, min(1.0, x))
        return math.asin(x) + self._THETA_0

    def angle_to_stroke(self, angle: float) -> float:
        """Low-level: angle(rad) → stroke(m)."""
        half = self._LEG * (math.sin(angle - self._THETA_0) + math.sin(self._THETA_0))
        return 2.0 * half

    # AR5: sim=angle, real=stroke (opposite of Y1 convention)
    def sim_to_real_pos(self, sim_pos: float) -> float:
        """sim(angle) → real(stroke): publish joint states."""
        return self.angle_to_stroke(sim_pos)

    def real_to_sim_pos(self, real_pos: float) -> float:
        """real(stroke) → sim(angle): apply control command."""
        return self.stroke_to_angle(real_pos)


class GripperMapperFactory:
    """
    Factory class for creating robot-specific gripper mappers.
    
    Register new robot types by adding them to the ROBOT_MAPPERS dictionary.
    """
    
    # Registry of robot types to their gripper mapper classes
    ROBOT_MAPPERS = {
        # Y1 robot family
        'y1': Y1GripperMapper,
        'y1_master': Y1GripperMapper,
        'y1_slave': Y1GripperMapper,
        # Nexus-Arm robot
        'nexus-arm': NexusArmGripperMapper,
        'nexus_arm': NexusArmGripperMapper,
        'nexus_arm_v15': NexusArmGripperMapper,
        'nexus_arm_v15_right': IdentityGripperMapper,
        # Piper robot (uses identity mapping)
        'piper': IdentityGripperMapper,
        # AR5 robot
        'ar5_gripper': AR5GripperMapper,
        'ar5_suction_cup': IdentityGripperMapper,
        'franka_hand': IdentityGripperMapper,
        # Default/generic robots
        'generic': IdentityGripperMapper,
        'default': IdentityGripperMapper,
    }
    
    @classmethod
    def create(cls, robot_type: str) -> Optional[GripperMapperBase]:
        """
        Create a gripper mapper for the specified robot type.
        
        Args:
            robot_type: Robot type identifier (e.g., 'y1', 'nexus-arm')
            
        Returns:
            GripperMapperBase instance, or None if robot_type is empty/None
        """
        if not robot_type:
            return None
        
        # Normalize robot type (lowercase, replace hyphens with underscores)
        normalized_type = robot_type.lower().replace('-', '_')
        
        # Look up in registry
        mapper_class = cls.ROBOT_MAPPERS.get(normalized_type)
        
        if mapper_class is None:
            # Default to IdentityGripperMapper for unknown robot types
            return IdentityGripperMapper()
        
        return mapper_class()
    
    @classmethod
    def register(cls, robot_type: str, mapper_class: type):
        """
        Register a new robot type with its gripper mapper class.
        
        Args:
            robot_type: Robot type identifier
            mapper_class: GripperMapperBase subclass
        """
        cls.ROBOT_MAPPERS[robot_type.lower()] = mapper_class
    
    @classmethod
    def get_supported_types(cls) -> list:
        """Get list of supported robot types."""
        return list(cls.ROBOT_MAPPERS.keys())


class MuJoCoSimulatorNode(Node):
    """
    Generic MuJoCo simulator ROS2 node.
    
    Parameters:
        - model_path: Path to robot URDF/XML file
        - publish_rate: Rate for publishing joint states (Hz)
        - enable_viewer: Enable MuJoCo viewer window (default: True)
        - robot_type: Robot type for gripper mapping (e.g., 'y1', 'nexus-arm')
        - enable_end_pose: Enable end effector pose publishing (default: True)
        - joint_effort_source: What fills JointState.effort / motor torque feedback:
            - actuator_force (default): per-actuator output (MIT PD + ff); NOT equal to
              rigid-body ID torque τ=Mq̈+b in general.
            - mj_inverse: MuJoCo inverse dynamics from current (q,q̇,q̈) → data.qfrc_inverse,
              mapped to the same joint indices as joint_state; use to validate τ_meas≈τ_model
              when no external force and Pinocchio model matches MJCF.
    """
    
    def __init__(self):
        """Initialize MuJoCo simulator node."""
        super().__init__('mujoco_simulator')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('publish_rate', 500.0)
        self.declare_parameter('enable_viewer', True)
        self.declare_parameter('arm_status_rate', 10.0)
        self.declare_parameter('robot_name', 'mujoco')
        self.declare_parameter(
            'joint_state_topic', 'teleop/table_arm/joint_state'
        )
        self.declare_parameter('status_topic', 'teleop/table_arm/status')
        self.declare_parameter(
            'joint_mit_control_topic', 'teleop/table_arm/joint_cmd'
        )
        self.declare_parameter('end_pose_topic', 'teleop/table_arm/end_pose')
        # joint7 is at index 6 (0-based)
        self.declare_parameter('gripper_joint_index', 6)
        self.declare_parameter('enable_gripper_mapping', True)
        self.declare_parameter('ee_site_name', 'ee_link6')
        # New parameters for decoupled robot support
        self.declare_parameter('robot_type', '')  # Robot type for gripper mapping
        self.declare_parameter('enable_end_pose', True)  # Enable end pose publishing
        # See class docstring: actuator_force vs rigid-body ID torque for dynamics validation
        self.declare_parameter('joint_effort_source', 'actuator_force')
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        publish_rate = self.get_parameter('publish_rate').value
        enable_viewer = self.get_parameter('enable_viewer').value
        arm_status_rate = self.get_parameter('arm_status_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        topic_joint_states = self.get_parameter('joint_state_topic').value
        topic_arm_status = self.get_parameter('status_topic').value
        topic_joint_commands = self.get_parameter(
            'joint_mit_control_topic'
        ).value
        topic_end_pose = self.get_parameter('end_pose_topic').value
        gripper_idx = self.get_parameter('gripper_joint_index').value
        self.gripper_joint_index = gripper_idx
        enable_mapping = self.get_parameter('enable_gripper_mapping').value
        self.enable_gripper_mapping = enable_mapping
        self.ee_site_name = self.get_parameter('ee_site_name').value
        self.robot_type = self.get_parameter('robot_type').value
        self.enable_end_pose = self.get_parameter('enable_end_pose').value
        _eff_src = (
            self.get_parameter('joint_effort_source').value or 'actuator_force'
        )
        _eff_src = str(_eff_src).strip().lower().replace('-', '_')
        if _eff_src in ('mj_inverse', 'inverse_dynamics', 'qfrc_inverse', 'id'):
            self.joint_effort_source = 'mj_inverse'
        elif _eff_src in ('actuator_force', 'actuator', 'default'):
            self.joint_effort_source = 'actuator_force'
        else:
            self.get_logger().warn(
                f'Unknown joint_effort_source "{_eff_src}", using actuator_force. '
                'Valid: actuator_force, mj_inverse.'
            )
            self.joint_effort_source = 'actuator_force'
        
        # Track if ee_site was found (checked after model load)
        self.ee_site_available = False
        self.ee_site_warning_logged = False
        
        # Initialize gripper mapper using factory pattern
        self.gripper_mapper = None
        if self.enable_gripper_mapping:
            try:
                # Use robot_type to get appropriate mapper
                # Fall back to robot_name if robot_type not specified
                mapper_type = self.robot_type if self.robot_type else self.robot_name
                self.gripper_mapper = GripperMapperFactory.create(mapper_type)
                
                if self.gripper_mapper is not None:
                    mapper_name = type(self.gripper_mapper).__name__
                    log_msg = (
                        f'Gripper mapping enabled for robot type "{mapper_type}" '
                        f'using {mapper_name} at joint index {self.gripper_joint_index}'
                    )
                    self.get_logger().info(log_msg)
                else:
                    self.get_logger().info(
                        f'No gripper mapping configured for robot type "{mapper_type}"'
                    )
                    self.enable_gripper_mapping = False
            except Exception as e:
                err_msg = f'Failed to initialize gripper mapper: {e}'
                self.get_logger().error(err_msg)
                self.enable_gripper_mapping = False
        
        # Add robot_name prefix to topics
        if self.robot_name:
            topic_joint_states = f'/{self.robot_name}{topic_joint_states}'
            topic_arm_status = f'/{self.robot_name}{topic_arm_status}'
            topic_joint_commands = f'/{self.robot_name}{topic_joint_commands}'
            topic_end_pose = f'/{self.robot_name}{topic_end_pose}'
        
        # Initialize MuJoCo simulation
        self.model = None
        self.data = None
        self.viewer = None
        
        # Thread lock for data access
        self.motor_state_lock = threading.Lock()
        self.motor_state = {
            'pos': None,
            'vel': None,
            'frc': None
        }
        
        # Load model
        if not model_path:
            self.get_logger().error('model_path parameter is required!')
            raise ValueError('model_path parameter not set')
        
        # Resolve model path - support both absolute and relative paths
        resolved_model_path = self._resolve_model_path(model_path)
        self.load_model(resolved_model_path)
        if self.joint_effort_source == 'mj_inverse':
            self.get_logger().info(
                'joint_effort_source=mj_inverse: JointState.effort uses MuJoCo '
                'mj_inverse → qfrc_inverse (rigid-body ID torque for current '
                'sim q,q̇,q̈). Use for τ_meas≈τ_model validation; default '
                'actuator_force remains MIT PD + feedforward.'
            )
        
        # Launch viewer if enabled
        if enable_viewer:
            self.launch_viewer()
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, topic_joint_states, 10
        )
        self.arm_status_pub = self.create_publisher(
            TeleOpArmStatus, topic_arm_status, 10
        )
        self.end_pose_pub = self.create_publisher(
            EndPoseData, topic_end_pose, 10
        )
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointMitControl,
            topic_joint_commands,
            self.joint_command_callback,
            10
        )
        
        # Store latest control command (protected by lock)
        self.control_cmd_lock = threading.Lock()
        self.latest_cmd = None
        
        # Timer for simulation and publishing
        self.timer = self.create_timer(
            1.0 / publish_rate, self.simulation_step
        )
        
        # Timer for arm status publishing
        self.arm_status_timer = self.create_timer(
            1.0 / arm_status_rate, self.publish_arm_status
        )
        
        # Statistics tracking
        self.total_packets = 0
        self.packet_loss_count = 0
        
        self.get_logger().info('MuJoCo Simulator Node initialized')
        self.get_logger().info(f'Loaded model: {model_path}')
        self.get_logger().info(
            f'Robot name: {self.robot_name}, Robot type: {self.robot_type or "auto"}'
        )
        self.get_logger().info(
            f'Publishing joint states to: {topic_joint_states}'
        )
        self.get_logger().info(
            f'Publishing arm status to: {topic_arm_status}'
        )
        if self.enable_end_pose:
            self.get_logger().info(
                f'Publishing end pose to: {topic_end_pose}'
            )
        else:
            self.get_logger().info('End pose publishing disabled')
        self.get_logger().info(
            f'Subscribing to joint commands from: '
            f'{topic_joint_commands}'
        )
        if enable_viewer:
            self.get_logger().info('Viewer enabled')
    
    def _resolve_model_path(self, model_path: str) -> str:
        """
        Resolve model path to absolute path.
        
        Supports:
        - Absolute paths (returned as-is)
        - Relative paths (resolved relative to package share directory)
        
        Args:
            model_path: Path from configuration (absolute or relative)
            
        Returns:
            Absolute path to the model file
        """
        # If already absolute, return as-is
        if os.path.isabs(model_path):
            return model_path
        
        # Resolve relative to package share directory
        try:
            pkg_share = get_package_share_directory('mujoco_sim')
            resolved_path = os.path.join(pkg_share, model_path)
            self.get_logger().info(
                f'Resolved model path: {model_path} -> {resolved_path}'
            )
            return resolved_path
        except Exception as e:
            self.get_logger().warn(
                f'Failed to resolve package share directory: {e}. '
                f'Using model_path as-is: {model_path}'
            )
            return model_path
    
    def launch_viewer(self):
        """Launch MuJoCo passive viewer in separate thread."""
        try:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data
            )
            self.get_logger().info('MuJoCo viewer launched')
        except Exception as e:
            self.get_logger().warn(
                f'Failed to launch viewer: {e}. '
                'Continuing without visualization.'
            )

    def close_viewer(self):
        """Close passive viewer before ROS teardown to reduce GLX/X11 errors on exit."""
        v = self.viewer
        self.viewer = None
        if v is None:
            return
        try:
            closer = getattr(v, 'close', None)
            if callable(closer):
                closer()
        except Exception:
            pass
    
    def load_model(self, model_path: str):
        """
        Load MuJoCo model from file.
        
        Args:
            model_path: Path to robot model (URDF or MuJoCo XML)
        """
        try:
            if not os.path.exists(model_path):
                raise FileNotFoundError(f"Model file not found: {model_path}")
            
            # Load model based on file extension
            if model_path.endswith('.urdf'):
                self.model = mujoco.MjModel.from_xml_path(model_path)
            elif model_path.endswith('.xml'):
                self.model = mujoco.MjModel.from_xml_path(model_path)
            else:
                raise ValueError(f"Unsupported model format: {model_path}")
            
            # Create data
            self.data = mujoco.MjData(self.model)
            
            # Reset to initial state
            mujoco.mj_resetData(self.model, self.data)
            
            self.get_logger().info(
                f'Successfully loaded model with {self.model.nq} DOF'
            )
            
            # Check if end effector site exists in the model
            self._check_ee_site_availability()
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise
    
    def _check_ee_site_availability(self):
        """Check if the end effector site exists in the loaded model."""
        if self.model is None:
            self.ee_site_available = False
            return
        
        # Check if site exists
        site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, self.ee_site_name
        )
        
        if site_id >= 0:
            self.ee_site_available = True
            self.get_logger().info(
                f'End effector site "{self.ee_site_name}" found in model. '
                'End pose publishing enabled.'
            )
        else:
            self.ee_site_available = False
            if self.enable_end_pose:
                self.get_logger().info(
                    f'End effector site "{self.ee_site_name}" not found in model. '
                    'End pose publishing will be skipped. '
                    'This is normal for robots without site markers.'
                )
    
    def _inverse_dynamics_efforts_for_published_joints(self) -> list:
        """
        Map data.qfrc_inverse (length nv) to one scalar per published joint.

        Published joint count matches publish_joint_states:
        min(nu, njnt). Uses jnt_dofadr and consecutive dof span per joint.
        Multi-dof joints use the Euclidean norm of their ID torque subvector.
        """
        n = min(self.model.nu, self.model.njnt)
        out = []
        for j in range(n):
            ad = int(self.model.jnt_dofadr[j])
            if j + 1 < self.model.njnt:
                ad_next = int(self.model.jnt_dofadr[j + 1])
                ndof = max(1, ad_next - ad)
            else:
                ndof = max(1, self.model.nv - ad)
            if ndof == 1:
                out.append(float(self.data.qfrc_inverse[ad]))
            else:
                vec = np.asarray(self.data.qfrc_inverse[ad:ad + ndof], dtype=float)
                out.append(float(np.linalg.norm(vec)))
        # Downstream slices [:nu]; pad if model has nu > len(out)
        if len(out) < self.model.nu:
            out.extend([0.0] * (self.model.nu - len(out)))
        elif len(out) > self.model.nu:
            out = out[: self.model.nu]
        return out
    
    def simulation_step(self):
        """Execute one simulation step and publish joint states."""
        if self.model is None or self.data is None:
            return
        
        # Step simulation
        mujoco.mj_step(self.model, self.data)
        if self.joint_effort_source == 'mj_inverse':
            # mj_step 积分后 (qpos, qvel) 已更新，但 qacc 仍是积分前的值
            # → 直接调 mj_inverse 会使用不匹配的 (qpos_new, qvel_new, qacc_old)
            # → qfrc_inverse 输出垃圾数据（10^14 量级）
            # 必须先 mj_forward 使 qacc 与当前 (qpos, qvel, ctrl) 一致
            mujoco.mj_forward(self.model, self.data)
            mujoco.mj_inverse(self.model, self.data)

        with self.motor_state_lock:
            self.motor_state['pos'] = self.data.qpos.tolist()
            self.motor_state['vel'] = self.data.qvel.tolist()
            if self.joint_effort_source == 'mj_inverse':
                self.motor_state['frc'] = (
                    self._inverse_dynamics_efforts_for_published_joints()
                )
            else:
                # MIT PD + ff at actuators; not rigid-body ID torque in general
                self.motor_state['frc'] = self.data.actuator_force.tolist()
        
        # Sync viewer if enabled (viewer may be torn down during shutdown)
        if self.viewer is not None:
            try:
                self.viewer.sync()
            except Exception:
                self.viewer = None
        
        # Publish joint states
        self.publish_joint_states()
        
        # Publish end effector pose
        self.publish_end_pose()

        # Compute and apply control (MIT control runs every cycle)
        with self.control_cmd_lock:
            cmd = self.latest_cmd
        
        if cmd is not None:
            ctrl = self.compute_control(cmd)
            if ctrl is not None:
                self.data.ctrl[:] = ctrl
    
    def publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Get joint names and states (only publish actuated joints)
        msg.name = []
        msg.position = []
        msg.velocity = []
        
        # Only publish joints with actuators (exclude coupled joints)
        num_joints_to_publish = min(self.model.nu, self.model.njnt)
        
        for i in range(num_joints_to_publish):
            joint_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, i
            )
            if joint_name:
                msg.name.append(joint_name)
            else:
                msg.name.append(f'joint_{i}')
        
        # Get positions, velocities and efforts (actuated joints only)
        with self.motor_state_lock:
            if self.motor_state['pos'] is not None:
                msg.position = self.motor_state['pos'][:num_joints_to_publish]
                msg.velocity = self.motor_state['vel'][:num_joints_to_publish]
                msg.effort = self.motor_state['frc'][:self.model.nu]
                
                # Apply gripper mapping for gripper joint (simulation -> real robot)
                if self.enable_gripper_mapping and self.gripper_mapper is not None:
                    idx = self.gripper_joint_index

                    # Convert position: simulation -> real robot
                    if idx < len(msg.position):
                        msg.position[idx] = self.gripper_mapper.sim_to_real_pos(
                            msg.position[idx]
                        )
                    
                    # Convert velocity: simulation -> real robot
                    if idx < len(msg.velocity):
                        msg.velocity[idx] = self.gripper_mapper.velocity_sim_to_real(
                            msg.velocity[idx]
                        )
                    
                    # Convert effort: simulation -> real robot
                    if idx < len(msg.effort):
                        msg.effort[idx] = self.gripper_mapper.effort_sim_to_real(
                            msg.effort[idx]
                        )
            else:
                # Initial state not ready
                return
        
        # DEBUG: log first joint_state dimension to verify nu counts position actuator
        if not hasattr(self, '_joint_dim_logged'):
            self._joint_dim_logged = True
            self.get_logger().info(
                f'[{self.robot_name}] joint_state dimension: {len(msg.position)} '
                f'(model.nu={self.model.nu}, model.njnt={self.model.njnt}, names={msg.name})'
            )

        self.joint_state_pub.publish(msg)

    def publish_end_pose(self):
        """Publish end effector pose as 4x4 homogeneous transformation matrix."""
        if self.model is None or self.data is None:
            return
        
        # Skip if end pose publishing is disabled
        if not self.enable_end_pose:
            return
        
        # Skip if site is not available (checked during model load)
        if not self.ee_site_available:
            return
        
        try:
            # Get site ID by name
            site_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, self.ee_site_name
            )
            
            if site_id < 0:
                # Site not found, mark as unavailable and log once
                self.ee_site_available = False
                if not self.ee_site_warning_logged:
                    self.get_logger().warn(
                        f'End effector site "{self.ee_site_name}" not found in model. '
                        'End pose publishing disabled. This is normal for robots '
                        'without site markers defined in their MJCF/URDF.'
                    )
                    self.ee_site_warning_logged = True
                return
            
            # Get site position (3D vector in world/baselink frame)
            site_pos = self.data.site_xpos[site_id]  # [x, y, z]
            
            # Get site rotation matrix (3x3, row-major as 9 elements)
            site_rot = self.data.site_xmat[site_id].reshape(3, 3)
            
            # Construct 4x4 homogeneous transformation matrix
            # [R11, R12, R13, Tx]
            # [R21, R22, R23, Ty]
            # [R31, R32, R33, Tz]
            # [0,   0,   0,   1 ]
            T = np.eye(4)
            T[:3, :3] = site_rot  # Rotation part
            T[:3, 3] = site_pos   # Translation part
            
            # Flatten to 16-element array (row-major)
            homogeneous_matrix = T.flatten().tolist()
            
            # Create and publish EndPoseData message
            msg = EndPoseData()
            msg.name = self.ee_site_name
            msg.type = 'ee'
            msg.homogeneous_matrix = homogeneous_matrix
            
            # Get gripper value (joint7 position)
            with self.motor_state_lock:
                if self.motor_state['pos'] is not None:
                    if self.gripper_joint_index < len(self.motor_state['pos']):
                        gripper_pos = self.motor_state['pos'][self.gripper_joint_index]
                        
                        # Apply gripper mapping if enabled
                        if self.enable_gripper_mapping and self.gripper_mapper is not None:
                            msg.gripper = self.gripper_mapper.sim_to_real_pos(gripper_pos)
                        else:
                            msg.gripper = gripper_pos
                    else:
                        msg.gripper = 0.0
                    
                    # Fill joint information
                    msg.joint_position = self.motor_state['pos'][:self.model.nu]
                    msg.joint_velocity = self.motor_state['vel'][:self.model.nu]
                    msg.joint_effort = self.motor_state['frc'][:self.model.nu]
                else:
                    msg.gripper = 0.0
                    msg.joint_position = []
                    msg.joint_velocity = []
                    msg.joint_effort = []
            
            self.end_pose_pub.publish(msg)
            
        except Exception as e:
            # Log error once to help with debugging
            if not self.ee_site_warning_logged:
                self.get_logger().warn(
                    f'Error publishing end pose: {e}. '
                    'End pose publishing will be disabled.'
                )
                self.ee_site_warning_logged = True
                self.ee_site_available = False
    
    def joint_command_callback(self, msg: JointMitControl):
        """Handle incoming joint control commands."""
        # Store the latest command, control will be computed in simulation_step
        with self.control_cmd_lock:
            self.latest_cmd = msg
    
    def compute_control(self, msg: JointMitControl):
        """Compute MIT-style control commands."""
        if len(msg.joint_position) != self.model.nu:
            self.get_logger().warn(
                f'Joint position array length {len(msg.joint_position)} '
                f'does not match number of actuators {self.model.nu}'
            )
            return None
        
        # For MIT control, we need to implement PD control
        # ctrl = kp * (pos_des - pos) + kd * (vel_des - vel) + torque_ff
        kp = np.array(msg.kp) if msg.kp else np.zeros(self.model.nu)
        kd = np.array(msg.kd) if msg.kd else np.zeros(self.model.nu)
        pos_des = np.array(msg.joint_position)
        if msg.joint_velocity:
            vel_des = np.array(msg.joint_velocity)
        else:
            vel_des = np.zeros(self.model.nu)
        if msg.torque:
            torque_ff = np.array(msg.torque)
        else:
            torque_ff = np.zeros(self.model.nu)
        
        # Apply gripper mapping for gripper joint (real robot -> simulation)
        if self.enable_gripper_mapping and self.gripper_mapper is not None:
            idx = self.gripper_joint_index

            # Convert position: real robot -> simulation
            if idx < len(pos_des):
                pos_des[idx] = self.gripper_mapper.real_to_sim_pos(pos_des[idx])
            
            # Convert velocity: real robot -> simulation
            if idx < len(vel_des):
                vel_des[idx] = self.gripper_mapper.velocity_real_to_sim(vel_des[idx])
            
            # Convert torque: real robot -> simulation
            if idx < len(torque_ff):
                torque_ff[idx] = self.gripper_mapper.effort_real_to_sim(torque_ff[idx])
        
        # Current joint states (read with lock)
        with self.motor_state_lock:
            if self.motor_state['pos'] is None:
                # State not ready yet
                return None
            pos = np.array(self.motor_state['pos'][:self.model.nu])
            vel = np.array(self.motor_state['vel'][:self.model.nu])
        
        # PD control
        pos_error = pos_des - pos
        vel_error = vel_des - vel
        
        # Control signal
        ctrl = kp * pos_error + kd * vel_error + torque_ff

        # AR5 gripper / suction cup use non-standard actuators (position or
        # revolute identity); MIT-formula has force semantics that don't match.
        # Set ctrl=pos_des directly to bypass PD.
        if (self.enable_gripper_mapping and self.gripper_mapper is not None
                and self.robot_type in ('ar5_gripper', 'ar5_suction_cup')):
            idx = self.gripper_joint_index
            if idx < self.model.nu:
                ctrl[idx] = pos_des[idx]

        return ctrl
    
    def publish_arm_status(self):
        """Publish TeleOpArmStatus message at 10 Hz."""
        msg = TeleOpArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # 机械臂名称
        msg.arm_name = 'mujoco_sim_arm'
        
        # 连接状态 (仿真环境默认有线连接)
        msg.connection_status = TeleOpArmStatus.CONNECTION_WIRED
        
        # 设备状态 (仿真环境默认正常)
        msg.device_status = TeleOpArmStatus.DEVICE_STATUS_NORMAL
        
        # 通讯统计 (仿真环境模拟)
        self.total_packets += 1
        msg.total_packet_count = self.total_packets
        msg.total_packet_loss_count = self.packet_loss_count
        
        if self.total_packets > 0:
            loss_rate = self.packet_loss_count / self.total_packets
            msg.overall_packet_loss_rate = float(loss_rate)
        else:
            msg.overall_packet_loss_rate = 0.0
        
        # 错误信息
        msg.error_code = 0
        msg.error_message = ''
        
        # 填充各电机状态
        msg.motor_status_array = []
        
        if self.model is not None and self.data is not None:
            with self.motor_state_lock:
                motor_positions = self.motor_state['pos']
                motor_forces = self.motor_state['frc']
                
                if motor_positions is not None:
                    # 为每个执行器创建 MotorStatus
                    for i in range(min(self.model.nu, len(motor_forces))):
                        motor_status = MotorStatus()
                        motor_status.motor_id = i
                        
                        # 获取电机名称
                        actuator_name = mujoco.mj_id2name(
                            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
                        )
                        if actuator_name:
                            motor_status.motor_name = actuator_name
                        else:
                            motor_status.motor_name = f'motor_{i}'
                        
                        # 电机温度 (仿真环境模拟，基于力矩估算)
                        # 温度 = 基础温度 + 力矩贡献
                        base_temp = 25.0  # 室温
                        torque_contribution = abs(motor_forces[i]) * 2.0  # 简化模型
                        motor_status.temperature = base_temp + torque_contribution
                        
                        # 电机在线状态 (仿真环境默认在线)
                        motor_status.is_online = True
                        
                        # 电机工作模式 (根据是否有控制指令判断)
                        with self.control_cmd_lock:
                            if self.latest_cmd is not None:
                                motor_status.current_mode = MotorStatus.MODE_MIT_CONTROL
                            else:
                                motor_status.current_mode = MotorStatus.MODE_IDLE
                        
                        # 电机丢包率 (仿真环境模拟，随机小波动)
                        motor_status.packet_loss_rate = 0.0
                        
                        msg.motor_status_array.append(motor_status)
        
        # 发布消息
        self.arm_status_pub.publish(msg)


def main(args=None):
    """Main entry point for MuJoCo simulator node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MuJoCoSimulatorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}', flush=True)
    finally:
        if node is not None:
            try:
                node.close_viewer()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            # Context may already be shut down (signal / nested interrupt)
            pass


if __name__ == '__main__':
    main()

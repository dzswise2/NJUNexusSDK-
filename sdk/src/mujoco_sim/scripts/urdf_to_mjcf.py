#!/usr/bin/env python3
"""
URDF to MJCF Converter (Split Mode)

Converts URDF to pure MJCF, then creates a wrapper with actuators.
This avoids URDF parser issues while maintaining modularity.
"""

import argparse
import os
import sys
import yaml
import xml.etree.ElementTree as ET
import mujoco
import tempfile
import shutil
import glob
import re
from pathlib import Path


def parse_urdf_joints(urdf_path):
    """Parse URDF file to extract joint information.
    
    Only parses direct child <joint> elements of <robot>, skipping
    <joint> elements inside <transmission> blocks which are ROS/Gazebo
    specific and not relevant to MuJoCo.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    robot_name = root.get('name', '')
    
    joints = []
    for joint in root.findall('joint'):
        joint_type = joint.get('type')
        
        if joint_type == 'fixed':
            continue
        
        joint_name = joint.get('name')
        
        limit = joint.find('limit')
        if limit is not None:
            lower = float(limit.get('lower', '-3.14'))
            upper = float(limit.get('upper', '3.14'))
            effort = float(limit.get('effort', '100'))
            velocity = float(limit.get('velocity', '10'))
        else:
            lower, upper, effort, velocity = -3.14, 3.14, 100, 10
        
        joints.append({
            'name': joint_name,
            'type': joint_type,
            'lower': lower,
            'upper': upper,
            'effort': effort,
            'velocity': velocity
        })
    
    return joints, robot_name


def get_default_joint_friction(joint):
    """
    Get default friction parameters based on joint effort limit.
    
    Uses empirical formulas:
    - damping ≈ effort × 0.05 (5% of max torque)
    - frictionloss ≈ effort × 0.01 (1% of max torque)
    - armature ≈ effort × 0.001 (scales with joint size)
    
    With minimum values to ensure stability.
    
    Returns dict with damping, frictionloss, armature.
    """
    joint_type = joint.get('type', 'revolute')
    effort = joint.get('effort', 10.0)  # Default to 10 if not specified
    
    # Empirical coefficients (can be tuned)
    DAMPING_RATIO = 0.05       # 5% of max effort
    FRICTIONLOSS_RATIO = 0.01  # 1% of max effort
    ARMATURE_RATIO = 0.001     # 0.1% of max effort
    
    # Minimum values to ensure stability
    MIN_DAMPING = 0.05
    MIN_FRICTIONLOSS = 0.01
    MIN_ARMATURE = 0.0005
    
    # Calculate based on effort
    damping = max(effort * DAMPING_RATIO, MIN_DAMPING)
    frictionloss = max(effort * FRICTIONLOSS_RATIO, MIN_FRICTIONLOSS)
    armature = max(effort * ARMATURE_RATIO, MIN_ARMATURE)
    
    # For prismatic joints, use different scaling
    # (force-based instead of torque-based)
    if joint_type == 'prismatic':
        # Prismatic joints typically need less damping relative to force
        damping = max(effort * 0.1, MIN_DAMPING)  # 10% for linear
        frictionloss = max(effort * 0.02, MIN_FRICTIONLOSS)  # 2% for linear
        armature = max(effort * 0.01, MIN_ARMATURE)  # Higher for linear mass
    
    # Round to reasonable precision
    return {
        'damping': round(damping, 4),
        'frictionloss': round(frictionloss, 4),
        'armature': round(armature, 5)
    }


def generate_actuator_config(joints, config_path):
    """Generate YAML configuration file for actuators and joint properties."""
    config = {
        'actuators': {},
        'joint_properties': {},
        'scene': {
            'timestep': 0.002,
            'gravity': [0, 0, -9.81],
            'lighting': True,
            'ground': True
        },
        'geom_properties': {
            'base_link': {
                'contype': 0,
                'conaffinity': 0,
                'density': 0,
                'group': 1
            }
        }
    }
    
    for joint in joints:
        # Default to motor (torque control) for all joint types
        actuator_type = 'motor'
        gear = 1
        
        config['actuators'][joint['name']] = {
            'type': actuator_type,
            'gear': gear,
            'ctrlrange': [-joint['effort'], joint['effort']],
            'forcerange': [-joint['effort'], joint['effort']]
        }
        
        # Add joint friction/damping properties
        friction_params = get_default_joint_friction(joint)
        config['joint_properties'][joint['name']] = {
            'damping': friction_params['damping'],
            'frictionloss': friction_params['frictionloss'],
            'armature': friction_params['armature']
        }
        
        # Add default geom properties for each link
        # Extract link name from joint name (assuming naming convention)
        link_name = joint['name'].replace('joint', 'link')
        if link_name not in config['geom_properties']:
            config['geom_properties'][link_name] = {
                'contype': 1,
                'conaffinity': 15,
                'density': 0,
                'group': 0
            }
    
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print(f"✅ Generated actuator config: {config_path}")
    return config


def apply_geom_defaults(root, config):
    """
    Apply geom properties from config to specific links.
    """
    geom_properties = config.get('geom_properties', {})
    
    # Find worldbody
    worldbody = root.find('worldbody')
    if worldbody is None:
        return
    
    # Process geoms directly under worldbody (base_link)
    for geom in worldbody.findall('geom'):
        if geom.get('group') is None:  # Not already processed
            props = geom_properties.get('base_link', {})
            for key, value in props.items():
                if geom.get(key) is None:
                    geom.set(key, str(value))
    
    # Process geoms in bodies (named links)
    for body in root.iter('body'):
        body_name = body.get('name')
        if not body_name:
            continue
        
        # Get properties for this specific link
        props = geom_properties.get(body_name, {})
        if not props:
            continue
        
        # Apply to all geoms in this body
        for geom in body.findall('geom'):
            if geom.get('group') is not None:  # Already processed
                continue
            
            for key, value in props.items():
                if geom.get(key) is None:
                    geom.set(key, str(value))


def apply_joint_properties(root, config):
    """
    Apply joint friction/damping properties from config to joints in MJCF.
    
    Adds damping, frictionloss, and armature attributes to each joint
    based on the joint_properties section in config.
    """
    joint_properties = config.get('joint_properties', {})
    
    if not joint_properties:
        return
    
    # Iterate through all joints in the model
    for joint in root.iter('joint'):
        joint_name = joint.get('name')
        if not joint_name:
            continue
        
        # Get properties for this joint
        props = joint_properties.get(joint_name, {})
        if not props:
            continue
        
        # Apply damping
        if 'damping' in props and joint.get('damping') is None:
            joint.set('damping', str(props['damping']))
        
        # Apply frictionloss
        if 'frictionloss' in props and joint.get('frictionloss') is None:
            joint.set('frictionloss', str(props['frictionloss']))
        
        # Apply armature
        if 'armature' in props and joint.get('armature') is None:
            joint.set('armature', str(props['armature']))
        
        # Support additional joint properties if needed
        for key in ['stiffness', 'springref', 'solreflimit', 'solimplimit']:
            if key in props and joint.get(key) is None:
                value = props[key]
                if isinstance(value, list):
                    joint.set(key, ' '.join(map(str, value)))
                else:
                    joint.set(key, str(value))


def convert_urdf_to_pure_mjcf(urdf_path, output_path, config=None):
    """
    Convert URDF to pure MJCF using MuJoCo.
    This creates a standalone MJCF without actuators.
    """
    urdf_dir = os.path.dirname(urdf_path)
    mesh_dir = os.path.join(urdf_dir, '..', 'meshes')
    
    # Create temp directory
    temp_dir = tempfile.mkdtemp(prefix='mjcf_convert_')
    
    try:
        # Copy URDF and fix mesh paths
        temp_urdf = os.path.join(temp_dir, 'robot.urdf')
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        
        # Replace mesh paths: ../meshes/foo/bar -> foo/bar (resolved via meshdir)
        urdf_content = re.sub(
            r'filename="\.\./meshes/([^"]+)"',
            r'filename="\1"',
            urdf_content
        )

        # Inject compiler options for nested meshdirs and inertia fix
        urdf_content = re.sub(
            r'(<robot\b[^>]*>)',
            r'\1\n  <mujoco><compiler meshdir="meshes" balanceinertia="true"/></mujoco>',
            urdf_content,
            count=1
        )

        with open(temp_urdf, 'w') as f:
            f.write(urdf_content)

        # Copy meshes (preserve subdirectory layout for package:// or nested paths)
        if os.path.exists(mesh_dir):
            dest_mesh_dir = os.path.join(temp_dir, 'meshes')
            shutil.copytree(mesh_dir, dest_mesh_dir, dirs_exist_ok=True)

        # Load with MuJoCo
        model = mujoco.MjModel.from_xml_path(temp_urdf)
        
        # Save as MJCF
        temp_xml = os.path.join(temp_dir, 'converted.xml')
        mujoco.mj_saveLastXML(temp_xml, model)
        
        # Read and clean up the XML
        tree = ET.parse(temp_xml)
        root = tree.getroot()
        
        # Update compiler to use correct mesh directory
        compiler = root.find('compiler')
        if compiler is None:
            compiler = ET.Element('compiler')
            root.insert(0, compiler)
        
        output_dir = os.path.dirname(output_path)
        rel_mesh_dir = os.path.relpath(mesh_dir, output_dir)
        compiler.set('meshdir', rel_mesh_dir)
        compiler.set('angle', 'radian')
        
        # Apply geom defaults from config if provided
        if config:
            apply_geom_defaults(root, config)
            # Apply joint friction/damping properties
            apply_joint_properties(root, config)
        
        # Pretty print and save
        indent_xml(root)
        tree.write(output_path, encoding='utf-8', xml_declaration=True)
        
        print(f"✅ Converted URDF to base MJCF: {output_path}")
        
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


def generate_wrapper_mjcf(base_mjcf_path, output_path, config, robot_name=''):
    """
    Generate wrapper MJCF that includes base MJCF and adds actuators.
    """
    output_dir = os.path.dirname(output_path)
    rel_base_path = os.path.relpath(base_mjcf_path, output_dir)
    
    # Build wrapper XML, use robot_name from URDF if available
    model_name = robot_name if robot_name else 'robot_with_actuators'
    root = ET.Element('mujoco', model=model_name)
    
    # Options
    option = ET.SubElement(root, 'option')
    timestep = config.get('scene', {}).get('timestep', 0.002)
    option.set('timestep', str(timestep))
    gravity = config.get('scene', {}).get('gravity', [0, 0, -9.81])
    option.set('gravity', ' '.join(map(str, gravity)))
    
    # Visual settings
    visual = ET.SubElement(root, 'visual')
    headlight = ET.SubElement(visual, 'headlight')
    headlight.set('ambient', '0.5 0.5 0.5')
    
    # Assets for ground
    if config.get('scene', {}).get('ground', True):
        asset = ET.SubElement(root, 'asset')
        texture = ET.SubElement(asset, 'texture')
        texture.set('name', 'grid')
        texture.set('type', '2d')
        texture.set('builtin', 'checker')
        texture.set('width', '512')
        texture.set('height', '512')
        texture.set('rgb1', '0.1 0.2 0.3')
        texture.set('rgb2', '0.2 0.3 0.4')
        
        material = ET.SubElement(asset, 'material')
        material.set('name', 'grid')
        material.set('texture', 'grid')
        material.set('texrepeat', '1 1')
        material.set('texuniform', 'true')
        material.set('reflectance', '0.2')
    
    # Include base MJCF
    include = ET.SubElement(root, 'include')
    include.set('file', rel_base_path)
    
    # Worldbody additions (ground, lighting)
    worldbody = ET.SubElement(root, 'worldbody')
    
    if config.get('scene', {}).get('lighting', True):
        light = ET.SubElement(worldbody, 'light')
        light.set('name', 'top')
        light.set('pos', '0 0 2')
        light.set('dir', '0 0 -1')
        light.set('directional', 'true')
    
    if config.get('scene', {}).get('ground', True):
        ground = ET.SubElement(worldbody, 'geom')
        ground.set('name', 'ground')
        ground.set('type', 'plane')
        ground.set('size', '0 0 0.05')
        ground.set('material', 'grid')
        ground.set('condim', '3')
    
    # Actuators
    actuator = ET.SubElement(root, 'actuator')
    for joint_name, act_config in config.get('actuators', {}).items():
        act_type = act_config['type']
        act_elem = ET.SubElement(actuator, act_type)
        act_elem.set('name', f'act_{joint_name}')
        act_elem.set('joint', joint_name)
        
        if 'gear' in act_config:
            act_elem.set('gear', str(act_config['gear']))
        if 'kp' in act_config and act_config['kp'] > 0:
            act_elem.set('kp', str(act_config['kp']))
        if 'ctrlrange' in act_config:
            act_elem.set('ctrlrange', 
                        ' '.join(map(str, act_config['ctrlrange'])))
        if 'forcerange' in act_config:
            act_elem.set('forcerange', 
                        ' '.join(map(str, act_config['forcerange'])))
    
    # Save
    indent_xml(root)
    tree = ET.ElementTree(root)
    tree.write(output_path, encoding='utf-8', xml_declaration=True)
    
    print(f"✅ Generated wrapper MJCF: {output_path}")


def indent_xml(elem, level=0):
    """Pretty print XML with indentation."""
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for child in elem:
            indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def main():
    parser = argparse.ArgumentParser(
        description='Convert URDF to MJCF (split mode: base + wrapper)'
    )
    parser.add_argument(
        'urdf_path',
        help='Path to input URDF file'
    )
    parser.add_argument(
        '-o', '--output',
        help='Path to output wrapper MJCF (default: <urdf_name>.xml)'
    )
    parser.add_argument(
        '-c', '--config',
        help='Path to actuator config YAML'
    )
    parser.add_argument(
        '--no-scene',
        action='store_true',
        help='Do not add scene elements'
    )
    
    args = parser.parse_args()
    
    # Validate input
    if not os.path.exists(args.urdf_path):
        print(f"❌ Error: URDF file not found: {args.urdf_path}")
        sys.exit(1)
    
    # Determine output paths
    urdf_dir = os.path.dirname(args.urdf_path)
    urdf_name = os.path.splitext(os.path.basename(args.urdf_path))[0]
    
    # Output to mjcf directory instead of urdf directory
    robot_desc_dir = os.path.dirname(urdf_dir)  # y1_description/
    mjcf_dir = os.path.join(robot_desc_dir, 'mjcf')
    
    # Create mjcf directory if it doesn't exist
    os.makedirs(mjcf_dir, exist_ok=True)
    
    if args.output:
        wrapper_path = args.output
    else:
        wrapper_path = os.path.join(mjcf_dir, f"{urdf_name}.xml")
    
    base_path = os.path.join(mjcf_dir, f"{urdf_name}_base.xml")
    config_path = args.config or os.path.join(mjcf_dir, f"{urdf_name}_config.yaml")
    
    # Parse joints
    print(f"📖 Parsing URDF: {args.urdf_path}")
    joints, robot_name = parse_urdf_joints(args.urdf_path)
    print(f"   Robot name: {robot_name}")
    print(f"   Found {len(joints)} movable joints")
    
    # Generate or load config
    if args.config and os.path.exists(args.config):
        print(f"📖 Loading config: {args.config}")
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)
    else:
        config = generate_actuator_config(joints, config_path)
    
    # Override scene settings
    if args.no_scene:
        config['scene']['lighting'] = False
        config['scene']['ground'] = False
    
    # Step 1: Convert URDF to base MJCF
    print("\n🔨 Step 1: Converting URDF to base MJCF...")
    convert_urdf_to_pure_mjcf(args.urdf_path, base_path, config)
    
    # Step 2: Generate wrapper with actuators
    print("\n🔨 Step 2: Generating wrapper with actuators...")
    generate_wrapper_mjcf(base_path, wrapper_path, config, robot_name)
    
    print("\n✅ Conversion complete!")
    print(f"   Base MJCF: {base_path}")
    print(f"   Wrapper MJCF: {wrapper_path}")
    print(f"   Config: {config_path if not args.config else args.config}")
    print("\n💡 Tips:")
    print("   - Base MJCF is the converted robot (no actuators)")
    print("   - Wrapper MJCF includes base + actuators + scene")
    print("   - Use wrapper MJCF for simulation")


if __name__ == '__main__':
    main()

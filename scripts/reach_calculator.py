#!/usr/bin/env python3
import sys
import subprocess
import tempfile
import os
import math
import numpy as np

import pybullet as p
import pybullet_data

def expand_xacro(xacro_path):
    """Run xacro and return the expanded URDF text."""
    try:
        return subprocess.check_output(['xacro', xacro_path]).decode()
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] xacro failed:\n{e}", file=sys.stderr)
        sys.exit(1)

def patch_mesh_paths(urdf_xml):
    """Replace package://so_arm_101_gazebo/meshes/... with absolute path."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    meshes_dir = os.path.abspath(os.path.join(script_dir, '..', 'meshes'))
    return urdf_xml.replace(
        'package://so_arm_101_gazebo/meshes/', 
        meshes_dir + os.sep
    )

def write_temp_urdf(xml_text):
    """Write URDF text to a temp file and return its path."""
    fd, path = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(xml_text)
    return path

def find_link_index(robot_id, target_link_name):
    """Search through getJointInfo to find the link index for the given link name."""
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        link_name = info[12].decode('utf-8')
        if link_name == target_link_name:
            return i
    return None

def get_joint_limits(robot_id):
    """Extract joint limits from the robot."""
    joint_limits = []
    joint_names = []
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        if info[2] == p.JOINT_REVOLUTE:  # Only revolute joints
            joint_name = info[1].decode('utf-8')
            lower_limit = info[8]
            upper_limit = info[9]
            joint_limits.append((lower_limit, upper_limit))
            joint_names.append(joint_name)
    return joint_names, joint_limits

def calculate_reach_envelope(robot_id, tip_link_idx, num_samples=1000):
    """Calculate reach envelope by sampling joint configurations."""
    joint_names, joint_limits = get_joint_limits(robot_id)
    
    # Filter to only arm joints (exclude gripper)
    arm_joints = []
    arm_limits = []
    for i, name in enumerate(joint_names):
        if 'gripper' not in name.lower():
            arm_joints.append(i)
            arm_limits.append(joint_limits[i])
    
    print(f"\nArm joints found: {[joint_names[i] for i in arm_joints]}")
    
    reaches = []
    positions = []
    max_reach = 0
    max_config = None
    min_reach = float('inf')
    min_config = None
    
    # Random sampling
    for _ in range(num_samples):
        # Generate random joint configuration
        config = []
        for lower, upper in arm_limits:
            config.append(np.random.uniform(lower, upper))
        
        # Set joint positions
        for i, angle in enumerate(config):
            p.resetJointState(robot_id, arm_joints[i], angle)
        
        # Get end-effector position
        state = p.getLinkState(robot_id, tip_link_idx, computeForwardKinematics=True)
        pos = state[4]
        positions.append(pos)
        
        # Calculate horizontal reach
        reach = math.hypot(pos[0], pos[1])
        reaches.append(reach)
        
        if reach > max_reach:
            max_reach = reach
            max_config = config.copy()
        
        if reach < min_reach:
            min_reach = reach
            min_config = config.copy()
    
    # Also test some specific configurations
    print("\n=== Testing specific configurations ===")
    
    # All zeros (home position)
    for i in range(len(arm_joints)):
        p.resetJointState(robot_id, arm_joints[i], 0.0)
    state = p.getLinkState(robot_id, tip_link_idx, computeForwardKinematics=True)
    x, y, z = state[4]
    reach_home = math.hypot(x, y)
    print(f"\nHome position (all zeros):")
    print(f"  Position: ({x:.4f}, {y:.4f}, {z:.4f})")
    print(f"  Reach: {reach_home:.4f} m")
    
    # Maximum extension (try to extend forward)
    # This is a rough approximation - adjust based on your robot's kinematics
    extension_config = [0, -0.5, -0.5, 0, 0]  # Extend shoulder and elbow
    for i, angle in enumerate(extension_config[:len(arm_joints)]):
        p.resetJointState(robot_id, arm_joints[i], angle)
    state = p.getLinkState(robot_id, tip_link_idx, computeForwardKinematics=True)
    x, y, z = state[4]
    reach_extended = math.hypot(x, y)
    print(f"\nExtended position:")
    print(f"  Config: {extension_config[:len(arm_joints)]}")
    print(f"  Position: ({x:.4f}, {y:.4f}, {z:.4f})")
    print(f"  Reach: {reach_extended:.4f} m")
    
    return {
        'max_reach': max_reach,
        'max_config': max_config,
        'min_reach': min_reach,
        'min_config': min_config,
        'positions': positions,
        'reaches': reaches,
        'joint_names': [joint_names[i] for i in arm_joints]
    }

def analyze_workspace(positions):
    """Analyze the workspace from sampled positions."""
    positions = np.array(positions)
    
    x_min, x_max = positions[:, 0].min(), positions[:, 0].max()
    y_min, y_max = positions[:, 1].min(), positions[:, 1].max()
    z_min, z_max = positions[:, 2].min(), positions[:, 2].max()
    
    print("\n=== Workspace Analysis ===")
    print(f"X range: [{x_min:.4f}, {x_max:.4f}] m")
    print(f"Y range: [{y_min:.4f}, {y_max:.4f}] m")
    print(f"Z range: [{z_min:.4f}, {z_max:.4f}] m")
    
    # Calculate practical reach at table height
    table_height = 0.0  # Relative to robot base
    near_table_positions = positions[np.abs(positions[:, 2] - table_height) < 0.1]
    if len(near_table_positions) > 0:
        table_reaches = np.sqrt(near_table_positions[:, 0]**2 + near_table_positions[:, 1]**2)
        print(f"\nReach at table height (z ≈ {table_height} m):")
        print(f"  Min: {table_reaches.min():.4f} m")
        print(f"  Max: {table_reaches.max():.4f} m")
        print(f"  Mean: {table_reaches.mean():.4f} m")

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} path/to/so_arm_101_gazebo.urdf.xacro")
        sys.exit(1)

    xacro_path = sys.argv[1]
    print(f"1) Expanding xacro: {xacro_path}")
    urdf_xml = expand_xacro(xacro_path)

    print("2) Patching mesh paths → absolute …")
    urdf_xml = patch_mesh_paths(urdf_xml)

    print("3) Writing patched URDF to temp file …")
    urdf_path = write_temp_urdf(urdf_xml)

    # Start PyBullet in DIRECT mode (no GUI)
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    print("4) Loading URDF in PyBullet …")
    robot_id = p.loadURDF(urdf_path, useFixedBase=True)

    tip_link_name = 'gripper_jaw'
    link_idx = find_link_index(robot_id, tip_link_name)
    if link_idx is None:
        print(f"[ERROR] Could not find link '{tip_link_name}' in the URDF.", file=sys.stderr)
        sys.exit(1)

    # Calculate reach envelope
    print("\n5) Calculating reach envelope (this may take a moment)...")
    envelope = calculate_reach_envelope(robot_id, link_idx, num_samples=2000)
    
    print(f"\n=== Maximum Reach Configuration ===")
    print(f"Max reach: {envelope['max_reach']:.4f} m")
    print(f"Joint configuration:")
    for name, angle in zip(envelope['joint_names'], envelope['max_config']):
        print(f"  {name}: {angle:.4f} rad ({math.degrees(angle):.1f}°)")
    
    print(f"\n=== Minimum Reach Configuration ===")
    print(f"Min reach: {envelope['min_reach']:.4f} m")
    
    # Analyze workspace
    analyze_workspace(envelope['positions'])
    
    # Calculate recommended board size
    print("\n=== Chessboard Recommendations ===")
    # Conservative estimate: use 80% of max reach for reliable operation
    safe_max_reach = envelope['max_reach'] * 0.8
    print(f"Safe operating reach: {safe_max_reach:.4f} m")
    
    # For a square board where robot can reach all corners
    # If robot is at one edge center, max diagonal = safe_max_reach
    max_board_size = safe_max_reach * 2 / math.sqrt(2)
    print(f"Maximum board size (robot at edge center): {max_board_size:.4f} m")
    
    # Standard chess board is 8x8 squares
    # Typical tournament square size is 5-6.5cm
    print(f"\nWith 5cm squares: {0.05 * 8:.3f} m board")
    print(f"With 6cm squares: {0.06 * 8:.3f} m board")
    
    # Clean up
    p.disconnect(client)
    os.remove(urdf_path)

if __name__ == '__main__':
    main()

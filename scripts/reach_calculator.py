#!/usr/bin/env python3
import sys
import subprocess
import tempfile
import os
import math

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
    """
    Replace package://so_arm_101_gazebo/meshes/... with an absolute
    path to your local meshes directory.
    """
    # where this script lives
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # adjust if your meshes folder is elsewhere
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
        link_name = info[12].decode('utf-8')  # joint’s child link
        if link_name == target_link_name:
            return i
    return None

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

    # Zero out all joints
    for j in range(p.getNumJoints(robot_id)):
        p.resetJointState(robot_id, j, 0.0)

    # Compute forward kinematics
    state = p.getLinkState(robot_id, link_idx, computeForwardKinematics=True)
    x, y, z = state[4]
    reach = math.hypot(x, y)

    print("\n=== Reach Results ===")
    print(f"  x = {x:.4f} m")
    print(f"  y = {y:.4f} m")
    print(f"  z = {z:.4f} m")
    print(f"  horizontal reach = √(x² + y²) = {reach:.4f} m")

    # Clean up
    p.disconnect(client)
    os.remove(urdf_path)

if __name__ == '__main__':
    main()

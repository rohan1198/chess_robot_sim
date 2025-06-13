#!/usr/bin/env python3
"""
Ultra Final Board Center Fix
============================

FINAL fix based on visual confirmation:
- Gripper tips extend ~50mm below gripper_jaw origin
- Need 80mm hover height to get gripper tips 30mm above board
- Keep perfect alignment from previous solution
"""

import os
import numpy as np
import subprocess
import tempfile
import pybullet as p
import pybullet_data
import json
import math
from pathlib import Path

def setup_pybullet_robot(urdf_path):
    """Setup PyBullet robot for IK calculation."""
    
    # Expand and process URDF
    urdf_xml = subprocess.check_output(['xacro', urdf_path]).decode()
    
    # Patch mesh paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    meshes_dir = os.path.abspath(os.path.join(script_dir, '..', 'meshes'))
    urdf_xml = urdf_xml.replace('package://so_arm_101_gazebo/meshes/', meshes_dir + os.sep)
    
    # Write temp file
    fd, temp_urdf_path = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(urdf_xml)
    
    # Start PyBullet
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Load robot
    robot_id = p.loadURDF(temp_urdf_path, useFixedBase=True)
    
    # Find controllable joints
    arm_joint_names = ['shoulder_rotation', 'shoulder_pitch', 'elbow', 'wrist_pitch', 'wrist_roll']
    controllable_joints = []
    
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode('utf-8')
        
        if joint_name in arm_joint_names and info[2] == p.JOINT_REVOLUTE:
            controllable_joints.append((i, joint_name))
    
    # Sort by desired order
    joint_order = {name: idx for idx, name in enumerate(arm_joint_names)}
    controllable_joints.sort(key=lambda x: joint_order[x[1]])
    
    # Find end-effector
    end_effector_link_idx = None
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        link_name = info[12].decode('utf-8')
        if link_name == 'gripper_jaw':
            end_effector_link_idx = i
            break
    
    # Clean up temp file
    os.remove(temp_urdf_path)
    
    return physics_client, robot_id, controllable_joints, end_effector_link_idx

def solve_board_center_ik_ultra_final():
    """ULTRA FINAL fix: 80mm hover height for proper gripper tip clearance."""
    
    print("🎯 ULTRA FINAL BOARD CENTER FIX")
    print("=" * 35)
    
    # Calculate target position
    print("📍 COORDINATE ANALYSIS:")
    print("-" * 25)
    
    board_center_world = np.array([0.75, 0.0, 0.405])
    robot_base_world = np.array([0.75, 0.275, 0.375])
    
    board_center_robot_frame = board_center_world - robot_base_world
    
    print(f"Board center (world): {board_center_world}")
    print(f"Robot base (world): {robot_base_world}")
    print(f"Board center (robot frame): {board_center_robot_frame}")
    
    # ULTRA FINAL hover height: 80mm (accounts for gripper tip extension)
    print(f"\n🔧 GRIPPER TIP ANALYSIS:")
    print("-" * 25)
    print("From visual inspection:")
    print("- Gripper_jaw origin was 50mm above board")
    print("- Gripper tips were touching board")
    print("- Therefore: Gripper tips extend ~50mm below gripper_jaw origin")
    print("- Target: Gripper tips 30mm above board")
    print("- Required: Gripper_jaw origin at 30mm + 50mm = 80mm above board")
    
    hover_height = 0.080  # 80mm hover (was 50mm)
    target_for_gripper_jaw_origin = board_center_robot_frame + np.array([0.0, 0.0, hover_height])
    
    print(f"\nTarget with ULTRA FINAL hover height (80mm): {target_for_gripper_jaw_origin}")
    
    # Keep the same gripper tip offset (worked perfectly for alignment)
    print(f"\n🔧 GRIPPER TIP OFFSET (UNCHANGED):")
    print("-" * 35)
    
    # Keep the final offset from previous attempt
    original_offset = np.array([0.0202, 0.0188, -0.0045])
    gripper_reach_extension = 0.010  # 10mm (this worked well)
    ultra_final_offset = original_offset + np.array([0.0, gripper_reach_extension, 0.0])
    
    print(f"Keeping successful offset: {ultra_final_offset}")
    print(f"Offset (mm): {ultra_final_offset * 1000}")
    print("(This gave us perfect X/Y alignment)")
    
    # Apply offset
    robot_target = target_for_gripper_jaw_origin + ultra_final_offset
    
    print(f"\n🎯 ULTRA FINAL IK TARGET:")
    print(f"Target position: {robot_target}")
    print(f"Distance from robot base: {np.linalg.norm(robot_target):.3f}m")
    
    print(f"\nComparison to previous attempts:")
    previous_target = np.array([0.0202, -0.2462, 0.0755])
    print(f"Previous target: {previous_target}")
    print(f"ULTRA target:    {robot_target}")
    print(f"Change (mm): {(robot_target - previous_target) * 1000}")
    print(f"  - Height increase: {(robot_target[2] - previous_target[2]) * 1000:.0f}mm")
    
    # Setup PyBullet
    script_dir = Path(__file__).parent.absolute()
    urdf_path = script_dir.parent / "urdf" / "so_arm_101_gazebo.urdf.xacro"
    
    physics_client, robot_id, controllable_joints, end_effector_link_idx = setup_pybullet_robot(str(urdf_path))
    
    # Joint limits
    joint_limits = {
        'shoulder_rotation': (-1.91986, 1.91986),
        'shoulder_pitch': (-1.74533, 1.74533),
        'elbow': (-1.74533, 1.74533),
        'wrist_pitch': (-1.74533, 1.74533),
        'wrist_roll': (-2.79253, 2.79253),
    }
    
    joint_names = [name for idx, name in controllable_joints]
    joint_indices = [idx for idx, name in controllable_joints]
    
    lower_limits = [joint_limits[name][0] for name in joint_names]
    upper_limits = [joint_limits[name][1] for name in joint_names]
    
    # Try IK starting from final solution (which had perfect alignment)
    final_solution = [-0.0014, 0.2672, 0.6543, 0.2497, 0.0649]  # From final fix
    
    strategies = [
        final_solution,  # Start with final solution
        [0.0, 0.0, 0.0, 0.0, 0.0],  # Home position backup
    ]
    
    solution = None
    
    print(f"\n🔧 IK SOLVING:")
    print("-" * 15)
    
    for i, start_pose in enumerate(strategies):
        strategy_name = "Final solution" if i == 0 else "Home position"
        print(f"\nTrying {strategy_name}: {[f'{math.degrees(a):.1f}°' for a in start_pose]}")
        
        # Set starting pose
        for j, angle in enumerate(start_pose):
            if j < len(joint_indices):
                p.resetJointState(robot_id, joint_indices[j], angle)
        
        # Solve IK
        ik_solution = p.calculateInverseKinematics(
            robot_id,
            end_effector_link_idx,
            robot_target,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=[upper - lower for lower, upper in zip(lower_limits, upper_limits)],
            restPoses=start_pose,
            maxNumIterations=500,
            residualThreshold=1e-5
        )
        
        # Extract solution
        joint_solution = list(ik_solution[:len(joint_indices)])
        
        # Validate joint limits
        valid = all(lower_limits[j] <= joint_solution[j] <= upper_limits[j] for j in range(len(joint_names)))
        
        if not valid:
            print("  ❌ Joint limits violated")
            continue
        
        # Verify with forward kinematics
        for j, angle in enumerate(joint_solution):
            p.resetJointState(robot_id, joint_indices[j], angle)
        
        state = p.getLinkState(robot_id, end_effector_link_idx, computeForwardKinematics=True)
        achieved_pos = np.array(state[4])
        error = np.linalg.norm(achieved_pos - robot_target)
        
        print(f"  Achieved position: {achieved_pos}")
        print(f"  Position error: {error*1000:.1f}mm")
        
        if error <= 0.01:  # 10mm tolerance
            solution = joint_solution
            print(f"  ✅ Success! Error: {error*1000:.1f}mm")
            break
        else:
            print(f"  ❌ Error too large: {error*1000:.1f}mm")
    
    p.disconnect(physics_client)
    
    if solution is None:
        print("\n❌ Could not find valid IK solution")
        return None
    
    print(f"\n🎉 ULTRA FINAL SOLUTION:")
    print("-" * 25)
    for name, angle in zip(joint_names, solution):
        print(f"  {name}: {angle:.4f} rad ({math.degrees(angle):+6.1f}°)")
    
    print(f"\nComparison to final solution:")
    final_joints = [-0.0014, 0.2672, 0.6543, 0.2497, 0.0649]
    for i, (name, new_angle, final_angle) in enumerate(zip(joint_names, solution, final_joints)):
        diff_deg = math.degrees(new_angle - final_angle)
        print(f"  {name}: {diff_deg:+6.1f}° change")
    
    return solution, robot_target, ultra_final_offset

def create_ultra_final_test_file(joint_solution, target_position, gripper_offset):
    """Create test file for ultra final fix."""
    
    script_dir = Path(__file__).parent.absolute()
    test_file = script_dir / "board_center_solution_ultra_final.json"
    
    board_center_data = {
        "metadata": {
            "description": "Board center test with ULTRA FINAL gripper tip clearance",
            "purpose": "Gripper TIP 30mm above magenta center cylinder - ULTRA FINAL",
            "expected_alignment": "Perfect gripper tip positioning 30mm above board center",
            "tolerance": "5mm",
            "fixes_applied": [
                "ULTRA FINAL: Increased hover height to 80mm (accounts for 50mm gripper extension)",
                "VISUAL: Based on direct observation of gripper touching board at 50mm hover",
                "CALCULATION: 30mm target + 50mm gripper extension = 80mm hover required",
                "KEPT: Perfect X/Y alignment and reach from previous solutions",
                "RESULT: Gripper tips should be exactly 30mm above board center"
            ]
        },
        "board_center": {
            "position": target_position.tolist(),
            "joints": joint_solution,
            "gripper_tip_offset": gripper_offset.tolist(),
            "world_position": [0.75, 0.0, 0.485],  # 80mm hover
            "description": "Board center + 80mm hover height + perfect alignment",
            "gripper_analysis": {
                "gripper_jaw_height_above_board": "80mm",
                "gripper_tip_extension_below_jaw": "50mm", 
                "resulting_tip_height_above_board": "30mm"
            }
        }
    }
    
    with open(test_file, 'w') as f:
        json.dump(board_center_data, f, indent=2)
    
    print(f"\n💾 ULTRA FINAL solution saved to: {test_file.name}")
    return test_file

def main():
    """Main function."""
    
    try:
        # Solve IK with ultra final adjustments
        result = solve_board_center_ik_ultra_final()
        
        if result is None:
            print("❌ Could not solve board center IK")
            return
        
        joint_solution, target_position, gripper_offset = result
        
        # Create test files
        create_ultra_final_test_file(joint_solution, target_position, gripper_offset)
        
        print("\n🚀 ULTRA FINAL TEST:")
        print("=" * 20)
        print("1. Launch simulation:")
        print("   ros2 launch chess_robot_sim chess_gazebo.launch.py")
        print("")
        print("2. Test ULTRA FINAL positioning:")
        print("   python3 test_board_center_ultra_final.py")
        print("")
        print("3. Expected result:")
        print("   ✓ Perfect X/Y alignment (maintained)")
        print("   ✓ Gripper_jaw origin 80mm above board")
        print("   ✓ Gripper tips 30mm above board center")
        print("   ✓ PERFECT for chess piece operations!")
        print("")
        print("🧮 Math check:")
        print("   Gripper_jaw at 80mm + Board at 0mm - Gripper_extension 50mm = Tips at 30mm ✅")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()

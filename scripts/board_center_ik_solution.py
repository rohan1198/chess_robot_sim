#!/usr/bin/env python3
"""
Board Center IK Solution
========================

Generate exact IK solution for board center position for testing.
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

def solve_board_center_ik():
    """Solve IK for board center position."""
    
    print("🎯 SOLVING BOARD CENTER IK")
    print("=" * 40)
    
    # Target position (from board_center_test.py)
    robot_target = np.array([0.025, -0.21, 0.11])
    
    print(f"Target position (robot frame): {robot_target}")
    print(f"Distance from robot base: {np.linalg.norm(robot_target):.3f}m")
    
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
    
    # Try IK with different starting positions
    strategies = [
        [0.0, 0.0, 0.0, 0.0, 0.0],  # Home position
        [0.1, -0.2, 0.5, 0.0, 0.0],  # Slightly forward
        [0.0, -0.3, 0.8, -0.3, 0.0],  # More bent
    ]
    
    solution = None
    
    for i, start_pose in enumerate(strategies):
        print(f"\nTrying strategy {i+1}: {[f'{math.degrees(a):.1f}°' for a in start_pose]}")
        
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
        print("\n❌ Could not find valid IK solution for board center")
        return None
    
    print(f"\n🎉 BOARD CENTER IK SOLUTION:")
    print("-" * 35)
    for name, angle in zip(joint_names, solution):
        print(f"  {name}: {angle:.4f} rad ({math.degrees(angle):+6.1f}°)")
    
    return solution, robot_target

def create_board_center_test_file(joint_solution, target_position):
    """Create a test file for board center."""
    
    script_dir = Path(__file__).parent.absolute()
    test_file = script_dir / "board_center_solution.json"
    
    board_center_data = {
        "metadata": {
            "description": "Board center test for coordinate frame validation",
            "purpose": "Should align gripper with magenta center cylinder",
            "expected_alignment": "Perfect visual alignment",
            "tolerance": "5mm"
        },
        "board_center": {
            "position": target_position.tolist(),
            "joints": joint_solution,
            "world_position": [0.75, 0.04, 0.485],
            "description": "Board center + 80mm hover height"
        }
    }
    
    with open(test_file, 'w') as f:
        json.dump(board_center_data, f, indent=2)
    
    print(f"\n💾 Board center solution saved to: {test_file.name}")
    return test_file

def create_test_script():
    """Create a simple test script to move to board center."""
    
    script_dir = Path(__file__).parent.absolute()
    test_script = script_dir / "test_board_center.py"
    
    test_code = '''#!/usr/bin/env python3
"""
Board Center Test Script
=======================

Simple script to move robot to board center and validate alignment.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json
import time
from pathlib import Path

class BoardCenterTester(Node):
    """Test board center alignment."""
    
    def __init__(self):
        super().__init__('board_center_tester')
        
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        
        self.joint_names = [
            'shoulder_rotation', 'shoulder_pitch', 'elbow', 'wrist_pitch', 'wrist_roll'
        ]
        
        # Load board center solution
        script_dir = Path(__file__).parent.absolute()
        solution_file = script_dir / "board_center_solution.json"
        
        with open(solution_file, 'r') as f:
            data = json.load(f)
        
        self.board_center_joints = data['board_center']['joints']
        
        self.get_logger().info('🎯 Board Center Tester initialized!')
    
    def move_to_board_center(self):
        """Move robot to board center."""
        
        self.get_logger().info('🎯 Moving to board center...')
        self.get_logger().info('   Expected: Gripper aligns with MAGENTA center cylinder')
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.board_center_joints
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=5, nanosec=0)
        
        trajectory.points = [point]
        
        # Send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('❌ Goal send timeout')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            return False
        
        self.get_logger().info('📈 Movement in progress...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        
        if result_future.done():
            self.get_logger().info('✅ Movement completed!')
            self.get_logger().info('👁️  Visual check: Is gripper aligned with magenta cylinder?')
            return True
        else:
            self.get_logger().error('❌ Movement timeout')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    tester = BoardCenterTester()
    
    try:
        # Wait for action server
        tester.get_logger().info('Waiting for arm controller...')
        if not tester.arm_action_client.wait_for_server(timeout_sec=10.0):
            tester.get_logger().error('❌ Arm controller not available')
            return
        
        # Move to board center
        success = tester.move_to_board_center()
        
        if success:
            tester.get_logger().info('\\n📏 Use manual_measurement.py to measure actual position:')
            tester.get_logger().info('   python3 manual_measurement.py')
            tester.get_logger().info('   (Press ENTER to measure)')
        
    except KeyboardInterrupt:
        tester.get_logger().info('🛑 Test stopped')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
    
    with open(test_script, 'w') as f:
        f.write(test_code)
    
    # Make executable
    test_script.chmod(0o755)
    
    print(f"📝 Test script created: {test_script.name}")
    return test_script

def main():
    """Main function."""
    
    try:
        # Solve IK for board center
        result = solve_board_center_ik()
        
        if result is None:
            print("❌ Could not solve board center IK")
            return
        
        joint_solution, target_position = result
        
        # Create test files
        create_board_center_test_file(joint_solution, target_position)
        create_test_script()
        
        print("\n🚀 NEXT STEPS:")
        print("=" * 20)
        print("1. Launch simulation:")
        print("   ros2 launch chess_robot_sim chess_gazebo.launch.py")
        print("")
        print("2. Run board center test:")
        print("   python3 test_board_center.py")
        print("")
        print("3. Visually check alignment with magenta cylinder")
        print("")
        print("4. Measure actual position:")
        print("   python3 manual_measurement.py")
        print("   (Press ENTER when robot is at center)")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()

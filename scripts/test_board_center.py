#!/usr/bin/env python3
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
            tester.get_logger().info('\n📏 Use manual_measurement.py to measure actual position:')
            tester.get_logger().info('   python3 manual_measurement.py')
            tester.get_logger().info('   (Press ENTER to measure)')
        
    except KeyboardInterrupt:
        tester.get_logger().info('🛑 Test stopped')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

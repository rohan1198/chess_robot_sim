#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ChessCornerMover(Node):
    """
    Move the SO-ARM101 robot arm to all four corners of the chessboard in clockwise order.
    """
    
    def __init__(self):
        super().__init__('chess_corner_mover')
        
        # Action clients for robot control
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_action_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.arm_action_client.wait_for_server(timeout_sec=10.0)
        self.gripper_action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Action servers ready!')
        
        # Joint names for the arm (5 DOF)
        self.joint_names = [
            'base_joint',
            'shoulder_joint', 
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint'
        ]
        
        # Chess board corner coordinates (corrected)
        self.setup_corner_coordinates()
        
        # Predefined joint positions for reaching each corner
        self.setup_corner_joint_positions()
        
        self.get_logger().info('Chess Corner Mover initialized!')
    
    def setup_corner_coordinates(self):
        """Setup the corrected chess board corner coordinates"""
        self.board_center_x = 0.05
        self.board_center_y = 0.0
        self.board_height = 0.8315
        self.square_size = 0.04
        
        # Corrected board boundaries (8 squares × 0.04m = 0.32m total)
        self.board_min_x = self.board_center_x - 0.16  # -0.11
        self.board_max_x = self.board_center_x + 0.16  # +0.21
        self.board_min_y = self.board_center_y - 0.16  # -0.16
        self.board_max_y = self.board_center_y + 0.16  # +0.16
        
        # Corner coordinates (clockwise from bottom-left)
        self.corners = {
            'a1': (self.board_min_x, self.board_min_y, self.board_height),       # (-0.11, -0.16, 0.8315)
            'h1': (self.board_min_x, self.board_max_y, self.board_height),       # (-0.11, +0.16, 0.8315) 
            'h8': (self.board_max_x, self.board_max_y, self.board_height),       # (+0.21, +0.16, 0.8315)
            'a8': (self.board_max_x, self.board_min_y, self.board_height)        # (+0.21, -0.16, 0.8315)
        }
        
        self.get_logger().info('Chess board corners:')
        for corner, coords in self.corners.items():
            self.get_logger().info(f'  {corner}: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})')
    
    def setup_corner_joint_positions(self):
        """
        Define joint positions to reach each corner.
        These are estimated positions - you may need to tune them.
        
        Joint order: [base, shoulder, elbow, wrist_pitch, wrist_roll]
        """
        self.corner_joints = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],
            
            # a1: Bottom-left corner (closest to robot, left side)
            'a1': [0.4, -0.8, 0.7, -0.6, 0.0],
            
            # h1: Bottom-right corner (closest to robot, right side)  
            'h1': [-0.4, -0.8, 0.7, -0.6, 0.0],
            
            # h8: Top-right corner (farthest from robot, right side)
            'h8': [-0.2, -1.2, 1.0, -0.8, 0.0],
            
            # a8: Top-left corner (farthest from robot, left side)
            'a8': [0.2, -1.2, 1.0, -0.8, 0.0],
            
            # Safe observation position
            'observe': [0.0, -0.5, 0.3, -0.2, 0.0]
        }
    
    def send_arm_goal(self, joint_positions, duration_sec=3.0):
        """Send joint trajectory goal to arm controller"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set trajectory
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        # Send goal and wait for result
        self.get_logger().info(f'Moving to joint positions: {joint_positions}')
        future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('Movement completed successfully!')
            return True
        else:
            self.get_logger().error(f'Movement failed with error code: {result.result.error_code}')
            return False
    
    def send_gripper_goal(self, position, max_effort=3.0):
        """Send gripper command"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            return True
        return False
    
    def visit_corner(self, corner_name, pause_duration=2.0):
        """Visit a specific corner"""
        self.get_logger().info(f'\n=== Moving to corner {corner_name} ===')
        
        # Get target coordinates
        target_coords = self.corners[corner_name]
        self.get_logger().info(f'Target coordinates: ({target_coords[0]:.3f}, {target_coords[1]:.3f}, {target_coords[2]:.3f})')
        
        # Move to corner
        joint_positions = self.corner_joints[corner_name]
        success = self.send_arm_goal(joint_positions, duration_sec=4.0)
        
        if success:
            self.get_logger().info(f'✅ Reached corner {corner_name}')
            time.sleep(pause_duration)
        else:
            self.get_logger().error(f'❌ Failed to reach corner {corner_name}')
        
        return success
    
    def tour_corners(self):
        """Complete tour of all four corners in clockwise order"""
        self.get_logger().info('\n🏁 Starting chess board corner tour!')
        
        # 1. Move to home position
        self.get_logger().info('\n📍 Moving to home position...')
        self.send_arm_goal(self.corner_joints['home'], duration_sec=3.0)
        time.sleep(1.0)
        
        # 2. Open gripper for safety
        self.get_logger().info('🤲 Opening gripper...')
        self.send_gripper_goal(1.5)  # Open gripper
        time.sleep(1.0)
        
        # 3. Move to observation position
        self.get_logger().info('\n👀 Moving to observation position...')
        self.send_arm_goal(self.corner_joints['observe'], duration_sec=3.0)
        time.sleep(1.0)
        
        # 4. Visit corners in clockwise order
        corners_order = ['a1', 'h1', 'h8', 'a8']
        
        for i, corner in enumerate(corners_order, 1):
            self.get_logger().info(f'\n🎯 Corner {i}/4: {corner}')
            success = self.visit_corner(corner, pause_duration=3.0)
            
            if not success:
                self.get_logger().error(f'Tour aborted at corner {corner}')
                break
        
        # 5. Return to observation position
        self.get_logger().info('\n🏠 Returning to observation position...')
        self.send_arm_goal(self.corner_joints['observe'], duration_sec=3.0)
        time.sleep(1.0)
        
        # 6. Return to home
        self.get_logger().info('\n🏁 Returning to home position...')
        self.send_arm_goal(self.corner_joints['home'], duration_sec=3.0)
        
        self.get_logger().info('\n✅ Chess board corner tour completed!')
    
    def interactive_mode(self):
        """Interactive mode for testing individual corners"""
        print("\n🎮 Interactive Corner Mover")
        print("Commands:")
        print("  home    - Move to home position")
        print("  observe - Move to observation position") 
        print("  a1      - Move to corner a1 (bottom-left)")
        print("  h1      - Move to corner h1 (bottom-right)")
        print("  h8      - Move to corner h8 (top-right)")
        print("  a8      - Move to corner a8 (top-left)")
        print("  tour    - Complete clockwise tour of all corners")
        print("  open    - Open gripper")
        print("  close   - Close gripper")
        print("  quit    - Exit")
        
        while rclpy.ok():
            try:
                command = input("\n> ").strip().lower()
                
                if command == 'quit':
                    break
                elif command == 'home':
                    self.send_arm_goal(self.corner_joints['home'])
                elif command == 'observe':
                    self.send_arm_goal(self.corner_joints['observe'])
                elif command in ['a1', 'h1', 'h8', 'a8']:
                    self.visit_corner(command)
                elif command == 'tour':
                    self.tour_corners()
                elif command == 'open':
                    self.send_gripper_goal(1.5)
                elif command == 'close':
                    self.send_gripper_goal(0.2)
                elif command == '':
                    continue
                else:
                    print("Unknown command")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break


def main():
    rclpy.init()
    
    try:
        mover = ChessCornerMover()
        
        import sys
        if len(sys.argv) > 1 and sys.argv[1] == '--tour':
            # Run automatic tour
            mover.tour_corners()
        else:
            # Run interactive mode
            mover.interactive_mode()
            
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

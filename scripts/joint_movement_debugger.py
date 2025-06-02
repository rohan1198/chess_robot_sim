#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import json
import numpy as np
import math


class EnhancedJointDebugger(Node):
    """
    Enhanced debugger for testing coordinated joint movements to specific coordinates
    """
    
    def __init__(self):
        super().__init__('enhanced_joint_debugger')
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action client for arm control
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        
        self.latest_joint_state = None
        self.arm_joints = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
        
        # Chess board coordinate system (corrected)
        self.setup_chess_coordinates()
        
        # Storage for tested joint combinations
        self.saved_positions = {}
        self.load_saved_positions()
        
        self.get_logger().info('Enhanced Joint Debugger initialized')
        
        # Wait for action server
        if self.arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('✅ Arm action server available')
        else:
            self.get_logger().error('❌ Arm action server not available')
    
    def setup_chess_coordinates(self):
        """Setup chess board coordinate system"""
        self.board_center_x = 0.05
        self.board_center_y = 0.0
        self.board_height = 0.8315
        self.square_size = 0.04
        
        # Corrected board boundaries
        self.board_min_x = self.board_center_x - 0.16  # -0.11
        self.board_max_x = self.board_center_x + 0.16  # +0.21
        self.board_min_y = self.board_center_y - 0.16  # -0.16
        self.board_max_y = self.board_center_y + 0.16  # +0.16
        
        # Chess square coordinates
        self.chess_coords = {}
        files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        ranks = ['1', '2', '3', '4', '5', '6', '7', '8']
        
        for file_idx, file_char in enumerate(files):
            for rank_idx, rank_char in enumerate(ranks):
                square = file_char + rank_char
                x = self.board_min_x + (rank_idx * self.square_size)
                y = self.board_min_y + (file_idx * self.square_size)
                z = self.board_height
                self.chess_coords[square] = (x, y, z)
        
        # Key positions for testing
        self.test_positions = {
            'board_center': (self.board_center_x, self.board_center_y, self.board_height),
            'a1': self.chess_coords['a1'],  # (-0.11, -0.16, 0.8315)
            'h1': self.chess_coords['h1'],  # (-0.11, +0.16, 0.8315)
            'h8': self.chess_coords['h8'],  # (+0.21, +0.16, 0.8315)
            'a8': self.chess_coords['a8'],  # (+0.21, -0.16, 0.8315)
            'e4': self.chess_coords['e4'],  # Center-ish square
            'safe_above': (0.0, 0.0, 0.9),  # Safe position above board
        }
        
        self.get_logger().info('Chess coordinate system initialized')
        for name, coords in self.test_positions.items():
            self.get_logger().info(f'  {name}: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})')
    
    def joint_state_callback(self, msg):
        """Monitor current joint states"""
        self.latest_joint_state = msg
    
    def get_current_joint_positions(self):
        """Get current positions for all arm joints"""
        if not self.latest_joint_state:
            return [0.0] * 5
        
        positions = []
        for joint in self.arm_joints:
            if joint in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(joint)
                positions.append(self.latest_joint_state.position[idx])
            else:
                positions.append(0.0)
        return positions
    
    def send_joint_goal(self, joint_positions, duration=3.0):
        """Send coordinated joint movement goal"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        # Display what we're trying to do
        self.get_logger().info(f'Target joint positions:')
        for i, (joint, pos) in enumerate(zip(self.arm_joints, joint_positions)):
            self.get_logger().info(f'  {joint}: {pos:.3f} rad ({pos*180/3.14159:.1f}°)')
        
        # Send goal
        future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected!')
            return False
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('✅ Movement completed!')
            return True
        else:
            self.get_logger().error(f'❌ Movement failed with error: {result.result.error_code}')
            return False
    
    def test_coordinate_reach(self, target_name, joint_positions):
        """Test if given joint positions can reach target coordinates"""
        if target_name not in self.test_positions:
            self.get_logger().error(f'Unknown target: {target_name}')
            return False
        
        target_coords = self.test_positions[target_name]
        self.get_logger().info(f'\n🎯 Testing reach to {target_name}')
        self.get_logger().info(f'Target coordinates: ({target_coords[0]:.3f}, {target_coords[1]:.3f}, {target_coords[2]:.3f})')
        
        success = self.send_joint_goal(joint_positions)
        
        if success:
            # Give it a moment to settle
            time.sleep(1.0)
            
            # Ask user for visual feedback
            print(f"\n👀 Visual Check:")
            print(f"Target: {target_name} at ({target_coords[0]:.3f}, {target_coords[1]:.3f}, {target_coords[2]:.3f})")
            print(f"Does the robot arm end effector appear to be close to this position?")
            
            response = input("Rate the accuracy (1=way off, 5=perfect, q=quit): ").strip()
            
            if response == 'q':
                return False
            
            try:
                rating = int(response)
                if rating >= 4:
                    self.get_logger().info(f'✅ Good position for {target_name}! Saving...')
                    self.saved_positions[target_name] = joint_positions.copy()
                    self.save_positions_to_file()
                elif rating >= 2:
                    self.get_logger().info(f'⚠️ Decent position for {target_name}, but could be better')
                else:
                    self.get_logger().info(f'❌ Poor position for {target_name}')
                    
            except ValueError:
                self.get_logger().info('Invalid rating')
        
        return success
    
    def adjust_joint_positions(self, base_positions):
        """Interactive joint adjustment mode"""
        current_positions = base_positions.copy()
        
        print(f"\n🔧 Joint Adjustment Mode")
        print(f"Current positions: {[f'{p:.2f}' for p in current_positions]}")
        print(f"Commands:")
        print(f"  +<joint> <amount>  - Increase joint angle (e.g., '+base 0.1')")
        print(f"  -<joint> <amount>  - Decrease joint angle (e.g., '-shoulder 0.2')")
        print(f"  set <joint> <angle> - Set joint to specific angle")
        print(f"  move               - Execute current position")
        print(f"  show               - Show current joint values")
        print(f"  reset              - Reset to starting position")
        print(f"  done               - Return adjusted position")
        
        joint_map = {'base': 0, 'shoulder': 1, 'elbow': 2, 'wrist_p': 3, 'wrist_r': 4}
        
        while True:
            try:
                cmd = input("\nadjust> ").strip().split()
                
                if not cmd:
                    continue
                elif cmd[0] == 'done':
                    break
                elif cmd[0] == 'show':
                    for i, (joint, pos) in enumerate(zip(self.arm_joints, current_positions)):
                        print(f'  {joint}: {pos:.3f} rad ({pos*180/3.14159:.1f}°)')
                elif cmd[0] == 'move':
                    self.send_joint_goal(current_positions)
                elif cmd[0] == 'reset':
                    current_positions = base_positions.copy()
                    print("Reset to starting position")
                elif cmd[0].startswith(('+', '-', 'set')) and len(cmd) >= 3:
                    operation = cmd[0][0] if cmd[0][0] in ['+', '-'] else 'set'
                    joint_name = cmd[0][1:] if operation != 'set' else cmd[1]
                    
                    if joint_name in joint_map:
                        try:
                            amount = float(cmd[1] if operation != 'set' else cmd[2])
                            joint_idx = joint_map[joint_name]
                            
                            if operation == '+':
                                current_positions[joint_idx] += amount
                            elif operation == '-':
                                current_positions[joint_idx] -= amount
                            else:  # set
                                current_positions[joint_idx] = amount
                            
                            # Clamp to reasonable limits
                            limits = [(-1.9, 1.9), (-1.7, 1.7), (-1.7, 1.5), (-1.6, 1.6), (-2.7, 2.7)]
                            min_val, max_val = limits[joint_idx]
                            current_positions[joint_idx] = max(min_val, min(max_val, current_positions[joint_idx]))
                            
                            print(f"Updated {joint_name}: {current_positions[joint_idx]:.3f} rad ({current_positions[joint_idx]*180/3.14159:.1f}°)")
                            
                        except ValueError:
                            print("Invalid number")
                    else:
                        print("Invalid joint name. Use: base, shoulder, elbow, wrist_p, wrist_r")
                else:
                    print("Unknown command")
                    
            except (KeyboardInterrupt, EOFError):
                break
        
        return current_positions
    
    def save_positions_to_file(self):
        """Save tested positions to file"""
        try:
            with open('tested_positions.json', 'w') as f:
                json.dump(self.saved_positions, f, indent=2)
            self.get_logger().info('💾 Positions saved to tested_positions.json')
        except Exception as e:
            self.get_logger().error(f'Failed to save positions: {e}')
    
    def load_saved_positions(self):
        """Load previously saved positions"""
        try:
            with open('tested_positions.json', 'r') as f:
                self.saved_positions = json.load(f)
            self.get_logger().info(f'📂 Loaded {len(self.saved_positions)} saved positions')
        except FileNotFoundError:
            self.get_logger().info('No saved positions file found - starting fresh')
        except Exception as e:
            self.get_logger().error(f'Failed to load positions: {e}')
    
    def interactive_coordinate_debugger(self):
        """Interactive mode for testing coordinate targeting"""
        print("\n🎯 Enhanced Joint Debugger - Coordinate Targeting")
        print("=" * 50)
        print("Commands:")
        print("  test <target>      - Test reaching target position")
        print("  tune <target>      - Interactively tune position for target")
        print("  saved <target>     - Use saved position for target")
        print("  list               - List available targets")
        print("  show_saved         - Show saved positions")
        print("  home               - Move to home position")
        print("  current            - Show current joint positions")
        print("  custom <x> <y> <z> - Test custom coordinate")
        print("  quit               - Exit")
        print()
        print("Available targets:", list(self.test_positions.keys()))
        
        # Some reasonable starting positions for each target
        starting_guesses = {
            'board_center': [0.0, -0.5, 0.4, -0.3, 0.0],
            'a1': [0.4, -0.6, 0.6, -0.4, 0.0],    # Bottom-left
            'h1': [-0.4, -0.6, 0.6, -0.4, 0.0],   # Bottom-right
            'h8': [-0.3, -1.0, 0.9, -0.6, 0.0],   # Top-right
            'a8': [0.3, -1.0, 0.9, -0.6, 0.0],    # Top-left
            'e4': [0.0, -0.7, 0.7, -0.5, 0.0],    # Center square
            'safe_above': [0.0, -0.3, 0.2, -0.1, 0.0],
        }
        
        while rclpy.ok():
            try:
                cmd = input("\ncoord> ").strip().split()
                
                if not cmd:
                    continue
                elif cmd[0] == 'quit':
                    break
                elif cmd[0] == 'list':
                    print("Available targets:")
                    for name, coords in self.test_positions.items():
                        print(f"  {name}: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})")
                elif cmd[0] == 'show_saved':
                    if self.saved_positions:
                        print("Saved positions:")
                        for name, joints in self.saved_positions.items():
                            print(f"  {name}: {[f'{j:.2f}' for j in joints]}")
                    else:
                        print("No saved positions yet")
                elif cmd[0] == 'home':
                    self.send_joint_goal([0.0, 0.0, 0.0, 0.0, 0.0])
                elif cmd[0] == 'current':
                    positions = self.get_current_joint_positions()
                    print("Current joint positions:")
                    for i, (joint, pos) in enumerate(zip(self.arm_joints, positions)):
                        print(f"  {joint}: {pos:.3f} rad ({pos*180/3.14159:.1f}°)")
                elif cmd[0] == 'test' and len(cmd) == 2:
                    target = cmd[1]
                    if target in self.test_positions:
                        if target in starting_guesses:
                            guess = starting_guesses[target]
                        else:
                            guess = [0.0, -0.5, 0.3, -0.2, 0.0]  # Default guess
                        self.test_coordinate_reach(target, guess)
                    else:
                        print(f"Unknown target: {target}")
                elif cmd[0] == 'tune' and len(cmd) == 2:
                    target = cmd[1]
                    if target in self.test_positions:
                        # Start with saved position or reasonable guess
                        if target in self.saved_positions:
                            start_pos = self.saved_positions[target]
                            print(f"Starting with saved position for {target}")
                        elif target in starting_guesses:
                            start_pos = starting_guesses[target]
                            print(f"Starting with guess for {target}")
                        else:
                            start_pos = [0.0, -0.5, 0.3, -0.2, 0.0]
                            print(f"Starting with default position for {target}")
                        
                        # Move to starting position first
                        self.send_joint_goal(start_pos)
                        time.sleep(1.0)
                        
                        # Enter adjustment mode
                        tuned_pos = self.adjust_joint_positions(start_pos)
                        
                        # Test the final position
                        print(f"\nTesting final tuned position for {target}...")
                        self.test_coordinate_reach(target, tuned_pos)
                    else:
                        print(f"Unknown target: {target}")
                elif cmd[0] == 'saved' and len(cmd) == 2:
                    target = cmd[1]
                    if target in self.saved_positions:
                        self.send_joint_goal(self.saved_positions[target])
                    else:
                        print(f"No saved position for {target}")
                elif cmd[0] == 'custom' and len(cmd) == 4:
                    try:
                        x, y, z = float(cmd[1]), float(cmd[2]), float(cmd[3])
                        print(f"Custom coordinate testing not yet implemented")
                        print(f"Target: ({x:.3f}, {y:.3f}, {z:.3f})")
                        # Would need inverse kinematics for this
                    except ValueError:
                        print("Invalid coordinates")
                else:
                    print("Unknown command or wrong number of arguments")
                    
            except (KeyboardInterrupt, EOFError):
                break


def main():
    rclpy.init()
    
    try:
        debugger = EnhancedJointDebugger()
        
        # Give it a moment to receive joint states
        time.sleep(1.0)
        
        debugger.interactive_coordinate_debugger()
        
    except KeyboardInterrupt:
        print("\n👋 Interrupted")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

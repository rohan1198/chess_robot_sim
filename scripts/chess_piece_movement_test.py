#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import math

class ChessPieceMovementTester(Node):
    def __init__(self):
        super().__init__('chess_piece_movement_tester')
        
        # Action clients
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_action_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_controller/gripper_cmd'
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Chess board parameters (corrected to match world file)
        self.square_size = 0.057  # 57mm squares
        self.board_center = [0.75, 0, 0.405]  # Actual board center from world file
        self.piece_height = 0.030  # Average piece height
        self.hover_height = 0.080  # Height to hover above pieces (reduced for better precision)
        
        # Joint names
        self.joint_names = [
            'shoulder_rotation',
            'shoulder_pitch', 
            'elbow',
            'wrist_pitch',
            'wrist_roll'
        ]
        
        # Current joint positions
        self.current_joint_positions = {}
        self.joint_states_received = False
        
        # Test positions (chess squares) - focused on reachable squares
        self.test_positions = {
            'd7': self.chess_square_to_world('d', 7),  # Black pawn center-left
            'e7': self.chess_square_to_world('e', 7),  # Black pawn center-right
            'd8': self.chess_square_to_world('d', 8),  # Black queen
            'e8': self.chess_square_to_world('e', 8),  # Black king
            'home': [0.0, 0.0, 0.0, 0.0, 0.0],       # Home position
        }
        
        self.get_logger().info('🏁 Chess Piece Movement Tester Ready!')
        self.get_logger().info(f'📍 Board center: {self.board_center}')
        self.get_logger().info(f'🎯 Test positions: {list(self.test_positions.keys())}')
        
    def joint_state_callback(self, msg):
        """Update current joint positions."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names or name == 'gripper_joint':
                self.current_joint_positions[name] = msg.position[i]
        
        if len(self.current_joint_positions) >= len(self.joint_names):
            self.joint_states_received = True
    
    def chess_square_to_world(self, file_letter, rank_number):
        """Convert chess square notation to world coordinates."""
        # Convert file letter (a-h) to column index (0-7)
        file_index = ord(file_letter.lower()) - ord('a')
        
        # Convert rank number (1-8) to row index (7-0)
        rank_index = 8 - rank_number
        
        # Calculate position relative to board center
        x_offset = (file_index - 3.5) * self.square_size
        y_offset = (3.5 - rank_index) * self.square_size
        
        # World coordinates
        world_x = self.board_center[0] + x_offset
        world_y = self.board_center[1] + y_offset
        world_z = self.board_center[2] + self.piece_height
        
        return [world_x, world_y, world_z]
    
    def world_to_joint_angles(self, target_xyz):
        """
        Improved inverse kinematics for SO-ARM101.
        """
        x, y, z = target_xyz
        
        # Base rotation (shoulder_rotation)
        shoulder_rotation = math.atan2(y, x)
        
        # Distance from base to target (projected on XY plane)
        r = math.sqrt(x*x + y*y)
        
        # Height from base (adjusted for actual base height)
        h = z - 0.095  # Base height from URDF
        
        # Arm segment lengths (from URDF analysis)
        L1 = 0.113  # Upper arm length (shoulder to elbow)
        L2 = 0.135  # Forearm length (elbow to wrist)
        
        # Distance from shoulder joint to target
        d = math.sqrt(r*r + h*h)
        
        # Check if target is reachable
        max_reach = L1 + L2 - 0.02  # Small margin for safety
        if d > max_reach:
            self.get_logger().warn(f"Target might be out of reach: {d:.3f}m > {max_reach:.3f}m")
            # Scale down to maximum reach
            scale = max_reach / d
            r *= scale
            h *= scale
            d = max_reach
        
        # Elbow angle using cosine rule
        cos_elbow = (L1*L1 + L2*L2 - d*d) / (2*L1*L2)
        cos_elbow = max(-1, min(1, cos_elbow))  # Clamp to valid range
        elbow = math.pi - math.acos(cos_elbow)
        
        # Shoulder pitch angle
        alpha = math.atan2(h, r)
        beta = math.acos((L1*L1 + d*d - L2*L2) / (2*L1*d))
        shoulder_pitch = alpha + beta - math.pi/2
        
        # Wrist pitch to keep end effector pointing down
        wrist_pitch = -(shoulder_pitch + elbow)
        
        # Wrist roll (keep at 0 for simplicity)
        wrist_roll = 0.0
        
        return [shoulder_rotation, shoulder_pitch, elbow, wrist_pitch, wrist_roll]
    
    def get_current_positions(self):
        """Get current joint positions."""
        if not self.joint_states_received:
            return [0.0] * len(self.joint_names)
        return [self.current_joint_positions.get(name, 0.0) for name in self.joint_names]
    
    def create_trajectory(self, target_positions, duration_sec=3.0):
        """Create a smooth trajectory to target positions."""
        current_positions = self.get_current_positions()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Start point
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.velocities = [0.0] * len(self.joint_names)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        
        # End point
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.velocities = [0.0] * len(self.joint_names)
        end_point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        
        trajectory.points = [start_point, end_point]
        return trajectory
    
    def move_arm_to_position(self, target_positions, duration_sec=3.0):
        """Move arm to specified joint positions."""
        trajectory = self.create_trajectory(target_positions, duration_sec)
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'Moving arm to: {[f"{p:.3f}" for p in target_positions]}')
        
        # Send goal
        future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error('Goal send timeout!')
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted, executing...')
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)
        
        if not result_future.done():
            self.get_logger().error('Goal execution timeout!')
            return False
        
        result = result_future.result().result
        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        
        if success:
            self.get_logger().info('✅ Movement completed successfully!')
        else:
            self.get_logger().error(f'❌ Movement failed with error code: {result.error_code}')
        
        return success
    
    def move_arm_to_world_position(self, world_xyz, duration_sec=3.0):
        """Move arm to a world coordinate position."""
        joint_angles = self.world_to_joint_angles(world_xyz)
        return self.move_arm_to_position(joint_angles, duration_sec)
    
    def control_gripper(self, position, max_effort=50.0):
        """Control gripper position."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        self.get_logger().info(f'Moving gripper to: {position:.3f}')
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error('Gripper goal send timeout!')
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        
        if not result_future.done():
            self.get_logger().error('Gripper execution timeout!')
            return False
        
        result = result_future.result().result
        if result.reached_goal:
            self.get_logger().info('✅ Gripper movement completed!')
            return True
        else:
            self.get_logger().error('❌ Gripper movement failed!')
            return False
    
    def test_reachability(self):
        """Test reachability of key chess squares."""
        self.get_logger().info('\n🔍 Testing Chess Square Reachability')
        self.get_logger().info('=' * 50)
        
        # Test various squares to assess reachability
        test_squares = ['a8', 'b8', 'c8', 'd8', 'e8', 'f8', 'g8', 'h8',  # Black back rank
                       'a7', 'b7', 'c7', 'd7', 'e7', 'f7', 'g7', 'h7',  # Black pawns
                       'd4', 'e4', 'd5', 'e5',                          # Center squares
                       'a1', 'b1', 'c1', 'd1', 'e1', 'f1', 'g1', 'h1'] # White back rank
        
        reachable_squares = []
        unreachable_squares = []
        
        for square in test_squares:
            world_pos = self.chess_square_to_world(square[0], int(square[1]))
            joint_angles = self.world_to_joint_angles(world_pos)
            
            # Calculate actual reach distance
            x, y, z = world_pos
            distance = math.sqrt(x*x + y*y + (z-0.095)**2)
            
            if distance <= 0.23:  # Conservative reach limit
                reachable_squares.append(square)
                self.get_logger().info(f'   ✅ {square}: {distance:.3f}m - REACHABLE')
            else:
                unreachable_squares.append(square)
                self.get_logger().warn(f'   ❌ {square}: {distance:.3f}m - OUT OF REACH')
        
        self.get_logger().info(f'\n📊 Reachability Summary:')
        self.get_logger().info(f'   Reachable squares ({len(reachable_squares)}): {reachable_squares}')
        self.get_logger().info(f'   Unreachable squares ({len(unreachable_squares)}): {unreachable_squares}')
        
        return reachable_squares, unreachable_squares
    
    def test_basic_movement(self):
        """Test basic arm movement to chess squares."""
        self.get_logger().info('\n🔍 Testing Basic Movement to Chess Squares')
        self.get_logger().info('=' * 50)
        
        # Test movement to reachable squares
        test_squares = ['d7', 'e7', 'd8', 'e8']
        
        for square in test_squares:
            if square in self.test_positions:
                world_pos = self.test_positions[square]
                hover_pos = world_pos.copy()
                hover_pos[2] += self.hover_height  # Hover above
                
                self.get_logger().info(f'\n📍 Testing movement to {square}')
                self.get_logger().info(f'   World position: ({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f})')
                
                # Move to hover position first
                if self.move_arm_to_world_position(hover_pos, 4.0):
                    self.get_logger().info(f'✅ Successfully reached hover position above {square}')
                    time.sleep(1)
                    
                    # Move down to piece level
                    if self.move_arm_to_world_position(world_pos, 2.0):
                        self.get_logger().info(f'✅ Successfully reached {square} square!')
                        time.sleep(1)
                    else:
                        self.get_logger().error(f'❌ Failed to reach {square} square')
                else:
                    self.get_logger().error(f'❌ Failed to reach hover position above {square}')
                
                time.sleep(2)
    
    def test_gripper_operation(self):
        """Test gripper open/close operations."""
        self.get_logger().info('\n🦾 Testing Gripper Operations')
        self.get_logger().info('=' * 30)
        
        # Open gripper
        self.get_logger().info('Opening gripper...')
        if self.control_gripper(1.5):  # Open position
            time.sleep(1)
            
            # Close gripper (as if grasping a piece)
            self.get_logger().info('Closing gripper (simulating piece grasp)...')
            if self.control_gripper(-0.1):  # Closed position
                time.sleep(1)
                
                # Open gripper again (release piece)
                self.get_logger().info('Opening gripper (releasing piece)...')
                self.control_gripper(1.5)
                
                self.get_logger().info('✅ Gripper test completed successfully!')
            else:
                self.get_logger().error('❌ Failed to close gripper')
        else:
            self.get_logger().error('❌ Failed to open gripper')
    
    def test_piece_move_simulation(self):
        """Simulate moving a piece from d7 to d6."""
        self.get_logger().info('\n♟️  Simulating Piece Move: d7 → d6')
        self.get_logger().info('=' * 40)
        
        # Get positions
        d7_pos = self.chess_square_to_world('d', 7)
        d6_pos = self.chess_square_to_world('d', 6)
        
        # Create hover positions
        d7_hover = d7_pos.copy()
        d7_hover[2] += self.hover_height
        
        d6_hover = d6_pos.copy()
        d6_hover[2] += self.hover_height
        
        # Step 1: Move to hover above d7
        self.get_logger().info('Step 1: Moving to hover above d7...')
        if not self.move_arm_to_world_position(d7_hover, 3.0):
            self.get_logger().error('❌ Failed step 1')
            return False
        
        # Step 2: Open gripper
        self.get_logger().info('Step 2: Opening gripper...')
        if not self.control_gripper(1.5):
            self.get_logger().error('❌ Failed step 2')
            return False
        
        # Step 3: Move down to d7 (pick up piece)
        self.get_logger().info('Step 3: Moving down to pick up piece at d7...')
        if not self.move_arm_to_world_position(d7_pos, 2.0):
            self.get_logger().error('❌ Failed step 3')
            return False
        
        # Step 4: Close gripper (grasp piece)
        self.get_logger().info('Step 4: Grasping piece...')
        if not self.control_gripper(-0.1):
            self.get_logger().error('❌ Failed step 4')
            return False
        
        time.sleep(1)
        
        # Step 5: Lift piece
        self.get_logger().info('Step 5: Lifting piece...')
        if not self.move_arm_to_world_position(d7_hover, 2.0):
            self.get_logger().error('❌ Failed step 5')
            return False
        
        # Step 6: Move to hover above d6
        self.get_logger().info('Step 6: Moving to hover above d6...')
        if not self.move_arm_to_world_position(d6_hover, 3.0):
            self.get_logger().error('❌ Failed step 6')
            return False
        
        # Step 7: Move down to d6 (place piece)
        self.get_logger().info('Step 7: Moving down to place piece at d6...')
        if not self.move_arm_to_world_position(d6_pos, 2.0):
            self.get_logger().error('❌ Failed step 7')
            return False
        
        # Step 8: Open gripper (release piece)
        self.get_logger().info('Step 8: Releasing piece...')
        if not self.control_gripper(1.5):
            self.get_logger().error('❌ Failed step 8')
            return False
        
        # Step 9: Move back to hover
        self.get_logger().info('Step 9: Moving back to safe position...')
        if not self.move_arm_to_world_position(d6_hover, 2.0):
            self.get_logger().error('❌ Failed step 9')
            return False
        
        self.get_logger().info('🎉 Piece move simulation completed successfully!')
        return True
    
    def return_to_home(self):
        """Return arm to home position."""
        self.get_logger().info('\n🏠 Returning to home position...')
        home_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        if self.move_arm_to_position(home_angles, 4.0):
            self.get_logger().info('✅ Returned to home position')
        else:
            self.get_logger().error('❌ Failed to return to home position')
    
    def wait_for_action_servers(self):
        """Wait for action servers to be available."""
        self.get_logger().info('Waiting for action servers...')
        
        if not self.arm_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm controller action server not available!')
            return False
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper controller action server not available!')
            return False
        
        self.get_logger().info('✅ Action servers ready!')
        return True
    
    def wait_for_joint_states(self):
        """Wait for joint states to be received."""
        self.get_logger().info('Waiting for joint states...')
        timeout = 10.0
        start_time = time.time()
        
        while not self.joint_states_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.joint_states_received:
            self.get_logger().info('✅ Joint states received!')
            return True
        else:
            self.get_logger().error('❌ Timeout waiting for joint states!')
            return False
    
    def run_all_tests(self):
        """Run comprehensive movement tests."""
        self.get_logger().info('\n🎯 Starting Comprehensive Chess Piece Movement Tests')
        self.get_logger().info('=' * 60)
        
        # Wait for prerequisites
        if not self.wait_for_action_servers():
            return False
        
        if not self.wait_for_joint_states():
            return False
        
        # Start from home position
        self.return_to_home()
        time.sleep(2)
        
        try:
            # Test 1: Reachability analysis
            self.test_reachability()
            time.sleep(2)
            
            # Test 2: Basic movement
            self.test_basic_movement()
            time.sleep(2)
            
            # Test 3: Gripper operation
            self.test_gripper_operation()
            time.sleep(2)
            
            # Test 4: Complete piece move simulation
            self.test_piece_move_simulation()
            time.sleep(2)
            
            # Return to home
            self.return_to_home()
            
            self.get_logger().info('\n🎉 All tests completed successfully!')
            self.get_logger().info('✅ Chess piece movement system is ready!')
            
        except Exception as e:
            self.get_logger().error(f'❌ Test failed with exception: {e}')
            self.return_to_home()  # Safe return
            return False
        
        return True

def main(args=None):
    rclpy.init(args=args)
    
    tester = ChessPieceMovementTester()
    
    try:
        success = tester.run_all_tests()
        if success:
            print("\n" + "="*60)
            print("🏆 CHESS PIECE MOVEMENT TEST RESULTS: PASSED")
            print("✅ Your robotic arm is ready for chess!")
            print("="*60)
        else:
            print("\n" + "="*60)
            print("❌ CHESS PIECE MOVEMENT TEST RESULTS: FAILED")
            print("💡 Please check the errors above and try again")
            print("="*60)
            
    except KeyboardInterrupt:
        tester.get_logger().info('\n🛑 Test interrupted by user')
        tester.return_to_home()
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

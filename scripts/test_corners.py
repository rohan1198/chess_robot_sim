#!/usr/bin/env python3
"""
Chess Corner Movement Tester - Phase 3
=======================================

ROS2 node for executing validated chess corner movements in Gazebo simulation.
Loads solutions from Phase 2 and executes safe trajectory sequences.

Usage:
    ros2 run chess_robot_sim test_corners.py
    # OR
    python3 test_corners.py

Features:
- Safe movement sequences (home → approach → hover → return)
- Real-time position validation
- Interactive corner selection
- Integration with existing controllers
- Comprehensive error handling

Author: Chess Robot Project
Phase: 3 - ROS2 Trajectory Execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
import json
import numpy as np
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

class ChessCornerTester(Node):
    """ROS2 node for testing chess corner movements."""
    
    def __init__(self):
        super().__init__('chess_corner_tester')
        
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
        
        # Robot configuration
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
        
        # Corner solutions (loaded from Phase 2)
        self.corner_solutions = {}
        self.solution_metadata = {}
        
        # Home position
        self.home_position = [0.0] * len(self.joint_names)
        
        # Movement parameters
        self.approach_height_offset = 0.05  # 50mm above hover position
        self.movement_speed_factor = 0.8    # Conservative speed
        
        self.get_logger().info('♟️  Chess Corner Tester initialized!')
        self.get_logger().info('📂 Loading corner solutions from Phase 2...')
        
        # Load corner solutions
        if self.load_corner_solutions():
            self.get_logger().info(f'✅ Loaded {len(self.corner_solutions)} corner solutions')
        else:
            self.get_logger().error('❌ Failed to load corner solutions')
    
    def load_corner_solutions(self) -> bool:
        """Load corner solutions from Phase 2 JSON file."""
        
        script_dir = Path(__file__).parent.absolute()
        solution_file = script_dir / "chess_corner_complete_solution.json"
        
        if not solution_file.exists():
            self.get_logger().error(f'Solution file not found: {solution_file}')
            return False
        
        try:
            with open(solution_file, 'r') as f:
                data = json.load(f)
            
            self.solution_metadata = data.get('metadata', {})
            self.corner_solutions = data.get('ik_solutions', {})
            
            # Log loaded solutions
            self.get_logger().info(f'📊 Phase 2 Results:')
            self.get_logger().info(f'   Success Rate: {self.solution_metadata.get("success_rate", 0)*100:.0f}%')
            self.get_logger().info(f'   Solutions: {list(self.corner_solutions.keys())}')
            
            # Convert solutions to proper format
            for corner_name, solution_data in self.corner_solutions.items():
                joints = solution_data['joints']
                position = solution_data['position']
                method = solution_data['method']
                
                self.get_logger().info(f'   {corner_name}: {method} method, {len(joints)} joints')
            
            return len(self.corner_solutions) > 0
            
        except Exception as e:
            self.get_logger().error(f'Error loading solutions: {e}')
            return False
    
    def joint_state_callback(self, msg):
        """Update current joint positions from joint states."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_positions[name] = msg.position[i]
        
        if len(self.current_joint_positions) >= len(self.joint_names):
            self.joint_states_received = True
    
    def get_current_positions(self) -> List[float]:
        """Get current joint positions in correct order."""
        if not self.joint_states_received:
            self.get_logger().warn("Joint states not received yet, using home position")
            return self.home_position.copy()
        
        return [self.current_joint_positions.get(name, 0.0) for name in self.joint_names]
    
    def wait_for_connections(self, timeout=10.0) -> bool:
        """Wait for action servers and joint states."""
        
        self.get_logger().info('🔌 Waiting for connections...')
        
        # Wait for action servers
        self.get_logger().info('   Arm controller...')
        if not self.arm_action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('❌ Arm controller timeout')
            return False
        
        self.get_logger().info('   Gripper controller...')
        if not self.gripper_action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('❌ Gripper controller timeout')
            return False
        
        # Wait for joint states
        self.get_logger().info('   Joint states...')
        start_time = time.time()
        while not self.joint_states_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.joint_states_received:
            self.get_logger().error('❌ Joint states timeout')
            return False
        
        self.get_logger().info('✅ All connections established!')
        return True
    
    def calculate_trajectory_duration(self, current_pos: List[float], target_pos: List[float], 
                                    min_duration: float = 3.0) -> float:
        """Calculate appropriate trajectory duration based on movement distance."""
        
        max_distance = max(abs(target_pos[i] - current_pos[i]) for i in range(len(current_pos)))
        
        # Conservative duration: 2.5 seconds per radian of movement
        calculated_duration = max_distance * 2.5 * self.movement_speed_factor
        
        # Apply bounds
        duration = max(min_duration, calculated_duration)
        duration = min(duration, 15.0)  # Max 15 seconds
        
        return duration
    
    def create_trajectory(self, target_positions: List[float], duration_sec: float) -> JointTrajectory:
        """Create a smooth trajectory to target positions."""
        
        current_positions = self.get_current_positions()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Simple 2-point trajectory for reliability
        points = []
        
        # Start point (current position)
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.velocities = [0.0] * len(self.joint_names)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        points.append(start_point)
        
        # End point (target position)
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.velocities = [0.0] * len(self.joint_names)
        end_point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        points.append(end_point)
        
        trajectory.points = points
        return trajectory
    
    def execute_trajectory(self, target_positions: List[float], description: str = "") -> bool:
        """Execute a trajectory and wait for completion."""
        
        current_positions = self.get_current_positions()
        duration = self.calculate_trajectory_duration(current_positions, target_positions)
        
        trajectory = self.create_trajectory(target_positions, duration)
        
        # Create and send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        max_distance = max(abs(target_positions[i] - current_positions[i]) for i in range(len(current_positions)))
        
        self.get_logger().info(f'🎯 {description}')
        self.get_logger().info(f'   Duration: {duration:.1f}s, Max joint movement: {math.degrees(max_distance):.1f}°')
        
        # Send goal
        future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('❌ Goal send timeout')
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            return False
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        wait_timeout = duration + 10.0
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=wait_timeout)
        
        if not result_future.done():
            self.get_logger().error('❌ Execution timeout')
            return False
        
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ Movement completed successfully!')
            return True
        else:
            self.get_logger().error(f'❌ Movement failed with error code: {result.error_code}')
            return False
    
    def move_to_home(self) -> bool:
        """Move robot to home position."""
        return self.execute_trajectory(self.home_position, "Moving to home position")
    
    def generate_approach_position(self, hover_position: List[float]) -> List[float]:
        """Generate a safe approach position above the hover position."""
        
        # For chess corners, we'll modify the shoulder_pitch and elbow to create
        # a higher approach position while keeping the same X-Y location
        approach_pos = hover_position.copy()
        
        # Conservative approach: raise the arm by adjusting shoulder and elbow
        # This creates a position ~5cm higher than hover
        approach_pos[1] -= 0.2  # Shoulder pitch up (negative = up)
        approach_pos[2] += 0.15  # Elbow slightly more extended
        
        return approach_pos
    
    def move_to_corner_sequence(self, corner_name: str) -> bool:
        """Execute complete movement sequence to a corner."""
        
        if corner_name not in self.corner_solutions:
            self.get_logger().error(f'❌ No solution for corner: {corner_name}')
            return False
        
        solution_data = self.corner_solutions[corner_name]
        hover_joints = solution_data['joints']
        target_position = solution_data['position']
        method = solution_data['method']
        
        self.get_logger().info(f'🎯 MOVING TO {corner_name.upper()} CORNER')
        self.get_logger().info(f'   Method: {method}')
        self.get_logger().info(f'   Target: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})')
        
        # Step 1: Move to home position
        self.get_logger().info('📍 Step 1: Moving to home position...')
        if not self.move_to_home():
            return False
        time.sleep(1)
        
        # Step 2: Move to approach position (safe high position)
        self.get_logger().info('📍 Step 2: Moving to approach position...')
        approach_joints = self.generate_approach_position(hover_joints)
        if not self.execute_trajectory(approach_joints, "Moving to approach position"):
            self.get_logger().warn('⚠️  Approach position failed, proceeding directly to hover')
        else:
            time.sleep(1)
        
        # Step 3: Move to hover position (main target)
        self.get_logger().info('📍 Step 3: Moving to hover position...')
        if not self.execute_trajectory(hover_joints, f"Moving to {corner_name} hover position"):
            self.get_logger().error(f'❌ Failed to reach {corner_name} hover position')
            return False
        
        # Step 4: Hold position briefly
        self.get_logger().info('📍 Step 4: Holding position...')
        time.sleep(2)
        
        # Step 5: Return to approach (if it worked before)
        self.get_logger().info('📍 Step 5: Returning to approach position...')
        if not self.execute_trajectory(approach_joints, "Returning to approach position"):
            self.get_logger().warn('⚠️  Return to approach failed, going directly home')
        else:
            time.sleep(1)
        
        # Step 6: Return to home
        self.get_logger().info('📍 Step 6: Returning to home position...')
        if not self.move_to_home():
            self.get_logger().error('❌ Failed to return home')
            return False
        
        self.get_logger().info(f'🎉 {corner_name.upper()} corner sequence completed successfully!')
        return True
    
    def test_all_corners(self) -> bool:
        """Test movement to all available corners."""
        
        self.get_logger().info('🏁 TESTING ALL CORNERS')
        self.get_logger().info('=' * 50)
        
        successful_corners = 0
        total_corners = len(self.corner_solutions)
        
        # Test each corner
        for corner_name in ['front_left', 'front_right', 'back_left', 'back_right']:
            if corner_name in self.corner_solutions:
                self.get_logger().info(f'\n🎯 Testing {corner_name}...')
                
                if self.move_to_corner_sequence(corner_name):
                    successful_corners += 1
                    self.get_logger().info(f'✅ {corner_name} - SUCCESS')
                else:
                    self.get_logger().error(f'❌ {corner_name} - FAILED')
                
                # Pause between corners
                time.sleep(2)
            else:
                self.get_logger().info(f'⏭️  Skipping {corner_name} (no solution available)')
        
        # Final results
        success_rate = successful_corners / total_corners * 100 if total_corners > 0 else 0
        
        self.get_logger().info(f'\n🏆 ALL CORNERS TEST COMPLETE')
        self.get_logger().info(f'   Successful: {successful_corners}/{total_corners} ({success_rate:.0f}%)')
        
        if success_rate >= 100:
            self.get_logger().info('🎉 PERFECT! All corners reached successfully!')
        elif success_rate >= 75:
            self.get_logger().info('✅ EXCELLENT! Chess robot workspace validated!')
        elif success_rate >= 50:
            self.get_logger().info('👍 GOOD! Most corners reachable!')
        else:
            self.get_logger().info('⚠️  NEEDS WORK! Check robot configuration!')
        
        return success_rate >= 75
    
    def interactive_menu(self):
        """Interactive menu for testing individual corners."""
        
        while rclpy.ok():
            # Print available corners
            available_corners = list(self.corner_solutions.keys())
            
            print("\n" + "="*60)
            print("           🤖 Chess Corner Tester - Phase 3")
            print("="*60)
            print(f"📊 Available corners: {len(available_corners)}")
            
            for i, corner in enumerate(available_corners):
                solution = self.corner_solutions[corner]
                method = solution['method']
                position = solution['position']
                print(f"  {i+1}: {corner:<12s} ({method:11s}) - pos: ({position[0]:5.3f}, {position[1]:6.3f}, {position[2]:5.3f})")
            
            print(f"\n🎯 Options:")
            print(f"  a: Test ALL corners (complete sequence)")
            print(f"  h: Move to HOME position")
            print(f"  1-{len(available_corners)}: Test specific corner")
            print(f"  q: Quit")
            print("="*60)
            
            try:
                choice = input("Enter your choice: ").strip().lower()
                
                if choice == 'q':
                    print("👋 Goodbye!")
                    break
                elif choice == 'h':
                    print("🏠 Moving to home position...")
                    self.move_to_home()
                elif choice == 'a':
                    print("🏁 Testing ALL corners...")
                    self.test_all_corners()
                elif choice.isdigit():
                    corner_index = int(choice) - 1
                    if 0 <= corner_index < len(available_corners):
                        corner_name = available_corners[corner_index]
                        print(f"🎯 Testing {corner_name}...")
                        self.move_to_corner_sequence(corner_name)
                    else:
                        print("❌ Invalid corner number!")
                else:
                    print("❌ Invalid choice! Please try again.")
                    
            except KeyboardInterrupt:
                print("\n👋 Exiting...")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

def main(args=None):
    """Main function."""
    
    rclpy.init(args=args)
    
    print("🚀 CHESS CORNER TESTER - PHASE 3")
    print("=" * 60)
    print("ROS2 Trajectory Execution for Chess Corner Movement")
    print("Loading validated solutions from Phase 2...")
    
    node = ChessCornerTester()
    
    try:
        # Wait for connections
        if not node.wait_for_connections():
            print("❌ Failed to establish connections")
            return
        
        # Move to home initially
        print("🏠 Moving to initial home position...")
        node.move_to_home()
        
        # Start interactive menu
        print("✅ Ready! Starting interactive menu...")
        node.interactive_menu()
        
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Return to home before exiting
        print("🏠 Returning to home position before exit...")
        try:
            node.move_to_home()
        except Exception:
            pass
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

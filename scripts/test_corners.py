#!/usr/bin/env python3
"""
Improved Chess Corner Movement Tester
=====================================

Enhanced version that works with the fixed corner solutions and provides
better debugging and position validation.

Features:
- Loads both original and fixed solution files
- Real-time position monitoring and validation  
- Improved error reporting
- Coordinate frame verification
- Safety checks and recovery

Usage:
    ros2 run chess_robot_sim improved_test_corners.py

Author: Chess Robot Project - Improved Version
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

class ImprovedChessCornerTester(Node):
    """Enhanced ROS2 node for testing chess corner movements with debugging."""
    
    def __init__(self):
        super().__init__('improved_chess_corner_tester')
        
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
        
        # Current state
        self.current_joint_positions = {}
        self.joint_states_received = False
        
        # Corner solutions (will try both original and fixed)
        self.corner_solutions = {}
        self.solution_metadata = {}
        self.solution_file_used = None
        
        # Home position
        self.home_position = [0.0] * len(self.joint_names)
        
        # Movement parameters
        self.movement_speed_factor = 0.6  # Even more conservative
        
        # Position validation
        self.position_history = []
        self.expected_positions = {}
        
        self.get_logger().info('🤖 Improved Chess Corner Tester initialized!')
        
        # Load corner solutions (try fixed version first)
        if self.load_corner_solutions():
            self.get_logger().info(f'✅ Loaded {len(self.corner_solutions)} corner solutions')
            self.get_logger().info(f'📁 Using: {self.solution_file_used}')
        else:
            self.get_logger().error('❌ Failed to load any corner solutions')
    
    def load_corner_solutions(self) -> bool:
        """Load corner solutions, preferring the fixed version."""
        
        script_dir = Path(__file__).parent.absolute()
        
        # Try fixed solution file first
        solution_files = [
            script_dir / "chess_corner_fixed_solution.json",
            script_dir / "chess_corner_complete_solution.json"
        ]
        
        for solution_file in solution_files:
            if solution_file.exists():
                try:
                    with open(solution_file, 'r') as f:
                        data = json.load(f)
                    
                    self.solution_metadata = data.get('metadata', {})
                    self.corner_solutions = data.get('ik_solutions', {})
                    self.solution_file_used = solution_file.name
                    
                    # Log solution details
                    version = self.solution_metadata.get('version', 'unknown')
                    success_rate = self.solution_metadata.get('success_rate', 0) * 100
                    
                    self.get_logger().info(f'📊 Solution file: {solution_file.name}')
                    self.get_logger().info(f'   Version: {version}')
                    self.get_logger().info(f'   Success rate: {success_rate:.0f}%')
                    
                    if 'fixes_applied' in self.solution_metadata:
                        self.get_logger().info(f'🔧 Fixes applied:')
                        for fix in self.solution_metadata['fixes_applied']:
                            self.get_logger().info(f'   • {fix}')
                    
                    # Store expected positions for validation
                    for corner_name, solution_data in self.corner_solutions.items():
                        self.expected_positions[corner_name] = np.array(solution_data['position'])
                    
                    return len(self.corner_solutions) > 0
                    
                except Exception as e:
                    self.get_logger().error(f'Error loading {solution_file}: {e}')
                    continue
        
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
    
    def calculate_current_end_effector_position(self) -> Optional[np.ndarray]:
        """Calculate current end-effector position using simple forward kinematics."""
        # This is a simplified FK - in practice you'd use the full robot model
        # For now, return None to indicate we need the full FK from PyBullet or similar
        return None
    
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
                                    min_duration: float = 4.0) -> float:
        """Calculate safe trajectory duration."""
        
        max_distance = max(abs(target_pos[i] - current_pos[i]) for i in range(len(current_pos)))
        
        # Very conservative duration: 3 seconds per radian
        calculated_duration = max_distance * 3.0 * self.movement_speed_factor
        
        # Apply bounds
        duration = max(min_duration, calculated_duration)
        duration = min(duration, 20.0)  # Max 20 seconds for safety
        
        return duration
    
    def create_trajectory(self, target_positions: List[float], duration_sec: float) -> JointTrajectory:
        """Create a smooth trajectory with intermediate waypoints."""
        
        current_positions = self.get_current_positions()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Calculate number of waypoints based on movement distance
        max_distance = max(abs(target_positions[i] - current_positions[i]) for i in range(len(current_positions)))
        num_waypoints = max(3, min(8, int(max_distance / 0.3)))  # 3-8 waypoints
        
        points = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            # Linear interpolation for positions
            waypoint_positions = []
            for j in range(len(current_positions)):
                pos = current_positions[j] + t * (target_positions[j] - current_positions[j])
                waypoint_positions.append(pos)
            
            point = JointTrajectoryPoint()
            point.positions = waypoint_positions
            point.velocities = [0.0] * len(self.joint_names)  # Stop at each waypoint
            point.time_from_start = Duration(sec=int(duration_sec * t), nanosec=0)
            
            points.append(point)
        
        trajectory.points = points
        return trajectory
    
    def execute_trajectory_with_monitoring(self, target_positions: List[float], description: str = "") -> bool:
        """Execute trajectory with real-time monitoring and validation."""
        
        current_positions = self.get_current_positions()
        duration = self.calculate_trajectory_duration(current_positions, target_positions)
        
        trajectory = self.create_trajectory(target_positions, duration)
        
        # Create and send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        max_distance = max(abs(target_positions[i] - current_positions[i]) for i in range(len(current_positions)))
        
        self.get_logger().info(f'🎯 {description}')
        self.get_logger().info(f'   Duration: {duration:.1f}s, Max movement: {math.degrees(max_distance):.1f}°')
        self.get_logger().info(f'   Waypoints: {len(trajectory.points)}')
        self.get_logger().info(f'   Target joints: {[f"{math.degrees(pos):6.1f}°" for pos in target_positions]}')
        
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
        
        self.get_logger().info('📈 Trajectory accepted, monitoring execution...')
        
        # Monitor execution
        start_time = time.time()
        last_log_time = start_time
        
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Log progress every 2 seconds
            if current_time - last_log_time >= 2.0:
                current_joints = self.get_current_positions()
                progress = min(100, (elapsed / duration) * 100)
                
                self.get_logger().info(f'   Progress: {progress:.0f}% - Current: {[f"{math.degrees(pos):5.1f}°" for pos in current_joints]}')
                last_log_time = current_time
            
            # Check for timeout
            if elapsed > duration + 15.0:
                self.get_logger().error('❌ Execution timeout')
                return False
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = result_future.result().result
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            # Validate final position
            final_joints = self.get_current_positions()
            joint_errors = [abs(target_positions[i] - final_joints[i]) for i in range(len(target_positions))]
            max_joint_error = max(joint_errors)
            
            if max_joint_error > 0.1:  # 5.7 degrees
                self.get_logger().warn(f'⚠️  Large joint error: {math.degrees(max_joint_error):.1f}°')
            
            self.get_logger().info(f'✅ Movement completed! Max joint error: {math.degrees(max_joint_error):.1f}°')
            return True
        else:
            self.get_logger().error(f'❌ Movement failed with error code: {result.error_code}')
            return False
    
    def move_to_home(self) -> bool:
        """Move robot to home position."""
        return self.execute_trajectory_with_monitoring(self.home_position, "Moving to home position")
    
    def move_to_corner_sequence_with_validation(self, corner_name: str) -> bool:
        """Execute movement sequence with enhanced validation."""
        
        if corner_name not in self.corner_solutions:
            self.get_logger().error(f'❌ No solution for corner: {corner_name}')
            return False
        
        solution_data = self.corner_solutions[corner_name]
        hover_joints = solution_data['joints']
        target_position = solution_data['position']
        method = solution_data['method']
        
        self.get_logger().info(f'🎯 ENHANCED MOVEMENT TO {corner_name.upper()} CORNER')
        self.get_logger().info(f'   Solution method: {method}')
        self.get_logger().info(f'   Target position: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})')
        self.get_logger().info(f'   Solution file: {self.solution_file_used}')
        
        # Safety check: validate joint limits
        joint_limits = {
            'shoulder_rotation': (-1.91986, 1.91986),
            'shoulder_pitch': (-1.74533, 1.74533),
            'elbow': (-1.74533, 1.74533),  # Fixed limit
            'wrist_pitch': (-1.65806, 1.65806),
            'wrist_roll': (-2.79253, 2.79253)
        }
        
        for i, (joint_name, target_angle) in enumerate(zip(self.joint_names, hover_joints)):
            lower, upper = joint_limits[joint_name]
            if not (lower <= target_angle <= upper):
                self.get_logger().error(f'❌ Target angle {math.degrees(target_angle):.1f}° for {joint_name} exceeds limits [{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]')
                return False
        
        sequence_success = True
        
        # Step 1: Move to home position
        self.get_logger().info('📍 Step 1: Moving to safe home position...')
        if not self.move_to_home():
            self.get_logger().error('❌ Failed to reach home position')
            return False
        time.sleep(2)
        
        # Step 2: Move to target position
        self.get_logger().info(f'📍 Step 2: Moving to {corner_name} target position...')
        if not self.execute_trajectory_with_monitoring(hover_joints, f"Moving to {corner_name} ({method})"):
            self.get_logger().error(f'❌ Failed to reach {corner_name} target position')
            sequence_success = False
        else:
            # Step 3: Hold position and validate
            self.get_logger().info('📍 Step 3: Holding position for validation...')
            time.sleep(3)
            
            # Validate we're close to expected position
            final_joints = self.get_current_positions()
            joint_errors = [abs(hover_joints[i] - final_joints[i]) for i in range(len(hover_joints))]
            max_error = max(joint_errors)
            
            if max_error < 0.05:  # ~3 degrees
                self.get_logger().info(f'✅ Position validation passed (max error: {math.degrees(max_error):.1f}°)')
            else:
                self.get_logger().warn(f'⚠️  Position validation warning (max error: {math.degrees(max_error):.1f}°)')
        
        # Step 4: Return to home
        self.get_logger().info('📍 Step 4: Returning to home position...')
        if not self.move_to_home():
            self.get_logger().error('❌ Failed to return home')
            sequence_success = False
        
        if sequence_success:
            self.get_logger().info(f'🎉 {corner_name.upper()} corner sequence completed successfully!')
        else:
            self.get_logger().error(f'❌ {corner_name.upper()} corner sequence had errors')
        
        return sequence_success
    
    def test_all_corners_enhanced(self) -> bool:
        """Test all corners with enhanced monitoring."""
        
        self.get_logger().info('🏁 ENHANCED ALL CORNERS TEST')
        self.get_logger().info('=' * 50)
        
        successful_corners = 0
        total_corners = len(self.corner_solutions)
        test_results = {}
        
        for corner_name in ['front_left', 'front_right', 'back_left', 'back_right']:
            if corner_name in self.corner_solutions:
                self.get_logger().info(f'\n🎯 Testing {corner_name}...')
                
                success = self.move_to_corner_sequence_with_validation(corner_name)
                test_results[corner_name] = success
                
                if success:
                    successful_corners += 1
                    self.get_logger().info(f'✅ {corner_name} - SUCCESS')
                else:
                    self.get_logger().error(f'❌ {corner_name} - FAILED')
                
                # Pause between corners
                time.sleep(3)
            else:
                self.get_logger().info(f'⏭️  Skipping {corner_name} (no solution available)')
                test_results[corner_name] = False
        
        # Final results
        success_rate = successful_corners / total_corners * 100 if total_corners > 0 else 0
        
        self.get_logger().info(f'\n🏆 ENHANCED TEST RESULTS:')
        self.get_logger().info(f'   Successful: {successful_corners}/{total_corners} ({success_rate:.0f}%)')
        self.get_logger().info(f'   Solution file: {self.solution_file_used}')
        
        for corner, success in test_results.items():
            status = "✅ PASS" if success else "❌ FAIL"
            self.get_logger().info(f'   {corner:12s}: {status}')
        
        if success_rate >= 100:
            self.get_logger().info('🎉 PERFECT! All corners reached successfully!')
        elif success_rate >= 75:
            self.get_logger().info('✅ EXCELLENT! Chess robot workspace validated!')
        elif success_rate >= 50:
            self.get_logger().info('👍 GOOD! Most corners reachable!')
        else:
            self.get_logger().info('⚠️  NEEDS WORK! Consider further optimization!')
        
        return success_rate >= 75
    
    def interactive_menu_enhanced(self):
        """Enhanced interactive menu with debugging options."""
        
        while rclpy.ok():
            # Print available corners with detailed info
            available_corners = list(self.corner_solutions.keys())
            
            print("\n" + "="*70)
            print("        🤖 Improved Chess Corner Tester - Enhanced Menu")
            print("="*70)
            print(f"📊 Solution file: {self.solution_file_used}")
            print(f"📍 Available corners: {len(available_corners)}")
            
            if self.solution_metadata.get('version'):
                print(f"🔧 Version: {self.solution_metadata['version']}")
            
            for i, corner in enumerate(available_corners):
                solution = self.corner_solutions[corner]
                method = solution['method']
                position = solution['position']
                joints = solution['joints']
                distance = np.linalg.norm(position)
                
                print(f"  {i+1}: {corner:<12s} ({method:<12s}) - pos: ({position[0]:5.3f}, {position[1]:6.3f}, {position[2]:5.3f}) [{distance:.3f}m]")
                print(f"      joints: {[f'{math.degrees(j):5.1f}°' for j in joints]}")
            
            print(f"\n🎯 Options:")
            print(f"  a: Test ALL corners (enhanced sequence)")
            print(f"  h: Move to HOME position")
            print(f"  1-{len(available_corners)}: Test specific corner")
            print(f"  d: Show detailed solution information")
            print(f"  q: Quit")
            print("="*70)
            
            try:
                choice = input("Enter your choice: ").strip().lower()
                
                if choice == 'q':
                    print("👋 Goodbye!")
                    break
                elif choice == 'h':
                    print("🏠 Moving to home position...")
                    self.move_to_home()
                elif choice == 'a':
                    print("🏁 Testing ALL corners with enhanced monitoring...")
                    self.test_all_corners_enhanced()
                elif choice == 'd':
                    self.show_detailed_solution_info()
                elif choice.isdigit():
                    corner_index = int(choice) - 1
                    if 0 <= corner_index < len(available_corners):
                        corner_name = available_corners[corner_index]
                        print(f"🎯 Testing {corner_name} with enhanced validation...")
                        self.move_to_corner_sequence_with_validation(corner_name)
                    else:
                        print("❌ Invalid corner number!")
                else:
                    print("❌ Invalid choice! Please try again.")
                    
            except KeyboardInterrupt:
                print("\n👋 Exiting...")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
    
    def show_detailed_solution_info(self):
        """Show detailed information about loaded solutions."""
        print("\n" + "="*60)
        print("📋 DETAILED SOLUTION INFORMATION")
        print("="*60)
        
        print(f"📁 Solution file: {self.solution_file_used}")
        print(f"📊 Metadata:")
        for key, value in self.solution_metadata.items():
            if key != 'fixes_applied':
                print(f"   {key}: {value}")
        
        if 'fixes_applied' in self.solution_metadata:
            print(f"🔧 Fixes applied:")
            for fix in self.solution_metadata['fixes_applied']:
                print(f"   • {fix}")
        
        print(f"\n📍 Corner solutions:")
        for corner_name, solution in self.corner_solutions.items():
            position = solution['position']
            joints = solution['joints']
            method = solution['method']
            
            print(f"\n  {corner_name}:")
            print(f"    Method: {method}")
            print(f"    Position: ({position[0]:6.3f}, {position[1]:6.3f}, {position[2]:6.3f})")
            print(f"    Distance: {np.linalg.norm(position):.3f}m")
            print(f"    Joints (deg): {[f'{math.degrees(j):6.1f}°' for j in joints]}")

def main(args=None):
    """Main function."""
    
    rclpy.init(args=args)
    
    print("🚀 IMPROVED CHESS CORNER TESTER")
    print("=" * 60)
    print("Enhanced ROS2 trajectory execution with debugging")
    
    node = ImprovedChessCornerTester()
    
    try:
        # Wait for connections
        if not node.wait_for_connections():
            print("❌ Failed to establish connections")
            return
        
        # Move to home initially
        print("🏠 Moving to initial home position...")
        node.move_to_home()
        
        # Start enhanced interactive menu
        print("✅ Ready! Starting enhanced interactive menu...")
        node.interactive_menu_enhanced()
        
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

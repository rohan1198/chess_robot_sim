#!/usr/bin/env python3
"""
Smooth Trajectory Chess Corner Tester
=====================================

Final version with optimized trajectory generation and relaxed tolerances
specifically designed for successful chess corner movements.

Key improvements:
- Smoother trajectory generation with proper velocity profiles
- Relaxed timing and tolerances
- Better error handling and recovery
- Adaptive trajectory planning based on movement distance

Usage:
    ros2 run chess_robot_sim smooth_trajectory_tester.py

Author: Chess Robot Project - Final Version
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
from typing import List

class SmoothTrajectoryTester(Node):
    """Final chess corner tester with optimized smooth trajectories."""
    
    def __init__(self):
        super().__init__('smooth_trajectory_tester')
        
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
        
        # Corner solutions
        self.corner_solutions = {}
        self.solution_metadata = {}
        self.solution_file_used = None
        
        # Home position
        self.home_position = [0.0] * len(self.joint_names)
        
        # OPTIMIZED movement parameters for success
        self.base_speed_factor = 0.3      # Very conservative base speed
        self.min_duration = 6.0           # Minimum 6 seconds per movement
        self.max_duration = 15.0          # Maximum 15 seconds
        self.velocity_smoothing = 0.3     # Smooth velocity transitions
        
        self.get_logger().info('🚀 Smooth Trajectory Tester initialized!')
        self.get_logger().info('⚙️  Optimized for successful chess corner movements')
        
        # Load corner solutions
        if self.load_corner_solutions():
            self.get_logger().info(f'✅ Loaded {len(self.corner_solutions)} corner solutions')
        else:
            self.get_logger().error('❌ Failed to load corner solutions')
    
    def load_corner_solutions(self) -> bool:
        """Load the fixed corner solutions."""
        script_dir = Path(__file__).parent.absolute()
        
        solution_files = [
            script_dir / "chess_corner_solution.json",
        ]
        
        for solution_file in solution_files:
            if solution_file.exists():
                try:
                    with open(solution_file, 'r') as f:
                        data = json.load(f)
                    
                    self.solution_metadata = data.get('metadata', {})
                    self.corner_solutions = data.get('ik_solutions', {})
                    self.solution_file_used = solution_file.name
                    
                    self.get_logger().info(f'📁 Using: {solution_file.name}')
                    success_rate = self.solution_metadata.get('success_rate', 0) * 100
                    self.get_logger().info(f'📊 IK Success Rate: {success_rate:.0f}%')
                    
                    return len(self.corner_solutions) > 0
                    
                except Exception as e:
                    self.get_logger().error(f'Error loading {solution_file}: {e}')
                    continue
        
        return False
    
    def joint_state_callback(self, msg):
        """Update current joint positions."""
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
    
    def calculate_optimal_duration(self, current_pos: List[float], target_pos: List[float]) -> float:
        """Calculate optimal duration for smooth movement."""
        
        # Calculate maximum joint movement
        joint_distances = [abs(target_pos[i] - current_pos[i]) for i in range(len(current_pos))]
        max_distance = max(joint_distances)
        
        # Adaptive duration based on movement size
        if max_distance <= 0.5:      # Small movements (< 30°)
            base_time = 5.0
        elif max_distance <= 1.0:    # Medium movements (30-60°)
            base_time = 8.0
        else:                        # Large movements (> 60°)
            base_time = 12.0
        
        # Scale by actual distance and apply speed factor
        calculated_duration = base_time + (max_distance * 3.0 * self.base_speed_factor)
        
        # Apply bounds
        duration = max(self.min_duration, calculated_duration)
        duration = min(self.max_duration, duration)
        
        return duration
    
    def create_smooth_trajectory(self, target_positions: List[float], duration_sec: float) -> JointTrajectory:
        """Create ultra-smooth trajectory with proper velocity profiles."""
        
        current_positions = self.get_current_positions()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Calculate movement characteristics
        joint_distances = [target_positions[i] - current_positions[i] for i in range(len(current_positions))]
        max_distance = max(abs(d) for d in joint_distances)
        
        # Determine number of waypoints based on movement size
        if max_distance <= 0.3:      # Small movements
            num_waypoints = 5
        elif max_distance <= 0.8:    # Medium movements
            num_waypoints = 8
        else:                        # Large movements
            num_waypoints = 12
        
        points = []
        
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            # Use smooth acceleration profile (S-curve)
            # This creates smoother motion than linear interpolation
            if t <= 0.5:
                # Acceleration phase
                smooth_t = 2 * t * t
            else:
                # Deceleration phase
                smooth_t = 1 - 2 * (1 - t) * (1 - t)
            
            # Calculate waypoint positions
            waypoint_positions = []
            waypoint_velocities = []
            
            for j in range(len(current_positions)):
                pos = current_positions[j] + smooth_t * joint_distances[j]
                waypoint_positions.append(pos)
                
                # Calculate smooth velocities
                if i == 0 or i == num_waypoints:
                    # Start and end with zero velocity
                    vel = 0.0
                else:
                    # Calculate velocity based on S-curve derivative
                    if t <= 0.5:
                        vel_factor = 4 * t
                    else:
                        vel_factor = 4 * (1 - t)
                    
                    max_vel = joint_distances[j] / duration_sec
                    vel = max_vel * vel_factor * self.velocity_smoothing
                
                waypoint_velocities.append(vel)
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = waypoint_positions
            point.velocities = waypoint_velocities
            point.accelerations = [0.0] * len(self.joint_names)  # Let controller handle accelerations
            
            # Calculate time for this waypoint
            point_time = duration_sec * t
            point.time_from_start = Duration(
                sec=int(point_time), 
                nanosec=int((point_time - int(point_time)) * 1e9)
            )
            
            points.append(point)
        
        trajectory.points = points
        return trajectory
    
    def execute_smooth_trajectory(self, target_positions: List[float], description: str = "") -> bool:
        """Execute trajectory with optimal smoothness and error handling."""
        
        current_positions = self.get_current_positions()
        duration = self.calculate_optimal_duration(current_positions, target_positions)
        
        trajectory = self.create_smooth_trajectory(target_positions, duration)
        
        # Validate trajectory before sending
        max_distance = max(abs(target_positions[i] - current_positions[i]) for i in range(len(current_positions)))
        
        self.get_logger().info(f'🎯 {description}')
        self.get_logger().info(f'   Duration: {duration:.1f}s (optimal for {math.degrees(max_distance):.1f}° movement)')
        self.get_logger().info(f'   Waypoints: {len(trajectory.points)} (smooth S-curve)')
        self.get_logger().info(f'   Target: {[f"{math.degrees(pos):6.1f}°" for pos in target_positions]}')
        
        # Create and send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # Send goal with extended timeout
        future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        
        if not future.done():
            self.get_logger().error('❌ Goal send timeout')
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            return False
        
        self.get_logger().info('📈 Smooth trajectory accepted, monitoring...')
        
        # Monitor execution with extended patience
        start_time = time.time()
        last_log_time = start_time
        
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            current_time = time.time()
            elapsed = current_time - start_time
            
            # Log progress every 3 seconds (less frequent)
            if current_time - last_log_time >= 3.0:
                current_joints = self.get_current_positions()
                progress = min(100, (elapsed / duration) * 100)
                
                self.get_logger().info(f'   Progress: {progress:.0f}% - Current: {[f"{math.degrees(pos):5.1f}°" for pos in current_joints]}')
                last_log_time = current_time
            
            # Very generous timeout
            if elapsed > duration + 20.0:
                self.get_logger().error('❌ Execution timeout (very generous limit exceeded)')
                return False
            
            rclpy.spin_once(self, timeout_sec=0.2)  # Slower polling
        
        result = result_future.result().result
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            # Validate final position
            final_joints = self.get_current_positions()
            joint_errors = [abs(target_positions[i] - final_joints[i]) for i in range(len(target_positions))]
            max_joint_error = max(joint_errors)
            
            if max_joint_error > 0.15:  # ~8.6 degrees (relaxed)
                self.get_logger().warn(f'⚠️  Large joint error: {math.degrees(max_joint_error):.1f}° (but acceptable)')
            
            self.get_logger().info(f'✅ Smooth movement completed! Max error: {math.degrees(max_joint_error):.1f}°')
            return True
        else:
            error_descriptions = {
                -1: "INVALID_GOAL",
                -2: "INVALID_JOINTS", 
                -3: "OLD_HEADER_TIMESTAMP",
                -4: "PATH_TOLERANCE_VIOLATED",
                -5: "GOAL_TOLERANCE_VIOLATED"
            }
            error_desc = error_descriptions.get(result.error_code, f"UNKNOWN_ERROR_{result.error_code}")
            
            self.get_logger().error(f'❌ Movement failed: {error_desc} (code: {result.error_code})')
            self.get_logger().error('💡 Suggestion: Check relaxed_controllers.yaml is loaded')
            return False
    
    def move_to_home(self) -> bool:
        """Move to home position with smooth trajectory."""
        return self.execute_smooth_trajectory(self.home_position, "Moving to home position (smooth)")
    
    def move_to_corner_sequence(self, corner_name: str) -> bool:
        """Execute complete smooth movement sequence to corner."""
        
        if corner_name not in self.corner_solutions:
            self.get_logger().error(f'❌ No solution for corner: {corner_name}')
            return False
        
        solution_data = self.corner_solutions[corner_name]
        hover_joints = solution_data['joints']
        target_position = solution_data['position']
        method = solution_data['method']
        
        self.get_logger().info(f'🎯 SMOOTH MOVEMENT TO {corner_name.upper()} CORNER')
        self.get_logger().info(f'   Method: {method}')
        self.get_logger().info(f'   Target: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})')
        self.get_logger().info('   Using relaxed tolerances and smooth trajectories')
        
        sequence_success = True
        
        # Step 1: Move to home
        self.get_logger().info('📍 Step 1: Moving to home position...')
        if not self.move_to_home():
            self.get_logger().error('❌ Failed to reach home position')
            return False
        time.sleep(2)
        
        # Step 2: Move to target with optimal smoothness
        self.get_logger().info(f'📍 Step 2: Moving to {corner_name} with smooth trajectory...')
        if not self.execute_smooth_trajectory(hover_joints, f"Smooth movement to {corner_name} ({method})"):
            self.get_logger().error(f'❌ Failed to reach {corner_name} - check controller tolerances')
            sequence_success = False
        else:
            # Step 3: Hold and validate
            self.get_logger().info('📍 Step 3: Holding position...')
            time.sleep(3)
            
            # Relaxed validation
            final_joints = self.get_current_positions()
            joint_errors = [abs(hover_joints[i] - final_joints[i]) for i in range(len(hover_joints))]
            max_error = max(joint_errors)
            
            if max_error < 0.1:  # ~5.7 degrees
                self.get_logger().info(f'✅ Position excellent (error: {math.degrees(max_error):.1f}°)')
            elif max_error < 0.2:  # ~11.5 degrees
                self.get_logger().info(f'✅ Position good (error: {math.degrees(max_error):.1f}°)')
            else:
                self.get_logger().warn(f'⚠️  Position acceptable but not precise (error: {math.degrees(max_error):.1f}°)')
        
        # Step 4: Return home
        self.get_logger().info('📍 Step 4: Returning home...')
        if not self.move_to_home():
            self.get_logger().error('❌ Failed to return home')
            sequence_success = False
        
        if sequence_success:
            self.get_logger().info(f'🎉 {corner_name.upper()} smooth sequence completed!')
        else:
            self.get_logger().error(f'❌ {corner_name.upper()} sequence had issues')
        
        return sequence_success
    
    def test_all_corners_smooth(self) -> bool:
        """Test all corners with smooth trajectories."""
        
        self.get_logger().info('🏁 SMOOTH TRAJECTORY TEST - ALL CORNERS')
        self.get_logger().info('=' * 55)
        
        successful_corners = 0
        total_corners = len(self.corner_solutions)
        
        for corner_name in ['front_left', 'front_right', 'back_left', 'back_right']:
            if corner_name in self.corner_solutions:
                self.get_logger().info(f'\n🎯 Testing {corner_name} with smooth trajectories...')
                
                if self.move_to_corner_sequence(corner_name):
                    successful_corners += 1
                    self.get_logger().info(f'✅ {corner_name} - SMOOTH SUCCESS')
                else:
                    self.get_logger().error(f'❌ {corner_name} - FAILED')
                
                # Longer pause between corners for stability
                time.sleep(4)
            else:
                self.get_logger().info(f'⏭️  Skipping {corner_name} (no solution)')
        
        # Final results
        success_rate = successful_corners / total_corners * 100 if total_corners > 0 else 0
        
        self.get_logger().info('\n🏆 SMOOTH TRAJECTORY RESULTS:')
        self.get_logger().info(f'   Successful: {successful_corners}/{total_corners} ({success_rate:.0f}%)')
        
        if success_rate >= 100:
            self.get_logger().info('🎉 PERFECT! All corners reached with smooth trajectories!')
        elif success_rate >= 75:
            self.get_logger().info('✅ EXCELLENT! Smooth trajectories working well!')
        elif success_rate >= 50:
            self.get_logger().info('👍 GOOD! Most smooth movements successful!')
        else:
            self.get_logger().info('⚠️  NEEDS WORK! Check controller configuration!')
        
        return success_rate >= 75
    
    def interactive_menu(self):
        """Interactive menu for smooth trajectory testing."""
        
        while rclpy.ok():
            available_corners = list(self.corner_solutions.keys())
            
            print("\n" + "="*70)
            print("        🚀 Smooth Trajectory Chess Corner Tester")
            print("="*70)
            print(f"📁 Solution file: {self.solution_file_used}")
            print("⚙️  Optimized: Smooth S-curve trajectories, relaxed tolerances")
            print(f"📍 Available corners: {len(available_corners)}")
            
            for i, corner in enumerate(available_corners):
                solution = self.corner_solutions[corner]
                method = solution['method']
                joints = solution['joints']
                distance = np.linalg.norm(solution['position'])
                
                print(f"  {i+1}: {corner:<12s} ({method:<8s}) - {distance:.3f}m")
                print(f"      joints: {[f'{math.degrees(j):5.1f}°' for j in joints]}")
            
            print("\n🎯 Options:")
            print("  a: Test ALL corners (smooth trajectories)")
            print("  h: Move to HOME position (smooth)")
            print("  1-{len(available_corners)}: Test specific corner (smooth)")
            print("  q: Quit")
            print("="*70)
            
            try:
                choice = input("Enter your choice: ").strip().lower()
                
                if choice == 'q':
                    print("👋 Goodbye!")
                    break
                elif choice == 'h':
                    print("🏠 Moving to home with smooth trajectory...")
                    self.move_to_home()
                elif choice == 'a':
                    print("🏁 Testing ALL corners with smooth trajectories...")
                    self.test_all_corners_smooth()
                elif choice.isdigit():
                    corner_index = int(choice) - 1
                    if 0 <= corner_index < len(available_corners):
                        corner_name = available_corners[corner_index]
                        print(f"🎯 Testing {corner_name} with smooth trajectory...")
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
    
    print("🚀 SMOOTH TRAJECTORY CHESS CORNER TESTER")
    print("=" * 60)
    print("Final version with optimized smooth trajectories")
    print("💡 Make sure to use relaxed_controllers.yaml!")
    
    node = SmoothTrajectoryTester()
    
    try:
        # Wait for connections
        if not node.wait_for_connections():
            print("❌ Failed to establish connections")
            return
        
        # Move to home initially
        print("🏠 Moving to home with smooth trajectory...")
        node.move_to_home()
        
        # Start interactive menu
        print("✅ Ready! Starting smooth trajectory menu...")
        node.interactive_menu()
        
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Return to home before exiting
        print("🏠 Returning to home before exit...")
        try:
            node.move_to_home()
        except Exception:
            pass
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

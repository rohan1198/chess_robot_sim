#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time

class SO101JointTester(Node):
    def __init__(self):
        super().__init__('so101_joint_tester')
        
        # Action client for arm controller
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # Action client for gripper controller
        self.gripper_action_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_controller/gripper_cmd'
        )
        
        # Joint state subscriber to get actual positions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint names (arm joints + gripper)
        self.joint_names = [
            'shoulder_rotation',
            'shoulder_pitch', 
            'elbow',
            'wrist_pitch',
            'wrist_roll'
        ]
        
        # Gripper joint (separate from arm joints)
        self.gripper_joint_name = 'gripper_joint'
        
        # Joint limits [min, max] for each joint (conservative values)
        self.joint_limits = {
            'shoulder_rotation': [-1.5, 1.5],        # Reduced from [-1.920, 1.920]
            'shoulder_pitch': [-1.2, 1.2],           # Reduced from [-1.750, 1.750]
            'elbow': [-1.2, 1.2],                    # Reduced from [-1.750, 1.750]  
            'wrist_pitch': [-1.0, 1.0],             # Reduced from [-1.660, 1.660]
            'wrist_roll': [-2.0, 2.0],              # Reduced from [-2.790, 2.790]
        }
        
        # Gripper limits
        self.gripper_limits = [-0.15, 1.5]  # Close to open
        
        # Home position (all joints at 0)
        self.home_position = [0.0] * len(self.joint_names)
        self.gripper_home_position = 0.0  # Neutral position
        
        # Current joint positions from /joint_states
        self.current_joint_positions = {}
        self.joint_states_received = False
        
        self.get_logger().info('SO-ARM101 Joint Tester Ready!')
        
    def joint_state_callback(self, msg):
        """Update current joint positions from joint states."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names or name == self.gripper_joint_name:
                self.current_joint_positions[name] = msg.position[i]
        
        # Check if we have all joint positions (arm + gripper)
        expected_joints = len(self.joint_names) + 1  # arm joints + gripper
        if len(self.current_joint_positions) >= len(self.joint_names):  # At least arm joints
            self.joint_states_received = True
    
    def get_current_positions(self):
        """Get current joint positions in the correct order."""
        if not self.joint_states_received:
            self.get_logger().warn("Joint states not received yet, using home position")
            return self.home_position.copy()
        
        return [self.current_joint_positions.get(name, 0.0) for name in self.joint_names]
    
    def get_current_gripper_position(self):
        """Get current gripper position."""
        return self.current_joint_positions.get(self.gripper_joint_name, self.gripper_home_position)
    
    def wait_for_action_server(self):
        """Wait for the action servers to be available."""
        self.get_logger().info('Waiting for arm controller action server...')
        self.arm_action_client.wait_for_server()
        self.get_logger().info('Arm controller connected!')
        
        self.get_logger().info('Waiting for gripper controller action server...')
        self.gripper_action_client.wait_for_server()
        self.get_logger().info('Gripper controller connected!')
    
    def wait_for_joint_states(self, timeout=10.0):
        """Wait for joint states to be received."""
        self.get_logger().info('Waiting for joint states...')
        start_time = time.time()
        while not self.joint_states_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.joint_states_received:
            self.get_logger().info('Joint states received!')
            return True
        else:
            self.get_logger().error('Timeout waiting for joint states!')
            return False
    
    def calculate_smart_duration(self, current_positions, target_positions, min_duration=3.0):
        """Calculate an appropriate duration based on the largest joint movement."""
        max_distance = max(abs(target_positions[i] - current_positions[i]) for i in range(len(current_positions)))
        
        # More aggressive duration calculation: allow 2 seconds per radian (was 3.0)
        # This should work with our increased velocity limits
        calculated_duration = max_distance * 2.0
        
        # Ensure minimum duration
        duration = max(min_duration, calculated_duration)
        
        # Cap at reasonable maximum
        duration = min(duration, 12.0)  # Reduced from 15.0
        
        return duration
    
    def calculate_velocity_at_waypoint(self, prev_pos, current_pos, next_pos, time_to_prev, time_to_next):
        """Calculate smooth velocity at a waypoint using finite differences."""
        # Calculate velocities from both directions
        vel_from_prev = [(current_pos[i] - prev_pos[i]) / time_to_prev for i in range(len(current_pos))]
        vel_to_next = [(next_pos[i] - current_pos[i]) / time_to_next for i in range(len(current_pos))]
        
        # Average them for smooth transition, but be more aggressive
        smooth_vel = [(vel_from_prev[i] + vel_to_next[i]) * 0.7 for i in range(len(current_pos))]  # Increased from 0.5
        
        return smooth_vel
    
    def create_smooth_trajectory(self, target_positions, duration_sec=None):
        """Create a smooth trajectory to target positions with proper velocity profiles."""
        current_positions = self.get_current_positions()
        
        # Calculate smart duration if not provided
        if duration_sec is None:
            duration_sec = self.calculate_smart_duration(current_positions, target_positions)
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Calculate the largest movement
        distances = [abs(target_positions[i] - current_positions[i]) for i in range(len(current_positions))]
        max_distance = max(distances)
        
        points = []
        
        # For small movements, use simple two-point trajectory
        if max_distance <= 0.5:  # Reduced from 1.0 for faster small movements
            # Start point
            start_point = JointTrajectoryPoint()
            start_point.positions = current_positions
            start_point.velocities = [0.0] * len(self.joint_names)
            start_point.accelerations = [0.0] * len(self.joint_names)
            start_point.time_from_start = Duration(sec=0, nanosec=0)
            points.append(start_point)
            
            # End point
            end_point = JointTrajectoryPoint()
            end_point.positions = target_positions
            end_point.velocities = [0.0] * len(self.joint_names)
            end_point.accelerations = [0.0] * len(self.joint_names)
            end_point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
            points.append(end_point)
        else:
            # For large movements, create smooth trajectory with fewer segments for speed
            num_segments = max(2, int(max_distance / 1.0))  # Fewer segments, larger steps
            
            self.get_logger().info(f'Large movement ({max_distance:.2f} rad), creating {num_segments} segments')
            
            # Create all positions first
            all_positions = [current_positions]
            for i in range(1, num_segments):
                t = i / num_segments
                waypoint_positions = []
                for j in range(len(current_positions)):
                    pos = current_positions[j] + t * (target_positions[j] - current_positions[j])
                    waypoint_positions.append(pos)
                all_positions.append(waypoint_positions)
            all_positions.append(target_positions)
            
            # Create trajectory points with proper velocities
            for i, pos in enumerate(all_positions):
                point = JointTrajectoryPoint()
                point.positions = pos
                point.accelerations = [0.0] * len(self.joint_names)
                
                # Calculate time for this point
                t = i / (len(all_positions) - 1)
                point_time = duration_sec * t
                point.time_from_start = Duration(
                    sec=int(point_time), 
                    nanosec=int((point_time - int(point_time)) * 1e9)
                )
                
                # Calculate velocities for smooth motion
                if i == 0:
                    # Start with zero velocity
                    point.velocities = [0.0] * len(self.joint_names)
                elif i == len(all_positions) - 1:
                    # End with zero velocity
                    point.velocities = [0.0] * len(self.joint_names)
                else:
                    # Calculate smooth velocity at waypoint
                    prev_pos = all_positions[i-1]
                    next_pos = all_positions[i+1]
                    time_between = duration_sec / (len(all_positions) - 1)
                    point.velocities = self.calculate_velocity_at_waypoint(
                        prev_pos, pos, next_pos, time_between, time_between
                    )
                
                points.append(point)
        
        trajectory.points = points
        self.get_logger().info(f'Created smooth trajectory with {len(points)} points over {duration_sec:.1f}s')
        return trajectory, duration_sec
    
    def send_trajectory_goal(self, target_positions, duration_sec=None, retry_count=0):
        """Send a trajectory goal and wait for completion with retry logic."""
        current_positions = self.get_current_positions()
        
        # Check if target positions are within limits
        for i, (joint_name, target_pos) in enumerate(zip(self.joint_names, target_positions)):
            min_pos, max_pos = self.joint_limits[joint_name]
            if target_pos < min_pos or target_pos > max_pos:
                self.get_logger().error(f"Target position {target_pos:.3f} for {joint_name} is outside limits [{min_pos:.3f}, {max_pos:.3f}]")
                return False
        
        trajectory, actual_duration = self.create_smooth_trajectory(target_positions, duration_sec)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        # Calculate movement distance for logging
        max_distance = max(abs(target_positions[i] - current_positions[i]) for i in range(len(current_positions)))
        
        retry_info = f" (retry {retry_count})" if retry_count > 0 else ""
        self.get_logger().info(f'Moving from {[f"{p:.2f}" for p in current_positions]} to {[f"{p:.2f}" for p in target_positions]} in {actual_duration:.1f}s (max dist: {max_distance:.2f}){retry_info}')
        
        # Send goal
        future = self.arm_action_client.send_goal_async(goal_msg)
        
        # Wait for goal to be accepted
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)  # Increased timeout
        
        if not future.done():
            self.get_logger().error('Goal send timeout!')
            return self._retry_trajectory(target_positions, duration_sec, retry_count)
            
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return self._retry_trajectory(target_positions, duration_sec, retry_count)
        
        self.get_logger().info('Goal accepted, executing...')
        
        # Wait for completion with extra time for large movements
        wait_timeout = actual_duration + 15.0  # Increased buffer
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=wait_timeout)
        
        if not result_future.done():
            self.get_logger().error('Goal execution timeout!')
            return self._retry_trajectory(target_positions, duration_sec, retry_count)
        
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory completed successfully!')
            return True
        else:
            self.get_logger().error(f'Trajectory failed with error code: {result.error_code}')
            return self._retry_trajectory(target_positions, duration_sec, retry_count)
    
    def _retry_trajectory(self, target_positions, duration_sec, retry_count):
        """Retry trajectory with longer duration."""
        if retry_count >= 2:  # Max 2 retries
            self.get_logger().error('Max retries reached, trajectory failed')
            return False
        
        # Increase duration by 50% for retry
        new_duration = (duration_sec or 5.0) * 1.5
        self.get_logger().warn(f'Retrying with longer duration: {new_duration:.1f}s')
        time.sleep(1)  # Brief pause before retry
        return self.send_trajectory_goal(target_positions, new_duration, retry_count + 1)
    
    def move_to_position(self, target_positions, duration_sec=None):
        """Move to a target position smoothly with smart duration calculation."""
        return self.send_trajectory_goal(target_positions, duration_sec)
    
    def reset_to_home(self):
        """Reset robot to home position."""
        return self.move_to_position(self.home_position)
    
    def test_joint(self, joint_index, use_safe_limits=True):
        """Test a specific joint through its range of motion."""
        joint_name = self.joint_names[joint_index]
        min_pos, max_pos = self.joint_limits[joint_name]
        
        # Use even more conservative limits for testing, especially for wrist_roll
        if use_safe_limits:
            if joint_name == 'wrist_roll':
                # Extra conservative for wrist_roll due to large range
                min_pos *= 0.4  # 40% of limits for wrist_roll
                max_pos *= 0.4
            else:
                min_pos *= 0.6  # 60% of limits for other joints
                max_pos *= 0.6
        
        self.get_logger().info(f'\n=== Testing Joint {joint_index}: {joint_name} ===')
        self.get_logger().info(f'Range: {min_pos:.3f} to {max_pos:.3f} radians ({"safe" if use_safe_limits else "full"} limits)')
        
        # 1. Move to home position
        self.get_logger().info('1. Moving to home position...')
        if not self.reset_to_home():
            return False
        time.sleep(1)
        
        # 2. Move to negative limit
        neg_position = self.home_position.copy()
        neg_position[joint_index] = min_pos
        self.get_logger().info(f'2. Moving to negative limit ({min_pos:.3f} rad)...')
        if not self.move_to_position(neg_position):
            self.get_logger().error('Failed to reach negative limit.')
            return False
        time.sleep(1)
        
        # 3. Move to positive limit
        pos_position = self.home_position.copy()
        pos_position[joint_index] = max_pos
        self.get_logger().info(f'3. Moving to positive limit ({max_pos:.3f} rad)...')
        if not self.move_to_position(pos_position):
            self.get_logger().error('Failed to reach positive limit.')
            return False
        time.sleep(1)
        
        # 4. Return to home
        self.get_logger().info('4. Returning to home position...')
        if not self.reset_to_home():
            return False
        
        self.get_logger().info(f'=== Joint {joint_index} test completed! ===\n')
        return True
    
    def test_all_joints(self):
        """Test all joints sequentially with safe limits."""
        self.get_logger().info('\n=== Testing ALL JOINTS (SAFE LIMITS) ===')
        for i in range(len(self.joint_names)):
            if not self.test_joint(i, use_safe_limits=True):
                self.get_logger().error(f'Failed to test joint {i}')
                return False
            time.sleep(1)  # Short pause between joint tests
        self.get_logger().info('=== ALL JOINTS TEST COMPLETED! ===')
        return True
    
    def print_menu(self):
        """Print the interactive menu."""
        current_pos = self.get_current_positions()
        current_gripper = self.get_current_gripper_position()
        
        print("\n" + "="*60)
        print("           SO-ARM101 Joint Tester")
        print("="*60)
        print(f"Current position: {[f'{p:.2f}' for p in current_pos]}")
        print(f"Current gripper: {current_gripper:.2f}")
        print("Select a joint to test:")
        for i, joint_name in enumerate(self.joint_names):
            min_pos, max_pos = self.joint_limits[joint_name]
            if joint_name == 'wrist_roll':
                safe_min, safe_max = min_pos * 0.4, max_pos * 0.4
                limit_info = "40% of max"
            else:
                safe_min, safe_max = min_pos * 0.6, max_pos * 0.6
                limit_info = "60% of max"
            print(f"  {i}: {joint_name:<20} (safe: {safe_min:6.2f} to {safe_max:5.2f} rad, {limit_info})")
        print(f"  {len(self.joint_names)}: Test ALL joints (safe limits)")
        print("  g: Test gripper (open/close)")
        print("  o: Open gripper")
        print("  c: Close gripper")
        print("  h: Move to home position")
        print("  t: Test trajectory (demo movement)")
        print("  s: Single step test (manual control)")
        print("  q: Quit")
        print("="*60)

    def single_step_test(self):
        """Allow manual single-step joint testing."""
        print("\nSingle Step Test Mode")
        print("Enter joint positions as: joint_index value")
        print("Example: '0 0.5' moves shoulder_rotation to 0.5 rad")
        print("Type 'home' to return to home, 'menu' to return to main menu")
        
        while True:
            try:
                cmd = input("Command (joint_index value): ").strip().lower()
                
                if cmd == 'menu':
                    break
                elif cmd == 'home':
                    self.reset_to_home()
                    continue
                
                parts = cmd.split()
                if len(parts) != 2:
                    print("Invalid format. Use: joint_index value")
                    continue
                
                joint_idx = int(parts[0])
                value = float(parts[1])
                
                if joint_idx < 0 or joint_idx >= len(self.joint_names):
                    print(f"Invalid joint index. Use 0-{len(self.joint_names)-1}")
                    continue
                
                joint_name = self.joint_names[joint_idx]
                min_pos, max_pos = self.joint_limits[joint_name]
                
                if value < min_pos or value > max_pos:
                    print(f"Value {value:.3f} outside safe range [{min_pos:.3f}, {max_pos:.3f}]")
                    continue
                
                target_pos = self.get_current_positions()
                target_pos[joint_idx] = value
                
                print(f"Moving {joint_name} to {value:.3f} rad...")
                self.move_to_position(target_pos)
                
            except ValueError:
                print("Invalid input. Use: joint_index value")
            except KeyboardInterrupt:
                break

    def demo_trajectory(self):
        """Demonstrate a smooth multi-joint movement."""
        self.get_logger().info("Running demo trajectory...")
        
        # Conservative demo position
        demo_position = [0.3, -0.3, 0.2, 0.0, 0.5]
        
        if not self.move_to_position(demo_position):
            return False
        time.sleep(2)
        
        # Return to home
        return self.move_to_position(self.home_position)

    def send_gripper_goal(self, target_position, max_effort=50.0):
        """Send a gripper movement goal using GripperCommand."""
        current_gripper_pos = self.get_current_gripper_position()
        
        # Check limits
        if target_position < self.gripper_limits[0] or target_position > self.gripper_limits[1]:
            self.get_logger().error(f"Gripper position {target_position:.3f} outside limits {self.gripper_limits}")
            return False
        
        # Create gripper command goal
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = target_position
        goal_msg.command.max_effort = max_effort
        
        self.get_logger().info(f'Moving gripper from {current_gripper_pos:.2f} to {target_position:.2f}')
        
        # Send goal
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done():
            self.get_logger().error('Gripper goal send timeout!')
            return False
            
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
        
        self.get_logger().info('Gripper goal accepted, executing...')
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        
        if not result_future.done():
            self.get_logger().error('Gripper execution timeout!')
            return False
        
        result = result_future.result().result
        if result.reached_goal:
            self.get_logger().info('Gripper movement completed successfully!')
            return True
        else:
            self.get_logger().error(f'Gripper movement failed. Position: {result.position:.3f}, Effort: {result.effort:.3f}')
            return False
    
    def open_gripper(self):
        """Open the gripper."""
        return self.send_gripper_goal(self.gripper_limits[1])  # Max position = open
    
    def close_gripper(self):
        """Close the gripper."""
        return self.send_gripper_goal(self.gripper_limits[0])  # Min position = close
    
    def reset_gripper(self):
        """Reset gripper to neutral position."""
        return self.send_gripper_goal(self.gripper_home_position)
    
    def test_gripper(self):
        """Test gripper open and close functionality."""
        self.get_logger().info('\n=== Testing Gripper ===')
        self.get_logger().info(f'Range: {self.gripper_limits[0]:.3f} to {self.gripper_limits[1]:.3f}')
        
        # 1. Reset to neutral
        self.get_logger().info('1. Moving to neutral position...')
        if not self.reset_gripper():
            return False
        time.sleep(1)
        
        # 2. Close gripper
        self.get_logger().info('2. Closing gripper...')
        if not self.close_gripper():
            self.get_logger().error('Failed to close gripper.')
            return False
        time.sleep(1)
        
        # 3. Open gripper
        self.get_logger().info('3. Opening gripper...')
        if not self.open_gripper():
            self.get_logger().error('Failed to open gripper.')
            return False
        time.sleep(1)
        
        # 4. Return to neutral
        self.get_logger().info('4. Returning to neutral position...')
        if not self.reset_gripper():
            return False
        
        self.get_logger().info('=== Gripper test completed! ===\n')
        return True

def main(args=None):
    rclpy.init(args=args)
    
    node = SO101JointTester()
    
    try:
        # Wait for action server and joint states
        node.wait_for_action_server()
        
        if not node.wait_for_joint_states():
            print("Failed to receive joint states. Exiting...")
            return
        
        # Move to home position initially
        print("Moving to initial home position...")
        node.reset_to_home()
        
        # Interactive loop
        while rclpy.ok():
            node.print_menu()
            try:
                choice = input("Enter your choice: ").strip().lower()
                
                if choice == 'q':
                    print("Exiting...")
                    break
                elif choice == 'h':
                    print("Moving to home position...")
                    node.reset_to_home()
                elif choice == 't':
                    node.demo_trajectory()
                elif choice == 's':
                    node.single_step_test()
                elif choice.isdigit():
                    joint_index = int(choice)
                    if 0 <= joint_index < len(node.joint_names):
                        node.test_joint(joint_index)
                    elif joint_index == len(node.joint_names):
                        node.test_all_joints()
                    else:
                        print(f"Invalid choice! Please select 0-{len(node.joint_names)}")
                elif choice == 'g':
                    node.test_gripper()
                elif choice == 'o':
                    node.open_gripper()
                elif choice == 'c':
                    node.close_gripper()
                else:
                    print("Invalid choice! Please try again.")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
            except Exception as e:
                print(f"Error: {e}")
                
    except KeyboardInterrupt:
        pass
    finally:
        # Return to home before exiting
        print("Returning to home position before exit...")
        try:
            node.reset_to_home()
        except Exception:
            pass
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

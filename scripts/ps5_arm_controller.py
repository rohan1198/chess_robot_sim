#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
import time

class PS5ArmController(Node):
    def __init__(self):
        super().__init__('ps5_arm_controller')
        
        # Joy subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
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
        
        # Joint configuration
        self.joint_names = [
            'shoulder_rotation',    # Joint 0
            'shoulder_pitch',       # Joint 1
            'elbow',               # Joint 2
            'wrist_pitch',         # Joint 3
            'wrist_roll'           # Joint 4
        ]
        
        self.gripper_joint_name = 'gripper_joint'
        
        # Current joint positions
        self.current_joint_positions = {}
        self.joint_states_received = False
        
        # Control parameters
        self.movement_step = 0.1  # radians per button press
        self.fine_movement_step = 0.02  # fine control step
        self.movement_duration = 1.0  # seconds
        
        # Button state tracking (to detect button press, not hold)
        self.previous_buttons = [0] * 13
        self.button_press_time = {}
        self.repeat_delay = 0.3  # seconds between repeats when holding
        
        # PS5 Button mapping (based on your testing)
        self.BUTTON_X = 0
        self.BUTTON_O = 1  
        self.BUTTON_TRIANGLE = 2
        self.BUTTON_SQUARE = 3
        self.BUTTON_L1 = 4
        self.BUTTON_R1 = 5
        self.BUTTON_L2 = 6
        self.BUTTON_R2 = 7
        self.BUTTON_SELECT = 8
        self.BUTTON_START = 9
        self.BUTTON_PS = 10
        self.BUTTON_L3 = 11
        self.BUTTON_R3 = 12
        
        # Control scheme
        self.control_mode = 0  # 0=shoulder_rotation, 1=shoulder_pitch, 2=elbow, 3=wrist_pitch, 4=wrist_roll
        self.fine_control = False
        
        # Status
        self.last_command_time = time.time()
        self.command_timeout = 2.0  # seconds
        
        self.get_logger().info('PS5 Arm Controller Ready!')
        self.print_controls()
        
    def print_controls(self):
        """Print the control scheme."""
        print("\n" + "="*70)
        print("                    PS5 ARM CONTROLLER")
        print("="*70)
        print("JOINT SELECTION:")
        print("  L1 + X        : Shoulder Rotation")
        print("  L1 + O        : Shoulder Pitch") 
        print("  L1 + Triangle : Elbow")
        print("  L1 + Square   : Wrist Pitch")
        print("  L1 + Start    : Wrist Roll")
        print("")
        print("MOVEMENT (for selected joint):")
        print("  X             : Move Negative (-)")
        print("  O             : Move Positive (+)")
        print("  R1 + Movement : Fine Control (smaller steps)")
        print("")
        print("GRIPPER:")
        print("  Triangle      : Open Gripper")
        print("  Square        : Close Gripper")
        print("")
        print("SPECIAL:")
        print("  L2 + R2       : Emergency Stop & Home Position")
        print("  Select        : Print Current Status")
        print("="*70)
    
    def joint_state_callback(self, msg):
        """Update current joint positions."""
        for i, name in enumerate(msg.name):
            if name in self.joint_names or name == self.gripper_joint_name:
                self.current_joint_positions[name] = msg.position[i]
        
        if len(self.current_joint_positions) >= len(self.joint_names):
            self.joint_states_received = True
    
    def get_current_positions(self):
        """Get current joint positions in order."""
        if not self.joint_states_received:
            return [0.0] * len(self.joint_names)
        return [self.current_joint_positions.get(name, 0.0) for name in self.joint_names]
    
    def get_current_gripper_position(self):
        """Get current gripper position."""
        return self.current_joint_positions.get(self.gripper_joint_name, 0.0)
    
    def joy_callback(self, msg):
        """Handle joystick input."""
        current_time = time.time()
        buttons = list(msg.buttons)  # Convert array.array to list
        
        # Ensure we have enough buttons
        while len(buttons) < 13:
            buttons.append(0)
        
        # Check for button presses and holds
        for i, (current, previous) in enumerate(zip(buttons, self.previous_buttons)):
            if current and not previous:
                # Button just pressed
                self.button_press_time[i] = current_time
                self.handle_button_press(i, buttons, just_pressed=True)
            elif current and previous:
                # Button held - check if enough time passed for repeat
                if i in self.button_press_time:
                    if current_time - self.button_press_time[i] >= self.repeat_delay:
                        self.handle_button_press(i, buttons, just_pressed=False)
                        self.button_press_time[i] = current_time
        
        self.previous_buttons = buttons  # Now it's a list, so this works
    
    def handle_button_press(self, button_id, all_buttons, just_pressed=True):
        """Handle individual button presses."""
        # Check for modifier keys
        l1_pressed = all_buttons[self.BUTTON_L1] if len(all_buttons) > self.BUTTON_L1 else False
        r1_pressed = all_buttons[self.BUTTON_R1] if len(all_buttons) > self.BUTTON_R1 else False
        l2_pressed = all_buttons[self.BUTTON_L2] if len(all_buttons) > self.BUTTON_L2 else False
        r2_pressed = all_buttons[self.BUTTON_R2] if len(all_buttons) > self.BUTTON_R2 else False
        
        # Emergency stop / home
        if l2_pressed and r2_pressed:
            if just_pressed:
                self.get_logger().info("Emergency Stop - Moving to Home Position!")
                self.move_to_home()
            return
        
        # Joint selection with L1 modifier
        if l1_pressed and just_pressed:
            if button_id == self.BUTTON_X:
                self.control_mode = 0
                self.get_logger().info("Selected: Shoulder Rotation")
            elif button_id == self.BUTTON_O:
                self.control_mode = 1
                self.get_logger().info("Selected: Shoulder Pitch")
            elif button_id == self.BUTTON_TRIANGLE:
                self.control_mode = 2
                self.get_logger().info("Selected: Elbow")
            elif button_id == self.BUTTON_SQUARE:
                self.control_mode = 3
                self.get_logger().info("Selected: Wrist Pitch")
            elif button_id == self.BUTTON_START:
                self.control_mode = 4
                self.get_logger().info("Selected: Wrist Roll")
            return
        
        # Status print
        if button_id == self.BUTTON_SELECT and just_pressed:
            self.print_status()
            return
        
        # Fine control mode
        self.fine_control = r1_pressed
        
        # Joint movements (without L1 modifier)
        if not l1_pressed:
            step = self.fine_movement_step if self.fine_control else self.movement_step
            
            if button_id == self.BUTTON_X:
                # Move current joint negative
                self.move_joint(self.control_mode, -step)
            elif button_id == self.BUTTON_O:
                # Move current joint positive  
                self.move_joint(self.control_mode, step)
            elif button_id == self.BUTTON_TRIANGLE and just_pressed:
                # Open gripper
                self.control_gripper(1.5)  # Open position
            elif button_id == self.BUTTON_SQUARE and just_pressed:
                # Close gripper
                self.control_gripper(-0.15)  # Close position
    
    def move_joint(self, joint_index, delta):
        """Move a specific joint by delta amount."""
        if not self.joint_states_received:
            self.get_logger().warn("Joint states not received yet!")
            return
        
        if joint_index < 0 or joint_index >= len(self.joint_names):
            return
        
        # Get current positions
        current_positions = self.get_current_positions()
        target_positions = current_positions.copy()
        
        # Apply movement
        target_positions[joint_index] += delta
        
        # Safety limits (conservative)
        joint_limits = {
            0: [-1.5, 1.5],   # shoulder_rotation
            1: [-1.2, 1.2],   # shoulder_pitch
            2: [-1.2, 1.2],   # elbow
            3: [-1.0, 1.0],   # wrist_pitch
            4: [-2.0, 2.0],   # wrist_roll
        }
        
        min_pos, max_pos = joint_limits[joint_index]
        target_positions[joint_index] = max(min_pos, min(max_pos, target_positions[joint_index]))
        
        # Send movement command
        joint_name = self.joint_names[joint_index]
        self.get_logger().info(f"Moving {joint_name}: {current_positions[joint_index]:.3f} → {target_positions[joint_index]:.3f}")
        
        self.send_arm_command(target_positions)
    
    def send_arm_command(self, target_positions):
        """Send arm movement command."""
        current_positions = self.get_current_positions()
        
        # Create trajectory
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
        end_point.time_from_start = Duration(sec=int(self.movement_duration), nanosec=0)
        
        trajectory.points = [start_point, end_point]
        
        # Send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.arm_action_client.send_goal_async(goal_msg)
    
    def control_gripper(self, target_position):
        """Control gripper position."""
        current_pos = self.get_current_gripper_position()
        self.get_logger().info(f"Moving gripper: {current_pos:.3f} → {target_position:.3f}")
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = target_position
        goal_msg.command.max_effort = 50.0
        
        self.gripper_action_client.send_goal_async(goal_msg)
    
    def move_to_home(self):
        """Move arm to home position."""
        home_position = [0.0] * len(self.joint_names)
        self.send_arm_command(home_position)
        
        # Also reset gripper
        self.control_gripper(0.0)
    
    def print_status(self):
        """Print current robot status."""
        if not self.joint_states_received:
            print("Joint states not available yet")
            return
        
        current_pos = self.get_current_positions()
        current_gripper = self.get_current_gripper_position()
        
        print("\n" + "="*50)
        print("CURRENT ROBOT STATUS")
        print("="*50)
        for i, (name, pos) in enumerate(zip(self.joint_names, current_pos)):
            marker = " <-- SELECTED" if i == self.control_mode else ""
            print(f"  {name:<18}: {pos:7.3f} rad{marker}")
        print(f"  {'gripper':<18}: {current_gripper:7.3f} rad")
        print("")
        print(f"Control Mode: {self.joint_names[self.control_mode]}")
        print(f"Fine Control: {'ON' if self.fine_control else 'OFF'}")
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    
    print("Starting PS5 Arm Controller...")
    print("Make sure to run: ros2 run joy joy_node --ros-args -p device:=/dev/input/js0")
    
    controller = PS5ArmController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down PS5 controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 

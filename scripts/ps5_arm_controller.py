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
        
        # Joint configuration (5 arm joints)
        self.joint_names = [
            'shoulder_rotation',    # Joint 0
            'shoulder_pitch',       # Joint 1
            'elbow',               # Joint 2
            'wrist_pitch',         # Joint 3
            'wrist_roll',          # Joint 4
        ]
        
        # Filter out joints that don't exist
        self.active_joint_names = []
        self.joint_name_to_index = {}
        
        # Gripper is separate - using dedicated gripper controller
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
        self.previous_axes = [0] * 8
        self.button_press_time = {}
        self.repeat_delay = 0.2  # seconds between repeats when holding (faster for responsive control)
        
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
        
        # Axes mapping for D-pad arrows
        self.AXIS_DPAD_LR = 6  # Left/Right arrows: -32767 left, 32767 right
        self.AXIS_DPAD_UD = 7  # Up/Down arrows: -32767 up, 32767 down
        
        # Joint control mapping - 2 buttons per joint for +/- movement
        # This gives us direct control over 6 joints using 12 controls
        self.joint_controls = {
            # Joint 0: Shoulder Rotation
            0: {'positive': self.BUTTON_L1, 'negative': self.BUTTON_L2},
            # Joint 1: Shoulder Pitch  
            1: {'positive': self.BUTTON_R1, 'negative': self.BUTTON_R2},
            # Joint 2: Elbow
            2: {'positive': self.BUTTON_X, 'negative': self.BUTTON_O},
            # Joint 3: Wrist Pitch
            3: {'positive': self.BUTTON_TRIANGLE, 'negative': self.BUTTON_SQUARE},
            # Joint 4: Wrist Roll
            4: {'positive': self.BUTTON_L3, 'negative': self.BUTTON_R3},
        }
        
        # Gripper controls (D-pad up/down)
        self.gripper_controls = {
            'open': 'dpad_up',
            'close': 'dpad_down'
        }
        
        # Status
        self.last_command_time = time.time()
        self.command_timeout = 2.0  # seconds
        
        self.get_logger().info('PS5 Arm Controller Ready!')
        self.print_controls()
        
    def print_controls(self):
        """Print the control scheme."""
        print("\n" + "="*70)
        print("                    PS5 ARM CONTROLLER - DIRECT CONTROL")
        print("="*70)
        print("ARM JOINT CONTROLS (Direct mapping - 2 buttons per joint):")
        print("  Joint 0 (Shoulder Rotation): L1(+) / L2(-)")
        print("  Joint 1 (Shoulder Pitch)  : R1(+) / R2(-)")
        print("  Joint 2 (Elbow)           : X(+)  / O(-)")
        print("  Joint 3 (Wrist Pitch)     : △(+)  / □(-)")
        print("  Joint 4 (Wrist Roll)      : L3(+) / R3(-)")
        print("")
        print("GRIPPER CONTROLS (Dedicated Controller):")
        print("  Open Gripper              : ↑ (D-pad Up)")
        print("  Close Gripper             : ↓ (D-pad Down)")
        print("")
        print("SPECIAL:")
        print("  Emergency Home            : SELECT + START")
        print("  Print Status              : PS Button")
        print("")
        print("MODIFIERS:")
        print("  Fine Control              : Hold any shoulder button while moving")
        print("                              (L1, L2, R1, R2 reduce movement step)")
        print("")
        print("FUTURE EXPANSION:")
        print("  D-pad ←/→ reserved for Joint 5 if arm is upgraded to 6DoF")
        print("="*70)
    
    def joint_state_callback(self, msg):
        """Update current joint positions and filter available joints."""
        available_joints = []
        for i, name in enumerate(msg.name):
            if name in self.joint_names:  # 5 arm joints
                self.current_joint_positions[name] = msg.position[i]
                if name not in self.active_joint_names:
                    available_joints.append(name)
            elif name == self.gripper_joint_name:  # Gripper handled separately
                self.current_joint_positions[name] = msg.position[i]
        
        # Update active joints list only once
        if not self.joint_states_received and available_joints:
            self.active_joint_names = [name for name in self.joint_names if name in msg.name]
            self.joint_name_to_index = {name: i for i, name in enumerate(self.active_joint_names)}
            self.get_logger().info(f"Active arm joints detected: {self.active_joint_names}")
            if self.gripper_joint_name in self.current_joint_positions:
                self.get_logger().info(f"Gripper joint detected: {self.gripper_joint_name}")
        
        if len(self.current_joint_positions) >= len(self.active_joint_names):
            self.joint_states_received = True
    
    def get_current_positions(self):
        """Get current joint positions in order for active joints."""
        if not self.joint_states_received:
            return [0.0] * len(self.active_joint_names)
        return [self.current_joint_positions.get(name, 0.0) for name in self.active_joint_names]
    
    def get_current_gripper_position(self):
        """Get current gripper position."""
        return self.current_joint_positions.get(self.gripper_joint_name, 0.0)
    
    def joy_callback(self, msg):
        """Handle joystick input."""
        current_time = time.time()
        buttons = list(msg.buttons)  # Convert array.array to list
        axes = list(msg.axes) if hasattr(msg, 'axes') else [0] * 8
        
        # Ensure we have enough buttons and axes
        while len(buttons) < 13:
            buttons.append(0)
        while len(axes) < 8:
            axes.append(0)
        
        # Handle D-pad axes (convert to button-like behavior)
        dpad_states = self.process_dpad_axes(axes)
        
        # Check for button presses and holds
        for i, (current, previous) in enumerate(zip(buttons, self.previous_buttons)):
            if current and not previous:
                # Button just pressed
                self.button_press_time[i] = current_time
                self.handle_button_action(i, buttons, dpad_states, just_pressed=True)
            elif current and previous:
                # Button held - check if enough time passed for repeat
                if i in self.button_press_time:
                    if current_time - self.button_press_time[i] >= self.repeat_delay:
                        self.handle_button_action(i, buttons, dpad_states, just_pressed=False)
                        self.button_press_time[i] = current_time
        
        # Handle D-pad actions (gripper control)
        self.handle_dpad_actions(dpad_states, current_time)
        
        self.previous_buttons = buttons
        self.previous_axes = axes
    
    def process_dpad_axes(self, axes):
        """Convert D-pad axes to button-like states."""
        dpad_states = {
            'dpad_left': False,
            'dpad_right': False, 
            'dpad_up': False,
            'dpad_down': False
        }
        
        if len(axes) > self.AXIS_DPAD_LR:
            lr_value = axes[self.AXIS_DPAD_LR]
            if lr_value < -16000:  # Left
                dpad_states['dpad_left'] = True
            elif lr_value > 16000:  # Right
                dpad_states['dpad_right'] = True
        
        if len(axes) > self.AXIS_DPAD_UD:
            ud_value = axes[self.AXIS_DPAD_UD]
            if ud_value < -16000:  # Up
                dpad_states['dpad_up'] = True
            elif ud_value > 16000:  # Down
                dpad_states['dpad_down'] = True
        
        return dpad_states
    
    def handle_dpad_actions(self, dpad_states, current_time):
        """Handle D-pad based actions for gripper control."""
        # Gripper control using dedicated gripper controller
        if dpad_states['dpad_up']:
            self.control_gripper(1.5, current_time)  # Open gripper
        elif dpad_states['dpad_down']:
            self.control_gripper(-0.15, current_time)  # Close gripper
        
        # D-pad left/right reserved for future Joint 5 if arm is expanded
        # if dpad_states['dpad_right']:
        #     self.handle_joint_movement(5, 1, current_time)  # Future Joint 5
        # elif dpad_states['dpad_left']:
        #     self.handle_joint_movement(5, -1, current_time)  # Future Joint 5
    
    def handle_button_action(self, button_id, all_buttons, dpad_states, just_pressed=True):
        """Handle individual button presses."""
        current_time = time.time()
        
        # Special combinations (only on just_pressed)
        if just_pressed:
            # Emergency home: SELECT + START
            if (all_buttons[self.BUTTON_SELECT] and all_buttons[self.BUTTON_START]):
                self.get_logger().info("Emergency Stop - Moving to Home Position!")
                self.move_to_home()
                return
            
            # Status print: PS button
            if button_id == self.BUTTON_PS:
                self.print_status()
                return
        
        # Joint movement controls (only for arm joints 0-4)
        for joint_idx, controls in self.joint_controls.items():
            if joint_idx >= len(self.active_joint_names):
                continue  # Skip if joint doesn't exist
                
            if button_id == controls['positive']:
                self.handle_joint_movement(joint_idx, 1, all_buttons)
                return
            elif button_id == controls['negative']:
                self.handle_joint_movement(joint_idx, -1, all_buttons)
                return
    
    def handle_joint_movement(self, joint_idx, direction, all_buttons):
        """Handle movement for a specific joint."""
        if joint_idx >= len(self.active_joint_names):
            return
        
        # Check for fine control (any shoulder button pressed)
        fine_control = (all_buttons[self.BUTTON_L1] or all_buttons[self.BUTTON_L2] or 
                       all_buttons[self.BUTTON_R1] or all_buttons[self.BUTTON_R2])
        
        step = self.fine_movement_step if fine_control else self.movement_step
        delta = direction * step
        
        self.move_joint(joint_idx, delta)
    
    def move_joint(self, joint_index, delta):
        """Move a specific joint by delta amount."""
        if not self.joint_states_received:
            self.get_logger().warn("Joint states not received yet!")
            return
        
        if joint_index < 0 or joint_index >= len(self.active_joint_names):
            return
        
        # Get current positions
        current_positions = self.get_current_positions()
        target_positions = current_positions.copy()
        
        # Apply movement
        target_positions[joint_index] += delta
        
        # Safety limits (conservative) - adjust based on your robot
        joint_limits = [
            [-1.5, 1.5],   # Joint 0: shoulder_rotation
            [-1.2, 1.2],   # Joint 1: shoulder_pitch
            [-1.2, 1.2],   # Joint 2: elbow
            [-1.0, 1.0],   # Joint 3: wrist_pitch
            [-2.0, 2.0],   # Joint 4: wrist_roll
        ]
        
        if joint_index < len(joint_limits):
            min_pos, max_pos = joint_limits[joint_index]
            target_positions[joint_index] = max(min_pos, min(max_pos, target_positions[joint_index]))
        
        # Send movement command
        joint_name = self.active_joint_names[joint_index]
        self.get_logger().info(f"Moving {joint_name}: {current_positions[joint_index]:.3f} → {target_positions[joint_index]:.3f}")
        
        self.send_arm_command(target_positions)
    
    def send_arm_command(self, target_positions):
        """Send arm movement command."""
        current_positions = self.get_current_positions()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.active_joint_names
        
        # Start point
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.velocities = [0.0] * len(self.active_joint_names)
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        
        # End point
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.velocities = [0.0] * len(self.active_joint_names)
        end_point.time_from_start = Duration(sec=int(self.movement_duration), nanosec=0)
        
        trajectory.points = [start_point, end_point]
        
        # Send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.arm_action_client.send_goal_async(goal_msg)
    
    def control_gripper(self, target_position, current_time=None):
        """Control gripper position using dedicated gripper controller."""
        # Prevent rapid gripper commands
        if hasattr(self, 'last_gripper_command_time'):
            if current_time and (current_time - self.last_gripper_command_time) < 0.5:
                return
        
        current_pos = self.get_current_gripper_position()
        self.get_logger().info(f"Moving gripper: {current_pos:.3f} → {target_position:.3f}")
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = target_position
        goal_msg.command.max_effort = 50.0
        
        self.gripper_action_client.send_goal_async(goal_msg)
        
        if current_time:
            self.last_gripper_command_time = current_time
    
    def move_to_home(self):
        """Move arm to home position."""
        home_position = [0.0] * len(self.active_joint_names)
        self.send_arm_command(home_position)
        
        # Also reset gripper using dedicated controller
        self.control_gripper(0.0)
    
    def print_status(self):
        """Print current robot status."""
        if not self.joint_states_received:
            print("Joint states not available yet")
            return
        
        current_pos = self.get_current_positions()
        current_gripper = self.get_current_gripper_position()
        
        print("\n" + "="*70)
        print("CURRENT ROBOT STATUS")
        print("="*70)
        print("ARM JOINTS:")
        for i, (name, pos) in enumerate(zip(self.active_joint_names, current_pos)):
            controls = self.joint_controls.get(i, {})
            pos_btn = controls.get('positive', 'N/A')
            neg_btn = controls.get('negative', 'N/A')
            if isinstance(pos_btn, int):
                pos_btn = ['X','O','△','□','L1','R1','L2','R2','SEL','START','PS','L3','R3'][pos_btn]
            if isinstance(neg_btn, int):
                neg_btn = ['X','O','△','□','L1','R1','L2','R2','SEL','START','PS','L3','R3'][neg_btn]
            
            print(f"  Joint {i} ({name:<18}): {pos:7.3f} rad | Controls: {pos_btn}(+) / {neg_btn}(-)")
        
        print("")
        print("GRIPPER:")
        print(f"  {'gripper_joint':<25}: {current_gripper:7.3f} rad | Controls: ↑(open) / ↓(close)")
        
        print("")
        print(f"Active Arm Joints: {len(self.active_joint_names)}")
        print(f"Gripper Available: {'Yes' if self.gripper_joint_name in self.current_joint_positions else 'No'}")
        print("="*70)

def main(args=None):
    rclpy.init(args=args)
    
    print("Starting PS5 Arm Controller with Direct Joint Control...")
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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import json
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

class ArmCalibrationGUI(Node):
    """
    GUI for calibrating arm positions using sliders
    """
    
    def __init__(self):
        super().__init__('arm_calibration_gui')
        
        # ROS setup
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_action_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.latest_joint_state = None
        self.arm_joints = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
        
        # Joint limits (from your URDF)
        self.joint_limits = {
            'base_joint': (-1.92, 1.92),
            'shoulder_joint': (-1.75, 1.75),
            'elbow_joint': (-1.75, 1.57),
            'wrist_pitch_joint': (-1.66, 1.66),
            'wrist_roll_joint': (-2.79, 2.79),
            'gripper_joint': (-0.175, 1.75)
        }
        
        # Chess coordinate system
        self.setup_coordinate_mapping()
        
        # Saved positions
        self.saved_positions = {}
        self.load_saved_positions()
        
        # GUI setup
        self.setup_gui()
        
        # Wait for action servers
        self.get_logger().info('Waiting for action servers...')
        self.arm_action_client.wait_for_server()
        self.gripper_action_client.wait_for_server()
        self.get_logger().info('Action servers connected!')
        
        # Start ROS spinning in separate thread
        self.ros_thread = threading.Thread(target=self.spin_ros)
        self.ros_thread.daemon = True
        self.ros_thread.start()
    
    def setup_coordinate_mapping(self):
        """Setup chess coordinate system"""
        self.board_center_x = 0.05
        self.board_center_y = 0.0
        self.board_height = 0.8315
        self.square_size = 0.04
        
        self.board_min_x = self.board_center_x - 0.16
        self.board_max_x = self.board_center_x + 0.16
        self.board_min_y = self.board_center_y - 0.16
        self.board_max_y = self.board_center_y + 0.16
        
        # Robot position
        self.robot_x = 0.35
        self.robot_y = 0.0
        self.robot_z = 0.778
        
        # Create coordinate mapping
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
    
    def setup_gui(self):
        """Create the GUI interface"""
        self.root = tk.Tk()
        self.root.title("Chess Robot Arm Calibration")
        self.root.geometry("800x700")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Chess Robot Arm Calibration", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Joint sliders frame
        sliders_frame = ttk.LabelFrame(main_frame, text="Joint Positions", padding="10")
        sliders_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.joint_vars = {}
        self.joint_sliders = {}
        
        for i, joint in enumerate(self.arm_joints):
            # Joint label
            joint_label = ttk.Label(sliders_frame, text=f"{joint}:")
            joint_label.grid(row=i, column=0, sticky=tk.W, padx=(0, 10))
            
            # Value variable
            self.joint_vars[joint] = tk.DoubleVar(value=0.0)
            
            # Slider
            min_val, max_val = self.joint_limits[joint]
            slider = ttk.Scale(
                sliders_frame, 
                from_=min_val, 
                to=max_val, 
                orient=tk.HORIZONTAL, 
                length=300,
                variable=self.joint_vars[joint],
                command=lambda val, j=joint: self.on_slider_change(j, val)
            )
            slider.grid(row=i, column=1, padx=(0, 10))
            self.joint_sliders[joint] = slider
            
            # Value display
            value_label = ttk.Label(sliders_frame, text="0.000")
            value_label.grid(row=i, column=2, sticky=tk.W)
            setattr(self, f"{joint}_label", value_label)
        
        # Gripper control
        gripper_frame = ttk.LabelFrame(main_frame, text="Gripper Control", padding="10")
        gripper_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Button(gripper_frame, text="Open Gripper", command=self.open_gripper).grid(row=0, column=0, padx=(0, 10))
        ttk.Button(gripper_frame, text="Close Gripper", command=self.close_gripper).grid(row=0, column=1)
        
        # Control buttons
        control_frame = ttk.LabelFrame(main_frame, text="Movement Control", padding="10")
        control_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Button(control_frame, text="Move to Current Position", command=self.move_to_current).grid(row=0, column=0, padx=(0, 10))
        ttk.Button(control_frame, text="Read Current Position", command=self.read_current_position).grid(row=0, column=1, padx=(0, 10))
        ttk.Button(control_frame, text="Reset to Home", command=self.reset_to_home).grid(row=0, column=2)
        
        # Target square selection
        target_frame = ttk.LabelFrame(main_frame, text="Target Square", padding="10")
        target_frame.grid(row=4, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(target_frame, text="Square:").grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        
        self.target_square_var = tk.StringVar(value="a1")
        square_combo = ttk.Combobox(target_frame, textvariable=self.target_square_var, width=10)
        square_combo['values'] = [f"{f}{r}" for f in "abcdefgh" for r in "12345678"] + ["home", "safe_above"]
        square_combo.grid(row=0, column=1, padx=(0, 10))
        
        ttk.Button(target_frame, text="Show Target Coords", command=self.show_target_coords).grid(row=0, column=2)
        
        # Save/Load positions
        save_frame = ttk.LabelFrame(main_frame, text="Position Management", padding="10")
        save_frame.grid(row=5, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Button(save_frame, text="Save Current Position", command=self.save_current_position).grid(row=0, column=0, padx=(0, 10))
        ttk.Button(save_frame, text="Load Position", command=self.load_position).grid(row=0, column=1, padx=(0, 10))
        ttk.Button(save_frame, text="Save All to File", command=self.save_to_file).grid(row=0, column=2, padx=(0, 10))
        ttk.Button(save_frame, text="Load from File", command=self.load_from_file).grid(row=0, column=3)
        
        # Status display
        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(main_frame, textvariable=self.status_var, font=("Arial", 10))
        status_label.grid(row=6, column=0, columnspan=3, pady=(10, 0))
        
        # Update display initially
        self.update_displays()
    
    def joint_state_callback(self, msg):
        """Monitor current joint states"""
        self.latest_joint_state = msg
    
    def on_slider_change(self, joint_name, value):
        """Called when slider value changes"""
        self.update_displays()
    
    def update_displays(self):
        """Update all value displays"""
        for joint in self.arm_joints:
            value = self.joint_vars[joint].get()
            label = getattr(self, f"{joint}_label")
            label.config(text=f"{value:.3f} rad ({value*180/3.14159:.1f}°)")
    
    def get_current_slider_positions(self):
        """Get current slider positions"""
        return [self.joint_vars[joint].get() for joint in self.arm_joints]
    
    def set_slider_positions(self, positions):
        """Set slider positions"""
        for joint, pos in zip(self.arm_joints, positions):
            self.joint_vars[joint].set(pos)
        self.update_displays()
    
    def send_joint_goal(self, joint_positions, duration=3.0):
        """Send joint goal to robot"""
        if not self.arm_action_client.server_is_ready():
            self.status_var.set("❌ Arm action server not ready")
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.status_var.set("Moving arm...")
        self.get_logger().info(f'Sending joint positions: {[f"{p:.3f}" for p in joint_positions]}')
        
        # Send goal
        future = self.arm_action_client.send_goal_async(goal_msg)
        
        def goal_response_callback(future):
            goal_handle = future.result()
            if goal_handle.accepted:
                self.status_var.set("✅ Movement completed")
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(lambda f: self.status_var.set("✅ Ready"))
            else:
                self.status_var.set("❌ Goal rejected")
        
        future.add_done_callback(goal_response_callback)
        return True
    
    def move_to_current(self):
        """Move robot to current slider positions"""
        positions = self.get_current_slider_positions()
        self.send_joint_goal(positions)
    
    def read_current_position(self):
        """Read current robot position and update sliders"""
        if not self.latest_joint_state:
            self.status_var.set("❌ No joint state available")
            return
        
        positions = []
        for joint in self.arm_joints:
            if joint in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(joint)
                positions.append(self.latest_joint_state.position[idx])
            else:
                positions.append(0.0)
        
        self.set_slider_positions(positions)
        self.status_var.set("✅ Current position loaded")
    
    def reset_to_home(self):
        """Reset to home position"""
        home_positions = [0.0] * 5
        self.set_slider_positions(home_positions)
        self.send_joint_goal(home_positions)
    
    def open_gripper(self):
        """Open the gripper"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 1.5
        goal_msg.command.max_effort = 5.0
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        self.status_var.set("Opening gripper...")
        
        def done_callback(future):
            self.status_var.set("✅ Gripper opened")
        
        future.add_done_callback(done_callback)
    
    def close_gripper(self):
        """Close the gripper"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.2
        goal_msg.command.max_effort = 3.0
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        self.status_var.set("Closing gripper...")
        
        def done_callback(future):
            self.status_var.set("✅ Gripper closed")
        
        future.add_done_callback(done_callback)
    
    def show_target_coords(self):
        """Show target coordinates for selected square"""
        square = self.target_square_var.get()
        if square in self.chess_coords:
            coords = self.chess_coords[square]
            message = f"Square {square}:\nCoordinates: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})"
            distance = ((coords[0] - self.robot_x)**2 + (coords[1] - self.robot_y)**2)**0.5
            message += f"\nDistance from robot: {distance:.3f}m"
        else:
            message = f"No coordinates defined for {square}"
        
        messagebox.showinfo("Target Coordinates", message)
    
    def save_current_position(self):
        """Save current slider position for target square"""
        square = self.target_square_var.get()
        positions = self.get_current_slider_positions()
        self.saved_positions[square] = positions
        self.status_var.set(f"✅ Position saved for {square}")
        self.get_logger().info(f'Saved position for {square}: {[f"{p:.3f}" for p in positions]}')
    
    def load_position(self):
        """Load saved position for target square"""
        square = self.target_square_var.get()
        if square in self.saved_positions:
            self.set_slider_positions(self.saved_positions[square])
            self.status_var.set(f"✅ Position loaded for {square}")
        else:
            self.status_var.set(f"❌ No saved position for {square}")
    
    def save_to_file(self):
        """Save all positions to JSON file"""
        filename = filedialog.asksaveasfilename(
            title="Save Positions",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.saved_positions, f, indent=2)
                self.status_var.set(f"✅ Saved to {filename}")
            except Exception as e:
                self.status_var.set(f"❌ Save failed: {e}")
    
    def load_from_file(self):
        """Load positions from JSON file"""
        filename = filedialog.askopenfilename(
            title="Load Positions",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    self.saved_positions = json.load(f)
                self.status_var.set(f"✅ Loaded from {filename}")
            except Exception as e:
                self.status_var.set(f"❌ Load failed: {e}")
    
    def load_saved_positions(self):
        """Load previously saved positions"""
        try:
            with open('arm_positions.json', 'r') as f:
                self.saved_positions = json.load(f)
            self.get_logger().info(f'Loaded {len(self.saved_positions)} saved positions')
        except FileNotFoundError:
            self.saved_positions = {}
            self.get_logger().info('No saved positions file found')
        except Exception as e:
            self.saved_positions = {}
            self.get_logger().error(f'Failed to load positions: {e}')
    
    def spin_ros(self):
        """Spin ROS in separate thread"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def run(self):
        """Run the GUI"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            # Save positions on exit
            try:
                with open('arm_positions.json', 'w') as f:
                    json.dump(self.saved_positions, f, indent=2)
            except Exception as e:
                self.get_logger().error(f'Failed to save positions: {e}')


def main():
    rclpy.init()
    
    try:
        gui = ArmCalibrationGUI()
        gui.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'gui' in locals():
            gui.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

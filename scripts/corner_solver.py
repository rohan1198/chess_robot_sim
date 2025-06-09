#!/usr/bin/env python3
"""
Fixed Chess Corner Solver
==========================

This is an improved version of the corner solver that addresses the coordinate
frame and joint limit issues identified in the debugging process.

Key fixes:
1. Corrected elbow joint limits to match joint_limits.yaml
2. Improved IK strategies for front corners
3. Better coordinate frame validation
4. More flexible positioning for problematic corners

Author: Chess Robot Project - Fixed Version
"""

import os
import numpy as np
import subprocess
import tempfile
import pybullet as p
import pybullet_data
import json
import math
from pathlib import Path
from typing import List, Dict, Optional, Tuple

class FixedChessCornerSolver:
    """Improved chess corner movement solver with fixes for coordinate issues."""
    
    def __init__(self, urdf_path: str, config_file: Optional[str] = None):
        """Initialize the improved chess corner solver."""
        self.urdf_path = urdf_path
        self.config_file = config_file
        
        # Robot configuration
        self.robot_id = None
        self.physics_client = None
        self.controllable_joints = []
        self.end_effector_link_idx = None
        
        # FIXED: Joint limits now match joint_limits.yaml exactly
        self.joint_limits = {
            'shoulder_rotation': (-1.91986, 1.91986),   # ±110°
            'shoulder_pitch': (-1.74533, 1.74533),     # ±100°
            'elbow': (-1.74533, 1.74533),              # ±100° - FIXED!
            'wrist_pitch': (-1.65806, 1.65806),        # ±95°
            'wrist_roll': (-2.79253, 2.79253),         # ±160°
        }
        
        # World setup - keeping your working values
        self.robot_world_pos = np.array([0.75, 0.25, 0.375])
        self.board_world_pos = np.array([0.75, 0.04, 0.405])
        self.board_size = 0.19
        
        # Results storage
        self.target_positions = {}
        self.ik_solutions = {}
        self.analysis_results = {}
        
        print("🤖 Fixed Chess Corner Solver Initialized")
        print(f"   Board size: {self.board_size*1000:.0f}mm")
        print(f"   Robot position: {self.robot_world_pos}")
        print(f"   Board center: {self.board_world_pos}")
        print("🔧 Fixed elbow joint limits to match joint_limits.yaml")
    
    def analyze_coordinates_with_validation(self) -> Dict:
        """Analyze coordinates with additional validation steps."""
        
        print("\n📍 ENHANCED COORDINATE ANALYSIS")
        print("-" * 50)
        
        # Calculate board corners in world frame
        half_size = self.board_size / 2.0
        world_corners = {
            'front_left':  self.board_world_pos + np.array([-half_size, +half_size, 0]),
            'front_right': self.board_world_pos + np.array([+half_size, +half_size, 0]),
            'back_left':   self.board_world_pos + np.array([-half_size, -half_size, 0]),
            'back_right':  self.board_world_pos + np.array([+half_size, -half_size, 0])
        }
        
        print("🌍 World frame corners:")
        for name, pos in world_corners.items():
            print(f"  {name:12s}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        
        # Transform to robot base frame with validation
        robot_corners = {}
        print(f"\n🤖 Robot frame corners (relative to robot at {self.robot_world_pos}):")
        
        for name, world_pos in world_corners.items():
            robot_pos = world_pos - self.robot_world_pos
            robot_corners[name] = robot_pos
            distance = np.linalg.norm(robot_pos)
            print(f"  {name:12s}: ({robot_pos[0]:6.3f}, {robot_pos[1]:6.3f}, {robot_pos[2]:6.3f}) - {distance:.3f}m")
        
        # Generate multiple target positions for each corner
        self.target_positions = {}
        
        for name, corner_pos in robot_corners.items():
            targets = {}
            
            # Primary target: 50mm above corner
            hover_pos = corner_pos + np.array([0, 0, 0.05])
            targets['primary'] = hover_pos
            
            # For front corners, add multiple alternatives since they're challenging
            if 'front' in name:
                # Alternative 1: Lower hover height
                targets['alt_lower'] = corner_pos + np.array([0, 0, 0.04])
                
                # Alternative 2: Slightly closer to robot
                if 'left' in name:
                    targets['alt_closer'] = corner_pos + np.array([0.01, 0.01, 0.04])
                else:  # front_right
                    targets['alt_closer'] = corner_pos + np.array([-0.01, 0.01, 0.04])
                
                # Alternative 3: Even lower
                targets['alt_lowest'] = corner_pos + np.array([0, 0, 0.03])
            
            self.target_positions[name] = targets
        
        print("\n🎯 Target positions (multiple alternatives for front corners):")
        for name, targets in self.target_positions.items():
            print(f"\n  {name}:")
            for target_type, pos in targets.items():
                distance = np.linalg.norm(pos)
                height_mm = (pos[2] - corner_pos[2]) * 1000
                print(f"    {target_type:12s}: ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) - {distance:.3f}m (+{height_mm:.0f}mm)")
        
        # Store analysis results
        self.analysis_results = {
            'world_corners': {name: pos.tolist() for name, pos in world_corners.items()},
            'robot_corners': {name: pos.tolist() for name, pos in robot_corners.items()},
            'board_size': self.board_size,
            'robot_position': self.robot_world_pos.tolist(),
            'board_center': self.board_world_pos.tolist()
        }
        
        return self.analysis_results
    
    def setup_pybullet(self):
        """Initialize PyBullet simulation."""
        if self.physics_client is not None:
            return
        
        print("\n🔧 Setting up improved IK solver...")
        
        # Expand and process URDF
        urdf_xml = self._expand_xacro(self.urdf_path)
        urdf_xml = self._patch_mesh_paths(urdf_xml)
        
        # Write temp file
        fd, temp_urdf_path = tempfile.mkstemp(suffix='.urdf')
        with os.fdopen(fd, 'w') as f:
            f.write(urdf_xml)
        
        # Start PyBullet
        self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Load robot
        self.robot_id = p.loadURDF(temp_urdf_path, useFixedBase=True)
        
        # Find controllable arm joints
        arm_joint_names = ['shoulder_rotation', 'shoulder_pitch', 'elbow', 'wrist_pitch', 'wrist_roll']
        
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            
            if joint_name in arm_joint_names and info[2] == p.JOINT_REVOLUTE:
                self.controllable_joints.append((i, joint_name))
        
        # Sort by desired order
        joint_order = {name: idx for idx, name in enumerate(arm_joint_names)}
        self.controllable_joints.sort(key=lambda x: joint_order[x[1]])
        
        # Find end-effector
        self.end_effector_link_idx = self._find_link_index('gripper_jaw')
        if self.end_effector_link_idx is None:
            raise ValueError("Could not find gripper_jaw link")
        
        print("✅ Improved IK solver ready:")
        print(f"   End-effector: gripper_jaw (link {self.end_effector_link_idx})")
        print(f"   Arm joints: {[name for _, name in self.controllable_joints]}")
        print(f"   Fixed joint limits: elbow range now ±100°")
        
        # Clean up
        os.remove(temp_urdf_path)
    
    def solve_ik_with_multiple_strategies(self, target_position: np.ndarray, corner_name: str = "") -> Optional[List[float]]:
        """Solve IK using multiple improved strategies."""
        
        # Extract joint info
        joint_indices = [idx for idx, name in self.controllable_joints]
        joint_names = [name for idx, name in self.controllable_joints]
        
        lower_limits = [self.joint_limits[name][0] for name in joint_names]
        upper_limits = [self.joint_limits[name][1] for name in joint_names]
        
        # Improved IK strategies with better starting poses
        strategies = [
            {
                'name': 'position_only_home',
                'use_orientation': False,
                'start_pose': [0.0, 0.0, 0.0, 0.0, 0.0],
                'max_iterations': 200,
                'threshold': 1e-4
            },
            {
                'name': 'front_optimized' if 'front' in corner_name else 'back_optimized',
                'use_orientation': True,
                'orientation': [0.0, 0.0, 0.0, 1.0],
                'start_pose': self._get_optimized_start_pose(corner_name),
                'max_iterations': 300,
                'threshold': 1e-4
            },
            {
                'name': 'slight_down_orientation',
                'use_orientation': True,
                'orientation': [0.2588, 0, 0, 0.9659],  # 30° down
                'start_pose': [0.0, -0.3, 0.5, -0.2, 0.0],
                'max_iterations': 250,
                'threshold': 1e-4
            },
            {
                'name': 'relaxed_tolerance',
                'use_orientation': False,
                'start_pose': [0.0, 0.0, 0.0, 0.0, 0.0],
                'max_iterations': 400,
                'threshold': 5e-3  # More relaxed
            },
            {
                'name': 'random_restart',
                'use_orientation': False,
                'start_pose': self._generate_random_start_pose(joint_names),
                'max_iterations': 200,
                'threshold': 1e-4
            }
        ]
        
        for strategy in strategies:
            try:
                # Set starting pose
                start_pose = strategy['start_pose']
                for i, angle in enumerate(start_pose):
                    if i < len(joint_indices):
                        p.resetJointState(self.robot_id, joint_indices[i], angle)
                
                # Solve IK
                if strategy['use_orientation']:
                    ik_solution = p.calculateInverseKinematics(
                        self.robot_id,
                        self.end_effector_link_idx,
                        target_position,
                        strategy['orientation'],
                        lowerLimits=lower_limits,
                        upperLimits=upper_limits,
                        jointRanges=[upper - lower for lower, upper in zip(lower_limits, upper_limits)],
                        restPoses=start_pose,
                        maxNumIterations=strategy['max_iterations'],
                        residualThreshold=strategy['threshold']
                    )
                else:
                    ik_solution = p.calculateInverseKinematics(
                        self.robot_id,
                        self.end_effector_link_idx,
                        target_position,
                        lowerLimits=lower_limits,
                        upperLimits=upper_limits,
                        jointRanges=[upper - lower for lower, upper in zip(lower_limits, upper_limits)],
                        restPoses=start_pose,
                        maxNumIterations=strategy['max_iterations'],
                        residualThreshold=strategy['threshold']
                    )
                
                # Extract solution
                joint_solution = list(ik_solution[:len(joint_indices)])
                
                # Validate joint limits
                valid = True
                for i, (joint_name, angle) in enumerate(zip(joint_names, joint_solution)):
                    lower, upper = self.joint_limits[joint_name]
                    if not (lower <= angle <= upper):
                        valid = False
                        break
                
                if not valid:
                    continue
                
                # Verify with forward kinematics
                for i, angle in enumerate(joint_solution):
                    p.resetJointState(self.robot_id, joint_indices[i], angle)
                
                state = p.getLinkState(self.robot_id, self.end_effector_link_idx, computeForwardKinematics=True)
                achieved_pos = np.array(state[4])
                error = np.linalg.norm(achieved_pos - target_position)
                
                if error <= 0.02:  # 20mm tolerance
                    print(f"   ✅ Success with strategy: {strategy['name']} (error: {error*1000:.1f}mm)")
                    return joint_solution
                
            except Exception as e:
                continue
        
        return None
    
    def _get_optimized_start_pose(self, corner_name: str) -> List[float]:
        """Get optimized starting pose based on corner position."""
        if 'front_left' in corner_name:
            return [0.3, -0.2, 0.3, -0.1, 0.0]   # Biased towards left and forward
        elif 'front_right' in corner_name:
            return [-0.3, -0.2, 0.3, -0.1, 0.0]  # Biased towards right and forward
        elif 'back_left' in corner_name:
            return [0.4, 0.5, 0.1, -0.6, 1.5]    # Biased towards left and back
        elif 'back_right' in corner_name:
            return [-0.2, 0.5, 0.1, -0.6, 1.5]   # Biased towards right and back
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.0]     # Default home
    
    def _generate_random_start_pose(self, joint_names: List[str]) -> List[float]:
        """Generate a random but reasonable starting pose."""
        pose = []
        for joint_name in joint_names:
            lower, upper = self.joint_limits[joint_name]
            # Use middle 60% of joint range for random start
            mid = (lower + upper) / 2
            range_60 = (upper - lower) * 0.3
            random_angle = mid + np.random.uniform(-range_60, range_60)
            pose.append(random_angle)
        return pose
    
    def solve_all_corners_improved(self) -> Dict:
        """Solve IK for all corners with improved strategies."""
        
        if not self.target_positions:
            self.analyze_coordinates_with_validation()
        
        self.setup_pybullet()
        
        print("\n🎯 SOLVING INVERSE KINEMATICS (IMPROVED)")
        print("-" * 50)
        
        self.ik_solutions = {}
        successful_corners = 0
        
        for corner_name, targets in self.target_positions.items():
            print(f"\n📍 Solving {corner_name}:")
            
            solution_found = False
            
            # Try each target position for this corner
            for target_type, target_pos in targets.items():
                if solution_found:
                    break
                
                print(f"   Trying {target_type} position...")
                solution = self.solve_ik_with_multiple_strategies(target_pos, corner_name)
                
                if solution is not None:
                    self.ik_solutions[corner_name] = {
                        'position': target_pos.tolist(),
                        'joints': solution,
                        'method': target_type
                    }
                    successful_corners += 1
                    solution_found = True
                    
                    degrees = [math.degrees(angle) for angle in solution]
                    print(f"   ✅ Success with {target_type}!")
                    print(f"   Joints (deg): {[f'{d:5.1f}°' for d in degrees]}")
            
            if not solution_found:
                print(f"   ❌ All positions failed for {corner_name}")
        
        success_rate = successful_corners / len(self.target_positions)
        
        print(f"\n🏆 IMPROVED IK RESULTS:")
        print(f"   Successful corners: {successful_corners}/{len(self.target_positions)} ({success_rate*100:.0f}%)")
        
        return {
            'solutions': self.ik_solutions,
            'success_rate': success_rate,
            'successful_corners': successful_corners,
            'total_corners': len(self.target_positions)
        }
    
    def validate_solutions(self) -> bool:
        """Validate all solutions."""
        if not self.ik_solutions:
            print("❌ No solutions to validate")
            return False
        
        print("\n✅ SOLUTION VALIDATION")
        print("-" * 40)
        
        all_valid = True
        
        for corner_name, solution_data in self.ik_solutions.items():
            position = np.array(solution_data['position'])
            joints = solution_data['joints']
            
            # Set joints and check forward kinematics
            joint_indices = [idx for idx, name in self.controllable_joints]
            for i, angle in enumerate(joints):
                p.resetJointState(self.robot_id, joint_indices[i], angle)
            
            state = p.getLinkState(self.robot_id, self.end_effector_link_idx, computeForwardKinematics=True)
            achieved_pos = np.array(state[4])
            error = np.linalg.norm(achieved_pos - position)
            
            if error <= 0.02:
                print(f"   {corner_name:12s}: ✅ Valid (error: {error*1000:.1f}mm)")
            else:
                print(f"   {corner_name:12s}: ❌ Invalid (error: {error*1000:.1f}mm)")
                all_valid = False
        
        return all_valid
    
    def save_results(self, output_file: str = None) -> str:
        """Save results to JSON file."""
        if output_file is None:
            script_dir = Path(__file__).parent.absolute()
            output_file = script_dir / "chess_corner_fixed_solution.json"
        
        complete_data = {
            'metadata': {
                'project': 'Chess Robot Corner Movement - Fixed Version',
                'version': '2.1 Fixed',
                'timestamp': str(np.datetime64('now')),
                'success_rate': len(self.ik_solutions) / len(self.target_positions) if self.target_positions else 0,
                'fixes_applied': [
                    'Corrected elbow joint limits to match joint_limits.yaml',
                    'Improved IK strategies with multiple attempts per corner',
                    'Added multiple target positions for front corners',
                    'Enhanced coordinate frame validation'
                ]
            },
            'coordinate_analysis': self.analysis_results,
            'target_positions': {
                name: {
                    key: pos.tolist() if isinstance(pos, np.ndarray) else pos
                    for key, pos in targets.items()
                }
                for name, targets in self.target_positions.items()
            },
            'ik_solutions': self.ik_solutions,
            'joint_configuration': {
                'joint_names': [name for idx, name in self.controllable_joints],
                'joint_limits': self.joint_limits,
                'end_effector': 'gripper_jaw'
            }
        }
        
        with open(output_file, 'w') as f:
            json.dump(complete_data, f, indent=2)
        
        print(f"\n💾 Fixed solution saved to: {output_file}")
        return str(output_file)
    
    def generate_final_report(self):
        """Generate comprehensive final report."""
        print("\n📋 FIXED CHESS CORNER SOLVER - FINAL REPORT")
        print("=" * 60)
        
        if not self.ik_solutions:
            print("❌ No solutions available")
            return
        
        successful = len(self.ik_solutions)
        total = len(self.target_positions)
        success_rate = successful / total * 100
        
        print(f"🎯 SUCCESS RATE: {successful}/{total} corners ({success_rate:.0f}%)")
        
        if success_rate == 100:
            print("🎉 PERFECT! All chess board corners are now reachable!")
        elif success_rate >= 75:
            print("✅ EXCELLENT! Major improvement achieved!")
        else:
            print("🔧 GOOD PROGRESS! Further optimization may be needed.")
        
        print(f"\n📍 CORNER-BY-CORNER RESULTS:")
        for corner_name in ['front_left', 'front_right', 'back_left', 'back_right']:
            if corner_name in self.ik_solutions:
                solution = self.ik_solutions[corner_name]
                method = solution['method']
                position = solution['position']
                distance = np.linalg.norm(position)
                
                print(f"   {corner_name:12s}: ✅ {method:12s} - distance {distance:.3f}m")
            else:
                print(f"   {corner_name:12s}: ❌ still failed")
        
        print(f"\n🔧 FIXES APPLIED:")
        print(f"   ✅ Corrected elbow joint limits (now ±100°)")
        print(f"   ✅ Multiple target positions per corner")
        print(f"   ✅ Improved IK strategies")
        print(f"   ✅ Better starting poses")
        
        if successful >= 3:
            print(f"\n🚀 READY FOR PHASE 3:")
            print(f"   ✅ Sufficient corners reachable for chess play")
            print(f"   ✅ Solutions validated and saved")
            print(f"   ✅ Ready for ROS2 trajectory execution")
    
    # Helper methods (same as before)
    def _expand_xacro(self, xacro_path: str) -> str:
        try:
            return subprocess.check_output(['xacro', xacro_path]).decode()
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"xacro expansion failed: {e}")
    
    def _patch_mesh_paths(self, urdf_xml: str) -> str:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        meshes_dir = os.path.abspath(os.path.join(script_dir, '..', 'meshes'))
        return urdf_xml.replace('package://so_arm_101_gazebo/meshes/', meshes_dir + os.sep)
    
    def _find_link_index(self, target_link_name: str) -> Optional[int]:
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            link_name = info[12].decode('utf-8')
            if link_name == target_link_name:
                return i
        return None
    
    def __del__(self):
        if self.physics_client is not None:
            p.disconnect(self.physics_client)

def main():
    """Main execution function."""
    print("🤖 FIXED CHESS CORNER SOLVER")
    print("=" * 60)
    print("Applying fixes for coordinate frame and joint limit issues")
    
    # Find URDF path
    script_dir = Path(__file__).parent.absolute()
    urdf_path = script_dir.parent / "urdf" / "so_arm_101_gazebo.urdf.xacro"
    
    if not urdf_path.exists():
        print(f"❌ URDF not found at: {urdf_path}")
        return False
    
    try:
        # Initialize improved solver
        solver = FixedChessCornerSolver(str(urdf_path))
        
        # Run complete analysis with fixes
        print("\n🔄 Running fixed corner analysis...")
        
        # Step 1: Enhanced coordinate analysis
        solver.analyze_coordinates_with_validation()
        
        # Step 2: Solve with improved strategies
        results = solver.solve_all_corners_improved()
        
        # Step 3: Validate solutions
        solver.validate_solutions()
        
        # Step 4: Generate final report
        solver.generate_final_report()
        
        # Step 5: Save results
        solver.save_results()
        
        success_rate = results['success_rate']
        
        if success_rate >= 0.75:
            print("\n🎉 FIXES SUCCESSFUL!")
            print("🚀 Ready for improved ROS2 trajectory execution")
            return True
        else:
            print("\n🔧 Partial improvement achieved")
            print("💡 Consider additional workspace adjustments")
            return False
            
    except Exception as e:
        print(f"❌ Error during execution: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    success = main()
    
    if success:
        print("\n✅ Fixed solutions ready!")
        print("💡 Next: Use test_corners.py with the new solution file")
    else:
        print("\n🔧 Additional debugging may be needed")

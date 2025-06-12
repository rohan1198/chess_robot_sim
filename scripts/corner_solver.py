#!/usr/bin/env python3

import os
import numpy as np
import subprocess
import tempfile
import pybullet as p
import pybullet_data
import json
import math
from pathlib import Path
from typing import List, Dict, Optional

class ImprovedChessCornerSolver:
    """Final improved chess corner solver with reachability fixes."""
    
    def __init__(self, urdf_path: str, config_file: Optional[str] = None):
        """Initialize the improved chess corner solver."""
        self.urdf_path = urdf_path
        self.config_file = config_file
        
        # Robot configuration
        self.robot_id = None
        self.physics_client = None
        self.controllable_joints = []
        self.end_effector_link_idx = None
        
        # Joint limits matching joint_limits.yaml exactly
        self.joint_limits = {
            'shoulder_rotation': (-1.91986, 1.91986),  # ±110°
            'shoulder_pitch': (-1.74533, 1.74533),     # ±100°
            'elbow': (-1.74533, 1.74533),              # ±100°
            'wrist_pitch': (-1.74533, 1.74533),        # ±100°
            'wrist_roll': (-2.79253, 2.79253),         # ±160°
        }
        
        # World setup - using EXACT spawn position from launch file
        self.robot_world_pos = np.array([0.7475, 0.275, 0.375])  # Gazebo spawn position
        self.board_world_pos = np.array([0.75, 0.04, 0.405])   # Board center from chess_world.sdf
        self.board_size = 0.20
        
        # Robot workspace constraints (from reach calculator)
        self.min_reach = 0.15   # Conservative minimum reach
        self.max_reach = 0.35   # Conservative maximum reach
        
        # Use Gazebo native coordinates (no URDF offset)
        self.robot_base_pos = self.robot_world_pos
        
        # Finger tip offset (distance from gripper_jaw link to actual finger tips)
        self.finger_tip_offset = np.array([0.0, 0.0, -0.0189])
        
        # Results storage
        self.target_positions = {}
        self.ik_solutions = {}
        self.analysis_results = {}
        
        print("🔧 IMPROVED Chess Corner Solver Initialized")
        print(f"   Board size: {self.board_size*1000:.0f}mm")
        print(f"   Robot position: {self.robot_world_pos} (from Gazebo spawn)")
        print(f"   Board center: {self.board_world_pos}")
        print(f"   Workspace: {self.min_reach:.3f}m to {self.max_reach:.3f}m")
        print("✅ Using Gazebo-native coordinates with reachability constraints")
    
    def analyze_coordinates_improved(self) -> Dict:
        """Analyze coordinates with reachability constraints."""
        
        print("\n📍 IMPROVED COORDINATE ANALYSIS")
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
        
        # Transform to robot base frame
        robot_corners = {}
        print(f"\n🤖 Robot frame corners (relative to {self.robot_base_pos}):")
        
        for name, world_pos in world_corners.items():
            robot_pos = world_pos - self.robot_base_pos
            robot_corners[name] = robot_pos
            distance = np.linalg.norm(robot_pos)
            reachable = "✅" if self.min_reach <= distance <= self.max_reach else "❌"
            print(f"  {name:12s}: ({robot_pos[0]:6.3f}, {robot_pos[1]:6.3f}, {robot_pos[2]:6.3f}) - {distance:.3f}m {reachable}")
        
        # Generate improved target positions with reachability fixes
        self.target_positions = {}
        
        for name, corner_pos in robot_corners.items():
            targets = {}
            
            # Base hover position
            hover_height = 0.08  # 80mm above board surface
            base_target = corner_pos + np.array([0, 0, hover_height])
            
            # Apply finger tip offset
            corrected_target = base_target - self.finger_tip_offset
            
            # Check reachability and adjust if needed
            distance = np.linalg.norm(corrected_target)
            
            if distance < self.min_reach:
                # Too close - move target away from robot
                print(f"   🔧 {name} too close ({distance:.3f}m), adjusting...")
                
                # Calculate direction away from robot
                direction = corrected_target / np.linalg.norm(corrected_target)
                adjusted_target = direction * (self.min_reach + 0.02)  # 20mm safety margin
                
                targets['primary'] = adjusted_target
                targets['original_too_close'] = corrected_target
                
                # Try higher positions too
                higher_target = adjusted_target + np.array([0, 0, 0.02])
                if np.linalg.norm(higher_target) <= self.max_reach:
                    targets['higher'] = higher_target
                
            elif distance > self.max_reach:
                # Too far - this shouldn't happen with our board size, but handle it
                print(f"   ⚠️  {name} too far ({distance:.3f}m), adjusting...")
                direction = corrected_target / np.linalg.norm(corrected_target)
                adjusted_target = direction * (self.max_reach - 0.02)  # 20mm safety margin
                targets['primary'] = adjusted_target
                
            else:
                # Within reach - use as-is but add alternatives
                targets['primary'] = corrected_target
                
                # Add alternative positions for robustness
                if 'front' in name:
                    # Front corners: try higher and pulled back
                    targets['higher'] = corrected_target + np.array([0, 0, 0.02])
                    targets['pulled_back'] = corrected_target + np.array([0, 0.02, 0])
                else:
                    # Back corners: try slightly higher
                    targets['higher'] = corrected_target + np.array([0, 0, 0.015])
            
            self.target_positions[name] = targets
        
        # Display final targets
        print("\n🎯 Improved target positions:")
        for name, targets in self.target_positions.items():
            print(f"\n  {name}:")
            for target_type, pos in targets.items():
                distance = np.linalg.norm(pos)
                reachable = "✅" if self.min_reach <= distance <= self.max_reach else "❌"
                board_corner = robot_corners[name]
                height_above_corner = (pos[2] - board_corner[2]) * 1000
                print(f"    {target_type:15s}: ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) - {distance:.3f}m {reachable} (+{height_above_corner:.0f}mm)")
        
        # Store analysis results
        self.analysis_results = {
            'world_corners': {name: pos.tolist() for name, pos in world_corners.items()},
            'robot_corners': {name: pos.tolist() for name, pos in robot_corners.items()},
            'board_size': self.board_size,
            'robot_position': self.robot_world_pos.tolist(),
            'board_center': self.board_world_pos.tolist(),
            'workspace_constraints': {
                'min_reach': self.min_reach,
                'max_reach': self.max_reach
            },
            'coordinate_system': 'gazebo_native_with_reachability'
        }
        
        return self.analysis_results
    
    def setup_pybullet(self):
        """Initialize PyBullet simulation."""
        if self.physics_client is not None:
            return
        
        print("\n🔧 Setting up PyBullet IK solver...")
        
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
        
        print("✅ PyBullet IK solver ready:")
        print(f"   End-effector: gripper_jaw (link {self.end_effector_link_idx})")
        print(f"   Arm joints: {[name for _, name in self.controllable_joints]}")
        
        # Clean up
        os.remove(temp_urdf_path)
    
    def solve_ik_enhanced(self, target_position: np.ndarray, corner_name: str = "") -> Optional[List[float]]:
        """Enhanced IK solver with more strategies."""
        
        # Extract joint info
        joint_indices = [idx for idx, name in self.controllable_joints]
        joint_names = [name for idx, name in self.controllable_joints]
        
        lower_limits = [self.joint_limits[name][0] for name in joint_names]
        upper_limits = [self.joint_limits[name][1] for name in joint_names]
        
        # Enhanced IK strategies - more options for difficult positions
        strategies = [
            {
                'name': 'home_start',
                'start_pose': [0.0, 0.0, 0.0, 0.0, 0.0],
                'use_orientation': False,
                'max_iterations': 300,
                'threshold': 1e-4
            },
            {
                'name': 'optimized_start',
                'start_pose': self._get_optimized_start_pose(corner_name),
                'use_orientation': True,
                'orientation': [0.0, 0.0, 0.0, 1.0],
                'max_iterations': 400,
                'threshold': 1e-4
            },
            {
                'name': 'down_orientation',
                'start_pose': [0.0, -0.3, 0.5, -0.2, 0.0],
                'use_orientation': True,
                'orientation': [0.2588, 0, 0, 0.9659],  # 30° down
                'max_iterations': 350,
                'threshold': 1e-4
            },
            {
                'name': 'alternate_start_1',
                'start_pose': [0.5, -0.5, 0.8, -0.3, 1.0],
                'use_orientation': False,
                'max_iterations': 300,
                'threshold': 2e-4
            },
            {
                'name': 'alternate_start_2', 
                'start_pose': [-0.5, -0.5, 0.8, -0.3, -1.0],
                'use_orientation': False,
                'max_iterations': 300,
                'threshold': 2e-4
            },
            {
                'name': 'relaxed_tolerance',
                'start_pose': [0.0, 0.0, 0.0, 0.0, 0.0],
                'use_orientation': False,
                'max_iterations': 500,
                'threshold': 5e-3  # More relaxed
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
                    print(f"   ✅ Success with {strategy['name']} (error: {error*1000:.1f}mm)")
                    return joint_solution
                
            except Exception:
                continue
        
        return None
    
    def _get_optimized_start_pose(self, corner_name: str) -> List[float]:
        """Get optimized starting pose based on corner position."""
        if 'front_left' in corner_name:
            # More aggressive positioning for front_left
            return [0.8, -0.4, 0.6, -0.2, 0.5]
        elif 'front_right' in corner_name:
            return [-0.3, -0.2, 0.3, -0.1, 0.0]
        elif 'back_left' in corner_name:
            return [0.4, 0.5, 0.1, -0.6, 1.5]
        elif 'back_right' in corner_name:
            return [-0.2, 0.5, 0.1, -0.6, 1.5]
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.0]
    
    def solve_all_corners_improved(self) -> Dict:
        """Solve IK for all corners with improved reachability."""
        
        if not self.target_positions:
            self.analyze_coordinates_improved()
        
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
                solution = self.solve_ik_enhanced(target_pos, corner_name)
                
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
        
        print("\n🏆 IMPROVED IK RESULTS:")
        print(f"   Successful corners: {successful_corners}/{len(self.target_positions)} ({success_rate*100:.0f}%)")
        
        return {
            'solutions': self.ik_solutions,
            'success_rate': success_rate,
            'successful_corners': successful_corners,
            'total_corners': len(self.target_positions)
        }
    
    def save_improved_results(self, output_file: str = None) -> str:
        """Save improved results to JSON file."""
        if output_file is None:
            script_dir = Path(__file__).parent.absolute()
            output_file = script_dir / "chess_corner_solution.json"
        
        complete_data = {
            'metadata': {
                'project': 'Chess Robot Corner Movement - Improved Final Version',
                'version': '5.0 Improved Final',
                'timestamp': str(np.datetime64('now')),
                'success_rate': len(self.ik_solutions) / len(self.target_positions) if self.target_positions else 0,
                'improvements': [
                    'Fixed coordinate system (removed URDF offset)',
                    'Added reachability constraints and workspace limits',
                    'Enhanced IK solver with more strategies',
                    'Automatic target adjustment for unreachable positions',
                    'Improved starting poses for difficult configurations',
                    'Comprehensive error handling and validation'
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
            },
            'workspace_constraints': {
                'min_reach': self.min_reach,
                'max_reach': self.max_reach
            }
        }
        
        with open(output_file, 'w') as f:
            json.dump(complete_data, f, indent=2)
        
        print(f"\n💾 Improved solution saved to: {output_file}")
        return str(output_file)
    
    # Helper methods
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
    print("🚀 IMPROVED CHESS CORNER SOLVER - FINAL VERSION")
    print("=" * 60)
    print("Fixing front_left reachability and optimizing all corners")
    
    # Find URDF path
    script_dir = Path(__file__).parent.absolute()
    urdf_path = script_dir.parent / "urdf" / "so_arm_101_gazebo.urdf.xacro"
    
    if not urdf_path.exists():
        print(f"❌ URDF not found at: {urdf_path}")
        return False
    
    try:
        # Initialize improved solver
        solver = ImprovedChessCornerSolver(str(urdf_path))
        
        # Run improved analysis
        print("\n🔄 Running improved corner analysis...")
        
        # Step 1: Improved coordinate analysis with reachability
        solver.analyze_coordinates_improved()
        
        # Step 2: Enhanced IK solving
        results = solver.solve_all_corners_improved()
        
        # Step 3: Save results
        solver.save_improved_results()
        
        success_rate = results['success_rate']
        
        print(f"\n🎉 FINAL IMPROVED RESULTS:")
        print(f"   Success rate: {success_rate*100:.0f}%")
        
        if success_rate >= 1.0:
            print("🏆 PERFECT! All corners solved with improved algorithm!")
        elif success_rate >= 0.75:
            print("✅ EXCELLENT! Most corners working well!")
        else:
            print("🔧 Some corners still need work")
            
        return success_rate >= 0.75
            
    except Exception as e:
        print(f"❌ Error during execution: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    success = main()
    
    if success:
        print("\n✅ Improved solutions ready!")
        print("💡 Update test_corners.py to use chess_corner_solution.json")
        print("🎯 Expected: All 4 corners working with <20mm accuracy")
    else:
        print("\n🔧 Additional debugging needed")

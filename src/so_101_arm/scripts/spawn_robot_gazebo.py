#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os
import re
from ament_index_python.packages import get_package_share_directory

class IgnitionGazeboRobotSpawner(Node):
    def __init__(self):
        super().__init__('ignition_gazebo_robot_spawner')
        self.get_logger().info('Starting Ignition Gazebo robot spawner...')
        
        try:
            # Get the URDF content and process it for Ignition Gazebo
            urdf_content = self.get_processed_urdf()
            
            # Spawn the robot in Ignition Gazebo
            self.spawn_robot(urdf_content)
            
        except Exception as e:
            self.get_logger().error(f'Failed to spawn robot: {str(e)}')
        
    def get_processed_urdf(self):
        """Process XACRO file and convert package:// URIs to file:// URIs for Ignition Gazebo"""
        
        try:
            package_share_dir = get_package_share_directory('so_101_arm')
            xacro_path = os.path.join(package_share_dir, 'urdf', 'so_101_arm_6dof.urdf.xacro')
            controller_config_path = os.path.join(package_share_dir, 'config', 'controllers_6dof.yaml')
            
            self.get_logger().info(f'Processing XACRO from: {xacro_path}')
            
            # Process the xacro file to generate URDF content
            xacro_cmd = [
                'xacro', 
                xacro_path,
                f'controller_config_file:={controller_config_path}'
            ]
            
            self.get_logger().info(f'Running xacro command: {" ".join(xacro_cmd)}')
            
            # Increased timeout for xacro processing, just in case
            result = subprocess.run(xacro_cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode != 0:
                self.get_logger().error(f'XACRO processing failed: {result.stderr}')
                raise Exception(f'XACRO processing failed: {result.stderr}')
            
            urdf_content = result.stdout
            self.get_logger().info('XACRO processing completed successfully')
            
            # Replace package:// with file:// using the actual package path
            def replace_package_uri(match):
                relative_path = match.group(1)
                # Ensure the path is correctly joined for the OS
                full_path = os.path.abspath(os.path.join(package_share_dir, relative_path))
                return f'file://{full_path}'
            
            pattern = r'package://so_101_arm/([^"\']+)'
            processed_content = re.sub(pattern, replace_package_uri, urdf_content)
            
            package_uris_original_count = len(re.findall(r'package://so_101_arm/', urdf_content))
            file_uris_created_count = len(re.findall(r'file://' + re.escape(os.path.abspath(package_share_dir)), processed_content))

            self.get_logger().info(f'URDF processing complete: {package_uris_original_count} package:// URIs processed, resulting in {file_uris_created_count} file:// URIs.')
            
            return processed_content
            
        except subprocess.TimeoutExpired:
            self.get_logger().error('XACRO processing timed out.')
            raise
        except Exception as e:
            self.get_logger().error(f'Error processing URDF: {str(e)}')
            raise
    
    def spawn_robot(self, urdf_content):
        """Spawn robot in Ignition Gazebo using the processed URDF content"""
        
        try:
            temp_urdf_path = '/tmp/so_101_arm_ignition_gazebo.urdf'
            with open(temp_urdf_path, 'w') as f:
                f.write(urdf_content)
            
            self.get_logger().info(f'Temporary URDF written to: {temp_urdf_path}')
            
            # Z-coordinate for spawn (base_link origin to place robot bottom on mount surface)
            # Mount top surface Z_world = 0.845m
            # Offset of Waveshare plate bottom from base_link origin = 0.0698817m (visual_z - thickness/2)
            # Target Z for base_link origin = 0.845 - 0.0698817 = 0.7751183
            spawn_z = '0.77512' 

            # Yaw-coordinate:
            # If previous '3.14159' (pi) resulted in arm pointing AWAY (world +X),
            # then '0.0' should result in arm pointing INWARDS (world -X).
            spawn_yaw = '0.0' 

            spawn_cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', temp_urdf_path,
                '-name', 'so_101_arm',
                '-x', '0.4',    
                '-y', '0',      
                '-z', spawn_z,  
                '-R', '0',      
                '-P', '0',      
                '-Y', spawn_yaw 
            ]
            
            self.get_logger().info(f'Spawning robot with command: {" ".join(spawn_cmd)}')
            
            # Increased timeout for spawning, just in case
            result = subprocess.run(spawn_cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                self.get_logger().info('Robot spawned successfully.')
                if result.stdout:
                    self.get_logger().info(f'Spawn output: {result.stdout}')
            else:
                self.get_logger().error(f'Failed to spawn robot: {result.stderr}')
                if result.stdout: 
                    self.get_logger().error(f'Spawn stdout (on error): {result.stdout}')
            
            if os.path.exists(temp_urdf_path):
                os.remove(temp_urdf_path)
                self.get_logger().info('Temporary URDF file cleaned up')
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Robot spawning timed out.')
            raise
        except Exception as e:
            self.get_logger().error(f'Error spawning robot: {str(e)}')
            raise

def main(args=None):
    rclpy.init(args=args)
    spawner_node = None
    try:
        spawner_node = IgnitionGazeboRobotSpawner()
        # Since the work is done in __init__, we don't need to spin.
        # However, rclpy.shutdown() should be called after the node's work is done.
        # The __init__ method blocks until spawn_robot is finished or throws.
        if rclpy.ok(): # Check if shutdown hasn't been called already by an exception
             pass # Node did its job in init
    except Exception as e:
        if spawner_node:
            spawner_node.get_logger().fatal(f'Unhandled exception in spawner: {e}')
        else:
            print(f'Failed to initialize spawner node: {e}')
    finally:
        if spawner_node:
            spawner_node.get_logger().info('Spawner node shutting down.')
            spawner_node.destroy_node()
        if rclpy.ok(): # Avoid shutting down if already shut down
            rclpy.shutdown()

if __name__ == '__main__':
    main()

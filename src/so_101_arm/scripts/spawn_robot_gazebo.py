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
            
            result = subprocess.run(xacro_cmd, capture_output=True, text=True)
            
            if result.returncode != 0:
                self.get_logger().error(f'XACRO processing failed: {result.stderr}')
                raise Exception(f'XACRO processing failed: {result.stderr}')
            
            urdf_content = result.stdout
            self.get_logger().info('XACRO processing completed successfully')
            
            # Replace package:// with file:// using the actual package path
            def replace_package_uri(match):
                relative_path = match.group(1)
                full_path = os.path.join(package_share_dir, relative_path)
                self.get_logger().info(f'Converting package URI: {match.group(0)} -> file://{full_path}')
                return f'file://{full_path}'
            
            # Pattern to match package://so_101_arm/...
            pattern = r'package://so_101_arm/([^"\']+)'
            processed_content = re.sub(pattern, replace_package_uri, urdf_content)
            
            # Count the replacements
            package_uris = len(re.findall(r'package://so_101_arm/', urdf_content))
            self.get_logger().info(f'URDF processing complete: {package_uris} package:// URIs converted to file:// URIs')
            
            return processed_content
            
        except Exception as e:
            self.get_logger().error(f'Error processing URDF: {str(e)}')
            raise
    
    def spawn_robot(self, urdf_content):
        """Spawn robot in Ignition Gazebo using the processed URDF content"""
        
        try:
            # Write processed URDF to temporary file
            temp_urdf_path = '/tmp/so_101_arm_ignition_gazebo.urdf'
            with open(temp_urdf_path, 'w') as f:
                f.write(urdf_content)
            
            self.get_logger().info(f'Temporary URDF written to: {temp_urdf_path}')
            
            # Spawn robot using ros_gz_sim (for Ignition Gazebo)
            spawn_cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', temp_urdf_path,
                '-name', 'so_101_arm',
                '-x', '0',
                '-y', '0', 
                '-z', '0.1'
            ]
            
            self.get_logger().info(f'Spawning robot with command: {" ".join(spawn_cmd)}')
            
            result = subprocess.run(spawn_cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info('Robot spawned successfully in Ignition Gazebo')
                if result.stdout:
                    self.get_logger().info(f'Spawn output: {result.stdout}')
            else:
                self.get_logger().error(f'Failed to spawn robot: {result.stderr}')
                if result.stdout:
                    self.get_logger().error(f'Spawn stdout: {result.stdout}')
            
            # Clean up temporary file
            if os.path.exists(temp_urdf_path):
                os.remove(temp_urdf_path)
                self.get_logger().info('Temporary URDF file cleaned up')
                
        except Exception as e:
            self.get_logger().error(f'Error spawning robot: {str(e)}')
            raise

def main(args=None):
    rclpy.init(args=args)
    
    spawner = IgnitionGazeboRobotSpawner()
    
    # Keep the node alive briefly to see any final logs
    import time
    time.sleep(1)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

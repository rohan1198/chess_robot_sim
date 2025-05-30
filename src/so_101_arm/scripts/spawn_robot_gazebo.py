#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import re
from ament_index_python.packages import get_package_share_directory

class GazeboRobotSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_robot_spawner')
        self.get_logger().info('Starting Gazebo robot spawner...')
        
        try:
            # Get the URDF content and process it for Gazebo
            urdf_content = self.get_processed_urdf()
            
            # Spawn the robot in Gazebo
            self.spawn_robot(urdf_content)
            
        except Exception as e:
            self.get_logger().error(f'Failed to spawn robot: {str(e)}')
        
    def get_processed_urdf(self):
        """Read URDF and convert package:// URIs to file:// URIs for Gazebo"""
        
        try:
            package_share_dir = get_package_share_directory('so_101_arm')
            urdf_path = os.path.join(package_share_dir, 'urdf', 'so_101_arm_6dof.urdf')
            
            self.get_logger().info(f'Reading URDF from: {urdf_path}')
            
            with open(urdf_path, 'r') as f:
                urdf_content = f.read()
            
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
            file_uris = len(re.findall(r'file://', processed_content))
            
            self.get_logger().info(f'URDF processing complete: {package_uris} package:// URIs converted to file:// URIs')
            
            return processed_content
            
        except Exception as e:
            self.get_logger().error(f'Error processing URDF: {str(e)}')
            raise
    
    def spawn_robot(self, urdf_content):
        """Spawn robot in Gazebo using the processed URDF content"""
        
        try:
            # Write processed URDF to temporary file
            temp_urdf_path = '/tmp/so_101_arm_gazebo.urdf'
            with open(temp_urdf_path, 'w') as f:
                f.write(urdf_content)
            
            self.get_logger().info(f'Temporary URDF written to: {temp_urdf_path}')
            
            # Spawn robot using ros_gz_sim
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
                self.get_logger().info('Robot spawned successfully in Gazebo')
                self.get_logger().info(f'Spawn output: {result.stdout}')
            else:
                self.get_logger().error(f'Failed to spawn robot: {result.stderr}')
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
    
    spawner = GazeboRobotSpawner()
    
    # Keep the node alive briefly to see any final logs
    import time
    time.sleep(1)
    
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

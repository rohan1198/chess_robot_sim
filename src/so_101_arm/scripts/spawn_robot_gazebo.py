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
        self.get_logger().info('Creating processed URDF for robot spawning...')
        
        try:
            # Get the URDF content and process it for Ignition Gazebo
            urdf_content = self.get_processed_urdf()
            
            # Save processed URDF to fixed location for spawn command
            self.save_processed_urdf(urdf_content)
            
        except Exception as e:
            self.get_logger().error(f'Failed to process URDF: {str(e)}')
        
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
            
            result = subprocess.run(xacro_cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode != 0:
                self.get_logger().error(f'XACRO processing failed: {result.stderr}')
                raise Exception(f'XACRO processing failed: {result.stderr}')
            
            urdf_content = result.stdout
            self.get_logger().info('XACRO processing completed successfully')
            
            # Replace package:// with file:// using the actual package path
            def replace_package_uri(match):
                relative_path = match.group(1)
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
    
    def save_processed_urdf(self, urdf_content):
        """Save the processed URDF content to a file for spawning"""
        
        try:
            temp_urdf_path = '/tmp/so_101_arm_processed.urdf'
            with open(temp_urdf_path, 'w') as f:
                f.write(urdf_content)
            
            self.get_logger().info(f'Processed URDF written to: {temp_urdf_path}')
            self.get_logger().info('URDF file ready for robot spawning')
            
        except Exception as e:
            self.get_logger().error(f'Error saving processed URDF: {str(e)}')
            raise

def main(args=None):
    rclpy.init(args=args)
    spawner_node = None
    try:
        spawner_node = IgnitionGazeboRobotSpawner()
        if rclpy.ok():
             pass
    except Exception as e:
        if spawner_node:
            spawner_node.get_logger().fatal(f'Unhandled exception in spawner: {e}')
        else:
            print(f'Failed to initialize spawner node: {e}')
    finally:
        if spawner_node:
            spawner_node.get_logger().info('Spawner node shutting down.')
            spawner_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

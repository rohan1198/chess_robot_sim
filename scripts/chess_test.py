#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, CameraInfo
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class ChessEnvironmentTester(Node):
    def __init__(self):
        super().__init__('chess_environment_tester')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Chess board parameters
        self.square_size = 0.057  # 57mm squares
        self.board_center = [0.75, 0, 0.755]  # Table position (optimized for arm reach)
        
        # Test data storage
        self.camera_working = False
        self.robot_ready = False
        self.latest_image = None
        self.joint_positions = {}
        
        # Subscribers
        self.camera_sub = self.create_subscription(
            Image,
            '/chess_camera/image_raw',
            self.camera_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/chess_camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Test timers
        self.test_timer = self.create_timer(5.0, self.run_periodic_tests)
        
        self.get_logger().info('Chess Environment Tester Started!')
        self.get_logger().info('Running comprehensive tests...')
    
    def camera_callback(self, msg):
        """Handle incoming camera images."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            
            if not self.camera_working:
                self.camera_working = True
                self.get_logger().info('✅ Camera feed is working!')
                
                # Save first image for analysis
                cv2.imwrite('/tmp/chess_camera_test.jpg', cv_image)
                self.get_logger().info('📸 Saved test image to /tmp/chess_camera_test.jpg')
                
        except Exception as e:
            self.get_logger().error(f'❌ Camera callback error: {e}')
    
    def camera_info_callback(self, msg):
        """Handle camera info messages."""
        self.get_logger().info(f'📷 Camera Info: {msg.width}x{msg.height}, FOV: {msg.k[0]:.1f}px focal length')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates."""
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
        
        if not self.robot_ready and len(self.joint_positions) >= 6:  # 5 arm joints + gripper
            self.robot_ready = True
            self.get_logger().info('✅ Robot joint states are available!')
    
    def run_periodic_tests(self):
        """Run periodic tests to verify system status."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🔍 CHESS ENVIRONMENT STATUS CHECK')
        self.get_logger().info('='*60)
        
        # Test 1: Camera System
        self.test_camera_system()
        
        # Test 2: Robot System  
        self.test_robot_system()
        
        # Test 3: Chess Board Analysis
        self.test_board_detection()
        
        # Test 4: Coordinate System
        self.test_coordinate_system()
        
        self.get_logger().info('='*60)
    
    def test_camera_system(self):
        """Test camera functionality."""
        self.get_logger().info('📷 CAMERA SYSTEM TEST:')
        
        if self.camera_working and self.latest_image is not None:
            height, width = self.latest_image.shape[:2]
            self.get_logger().info(f'   ✅ Camera active: {width}x{height}')
            
            # Basic image analysis
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray)
            contrast = np.std(gray)
            
            self.get_logger().info(f'   📊 Image quality: brightness={brightness:.1f}, contrast={contrast:.1f}')
            
            if brightness < 50:
                self.get_logger().warn('   ⚠️  Image might be too dark')
            elif brightness > 200:
                self.get_logger().warn('   ⚠️  Image might be too bright')
            else:
                self.get_logger().info('   ✅ Image brightness is good')
                
            if contrast < 30:
                self.get_logger().warn('   ⚠️  Low contrast detected')
            else:
                self.get_logger().info('   ✅ Image contrast is good')
        else:
            self.get_logger().error('   ❌ Camera not working or no images received')
    
    def test_robot_system(self):
        """Test robot joint system."""
        self.get_logger().info('🤖 ROBOT SYSTEM TEST:')
        
        if self.robot_ready:
            self.get_logger().info(f'   ✅ Robot ready: {len(self.joint_positions)} joints detected')
            
            # Check specific joints
            expected_joints = ['shoulder_rotation', 'shoulder_pitch', 'elbow', 
                             'wrist_pitch', 'wrist_roll', 'gripper_joint']
            
            for joint in expected_joints:
                if joint in self.joint_positions:
                    pos = self.joint_positions[joint]
                    self.get_logger().info(f'   📍 {joint}: {pos:.3f} rad')
                else:
                    self.get_logger().warn(f'   ⚠️  Missing joint: {joint}')
                    
        else:
            self.get_logger().error('   ❌ Robot not ready or joint states not available')
    
    def test_board_detection(self):
        """Test basic board detection capabilities."""
        self.get_logger().info('♟️  CHESSBOARD DETECTION TEST:')
        
        if self.latest_image is None:
            self.get_logger().error('   ❌ No camera image available for board detection')
            return
        
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            
            # Try to detect chessboard corners (simplified test)
            board_size = (7, 7)  # Internal corners of 8x8 board
            ret, corners = cv2.findChessboardCorners(gray, board_size, None)
            
            if ret:
                self.get_logger().info('   ✅ Chessboard pattern detected!')
                self.get_logger().info(f'   📐 Found {len(corners)} corner points')
                
                # Save annotated image
                img_with_corners = self.latest_image.copy()
                cv2.drawChessboardCorners(img_with_corners, board_size, corners, ret)
                cv2.imwrite('/tmp/chess_board_detection.jpg', img_with_corners)
                self.get_logger().info('   💾 Saved detection result to /tmp/chess_board_detection.jpg')
                
            else:
                self.get_logger().warn('   ⚠️  Chessboard pattern not clearly detected')
                self.get_logger().info('   💡 Try adjusting camera angle or lighting')
                
        except Exception as e:
            self.get_logger().error(f'   ❌ Board detection error: {e}')
    
    def test_coordinate_system(self):
        """Test coordinate system calculations."""
        self.get_logger().info('📏 COORDINATE SYSTEM TEST:')
        
        # Test key chess squares in robot coordinates
        test_squares = {
            'a1': self.chess_square_to_world('a', 1),
            'e1': self.chess_square_to_world('e', 1),
            'h1': self.chess_square_to_world('h', 1),
            'a8': self.chess_square_to_world('a', 8),
            'e8': self.chess_square_to_world('e', 8),
            'h8': self.chess_square_to_world('h', 8),
        }
        
        self.get_logger().info('   📍 Key square positions (robot frame):')
        for square, pos in test_squares.items():
            x, y, z = pos
            distance = np.sqrt(x**2 + y**2)
            self.get_logger().info(f'      {square}: ({x:.3f}, {y:.3f}, {z:.3f}) - {distance:.3f}m from base')
            
            # Check reachability (approximate)
            if distance > 1.2:  # Rough arm reach limit
                self.get_logger().warn(f'      ⚠️  {square} might be out of reach!')
            else:
                self.get_logger().info(f'      ✅ {square} should be reachable')
    
    def chess_square_to_world(self, file_letter, rank_number):
        """Convert chess square notation to world coordinates."""
        # Convert file letter (a-h) to column index (0-7)
        file_index = ord(file_letter.lower()) - ord('a')
        
        # Convert rank number (1-8) to row index (7-0, since rank 1 is at the bottom)
        rank_index = 8 - rank_number
        
        # Calculate position relative to board center
        x_offset = (file_index - 3.5) * self.square_size
        y_offset = (3.5 - rank_index) * self.square_size
        
        # Add to board center position
        world_x = self.board_center[0] + x_offset
        world_y = self.board_center[1] + y_offset
        world_z = self.board_center[2] + 0.005  # Slightly above board surface
        
        return [world_x, world_y, world_z]
    
    def print_startup_info(self):
        """Print helpful startup information."""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('🏁 CHESS ENVIRONMENT TESTER')
        self.get_logger().info('='*60)
        self.get_logger().info('This script will test your chess environment setup.')
        self.get_logger().info('')
        self.get_logger().info('📋 CHECKLIST:')
        self.get_logger().info('   □ Camera feed working')
        self.get_logger().info('   □ Robot joints responsive') 
        self.get_logger().info('   □ Chessboard detection')
        self.get_logger().info('   □ Coordinate system validation')
        self.get_logger().info('')
        self.get_logger().info('🔍 Monitoring topics:')
        self.get_logger().info('   • /chess_camera/image_raw')
        self.get_logger().info('   • /chess_camera/camera_info')
        self.get_logger().info('   • /joint_states')
        self.get_logger().info('='*60)

def main(args=None):
    rclpy.init(args=args)
    
    tester = ChessEnvironmentTester()
    tester.print_startup_info()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('\n🛑 Test session ended by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

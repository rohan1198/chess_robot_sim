#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/chess_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/chess_camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # State variables
        self.latest_image = None
        self.camera_info = None
        self.frame_count = 0
        self.image_received = False
        
        # Display settings
        self.window_name = "Chess Camera Feed"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 540)  # Half resolution for easier viewing
        
        # Timer for display updates
        self.display_timer = self.create_timer(0.033, self.display_frame)  # ~30 FPS
        
        self.get_logger().info('🎥 Camera Viewer Started!')
        self.get_logger().info('📡 Subscribing to /chess_camera/image_raw')
        self.get_logger().info('📡 Subscribing to /chess_camera/camera_info')
        self.get_logger().info('👁️  Press "q" in the image window to quit')
        self.get_logger().info('👁️  Press "s" to save current frame')
        self.get_logger().info('👁️  Press "i" to show camera info')
        
    def image_callback(self, msg):
        """Handle incoming camera images."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image.copy()
            self.frame_count += 1
            
            if not self.image_received:
                self.image_received = True
                height, width = cv_image.shape[:2]
                self.get_logger().info(f'✅ First image received! Resolution: {width}x{height}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Image conversion error: {e}')
    
    def camera_info_callback(self, msg):
        """Handle camera info messages."""
        self.camera_info = msg
        if self.camera_info and not hasattr(self, 'info_logged'):
            self.info_logged = True
            self.get_logger().info(f'📷 Camera info received: {msg.width}x{msg.height}')
    
    def display_frame(self):
        """Display the latest frame with overlay information."""
        if self.latest_image is None:
            return
        
        # Create a copy for display
        display_image = self.latest_image.copy()
        
        # Add overlay information
        self.add_overlay_info(display_image)
        
        # Display the image
        cv2.imshow(self.window_name, display_image)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('🛑 Quit requested by user')
            rclpy.shutdown()
        elif key == ord('s'):
            self.save_frame()
        elif key == ord('i'):
            self.print_camera_info()
        elif key == ord('h'):
            self.print_help()
    
    def add_overlay_info(self, image):
        """Add helpful overlay information to the image."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)  # Green
        thickness = 2
        
        # Add frame counter
        text = f"Frame: {self.frame_count}"
        cv2.putText(image, text, (10, 30), font, font_scale, color, thickness)
        
        # Add resolution info
        height, width = image.shape[:2]
        text = f"Resolution: {width}x{height}"
        cv2.putText(image, text, (10, 60), font, font_scale, color, thickness)
        
        # Add chess board center marker if we can estimate it
        self.add_board_markers(image)
    
    def add_board_markers(self, image):
        """Add markers to help visualize chess board position."""
        height, width = image.shape[:2]
        
        # Simple crosshair at image center
        center_x, center_y = width // 2, height // 2
        cv2.line(image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
        cv2.line(image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)
        
        # Label it
        cv2.putText(image, "Center", (center_x + 25, center_y + 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    def save_frame(self):
        """Save the current frame to disk."""
        if self.latest_image is not None:
            filename = f"chess_camera_frame_{self.frame_count:06d}.jpg"
            cv2.imwrite(filename, self.latest_image)
            self.get_logger().info(f'💾 Saved frame: {filename}')
        else:
            self.get_logger().warn('⚠️  No frame to save')
    
    def print_camera_info(self):
        """Print detailed camera information."""
        if self.camera_info:
            info = self.camera_info
            self.get_logger().info(f'📷 Camera Information:')
            self.get_logger().info(f'   Resolution: {info.width}x{info.height}')
            self.get_logger().info(f'   Distortion Model: {info.distortion_model}')
            self.get_logger().info(f'   Camera Matrix K:')
            k = info.k
            self.get_logger().info(f'     [{k[0]:.2f}, {k[1]:.2f}, {k[2]:.2f}]')
            self.get_logger().info(f'     [{k[3]:.2f}, {k[4]:.2f}, {k[5]:.2f}]')
            self.get_logger().info(f'     [{k[6]:.2f}, {k[7]:.2f}, {k[8]:.2f}]')
            if info.d:
                self.get_logger().info(f'   Distortion: {[f"{d:.4f}" for d in info.d[:5]]}')
        else:
            self.get_logger().warn('⚠️  No camera info available')
    
    def print_help(self):
        """Print help information."""
        self.get_logger().info('🆘 Camera Viewer Help:')
        self.get_logger().info('   q - Quit application')
        self.get_logger().info('   s - Save current frame as JPEG')
        self.get_logger().info('   i - Show detailed camera information')
        self.get_logger().info('   h - Show this help')
    
    def analyze_chess_board(self):
        """Analyze the current frame for chess board detection."""
        if self.latest_image is None:
            return
        
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        
        # Try to detect chessboard corners
        board_size = (7, 7)  # Internal corners of 8x8 board
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        
        if ret:
            self.get_logger().info('✅ Chessboard detected in current frame!')
            
            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Draw corners on a copy for visualization
            board_image = self.latest_image.copy()
            cv2.drawChessboardCorners(board_image, board_size, corners_refined, ret)
            
            # Save the annotated image
            cv2.imwrite(f'chessboard_detection_{self.frame_count}.jpg', board_image)
            self.get_logger().info(f'💾 Saved chessboard detection image')
        else:
            self.get_logger().warn('⚠️  No chessboard pattern detected in current frame')

def main(args=None):
    print("🎥 Starting Chess Camera Viewer...")
    print("📋 Make sure your Gazebo simulation is running!")
    print("📋 Camera topic: /chess_camera/image_raw")
    
    rclpy.init(args=args)
    
    try:
        viewer = CameraViewer()
        
        # Give some time for initial setup
        import time
        time.sleep(2)
        
        # Print initial status
        if not viewer.image_received:
            viewer.get_logger().warn('⚠️  No images received yet. Is the simulation running?')
            viewer.get_logger().info('💡 Try running: ros2 launch chess_robot_sim chess_gazebo.launch.py')
        
        rclpy.spin(viewer)
        
    except KeyboardInterrupt:
        print("\n🛑 Camera viewer interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Camera viewer stopped")

if __name__ == '__main__':
    main()

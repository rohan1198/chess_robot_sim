#!/usr/bin/env python3

import subprocess
import sys
import argparse
import time
import rclpy
from rclpy.node import Node


class NativeGazeboPieceMover(Node):
    """
    Piece mover using native Gazebo/Ignition services directly.
    Uses 'ign service' or 'gz service' commands to manipulate entities.
    """
    def __init__(self):
        super().__init__('native_gazebo_piece_mover')
        self.setup_coordinate_mapping()
        self.setup_piece_names()
        
        self.gazebo_cmd = self.detect_gazebo_command()
        
        self.get_logger().info(f'Native Gazebo Piece Mover ready using: {self.gazebo_cmd}')
    
    def detect_gazebo_command(self):
        """Detect whether to use 'ign' or 'gz' command"""
        try:
            result = subprocess.run(['ign', 'service', '-l'], 
                                  capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                self.get_logger().info("Using 'ign' command for Gazebo")
                return 'ign'
        except Exception:
            pass
        
        try:
            result = subprocess.run(['gz', 'service', '-l'], 
                                  capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                self.get_logger().info("Using 'gz' command for Gazebo")
                return 'gz'
        except Exception:
            pass
        
        self.get_logger().error("Neither 'ign' nor 'gz' command available!")
        return None
    
    def setup_coordinate_mapping(self):
        """Setup chess square to Gazebo coordinate mapping"""
        self.board_center_x = 0.05
        self.board_center_y = 0.0
        self.board_height = 0.8315
        self.square_size = 0.04
        
        board_min_x = self.board_center_x - 0.14
        board_min_y = self.board_center_y - 0.14
        
        self.chess_to_coords = {}
        files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        ranks = ['1', '2', '3', '4', '5', '6', '7', '8']
        
        for file_idx, file_char in enumerate(files):
            for rank_idx, rank_char in enumerate(ranks):
                square = file_char + rank_char
                x = board_min_x + (rank_idx * self.square_size)
                y = board_min_y + (file_idx * self.square_size)
                z = self.board_height
                self.chess_to_coords[square] = (x, y, z)
    
    def setup_piece_names(self):
        """Setup piece name mapping"""
        self.piece_names = {
            # White pieces
            'white_rook_a1', 'white_knight_b1', 'white_bishop_c1', 'white_queen_d1',
            'white_king_e1', 'white_bishop_f1', 'white_knight_g1', 'white_rook_h1',
            'white_pawn_a2', 'white_pawn_b2', 'white_pawn_c2', 'white_pawn_d2',
            'white_pawn_e2', 'white_pawn_f2', 'white_pawn_g2', 'white_pawn_h2',
            
            # Black pieces
            'black_rook_a8', 'black_knight_b8', 'black_bishop_c8', 'black_queen_d8',
            'black_king_e8', 'black_bishop_f8', 'black_knight_g8', 'black_rook_h8',
            'black_pawn_a7', 'black_pawn_b7', 'black_pawn_c7', 'black_pawn_d7',
            'black_pawn_e7', 'black_pawn_f7', 'black_pawn_g7', 'black_pawn_h7',
        }
    
    def move_piece_to_coordinates(self, piece_name: str, x: float, y: float, z: float) -> bool:
        """Move piece using native Gazebo service"""
        if not self.gazebo_cmd:
            self.get_logger().error("No Gazebo command available")
            return False
        
        try:
            # Use native Gazebo set_pose service
            # Format: ign service -s /world/WORLD/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 1000 --req 'name: "model_name", position: {x: 1, y: 2, z: 3}'
            
            cmd = [
                self.gazebo_cmd, 'service',
                '-s', '/world/chess_table_world/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean', 
                '--timeout', '1000',
                '--req', f'name: "{piece_name}", position: {{x: {x}, y: {y}, z: {z}}}'
            ]
            
            self.get_logger().info(f"Executing: {' '.join(cmd)}")
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                if 'data: true' in result.stdout:
                    self.get_logger().info(f'Moved {piece_name} to ({x:.3f}, {y:.3f}, {z:.3f})')
                    return True
                else:
                    self.get_logger().warn(f'Service call succeeded but response unclear: {result.stdout}')
                    return True  # Assume success if service call worked
            else:
                self.get_logger().error(f'Failed to move {piece_name}: {result.stderr}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout moving {piece_name}')
            return False
        except Exception as e:
            self.get_logger().error(f'Error moving {piece_name}: {str(e)}')
            return False
    
    def move_piece_to_square(self, piece_name: str, square: str) -> bool:
        """Move a piece to a chess square"""
        try:
            x, y, z = self.chess_to_coords[square]
            return self.move_piece_to_coordinates(piece_name, x, y, z)
        except KeyError:
            self.get_logger().error(f"Invalid chess square: {square}")
            return False
    
    def list_gazebo_services(self):
        """List available Gazebo services"""
        if not self.gazebo_cmd:
            print("No Gazebo command available")
            return
        
        try:
            result = subprocess.run([self.gazebo_cmd, 'service', '-l'], 
                                  capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0:
                print("Available Gazebo services:")
                services = result.stdout.strip().split('\n')
                world_services = [s for s in services if 'world' in s]
                
                for service in sorted(world_services):
                    print(f"  {service}")
                    
                if not world_services:
                    print("  (No world services found)")
                    
            else:
                print(f"Failed to list services: {result.stderr}")
                
        except Exception as e:
            print(f"Error listing services: {e}")
    
    def test_service_call(self):
        """Test a simple service call"""
        if not self.gazebo_cmd:
            print("No Gazebo command available")
            return False
        
        try:
            # Test with a simple service call
            cmd = [
                self.gazebo_cmd, 'service',
                '-s', '/world/chess_table_world/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req', 'name: "white_pawn_e2", position: {x: -0.05, y: 0.02, z: 0.832}'
            ]
            
            print("Testing service call...")
            print(f"Command: {' '.join(cmd)}")
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            print(f"Return code: {result.returncode}")
            print(f"Stdout: {result.stdout}")
            print(f"Stderr: {result.stderr}")
            
            return result.returncode == 0
            
        except Exception as e:
            print(f"Test failed: {e}")
            return False
    
    def interactive_mode(self):
        """Interactive mode for manual piece movement"""
        print("\nInteractive Native Gazebo Piece Mover")
        print("Commands:")
        print("  move <piece_name> <square>  - Move piece to square")
        print("  coords <piece_name> <x> <y> <z> - Move piece to coordinates")
        print("  services - List available Gazebo services")
        print("  test - Test service call")
        print("  quit - Exit")
        
        while True:
            try:
                command = input("\n> ").strip()
                
                if command.lower() == 'quit':
                    break
                elif command.lower() == 'services':
                    self.list_gazebo_services()
                elif command.lower() == 'test':
                    self.test_service_call()
                elif command.startswith('move '):
                    parts = command.split()
                    if len(parts) == 3:
                        piece_name, square = parts[1], parts[2]
                        self.move_piece_to_square(piece_name, square)
                    else:
                        print("Usage: move <piece_name> <square>")
                elif command.startswith('coords '):
                    parts = command.split()
                    if len(parts) == 5:
                        piece_name = parts[1]
                        try:
                            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                            self.move_piece_to_coordinates(piece_name, x, y, z)
                        except ValueError:
                            print("Invalid coordinates")
                    else:
                        print("Usage: coords <piece_name> <x> <y> <z>")
                elif command == '':
                    continue
                else:
                    print("Unknown command")
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break

def main():
    parser = argparse.ArgumentParser(description='Move chess pieces using native Gazebo services')
    
    parser.add_argument('--move', nargs=2, metavar=('PIECE', 'SQUARE'))
    parser.add_argument('--coords', nargs=4, metavar=('PIECE', 'X', 'Y', 'Z'))
    parser.add_argument('--interactive', action='store_true')
    parser.add_argument('--test-opening', action='store_true')
    parser.add_argument('--test-service', action='store_true')
    parser.add_argument('--list-services', action='store_true')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        mover = NativeGazeboPieceMover()
        
        if args.move:
            piece_name, square = args.move
            success = mover.move_piece_to_square(piece_name, square)
            return 0 if success else 1
        
        elif args.coords:
            piece_name, x_str, y_str, z_str = args.coords
            try:
                x, y, z = float(x_str), float(y_str), float(z_str)
                success = mover.move_piece_to_coordinates(piece_name, x, y, z)
                return 0 if success else 1
            except ValueError:
                print("Invalid coordinates")
                return 1
        
        elif args.test_opening:
            print("Testing opening sequence with native Gazebo...")
            
            moves = [
                ('white_pawn_e2', 'e4'),
                ('black_pawn_e7', 'e5'),
                ('white_knight_g1', 'f3'),
                ('black_knight_b8', 'c6')
            ]
            
            for piece, square in moves:
                print(f"Moving {piece} to {square}...")
                success = mover.move_piece_to_square(piece, square)
                if not success:
                    print(f"Failed to move {piece}")
                    return 1
                time.sleep(1.5)
            
            print("Test opening completed!")
            return 0
        
        elif args.test_service:
            success = mover.test_service_call()
            return 0 if success else 1
        
        elif args.list_services:
            mover.list_gazebo_services()
            return 0
        
        elif args.interactive:
            mover.interactive_mode()
            return 0
        
        else:
            parser.print_help()
            return 0
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())

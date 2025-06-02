#!/usr/bin/env python3

from typing import Tuple
import numpy as np
import chess
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState


class ChessPieceController(Node):
    """
    ROS2 node for controlling chess pieces in gazebo.
    Provides chess game logic and piece movement coordination.
    """
    def __init__(self):
        super().__init__('chess_piece_controller')

        self.board = chess.Board()

        self.set_entity_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.get_entity_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        self.setup_coordinate_mapping()

        self.setup_piece_mapping()

        self.setup_robot_workspace()

        self.get_logger().info('Chess Piece Controller initialized')
        self.get_logger().info(f'Board center: ({self.board_center_x}, {self.board_center_y}, {self.board_height})')
        self.get_logger().info(f'Robot position: ({self.robot_x}, {self.robot_y}, {self.robot_z})')
    
    def setup_coordinate_mapping(self):
        """
        Setup mapping between chess notation and Gazebo coordinates
        """
        self.board_center_x = 0.005
        self.board_center_y = 0.0
        self.board_height = 0.8315
        self.square_size = 0.04  # 4cm per square

        # Calculate board corners
        self.board_min_x = self.board_center_x - 0.14  # -0.09
        self.board_max_x = self.board_center_x + 0.14  # +0.19 
        self.board_min_y = self.board_center_y - 0.14  # -0.14
        self.board_max_y = self.board_center_y + 0.14  # +0.14

        # Create coordinate mapping
        self.chess_to_gazebo = {}
        self.gazebo_to_chess = {}

        files = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        ranks = ['1', '2', '3', '4', '5', '6', '7', '8']

        for file_idx, file_char in enumerate(files):
            for rank_idx, rank_char in enumerate(ranks):
                square = file_char + rank_char

                # Rank 1 (white pieces) at board_min_x = -0.09
                # Rank 8 (black_pieces) at board_max_x = +0.19
                # Files a-h from board_min_y (-0.14) to board_max_y (0.14)

                gazebo_x = self.board_min_x + (rank_idx * self.square_size)
                gazebo_y = self.board_min_y + (file_idx * self.square_size)
                gazebo_z = self.board_height

                self.chess_to_gazebo[square] = (gazebo_x, gazebo_y, gazebo_z)

                key = (round(gazebo_x, 3), round(gazebo_y, 3))
                self.gazebo_to_chess[key] = square
    
    def setup_robot_workspace(self):
        """
        Analyse robot workspace and reachability
        """
        # - Base collision geometry offset after -90° rotation
        # - Proper mounting on platform surface
        self.robot_x = 0.3779  # Corrected X position (0.4 - rotated offset)
        self.robot_y = 0.0208  # Corrected Y position (0.0 + rotated offset)
        self.robot_z = 0.88    # Exact mount platform surface height

        self.robot_reach_radius = 0.45  # 45cm

        self.reachable_squares = {}
        self.unreachable_squares = []

        for square, (x, y, z) in self.chess_to_gazebo.items():
            distance = np.sqrt((x - self.robot_x)**2 + (y - self.robot_y)**2)
            is_reachable = distance <= self.robot_reach_radius
            
            self.reachable_squares[square] = {
                'reachable': is_reachable,
                'distance': distance,
                'coordinates': (x, y, z)
            }
            
            if not is_reachable:
                self.unreachable_squares.append(square)
        
        if self.unreachable_squares:
            self.get_logger().warn(f'Potentially unreachable squares: {self.unreachable_squares}')
        else:
            self.get_logger().info('All chess squares are within robot reach!')
            
        key_squares = ['e2', 'e4', 'd4', 'f4', 'a1', 'h1', 'a8', 'h8']
        for square in key_squares:
            if square in self.reachable_squares:
                info = self.reachable_squares[square]
                status = "✓" if info['reachable'] else "⚠"
                self.get_logger().info(
                    f'Square {square}: {info["distance"]:.3f}m [{status}]'
                )
    
    def setup_piece_mapping(self):
        """
        Setup mapping between chess pieces and gazebo model names.
        """
        self.initial_piece_positions = {
            # White pieces
            'a1': 'white_rook_a1', 'b1': 'white_knight_b1', 'c1': 'white_bishop_c1', 'd1': 'white_queen_d1',
            'e1': 'white_king_e1', 'f1': 'white_bishop_f1', 'g1': 'white_knight_g1', 'h1': 'white_rook_h1',
            'a2': 'white_pawn_a2', 'b2': 'white_pawn_b2', 'c2': 'white_pawn_c2', 'd2': 'white_pawn_d2',
            'e2': 'white_pawn_e2', 'f2': 'white_pawn_f2', 'g2': 'white_pawn_g2', 'h2': 'white_pawn_h2',
            
            # Black pieces
            'a8': 'black_rook_a8', 'b8': 'black_knight_b8', 'c8': 'black_bishop_c8', 'd8': 'black_queen_d8',
            'e8': 'black_king_e8', 'f8': 'black_bishop_f8', 'g8': 'black_knight_g8', 'h8': 'black_rook_h8',
            'a7': 'black_pawn_a7', 'b7': 'black_pawn_b7', 'c7': 'black_pawn_c7', 'd7': 'black_pawn_d7',
            'e7': 'black_pawn_e7', 'f7': 'black_pawn_f7', 'g7': 'black_pawn_g7', 'h7': 'black_pawn_h7',
        }

        self.current_piece_positions = self.initial_piece_positions.copy()
    
    def get_square_coordinates(self, square: str) -> Tuple[float, float, float]:
        """Get Gazebo coordinates for a chess square"""
        if square in self.chess_to_gazebo:
            return self.chess_to_gazebo[square]
        else:
            raise ValueError(f"Invalid chess square: {square}")
    
    def is_square_reachable(self, square: str) -> bool:
        """Check if a square is reachable by the robot arm"""
        if square in self.reachable_squares:
            return self.reachable_squares[square]['reachable']
        return False
    
    def get_square_distance_from_robot(self, square: str) -> float:
        """Get distance from robot to a chess square"""
        if square in self.reachable_squares:
            return self.reachable_squares[square]['distance']
        return float('inf')
    
    def move_piece_to_corrdinates(self, piece_name: str, x: float, y: float, z: float) -> bool:
        """Move a piece to specific Gazebo coordicates"""
        if not self.set_entity_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set entity service not available')
            return False
        
        entity_state = EntityState()
        entity_state.name = piece_name
        entity_state.pose.position.x = x
        entity_state.pose.position.y = y
        entity_state.pose.position.z = z
        entity_state.pose.orientation.w = 1.0  # No rotation

        request = SetEntityState.Request()
        request.state = entity_state

        try:
            future = self.set_entity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

            if result.success:
                self.get_logger().info(f'Successfully moved {piece_name} to ({x:.3f}, {y:.3f}, {z:.3f})')
                return True
            else:
                self.get_logger().error(f'Failed to move {piece_name}: {result.status_message}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error moving piece {piece_name}: {str(e)}')
            return False
    
    def move_piece_to_square(self, piece_name: str, target_square: str) -> bool:
        """Move a piece to a chess square"""
        try:
            # Check if target square is reachable
            if not self.is_square_reachable(target_square):
                self.get_logger().warn(f'Target square {target_square} may be difficult to reach')
            
            x, y, z = self.get_square_coordinates(target_square)
            return self.move_piece_to_coordinates(piece_name, x, y, z)
        except ValueError as e:
            self.get_logger().error(str(e))
            return False
        
    def move_piece_off_board(self, piece_name: str, side: str = 'right') -> bool:
        """Move captured piece to off-board storage area"""
        # Define storage areas for captured pieces
        if side == 'right':
            # Right side of board
            storage_x = 0.25  # Closer to robot
            storage_y = 0.35
        else:
            # Left side of board
            storage_x = 0.25
            storage_y = -0.35
        
        storage_z = self.board_height

        return self.move_piece_to_corrdinates(piece_name, storage_x, storage_y, storage_z)

    def execute_chess_move(self, move_uci: str) -> bool:
        """Execute a chess move in UCI notation (e.g. e2e4)"""
        try:
            move = chess.Move.from_uci(move_uci)

            if move not in self.board.legal_moves:
                self.get_logger().error(f"Illegal move: {move_uci}")
                return False
            
            from_square = chess.square_name(move.from_square)
            to_square = chess.square_name(move.to_square)

            if not self.is_square_reachable(from_square):
                self.get_logger().warn(f'Source square {from_square} is at edge of reach')
            if not self.is_square_reachable(to_square):
                self.get_logger().warn(f'Target square {to_square} is at edge of reach')
            
            piece_name = None
            for square, name in self.current_piece_positions.items():
                if square == from_square:
                    piece_name = name
                    break
            
            if piece_name is None:
                self.get_logger().error(f"No piece found at {from_square}")
                return False
            
            if to_square in self.current_piece_positions:
                captured_piece = self.current_piece_positions[to_square]
                # Move captured piece off the board
                capture_side = 'right' if 'white' in captured_piece else 'left'
                self.move_piece_off_board(captured_piece, capture_side)
                del self.current_piece_positions[to_square]
                self.get_logger().info(f'Captured piece {captured_piece} from {to_square}')
            
            success = self.move_piece_to_square(piece_name, to_square)
            
            if success:
                self.board.push(move)
                
                del self.current_piece_positions[from_square]
                self.current_piece_positions[to_square] = piece_name
                
                self.get_logger().info(f'Executed move: {move_uci} ({from_square} -> {to_square})')
                
                distance = self.get_square_distance_from_robot(to_square)
                self.get_logger().info(f'Piece now at distance {distance:.3f}m from robot')
                
                return True
            else:
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error executing move {move_uci}: {str(e)}')
            return False
    
    def get_legal_moves(self) -> list:
        """Get list of legal moves in current position"""
        return [move.uci() for move in self.board.legal_moves]
    
    def get_reachable_moves(self) -> list:
        """Get list of legal moves that are easily reachable"""
        legal_moves = self.get_legal_moves()
        reachable_moves = []
        
        for move_uci in legal_moves:
            move = chess.Move.from_uci(move_uci)
            from_square = chess.square_name(move.from_square)
            to_square = chess.square_name(move.to_square)
            
            if (self.is_square_reachable(from_square) and 
                self.is_square_reachable(to_square)):
                reachable_moves.append(move_uci)
        
        return reachable_moves
    
    def reset_board(self) -> bool:
        """Reset all pieces to starting positions"""
        self.get_logger().info('Resetting chess board to starting position')
        
        success = True
        for square, piece_name in self.initial_piece_positions.items():
            x, y, z = self.get_square_coordinates(square)
            if not self.move_piece_to_coordinates(piece_name, x, y, z):
                success = False
        
        if success:
            self.board.reset()
            self.current_piece_positions = self.initial_piece_positions.copy()
            self.get_logger().info('Board reset successfully')
        else:
            self.get_logger().error('Failed to reset some pieces')
        
        return success
    
    def print_board_state(self):
        """Print current board state"""
        self.get_logger().info(f'Current board position:\n{self.board}')
        self.get_logger().info(f'Turn: {"White" if self.board.turn else "Black"}')
        self.get_logger().info(f'Legal moves: {len(self.get_legal_moves())}')
        self.get_logger().info(f'Reachable moves: {len(self.get_reachable_moves())}')
    
    def print_coordinate_mapping(self):
        """Print coordinate mapping for debugging"""
        self.get_logger().info('Chess square to Gazebo coordinate mapping:')
        for square in ['a1', 'e1', 'h1', 'd4', 'e4', 'f4', 'a8', 'e8', 'h8']:
            if square in self.chess_to_gazebo:
                x, y, z = self.chess_to_gazebo[square]
                reachable = self.is_square_reachable(square)
                distance = self.get_square_distance_from_robot(square)
                self.get_logger().info(
                    f'  {square} → ({x:.3f}, {y:.3f}, {z:.3f}) '
                    f'[{"✓" if reachable else "⚠"}] dist: {distance:.3f}m'
                )


def main(args=None):
    rclpy.init(args=args)
    
    chess_controller = ChessPieceController()
    
    try:
        chess_controller.get_logger().info('Chess controller ready')
        chess_controller.print_board_state()
        chess_controller.print_coordinate_mapping()
        
        # Example: Move white pawn from e2 to e4
        # chess_controller.execute_chess_move('e2e4')
        
        rclpy.spin(chess_controller)
        
    except KeyboardInterrupt:
        chess_controller.get_logger().info('Shutting down chess controller')
    finally:
        chess_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

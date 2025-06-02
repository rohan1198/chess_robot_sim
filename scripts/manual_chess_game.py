#!/usr/bin/env python3

import os
import time
import numpy as np
import chess
import chess.engine
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


class ManualChessGame(Node):
    """
    Manual Chess Game System - Human vs Chess Engine
    The human makes moves manually (via CLI commands), and the chess engine
    responds automatically by moving pieces in Gazebo.
    """
    
    def __init__(self):
        super().__init__('manual_chess_game')
        self.board = chess.Board()
        self.game_active = True
        self.human_color = chess.WHITE
        
        self.set_entity_client = self.create_client(SetEntityState, '/world/chess_table_world/set_entity_state')
        
        while not self.set_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Ignition Gazebo set_entity_state service...')
        
        self.setup_coordinate_mapping()
        self.setup_piece_mapping()
        
        self.engine = None
        self.setup_chess_engine()
        
        self.get_logger().info('Manual Chess Game initialized!')
        self.get_logger().info('Human plays White, Engine plays Black')
        self.print_board()
        self.print_help()
    
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
    
    def setup_piece_mapping(self):
        """Setup piece name mapping"""
        self.current_piece_positions = {
            # White pieces (starting positions)
            'a1': 'white_rook_a1', 'b1': 'white_knight_b1', 'c1': 'white_bishop_c1', 'd1': 'white_queen_d1',
            'e1': 'white_king_e1', 'f1': 'white_bishop_f1', 'g1': 'white_knight_g1', 'h1': 'white_rook_h1',
            'a2': 'white_pawn_a2', 'b2': 'white_pawn_b2', 'c2': 'white_pawn_c2', 'd2': 'white_pawn_d2',
            'e2': 'white_pawn_e2', 'f2': 'white_pawn_f2', 'g2': 'white_pawn_g2', 'h2': 'white_pawn_h2',
            
            # Black pieces (starting positions)
            'a8': 'black_rook_a8', 'b8': 'black_knight_b8', 'c8': 'black_bishop_c8', 'd8': 'black_queen_d8',
            'e8': 'black_king_e8', 'f8': 'black_bishop_f8', 'g8': 'black_knight_g8', 'h8': 'black_rook_h8',
            'a7': 'black_pawn_a7', 'b7': 'black_pawn_b7', 'c7': 'black_pawn_c7', 'd7': 'black_pawn_d7',
            'e7': 'black_pawn_e7', 'f7': 'black_pawn_f7', 'g7': 'black_pawn_g7', 'h7': 'black_pawn_h7',
        }
        
        self.captured_pieces = []
        self.white_capture_area = (0.3, 0.4, self.board_height)
        self.black_capture_area = (0.3, -0.4, self.board_height)
    
    def setup_chess_engine(self):
        """Setup chess engine (Stockfish)"""
        try:
            stockfish_paths = [
                '/usr/bin/stockfish',
                '/usr/local/bin/stockfish', 
                '/opt/homebrew/bin/stockfish',
                'stockfish'  # If in PATH
            ]
            
            stockfish_path = None
            for path in stockfish_paths:
                if os.path.exists(path) or path == 'stockfish':
                    try:
                        engine = chess.engine.SimpleEngine.popen_uci(path)
                        engine.quit()
                        stockfish_path = path
                        break
                    except Exception:
                        continue
            
            if stockfish_path:
                self.engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)
                self.get_logger().info(f'Stockfish engine loaded from: {stockfish_path}')
            else:
                self.get_logger().warn('Stockfish not found. Engine moves will be random.')
                self.engine = None
                
        except Exception as e:
            self.get_logger().error(f'Failed to load chess engine: {e}')
            self.engine = None
    
    def move_piece_in_gazebo(self, piece_name: str, target_x: float, target_y: float, target_z: float) -> bool:
        """Move a piece to specific coordinates in Gazebo"""
        try:
            entity_state = EntityState()
            entity_state.name = piece_name
            entity_state.pose.position.x = target_x
            entity_state.pose.position.y = target_y
            entity_state.pose.position.z = target_z
            entity_state.pose.orientation.w = 1.0
            
            request = SetEntityState.Request()
            request.state = entity_state
            
            future = self.set_entity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            
            if result.success:
                self.get_logger().info(f'Moved {piece_name} to ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})')
                return True
            else:
                self.get_logger().error(f'Failed to move {piece_name}: {result.status_message}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error moving piece {piece_name}: {str(e)}')
            return False
    
    def move_piece_to_capture_area(self, piece_name: str, is_white_piece: bool) -> bool:
        """Move captured piece to storage area"""
        if is_white_piece:
            area = self.white_capture_area
        else:
            area = self.black_capture_area
        
        num_captured = len([p for p in self.captured_pieces if p[1] == is_white_piece])
        offset_x = 0.05 * (num_captured % 4)
        offset_y = 0.05 * (num_captured // 4)
        
        target_x = area[0] + offset_x
        target_y = area[1] + offset_y
        target_z = area[2]
        
        success = self.move_piece_in_gazebo(piece_name, target_x, target_y, target_z)
        if success:
            self.captured_pieces.append((piece_name, is_white_piece))
        
        return success
    
    def execute_move_in_gazebo(self, move: chess.Move) -> bool:
        """Execute a chess move by moving pieces in Gazebo"""
        from_square = chess.square_name(move.from_square)
        to_square = chess.square_name(move.to_square)
        
        piece_name = None
        for square, name in self.current_piece_positions.items():
            if square == from_square:
                piece_name = name
                break
        
        if not piece_name:
            self.get_logger().error(f'No piece found at {from_square}')
            return False
        
        captured_piece = None
        if to_square in self.current_piece_positions:
            captured_piece = self.current_piece_positions[to_square]
            is_white_capture = 'white' in captured_piece.lower()
            
            if not self.move_piece_to_capture_area(captured_piece, is_white_capture):
                self.get_logger().error(f'Failed to move captured piece {captured_piece}')
                return False
            
            del self.current_piece_positions[to_square]
            self.get_logger().info(f'Captured {captured_piece}')
        
        target_x, target_y, target_z = self.chess_to_coords[to_square]
        if not self.move_piece_in_gazebo(piece_name, target_x, target_y, target_z):
            return False
        
        del self.current_piece_positions[from_square]
        self.current_piece_positions[to_square] = piece_name
        
        self.get_logger().info(f'Moved {piece_name}: {from_square} → {to_square}')
        return True
    
    def make_human_move(self, move_str: str) -> bool:
        """Process a human move input"""
        try:
            move = chess.Move.from_uci(move_str)
            
            if move not in self.board.legal_moves:
                self.get_logger().error(f'Illegal move: {move_str}')
                self.get_logger().info('Legal moves: ' + ', '.join([m.uci() for m in self.board.legal_moves]))
                return False
            
            self.board.push(move)
            
            self.get_logger().info(f'Human move: {move_str}')
            self.print_board()
            
            if self.board.is_checkmate():
                self.get_logger().info('🎉 Human wins by checkmate!')
                self.game_active = False
                return True
            elif self.board.is_stalemate():
                self.get_logger().info('Draw by stalemate')
                self.game_active = False
                return True
            elif self.board.is_check():
                self.get_logger().info('🔥 Check!')
            
            return True
            
        except ValueError:
            self.get_logger().error(f'Invalid move format: {move_str}. Use format like "e2e4"')
            return False
    
    def make_engine_move(self) -> bool:
        """Let the chess engine make a move"""
        if not self.engine:
            legal_moves = list(self.board.legal_moves)
            if not legal_moves:
                return False
            
            move = np.random.choice(legal_moves)
            self.get_logger().info('Engine making random move (no Stockfish)')
        else:
            try:
                result = self.engine.play(self.board, chess.engine.Limit(time=1.0))
                move = result.move
            except Exception as e:
                self.get_logger().error(f'Engine error: {e}')
                return False
        
        self.board.push(move)
        
        if not self.execute_move_in_gazebo(move):
            self.get_logger().error('Failed to execute engine move in Gazebo')
            return False
        
        self.get_logger().info(f'🤖 Engine move: {move.uci()}')
        self.print_board()
        
        if self.board.is_checkmate():
            self.get_logger().info('🤖 Engine wins by checkmate!')
            self.game_active = False
        elif self.board.is_stalemate():
            self.get_logger().info('Draw by stalemate')
            self.game_active = False
        elif self.board.is_check():
            self.get_logger().info('🔥 Check!')
        
        return True
    
    def print_board(self):
        """Print current board state"""
        print("\n" + "="*50)
        print("Current Position:")
        print(self.board)
        print(f"Turn: {'White (Human)' if self.board.turn == chess.WHITE else 'Black (Engine)'}")
        print(f"Move: {self.board.fullmove_number}")
        if self.board.is_check():
            print("*** CHECK ***")
        print("="*50)
    
    def print_help(self):
        """Print help information"""
        print("""
Manual Chess Game Commands:
  
  To make a move: enter move in UCI format (e.g., 'e2e4', 'd7d5')
  
  Special commands:
    'help' - Show this help
    'board' - Show current board
    'moves' - Show legal moves  
    'quit' - End game
    'undo' - Undo last move (if possible)
    'reset' - Start new game
  
  Example moves:
    e2e4  - Move pawn from e2 to e4
    g1f3  - Move knight from g1 to f3
    e1g1  - Castle kingside (if legal)
    
  Goal: Beat the chess engine!
        """)
    
    def get_legal_moves_str(self) -> str:
        """Get formatted string of legal moves"""
        moves = [move.uci() for move in self.board.legal_moves]
        return ', '.join(sorted(moves))
    
    def undo_last_move(self) -> bool:
        """Undo the last move (human + engine)"""
        if len(self.board.move_stack) < 2:
            self.get_logger().warn('Not enough moves to undo')
            return False
        
        self.board.pop()
        self.board.pop()
        
        self.get_logger().info('Undid last move pair')
        self.print_board()
        return True
    
    def reset_game(self) -> bool:
        """Reset to new game"""
        self.board.reset()
        self.game_active = True
        self.get_logger().info('Game reset - new game started!')
        self.print_board()
        return True
    
    def cleanup(self):
        """Clean up resources"""
        if self.engine:
            self.engine.quit()

def main():
    rclpy.init()
    
    game = ManualChessGame()
    
    try:
        print("\n🎮 Manual Chess Game Started!")
        print("You are White. Enter your move (e.g., 'e2e4') or 'help' for commands.")
        
        while rclpy.ok() and game.game_active:
            if game.board.turn == chess.WHITE:
                try:
                    user_input = input("\nYour move: ").strip().lower()
                    
                    if user_input == 'quit':
                        break
                    elif user_input == 'help':
                        game.print_help()
                        continue
                    elif user_input == 'board':
                        game.print_board()
                        continue
                    elif user_input == 'moves':
                        print(f"Legal moves: {game.get_legal_moves_str()}")
                        continue
                    elif user_input == 'undo':
                        game.undo_last_move()
                        continue
                    elif user_input == 'reset':
                        game.reset_game()
                        continue
                    elif user_input == '':
                        continue
                    else:
                        if game.make_human_move(user_input):
                            if game.game_active and game.board.turn == chess.BLACK:
                                print("\nEngine thinking...")
                                time.sleep(1)  # Brief pause for effect
                                game.make_engine_move()
                        
                except KeyboardInterrupt:
                    break
                except EOFError:
                    break
            
            rclpy.spin_once(game, timeout_sec=0.1)
        
        print("\n🏁 Game ended. Thanks for playing!")
        
    except KeyboardInterrupt:
        print("\n👋 Game interrupted by user")
    finally:
        game.cleanup()
        game.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

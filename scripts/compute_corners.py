#!/usr/bin/env python3
import math
import numpy as np

def compute_corners_and_reach(robot_x, robot_y, robot_z, board_center_x, board_center_y, board_size, table_height):
    """
    Compute chessboard corners and analyze reachability from robot position.
    """
    # Board properties
    board_z = table_height + 0.005  # 5mm above table
    half_size = board_size / 2.0
    
    # Calculate corners
    corners = {
        'front_right': (board_center_x + half_size, board_center_y + half_size, board_z),
        'back_right':  (board_center_x + half_size, board_center_y - half_size, board_z),
        'back_left':   (board_center_x - half_size, board_center_y - half_size, board_z),
        'front_left':  (board_center_x - half_size, board_center_y + half_size, board_z),
    }
    
    # Calculate distances from robot base to each corner
    distances = {}
    for name, (x, y, z) in corners.items():
        # Horizontal distance from robot base
        dist = math.sqrt((x - robot_x)**2 + (y - robot_y)**2)
        distances[name] = dist
    
    return corners, distances

def analyze_board_placement(robot_pos, max_reach, board_size):
    """
    Analyze optimal board placement given robot position and reach.
    """
    robot_x, robot_y, robot_z = robot_pos
    
    print(f"\n=== Board Placement Analysis ===")
    print(f"Robot position: ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")
    print(f"Robot max reach: {max_reach:.3f} m")
    print(f"Desired board size: {board_size:.3f} m")
    
    # For a chess game, the robot plays black (far side)
    # Human sits opposite, playing white (near side)
    
    # Option 1: Board directly in front of robot
    print("\n--- Option 1: Board directly in front ---")
    # Place board so nearest edge is reachable
    board_center_x = robot_x + board_size/2 + 0.05  # 5cm clearance
    board_center_y = robot_y
    
    corners, distances = compute_corners_and_reach(
        robot_x, robot_y, robot_z,
        board_center_x, board_center_y, 
        board_size, robot_z
    )
    
    print(f"Board center: ({board_center_x:.3f}, {board_center_y:.3f})")
    print("Corner distances from robot:")
    for name, dist in distances.items():
        reachable = "✓" if dist <= max_reach else "✗"
        print(f"  {name:12s}: {dist:.3f} m {reachable}")
    
    # Option 2: Board offset to side (robot at corner)
    print("\n--- Option 2: Robot at board corner ---")
    # This gives maximum coverage
    board_center_x = robot_x + board_size/(2*math.sqrt(2))
    board_center_y = robot_y - board_size/(2*math.sqrt(2))
    
    corners, distances = compute_corners_and_reach(
        robot_x, robot_y, robot_z,
        board_center_x, board_center_y, 
        board_size, robot_z
    )
    
    print(f"Board center: ({board_center_x:.3f}, {board_center_y:.3f})")
    print("Corner distances from robot:")
    for name, dist in distances.items():
        reachable = "✓" if dist <= max_reach else "✗"
        print(f"  {name:12s}: {dist:.3f} m {reachable}")
    
    # Calculate maximum board size for each configuration
    print("\n=== Maximum Board Sizes ===")
    
    # For edge placement: furthest corner is at distance sqrt(board_size^2 + (board_size/2)^2)
    # Solving for board_size when this equals max_reach:
    max_board_edge = max_reach * 2 / math.sqrt(5)
    print(f"Max board size (robot at edge center): {max_board_edge:.3f} m")
    
    # For corner placement: furthest corner is at distance board_size*sqrt(2)
    max_board_corner = max_reach / math.sqrt(2)
    print(f"Max board size (robot at corner): {max_board_corner:.3f} m")

def suggest_optimal_setup():
    """
    Suggest optimal setup based on typical parameters.
    """
    print("\n=== OPTIMAL SETUP RECOMMENDATIONS ===")
    
    # From reach analysis (you'll update this with actual results)
    estimated_max_reach = 0.35  # Update this after running improved reach calculator
    safety_factor = 0.85  # Use 85% of max reach for reliable operation
    safe_reach = estimated_max_reach * safety_factor
    
    # Standard chess square sizes
    tournament_square = 0.055  # 5.5cm (middle of tournament range)
    board_size = tournament_square * 8
    
    print(f"\nAssuming max reach: {estimated_max_reach:.3f} m")
    print(f"Safe operating reach: {safe_reach:.3f} m")
    print(f"Standard tournament board: {board_size:.3f} m ({tournament_square*1000:.0f}mm squares)")
    
    # Calculate if standard board fits
    if board_size <= safe_reach * 2 / math.sqrt(5):
        print("✓ Standard tournament board FITS with robot at edge center!")
    else:
        # Calculate maximum square size
        max_board = safe_reach * 2 / math.sqrt(5)
        max_square = max_board / 8
        print(f"✗ Standard board too large. Maximum square size: {max_square*1000:.1f}mm")
        board_size = max_square * 8
    
    # Optimal position for human-robot game
    robot_x, robot_y, robot_z = 0.75, 0.25, 0.375  # From your launch file
    
    # Place board so robot is behind black pieces
    # Board center should be in front and slightly to the right
    optimal_board_x = robot_x + 0.05  # 5cm in front
    optimal_board_y = robot_y - board_size/2 - 0.05  # Offset so robot is behind black's right corner
    
    print(f"\nOptimal board placement:")
    print(f"  Board center: ({optimal_board_x:.3f}, {optimal_board_y:.3f})")
    print(f"  Board size: {board_size:.3f} m")
    print(f"  Square size: {board_size/8*1000:.1f} mm")
    
    # Human position
    human_x = optimal_board_x
    human_y = optimal_board_y - board_size/2 - 0.3  # 30cm from board edge
    print(f"\nHuman seating position: ({human_x:.3f}, {human_y:.3f})")
    print("  (Human plays white, faces the robot)")
    
    return optimal_board_x, optimal_board_y, board_size

def main():
    # Current setup from your files
    robot_x = 0.75   # From launch file
    robot_y = 0.25   # From launch file  
    robot_z = 0.375  # Table height
    
    table_center_x = 0.75
    table_center_y = 0.00
    table_height = 0.375
    
    current_square_size = 0.50  # Current blank square
    
    print("=== CURRENT SETUP ANALYSIS ===")
    
    # Analyze current setup
    corners, distances = compute_corners_and_reach(
        robot_x, robot_y, robot_z,
        table_center_x, table_center_y,
        current_square_size, table_height
    )
    
    print(f"\nRobot position: ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")
    print(f"Current square center: ({table_center_x:.3f}, {table_center_y:.3f})")
    print(f"Square size: {current_square_size:.3f} m")
    
    print("\nCurrent square corners (world frame):")
    for name, (x, y, z) in corners.items():
        dist = distances[name]
        print(f"  {name:12s}: ({x:.3f}, {y:.3f}, {z:.3f}) - Distance: {dist:.3f} m")
    
    # Show reach problem
    min_dist = min(distances.values())
    max_dist = max(distances.values())
    print(f"\nDistance range: {min_dist:.3f} m to {max_dist:.3f} m")
    print("With ~0.303m reach at rest, NO corners are reachable!")
    
    # Analyze better placements
    estimated_max_reach = 0.35  # Estimate - update after running reach calculator
    
    print("\n" + "="*50)
    analyze_board_placement((robot_x, robot_y, robot_z), estimated_max_reach, 0.4)
    
    print("\n" + "="*50)
    optimal_x, optimal_y, optimal_size = suggest_optimal_setup()
    
    # Generate updated values for the blank square
    print("\n" + "="*50)
    print("=== UPDATE YOUR blank_square IN chess_world.sdf ===")
    print(f"""
    <model name="blank_square">
      <pose>{optimal_x:.3f} {optimal_y:.3f} {table_height + 0.03:.3f} 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{optimal_size:.3f} {optimal_size:.3f} 0.01</size>
            </box>
          </geometry>
        </visual>
        <!-- ... rest of the model ... -->
      </link>
    </model>
    """)

if __name__ == '__main__':
    main()

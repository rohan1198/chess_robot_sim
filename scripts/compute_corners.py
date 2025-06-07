#!/usr/bin/env python3
import math

# Table and square parameters
table_center_x = 0.75   # must match your robot spawn X
table_center_y = 0.00   # must match your robot spawn Y
table_height   = 0.375  # your table top height
square_side    = 0.50   # full blank-square width
square_thick   = 0.005  # thickness

half = square_side / 2.0
z = table_height + square_thick / 2.0

corners = {
    'front_right': (table_center_x + half, table_center_y + half, z),
    'back_right' : (table_center_x + half, table_center_y - half, z),
    'back_left'  : (table_center_x - half, table_center_y - half, z),
    'front_left' : (table_center_x - half, table_center_y + half, z),
}

print("Chessboard calibration square corners (in world frame):\n")
for name, (x, y, z) in corners.items():
    print(f"{name:12s}: x = {x:7.3f} m, y = {y:7.3f} m, z = {z:7.3f} m")

# CHESS PIECE INTERACTION SEQUENCES

# 1. PREPARE FOR PIECE PICKUP - Open gripper first
echo "Opening gripper..."
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 1.5, max_effort: 5.0}}"

# Wait a bit, then continue...
sleep 3

# 2. REACH TOWARD WHITE PAWN (e2 - closest to robot)
echo "Reaching toward white pawn on e2..."
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 1}
  - positions: [-0.1, -0.8, 0.6, -0.5, 0.0]
    time_from_start: {sec: 5}
"

# 3. ALTERNATIVE: KNOCK DOWN A PIECE (sweep motion)
echo "Sweeping motion to knock down pieces..."
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 1}
  - positions: [-0.5, -0.6, 0.4, -0.3, 0.0]
    time_from_start: {sec: 4}
  - positions: [0.5, -0.6, 0.4, -0.3, 0.0]
    time_from_start: {sec: 8}
"

# 4. PRECISE PICKUP ATTEMPT (white pawn e2)
echo "Attempting precise pickup of white pawn..."
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 2}
  - positions: [0.05, -0.7, 0.5, -0.4, 0.0]
    time_from_start: {sec: 6}
  - positions: [0.05, -0.9, 0.7, -0.6, 0.0]
    time_from_start: {sec: 10}
"

# 5. CLOSE GRIPPER (after reaching piece)
echo "Closing gripper to grab piece..."
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.2, max_effort: 3.0}}"

# 6. LIFT PIECE UP
echo "Lifting piece up..."
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
  points:
  - positions: [0.05, -0.5, 0.3, -0.2, 0.0]
    time_from_start: {sec: 3}
"

# 7. MOVE PIECE TO NEW SQUARE (e4)
echo "Moving piece to e4..."
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
  points:
  - positions: [0.1, -0.7, 0.5, -0.4, 0.0]
    time_from_start: {sec: 4}
  - positions: [0.1, -0.9, 0.7, -0.6, 0.0]
    time_from_start: {sec: 7}
"

# 8. PLACE PIECE DOWN
echo "Placing piece down..."
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 1.5, max_effort: 3.0}}"

# 9. RETREAT TO SAFE POSITION
echo "Returning to safe position..."
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'wrist_roll_joint']
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 5}
"

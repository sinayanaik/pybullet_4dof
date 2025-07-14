# Simplified 3-DOF Planar Robot Arm Model

This is a simplified model of a planar robot arm with 3 moving joints (DOF) and one fixed vertical link, implemented using matplotlib for 2D visualization. The model is based on the URDF specification and provides basic forward kinematics and visualization capabilities. The robot operates in the X-Z plane, with all joints rotating around the Y-axis.

## Features

- Forward kinematics calculation for planar motion
- Inverse kinematics for target position control
- Interactive GUI for position control
- 2D visualization using matplotlib
- Simulation of robot motion in X-Z plane
- Workspace demonstration

## Robot Specifications

- 1 fixed vertical link (Link 1)
- 3 revolute joints (pitch joints around Y-axis)
  - Joint 2: Between Link 1 and Link 2
  - Joint 3: Between Link 2 and Link 3
  - Joint 4: Between Link 3 and Link 4
- Link lengths:
  - Link 1: 0.25m (fixed vertical)
  - Link 2: 0.15m
  - Link 3: 0.15m
  - Link 4: 0.10m
  - Gripper: 0.06m
- Joint limits: ±3.14 radians (±180 degrees) for all moving joints
- Planar motion: All movement confined to X-Z plane

## Files

- `robot_model.py`: Contains the `RobotModel` class with planar forward kinematics and 2D visualization methods
- `inverse_kinematics.py`: Implements inverse kinematics solver for target positions
- `interactive_control.py`: GUI for interactive end-effector position control
- `simulate_motion.py`: Demonstrates the robot's motion and workspace in the X-Z plane

## Usage

1. Make sure you have the required dependencies:
   ```bash
   pip install numpy matplotlib scipy
   ```

2. Run the interactive control:
   ```bash
   python interactive_control.py
   ```
   - Enter start and end coordinates in the input fields
   - Click "Move Robot" to execute the motion
   - The robot will move from start to end position smoothly

3. Run the motion simulation:
   ```bash
   python simulate_motion.py
   ```
   This will show:
   - A waving motion simulation in the X-Z plane
   - A workspace demonstration showing different configurations

## Interactive Control

The interactive control window allows you to:
- Enter X,Z coordinates for start and end positions
- Visualize the robot's current configuration
- Move the robot smoothly between positions
- Get feedback if positions are unreachable
- See the motion planning in real-time

## Visualization

- Blue lines: Robot links
- Black circle: Fixed base joint (Joint 1)
- Red circles: Moving joints (Joints 2, 3, and 4)
- Green circle: End-effector position
- Different colors in workspace demo: Various robot configurations
- Grid: Helps visualize the scale and position
- X-axis: Horizontal motion
- Z-axis: Vertical motion (height) 
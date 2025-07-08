# Joint State Display in PyBullet

This guide explains how to display joint states in a separate window using `joint_display.py`.

## Key Functions

### 1. `setup_joint_display(robot)`

#### Purpose
Sets up display parameters for each revolute joint.

#### Parameters
- `robot`: PyBullet body ID of the robot

#### Returns
- `joint_params`: Dictionary of joint display parameters
- `revolute_joints`: Dictionary mapping joint names to indices

### 2. `create_display_window()`

#### Purpose
Creates a Tkinter window for displaying joint information.

#### Returns
- `root`: Tkinter root window
- `frame`: Main frame for joint displays

### 3. `update_display(robot, joint_params, revolute_joints, root, frame)`

#### Purpose
Updates the Tkinter window with current joint states.

#### Parameters
- `robot`: PyBullet body ID
- `joint_params`: Joint parameters dictionary
- `revolute_joints`: Joint indices dictionary
- `root`: Tkinter root window
- `frame`: Main display frame

#### Implementation
```python
# Get joint state
state = p.getJointState(robot, joint_idx)
current_pos_rad = state[0]
current_vel = state[1]
reaction_forces = state[2]
motor_torque = state[3]

# Get link position
link_state = p.getLinkState(robot, joint_idx)
link_pos = link_state[0]  # World position
x_pos, z_pos = link_pos[0], link_pos[2]

# Convert to degrees and create display
current_pos_deg = rad2deg(current_pos_rad)
ttk.Label(joint_frame, text=f"Position: {current_pos_deg:.1f}°").grid(...)
ttk.Label(joint_frame, text=f"Coordinates: X={x_pos:.3f}, Z={z_pos:.3f} m").grid(...)
# ... etc
```

## Display Information

### 1. Joint Position
- Shown in degrees (°)
- One decimal precision
- Updates in real-time

### 2. Joint Coordinates
- X and Z coordinates in world frame
- Units: meters (m)
- Three decimal precision
- Real-time updates

### 3. Joint Velocity
- Angular velocity for revolute joints
- Units: radians/second
- One decimal precision

### 4. Reaction Forces
- Linear forces [fx, fy, fz]
- Units: Newtons (N)
- One decimal precision

### 5. Reaction Torques
- Angular torques [mx, my, mz]
- Units: Newton-meters (N⋅m)
- One decimal precision

### 6. Motor Torque
- Applied motor torque
- Units: Newton-meters (N⋅m)
- One decimal precision

## Window Configuration

### Layout
- Each joint has its own labeled frame
- Information organized in rows
- Clean, readable formatting

### Styling
- Padded frames and labels
- Clear hierarchical structure
- Consistent spacing

## Usage

### Basic Usage
```python
from joint_display import setup_joint_display, display_joint_states

# Get robot ID
robot, _ = spawn_robot()

# Setup displays
joint_params, revolute_joints = setup_joint_display(robot)

# Start display loop
display_joint_states(robot, joint_params, revolute_joints)
```

## Features

1. **Separate Window**
   - Independent from PyBullet window
   - Clean, native UI appearance
   - Easy to read and monitor

2. **Real-time Updates**
   - 60 Hz refresh rate
   - Smooth updates
   - Low resource usage

3. **Clean Layout**
   - Organized by joint
   - Clear data labeling
   - Professional appearance

4. **Position Tracking**
   - Joint angles in degrees
   - World coordinates (X, Z)
   - Real-time position updates

## Error Handling

1. **Window Management**
   - Handles window closing
   - Cleans up resources
   - Proper disconnection

2. **Data Processing**
   - Safe unit conversion
   - Proper number formatting
   - Handles invalid states

3. **Cleanup**
   - Window cleanup on exit
   - PyBullet disconnection
   - Resource management 
# Joint Control in PyBullet

This guide explains how to control robot joints using sliders in PyBullet using `joint_control.py`.

## Key Functions

### 1. Unit Conversion
```python
def rad2deg(rad):
    return rad * 180.0 / np.pi

def deg2rad(deg):
    return deg * np.pi / 180.0
```
- Convert between radians and degrees
- Used for displaying joint angles in human-readable format

### 2. `setup_joint_sliders(robot)`

#### Purpose
Creates interactive sliders and coordinate displays for controlling revolute joints.

#### Parameters
- `robot`: PyBullet body ID of the robot

#### Returns
- `joint_params`: Dictionary of joint parameters, sliders, and text displays
- `revolute_joints`: Dictionary mapping joint names to indices

#### Implementation Details
```python
# Get joint limits in radians
lower_limit_rad = joint_info[8]
upper_limit_rad = joint_info[9]

# Convert to degrees for display
lower_limit_deg = rad2deg(lower_limit_rad)
upper_limit_deg = rad2deg(upper_limit_rad)

# Create slider
slider = p.addUserDebugParameter(
    f"{joint_name} (deg)",  # Show units in name
    lower_limit_deg,        # Min value in degrees
    upper_limit_deg,        # Max value in degrees
    0                      # Initial position
)

# Create coordinate display
text_id = p.addUserDebugText(
    f"Coordinates: X=0.000, Z=0.000 m",
    [0.6, 0.8 - len(joint_params) * 0.1, 0],
    [1, 1, 1],  # White color
    textSize=1.2
)
```

### 3. `control_joints_with_sliders(robot, joint_params, revolute_joints)`

#### Purpose
Main control loop that reads slider values, updates joint positions, and displays coordinates.

#### Parameters
- `robot`: PyBullet body ID
- `joint_params`: Joint parameters dictionary
- `revolute_joints`: Joint indices dictionary

#### Implementation
```python
# Get slider value (in degrees)
target_pos_deg = p.readUserDebugParameter(params['slider'])

# Convert to radians for PyBullet
target_pos_rad = deg2rad(target_pos_deg)

# Set joint position
p.setJointMotorControl2(
    robot,
    joint_idx,
    p.POSITION_CONTROL,
    target_pos_rad,
    force=100
)

# Update coordinates display
link_state = p.getLinkState(robot, joint_idx)
link_pos = link_state[0]  # World position
x_pos, z_pos = link_pos[0], link_pos[2]
p.addUserDebugText(
    f"{name} coords: X={x_pos:.3f}, Z={z_pos:.3f} m",
    [0.6, 0.8 - index * 0.1, 0],
    [1, 1, 1]
)
```

## Usage

### Basic Usage
```python
from joint_control import setup_joint_sliders, control_joints_with_sliders

# Get robot ID
robot, _ = spawn_robot()

# Setup sliders
joint_params, revolute_joints = setup_joint_sliders(robot)

# Start control loop
control_joints_with_sliders(robot, joint_params, revolute_joints)
```

### Interface Elements
- Each joint has:
  - Position control slider (degrees)
  - Coordinate display (meters)
  - Real-time updates
  - Clear labeling

## Features

1. **Degree-Based Interface**
   - All angles shown in degrees
   - Automatic conversion to/from radians
   - Intuitive for users

2. **Joint Limits**
   - Respects URDF-defined limits
   - Prevents over-rotation
   - Shows valid range in UI

3. **Position Control**
   - Uses position control mode
   - Maximum force: 100 N
   - 240 Hz update rate

4. **Coordinate Display**
   - Real-time X,Z coordinates
   - World frame reference
   - 3 decimal precision
   - Units in meters

## Coordinate System

### World Frame
- Origin: At robot base
- X-axis: Forward/Backward
- Z-axis: Up/Down
- Units: Meters

### Display Format
- Position: Slider in degrees
- Coordinates: X,Z in meters
- Update Rate: 240 Hz
- Precision: 3 decimals

## Error Handling

1. **Slider Creation**
   - Only creates sliders for revolute joints
   - Skips fixed or prismatic joints
   - Validates joint limits

2. **Control Loop**
   - Handles KeyboardInterrupt
   - Cleans up UI elements
   - Proper PyBullet disconnection

3. **Value Conversion**
   - Safe conversion between units
   - Handles numerical precision
   - Prevents invalid positions

4. **Display Cleanup**
   - Removes all sliders
   - Cleans up coordinate displays
   - Proper resource management 
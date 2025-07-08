# Joint Monitoring in PyBullet

This guide explains how to monitor joint states in the terminal using `monitor_joints.py`.

## Key Functions

### 1. `get_joint_info(robot)`

#### Purpose
Prints detailed information about all joints in the robot.

#### Parameters
- `robot`: PyBullet body ID of the robot

#### Output Format
```
Robot has X joints

Joint Information:
--------------------------------------------------

Joint 0: joint_name
  Type: Revolute
  Limits (deg): [-90.0, 90.0]
  Max Force: 100.0
  Max Velocity: 10.0
```

#### Implementation
```python
joint_info = p.getJointInfo(robot, i)
joint_name = joint_info[1].decode('utf-8')
joint_type = joint_info[2]

# Get type name from dictionary
type_name = {
    p.JOINT_REVOLUTE: "Revolute",
    p.JOINT_PRISMATIC: "Prismatic",
    p.JOINT_SPHERICAL: "Spherical",
    p.JOINT_PLANAR: "Planar",
    p.JOINT_FIXED: "Fixed"
}.get(joint_type, "Unknown")
```

### 2. `monitor_joint_states(robot)`

#### Purpose
Continuously monitors and prints joint states to the terminal.

#### Parameters
- `robot`: PyBullet body ID of the robot

#### Output Format
```
Current Joint States:
--------------------------------------------------

joint_name:
  Position: 45.0°
  Coordinates: X=0.123, Z=0.456 m
  Velocity: 0.5
  Forces (xyz): [0.0, 0.0, -9.8]
  Torques (rpy): [0.0, 0.0, 0.0]
  Motor Torque: 0.0
```

#### Implementation
```python
# Get joint state
state = p.getJointState(robot, i)
pos_deg = rad2deg(state[0])
vel = state[1]
forces = state[2]
torque = state[3]

# Get link position
link_state = p.getLinkState(robot, i)
link_pos = link_state[0]  # World position
x_pos, z_pos = link_pos[0], link_pos[2]  # X and Z coordinates

print(f"\n{joint_name}:")
print(f"  Position: {pos_deg:.1f}°")
print(f"  Coordinates: X={x_pos:.3f}, Z={z_pos:.3f} m")
print(f"  Velocity: {vel:.1f}")
print(f"  Forces (xyz): [{forces[0]:.1f}, {forces[1]:.1f}, {forces[2]:.1f}]")
print(f"  Torques (rpy): [{forces[3]:.1f}, {forces[4]:.1f}, {forces[5]:.1f}]")
print(f"  Motor Torque: {torque:.1f}")
```

## Usage

### Basic Usage
```python
from monitor_joints import get_joint_info, monitor_joint_states

# Get robot ID
robot, _ = spawn_robot()

# Print initial info
get_joint_info(robot)

# Start monitoring
monitor_joint_states(robot)
```

## Features

1. **Initial Joint Information**
   - Joint types and names
   - Position limits in degrees
   - Force and velocity limits
   - Clear formatting

2. **Real-time Monitoring**
   - Position in degrees
   - World coordinates (X, Z)
   - Current velocity
   - Force/torque feedback
   - Motor torque status

3. **Terminal Output**
   - Clean text formatting
   - 0.5 second update rate
   - Easy-to-read layout
   - Units included

4. **Position Tracking**
   - Angular position in degrees
   - Cartesian coordinates in meters
   - Real-time updates
   - High precision (3 decimals for coordinates)

## Data Types

### Joint Types
- JOINT_REVOLUTE (0)
- JOINT_PRISMATIC (1)
- JOINT_SPHERICAL (2)
- JOINT_PLANAR (3)
- JOINT_FIXED (4)

### State Information
1. **Position**
   - Angular: Degrees for revolute joints
   - Linear: Meters for prismatic joints
   - Coordinates: X,Z position in world frame (meters)

2. **Velocity**
   - rad/s for revolute joints
   - m/s for prismatic joints

3. **Forces/Torques**
   - Forces in Newtons (N)
   - Torques in Newton-meters (N⋅m)

## Error Handling

1. **Type Conversion**
   - Safe byte string decoding
   - Unit conversion checks
   - Unknown joint type handling

2. **Data Display**
   - Consistent decimal places
   - Clear unit labeling
   - Organized formatting

3. **Program Control**
   - KeyboardInterrupt handling
   - Clean PyBullet disconnect
   - Resource cleanup

## Coordinate System

### World Frame
- Origin: At the base of the robot
- X-axis: Forward/Backward
- Z-axis: Up/Down
- Units: Meters

### Display Format
- X coordinate: 3 decimal precision
- Z coordinate: 3 decimal precision
- Updated in real-time
- Displayed with units (m) 
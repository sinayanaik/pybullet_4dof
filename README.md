# Robot Simulation and Analysis Project

This project implements a 3-DOF robot simulation with real-time visualization, joint control, and data analysis capabilities using PyBullet. The project focuses on simulating robot dynamics, tracking end-effector positions, and analyzing joint torques.

## Project Structure

```
.
├── scripts/
│   ├── test_robot.py         # Main simulation script
│   └── plot_robot_data.py    # Data visualization script
├── urdf/
│   ├── robot.urdf           # Compiled URDF file
│   ├── robot.urdf.xacro    # Main robot description
│   └── robot_materials.xacro # Material definitions
├── data/
│   ├── robot_data_*.csv    # Generated data files
│   └── robot_data_*.png    # Generated plot files
└── README.md
```

## Dependencies

- Python 3.x
- PyBullet (`pip install pybullet`)
- NumPy (`pip install numpy`)
- Pandas (`pip install pandas`)
- Matplotlib (`pip install matplotlib`)

## Robot Description

The robot is a 3-DOF manipulator with the following features:
- Fixed base
- 4 revolute joints
- Gripper end-effector
- Visual materials for better appearance
- Properly defined inertial properties

### URDF Structure
- Base link with fixed joint to world
- 4 links connected by revolute joints
- Joint limits: ±1.57 radians (±90 degrees)
- Maximum joint torque: 50 Nm
- Maximum joint velocity: 1.0 rad/s

## Main Components

### 1. Robot Simulation (`scripts/test_robot.py`)

#### Key Functions:

##### `get_end_effector_state(robot, gripper_link_index=-1)`
- **Purpose**: Gets end-effector position and orientation
- **Parameters**:
  - `robot`: PyBullet robot ID
  - `gripper_link_index`: Index of end-effector link (default: last link)
- **Returns**: Dictionary with:
  - `position`: [x, y, z] coordinates
  - `euler_angles`: [roll, pitch, yaw] in radians

##### `get_joint_forces(robot, joint_indices)`
- **Purpose**: Reads joint torque sensors
- **Parameters**:
  - `robot`: PyBullet robot ID
  - `joint_indices`: List of joint indices to monitor
- **Returns**: Dictionary mapping joint names to torque values (Nm)

## Control Strategy Details

### Position Control Implementation

The project uses PyBullet's built-in Position Control mode through `p.setJointMotorControl2()`. This is a PD (Proportional-Derivative) controller implemented internally by PyBullet.

#### Controller Type: PD Position Control
```python
p.setJointMotorControl2(
    robot,                    # Robot ID
    joint_id,                 # Joint to control
    p.POSITION_CONTROL,       # Control mode
    targetPosition=position,  # Desired position
    force=100.0,             # Maximum force/torque
    maxVelocity=1.0          # Velocity limit
)
```

#### Control Parameters:
1. **Position Target**
   - Generated using sinusoidal functions
   - `position = 0.5 * sin(t + i * π/4)`
   - Amplitude: ±0.5 radians (≈±28.6 degrees)
   - Phase offset: `i * π/4` for each joint
   - Smooth, continuous motion

2. **Force Limit (100.0 N)**
   - Maximum force/torque the controller can apply
   - Prevents excessive motor effort
   - Safety feature to avoid damaging the robot
   - Higher than joint torque limits (50 Nm) to ensure tracking

3. **Velocity Limit (1.0 rad/s)**
   - Maximum joint velocity
   - Ensures smooth, controlled motion
   - Matches URDF specifications
   - Safety feature to prevent rapid movements

4. **Control Frequency**
   - Update rate: 100 Hz (0.01s timestep)
   - Balances between:
     - Control precision
     - Computational load
     - Real-time performance

#### How the Controller Works:

1. **Position Error Calculation**
   ```
   error = targetPosition - currentPosition
   ```

2. **PD Control Law (internal to PyBullet)**
   ```
   torque = Kp * position_error + Kd * velocity_error
   torque = clamp(torque, -force_limit, +force_limit)
   ```
   Where:
   - `Kp`: Position gain (internally tuned)
   - `Kd`: Velocity gain (internally tuned)

3. **Velocity Limiting**
   ```
   velocity = clamp(computed_velocity, -maxVelocity, +maxVelocity)
   ```

4. **Joint Limits**
   - Enforced by URDF specifications
   - Position: ±1.57 radians (±90 degrees)
   - Maximum torque: 50 Nm
   - Maximum velocity: 1.0 rad/s

#### Trajectory Generation:

```python
positions = [
    0.5 * np.sin(t + i * np.pi/4)  # Different phase for each joint
    for i in range(len(controllable_joints))
]
```

- Each joint follows a sinusoidal trajectory
- Phase differences create coordinated motion
- Continuous, differentiable path
- Easy to modify for different patterns

#### Advantages of This Control Strategy:

1. **Simplicity**
   - Easy to implement and understand
   - Built-in PyBullet implementation
   - Minimal tuning required

2. **Stability**
   - Natural motion damping
   - Smooth trajectory following
   - Built-in safety limits

3. **Performance**
   - Computationally efficient
   - Real-time capable
   - Suitable for testing and prototyping

4. **Flexibility**
   - Easy to modify trajectories
   - Adjustable limits and parameters
   - Compatible with various robot configurations

#### Limitations:

1. **No Direct Force Control**
   - Cannot directly control interaction forces
   - Limited for contact tasks

2. **Fixed Gains**
   - Internal PD gains not directly accessible
   - Cannot optimize for specific tasks

3. **Simple Trajectory**
   - Basic sinusoidal motion
   - Not task-specific
   - No obstacle avoidance

#### Alternative Control Strategies:

1. **Velocity Control**
   ```python
   p.setJointMotorControl2(
       robot, joint_id,
       p.VELOCITY_CONTROL,
       targetVelocity=velocity
   )
   ```

2. **Torque Control**
   ```python
   p.setJointMotorControl2(
       robot, joint_id,
       p.TORQUE_CONTROL,
       force=torque
   )
   ```

3. **Future Improvements**
   - Implement inverse kinematics
   - Add trajectory planning
   - Include obstacle avoidance
   - Develop task-specific controllers

#### Data Logging:
- CSV format with timestamps
- End-effector position and orientation
- Joint torques
- Real-time data flushing for safety

### 2. Data Visualization (`scripts/plot_robot_data.py`)

#### Plot Types:
1. **End Effector Trajectory (3D)**
   - Complete path visualization
   - Start (green) and end (red) points marked
   - 3D perspective with grid

2. **End Effector Position vs Time**
   - X, Y, Z coordinates over time
   - Shows motion timing
   - Grid for easy reading

3. **End Effector Orientation**
   - Roll, pitch, yaw angles
   - Converted to degrees for readability
   - Shows orientation changes

4. **Joint Torques**
   - Individual torque plots for each joint
   - Shows load distribution
   - Helps identify peak loads

## Why This Approach?

1. **PyBullet Over ROS2**
   - Lighter weight solution
   - Easier setup and dependencies
   - Good for prototyping and testing
   - Built-in physics engine

2. **Position Control**
   - Simple and stable control method
   - Good for trajectory following
   - Suitable for most basic applications

3. **Data Collection Strategy**
   - Real-time logging prevents data loss
   - Comprehensive data collection for analysis
   - CSV format for easy post-processing

4. **Visualization Approach**
   - Multiple plot types for complete analysis
   - Real-time and post-process visualization
   - Clear representation of robot state

## Usage Guide

1. **Setup Environment**
   ```bash
   pip install pybullet numpy pandas matplotlib
   ```

2. **Run Simulation**
   ```bash
   python3 scripts/test_robot.py
   ```
   - Watch real-time visualization
   - Press Ctrl+C to stop and save data

3. **Analyze Data**
   ```bash
   python3 scripts/plot_robot_data.py
   ```
   - Views the most recent data file
   - Creates PNG plot file in data directory

## Customization Options

1. **Robot Parameters**
   - Modify URDF files for different robot configurations
   - Adjust joint limits and properties

2. **Control Parameters**
   - Change trajectory pattern in `scripts/test_robot.py`
   - Adjust control gains and limits

3. **Visualization**
   - Modify plot styles in `scripts/plot_robot_data.py`
   - Add new analysis plots

## Common Issues and Solutions

1. **PyBullet Installation**
   - Use pip for standard install
   - Check Python version compatibility

2. **URDF Loading**
   - Ensure correct file paths
   - Check URDF syntax
   - Verify joint definitions

3. **Data Collection**
   - Monitor CSV file size
   - Adjust sampling rate if needed
   - Check disk space

## Future Improvements

1. **Additional Features**
   - Inverse kinematics implementation
   - Path planning capabilities
   - Collision detection

2. **Analysis Tools**
   - Real-time plotting
   - Advanced analytics
   - Performance metrics

3. **User Interface**
   - GUI for parameter adjustment
   - Interactive control interface
   - Real-time data visualization

## Contributing

Feel free to contribute by:
1. Forking the repository
2. Creating feature branches
3. Submitting pull requests

## License

This project is open-source and available under the MIT License. 
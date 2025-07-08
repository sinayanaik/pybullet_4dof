# Robot Spawning in PyBullet

This guide explains how to spawn a robot in PyBullet using `spawn_robot.py`.

## Function: `spawn_robot()`

### Purpose
Initializes PyBullet and loads a robot URDF with proper configuration.

### Implementation
```python
def spawn_robot():
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load ground plane and robot
    planeId = p.loadURDF("plane.urdf")
    robot = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=True)

    # Configure camera
    p.resetDebugVisualizerCamera(...)

    return robot, physicsClient
```

### Parameters
- No input parameters required

### Returns
- `robot`: Unique integer ID for the loaded robot
- `physicsClient`: PyBullet physics server ID

### Key Components

1. **PyBullet Connection**
   - Uses GUI mode for visualization
   - Sets up search paths for assets
   - Configures gravity (-9.81 m/s² in Z direction)

2. **Ground Plane**
   - Loads infinite checkerboard ground at z=0
   - Provides collision surface for testing

3. **Robot Loading**
   - Loads URDF from `../urdf/robot.urdf`
   - Places robot at origin [0, 0, 0]
   - Sets zero orientation [0, 0, 0]
   - Uses fixed base for stationary operation

4. **Camera Setup**
   - Distance: 1.0 meters
   - Yaw: 0 degrees (front view)
   - Pitch: 0 degrees (horizontal)
   - Target: [0, 0, 0] (robot base)

## Usage

### Basic Usage
```python
from spawn_robot import spawn_robot

# Get robot and physics client IDs
robot, physics_client = spawn_robot()

# Use robot ID for control and monitoring
num_joints = p.getNumJoints(robot)
```

### Simulation Loop
```python
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)  # 240 Hz simulation
except KeyboardInterrupt:
    p.disconnect()
```

### Camera Controls
- Left Mouse + Drag: Rotate view
- Right Mouse + Drag: Pan view
- Mouse Wheel: Zoom
- Middle Mouse + Drag: Pan camera
- Ctrl + Left Mouse + Drag: Roll camera

## Error Handling

1. **URDF Loading Fails**
   - Check file path is correct
   - Verify URDF syntax
   - Ensure meshes exist

2. **GUI Issues**
   - Verify GPU drivers
   - Check OpenGL support
   - Try `p.DIRECT` mode

3. **Robot Position**
   - Verify startPos coordinates
   - Check URDF root link position
   - Confirm unit conversions 
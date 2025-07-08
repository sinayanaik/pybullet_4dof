## import all dependencies for working with pybullet and computing math
import pybullet as p
import pybullet_data
import numpy as np
import time
import os

## load the urdf
def setup_simulation():
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set up simulation
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)  # Disable real-time simulation for better control
    
    # Load ground plane
    planeId = p.loadURDF("plane.urdf")
    
    # Load our robot URDF
    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "urdf", "robot.urdf")
    robot = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=True)
    
    # Set camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.3]
    )
    
    return robot

def get_controllable_joints(robot):
    """Get indices of controllable (revolute) joints, excluding visual joints"""
    controllable_joints = []
    for i in range(p.getNumJoints(robot)):
        joint_info = p.getJointInfo(robot, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        # Only include revolute joints and exclude visual joints
        if joint_type == p.JOINT_REVOLUTE and 'visual' not in joint_name:
            controllable_joints.append(i)
            print(f"Found controllable joint: {joint_name} at index {i}")
    return controllable_joints

def get_end_effector_link(robot):
    """Get the end effector link index"""
    num_joints = p.getNumJoints(robot)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_name = joint_info[1].decode('utf-8')
        if 'link_4' in joint_name:  # The last link in our URDF
            return i
    return num_joints - 1  # Fallback to last joint

def calculate_workspace_points(robot, ee_link, resolution=5):  # Reduced to 5 points per joint
    """Calculate workspace points using forward kinematics"""
    controllable_joints = get_controllable_joints(robot)
    points = []
    debug_lines = []
    
    # Get the first three joints (excluding gripper)
    main_joints = controllable_joints[:3]
    
    # Define joint ranges
    joint_ranges = []
    for joint_id in main_joints:
        joint_info = p.getJointInfo(robot, joint_id)
        joint_ranges.append(np.linspace(joint_info[8], joint_info[9], resolution))
    
    print(f"\nCalculating workspace with {resolution}^3 = {resolution**3} points...")
    total_points = resolution**3
    points_done = 0
    
    # Sample points in workspace
    for theta1 in joint_ranges[0]:
        for theta2 in joint_ranges[1]:
            for theta3 in joint_ranges[2]:
                # Reset all joints to zero first
                for joint_id in controllable_joints:
                    p.resetJointState(robot, joint_id, 0)
                
                # Set positions for main joints
                for i, joint_id in enumerate(main_joints):
                    p.resetJointState(robot, joint_id, [theta1, theta2, theta3][i])
                
                # Get end effector position
                state = p.getLinkState(robot, ee_link)
                pos = state[0]
                points.append((pos[0], pos[2]))  # Only X and Z coordinates
                
                # Draw debug point in PyBullet (blue color)
                debug_lines.append(p.addUserDebugPoints(
                    [[pos[0], 0, pos[2]]], 
                    [[0, 0, 1]], 
                    pointSize=3  # Increased point size for better visibility
                ))
                
                # Step simulation to show smooth visualization
                p.stepSimulation()
                time.sleep(0.05)  # Increased delay for slower visualization
                
                # Show progress
                points_done += 1
                if points_done % resolution == 0:
                    print(f"Progress: {points_done}/{total_points} points ({(points_done/total_points*100):.1f}%)")
    
    print("Workspace calculation complete!")
    
    # Reset joint positions
    for joint_id in controllable_joints:
        p.resetJointState(robot, joint_id, 0)
    
    return debug_lines

def get_end_effector_state(robot, ee_link):
    """Get the position of the end effector"""
    state = p.getLinkState(robot, ee_link)
    return state[0]  # Return position [x, y, z]

def get_joint_info(robot):
    """Get detailed joint information including limits"""
    joint_info = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        joint_info[i] = {
            'name': info[1].decode('utf-8'),
            'type': info[2],
            'lower_limit': info[8],
            'upper_limit': info[9],
            'max_force': info[10],
            'max_velocity': info[11]
        }
    return joint_info

def calculate_ik_with_orientation(robot, ee_link, target_pos, current_positions=None, num_attempts=5):
    """Calculate IK with multiple attempts and orientation constraints"""
    best_solution = None
    min_error = float('inf')
    
    # Define multiple orientation attempts
    orientation_attempts = [
        p.getQuaternionFromEuler([0, 0, 0]),  # End effector level
        p.getQuaternionFromEuler([0, np.pi/6, 0]),  # Slightly tilted up
        p.getQuaternionFromEuler([0, -np.pi/6, 0]),  # Slightly tilted down
    ]
    
    for target_orn in orientation_attempts:
        for _ in range(num_attempts):
            # Add small random offset to current positions to avoid local minima
            if current_positions:
                positions = [pos + np.random.uniform(-0.1, 0.1) for pos in current_positions]
            else:
                positions = current_positions

            solution = p.calculateInverseKinematics(
                robot,
                ee_link,
                target_pos,
                target_orn,
                maxNumIterations=100,
                residualThreshold=1e-5,
                currentPositions=positions,
                jointDamping=[0.1] * 6  # Add damping to stabilize solution
            )
            
            # Check solution quality
            p.setJointMotorControlArray(
                robot,
                range(p.getNumJoints(robot)),
                p.POSITION_CONTROL,
                targetPositions=solution
            )
            p.stepSimulation()
            
            actual_pos = p.getLinkState(robot, ee_link)[0]
            error = np.sqrt(sum((a - b) ** 2 for a, b in zip(actual_pos, target_pos)))
            
            if error < min_error:
                min_error = error
                best_solution = solution
    
    return best_solution if best_solution is not None else solution

def main():
    # Setup simulation and load robot
    robot = setup_simulation()
    
    # Get controllable joints and end effector link
    controllable_joints = get_controllable_joints(robot)
    ee_link = get_end_effector_link(robot)
    
    # Get joint information
    joint_info = get_joint_info(robot)
    
    # Get the first three joints (excluding gripper)
    main_joints = controllable_joints[:3]
    
    # Visualize workspace
    workspace_points = calculate_workspace_points(robot, ee_link)
    
    # Add origin marker and text
    p.addUserDebugText(
        "(0,0)", 
        [0.05, 0, 0.05],
        textColorRGB=[1, 0, 0],
        textSize=1.5
    )
    
    # Add a small red sphere at origin
    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.01,
        rgbaColor=[1, 0, 0, 1]
    )
    p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[0, 0, 0]
    )
    
    # Add debug parameters for target position and simulation speed
    target_x = p.addUserDebugParameter("Target X", -0.4, 0.4, 0.2)
    target_z = p.addUserDebugParameter("Target Z", 0.0, 0.8, 0.4)
    sim_speed = p.addUserDebugParameter("Simulation Speed", 0.1, 2.0, 1.0)
    
    # Add line to visualize target and error
    target_visual = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 0, 0])
    error_line = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0])
    ee_pos_text = p.addUserDebugText("", [0.5, 0, 0])
    
    print("\nWorkspace visualization complete. Use sliders to control target position and simulation speed.")
    print("Press Ctrl+C to exit.")
    
    try:
        while True:
            # Get target position and simulation speed from sliders
            x = p.readUserDebugParameter(target_x)
            z = p.readUserDebugParameter(target_z)
            speed_factor = p.readUserDebugParameter(sim_speed)
            
            try:
                # Get current joint positions
                current_positions = []
                for joint_id in controllable_joints:
                    current_positions.append(p.getJointState(robot, joint_id)[0])
                
                # Calculate IK with improved orientation handling
                target_pos = [x, 0, z]
                joint_poses = calculate_ik_with_orientation(
                    robot,
                    ee_link,
                    target_pos,
                    current_positions
                )
                
                # Set joint positions for the main joints (excluding gripper)
                for i, joint_id in enumerate(main_joints):
                    # Get joint limits
                    lower_limit = joint_info[joint_id]['lower_limit']
                    upper_limit = joint_info[joint_id]['upper_limit']
                    
                    # Clamp joint angle to limits
                    target_angle = np.clip(joint_poses[i], lower_limit, upper_limit)
                    
                    p.setJointMotorControl2(
                        robot, 
                        joint_id,
                        p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        force=5.0,
                        maxVelocity=0.1 * speed_factor,
                        positionGain=0.3,  # Lower position gain for smoother motion
                        velocityGain=0.5   # Lower velocity gain for smoother motion
                    )
                
                # Keep gripper position unchanged
                p.setJointMotorControl2(
                    robot,
                    controllable_joints[-1],
                    p.POSITION_CONTROL,
                    targetPosition=0,
                    force=5.0,
                    maxVelocity=0.1 * speed_factor
                )
                
                # Get current end effector position
                ee_pos = get_end_effector_state(robot, ee_link)
                
                # Update target visualization (red dot)
                p.addUserDebugLine([x, 0, z], [x, 0, z], [1, 0, 0], 5.0, replaceItemUniqueId=target_visual)
                
                # Draw line between target and current position (green)
                p.addUserDebugLine(
                    [x, 0, z],
                    [ee_pos[0], 0, ee_pos[2]],
                    [0, 1, 0],
                    1.0,
                    replaceItemUniqueId=error_line
                )
                
                # Calculate error
                error = np.sqrt((x - ee_pos[0])**2 + (z - ee_pos[2])**2)
                
                # Update status text with error and speed
                status_text = f"Target: ({x:.3f}, {z:.3f})\n"
                status_text += f"Current: ({ee_pos[0]:.3f}, {ee_pos[2]:.3f})\n"
                status_text += f"Error: {error:.3f} m\n"
                status_text += f"Speed: {speed_factor:.1f}x"
                p.addUserDebugText(
                    status_text,
                    [0.5, 0, 0],
                    textColorRGB=[1, 1, 1],
                    replaceItemUniqueId=ee_pos_text
                )
                
            except Exception as e:
                # If IK fails, display error
                p.addUserDebugText(
                    f"Error: {str(e)}",
                    [0.5, 0, 0],
                    textColorRGB=[1, 0, 0],
                    replaceItemUniqueId=ee_pos_text
                )
            
            # Step simulation with speed-dependent delay
            p.stepSimulation()
            time.sleep(0.05 / speed_factor)
            
    except KeyboardInterrupt:
        # Clean up debug visualizations
        for point in workspace_points:
            p.removeUserDebugItem(point[0])
        p.disconnect()

if __name__ == "__main__":
    main()

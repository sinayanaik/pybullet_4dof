#!/usr/bin/env python3

import pybullet as p
import pybullet_data
import time
import numpy as np
import os
import csv
from datetime import datetime

def get_end_effector_state(robot, gripper_link_index=-1):
    """Get the position and orientation of the end effector."""
    if gripper_link_index == -1:
        # Get the last link (gripper)
        gripper_link_index = p.getNumJoints(robot) - 1
        
    state = p.getLinkState(robot, gripper_link_index)
    position = state[0]  # Cartesian world position [x,y,z]
    orientation = state[1]  # Quaternion [x,y,z,w]
    
    # Convert quaternion to euler angles for better readability
    euler = p.getEulerFromQuaternion(orientation)
    
    return {
        'position': position,
        'euler_angles': euler  # In radians [roll, pitch, yaw]
    }

def get_joint_forces(robot, joint_indices):
    """Get the forces/torques for the specified joints."""
    joint_forces = {}
    for joint_id in joint_indices:
        # Get joint state including applied forces
        joint_state = p.getJointState(robot, joint_id)
        # joint_state[2] is a 6-element tuple (fx, fy, fz, mx, my, mz)
        # For revolute joints, we're interested in the torque around the joint axis
        # The axis depends on the joint configuration, but typically it's around Y axis (my)
        joint_name = p.getJointInfo(robot, joint_id)[1].decode('utf-8')
        # Get the magnitude of the torque (using my since joints rotate around Y axis)
        torque = joint_state[2][4]  # my component
        joint_forces[joint_name] = torque
    return joint_forces

def main():
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
    # Update path to URDF file
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "urdf", "robot.urdf")
    robot = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=True)
    
    # Get number of joints and their info
    num_joints = p.getNumJoints(robot)
    print(f"\nNumber of joints: {num_joints}")
    print("\nJoint Information:")
    print("-" * 50)
    
    # Store the indices of controllable joints
    controllable_joints = []
    joint_names = {}  # Store joint names for display
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        
        if joint_type == p.JOINT_REVOLUTE:
            controllable_joints.append(i)
            joint_names[i] = joint_name
            print(f"Joint {i}: {joint_name}")
            print(f"  Type: Revolute")
            print(f"  Lower limit: {joint_info[8]:.2f}")
            print(f"  Upper limit: {joint_info[9]:.2f}")
            print(f"  Max force: {joint_info[10]:.2f}")
            print(f"  Max velocity: {joint_info[11]:.2f}")
            print("-" * 50)
    
    # Find gripper link index
    gripper_link_index = -1
    for i in range(num_joints):
        if p.getJointInfo(robot, i)[1].decode('utf-8') == "link_4_to_gripper":
            gripper_link_index = i
            break
    
    # Set camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.3]
    )
    
    # Add a debug line to visualize end effector position
    line_id = p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 0, 0])  # Red line for visualization
    
    # Add text for displaying coordinates and forces
    position_text_id = p.addUserDebugText("", [0.5, 0, 0])
    force_text_ids = {}
    y_offset = 0
    for joint_id in controllable_joints:
        force_text_ids[joint_id] = p.addUserDebugText("", [0.5, 0, 0.6 - y_offset])
        y_offset += 0.05
    
    # Enable joint control and torque sensors
    for joint_id in controllable_joints:
        p.enableJointForceTorqueSensor(robot, joint_id, 1)
    
    # Create CSV file for data logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Update path to data directory
    data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")
    os.makedirs(data_dir, exist_ok=True)
    csv_filename = os.path.join(data_dir, f"robot_data_{timestamp}.csv")
    print(f"\nLogging data to: {csv_filename}")
    
    # Prepare CSV headers
    headers = ['timestamp', 
              'ee_pos_x', 'ee_pos_y', 'ee_pos_z',
              'ee_rot_x', 'ee_rot_y', 'ee_rot_z']
    
    # Add joint torque headers
    for joint_id in controllable_joints:
        headers.append(f"{joint_names[joint_id]}_torque")
    
    with open(csv_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        
        try:
            print("\nMoving joints in a simple pattern...")
            print("Press Ctrl+C to exit")
            print("\nTracking end effector position and joint forces...")
            
            # Simple joint movement pattern
            t = 0
            start_time = time.time()
            
            while True:
                # Calculate joint positions using sine waves
                positions = [
                    0.5 * np.sin(t + i * np.pi/4)  # Different phase for each joint
                    for i in range(len(controllable_joints))
                ]
                
                # Set joint positions
                for i, joint_id in enumerate(controllable_joints):
                    p.setJointMotorControl2(
                        robot, joint_id,
                        p.POSITION_CONTROL,
                        targetPosition=positions[i],
                        force=100.0,
                        maxVelocity=1.0
                    )
                
                # Get end effector state
                ee_state = get_end_effector_state(robot, gripper_link_index)
                pos = ee_state['position']
                euler = ee_state['euler_angles']
                
                # Get joint forces
                joint_forces = get_joint_forces(robot, controllable_joints)
                
                # Update debug visualization
                p.addUserDebugLine([0, 0, 0], pos, [1, 0, 0], 1, replaceItemUniqueId=line_id)
                
                # Update position text
                status_text = f"End Effector:\n"
                status_text += f"Position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})\n"
                status_text += f"Rotation: ({euler[0]:.3f}, {euler[1]:.3f}, {euler[2]:.3f})"
                p.addUserDebugText(status_text, [0.5, 0, 0], replaceItemUniqueId=position_text_id)
                
                # Update force texts
                y_offset = 0
                for joint_id in controllable_joints:
                    joint_name = joint_names[joint_id]
                    force = joint_forces[joint_name]
                    force_text = f"{joint_name}: {force:.2f} Nm"
                    p.addUserDebugText(force_text, [0.5, 0, 0.6 - y_offset], 
                                     replaceItemUniqueId=force_text_ids[joint_id],
                                     textColorRGB=[1, 0.5, 0])  # Orange color for forces
                    y_offset += 0.05
                
                # Log data to CSV
                current_time = time.time() - start_time
                row_data = [current_time]
                row_data.extend([pos[0], pos[1], pos[2]])
                row_data.extend([euler[0], euler[1], euler[2]])
                
                # Add joint torques to row
                for joint_id in controllable_joints:
                    row_data.append(joint_forces[joint_names[joint_id]])
                
                writer.writerow(row_data)
                csvfile.flush()  # Ensure data is written immediately
                
                # Step simulation
                p.stepSimulation()
                time.sleep(0.01)
                t += 0.01
                
        except KeyboardInterrupt:
            print(f"\nExiting... Data saved to {csv_filename}")
        finally:
            p.disconnect()

if __name__ == "__main__":
    main() 
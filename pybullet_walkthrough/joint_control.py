import pybullet as p
import numpy as np
from spawn_robot import spawn_robot
import time

def rad2deg(rad):
    return rad * 180.0 / np.pi

def deg2rad(deg):
    return deg * np.pi / 180.0

def setup_joint_sliders(robot):
    # Dictionary to store joint parameters
    joint_params = {}
    revolute_joints = {}
    
    # Get number of joints
    num_joints = p.getNumJoints(robot)
    
    # Create sliders for revolute joints
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        
        # Only create controls for revolute joints
        if joint_type == p.JOINT_REVOLUTE:
            revolute_joints[joint_name] = i
            
            # Get joint limits in radians
            lower_limit_rad = joint_info[8]
            upper_limit_rad = joint_info[9]
            
            # Convert limits to degrees for display
            lower_limit_deg = rad2deg(lower_limit_rad)
            upper_limit_deg = rad2deg(upper_limit_rad)
            
            # Create slider with degree values
            slider = p.addUserDebugParameter(
                f"{joint_name} (deg)",  # Add units to name
                lower_limit_deg,
                upper_limit_deg,
                0  # initial position in degrees
            )
            
            # Create text display for coordinates
            text_id = p.addUserDebugText(
                f"Coordinates: X=0.000, Z=0.000 m",
                [0.6, 0.8 - len(joint_params) * 0.1, 0],  # Position in 3D space
                [1, 1, 1],  # White color
                textSize=1.2
            )
            
            # Store parameters
            joint_params[joint_name] = {
                'slider': slider,
                'text_id': text_id,
                'limits_rad': (lower_limit_rad, upper_limit_rad),
                'limits_deg': (lower_limit_deg, upper_limit_deg)
            }
            
            print(f"\nJoint {i}: {joint_name}")
            print(f"  Type: Revolute")
            print(f"  Limits (deg): [{lower_limit_deg:.1f}, {upper_limit_deg:.1f}]")
    
    return joint_params, revolute_joints

def control_joints_with_sliders(robot, joint_params, revolute_joints):
    try:
        while True:
            # Update joint states based on sliders
            for name, params in joint_params.items():
                joint_idx = revolute_joints[name]
                
                # Get current slider value (in degrees)
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
                
                # Get and display current coordinates
                link_state = p.getLinkState(robot, joint_idx)
                link_pos = link_state[0]  # World position of center of mass
                x_pos, z_pos = link_pos[0], link_pos[2]  # Extract X and Z coordinates
                
                # Update coordinate display
                if params['text_id'] is not None:
                    p.removeUserDebugItem(params['text_id'])
                params['text_id'] = p.addUserDebugText(
                    f"{name} coords: X={x_pos:.3f}, Z={z_pos:.3f} m",
                    [0.6, 0.8 - list(joint_params.keys()).index(name) * 0.1, 0],
                    [1, 1, 1],  # White color
                    textSize=1.2
                )
            
            p.stepSimulation()
            time.sleep(1./240.)  # 240 Hz simulation
            
    except KeyboardInterrupt:
        # Clean up debug items
        for params in joint_params.values():
            p.removeUserDebugItem(params['slider'])
            if params['text_id'] is not None:
                p.removeUserDebugItem(params['text_id'])
        p.disconnect()

def main():
    # Spawn robot and get the robot ID
    robot, physics_client = spawn_robot()
    
    # Setup joint sliders
    joint_params, revolute_joints = setup_joint_sliders(robot)
    
    # Start control loop
    control_joints_with_sliders(robot, joint_params, revolute_joints)

if __name__ == "__main__":
    main() 
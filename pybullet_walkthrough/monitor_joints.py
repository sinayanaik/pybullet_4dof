import pybullet as p
import numpy as np
from spawn_robot import spawn_robot
import time

def rad2deg(rad):
    return rad * 180.0 / np.pi

def get_joint_info(robot):
    """Get information about all joints"""
    num_joints = p.getNumJoints(robot)
    print(f"\nRobot has {num_joints} joints")
    print("\nJoint Information:")
    print("-" * 50)
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        
        # Get type name
        type_name = {
            p.JOINT_REVOLUTE: "Revolute",
            p.JOINT_PRISMATIC: "Prismatic",
            p.JOINT_SPHERICAL: "Spherical",
            p.JOINT_PLANAR: "Planar",
            p.JOINT_FIXED: "Fixed"
        }.get(joint_type, "Unknown")
        
        print(f"\nJoint {i}: {joint_name}")
        print(f"  Type: {type_name}")
        if joint_type == p.JOINT_REVOLUTE:
            lower_deg = rad2deg(joint_info[8])
            upper_deg = rad2deg(joint_info[9])
            print(f"  Limits (deg): [{lower_deg:.1f}, {upper_deg:.1f}]")
            print(f"  Max Force: {joint_info[10]:.1f}")
            print(f"  Max Velocity: {joint_info[11]:.1f}")

def monitor_joint_states(robot):
    """Monitor joint states in real-time"""
    try:
        while True:
            num_joints = p.getNumJoints(robot)
            print("\nCurrent Joint States:")
            print("-" * 50)
            
            for i in range(num_joints):
                joint_info = p.getJointInfo(robot, i)
                joint_name = joint_info[1].decode('utf-8')
                joint_type = joint_info[2]
                
                if joint_type == p.JOINT_REVOLUTE:
                    # Get joint state
                    state = p.getJointState(robot, i)
                    pos_deg = rad2deg(state[0])
                    vel = state[1]
                    forces = state[2]
                    torque = state[3]
                    
                    # Get link position
                    link_state = p.getLinkState(robot, i)
                    link_pos = link_state[0]  # World position of center of mass
                    x_pos, z_pos = link_pos[0], link_pos[2]  # Extract X and Z coordinates
                    
                    print(f"\n{joint_name}:")
                    print(f"  Position: {pos_deg:.1f}°")
                    print(f"  Coordinates: X={x_pos:.3f}, Z={z_pos:.3f} m")
                    print(f"  Velocity: {vel:.1f}")
                    print(f"  Forces (xyz): [{forces[0]:.1f}, {forces[1]:.1f}, {forces[2]:.1f}]")
                    print(f"  Torques (rpy): [{forces[3]:.1f}, {forces[4]:.1f}, {forces[5]:.1f}]")
                    print(f"  Motor Torque: {torque:.1f}")
            
            print("\nPress Ctrl+C to exit")
            p.stepSimulation()
            time.sleep(0.5)  # Update every 0.5 seconds for readability
            
    except KeyboardInterrupt:
        p.disconnect()

def main():
    # Spawn robot and get the robot ID
    robot, physics_client = spawn_robot()
    
    # Print initial joint information
    get_joint_info(robot)
    
    # Start monitoring
    monitor_joint_states(robot)

if __name__ == "__main__":
    main() 
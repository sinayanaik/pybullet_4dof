import pybullet as p
import numpy as np
from spawn_robot import spawn_robot
import time
import tkinter as tk
from tkinter import ttk

def rad2deg(rad):
    return rad * 180.0 / np.pi

def setup_joint_display(robot):
    # Dictionary to store joint parameters
    joint_params = {}
    revolute_joints = {}
    
    # Get number of joints
    num_joints = p.getNumJoints(robot)
    
    # Create text displays for revolute joints
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        
        # Only create displays for revolute joints
        if joint_type == p.JOINT_REVOLUTE:
            revolute_joints[joint_name] = i
            joint_params[joint_name] = {}
    
    return joint_params, revolute_joints

def create_display_window():
    # Create main window
    root = tk.Tk()
    root.title("Joint States Display")
    
    # Configure style
    style = ttk.Style()
    style.configure("Joint.TLabelframe", padding=10)
    style.configure("Joint.TLabel", padding=5)
    
    # Create frame for joint displays
    frame = ttk.Frame(root, padding="10")
    frame.grid(row=0, column=0, sticky="nsew")  # Changed to string format
    
    return root, frame

def update_display(robot, joint_params, revolute_joints, root, frame):
    # Clear previous widgets
    for widget in frame.winfo_children():
        widget.destroy()
    
    # Update joint state displays
    for row, (name, _) in enumerate(joint_params.items()):
        joint_idx = revolute_joints[name]
        
        # Get current joint state
        state = p.getJointState(robot, joint_idx)
        current_pos_rad = state[0]
        current_vel = state[1]
        reaction_forces = state[2]
        motor_torque = state[3]
        
        # Get link state for position
        link_state = p.getLinkState(robot, joint_idx)
        link_pos = link_state[0]  # World position of center of mass
        x_pos, z_pos = link_pos[0], link_pos[2]  # Extract X and Z coordinates
        
        # Convert position to degrees for display
        current_pos_deg = rad2deg(current_pos_rad)
        
        # Create frame for this joint
        joint_frame = ttk.LabelFrame(frame, text=name, style="Joint.TLabelframe")
        joint_frame.grid(row=row, column=0, padx=5, pady=5, sticky="ew")
        
        # Add labels with joint information
        ttk.Label(joint_frame, 
                 text=f"Position: {current_pos_deg:.1f}°",
                 style="Joint.TLabel").grid(row=0, column=0, sticky="w")
        
        ttk.Label(joint_frame,
                 text=f"Coordinates: X={x_pos:.3f}, Z={z_pos:.3f} m",
                 style="Joint.TLabel").grid(row=1, column=0, sticky="w")
        
        ttk.Label(joint_frame,
                 text=f"Velocity: {current_vel:.1f} rad/s",
                 style="Joint.TLabel").grid(row=2, column=0, sticky="w")
        
        ttk.Label(joint_frame,
                 text=f"Forces: [{reaction_forces[0]:.1f}, {reaction_forces[1]:.1f}, {reaction_forces[2]:.1f}] N",
                 style="Joint.TLabel").grid(row=3, column=0, sticky="w")
        
        ttk.Label(joint_frame,
                 text=f"Torques: [{reaction_forces[3]:.1f}, {reaction_forces[4]:.1f}, {reaction_forces[5]:.1f}] N⋅m",
                 style="Joint.TLabel").grid(row=4, column=0, sticky="w")
        
        ttk.Label(joint_frame,
                 text=f"Motor Torque: {motor_torque:.1f} N⋅m",
                 style="Joint.TLabel").grid(row=5, column=0, sticky="w")
    
    # Update the window
    root.update()

def display_joint_states(robot, joint_params, revolute_joints):
    # Create display window
    root, frame = create_display_window()
    
    try:
        while True:
            update_display(robot, joint_params, revolute_joints, root, frame)
            p.stepSimulation()
            time.sleep(1./60.)  # 60 Hz update rate for display
            
    except KeyboardInterrupt:
        root.destroy()
        p.disconnect()
    except tk.TclError:  # Handle window closing
        p.disconnect()

def main():
    # Spawn robot and get the robot ID
    robot, physics_client = spawn_robot()
    
    # Setup joint displays
    joint_params, revolute_joints = setup_joint_display(robot)
    
    # Start display loop
    display_joint_states(robot, joint_params, revolute_joints)

if __name__ == "__main__":
    main() 
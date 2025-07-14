import numpy as np
import matplotlib.pyplot as plt
from robot_model import RobotModel
import time

def simulate_wave_motion():
    """Simulate a waving motion of the robot"""
    robot = RobotModel()
    
    # Create figure
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Set axis limits
    ax.set_xlim([-0.8, 0.8])
    ax.set_ylim([0, 0.8])
    
    # Set title
    ax.set_title('3-DOF Planar Robot Arm Simulation')
    
    # Generate waving motion
    t = np.linspace(0, 2*np.pi, 50)
    for angle in t:
        ax.clear()
        
        # Set limits and labels
        ax.set_xlim([-0.8, 0.8])
        ax.set_ylim([0, 0.8])
        ax.set_title('3-DOF Planar Robot Arm Simulation')
        
        # Calculate joint angles for waving motion
        theta = [
            0.5 + 0.3 * np.sin(angle),  # Joint 2
            -0.5 + 0.3 * np.sin(angle), # Joint 3
            0.3 * np.sin(angle)         # Joint 4
        ]
        
        # Plot robot
        robot.plot_robot(ax, theta)
        
        # Draw and pause
        plt.draw()
        plt.pause(0.01)
    
    plt.show()

def demonstrate_workspace():
    """Demonstrate robot's workspace by showing multiple configurations"""
    robot = RobotModel()
    
    # Create figure
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Set axis limits
    ax.set_xlim([-0.8, 0.8])
    ax.set_ylim([0, 0.8])
    
    # Set title
    ax.set_title('Robot Workspace Demonstration')
    
    # Generate different configurations
    configs = [
        [0, 0, 0],                    # Home position
        [0.5, -0.5, 0],              # Reaching forward
        [0.8, -0.8, 0],              # Reaching up
        [-0.5, -0.5, 0.5],           # Reaching down
        [0.8, -0.3, 0]               # Extended position
    ]
    
    # Plot each configuration with different colors
    colors = ['blue', 'green', 'red', 'purple', 'orange']
    for theta, color in zip(configs, colors):
        robot.plot_robot(ax, theta, color=color)
    
    plt.show()

if __name__ == "__main__":
    print("Simulating wave motion...")
    simulate_wave_motion()
    
    print("\nDemonstrating workspace...")
    demonstrate_workspace() 
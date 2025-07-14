import numpy as np
import matplotlib.pyplot as plt

class RobotModel:
    def __init__(self):
        # Link lengths from URDF
        self.l1 = 0.25  # Link 1 length (fixed vertical)
        self.l2 = 0.15  # Link 2 length
        self.l3 = 0.15  # Link 3 length
        self.l4 = 0.10  # Link 4 length
        self.l_gripper = 0.06  # Gripper length
        
        # Joint limits from URDF (only for moving joints)
        self.joint_limits = np.array([
            [-3.14, 3.14],  # Joint 2
            [-3.14, 3.14],  # Joint 3
            [-3.14, 3.14]   # Joint 4
        ])
        
    def forward_kinematics(self, theta):
        """
        Calculate forward kinematics for the planar robot (X-Z plane)
        theta: list of 3 joint angles [θ2, θ3, θ4] for the moving joints
        Returns: list of points defining the robot's configuration
        """
        # Base position
        x = [0]  # X coordinates
        z = [0]  # Z coordinates
        
        # Link 1 - Fixed vertical
        x.append(0)
        z.append(self.l1)
        
        # Current angle (starts at 0 since first link is vertical)
        current_angle = 0
        
        # Link 2
        current_angle += theta[0]  # First moving joint (θ2)
        x.append(x[-1] + self.l2 * np.sin(current_angle))
        z.append(z[-1] + self.l2 * np.cos(current_angle))
        
        # Link 3
        current_angle += theta[1]  # Second moving joint (θ3)
        x.append(x[-1] + self.l3 * np.sin(current_angle))
        z.append(z[-1] + self.l3 * np.cos(current_angle))
        
        # Link 4
        current_angle += theta[2]  # Third moving joint (θ4)
        x.append(x[-1] + self.l4 * np.sin(current_angle))
        z.append(z[-1] + self.l4 * np.cos(current_angle))
        
        # Gripper
        x.append(x[-1] + self.l_gripper * np.sin(current_angle))
        z.append(z[-1] + self.l_gripper * np.cos(current_angle))
        
        return np.array([x, z])
    
    def plot_robot(self, ax, theta, color='blue'):
        """Plot the robot in a given configuration in X-Z plane"""
        points = self.forward_kinematics(theta)
        x, z = points
        
        # Plot links
        ax.plot(x, z, color=color, linewidth=2)
        
        # Plot joints as circles (excluding base and first joint which is fixed)
        ax.scatter(x[2:-1], z[2:-1], color='red', s=50)  # Only moving joints
        
        # Plot base joint in black (fixed)
        ax.scatter(x[1], z[1], color='black', s=50)
        
        # Plot end-effector
        ax.scatter(x[-1], z[-1], color='green', s=50)
        
        # Set equal aspect ratio
        ax.set_aspect('equal')
        
        # Add grid
        ax.grid(True)
        
        # Label axes
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Z (m)') 
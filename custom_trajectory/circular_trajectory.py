import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, messagebox
import time
import os

def spawn_robot():
    """Spawn the robot in PyBullet simulation"""
    # Connect to PyBullet
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load ground plane
    planeId = p.loadURDF("plane.urdf")

    # Load our robot
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "..", "urdf", "robot.urdf")
    
    # Set initial pose
    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    
    # Load robot with fixed base
    robot = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=True)

    # Set camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=0,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    return robot, physicsClient

class CircularTrajectoryGUI:
    def __init__(self):
        # Create main window
        self.root = tk.Tk()
        self.root.title("Circular Trajectory Control")
        
        # Spawn robot
        self.robot_id, self.physics_client = spawn_robot()
        
        # Get joint indices
        self.joint_indices = {
            'link_1_to_link_2': 2,
            'link_2_to_link_3': 3,
            'link_3_to_link_4': 4
        }
        self.gripper_joint_index = 5  # link_4_to_gripper is the end effector
        
        # Create control frame
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=0, column=0, sticky="nsew")
        
        # Create input fields for circle parameters
        ttk.Label(control_frame, text="Circle Center:").grid(row=0, column=0, columnspan=2)
        ttk.Label(control_frame, text="X:").grid(row=1, column=0)
        ttk.Label(control_frame, text="Z:").grid(row=2, column=0)
        self.center_x = ttk.Entry(control_frame, width=10)
        self.center_z = ttk.Entry(control_frame, width=10)
        self.center_x.grid(row=1, column=1)
        self.center_z.grid(row=2, column=1)
        self.center_x.insert(0, "0.3")  # Default X center
        self.center_z.insert(0, "0.3")  # Default Z center
        
        ttk.Label(control_frame, text="Radius:").grid(row=3, column=0)
        self.radius = ttk.Entry(control_frame, width=10)
        self.radius.grid(row=3, column=1)
        self.radius.insert(0, "0.1")  # Default radius
        
        # Create speed control
        ttk.Label(control_frame, text="Speed:").grid(row=4, column=0)
        self.speed = ttk.Scale(control_frame, from_=0.1, to=2.0, orient=tk.HORIZONTAL)
        self.speed.grid(row=4, column=1)
        self.speed.set(1.0)  # Default speed
        
        # Create control buttons
        ttk.Button(control_frame, text="Start Motion", command=self.start_circular_motion).grid(row=5, column=0, columnspan=2, pady=5)
        ttk.Button(control_frame, text="Stop Motion", command=self.stop_motion).grid(row=6, column=0, columnspan=2, pady=5)
        
        # Create matplotlib figures
        self.setup_plots()
        
        # Motion control variables
        self.is_running = False
        self.phase = 0  # Track circle phase
        
        # Data storage for plotting
        self.time_data = []
        self.actual_x = []
        self.actual_z = []
        self.desired_x = []
        self.desired_z = []
        self.error_x = []
        self.error_z = []
        self.start_time = None
        
        # Update plots
        self.update_plots()
    
    def setup_plots(self):
        """Setup matplotlib plots"""
        # Create figure with subplots
        self.fig = plt.figure(figsize=(12, 8))
        
        # Trajectory plot
        self.ax1 = self.fig.add_subplot(211)
        self.ax1.set_title('End Effector Trajectory')
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Z Position (m)')
        self.ax1.grid(True)
        self.ax1.set_aspect('equal')
        
        # Initialize plot lines for trajectory
        self.actual_line, = self.ax1.plot([], [], 'b-', label='Actual')
        self.desired_line, = self.ax1.plot([], [], 'r--', label='Desired')
        self.current_point, = self.ax1.plot([], [], 'rx', markersize=10)
        self.ax1.legend()
        
        # Error plot
        self.ax2 = self.fig.add_subplot(212)
        self.ax2.set_title('Position Error')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Error (m)')
        self.ax2.grid(True)
        
        # Initialize plot lines for error
        self.error_x_line, = self.ax2.plot([], [], 'r-', label='X Error')
        self.error_z_line, = self.ax2.plot([], [], 'b-', label='Z Error')
        self.ax2.legend()
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=0, column=1)
        
        plt.tight_layout()
    
    def update_plots(self):
        """Update all plots"""
        # Update trajectory plot
        self.actual_line.set_data(self.actual_x, self.actual_z)
        self.desired_line.set_data(self.desired_x, self.desired_z)
        
        if hasattr(self, 'center_x_val'):
            # Plot the target circle
            circle = plt.Circle(
                (self.center_x_val, self.center_z_val),
                self.radius_val,
                fill=False,
                linestyle='--',
                color='red'
            )
            # Remove old circle if exists
            for patch in self.ax1.patches:
                patch.remove()
            self.ax1.add_patch(circle)
            
            # Update current target point
            x = self.center_x_val + self.radius_val * np.cos(self.phase)
            z = self.center_z_val + self.radius_val * np.sin(self.phase)
            self.current_point.set_data([x], [z])
        
        # Auto-scale trajectory plot
        if len(self.actual_x) > 0:
            all_x = self.actual_x + self.desired_x
            all_z = self.actual_z + self.desired_z
            x_min, x_max = min(all_x), max(all_x)
            z_min, z_max = min(all_z), max(all_z)
            x_margin = (x_max - x_min) * 0.1
            z_margin = (z_max - z_min) * 0.1
            self.ax1.set_xlim(x_min - x_margin, x_max + x_margin)
            self.ax1.set_ylim(z_min - z_margin, z_max + z_margin)
        
        # Update error plot
        self.error_x_line.set_data(self.time_data, self.error_x)
        self.error_z_line.set_data(self.time_data, self.error_z)
        
        if len(self.time_data) > 0:
            self.ax2.set_xlim(min(self.time_data), max(self.time_data))
            all_errors = self.error_x + self.error_z
            error_min, error_max = min(all_errors), max(all_errors)
            error_margin = (error_max - error_min) * 0.1
            self.ax2.set_ylim(error_min - error_margin, error_max + error_margin)
        
        self.canvas.draw()
    
    def get_end_effector_state(self):
        """Get current end effector position"""
        state = p.getLinkState(self.robot_id, self.gripper_joint_index)
        return np.array([state[0][0], state[0][2]])  # x, z coordinates
    
    def stop_motion(self):
        """Stop the circular motion"""
        self.is_running = False
    
    def start_circular_motion(self):
        """Start the circular motion with given parameters"""
        try:
            # Get circle parameters
            self.center_x_val = float(self.center_x.get())
            self.center_z_val = float(self.center_z.get())
            self.radius_val = float(self.radius.get())
            
            # Reset data storage
            self.time_data = []
            self.actual_x = []
            self.actual_z = []
            self.desired_x = []
            self.desired_z = []
            self.error_x = []
            self.error_z = []
            self.phase = 0
            self.start_time = time.time()
            
            # Start motion
            self.is_running = True
            self.execute_circular_motion()
            
        except ValueError:
            messagebox.showerror("Error", "Invalid input! Please enter valid numbers.")
    
    def execute_circular_motion(self):
        """Execute the circular motion"""
        if not self.is_running:
            return
            
        try:
            # Get current speed setting
            speed = self.speed.get()
            
            # Update phase based on speed
            self.phase = (self.phase + 0.05 * speed) % (2 * np.pi)
            
            # Calculate current target point
            x = self.center_x_val + self.radius_val * np.cos(self.phase)
            z = self.center_z_val + self.radius_val * np.sin(self.phase)
            
            # Store desired position
            self.desired_x.append(x)
            self.desired_z.append(z)
            
            # Calculate inverse kinematics
            target_pos = [x, 0, z]  # y = 0 for planar motion
            joint_poses = p.calculateInverseKinematics(
                self.robot_id,
                self.gripper_joint_index,
                target_pos,
                maxNumIterations=100,
                residualThreshold=1e-5
            )
            
            # Apply joint positions
            for joint_name, joint_idx in self.joint_indices.items():
                # Map joint index to IK solution index
                ik_index = joint_idx - 2
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_idx,
                    p.POSITION_CONTROL,
                    targetPosition=joint_poses[ik_index],
                    force=100
                )
            
            # Step simulation
            p.stepSimulation()
            
            # Get actual end effector position
            actual_pos = self.get_end_effector_state()
            current_time = time.time() - self.start_time
            
            # Store actual position and error
            self.actual_x.append(actual_pos[0])
            self.actual_z.append(actual_pos[1])
            self.error_x.append(x - actual_pos[0])
            self.error_z.append(z - actual_pos[1])
            self.time_data.append(current_time)
            
            # Update plots every few steps for better performance
            if len(self.time_data) % 10 == 0:
                self.update_plots()
            
            # Schedule next update if still running
            if self.is_running:
                self.root.after(20, self.execute_circular_motion)
            
        except Exception as e:
            messagebox.showerror("Error", str(e))
            self.is_running = False
    
    def run(self):
        """Start the GUI"""
        self.root.mainloop()

if __name__ == "__main__":
    # Create and run circular trajectory GUI
    gui = CircularTrajectoryGUI()
    print("Starting Circular Trajectory Control GUI...")
    gui.run() 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, messagebox
from robot_model import RobotModel
from inverse_kinematics import inverse_kinematics

class RobotControlGUI:
    def __init__(self):
        self.robot = RobotModel()
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Robot Arm Control")
        
        # Create left frame for controls
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create coordinate input fields
        ttk.Label(control_frame, text="Start Position:").grid(row=0, column=0, columnspan=2)
        ttk.Label(control_frame, text="X:").grid(row=1, column=0)
        ttk.Label(control_frame, text="Z:").grid(row=2, column=0)
        self.start_x = ttk.Entry(control_frame, width=10)
        self.start_z = ttk.Entry(control_frame, width=10)
        self.start_x.grid(row=1, column=1)
        self.start_z.grid(row=2, column=1)
        
        ttk.Label(control_frame, text="End Position:").grid(row=3, column=0, columnspan=2)
        ttk.Label(control_frame, text="X:").grid(row=4, column=0)
        ttk.Label(control_frame, text="Z:").grid(row=5, column=0)
        self.end_x = ttk.Entry(control_frame, width=10)
        self.end_z = ttk.Entry(control_frame, width=10)
        self.end_x.grid(row=4, column=1)
        self.end_z.grid(row=5, column=1)
        
        # Create move button
        ttk.Button(control_frame, text="Move Robot", command=self.move_robot).grid(row=6, column=0, columnspan=2)
        
        # Create matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().grid(row=0, column=1)
        
        # Set up the plot
        self.setup_plot()
        
        # Initial robot configuration
        self.current_angles = [0, 0, 0]
        self.update_plot()
    
    def setup_plot(self):
        """Set up the matplotlib plot"""
        self.ax.set_xlim([-0.8, 0.8])
        self.ax.set_ylim([0, 0.8])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Z (m)')
        self.ax.set_title('3-DOF Planar Robot Arm')
        self.ax.grid(True)
    
    def update_plot(self):
        """Update the robot visualization"""
        self.ax.clear()
        self.setup_plot()
        self.robot.plot_robot(self.ax, self.current_angles)
        self.canvas.draw()
    
    def move_robot(self):
        """Move the robot from start to end position"""
        try:
            # Get target positions
            start_x = float(self.start_x.get())
            start_z = float(self.start_z.get())
            end_x = float(self.end_x.get())
            end_z = float(self.end_z.get())
            
            # Check if positions are within workspace
            max_reach = self.robot.l2 + self.robot.l3 + self.robot.l4 + self.robot.l_gripper
            if abs(start_x) > max_reach or abs(end_x) > max_reach or \
               start_z > max_reach or end_z > max_reach or \
               start_z < 0 or end_z < 0:
                messagebox.showerror("Error", "Position out of workspace!")
                return
            
            # Get joint angles for start and end positions
            start_angles = inverse_kinematics(self.robot, start_x, start_z)
            end_angles = inverse_kinematics(self.robot, end_x, end_z)
            
            # Create trajectory
            num_steps = 50
            trajectory = []
            for i in range(num_steps):
                t = i / (num_steps - 1)
                angles = start_angles * (1 - t) + end_angles * t
                trajectory.append(angles)
            
            # Execute trajectory
            for angles in trajectory:
                self.current_angles = angles
                self.update_plot()
                self.root.update()
                plt.pause(0.01)
            
        except ValueError as e:
            messagebox.showerror("Error", str(e))
        except Exception as e:
            messagebox.showerror("Error", "Invalid input!")
    
    def run(self):
        """Start the GUI"""
        self.root.mainloop()

if __name__ == "__main__":
    gui = RobotControlGUI()
    gui.run() 
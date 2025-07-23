import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import os
import csv
from datetime import datetime
from scipy.optimize import minimize
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

class RealTimeVisualizer:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Real-time Semi-Circle Trajectory and Torque Visualization")
        self.root.protocol("WM_DELETE_WINDOW", self.close)  # Handle window close button
        
        # Create main frame
        self.main_frame = tk.Frame(self.root)
        self.main_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)
        
        # Create control frame
        self.control_frame = tk.Frame(self.root)
        self.control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10)
        
        # Create matplotlib figure with three subplots
        self.fig = Figure(figsize=(10, 10))
        
        # Trajectory subplot
        self.ax_traj = self.fig.add_subplot(311)
        self.ax_traj.set_xlabel('X Position (m)')
        self.ax_traj.set_ylabel('Z Position (m)')
        self.ax_traj.grid(True)
        self.ax_traj.set_title('End-Effector Trajectory')
        
        # Torque subplot
        self.ax_torque = self.fig.add_subplot(312)
        self.ax_torque.set_xlabel('Time Step')
        self.ax_torque.set_ylabel('Torque (N⋅m)')
        self.ax_torque.grid(True)
        self.ax_torque.set_title('Joint Torques')
        
        # Joint angles subplot
        self.ax_angles = self.fig.add_subplot(313)
        self.ax_angles.set_xlabel('Time Step')
        self.ax_angles.set_ylabel('Joint Angle (rad)')
        self.ax_angles.grid(True)
        self.ax_angles.set_title('Joint Angles')
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.main_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        
        # Initialize trajectory data lists
        self.target_x = []
        self.target_z = []
        self.actual_x = []
        self.actual_z = []
        
        # Initialize torque data lists
        self.time_steps = []
        self.torques_joint1 = []
        self.torques_joint2 = []
        self.torques_joint3 = []
        
        # Initialize joint angle data lists
        self.angles_joint1 = []
        self.angles_joint2 = []
        self.angles_joint3 = []
        self.step_counter = 0
        
        # Initialize window closed flag
        self.window_closed = False
        
        # Create plot lines for trajectory
        self.target_line, = self.ax_traj.plot([], [], 'r-', label='Target')
        self.actual_line, = self.ax_traj.plot([], [], 'b--', label='Actual')
        self.ax_traj.legend()
        
        # Create plot lines for torques
        self.torque_lines = [
            self.ax_torque.plot([], [], label=f'Joint {i+1}')[0]
            for i in range(3)
        ]
        self.ax_torque.legend()
        
        # Create plot lines for joint angles
        self.angle_lines = [
            self.ax_angles.plot([], [], label=f'Joint {i+1}')[0]
            for i in range(3)
        ]
        self.ax_angles.legend()
        
        # Set fixed axis limits based on workspace
        self.ax_traj.set_xlim(0, 0.5)
        self.ax_traj.set_ylim(0, 0.5)
        self.ax_traj.set_aspect('equal')
        
        # Set initial torque plot limits
        self.ax_torque.set_xlim(0, 100)
        self.ax_torque.set_ylim(-50, 50)
        
        # Set initial joint angle plot limits
        self.ax_angles.set_xlim(0, 100)
        self.ax_angles.set_ylim(-3.14, 3.14)  # ±π range for joint angles
        
        # Add PD Control Gain Sliders
        tk.Label(self.control_frame, text="PD Control Gains", font=('Arial', 12, 'bold')).pack(pady=5)
        
        # Kp slider
        tk.Label(self.control_frame, text="Position Gain (Kp)").pack()
        self.kp_slider = tk.Scale(self.control_frame, from_=0.01, to=1.0, resolution=0.01, 
                                orient=tk.HORIZONTAL, length=200)
        self.kp_slider.set(0.10)  # Updated to match successful parameters
        self.kp_slider.pack(pady=5)
        
        # Kd slider
        tk.Label(self.control_frame, text="Velocity Gain (Kd)").pack()
        self.kd_slider = tk.Scale(self.control_frame, from_=0.1, to=2.0, resolution=0.1, 
                                orient=tk.HORIZONTAL, length=200)
        self.kd_slider.set(0.4)  # Updated to match successful parameters
        self.kd_slider.pack(pady=5)
        
        # Add Trajectory Parameters
        tk.Label(self.control_frame, text="Trajectory Parameters", font=('Arial', 12, 'bold')).pack(pady=(20,5))
        
        # Center X slider
        tk.Label(self.control_frame, text="Center X Position (m)").pack()
        self.center_x_slider = tk.Scale(self.control_frame, from_=0.1, to=0.4, resolution=0.01,
                                      orient=tk.HORIZONTAL, length=200)
        self.center_x_slider.set(0.25)  # Updated to match successful parameters
        self.center_x_slider.pack(pady=5)
        
        # Center Z slider
        tk.Label(self.control_frame, text="Center Z Position (m)").pack()
        self.center_z_slider = tk.Scale(self.control_frame, from_=0.1, to=0.4, resolution=0.01,
                                      orient=tk.HORIZONTAL, length=200)
        self.center_z_slider.set(0.10)  # Updated to match successful parameters
        self.center_z_slider.pack(pady=5)
        
        # Radius slider
        tk.Label(self.control_frame, text="Radius (m)").pack()
        self.radius_slider = tk.Scale(self.control_frame, from_=0.05, to=0.15, resolution=0.01,
                                    orient=tk.HORIZONTAL, length=200)
        self.radius_slider.set(0.10)  # Updated to match successful parameters
        self.radius_slider.pack(pady=5)
        
        # Add current values display
        self.values_frame = tk.Frame(self.control_frame)
        self.values_frame.pack(pady=10)
        
        self.kp_label = tk.Label(self.values_frame, text="Current Kp: 0.50")
        self.kp_label.pack()
        self.kd_label = tk.Label(self.values_frame, text="Current Kd: 0.5")
        self.kd_label.pack()
        
        # Add Save Data Button and Status
        tk.Label(self.control_frame, text="Data Logging", font=('Arial', 12, 'bold')).pack(pady=(20,5))
        self.semi_circle_label = tk.Label(self.control_frame, text="Semi-Circle #: 0")
        self.semi_circle_label.pack()
        self.save_button = tk.Button(self.control_frame, text="Save Current Semi-Circle Data", 
                                   command=self._save_button_clicked)
        self.save_button.pack(pady=5)
        self.save_status = tk.Label(self.control_frame, text="")
        self.save_status.pack()
        
        # Data caching
        self.current_semi_circle_data = []
        self.current_semi_circle_number = 0
        self.save_requested = False
        
        # Bind slider updates
        self.kp_slider.configure(command=self._update_labels)
        self.kd_slider.configure(command=self._update_labels)
    
    def _update_labels(self, _=None):
        """Update the display labels and print current values"""
        kp = self.kp_slider.get()
        kd = self.kd_slider.get()
        self.kp_label.configure(text=f"Current Kp: {kp:.2f}")
        self.kd_label.configure(text=f"Current Kd: {kd:.1f}")
        
        # Print current parameters to terminal
        center_x = self.center_x_slider.get()
        center_z = self.center_z_slider.get()
        radius = self.radius_slider.get()
        print(f"\nParameters updated:")
        print(f"PD Gains: Kp={kp:.2f}, Kd={kd:.1f}")
        print(f"Trajectory: Center=({center_x:.2f}, {center_z:.2f}), Radius={radius:.2f}")
    
    def _save_button_clicked(self):
        self.save_requested = True
        self.save_status.configure(text="Save requested...", fg="blue")
    
    def get_gains(self):
        return self.kp_slider.get(), self.kd_slider.get()
    
    def get_trajectory_params(self):
        return (
            self.center_x_slider.get(),
            self.center_z_slider.get(),
            self.radius_slider.get()
        )
    
    def cache_data_point(self, data_point):
        self.current_semi_circle_data.append(data_point)
    
    def clear_cache(self):
        self.current_semi_circle_data = []
    
    def save_cached_data(self, semi_circle_number):
        if not self.current_semi_circle_data:
            return False
            
        # Get current parameters for filename
        kp, kd = self.get_gains()
        center_x, center_z, radius = self.get_trajectory_params()
        
        # Create filename with parameters
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"semi_circle_trajectory_kp{kp:.2f}_kd{kd:.1f}_r{radius:.2f}_x{center_x:.2f}_z{center_z:.2f}_semicircle{semi_circle_number}_{timestamp_str}.csv"
        
        if not os.path.exists('data'):
            os.makedirs('data')
        filepath = os.path.join('data', filename)

        header = [
            'timestamp', 'target_x', 'target_z', 'actual_x', 'actual_z',
            'joint1_angle', 'joint2_angle', 'joint3_angle',
            'joint1_torque', 'joint2_torque', 'joint3_torque',
            'kp', 'kd', 'radius', 'center_x', 'center_z'
        ]
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.current_semi_circle_data)
        
        self.save_status.configure(text=f"Saved semi-circle #{semi_circle_number}", fg="green")
        return True
        
    def update(self, target_x, target_z, actual_x, actual_z, joint_torques, joint_angles):
        """Update the visualization"""
        if self.window_closed:
            return False
            
        try:
            # Update trajectory data
            self.target_x.append(target_x)
            self.target_z.append(target_z)
            self.actual_x.append(actual_x)
            self.actual_z.append(actual_z)
            
            # Update torque and angle data
            self.step_counter += 1
            self.time_steps.append(self.step_counter)
            
            # Update torques
            self.torques_joint1.append(joint_torques[0])
            self.torques_joint2.append(joint_torques[1])
            self.torques_joint3.append(joint_torques[2])
            
            # Update angles
            self.angles_joint1.append(joint_angles[0])
            self.angles_joint2.append(joint_angles[1])
            self.angles_joint3.append(joint_angles[2])
            
            # Keep only last 100 points for time series plots
            if len(self.time_steps) > 100:
                self.time_steps = self.time_steps[-100:]
                self.torques_joint1 = self.torques_joint1[-100:]
                self.torques_joint2 = self.torques_joint2[-100:]
                self.torques_joint3 = self.torques_joint3[-100:]
                self.angles_joint1 = self.angles_joint1[-100:]
                self.angles_joint2 = self.angles_joint2[-100:]
                self.angles_joint3 = self.angles_joint3[-100:]
                
                # Update x-axis limits to show moving window
                self.ax_torque.set_xlim(self.step_counter - 100, self.step_counter)
                self.ax_angles.set_xlim(self.step_counter - 100, self.step_counter)
            
            # Update trajectory plot
            self.target_line.set_data(self.target_x, self.target_z)
            self.actual_line.set_data(self.actual_x, self.actual_z)
            
            # Update torque plots
            torque_data = [self.torques_joint1, self.torques_joint2, self.torques_joint3]
            for line, torques in zip(self.torque_lines, torque_data):
                line.set_data(self.time_steps, torques)
            
            # Update joint angle plots
            angle_data = [self.angles_joint1, self.angles_joint2, self.angles_joint3]
            for line, angles in zip(self.angle_lines, angle_data):
                line.set_data(self.time_steps, angles)
            
            # Adjust y-axis limits of torque plot if needed
            all_torques = self.torques_joint1 + self.torques_joint2 + self.torques_joint3
            if all_torques:
                min_torque = min(all_torques)
                max_torque = max(all_torques)
                margin = (max_torque - min_torque) * 0.1
                self.ax_torque.set_ylim(min_torque - margin, max_torque + margin)
            
            # Adjust y-axis limits of angle plot if needed
            all_angles = self.angles_joint1 + self.angles_joint2 + self.angles_joint3
            if all_angles:
                min_angle = min(all_angles)
                max_angle = max(all_angles)
                margin = (max_angle - min_angle) * 0.1
                self.ax_angles.set_ylim(min_angle - margin, max_angle + margin)
            
            # Redraw canvas
            self.canvas.draw()
            
            # Process Tkinter events
            self.root.update()
            return True
        except:
            self.window_closed = True
            return False
    
    def update_semi_circle_number(self, number):
        self.current_semi_circle_number = number
        self.semi_circle_label.configure(text=f"Semi-Circle #: {number}")
        
    def clear_data(self):
        # Clear trajectory data
        self.target_x = []
        self.target_z = []
        self.actual_x = []
        self.actual_z = []
        self.target_line.set_data([], [])
        self.actual_line.set_data([], [])
        
        # Clear torque data
        self.time_steps = []
        self.torques_joint1 = []
        self.torques_joint2 = []
        self.torques_joint3 = []
        
        # Clear angle data
        self.angles_joint1 = []
        self.angles_joint2 = []
        self.angles_joint3 = []
        self.step_counter = 0
        
        # Clear all lines
        for line in self.torque_lines:
            line.set_data([], [])
        for line in self.angle_lines:
            line.set_data([], [])
        
        self.canvas.draw()
        
    def close(self):
        """Safely close the visualization window"""
        try:
            self.window_closed = True
            if self.root:
                self.root.quit()
                self.root.destroy()
        except:
            pass  # Ignore any errors during cleanup

    def is_closed(self):
        """Check if the window has been closed"""
        return self.window_closed

def setup_simulation(urdf_path):
    """
    Sets up the PyBullet simulation environment.
    """
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    
    p.loadURDF("plane.urdf")
    
    startPos = [0, 0, 0]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    
    robot_id = p.loadURDF(urdf_path, startPos, startOrientation, useFixedBase=True)
    
    p.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=0,
        cameraPitch=0,
        cameraTargetPosition=[0.25, 0, 0.25]
    )
    
    return robot_id

def get_joint_info(robot_id):
    """
    Retrieves information about the robot's joints.
    """
    actuated_joint_indices = []
    joint_names = []
    
    num_joints = p.getNumJoints(robot_id)
    end_effector_link_index = -1
    link3_to_link4_index = -1
    link4_to_gripper_index = -1

    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_name = info[1].decode('utf-8')
        
        if joint_name in ["link_1_to_link_2", "link_2_to_link_3", "link_3_to_link_4"]:
            actuated_joint_indices.append(i)
            joint_names.append(joint_name)
            p.enableJointForceTorqueSensor(robot_id, i, 1)

        if joint_name == "link_4_to_gripper":
            end_effector_link_index = i
            link4_to_gripper_index = i
        elif joint_name == "link_3_to_link_4":
            link3_to_link4_index = i

    print("Actuated Joints Found:")
    for i, name in zip(actuated_joint_indices, joint_names):
        print(f"- {name} (Index: {i})")
    print(f"End-Effector Link Index: {end_effector_link_index}\n")

    return actuated_joint_indices, end_effector_link_index, link3_to_link4_index, link4_to_gripper_index

def generate_semi_circular_trajectory(cx, cy, radius, num_points):
    """
    Generates a list of (x, y) points for a bidirectional semi-circular trajectory.
    The trajectory follows the same semi-circle path forward and backward.
    """
    trajectory = []
    
    # Use half points for each direction
    points_per_direction = num_points // 2
    
    # Generate forward semi-circle points (π to 0 radians for clockwise motion)
    for i in range(points_per_direction):
        angle = math.pi * (1 - i / (points_per_direction - 1))
        x = cx + radius * math.cos(angle)
        z = cy + radius * math.sin(angle)
        trajectory.append((x, z))
    
    # Generate return semi-circle points (0 to π radians for counter-clockwise return)
    for i in range(points_per_direction):
        angle = math.pi * i / (points_per_direction - 1)
        x = cx + radius * math.cos(angle)
        z = cy + radius * math.sin(angle)
        trajectory.append((x, z))
    
    return trajectory

def custom_inverse_kinematics(target_x, target_z, link_lengths):
    """
    Custom inverse kinematics solver based on the simplified mathematical model.
    """
    l1 = link_lengths['l1']
    l2 = link_lengths['l2']
    l3 = link_lengths['l3']
    l4 = link_lengths['l4']

    def forward_kinematics_for_ik(theta):
        x = l2 * np.sin(theta[0]) + l3 * np.sin(theta[0] + theta[1]) + l4 * np.sin(theta[0] + theta[1] + theta[2])
        z = l1 + l2 * np.cos(theta[0]) + l3 * np.cos(theta[0] + theta[1]) + l4 * np.cos(theta[0] + theta[1] + theta[2])
        return x, z

    def objective(theta):
        end_x, end_z = forward_kinematics_for_ik(theta)
        return np.sqrt((end_x - target_x)**2 + (end_z - target_z)**2)
    
    theta0 = [0, 0, 0]
    bounds = [(-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi)]
    
    result = minimize(objective, theta0, method='SLSQP', bounds=bounds)
    
    if result.success and result.fun < 1e-3:
        return result.x
    else:
        return None

def main():
    """
    Main function to run the simulation and track the semi-circular trajectory.
    """
    urdf_file = "/home/san/Public/pybullet_4dof/urdf/robot.urdf"
    
    link_lengths = {'l1': 0.27, 'l2': 0.15, 'l3': 0.15, 'l4': 0.10}

    if not os.path.exists(urdf_file):
        print(f"Error: '{urdf_file}' not found. Please update the URDF file path.")
        return

    robot_id = setup_simulation(urdf_file)
    actuated_joint_indices, ee_link_index, link3_to_link4_index, link4_to_gripper_index = get_joint_info(robot_id)

    visualizer = RealTimeVisualizer()
    
    center_x, center_z, radius = visualizer.get_trajectory_params()
    num_trajectory_points = 250  # Points will be split between forward and return paths
    
    trajectory_points = generate_semi_circular_trajectory(center_x, center_z, radius, num_trajectory_points)
    
    # Create visual marker for gripper effect (ball being manipulated)
    marker_radius = 0.0165  # Size of the ball being manipulated
    marker_color = [0, 1, 0, 1]  # Green color [R, G, B, Alpha]
    
    # Create visual shape for marker
    marker_visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=marker_radius,
        rgbaColor=marker_color
    )
    
    # Create the marker body (with no collision)
    gripper_marker = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=marker_visual,
        basePosition=[0, 0, 0]
    )
    
    # Create box containers for start and end points
    # Make boxes larger and rectangular (like storage containers)
    box_size = [0.06, 0.08, 0.08]  # Width, Length, Height (increased height)
    box_base_thickness = 0.01  # Thickness of the box base and walls
    
    # Colors for the boxes and their contents
    start_box_color = [0.8, 0.2, 0.2, 0.9]  # Red box
    end_box_color = [0.2, 0.2, 0.8, 0.9]    # Blue box
    
    # Create the container boxes (main body)
    start_box_visual = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[box_size[0]/2, box_size[1]/2, box_base_thickness/2],  # Base
        rgbaColor=start_box_color
    )
    
    end_box_visual = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[box_size[0]/2, box_size[1]/2, box_base_thickness/2],  # Base
        rgbaColor=end_box_color
    )
    
    # Create walls for the boxes
    wall_height = box_size[2]
    wall_thickness = box_base_thickness
    
    # Visual shapes for walls (same color as the base)
    wall_shapes = {
        'start': [],
        'end': []
    }
    
    # Create walls for both boxes
    for box_type in ['start', 'end']:
        color = start_box_color if box_type == 'start' else end_box_color
        
        # Front wall
        wall_shapes[box_type].append(
            p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[box_size[0]/2, wall_thickness/2, wall_height/2],
                rgbaColor=color
            )
        )
        
        # Back wall
        wall_shapes[box_type].append(
            p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[box_size[0]/2, wall_thickness/2, wall_height/2],
                rgbaColor=color
            )
        )
        
        # Left wall
        wall_shapes[box_type].append(
            p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[wall_thickness/2, box_size[1]/2, wall_height/2],
                rgbaColor=color
            )
        )
        
        # Right wall
        wall_shapes[box_type].append(
            p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[wall_thickness/2, box_size[1]/2, wall_height/2],
                rgbaColor=color
            )
        )
    
    # Calculate start and end positions for the boxes
    start_point = trajectory_points[0]
    end_point = trajectory_points[num_trajectory_points // 2 - 1]
    
    # Add offset to box positions (move red box left, blue box right)
    box_x_offset = 0.0175  # 1.75cm offset
    start_point = (start_point[0] - box_x_offset, start_point[1])  # Move red box left
    end_point = (end_point[0] + box_x_offset, end_point[1])      # Move blue box right
    
    # Create the box containers with their walls
    def create_container(base_pos, base_visual, wall_shapes):
        # Create base - position adjusted to sit on ground
        base = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=base_visual,
            basePosition=[base_pos[0], 0, box_base_thickness/2]  # Adjusted to sit on ground
        )
        
        # Wall positions relative to base
        wall_positions = [
            [0, box_size[1]/2, wall_height/2],    # Front wall
            [0, -box_size[1]/2, wall_height/2],   # Back wall
            [-box_size[0]/2, 0, wall_height/2],   # Left wall
            [box_size[0]/2, 0, wall_height/2]     # Right wall
        ]
        
        # Create walls
        walls = []
        for wall_shape, rel_pos in zip(wall_shapes, wall_positions):
            wall = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=wall_shape,
                basePosition=[
                    base_pos[0] + rel_pos[0],
                    0 + rel_pos[1],
                    rel_pos[2]  # Adjusted to be relative to ground
                ]
            )
            walls.append(wall)
        
        return [base] + walls
    
    # Create both containers with their walls
    start_container = create_container(
        [start_point[0], start_point[1]], 
        start_box_visual, 
        wall_shapes['start']
    )
    
    end_container = create_container(
        [end_point[0], end_point[1]], 
        end_box_visual, 
        wall_shapes['end']
    )
    
    # Draw initial trajectory in PyBullet workspace
    for i in range(len(trajectory_points) - 1):
        p1 = [trajectory_points[i][0], 0, trajectory_points[i][1]]
        p2 = [trajectory_points[i + 1][0], 0, trajectory_points[i + 1][1]]
        p.addUserDebugLine(p1, p2, lineColorRGB=[1, 0, 0], lineWidth=2)
    
    prev_params = (center_x, center_z, radius)
    semi_circle_count = 0
    
    print(f"Starting simulation. Use the sliders to adjust PD gains and trajectory parameters in real-time.")
    print("Click 'Save Current Semi-Circle Data' button to save data for the current semi-circle.")
    print("Press Ctrl+C or close the window to stop the simulation.")

    try:
        while not visualizer.is_closed():
            current_params = visualizer.get_trajectory_params()
            if current_params != prev_params:
                center_x, center_z, radius = current_params
                trajectory_points = generate_semi_circular_trajectory(center_x, center_z, radius, num_trajectory_points)
                prev_params = current_params
                
                # Update container positions when trajectory parameters change
                start_point = trajectory_points[0]
                end_point = trajectory_points[num_trajectory_points // 2 - 1]
                
                # Apply offset to box positions
                start_point = (start_point[0] - box_x_offset, start_point[1])  # Move red box left
                end_point = (end_point[0] + box_x_offset, end_point[1])      # Move blue box right
                
                # Update start container position
                for i, body_id in enumerate(start_container):
                    if i == 0:  # Base
                        p.resetBasePositionAndOrientation(
                            body_id,
                            [start_point[0], 0, box_base_thickness/2],  # Adjusted to sit on ground
                            [0, 0, 0, 1]
                        )
                    else:  # Walls
                        wall_idx = i - 1
                        wall_positions = [
                            [0, box_size[1]/2, wall_height/2],    # Front wall
                            [0, -box_size[1]/2, wall_height/2],   # Back wall
                            [-box_size[0]/2, 0, wall_height/2],   # Left wall
                            [box_size[0]/2, 0, wall_height/2]     # Right wall
                        ]
                        rel_pos = wall_positions[wall_idx]
                        p.resetBasePositionAndOrientation(
                            body_id,
                            [
                                start_point[0] + rel_pos[0],
                                0 + rel_pos[1],
                                rel_pos[2]  # Adjusted to be relative to ground
                            ],
                            [0, 0, 0, 1]
                        )
                
                # Update end container position
                for i, body_id in enumerate(end_container):
                    if i == 0:  # Base
                        p.resetBasePositionAndOrientation(
                            body_id,
                            [end_point[0], 0, box_base_thickness/2],  # Adjusted to sit on ground
                            [0, 0, 0, 1]
                        )
                    else:  # Walls
                        wall_idx = i - 1
                        wall_positions = [
                            [0, box_size[1]/2, wall_height/2],    # Front wall
                            [0, -box_size[1]/2, wall_height/2],   # Back wall
                            [-box_size[0]/2, 0, wall_height/2],   # Left wall
                            [box_size[0]/2, 0, wall_height/2]     # Right wall
                        ]
                        rel_pos = wall_positions[wall_idx]
                        p.resetBasePositionAndOrientation(
                            body_id,
                            [
                                end_point[0] + rel_pos[0],
                                0 + rel_pos[1],
                                rel_pos[2]  # Adjusted to be relative to ground
                            ],
                            [0, 0, 0, 1]
                        )
                
                p.removeAllUserDebugItems()
                
                for i in range(len(trajectory_points) - 1):
                    p1 = [trajectory_points[i][0], 0, trajectory_points[i][1]]
                    p2 = [trajectory_points[i + 1][0], 0, trajectory_points[i + 1][1]]
                    p.addUserDebugLine(p1, p2, lineColorRGB=[1, 0, 0], lineWidth=2)
            
            visualizer.clear_data()
            visualizer.clear_cache()
            
            for i, (target_x, target_z) in enumerate(trajectory_points):
                if visualizer.is_closed():
                    break
                    
                kp, kd = visualizer.get_gains()
                
                joint_poses = custom_inverse_kinematics(target_x, target_z, link_lengths)

                if joint_poses is not None:
                    for j, joint_index in enumerate(actuated_joint_indices):
                        p.setJointMotorControl2(
                            bodyUniqueId=robot_id,
                            jointIndex=joint_index,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_poses[j],
                            force=50,
                            positionGain=kp,
                            velocityGain=kd
                        )

                p.stepSimulation()
                time.sleep(1./240.)

                # Only show marker during the semi-circular part
                if i < num_trajectory_points // 2: # Check if it's in the forward semi-circle
                    # Get positions of the two joints
                    link3_state = p.getLinkState(robot_id, link3_to_link4_index)
                    link4_state = p.getLinkState(robot_id, link4_to_gripper_index)
                    
                    # Get the direction vector between the joints
                    dir_x = link4_state[0][0] - link3_state[0][0]
                    dir_y = link4_state[0][1] - link3_state[0][1]
                    dir_z = link4_state[0][2] - link3_state[0][2]
                    
                    # Normalize the direction vector
                    length = np.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
                    if length > 0:
                        dir_x /= length
                        dir_y /= length
                        dir_z /= length
                        
                        # Extend from the gripper joint by 0.03m (reduced from 0.08m)
                        offset = 0.03
                        marker_pos = [
                            link4_state[0][0] + dir_x * offset,
                            link4_state[0][1] + dir_y * offset,
                            link4_state[0][2] + dir_z * offset
                        ]
                        
                        # Update marker position with gripper orientation
                        p.resetBasePositionAndOrientation(gripper_marker, marker_pos, link4_state[1])
                else:
                    # Hide marker during straight line motion
                    hide_pos = [1000, 1000, 1000]
                    p.resetBasePositionAndOrientation(gripper_marker, hide_pos, [0, 0, 0, 1])
                
                # Get end effector state for visualization and data logging
                link_state = p.getLinkState(robot_id, ee_link_index)
                actual_pos = link_state[0]
                
                joint_torques = []
                for joint_index in actuated_joint_indices:
                    state = p.getJointState(robot_id, joint_index)
                    joint_torques.append(state[3])
                
                joint_angles = []
                for joint_index in actuated_joint_indices:
                    state = p.getJointState(robot_id, joint_index)
                    joint_angles.append(state[0])
                
                if not visualizer.update(target_x, target_z, actual_pos[0], actual_pos[2], joint_torques, joint_angles):
                    break

                timestamp = time.time()
                
                joint_angles_for_data = []
                joint_torques_for_data = []
                for joint_index in actuated_joint_indices:
                    state = p.getJointState(robot_id, joint_index)
                    joint_angles_for_data.append(state[0])
                    joint_torques_for_data.append(state[3])
                
                data_point = [
                    timestamp,
                    target_x, target_z,
                    actual_pos[0], actual_pos[2],
                    *joint_angles_for_data,
                    *joint_torques_for_data,
                    kp, kd,
                    radius,
                    center_x,
                    center_z
                ]
                visualizer.cache_data_point(data_point)

                if i == num_trajectory_points - 1:
                    semi_circle_count += 1
                    visualizer.update_semi_circle_number(semi_circle_count)
                    print(f"Semi-circle {semi_circle_count} completed. Gains: Kp={kp:.2f}, Kd={kd:.1f}, "
                          f"Center: ({center_x:.2f}, {center_z:.2f}), Radius: {radius:.2f}")
                    
                    if visualizer.save_requested:
                        visualizer.save_cached_data(semi_circle_count)
                        visualizer.save_requested = False

    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")

    finally:
        visualizer.close()
        p.disconnect()

if __name__ == "__main__":
    main() 
#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import numpy as np

def plot_robot_data(csv_file):
    # Read the CSV file
    data = pd.read_csv(csv_file)
    
    # Create a figure with subplots
    plt.style.use('seaborn')
    fig = plt.figure(figsize=(15, 10))
    
    # 1. End Effector Position Plot
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(data['ee_pos_x'], data['ee_pos_y'], data['ee_pos_z'])
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('End Effector Trajectory')
    
    # Add start and end points
    ax1.scatter(data['ee_pos_x'].iloc[0], data['ee_pos_y'].iloc[0], data['ee_pos_z'].iloc[0], 
                color='green', marker='o', s=100, label='Start')
    ax1.scatter(data['ee_pos_x'].iloc[-1], data['ee_pos_y'].iloc[-1], data['ee_pos_z'].iloc[-1], 
                color='red', marker='o', s=100, label='End')
    ax1.legend()
    
    # 2. End Effector Position vs Time
    ax2 = fig.add_subplot(222)
    ax2.plot(data['timestamp'], data['ee_pos_x'], label='X')
    ax2.plot(data['timestamp'], data['ee_pos_y'], label='Y')
    ax2.plot(data['timestamp'], data['ee_pos_z'], label='Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('End Effector Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # 3. End Effector Orientation
    ax3 = fig.add_subplot(223)
    ax3.plot(data['timestamp'], np.rad2deg(data['ee_rot_x']), label='Roll')
    ax3.plot(data['timestamp'], np.rad2deg(data['ee_rot_y']), label='Pitch')
    ax3.plot(data['timestamp'], np.rad2deg(data['ee_rot_z']), label='Yaw')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (degrees)')
    ax3.set_title('End Effector Orientation vs Time')
    ax3.legend()
    ax3.grid(True)
    
    # 4. Joint Torques
    ax4 = fig.add_subplot(224)
    torque_columns = [col for col in data.columns if 'torque' in col]
    for col in torque_columns:
        joint_name = col.replace('_torque', '')
        ax4.plot(data['timestamp'], data[col], label=joint_name)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Torque (Nm)')
    ax4.set_title('Joint Torques vs Time')
    ax4.legend()
    ax4.grid(True)
    
    # Adjust layout and display
    plt.tight_layout()
    
    # Save plot with same name as CSV but with .png extension
    plot_file = os.path.splitext(csv_file)[0] + '_plot.png'
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved as: {plot_file}")
    
    # Show plot
    plt.show()

def main():
    # Find the most recent CSV file in the data directory
    data_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data")
    csv_files = glob.glob(os.path.join(data_dir, 'robot_data_*.csv'))
    if not csv_files:
        print("No robot data files found!")
        return
    
    # Sort by modification time and get the most recent
    latest_file = max(csv_files, key=os.path.getmtime)
    print(f"Plotting data from: {latest_file}")
    
    # Plot the data
    plot_robot_data(latest_file)

if __name__ == "__main__":
    main() 
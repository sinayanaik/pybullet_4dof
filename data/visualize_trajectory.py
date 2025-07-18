#!/usr/bin/env python3

import sys
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np

def plot_trajectory_data(csv_file):
    """
    Visualize trajectory data from CSV file with multiple subplots
    showing position tracking, joint angles, torques, and PD gains.
    """
    # Read CSV data
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        sys.exit(1)

    # Create figure with subplots
    plt.style.use('seaborn')
    fig = plt.figure(figsize=(15, 10))
    gs = GridSpec(2, 3, figure=fig)
    
    # 1. Trajectory Plot (X-Z plane)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(df['target_x'], df['target_z'], 'r-', label='Target', linewidth=2)
    ax1.plot(df['actual_x'], df['actual_z'], 'b--', label='Actual', linewidth=2)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Z Position (m)')
    ax1.set_title('End-Effector Trajectory')
    ax1.grid(True)
    ax1.legend()

    # 2. Position Error Plot
    ax2 = fig.add_subplot(gs[0, 1])
    error_x = df['actual_x'] - df['target_x']
    error_z = df['actual_z'] - df['target_z']
    time = np.arange(len(df)) / len(df)  # Normalized time
    ax2.plot(time, error_x * 1000, 'r-', label='X Error', linewidth=1.5)
    ax2.plot(time, error_z * 1000, 'b-', label='Z Error', linewidth=1.5)
    ax2.set_xlabel('Normalized Time')
    ax2.set_ylabel('Position Error (mm)')
    ax2.set_title('Tracking Error')
    ax2.grid(True)
    ax2.legend()

    # 3. Joint Angles
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(time, df['joint1_angle'], 'r-', label='Joint 1', linewidth=1.5)
    ax3.plot(time, df['joint2_angle'], 'g-', label='Joint 2', linewidth=1.5)
    ax3.plot(time, df['joint3_angle'], 'b-', label='Joint 3', linewidth=1.5)
    ax3.set_xlabel('Normalized Time')
    ax3.set_ylabel('Joint Angle (rad)')
    ax3.set_title('Joint Angles')
    ax3.grid(True)
    ax3.legend()

    # 4. Joint Torques
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(time, df['joint1_torque'], 'r-', label='Joint 1', linewidth=1.5)
    ax4.plot(time, df['joint2_torque'], 'g-', label='Joint 2', linewidth=1.5)
    ax4.plot(time, df['joint3_torque'], 'b-', label='Joint 3', linewidth=1.5)
    ax4.set_xlabel('Normalized Time')
    ax4.set_ylabel('Torque (N⋅m)')
    ax4.set_title('Joint Torques')
    ax4.grid(True)
    ax4.legend()

    # 5. PD Gains
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(time, df['kp'], 'r-', label='Kp', linewidth=2)
    ax5.plot(time, df['kd'], 'b-', label='Kd', linewidth=2)
    ax5.set_xlabel('Normalized Time')
    ax5.set_ylabel('Gain Value')
    ax5.set_title('PD Control Gains')
    ax5.grid(True)
    ax5.legend()

    # 6. Trajectory Parameters
    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(time, df['radius'], 'r-', label='Radius', linewidth=2)
    ax6.plot(time, df['center_x'], 'g-', label='Center X', linewidth=2)
    ax6.plot(time, df['center_z'], 'b-', label='Center Z', linewidth=2)
    ax6.set_xlabel('Normalized Time')
    ax6.set_ylabel('Value (m)')
    ax6.set_title('Trajectory Parameters')
    ax6.grid(True)
    ax6.legend()

    # Adjust layout and display
    plt.tight_layout()
    plt.show()

    # Print some statistics
    print("\nTrajectory Statistics:")
    print("-" * 50)
    
    # Calculate RMS tracking error
    rms_error_x = np.sqrt(np.mean(error_x**2)) * 1000  # Convert to mm
    rms_error_z = np.sqrt(np.mean(error_z**2)) * 1000  # Convert to mm
    print(f"RMS Tracking Error:")
    print(f"  X: {rms_error_x:.2f} mm")
    print(f"  Z: {rms_error_z:.2f} mm")
    
    # Calculate max torques
    max_torques = [
        df['joint1_torque'].abs().max(),
        df['joint2_torque'].abs().max(),
        df['joint3_torque'].abs().max()
    ]
    print(f"\nMaximum Joint Torques:")
    for i, torque in enumerate(max_torques, 1):
        print(f"  Joint {i}: {torque:.2f} N⋅m")
    
    # Show PD gain ranges
    print(f"\nPD Gain Ranges:")
    print(f"  Kp: {df['kp'].min():.2f} to {df['kp'].max():.2f}")
    print(f"  Kd: {df['kd'].min():.2f} to {df['kd'].max():.2f}")

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 visualize_trajectory.py <trajectory_data.csv>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    plot_trajectory_data(csv_file)

if __name__ == "__main__":
    main() 
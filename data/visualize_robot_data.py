#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from matplotlib.gridspec import GridSpec

def load_data(file_path):
    """Load data from CSV file"""
    try:
        data = pd.read_csv(file_path)
        print(f"Loaded data from {file_path}")
        print("\nData columns:", data.columns.tolist())
        print(f"\nTotal data points: {len(data)}")
        return data
    except Exception as e:
        print(f"Error loading data: {str(e)}")
        return None

def plot_trajectory(data, save_path=None):
    """Create visualization plots for robot trajectory data"""
    # Create figure with GridSpec for better layout
    fig = plt.figure(figsize=(15, 10))
    gs = GridSpec(2, 2, figure=fig)
    
    # Trajectory plot (top left)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(data['Actual_X'], data['Actual_Z'], 'b-', label='Actual', linewidth=2)
    ax1.plot(data['Desired_X'], data['Desired_Z'], 'r--', label='Desired', linewidth=2)
    ax1.set_title('End Effector Trajectory')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Z Position (m)')
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')
    
    # Position error plot (top right)
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(data['Time'], data['Error_X'], 'r-', label='X Error', linewidth=2)
    ax2.plot(data['Time'], data['Error_Z'], 'b-', label='Z Error', linewidth=2)
    ax2.set_title('Position Error')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m)')
    ax2.grid(True)
    ax2.legend()
    
    # Joint angles plot (bottom left)
    ax3 = fig.add_subplot(gs[1, 0])
    for joint in ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']:
        if f'Angle_{joint}' in data.columns:
            ax3.plot(data['Time'], data[f'Angle_{joint}'], label=f'Joint {joint}', linewidth=2)
    ax3.set_title('Joint Angles')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (rad)')
    ax3.grid(True)
    ax3.legend()
    
    # Joint torques plot (bottom right)
    ax4 = fig.add_subplot(gs[1, 1])
    for joint in ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']:
        if f'Torque_{joint}' in data.columns:
            ax4.plot(data['Time'], data[f'Torque_{joint}'], label=f'Joint {joint}', linewidth=2)
    ax4.set_title('Joint Torques')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Torque (N⋅m)')
    ax4.grid(True)
    ax4.legend()
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path)
        print(f"Plot saved to {save_path}")
    else:
        plt.show()

def print_statistics(data):
    """Print statistical analysis of the data"""
    print("\nTrajectory Statistics:")
    print("-" * 50)
    
    # Position error statistics
    print("\nPosition Error Statistics (meters):")
    print(f"X Error - Mean: {data['Error_X'].mean():.6f}, Max: {data['Error_X'].max():.6f}, Min: {data['Error_X'].min():.6f}")
    print(f"Z Error - Mean: {data['Error_Z'].mean():.6f}, Max: {data['Error_Z'].max():.6f}, Min: {data['Error_Z'].min():.6f}")
    
    # RMS error
    rms_x = np.sqrt(np.mean(np.square(data['Error_X'])))
    rms_z = np.sqrt(np.mean(np.square(data['Error_Z'])))
    print(f"\nRMS Error:")
    print(f"X RMS Error: {rms_x:.6f} m")
    print(f"Z RMS Error: {rms_z:.6f} m")
    
    # Joint statistics
    print("\nJoint Statistics:")
    for joint in ['link_1_to_link_2', 'link_2_to_link_3', 'link_3_to_link_4']:
        if f'Angle_{joint}' in data.columns:
            angles = data[f'Angle_{joint}']
            torques = data[f'Torque_{joint}']
            print(f"\n{joint}:")
            print(f"  Angle (rad) - Mean: {angles.mean():.3f}, Max: {angles.max():.3f}, Min: {angles.min():.3f}")
            print(f"  Torque (N⋅m) - Mean: {torques.mean():.3f}, Max: {torques.max():.3f}, Min: {torques.min():.3f}")

def main():
    parser = argparse.ArgumentParser(description='Visualize robot trajectory data from CSV file')
    parser.add_argument('data_file', help='Path to the CSV data file')
    parser.add_argument('--save', help='Save plot to file instead of displaying', default=None)
    parser.add_argument('--stats-only', action='store_true', help='Only print statistics, no plotting')
    args = parser.parse_args()
    
    # Load data
    data = load_data(args.data_file)
    if data is None:
        return
    
    # Print statistics
    print_statistics(data)
    
    # Plot if not stats-only
    if not args.stats_only:
        if args.save:
            # If save path is provided, save the plot
            plot_trajectory(data, args.save)
        else:
            # Otherwise show interactive plot
            plot_trajectory(data)

if __name__ == "__main__":
    main() 
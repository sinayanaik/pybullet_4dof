import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import torch
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from .pinn_model import PINN
import argparse

def load_model(model_path):
    """Load the trained model and scalers."""
    # Load the model checkpoint
    checkpoint = torch.load(model_path)
    
    model = PINN()
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()
    
    feature_scaler = checkpoint['feature_scaler']
    target_scaler = checkpoint['target_scaler']
    
    return model, feature_scaler, target_scaler

def compute_derivatives(data):
    """
    Compute velocities and accelerations from position data
    Args:
        data: DataFrame containing joint angles and timestamps
    Returns:
        features: Array containing positions, velocities, and accelerations
    """
    # Get joint angles
    angles = data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values
    
    # Compute time steps from timestamp
    timestamps = data['timestamp'].values
    dt = np.diff(timestamps)
    dt = np.append(dt, dt[-1])  # Repeat last dt for the final point
    
    # Compute velocities using finite differences
    velocities = np.zeros_like(angles)
    velocities[1:] = (angles[1:] - angles[:-1]) / dt[:-1, np.newaxis]
    velocities[0] = velocities[1]  # Use second point's velocity for first point
    
    # Compute accelerations
    accelerations = np.zeros_like(angles)
    accelerations[1:] = (velocities[1:] - velocities[:-1]) / dt[:-1, np.newaxis]
    accelerations[0] = accelerations[1]  # Use second point's acceleration for first point
    
    # Combine features
    features = np.concatenate([angles, velocities, accelerations], axis=1)
    return features

def predict_torques(model, feature_scaler, target_scaler, test_data):
    """
    Make predictions using the trained model
    Args:
        model: Trained PINN model
        feature_scaler: Scaler for input features
        target_scaler: Scaler for output targets
        test_data: DataFrame containing test data
    Returns:
        predictions: Predicted torques
        actual: Actual torques
    """
    # Compute features (positions, velocities, accelerations)
    features = compute_derivatives(test_data)
    
    # Extract actual torques
    actual = test_data[[
        'joint1_torque', 'joint2_torque', 'joint3_torque'
    ]].values
    
    # Scale features
    features_scaled = feature_scaler.transform(features)
    features_tensor = torch.FloatTensor(features_scaled)
    
    # Make predictions
    with torch.no_grad():
        predictions_scaled = model(features_tensor)
        predictions = target_scaler.inverse_transform(predictions_scaled)
    
    return predictions, actual

def plot_predictions(predictions, actual, save_path='prediction_results.png'):
    """
    Plot predicted vs actual torques for all joints in one figure
    Args:
        predictions: Predicted torques
        actual: Actual torques
        save_path: Path to save the plot
    """
    plt.figure(figsize=(15, 10))
    plt.suptitle('PINN Model Predictions vs Actual Torques', fontsize=16)
    
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    colors = ['#2C3E50', '#E74C3C', '#2ECC71']  # Colors for each joint
    
    for i in range(3):
        plt.subplot(3, 1, i+1)
        plt.plot(actual[:, i], label='Actual', color=colors[0], alpha=0.7, linewidth=2)
        plt.plot(predictions[:, i], label='PINN', color=colors[1], alpha=0.7, linewidth=2)
        
        # Calculate metrics
        mse = np.mean((predictions[:, i] - actual[:, i])**2)
        rmse = np.sqrt(mse)
        mae = np.mean(np.abs(predictions[:, i] - actual[:, i]))
        r2 = 1 - np.sum((actual[:, i] - predictions[:, i])**2) / np.sum((actual[:, i] - np.mean(actual[:, i]))**2)
        
        plt.title(f'{joint_names[i]} (MSE: {mse:.6f}, RMSE: {rmse:.6f}, MAE: {mae:.6f}, R²: {r2:.6f})')
        plt.xlabel('Time Step')
        plt.ylabel('Torque (Nm)')
        plt.grid(True, alpha=0.3)
        plt.legend()
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()

def main():
    parser = argparse.ArgumentParser(description='Make predictions using trained PINN model')
    parser.add_argument('--model_path', type=str, required=True,
                      help='Path to the trained model file')
    parser.add_argument('--test_file', type=str, required=True,
                      help='Path to the CSV file containing test data')
    
    args = parser.parse_args()
    
    # Load model
    model, feature_scaler, target_scaler = load_model(args.model_path)
    
    # Load test data
    test_data = pd.read_csv(args.test_file)
    
    # Make predictions
    predictions, actual = predict_torques(model, feature_scaler, target_scaler, test_data)
    
    # Plot results
    plot_predictions(predictions, actual)
    
    # Save predictions to CSV
    results = pd.DataFrame({
        'joint1_torque_actual': actual[:, 0],
        'joint2_torque_actual': actual[:, 1],
        'joint3_torque_actual': actual[:, 2],
        'joint1_torque_predicted': predictions[:, 0],
        'joint2_torque_predicted': predictions[:, 1],
        'joint3_torque_predicted': predictions[:, 2]
    })
    results.to_csv('predictions.csv', index=False)
    
    # Print metrics
    print("\nPrediction Metrics:")
    for i, joint in enumerate(['Joint 1', 'Joint 2', 'Joint 3']):
        mse = np.mean((predictions[:, i] - actual[:, i])**2)
        rmse = np.sqrt(mse)
        mae = np.mean(np.abs(predictions[:, i] - actual[:, i]))
        r2 = 1 - np.sum((actual[:, i] - predictions[:, i])**2) / np.sum((actual[:, i] - np.mean(actual[:, i]))**2)
        print(f"\n{joint}:")
        print(f"MSE:  {mse:.6f}")
        print(f"RMSE: {rmse:.6f}")
        print(f"MAE:  {mae:.6f}")
        print(f"R²:   {r2:.6f}")

if __name__ == "__main__":
    main() 
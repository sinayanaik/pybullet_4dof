import sys
import os

# Add parent directory to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from FNN.predict_torque import load_model as load_fnn
from RNN.predict_torque import load_model as load_rnn
from FNN_plus_physics_loss.predict import load_model as load_pinn
import torch

def load_and_prepare_data(data_path):
    """Load and prepare the test data."""
    data = pd.read_csv(data_path)
    
    # Take only the forward semi-circle (first half of data)
    n_points = len(data) // 2
    data = data.iloc[:n_points]
    
    X = data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values
    y = data[['joint1_torque', 'joint2_torque', 'joint3_torque']].values
    
    return data, X, y

def predict_fnn(X, model, X_scaler, y_scaler):
    """Make predictions using FNN model."""
    X_scaled = X_scaler.transform(X)
    X_tensor = torch.FloatTensor(X_scaled)
    
    with torch.no_grad():
        y_pred_scaled = model(X_tensor)
        y_pred = y_scaler.inverse_transform(y_pred_scaled.numpy())
    
    return y_pred

def predict_rnn(X, model, X_scaler, y_scaler, sequence_length):
    """Make predictions using RNN model."""
    predictions = []
    X_scaled = X_scaler.transform(X)
    
    # Create sequences
    for i in range(len(X_scaled) - sequence_length):
        sequence = X_scaled[i:i + sequence_length]
        X_tensor = torch.FloatTensor(sequence).unsqueeze(0)
        
        with torch.no_grad():
            y_pred_scaled, _ = model(X_tensor)
            y_pred = y_scaler.inverse_transform(y_pred_scaled.numpy())
            predictions.append(y_pred[0])
    
    return np.array(predictions)

def predict_pinn(data, model, feature_scaler, target_scaler):
    """Make predictions using PINN model."""
    # Compute time steps from timestamp
    timestamps = data['timestamp'].values
    dt = np.diff(timestamps)
    dt = np.append(dt, dt[-1])
    
    # Get angles and compute derivatives
    angles = data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values
    
    # Compute velocities
    velocities = np.zeros_like(angles)
    velocities[1:] = (angles[1:] - angles[:-1]) / dt[:-1, np.newaxis]
    velocities[0] = velocities[1]
    
    # Compute accelerations
    accelerations = np.zeros_like(angles)
    accelerations[1:] = (velocities[1:] - velocities[:-1]) / dt[:-1, np.newaxis]
    accelerations[0] = accelerations[1]
    
    # Combine features
    features = np.concatenate([angles, velocities, accelerations], axis=1)
    
    # Scale and predict
    features_scaled = feature_scaler.transform(features)
    features_tensor = torch.FloatTensor(features_scaled)
    
    with torch.no_grad():
        predictions_scaled = model(features_tensor)
        predictions = target_scaler.inverse_transform(predictions_scaled)
    
    return predictions

def calculate_metrics(y_true, y_pred):
    """Calculate metrics for each joint."""
    metrics = []
    for i in range(y_true.shape[1]):
        mse = np.mean((y_true[:, i] - y_pred[:, i])**2)
        rmse = np.sqrt(mse)
        mae = np.mean(np.abs(y_true[:, i] - y_pred[:, i]))
        r2 = 1 - np.sum((y_true[:, i] - y_pred[:, i])**2) / np.sum((y_true[:, i] - np.mean(y_true[:, i]))**2)
        metrics.append({'MSE': mse, 'RMSE': rmse, 'MAE': mae, 'R²': r2})
    return metrics

def plot_comparison(y_true, y_pred_fnn, y_pred_rnn, y_pred_pinn):
    """Plot all model predictions vs actual values."""
    plt.figure(figsize=(15, 12))
    plt.suptitle('Joint Torque Predictions Comparison', fontsize=16, y=1.02)
    
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    colors = {
        'Actual': '#1f77b4',  # Blue
        'FNN': '#ff7f0e',     # Orange
        'RNN': '#2ca02c',     # Green
        'PINN': '#9467bd'     # Purple
    }
    
    for i in range(3):
        plt.subplot(3, 1, i+1)
        
        # Calculate metrics for each model
        metrics_fnn = calculate_metrics(y_true, y_pred_fnn)[i]
        metrics_rnn = calculate_metrics(y_true, y_pred_rnn)[i]
        metrics_pinn = calculate_metrics(y_true, y_pred_pinn)[i]
        
        # Add metrics to title
        title = f'{joint_names[i]} Torque Comparison\n'
        title += f'MSE - FNN: {metrics_fnn["MSE"]:.6f}, RNN: {metrics_rnn["MSE"]:.6f}, PINN: {metrics_pinn["MSE"]:.6f}'
        
        plt.title(title, fontsize=10, pad=10)
        
        # Plot predictions in specific order to match original
        plt.plot(y_true[:, i], label='Actual', color=colors['Actual'], alpha=0.7, linewidth=1)
        plt.plot(y_pred_fnn[:, i], label='FNN', color=colors['FNN'], alpha=0.7, linewidth=1)
        plt.plot(y_pred_rnn[:, i], label='RNN', color=colors['RNN'], alpha=0.7, linewidth=1)
        plt.plot(y_pred_pinn[:, i], label='PINN', color=colors['PINN'], alpha=0.7, linewidth=1)
        
        plt.xlabel('Time Step')
        plt.ylabel('Torque (Nm)')
        plt.grid(True, alpha=0.3, linestyle='-')
        plt.legend(loc='upper right')
        
        # Set y-axis limits based on data for each joint
        data_range = [
            y_true[:, i].min(), y_true[:, i].max(),
            y_pred_fnn[:, i].min(), y_pred_fnn[:, i].max(),
            y_pred_rnn[:, i].min(), y_pred_rnn[:, i].max(),
            y_pred_pinn[:, i].min(), y_pred_pinn[:, i].max()
        ]
        y_min, y_max = min(data_range), max(data_range)
        margin = (y_max - y_min) * 0.1  # Add 10% margin
        plt.ylim(y_min - margin, y_max + margin)
    
    plt.tight_layout()
    plt.savefig('joint_predictions.png', dpi=300, bbox_inches='tight')
    plt.close()

def plot_loss_comparison(fnn_loss, rnn_loss):
    """Plot training loss comparison."""
    plt.figure(figsize=(10, 6))
    plt.plot(fnn_loss, label='FNN', color='#E74C3C', alpha=0.7, linewidth=2)
    plt.plot(rnn_loss, label='RNN', color='#2ECC71', alpha=0.7, linewidth=2)
    
    plt.title('Training Loss Comparison')
    plt.xlabel('Epoch')
    plt.ylabel('MSE Loss')
    plt.yscale('log')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig('loss_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()

def main():
    # Get the project root directory
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Load data
    data_path = os.path.join(project_root, 'data', 'semi_circle_trajectory_kp0.10_kd0.4_r0.10_x0.25_z0.10_semicircle5_20250722_125106.csv')
    data, X, y = load_and_prepare_data(data_path)
    
    # Load models
    fnn_model, fnn_X_scaler, fnn_y_scaler = load_fnn(os.path.join(project_root, 'FNN', 'torque_model.pth'))
    rnn_model, rnn_X_scaler, rnn_y_scaler, sequence_length = load_rnn(os.path.join(project_root, 'RNN', 'torque_model.pth'))
    pinn_model, pinn_feature_scaler, pinn_target_scaler = load_pinn(os.path.join(project_root, 'models', 'pinn_model_20250723_155053', 'best_model.pth'))
    
    # Make predictions
    y_pred_fnn = predict_fnn(X, fnn_model, fnn_X_scaler, fnn_y_scaler)
    y_pred_rnn = predict_rnn(X, rnn_model, rnn_X_scaler, rnn_y_scaler, sequence_length)
    y_pred_pinn = predict_pinn(data, pinn_model, pinn_feature_scaler, pinn_target_scaler)
    
    # Adjust predictions to match RNN sequence length
    y = y[sequence_length:]
    y_pred_fnn = y_pred_fnn[sequence_length:]
    y_pred_pinn = y_pred_pinn[sequence_length:]
    
    # Plot comparisons
    plot_comparison(y, y_pred_fnn, y_pred_rnn, y_pred_pinn)
    
    # Plot loss comparison
    fnn_loss = np.load(os.path.join(project_root, 'FNN', 'training_loss.npy'))
    rnn_loss = np.load(os.path.join(project_root, 'RNN', 'training_loss.npy'))
    plot_loss_comparison(fnn_loss, rnn_loss)
    
    # Print detailed metrics
    print("\nDetailed Model Comparison:")
    for i, joint in enumerate(['Joint 1', 'Joint 2', 'Joint 3']):
        print(f"\n{joint}:")
        
        # Calculate metrics for each model
        metrics_fnn = calculate_metrics(y, y_pred_fnn)[i]
        metrics_rnn = calculate_metrics(y, y_pred_rnn)[i]
        metrics_pinn = calculate_metrics(y, y_pred_pinn)[i]
        
        print("\nFNN Metrics:")
        for metric, value in metrics_fnn.items():
            print(f"{metric:>6}: {value:.6f}")
        
        print("\nRNN Metrics:")
        for metric, value in metrics_rnn.items():
            print(f"{metric:>6}: {value:.6f}")
        
        print("\nPINN Metrics:")
        for metric, value in metrics_pinn.items():
            print(f"{metric:>6}: {value:.6f}")

if __name__ == "__main__":
    main() 
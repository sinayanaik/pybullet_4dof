import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from FNN.predict_torque import load_model as load_fnn
from RNN.predict_torque import load_model as load_rnn
import torch

def load_and_prepare_data(data_path):
    """Load and prepare the test data."""
    data = pd.read_csv(data_path)
    
    # Take only the forward semi-circle (first half of data)
    n_points = len(data) // 2
    data = data.iloc[:n_points]
    
    X = data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values
    y = data[['joint1_torque', 'joint2_torque', 'joint3_torque']].values
    
    return X, y

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

def calculate_metrics(y_true, y_pred):
    """Calculate MSE and MAE for each joint."""
    mse = np.mean((y_true - y_pred) ** 2, axis=0)
    mae = np.mean(np.abs(y_true - y_pred), axis=0)
    return mse, mae

def plot_comparison(y_true, y_pred_fnn, y_pred_rnn, joint_idx, joint_name):
    """Plot actual vs predicted values for both models."""
    plt.figure(figsize=(12, 6))
    plt.plot(y_true[:, joint_idx], label='Actual', alpha=0.7)
    plt.plot(y_pred_fnn[:, joint_idx], label='FNN', alpha=0.7)
    plt.plot(y_pred_rnn[:, joint_idx], label='RNN', alpha=0.7)
    plt.title(f'{joint_name} Torque Comparison')
    plt.xlabel('Time Step')
    plt.ylabel('Torque (Nm)')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'comparison_{joint_name.lower()}.png')
    plt.close()

def main():
    # Load data
    data_path = 'data/semi_circle_trajectory_kp0.10_kd0.4_r0.10_x0.25_z0.10_semicircle5_20250722_125106.csv'
    X, y = load_and_prepare_data(data_path)
    
    # Load FNN model
    fnn_model, fnn_X_scaler, fnn_y_scaler = load_fnn('FNN/torque_model.pth')
    
    # Load RNN model
    rnn_model, rnn_X_scaler, rnn_y_scaler, sequence_length = load_rnn('RNN/torque_model.pth')
    
    # Make predictions
    y_pred_fnn = predict_fnn(X, fnn_model, fnn_X_scaler, fnn_y_scaler)
    y_pred_rnn = predict_rnn(X, rnn_model, rnn_X_scaler, rnn_y_scaler, sequence_length)
    
    # Adjust actual values to match RNN predictions (due to sequence length)
    y = y[sequence_length:]
    y_pred_fnn = y_pred_fnn[sequence_length:]
    
    # Calculate metrics
    fnn_mse, fnn_mae = calculate_metrics(y, y_pred_fnn)
    rnn_mse, rnn_mae = calculate_metrics(y, y_pred_rnn)
    
    # Print results
    print("\nModel Performance Comparison:")
    print("\nMean Squared Error (MSE):")
    print(f"{'Joint':>10} {'FNN':>12} {'RNN':>12} {'Improvement':>12}")
    print("-" * 46)
    for i, joint in enumerate(['Joint 1', 'Joint 2', 'Joint 3']):
        improvement = ((fnn_mse[i] - rnn_mse[i]) / fnn_mse[i]) * 100
        print(f"{joint:>10} {fnn_mse[i]:>12.6f} {rnn_mse[i]:>12.6f} {improvement:>11.2f}%")
    
    print("\nMean Absolute Error (MAE):")
    print(f"{'Joint':>10} {'FNN':>12} {'RNN':>12} {'Improvement':>12}")
    print("-" * 46)
    for i, joint in enumerate(['Joint 1', 'Joint 2', 'Joint 3']):
        improvement = ((fnn_mae[i] - rnn_mae[i]) / fnn_mae[i]) * 100
        print(f"{joint:>10} {fnn_mae[i]:>12.6f} {rnn_mae[i]:>12.6f} {improvement:>11.2f}%")
    
    # Plot comparisons
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    for i, name in enumerate(joint_names):
        plot_comparison(y, y_pred_fnn, y_pred_rnn, i, name)
    
    # Plot combined loss curves
    fnn_loss = np.load('FNN/training_loss.npy')
    rnn_loss = np.load('RNN/training_loss.npy')
    
    plt.figure(figsize=(10, 6))
    plt.plot(fnn_loss, label='FNN', alpha=0.7)
    plt.plot(rnn_loss, label='RNN', alpha=0.7)
    plt.title('Training Loss Comparison')
    plt.xlabel('Epoch')
    plt.ylabel('MSE Loss')
    plt.legend()
    plt.grid(True)
    plt.savefig('loss_comparison.png')
    plt.close()

if __name__ == "__main__":
    main() 
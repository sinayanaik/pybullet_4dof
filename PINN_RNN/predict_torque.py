import torch
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from train_model import PINN_RNN, RobotDataset, MODELS_DIR, DATA_FILE
from torch.utils.data import random_split
import os

def load_latest_model():
    # Find the latest model directory
    model_dirs = [d for d in os.listdir(MODELS_DIR) if d.startswith('pinn_rnn_model_')]
    if not model_dirs:
        raise FileNotFoundError("No trained model found!")
    
    latest_dir = max(model_dirs)
    model_path = os.path.join(MODELS_DIR, latest_dir, 'best_model.pth')
    print(f"Loading model from: {model_path}")
    
    # Load model
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = PINN_RNN().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    
    return model, device, latest_dir

def plot_predictions(true_torques, pred_torques, save_path):
    fig, axes = plt.subplots(3, 1, figsize=(15, 12))
    time = np.arange(len(true_torques))
    
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    
    # Calculate MSE for each joint
    mse = np.mean((true_torques - pred_torques)**2, axis=0)
    
    for i in range(3):
        # Plot true and predicted torques
        axes[i].plot(time, true_torques[:, i], 
                    color='blue', linewidth=2, label='True Torque')
        axes[i].plot(time, pred_torques[:, i], 
                    color='red', linewidth=2, linestyle='--', label='Predicted Torque')
        
        # Add MSE value as text in the plot
        axes[i].text(0.02, 0.95, f'MSE: {mse[i]:.6f} (N⋅m)²', 
                    transform=axes[i].transAxes,
                    bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
                    verticalalignment='top')
        
        axes[i].set_title(f'{joint_names[i]} Torque Prediction')
        axes[i].set_xlabel('Time Step')
        axes[i].set_ylabel('Torque (N⋅m)')
        axes[i].grid(True, linestyle=':', alpha=0.6)
        axes[i].legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Saved prediction plot to: {save_path}")
    plt.close()

def main():
    # Load model
    model, device, model_dir = load_latest_model()
    
    # Create results directory in the same folder as the model
    results_dir = os.path.join(MODELS_DIR, model_dir)
    print(f"Saving results to: {results_dir}")
    
    # Load data
    data = pd.read_csv(DATA_FILE)
    
    # Convert data to tensors
    angles = torch.tensor(data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values, dtype=torch.float32)
    velocities = torch.tensor(data[['joint1_velocity', 'joint2_velocity', 'joint3_velocity']].values, dtype=torch.float32)
    accelerations = torch.tensor(data[['joint1_acceleration', 'joint2_acceleration', 'joint3_acceleration']].values, dtype=torch.float32)
    true_torques = torch.tensor(data[['joint1_torque', 'joint2_torque', 'joint3_torque']].values, dtype=torch.float32)
    
    # Create sequences for RNN
    sequence_length = 50
    num_sequences = len(angles) - sequence_length + 1
    
    # Initialize arrays for predictions
    all_pred_torques = np.zeros_like(true_torques.numpy())
    counts = np.zeros(len(true_torques))  # To average overlapping predictions
    
    # Make predictions
    model.eval()
    with torch.no_grad():
        for i in range(num_sequences):
            # Get sequence
            seq_angles = angles[i:i+sequence_length].unsqueeze(0).to(device)
            seq_velocities = velocities[i:i+sequence_length].unsqueeze(0).to(device)
            seq_accelerations = accelerations[i:i+sequence_length].unsqueeze(0).to(device)
            
            # Get predictions
            pred_sequence = model(seq_angles, seq_velocities, seq_accelerations)
            pred_sequence = pred_sequence.cpu().numpy().squeeze()
            
            # Add predictions to the arrays
            all_pred_torques[i:i+sequence_length] += pred_sequence
            counts[i:i+sequence_length] += 1
    
    # Average the predictions where there were overlapping sequences
    all_pred_torques = all_pred_torques / counts[:, np.newaxis]
    true_torques = true_torques.numpy()
    
    # Calculate metrics (using the last 20% as test set)
    test_start_idx = int(0.8 * len(true_torques))
    test_true_torques = true_torques[test_start_idx:]
    test_pred_torques = all_pred_torques[test_start_idx:]
    
    mse = np.mean((test_true_torques - test_pred_torques)**2, axis=0)
    rmse = np.sqrt(mse)
    mae = np.mean(np.abs(test_true_torques - test_pred_torques), axis=0)
    
    print("\nTest Set Metrics:")
    print("Mean Square Error (MSE):")
    print(f"Joint 1: {mse[0]:.6f} (N⋅m)²")
    print(f"Joint 2: {mse[1]:.6f} (N⋅m)²")
    print(f"Joint 3: {mse[2]:.6f} (N⋅m)²")
    print("\nRoot Mean Square Error (RMSE):")
    print(f"Joint 1: {rmse[0]:.4f} N⋅m")
    print(f"Joint 2: {rmse[1]:.4f} N⋅m")
    print(f"Joint 3: {rmse[2]:.4f} N⋅m")
    print("\nMean Absolute Error (MAE):")
    print(f"Joint 1: {mae[0]:.4f} N⋅m")
    print(f"Joint 2: {mae[1]:.4f} N⋅m")
    print(f"Joint 3: {mae[2]:.4f} N⋅m")
    
    # Save metrics to file
    metrics_path = os.path.join(results_dir, 'test_metrics.txt')
    with open(metrics_path, 'w') as f:
        f.write("Test Set Metrics:\n")
        f.write("Mean Square Error (MSE):\n")
        f.write(f"Joint 1: {mse[0]:.6f} (N⋅m)²\n")
        f.write(f"Joint 2: {mse[1]:.6f} (N⋅m)²\n")
        f.write(f"Joint 3: {mse[2]:.6f} (N⋅m)²\n")
        f.write("\nRoot Mean Square Error (RMSE):\n")
        f.write(f"Joint 1: {rmse[0]:.4f} N⋅m\n")
        f.write(f"Joint 2: {rmse[1]:.4f} N⋅m\n")
        f.write(f"Joint 3: {rmse[2]:.4f} N⋅m\n")
        f.write("\nMean Absolute Error (MAE):\n")
        f.write(f"Joint 1: {mae[0]:.4f} N⋅m\n")
        f.write(f"Joint 2: {mae[1]:.4f} N⋅m\n")
        f.write(f"Joint 3: {mae[2]:.4f} N⋅m\n")
    
    # Plot and save predictions
    plot_path = os.path.join(results_dir, 'predictions.png')
    plot_predictions(true_torques, all_pred_torques, plot_path)

if __name__ == '__main__':
    main() 
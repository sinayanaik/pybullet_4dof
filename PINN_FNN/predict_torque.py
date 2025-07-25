import torch
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from train_model import PINN, RobotDataset, MODELS_DIR, DATA_FILE
from torch.utils.data import random_split
import os

def load_latest_model():
    # Find the latest model directory
    model_dirs = [d for d in os.listdir(MODELS_DIR) if d.startswith('pinn_model_')]
    if not model_dirs:
        raise FileNotFoundError("No trained model found!")
    
    latest_dir = max(model_dirs)
    model_path = os.path.join(MODELS_DIR, latest_dir, 'best_model.pth')
    print(f"Loading model from: {model_path}")
    
    # Load model
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = PINN().to(device)
    model.load_state_dict(torch.load(model_path, map_location=device, weights_only=True))
    model.eval()
    
    return model, device, latest_dir

def plot_predictions(true_torques, pred_torques, save_path):
    fig, axes = plt.subplots(3, 1, figsize=(15, 12))
    time = np.arange(len(true_torques))
    
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    
    for i in range(3):
        # Plot true and predicted torques
        axes[i].plot(time, true_torques[:, i], 
                    color='blue', linewidth=2, label='True Torque')
        axes[i].plot(time, pred_torques[:, i], 
                    color='red', linewidth=2, linestyle='--', label='Predicted Torque')
        
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
    
    # Load full dataset and split into train/test
    full_dataset = RobotDataset(DATA_FILE)
    train_size = int(0.8 * len(full_dataset))
    test_size = len(full_dataset) - train_size
    _, test_dataset = random_split(full_dataset, [train_size, test_size])
    test_indices = test_dataset.indices
    print(f"Total dataset size: {len(full_dataset)}")
    print(f"Test set size: {len(test_dataset)}")
    
    # Make predictions on entire dataset
    model.eval()
    with torch.no_grad():
        # Get predictions for all data
        all_angles = full_dataset.angles.to(device)
        all_velocities = full_dataset.velocities.to(device)
        all_accelerations = full_dataset.accelerations.to(device)
        all_true_torques = full_dataset.torques.numpy()
        
        all_pred_torques = model(all_angles, all_velocities, all_accelerations)
        all_pred_torques = all_pred_torques.cpu().numpy()
        
        # Extract test set predictions for metrics
        test_true_torques = all_true_torques[test_indices]
        test_pred_torques = all_pred_torques[test_indices]
    
    # Calculate metrics on test set
    mse = np.mean((test_true_torques - test_pred_torques)**2, axis=0)
    rmse = np.sqrt(mse)
    mae = np.mean(np.abs(test_true_torques - test_pred_torques), axis=0)
    
    print("\nTest Set Metrics:")
    print("Root Mean Square Error (RMSE):")
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
        f.write("Root Mean Square Error (RMSE):\n")
        f.write(f"Joint 1: {rmse[0]:.4f} N⋅m\n")
        f.write(f"Joint 2: {rmse[1]:.4f} N⋅m\n")
        f.write(f"Joint 3: {rmse[2]:.4f} N⋅m\n")
        f.write("\nMean Absolute Error (MAE):\n")
        f.write(f"Joint 1: {mae[0]:.4f} N⋅m\n")
        f.write(f"Joint 2: {mae[1]:.4f} N⋅m\n")
        f.write(f"Joint 3: {mae[2]:.4f} N⋅m\n")
    print(f"Saved metrics to: {metrics_path}")
    
    # Plot and save predictions for all data
    plot_path = os.path.join(results_dir, 'predictions.png')
    plot_predictions(all_true_torques, all_pred_torques, plot_path)

if __name__ == '__main__':
    main() 
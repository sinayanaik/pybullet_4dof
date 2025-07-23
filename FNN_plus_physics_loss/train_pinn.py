import os
import torch
import numpy as np
import pandas as pd
from torch.utils.data import Dataset, DataLoader
from pinn_model import PINN
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
import argparse
from datetime import datetime

class RobotDataset(Dataset):
    def __init__(self, csv_file):
        """
        Dataset for robot joint data
        Args:
            csv_file: Path to the CSV file containing joint data
        """
        data = pd.read_csv(csv_file)
        
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
        self.features = np.concatenate([angles, velocities, accelerations], axis=1)
        self.features = torch.FloatTensor(self.features)
        
        # Get torques
        self.targets = torch.FloatTensor(data[[
            'joint1_torque', 'joint2_torque', 'joint3_torque'
        ]].values)
        
        # Scale the data
        self.feature_scaler = StandardScaler()
        self.target_scaler = StandardScaler()
        
        self.features = torch.FloatTensor(
            self.feature_scaler.fit_transform(self.features))
        self.targets = torch.FloatTensor(
            self.target_scaler.fit_transform(self.targets))
    
    def __len__(self):
        return len(self.features)
    
    def __getitem__(self, idx):
        return self.features[idx], self.targets[idx]
    
    def get_scalers(self):
        return self.feature_scaler, self.target_scaler

def train_model(args):
    # Set random seed for reproducibility
    torch.manual_seed(42)
    
    # Create save directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = os.path.join("models", f"pinn_model_{timestamp}")
    os.makedirs(save_dir, exist_ok=True)
    
    # Load and prepare data
    dataset = RobotDataset(args.data_file)
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size)
    
    # Initialize model and optimizer
    model = PINN(
        input_size=9,
        hidden_size=args.hidden_size,
        num_layers=args.num_layers,
        dropout_rate=args.dropout_rate
    )
    
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=10, verbose=True)
    
    # Training history
    history = {
        'train_loss': [], 'val_loss': [],
        'train_data_loss': [], 'train_physics_loss': [],
        'val_data_loss': [], 'val_physics_loss': []
    }
    
    best_val_loss = float('inf')
    
    print("Starting training...")
    for epoch in range(args.epochs):
        # Training
        model.train()
        train_total_loss = 0
        train_data_loss_sum = 0
        train_physics_loss_sum = 0
        
        for batch_features, batch_targets in train_loader:
            optimizer.zero_grad()
            
            total_loss, data_loss, physics_loss = model.compute_losses(
                batch_features, batch_targets, args.physics_weight)
            
            total_loss.backward()
            optimizer.step()
            
            train_total_loss += total_loss.item()
            train_data_loss_sum += data_loss.item()
            train_physics_loss_sum += physics_loss.item()
        
        avg_train_loss = train_total_loss / len(train_loader)
        avg_train_data_loss = train_data_loss_sum / len(train_loader)
        avg_train_physics_loss = train_physics_loss_sum / len(train_loader)
        
        # Validation
        model.eval()
        val_total_loss = 0
        val_data_loss_sum = 0
        val_physics_loss_sum = 0
        
        with torch.no_grad():
            for batch_features, batch_targets in val_loader:
                total_loss, data_loss, physics_loss = model.compute_losses(
                    batch_features, batch_targets, args.physics_weight)
                
                val_total_loss += total_loss.item()
                val_data_loss_sum += data_loss.item()
                val_physics_loss_sum += physics_loss.item()
        
        avg_val_loss = val_total_loss / len(val_loader)
        avg_val_data_loss = val_data_loss_sum / len(val_loader)
        avg_val_physics_loss = val_physics_loss_sum / len(val_loader)
        
        # Update learning rate
        scheduler.step(avg_val_loss)
        
        # Save history
        history['train_loss'].append(avg_train_loss)
        history['val_loss'].append(avg_val_loss)
        history['train_data_loss'].append(avg_train_data_loss)
        history['train_physics_loss'].append(avg_train_physics_loss)
        history['val_data_loss'].append(avg_val_data_loss)
        history['val_physics_loss'].append(avg_val_physics_loss)
        
        # Print progress
        if (epoch + 1) % args.print_every == 0:
            print(f"Epoch [{epoch+1}/{args.epochs}]")
            print(f"Train Loss: {avg_train_loss:.6f} "
                  f"(Data: {avg_train_data_loss:.6f}, Physics: {avg_train_physics_loss:.6f})")
            print(f"Val Loss: {avg_val_loss:.6f} "
                  f"(Data: {avg_val_data_loss:.6f}, Physics: {avg_val_physics_loss:.6f})")
        
        # Save best model
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'feature_scaler': dataset.feature_scaler,
                'target_scaler': dataset.target_scaler,
                'val_loss': best_val_loss,
            }, os.path.join(save_dir, 'best_model.pth'))
    
    # Plot training history
    plt.figure(figsize=(12, 8))
    plt.subplot(2, 1, 1)
    plt.plot(history['train_loss'], label='Train Total Loss')
    plt.plot(history['val_loss'], label='Val Total Loss')
    plt.title('Total Loss vs. Epoch')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(history['train_data_loss'], label='Train Data Loss')
    plt.plot(history['train_physics_loss'], label='Train Physics Loss')
    plt.plot(history['val_data_loss'], label='Val Data Loss')
    plt.plot(history['val_physics_loss'], label='Val Physics Loss')
    plt.title('Component Losses vs. Epoch')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'training_history.png'))
    plt.close()
    
    # Save training history
    np.save(os.path.join(save_dir, 'training_history.npy'), history)
    
    print(f"\nTraining completed. Model and history saved in {save_dir}")
    return model, history

def main():
    parser = argparse.ArgumentParser(description='Train PINN model for robot dynamics')
    parser.add_argument('--data_file', type=str, required=True,
                      help='Path to the CSV file containing training data')
    parser.add_argument('--epochs', type=int, default=1000,
                      help='Number of training epochs')
    parser.add_argument('--batch_size', type=int, default=32,
                      help='Batch size for training')
    parser.add_argument('--learning_rate', type=float, default=0.001,
                      help='Learning rate for optimizer')
    parser.add_argument('--hidden_size', type=int, default=128,
                      help='Number of neurons in hidden layers')
    parser.add_argument('--num_layers', type=int, default=4,
                      help='Number of hidden layers')
    parser.add_argument('--dropout_rate', type=float, default=0.1,
                      help='Dropout rate for regularization')
    parser.add_argument('--physics_weight', type=float, default=0.1,
                      help='Weight for physics loss term (λ)')
    parser.add_argument('--print_every', type=int, default=10,
                      help='Print progress every N epochs')
    
    args = parser.parse_args()
    
    # Create models directory if it doesn't exist
    os.makedirs("models", exist_ok=True)
    
    # Train the model
    model, history = train_model(args)

if __name__ == "__main__":
    main() 
import torch
import torch.nn as nn
import numpy as np
import pandas as pd
from torch.utils.data import Dataset, DataLoader, random_split
import matplotlib.pyplot as plt
from datetime import datetime
import os

# Constants from URDF and dynamics model
L1 = 0.15  # Length of link 1
L2 = 0.15  # Length of link 2
L3 = 0.10  # Length of link 3
Z_OFFSET = 0.27  # Z-offset from base
g = 9.81  # Gravity constant

# Mass parameters from URDF
m1 = 0.6  # Mass of link 2 (first movable link)
m2 = 0.4  # Mass of link 3 (second movable link)
m3 = 0.3  # Mass of link 4 (third movable link)

# Inertia parameters from URDF
I1 = (1/3) * m1 * L1**2  # Moment of inertia for link 1
I2 = (1/3) * m2 * L2**2  # Moment of inertia for link 2
I3 = (1/3) * m3 * L3**2  # Moment of inertia for link 3

# Get the absolute paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_FILE = os.path.join(os.path.dirname(SCRIPT_DIR), 'data', 'semi_circle_trajectory_kp0.10_kd0.4_r0.10_x0.25_z0.10_semicircle3_20250724_140439.csv')
MODELS_DIR = os.path.join(SCRIPT_DIR, 'models')
os.makedirs(MODELS_DIR, exist_ok=True)

class RobotDataset(Dataset):
    def __init__(self, csv_file=None, data_dict=None):
        if csv_file is not None:
            print(f"Loading data from: {csv_file}")
            data = pd.read_csv(csv_file)
            
            # Extract features and targets
            self.angles = torch.tensor(data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values, dtype=torch.float32)
            self.velocities = torch.tensor(data[['joint1_velocity', 'joint2_velocity', 'joint3_velocity']].values, dtype=torch.float32)
            self.accelerations = torch.tensor(data[['joint1_acceleration', 'joint2_acceleration', 'joint3_acceleration']].values, dtype=torch.float32)
            self.torques = torch.tensor(data[['joint1_torque', 'joint2_torque', 'joint3_torque']].values, dtype=torch.float32)
        else:
            self.angles = data_dict['angles']
            self.velocities = data_dict['velocities']
            self.accelerations = data_dict['accelerations']
            self.torques = data_dict['torques']
            
        print(f"Dataset size: {len(self.angles)} samples")
        
    def __len__(self):
        return len(self.angles)
    
    def __getitem__(self, idx):
        return {
            'angles': self.angles[idx],
            'velocities': self.velocities[idx],
            'accelerations': self.accelerations[idx],
            'torques': self.torques[idx]
        }

class PINN(nn.Module):
    def __init__(self, input_size=9, hidden_size=256):
        super(PINN, self).__init__()
        
        self.network = nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, 3)  # Output: 3 torques
        )
        
    def forward(self, angles, velocities, accelerations):
        # Combine inputs
        x = torch.cat([angles, velocities, accelerations], dim=-1)
        return self.network(x)

def compute_physics_loss(model, angles, velocities, accelerations, device):
    """
    Compute physics-based loss using the exact equations of motion from the dynamics model
    """
    # Convert inputs to require grad
    angles = angles.clone().detach().requires_grad_(True)
    velocities = velocities.clone().detach().requires_grad_(True)
    accelerations = accelerations.clone().detach().requires_grad_(True)
    
    # Get predicted torques
    pred_torques = model(angles, velocities, accelerations)
    
    # Compute gravitational terms (from dynamics model)
    g_term1 = m1 * g * L1/2 * torch.cos(angles[:, 0]) + \
              m2 * g * L1 * torch.cos(angles[:, 0]) + \
              m2 * g * L2/2 * torch.cos(angles[:, 0] + angles[:, 1]) + \
              m3 * g * (L1 * torch.cos(angles[:, 0]) + L2 * torch.cos(angles[:, 0] + angles[:, 1]) + \
                       L3/2 * torch.cos(angles[:, 0] + angles[:, 1] + angles[:, 2]))
    
    g_term2 = m2 * g * L2/2 * torch.cos(angles[:, 0] + angles[:, 1]) + \
              m3 * g * (L2 * torch.cos(angles[:, 0] + angles[:, 1]) + \
                       L3/2 * torch.cos(angles[:, 0] + angles[:, 1] + angles[:, 2]))
    
    g_term3 = m3 * g * L3/2 * torch.cos(angles[:, 0] + angles[:, 1] + angles[:, 2])
    
    # Compute inertial terms (from dynamics model)
    M11 = I1 + I2 + I3 + \
          m1 * (L1/2)**2 + \
          m2 * (L1**2 + (L2/2)**2 + 2 * L1 * L2/2 * torch.cos(angles[:, 1])) + \
          m3 * (L1**2 + L2**2 + (L3/2)**2 + \
                2 * L1 * L2 * torch.cos(angles[:, 1]) + \
                2 * L1 * L3/2 * torch.cos(angles[:, 1] + angles[:, 2]) + \
                2 * L2 * L3/2 * torch.cos(angles[:, 2]))
    
    M22 = I2 + I3 + \
          m2 * (L2/2)**2 + \
          m3 * (L2**2 + (L3/2)**2 + 2 * L2 * L3/2 * torch.cos(angles[:, 2]))
    
    M33 = I3 + m3 * (L3/2)**2
    
    # Compute Coriolis and centrifugal terms
    C11 = -(m2 * L1 * L2/2 * torch.sin(angles[:, 1]) + \
            m3 * L1 * L2 * torch.sin(angles[:, 1]) + \
            m3 * L1 * L3/2 * torch.sin(angles[:, 1] + angles[:, 2])) * velocities[:, 1]**2
    
    C12 = -(m3 * L2 * L3/2 * torch.sin(angles[:, 2])) * velocities[:, 2]**2
    
    C21 = (m2 * L1 * L2/2 * torch.sin(angles[:, 1]) + \
           m3 * L1 * L2 * torch.sin(angles[:, 1])) * velocities[:, 0]**2
    
    C22 = -(m3 * L2 * L3/2 * torch.sin(angles[:, 2])) * velocities[:, 2]**2
    
    C31 = (m3 * L1 * L3/2 * torch.sin(angles[:, 1] + angles[:, 2])) * velocities[:, 0]**2
    
    C32 = (m3 * L2 * L3/2 * torch.sin(angles[:, 2])) * velocities[:, 1]**2
    
    # Compute expected torques based on physics
    physics_torque1 = M11 * accelerations[:, 0] + C11 + C12 + g_term1
    physics_torque2 = M22 * accelerations[:, 1] + C21 + C22 + g_term2
    physics_torque3 = M33 * accelerations[:, 2] + C31 + C32 + g_term3
    
    physics_torques = torch.stack([physics_torque1, physics_torque2, physics_torque3], dim=1)
    
    # Compute physics loss
    physics_loss = torch.mean((pred_torques - physics_torques)**2)
    
    return physics_loss

def train_model(model, train_loader, val_loader, num_epochs, device, physics_weight=0.1):
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    mse_loss = nn.MSELoss()
    
    # Lists to store losses
    train_data_losses = []
    train_physics_losses = []
    train_total_losses = []
    val_losses = []
    
    best_val_loss = float('inf')
    best_model_state = None
    
    print("\nTraining Progress:")
    print("Epoch  Data Loss  Physics Loss  Total Loss  Val Loss")
    print("-" * 50)
    
    for epoch in range(num_epochs):
        # Training
        model.train()
        epoch_data_loss = 0
        epoch_physics_loss = 0
        epoch_total_loss = 0
        num_batches = 0
        
        for batch in train_loader:
            angles = batch['angles'].to(device)
            velocities = batch['velocities'].to(device)
            accelerations = batch['accelerations'].to(device)
            true_torques = batch['torques'].to(device)
            
            # Forward pass
            pred_torques = model(angles, velocities, accelerations)
            
            # Compute losses
            data_loss = mse_loss(pred_torques, true_torques)
            physics_loss = compute_physics_loss(model, angles, velocities, accelerations, device)
            
            # Total loss
            total_loss = data_loss + physics_weight * physics_loss
            
            # Backward pass and optimize
            optimizer.zero_grad()
            total_loss.backward()
            optimizer.step()
            
            # Accumulate losses
            epoch_data_loss += data_loss.item()
            epoch_physics_loss += physics_loss.item()
            epoch_total_loss += total_loss.item()
            num_batches += 1
        
        # Average training losses
        avg_data_loss = epoch_data_loss / num_batches
        avg_physics_loss = epoch_physics_loss / num_batches
        avg_total_loss = epoch_total_loss / num_batches
        
        train_data_losses.append(avg_data_loss)
        train_physics_losses.append(avg_physics_loss)
        train_total_losses.append(avg_total_loss)
        
        # Validation
        model.eval()
        val_loss = 0
        num_val_batches = 0
        
        with torch.no_grad():
            for batch in val_loader:
                angles = batch['angles'].to(device)
                velocities = batch['velocities'].to(device)
                accelerations = batch['accelerations'].to(device)
                true_torques = batch['torques'].to(device)
                
                pred_torques = model(angles, velocities, accelerations)
                val_loss += mse_loss(pred_torques, true_torques).item()
                num_val_batches += 1
        
        avg_val_loss = val_loss / num_val_batches
        val_losses.append(avg_val_loss)
        
        # Save best model
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            best_model_state = model.state_dict()
        
        # Print progress every 10 epochs
        if (epoch + 1) % 10 == 0:
            print(f"{epoch+1:5d}  {avg_data_loss:.4f}    {avg_physics_loss:.4f}     {avg_total_loss:.4f}    {avg_val_loss:.4f}")
    
    print("-" * 50)
    print("Training completed!")
    print(f"Best validation loss: {best_val_loss:.4f}")
    
    # Restore best model
    model.load_state_dict(best_model_state)
    return train_data_losses, train_physics_losses, train_total_losses, val_losses

def plot_losses(train_data_losses, train_physics_losses, train_total_losses, val_losses, save_path):
    plt.figure(figsize=(12, 6))
    plt.plot(train_data_losses, label='Train Data Loss')
    plt.plot(train_physics_losses, label='Train Physics Loss')
    plt.plot(train_total_losses, label='Train Total Loss')
    plt.plot(val_losses, label='Validation Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training and Validation Losses')
    plt.legend()
    plt.grid(True)
    plt.savefig(save_path)
    plt.close()

def main():
    # Set device
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    # Create timestamp for saving results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Create save directory inside models directory
    save_dir = os.path.join(MODELS_DIR, f'pinn_model_{timestamp}')
    os.makedirs(save_dir, exist_ok=True)
    print(f"Saving results to: {save_dir}")
    
    # Load data
    full_dataset = RobotDataset(DATA_FILE)
    
    # Split into train and validation sets
    train_size = int(0.8 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_dataset, val_dataset = random_split(full_dataset, [train_size, val_size])
    
    print(f"Training set size: {len(train_dataset)}")
    print(f"Validation set size: {len(val_dataset)}")
    
    # Create data loaders
    train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)
    
    # Create and train model
    model = PINN().to(device)
    num_epochs = 1000  # Increased to 1000 epochs
    train_data_losses, train_physics_losses, train_total_losses, val_losses = train_model(
        model, train_loader, val_loader, num_epochs, device, physics_weight=0.5
    )
    
    # Save model
    model_path = os.path.join(save_dir, 'best_model.pth')
    torch.save(model.state_dict(), model_path)
    print(f"\nSaved model to: {model_path}")
    
    # Save losses
    losses_path = os.path.join(save_dir, 'training_history.npy')
    losses = np.array([train_data_losses, train_physics_losses, train_total_losses, val_losses])
    np.save(losses_path, losses)
    print(f"Saved training history to: {losses_path}")
    
    # Plot and save losses
    plot_path = os.path.join(save_dir, 'training_history.png')
    plot_losses(train_data_losses, train_physics_losses, train_total_losses, val_losses, plot_path)
    print(f"Saved training plot to: {plot_path}")

if __name__ == '__main__':
    main() 
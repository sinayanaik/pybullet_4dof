import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
import os

# Define the neural network architecture
class TorquePredictor(nn.Module):
    def __init__(self, input_size=3, hidden_size=64, output_size=3):  # Changed output_size to 3
        super(TorquePredictor, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, output_size)
        )
        
    def forward(self, x):
        return self.network(x)

# Custom dataset class
class TorqueDataset(Dataset):
    def __init__(self, X, y):
        self.X = torch.FloatTensor(X)
        self.y = torch.FloatTensor(y)
        
    def __len__(self):
        return len(self.X)
    
    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

def train_model(model, train_loader, criterion, optimizer, num_epochs=1000):  # Doubled epochs
    losses = []
    
    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0
        for batch_X, batch_y in train_loader:
            # Forward pass
            outputs = model(batch_X)
            loss = criterion(outputs, batch_y)
            
            # Backward pass and optimize
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            epoch_loss += loss.item()
            
        avg_loss = epoch_loss / len(train_loader)
        losses.append(avg_loss)
        
        if (epoch + 1) % 100 == 0:  # Updated print frequency to every 100 epochs
            print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {avg_loss:.4f}')
    
    return losses

def main():
    # Load and preprocess data
    data_path = '../data/semi_circle_trajectory_kp0.10_kd0.4_r0.10_x0.25_z0.10_semicircle5_20250722_125106.csv'
    data = pd.read_csv(data_path)
    
    # Take only the forward semi-circle (first half of data)
    n_points = len(data) // 2
    data = data.iloc[:n_points]
    
    # Prepare input features (joint angles) and targets (torques)
    X = data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values
    y = data[['joint1_torque', 'joint2_torque', 'joint3_torque']].values  # Now including joint1_torque
    
    # Scale the data
    X_scaler = StandardScaler()
    y_scaler = StandardScaler()
    
    X_scaled = X_scaler.fit_transform(X)
    y_scaled = y_scaler.fit_transform(y)
    
    # Create dataset and dataloader
    dataset = TorqueDataset(X_scaled, y_scaled)
    train_loader = DataLoader(dataset, batch_size=32, shuffle=True)
    
    # Initialize model, loss function, and optimizer
    model = TorquePredictor()
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.005)  # Increased learning rate
    
    # Train the model
    losses = train_model(model, train_loader, criterion, optimizer)
    
    # Save the model and scalers
    torch.save({
        'model_state_dict': model.state_dict(),
        'X_scaler': X_scaler,
        'y_scaler': y_scaler
    }, 'torque_model.pth')
    
    # Save losses for comparison
    np.save('training_loss.npy', np.array(losses))
    
    # Plot training loss
    plt.figure(figsize=(10, 6))
    plt.plot(losses)
    plt.title('Training Loss over Epochs')
    plt.xlabel('Epoch')
    plt.ylabel('MSE Loss')
    plt.grid(True)
    plt.savefig('training_loss.png')
    plt.close()
    
    # Make predictions and visualize results
    model.eval()
    with torch.no_grad():
        X_tensor = torch.FloatTensor(X_scaled)
        y_pred_scaled = model(X_tensor)
        y_pred = y_scaler.inverse_transform(y_pred_scaled)
    
    # Calculate MSE for each joint
    mse_joint1 = np.mean((y[:, 0] - y_pred[:, 0])**2)
    mse_joint2 = np.mean((y[:, 1] - y_pred[:, 1])**2)
    mse_joint3 = np.mean((y[:, 2] - y_pred[:, 2])**2)
    
    # Plot predictions vs actual
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
    
    # Joint 1 Torque
    ax1.plot(y[:, 0], label='Actual', alpha=0.7)
    ax1.plot(y_pred[:, 0], label='Predicted', alpha=0.7)
    ax1.set_title(f'Joint 1 Torque (MSE: {mse_joint1:.6f})')
    ax1.set_xlabel('Time Step')
    ax1.set_ylabel('Torque (Nm)')
    ax1.legend()
    ax1.grid(True)
    
    # Joint 2 Torque
    ax2.plot(y[:, 1], label='Actual', alpha=0.7)
    ax2.plot(y_pred[:, 1], label='Predicted', alpha=0.7)
    ax2.set_title(f'Joint 2 Torque (MSE: {mse_joint2:.6f})')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Torque (Nm)')
    ax2.legend()
    ax2.grid(True)
    
    # Joint 3 Torque
    ax3.plot(y[:, 2], label='Actual', alpha=0.7)
    ax3.plot(y_pred[:, 2], label='Predicted', alpha=0.7)
    ax3.set_title(f'Joint 3 Torque (MSE: {mse_joint3:.6f})')
    ax3.set_xlabel('Time Step')
    ax3.set_ylabel('Torque (Nm)')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('prediction_results.png')
    plt.close()

if __name__ == "__main__":
    main() 
import torch
import torch.nn as nn
import numpy as np
import pandas as pd
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
from datetime import datetime
import os
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import filedialog

# Get absolute paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(PROJECT_ROOT, "data")
MODELS_DIR = os.path.join(SCRIPT_DIR, "models")

# Create directories if they don't exist
os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(MODELS_DIR, exist_ok=True)

# Constants for physics calculations
g = 9.81  # Gravity constant
# Link parameters from URDF
L1, L2, L3 = 0.15, 0.15, 0.10  # Link lengths
m1, m2, m3 = 0.6, 0.4, 0.3    # Link masses
I1 = (1/3) * m1 * L1**2  # Moment of inertia for link 1
I2 = (1/3) * m2 * L2**2  # Moment of inertia for link 2
I3 = (1/3) * m3 * L3**2  # Moment of inertia for link 3

class RobotDataset(Dataset):
    def __init__(self, csv_file, forward_only=True, sequence_length=10):
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"Data file not found: {csv_file}\nPlease place your data file in: {DATA_DIR}")
            
        data = pd.read_csv(csv_file)
        
        # Use only forward stroke (first half of data)
        if forward_only:
            data = data.iloc[:len(data)//2]
        
        # Input features: joint positions, velocities, accelerations
        self.q = torch.tensor(data[['joint1_angle', 'joint2_angle', 'joint3_angle']].values, dtype=torch.float32)
        self.qdot = torch.tensor(data[['joint1_velocity', 'joint2_velocity', 'joint3_velocity']].values, dtype=torch.float32)
        self.qddot = torch.tensor(data[['joint1_acceleration', 'joint2_acceleration', 'joint3_acceleration']].values, dtype=torch.float32)
        
        # Target: joint torques
        self.tau = torch.tensor(data[['joint1_torque', 'joint2_torque', 'joint3_torque']].values, dtype=torch.float32)
        
        self.sequence_length = sequence_length
        
    def __len__(self):
        return len(self.q) - self.sequence_length + 1
    
    def __getitem__(self, idx):
        X = torch.cat([
            self.q[idx:idx+self.sequence_length],
            self.qdot[idx:idx+self.sequence_length],
            self.qddot[idx:idx+self.sequence_length]
        ], dim=1)
        y = self.tau[idx+self.sequence_length-1]
        return X, y

def compute_physics_loss(q, qdot, qddot, pred_tau):
    """Compute physics-based loss using robot dynamics equations"""
    batch_size = q.shape[0]
    
    # Reshape tensors to handle batch dimension
    q_last = q[:, -1, :]  # Shape: [batch_size, 3]
    qdot_last = qdot[:, -1, :]  # Shape: [batch_size, 3]
    qddot_last = qddot[:, -1, :]  # Shape: [batch_size, 3]
    
    # Gravitational terms
    g_term1 = m1 * g * L1/2 * torch.cos(q_last[:, 0]) + \
              m2 * g * L1 * torch.cos(q_last[:, 0]) + \
              m2 * g * L2/2 * torch.cos(q_last[:, 0] + q_last[:, 1]) + \
              m3 * g * (L1 * torch.cos(q_last[:, 0]) + L2 * torch.cos(q_last[:, 0] + q_last[:, 1]) + \
                       L3/2 * torch.cos(q_last[:, 0] + q_last[:, 1] + q_last[:, 2]))
    
    g_term2 = m2 * g * L2/2 * torch.cos(q_last[:, 0] + q_last[:, 1]) + \
              m3 * g * (L2 * torch.cos(q_last[:, 0] + q_last[:, 1]) + \
                       L3/2 * torch.cos(q_last[:, 0] + q_last[:, 1] + q_last[:, 2]))
    
    g_term3 = m3 * g * L3/2 * torch.cos(q_last[:, 0] + q_last[:, 1] + q_last[:, 2])
    
    # Inertial terms
    M11 = I1 + I2 + I3 + \
          m1 * (L1/2)**2 + \
          m2 * (L1**2 + (L2/2)**2 + 2 * L1 * L2/2 * torch.cos(q_last[:, 1])) + \
          m3 * (L1**2 + L2**2 + (L3/2)**2 + \
                2 * L1 * L2 * torch.cos(q_last[:, 1]) + \
                2 * L1 * L3/2 * torch.cos(q_last[:, 1] + q_last[:, 2]) + \
                2 * L2 * L3/2 * torch.cos(q_last[:, 2]))
    
    M22 = I2 + I3 + \
          m2 * (L2/2)**2 + \
          m3 * (L2**2 + (L3/2)**2 + 2 * L2 * L3/2 * torch.cos(q_last[:, 2]))
    
    M33 = I3 + m3 * (L3/2)**2
    
    # Expected torques based on physics (use last timestep for prediction)
    physics_tau1 = M11 * qddot_last[:, 0] + g_term1
    physics_tau2 = M22 * qddot_last[:, 1] + g_term2
    physics_tau3 = torch.ones_like(qddot_last[:, 2]) * M33 * qddot_last[:, 2] + g_term3
    
    physics_tau = torch.stack([physics_tau1, physics_tau2, physics_tau3], dim=1)
    
    return nn.MSELoss()(pred_tau, physics_tau)

class PINN_RNN(nn.Module):
    def __init__(self, input_size=9, hidden_size=64, num_layers=2, dropout=0.1):
        super(PINN_RNN, self).__init__()
        
        self.rnn = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            dropout=dropout if num_layers > 1 else 0,
            batch_first=True
        )
        
        self.fc = nn.Sequential(
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(hidden_size, 3)
        )
    
    def forward(self, x):
        rnn_out, _ = self.rnn(x)
        return self.fc(rnn_out[:, -1, :])  # Use only last sequence output

class HyperparameterGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("PINN RNN Hyperparameters")
        
        # Default values
        self.params = {
            'hidden_size': 64,
            'num_layers': 2,
            'dropout': 0.1,
            'sequence_length': 10,
            'learning_rate': 0.001,
            'batch_size': 32,
            'epochs': 100,
            'train_split': 0.8,
            'data_weight': 1.0,
            'physics_weight': 0.0  # Start with pure data-driven
        }
        
        # Set default data file
        default_data_file = os.path.join(DATA_DIR, "semi_circle_trajectory_kp0.10_kd0.4_r0.10_x0.25_z0.10_semicircle3_20250724_140439.csv")
        self.data_file = default_data_file if os.path.exists(default_data_file) else None
        
        # Create and pack widgets
        self.create_widgets()
        
        self.result = None

    def browse_file(self):
        initial_dir = DATA_DIR if os.path.exists(DATA_DIR) else os.path.expanduser("~")
        filename = filedialog.askopenfilename(
            initialdir=initial_dir,
            title="Select Robot Data File",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*"))
        )
        if filename:
            self.data_file = filename
            display_name = os.path.basename(filename)
            self.file_label.config(text=display_name)
    
    def create_widgets(self):
        # Create a frame for better organization
        frame = ttk.Frame(self.root, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Add data file selection
        ttk.Label(frame, text="Data File:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.file_label = ttk.Label(frame, text="No file selected" if not self.data_file else os.path.basename(self.data_file), 
                                   font=('Arial', 8))
        self.file_label.grid(row=0, column=1, columnspan=2, sticky=tk.W, pady=5)
        ttk.Button(frame, text="Browse", command=self.browse_file).grid(row=0, column=3, pady=5)
        
        # Create entry fields for each parameter
        row = 1
        self.entries = {}
        
        for param, default in self.params.items():
            ttk.Label(frame, text=f"{param}:").grid(row=row, column=0, sticky=tk.W, pady=5)
            entry = ttk.Entry(frame)
            entry.insert(0, str(default))
            entry.grid(row=row, column=1, padx=5, pady=5)
            self.entries[param] = entry
            
            # Add help text
            help_text = self.get_help_text(param)
            ttk.Label(frame, text=help_text, font=('Arial', 8)).grid(row=row, column=2, sticky=tk.W, pady=5)
            
            row += 1
        
        # Add buttons
        ttk.Button(frame, text="Start Training", command=self.on_submit).grid(row=row, column=0, pady=20)
        ttk.Button(frame, text="Cancel", command=self.on_cancel).grid(row=row, column=1, pady=20)
    
    def get_help_text(self, param):
        help_texts = {
            'hidden_size': '(32-128) Number of neurons in hidden layers',
            'num_layers': '(1-3) Number of RNN layers',
            'dropout': '(0.0-0.5) Dropout rate for regularization',
            'sequence_length': '(5-20) Length of input sequences',
            'learning_rate': '(0.0001-0.01) Learning rate for optimizer',
            'batch_size': '(16-64) Number of samples per batch',
            'epochs': '(50-500) Number of training epochs',
            'train_split': '(0.6-0.9) Fraction of data for training',
            'data_weight': '(0.0-1.0) Weight for data-driven loss',
            'physics_weight': '(0.0-1.0) Weight for physics-based loss'
        }
        return help_texts.get(param, '')
    
    def validate_params(self):
        try:
            if not self.data_file:
                raise ValueError("Please select a data file")
                
            params = {}
            for key, entry in self.entries.items():
                value = float(entry.get())
                if key in ['hidden_size', 'num_layers', 'sequence_length', 'batch_size', 'epochs']:
                    value = int(value)
                params[key] = value
            
            # Add data file to params
            params['data_file'] = self.data_file
            
            # Validation checks
            if not (32 <= params['hidden_size'] <= 128):
                raise ValueError("Hidden size should be between 32 and 128")
            if not (1 <= params['num_layers'] <= 3):
                raise ValueError("Number of layers should be between 1 and 3")
            if not (0.0 <= params['dropout'] <= 0.5):
                raise ValueError("Dropout should be between 0.0 and 0.5")
            if not (5 <= params['sequence_length'] <= 20):
                raise ValueError("Sequence length should be between 5 and 20")
            if not (0.0001 <= params['learning_rate'] <= 0.01):
                raise ValueError("Learning rate should be between 0.0001 and 0.01")
            if not (16 <= params['batch_size'] <= 64):
                raise ValueError("Batch size should be between 16 and 64")
            if not (50 <= params['epochs'] <= 500):
                raise ValueError("Epochs should be between 50 and 500")
            if not (0.6 <= params['train_split'] <= 0.9):
                raise ValueError("Train split should be between 0.6 and 0.9")
            if not (0.0 <= params['data_weight'] <= 1.0):
                raise ValueError("Data weight should be between 0.0 and 1.0")
            if not (0.0 <= params['physics_weight'] <= 1.0):
                raise ValueError("Physics weight should be between 0.0 and 1.0")
            if abs(params['data_weight'] + params['physics_weight'] - 1.0) > 1e-6:
                raise ValueError("Data weight + Physics weight should equal 1.0")
            
            return params
        except ValueError as e:
            messagebox.showerror("Invalid Input", str(e))
            return None
    
    def on_submit(self):
        params = self.validate_params()
        if params:
            self.result = params
            self.root.destroy()
    
    def on_cancel(self):
        self.root.destroy()
    
    def get_params(self):
        self.root.mainloop()
        return self.result

def train_model(model, train_loader, test_loader, optimizer, criterion, params, is_physics=False):
    train_losses = []
    test_losses = []
    data_losses = []
    physics_losses = []
    best_test_loss = float('inf')
    
    for epoch in range(params['epochs']):
        # Training
        model.train()
        train_loss = 0
        epoch_data_loss = 0
        epoch_physics_loss = 0
        
        for X, y in train_loader:
            optimizer.zero_grad()
            y_pred = model(X)
            
            # Compute losses
            data_loss = criterion(y_pred, y)
            
            if is_physics:
                # Split X into q, qdot, qddot for physics computation
                batch_size, seq_len = X.shape[0], X.shape[1]
                q = X[:, :, :3].reshape(batch_size, seq_len, 3)
                qdot = X[:, :, 3:6].reshape(batch_size, seq_len, 3)
                qddot = X[:, :, 6:].reshape(batch_size, seq_len, 3)
                physics_loss = compute_physics_loss(q, qdot, qddot, y_pred)
                loss = params['data_weight'] * data_loss + params['physics_weight'] * physics_loss
                epoch_physics_loss += physics_loss.item()
            else:
                loss = data_loss
                physics_loss = torch.tensor(0.0)
            
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            epoch_data_loss += data_loss.item()
        
        train_loss /= len(train_loader)
        epoch_data_loss /= len(train_loader)
        epoch_physics_loss /= len(train_loader)
        train_losses.append(train_loss)
        data_losses.append(epoch_data_loss)
        physics_losses.append(epoch_physics_loss)
        
        # Testing
        model.eval()
        test_loss = 0
        with torch.no_grad():
            for X, y in test_loader:
                y_pred = model(X)
                test_loss += criterion(y_pred, y).item()
        test_loss /= len(test_loader)
        test_losses.append(test_loss)
        
        if (epoch + 1) % 10 == 0:
            if is_physics:
                print(f"Epoch {epoch+1}/{params['epochs']}, Train Loss: {train_loss:.6f}, "
                      f"Data Loss: {epoch_data_loss:.6f}, Physics Loss: {epoch_physics_loss:.6f}, "
                      f"Test Loss: {test_loss:.6f}")
            else:
                print(f"Epoch {epoch+1}/{params['epochs']}, Train Loss: {train_loss:.6f}, "
                      f"Test Loss: {test_loss:.6f}")
        
        if test_loss < best_test_loss:
            best_test_loss = test_loss
            best_state = model.state_dict()
    
    return best_state, train_losses, test_losses, data_losses, physics_losses

def evaluate_model(model, dataset):
    model.eval()
    all_predictions = []
    all_targets = []
    
    with torch.no_grad():
        for i in range(len(dataset)):
            X, y = dataset[i]
            X = X.unsqueeze(0)
            pred = model(X)
            all_predictions.append(pred.numpy())
            all_targets.append(y.numpy())
    
    predictions = np.concatenate(all_predictions)
    targets = np.concatenate(all_targets).reshape(-1, 3)
    
    return predictions, targets

def train_and_evaluate(params):
    # Create output directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_dir = os.path.join(MODELS_DIR, f"pinn_rnn_h{params['hidden_size']}_l{params['num_layers']}_d{params['dropout']:.2f}_s{params['sequence_length']}_{timestamp}")
    os.makedirs(base_dir, exist_ok=True)
    
    # Load and split data
    dataset = RobotDataset(params['data_file'], forward_only=True, sequence_length=params['sequence_length'])
    train_size = int(params['train_split'] * len(dataset))
    test_size = len(dataset) - train_size
    train_dataset, test_dataset = torch.utils.data.random_split(dataset, [train_size, test_size])
    
    train_loader = DataLoader(train_dataset, batch_size=params['batch_size'], shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=params['batch_size'])
    
    # Train data-driven model
    print("\nTraining Data-Driven Model...")
    data_model = PINN_RNN(hidden_size=params['hidden_size'], num_layers=params['num_layers'], dropout=params['dropout'])
    data_optimizer = torch.optim.Adam(data_model.parameters(), lr=params['learning_rate'])
    criterion = nn.MSELoss()
    
    data_best_state, data_train_losses, data_test_losses, _, _ = train_model(
        data_model, train_loader, test_loader, data_optimizer, criterion, params, is_physics=False
    )
    
    # Save data-driven model
    torch.save(data_best_state, os.path.join(base_dir, "model_data_driven.pth"))
    
    # Train PINN model if physics weight > 0
    if params['physics_weight'] > 0:
        print("\nTraining Physics-Informed Model...")
        pinn_model = PINN_RNN(hidden_size=params['hidden_size'], num_layers=params['num_layers'], dropout=params['dropout'])
        pinn_optimizer = torch.optim.Adam(pinn_model.parameters(), lr=params['learning_rate'])
        
        pinn_best_state, pinn_train_losses, pinn_test_losses, pinn_data_losses, pinn_physics_losses = train_model(
            pinn_model, train_loader, test_loader, pinn_optimizer, criterion, params, is_physics=True
        )
        
        # Save PINN model
        torch.save(pinn_best_state, os.path.join(base_dir, "model_pinn.pth"))
    
    # Generate predictions on full dataset
    data_model.load_state_dict(data_best_state)
    data_predictions, targets = evaluate_model(data_model, dataset)
    
    if params['physics_weight'] > 0:
        pinn_model.load_state_dict(pinn_best_state)
        pinn_predictions, _ = evaluate_model(pinn_model, dataset)
    
    # Plot training history
    plt.figure(figsize=(12, 6))
    plt.plot(data_train_losses, label='Data-Driven Train Loss')
    plt.plot(data_test_losses, label='Data-Driven Test Loss')
    if params['physics_weight'] > 0:
        plt.plot(pinn_train_losses, label='PINN Train Loss')
        plt.plot(pinn_test_losses, label='PINN Test Loss')
        plt.plot(pinn_data_losses, label='PINN Data Loss')
        plt.plot(pinn_physics_losses, label='PINN Physics Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training History')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(base_dir, "training_history.png"))
    plt.close()
    
    # Plot predictions
    fig, axes = plt.subplots(3, 1, figsize=(15, 12))
    time = np.arange(len(targets))
    
    # Calculate MSE for each joint
    data_mse = np.mean((targets - data_predictions)**2, axis=0)
    if params['physics_weight'] > 0:
        pinn_mse = np.mean((targets - pinn_predictions)**2, axis=0)
    
    for i in range(3):
        axes[i].plot(time, targets[:, i], color='blue', label='True Torque')
        axes[i].plot(time, data_predictions[:, i], color='red', linestyle='--', 
                    label=f'Data-Driven (MSE: {data_mse[i]:.6f})')
        if params['physics_weight'] > 0:
            axes[i].plot(time, pinn_predictions[:, i], color='green', linestyle=':', 
                        label=f'PINN (MSE: {pinn_mse[i]:.6f})')
        
        axes[i].set_title(f'Joint {i+1} Torque')
        axes[i].set_xlabel('Time Step')
        axes[i].set_ylabel('Torque (N⋅m)')
        axes[i].grid(True)
        axes[i].legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(base_dir, "predictions.png"))
    plt.close()
    
    # Save metrics
    with open(os.path.join(base_dir, "metrics.txt"), 'w') as f:
        f.write("Data-Driven Model MSE per joint:\n")
        for i, mse in enumerate(data_mse):
            f.write(f"Joint {i+1}: {mse:.6f} (N⋅m)²\n")
        f.write(f"\nData-Driven Average MSE: {np.mean(data_mse):.6f} (N⋅m)²\n")
        
        if params['physics_weight'] > 0:
            f.write("\nPINN Model MSE per joint:\n")
            for i, mse in enumerate(pinn_mse):
                f.write(f"Joint {i+1}: {mse:.6f} (N⋅m)²\n")
            f.write(f"\nPINN Average MSE: {np.mean(pinn_mse):.6f} (N⋅m)²\n")
        
        f.write("\nHyperparameters:\n")
        for param, value in params.items():
            f.write(f"{param}: {value}\n")

if __name__ == "__main__":
    # Launch GUI for hyperparameter selection
    gui = HyperparameterGUI()
    params = gui.get_params()
    
    if params:
        print("Starting training with parameters:", params)
        train_and_evaluate(params)
    else:
        print("Training cancelled") 
import torch
import torch.nn as nn
import numpy as np
import pandas as pd
from torch.utils.data import Dataset, DataLoader
from torch.optim.lr_scheduler import ReduceLROnPlateau
import matplotlib.pyplot as plt
from datetime import datetime
import os
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
from sklearn.metrics import mean_squared_error

# Get absolute paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(PROJECT_ROOT, "data")
MODELS_DIR = os.path.join(SCRIPT_DIR, "models")

# Create directories if they don't exist
os.makedirs(DATA_DIR, exist_ok=True)
os.makedirs(MODELS_DIR, exist_ok=True)

# Constants from URDF
L1, L2, L3 = 0.15, 0.15, 0.10  # Link lengths
m1, m2, m3 = 0.6, 0.4, 0.3    # Link masses
I1 = (1/3) * m1 * L1**2  # Moment of inertia for link 1
I2 = (1/3) * m2 * L2**2  # Moment of inertia for link 2
I3 = (1/3) * m3 * L3**2  # Moment of inertia for link 3
g = 9.81  # Gravity constant

class RobotDataset(Dataset):
    def __init__(self, csv_file, forward_only=True, sequence_length=10):
        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"Data file not found: {csv_file}")
            
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

class InertialResidualNN(nn.Module):
    """Neural network to learn residual terms in the inertia matrix"""
    def __init__(self, hidden_size=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(3, hidden_size),
            nn.LayerNorm(hidden_size),  # Add normalization
            nn.Tanh(),
            nn.Dropout(0.1),  # Add dropout
            nn.Linear(hidden_size, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Dropout(0.1),
            nn.Linear(hidden_size, hidden_size),  # Add one more layer
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, 9)  # Full 3x3 matrix residual
        )
        
        # Initialize weights with smaller values
        for m in self.net.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight, gain=0.1)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
    
    def forward(self, q):
        residual = self.net(q).reshape(-1, 3, 3)
        return residual

class CoriolisResidualNN(nn.Module):
    """Neural network to learn residual terms in the Coriolis matrix"""
    def __init__(self, hidden_size=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(6, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Dropout(0.1),
            nn.Linear(hidden_size, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Dropout(0.1),
            nn.Linear(hidden_size, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, 9)
        )
        
        # Initialize weights with smaller values
        for m in self.net.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight, gain=0.1)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
    
    def forward(self, q, qdot):
        x = torch.cat([q, qdot], dim=1)
        residual = self.net(x).reshape(-1, 3, 3)
        return residual

class GravityResidualNN(nn.Module):
    """Neural network to learn residual terms in the gravity vector"""
    def __init__(self, hidden_size=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(3, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Dropout(0.1),
            nn.Linear(hidden_size, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Dropout(0.1),
            nn.Linear(hidden_size, hidden_size),
            nn.LayerNorm(hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, 3)
        )
        
        # Initialize weights with smaller values
        for m in self.net.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight, gain=0.1)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
    
    def forward(self, q):
        residual = self.net(q)
        return residual

class E2NN(nn.Module):
    """Equation Embedded Neural Network"""
    def __init__(self, hidden_size=64):
        super().__init__()
        self.inertial_net = InertialResidualNN(hidden_size)
        self.coriolis_net = CoriolisResidualNN(hidden_size)
        self.gravity_net = GravityResidualNN(hidden_size)
        
        # Learnable scaling factors for residuals
        self.inertial_scale = nn.Parameter(torch.tensor(0.1))
        self.coriolis_scale = nn.Parameter(torch.tensor(0.1))
        self.gravity_scale = nn.Parameter(torch.tensor(0.1))
    
    def compute_analytical_inertia(self, q):
        """Compute the analytical inertia matrix"""
        # Handle input dimensions
        if len(q.shape) == 3:  # If input is [batch, seq, features]
            q = q[:, -1, :]  # Take last timestep
        batch_size = q.shape[0]
        
        # Initialize inertia matrix [batch_size, 3, 3]
        M = torch.zeros(batch_size, 3, 3, device=q.device)
        
        # Diagonal terms
        # M11
        M[:, 0, 0] = I1 + I2 + I3 + \
                     m1 * (L1/2)**2 + \
                     m2 * (L1**2 + (L2/2)**2 + 2 * L1 * L2/2 * torch.cos(q[:, 1])) + \
                     m3 * (L1**2 + L2**2 + (L3/2)**2 + \
                          2 * L1 * L2 * torch.cos(q[:, 1]) + \
                          2 * L1 * L3/2 * torch.cos(q[:, 1] + q[:, 2]) + \
                          2 * L2 * L3/2 * torch.cos(q[:, 2]))
        
        # M22
        M[:, 1, 1] = I2 + I3 + \
                     m2 * (L2/2)**2 + \
                     m3 * (L2**2 + (L3/2)**2 + 2 * L2 * L3/2 * torch.cos(q[:, 2]))
        
        # M33
        M[:, 2, 2] = I3 + m3 * (L3/2)**2
        
        # Off-diagonal terms (symmetric)
        # M12 = M21
        M12 = m2 * L1 * L2/2 * torch.cos(q[:, 1]) + \
              m3 * (L1 * L2 * torch.cos(q[:, 1]) + \
                    L1 * L3/2 * torch.cos(q[:, 1] + q[:, 2]))
        M[:, 0, 1] = M12
        M[:, 1, 0] = M12
        
        # M13 = M31
        M13 = m3 * L1 * L3/2 * torch.cos(q[:, 1] + q[:, 2])
        M[:, 0, 2] = M13
        M[:, 2, 0] = M13
        
        # M23 = M32
        M23 = m3 * L2 * L3/2 * torch.cos(q[:, 2])
        M[:, 1, 2] = M23
        M[:, 2, 1] = M23
        
        return M
    
    def compute_analytical_coriolis(self, q, qdot):
        """Compute the analytical Coriolis matrix"""
        # Handle input dimensions
        if len(q.shape) == 3:  # If input is [batch, seq, features]
            q = q[:, -1, :]
            qdot = qdot[:, -1, :]
        batch_size = q.shape[0]
        
        # Initialize Coriolis matrix [batch_size, 3, 3]
        C = torch.zeros(batch_size, 3, 3, device=q.device)
        
        # Compute Christoffel symbols and multiply with velocities
        h = m2 * L1 * L2/2 * torch.sin(q[:, 1]) + \
            m3 * (L1 * L2 * torch.sin(q[:, 1]) + \
                  L1 * L3/2 * torch.sin(q[:, 1] + q[:, 2]))
        
        C[:, 0, 1] = -h * qdot[:, 1]
        C[:, 1, 0] = h * qdot[:, 0]
        
        return C
    
    def compute_analytical_gravity(self, q):
        """Compute the analytical gravity vector"""
        # Handle input dimensions
        if len(q.shape) == 3:  # If input is [batch, seq, features]
            q = q[:, -1, :]
        
        # Initialize gravity vector [batch_size, 3]
        g_vec = torch.zeros(q.shape[0], 3, device=q.device)
        
        # Compute gravitational torques
        g_vec[:, 0] = (m1 * L1/2 + m2 * L1 + m3 * L1) * g * torch.cos(q[:, 0]) + \
                      (m2 * L2/2 + m3 * L2) * g * torch.cos(q[:, 0] + q[:, 1]) + \
                      m3 * L3/2 * g * torch.cos(q[:, 0] + q[:, 1] + q[:, 2])
        
        g_vec[:, 1] = (m2 * L2/2 + m3 * L2) * g * torch.cos(q[:, 0] + q[:, 1]) + \
                      m3 * L3/2 * g * torch.cos(q[:, 0] + q[:, 1] + q[:, 2])
        
        g_vec[:, 2] = m3 * L3/2 * g * torch.cos(q[:, 0] + q[:, 1] + q[:, 2])
        
        return g_vec
    
    def forward(self, q, qdot, qddot):
        # Compute analytical terms
        M_analytical = self.compute_analytical_inertia(q)
        C_analytical = self.compute_analytical_coriolis(q, qdot)
        G_analytical = self.compute_analytical_gravity(q)
        
        # Handle input dimensions for residual networks
        if len(q.shape) == 3:  # If input is [batch, seq, features]
            q_input = q[:, -1, :]
            qdot_input = qdot[:, -1, :]
        else:
            q_input = q
            qdot_input = qdot
        
        # Compute residual terms
        M_residual = self.inertial_net(q_input).reshape(-1, 3, 3)
        C_residual = self.coriolis_net(q_input, qdot_input).reshape(-1, 3, 3)
        G_residual = self.gravity_net(q_input)
        
        # Combine analytical and learned terms with learnable scaling
        M = M_analytical + torch.sigmoid(self.inertial_scale) * M_residual
        C = C_analytical + torch.sigmoid(self.coriolis_scale) * C_residual
        G = G_analytical + torch.sigmoid(self.gravity_scale) * G_residual
        
        # Handle input dimensions for acceleration
        if len(qddot.shape) == 3:  # If input is [batch, seq, features]
            qddot = qddot[:, -1, :]
        
        # Compute torques using the equation of motion
        term1 = torch.bmm(M, qddot.unsqueeze(2)).squeeze(2)
        term2 = torch.bmm(C, qdot_input.unsqueeze(2)).squeeze(2)
        tau = term1 + term2 + G
        
        return tau

class VanillaFNN(nn.Module):
    def __init__(self, hidden_size=64):
        super().__init__()
        # Architecture matching /FNN/train_torque_model.py
        self.net = nn.Sequential(
            nn.Linear(9, hidden_size),
            nn.BatchNorm1d(hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_size, hidden_size),
            nn.BatchNorm1d(hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_size, hidden_size // 2),
            nn.BatchNorm1d(hidden_size // 2),
            nn.ReLU(),
            nn.Linear(hidden_size // 2, 3)
        )
        
        # Initialize weights using Kaiming initialization
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.BatchNorm1d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
    
    def forward(self, q, qdot, qddot):
        # Combine inputs and reshape for batch norm
        x = torch.cat([q, qdot, qddot], dim=-1)
        if len(x.shape) == 3:  # If sequence data, take last timestep
            x = x[:, -1, :]
        return self.net(x)

class VanillaRNN(nn.Module):
    def __init__(self, hidden_size=64, sequence_length=10):
        super().__init__()
        self.sequence_length = sequence_length
        self.hidden_size = hidden_size
        
        # Input preprocessing (from /RNN/train_torque_model.py)
        self.input_net = nn.Sequential(
            nn.Linear(9, hidden_size),
            nn.BatchNorm1d(hidden_size),
            nn.ReLU(),
            nn.Dropout(0.2)
        )
        
        # LSTM layer
        self.lstm = nn.LSTM(
            input_size=hidden_size,
            hidden_size=hidden_size,
            num_layers=1,
            batch_first=True
        )
        
        # Output network
        self.output_net = nn.Sequential(
            nn.Linear(hidden_size, hidden_size // 2),
            nn.BatchNorm1d(hidden_size // 2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_size // 2, 3)
        )
        
        self._init_weights()
    
    def _init_weights(self):
        # Initialize input and output networks
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.kaiming_normal_(m.weight)
                if m.bias is not None:
                    nn.init.zeros_(m.bias)
            elif isinstance(m, nn.BatchNorm1d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
        
        # Initialize LSTM
        for name, param in self.lstm.named_parameters():
            if 'weight_ih' in name:
                nn.init.kaiming_normal_(param)
            elif 'weight_hh' in name:
                nn.init.orthogonal_(param)
            elif 'bias' in name:
                nn.init.zeros_(param)
    
    def forward(self, q, qdot, qddot):
        batch_size = q.shape[0]
        
        # Combine inputs: [batch_size, seq_len, 9]
        x = torch.cat([q, qdot, qddot], dim=-1)
        
        # Process each timestep through input network
        x_reshaped = x.reshape(-1, x.shape[-1])  # [batch_size * seq_len, 9]
        features = self.input_net(x_reshaped)
        features = features.reshape(batch_size, -1, self.hidden_size)
        
        # LSTM forward pass
        h0 = torch.zeros(1, batch_size, self.hidden_size, device=x.device)
        c0 = torch.zeros(1, batch_size, self.hidden_size, device=x.device)
        lstm_out, _ = self.lstm(features, (h0, c0))
        
        # Process final output
        final_out = lstm_out[:, -1, :]  # [batch_size, hidden_size]
        return self.output_net(final_out)

class HyperparameterGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("E2NN Hyperparameters")
        
        # Default values
        self.params = {
            'hidden_size': 64,
            'sequence_length': 10,
            'learning_rate': 0.001,
            'batch_size': 32,
            'epochs': 100,
            'train_split': 0.8
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
        self.file_label = ttk.Label(frame, text="No file selected" if not self.data_file else os.path.basename(self.data_file))
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
            'sequence_length': '(5-20) Length of input sequences',
            'learning_rate': '(0.0001-0.01) Learning rate for optimizer',
            'batch_size': '(16-64) Number of samples per batch',
            'epochs': '(50-500) Number of training epochs',
            'train_split': '(0.6-0.9) Fraction of data for training'
        }
        return help_texts.get(param, '')
    
    def validate_params(self):
        try:
            if not self.data_file:
                raise ValueError("Please select a data file")
                
            params = {}
            for key, entry in self.entries.items():
                value = float(entry.get())
                if key in ['hidden_size', 'sequence_length', 'batch_size', 'epochs']:
                    value = int(value)
                params[key] = value
            
            # Add data file to params
            params['data_file'] = self.data_file
            
            # Validation checks
            if not (32 <= params['hidden_size'] <= 128):
                raise ValueError("Hidden size should be between 32 and 128")
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

def train_model(model, train_loader, test_loader, optimizer, criterion, params):
    train_losses = []
    test_losses = []
    best_test_loss = float('inf')
    best_state = None
    
    # Learning rate scheduler matching original implementations
    scheduler = ReduceLROnPlateau(
        optimizer, mode='min', factor=0.2, patience=5, verbose=True,
        min_lr=1e-6, cooldown=5
    )
    
    for epoch in range(params['epochs']):
        # Training
        model.train()
        train_loss = 0
        
        for X, y in train_loader:
            optimizer.zero_grad()
            
            # Split features
            batch_size, seq_len = X.shape[0], X.shape[1]
            q = X[:, :, :3]
            qdot = X[:, :, 3:6]
            qddot = X[:, :, 6:]
            
            # Forward pass
            y_pred = model(q, qdot, qddot)
            
            # Basic MSE loss without additional regularization
            loss = criterion(y_pred, y)
            
            loss.backward()
            
            # Gradient clipping (value clipping for better stability)
            torch.nn.utils.clip_grad_value_(model.parameters(), 1.0)
            
            optimizer.step()
            train_loss += loss.item()
        
        train_loss /= len(train_loader)
        train_losses.append(train_loss)
        
        # Testing
        model.eval()
        test_loss = 0
        with torch.no_grad():
            for X, y in test_loader:
                batch_size, seq_len = X.shape[0], X.shape[1]
                q = X[:, :, :3]
                qdot = X[:, :, 3:6]
                qddot = X[:, :, 6:]
                
                y_pred = model(q, qdot, qddot)
                test_loss += criterion(y_pred, y).item()
        
        test_loss /= len(test_loader)
        test_losses.append(test_loss)
        
        # Learning rate scheduling
        scheduler.step(test_loss)
        
        if (epoch + 1) % 10 == 0:
            print(f"Epoch {epoch+1}/{params['epochs']}, "
                  f"Train Loss: {train_loss:.6f}, Test Loss: {test_loss:.6f}")
        
        # Save best model state
        if test_loss < best_test_loss:
            best_test_loss = test_loss
            best_state = model.state_dict()
    
    return best_state, train_losses, test_losses

def evaluate_model(model, dataset):
    model.eval()
    device = next(model.parameters()).device
    
    # For RNN, we need to handle the sequence length
    if isinstance(model, VanillaRNN):
        sequence_length = model.sequence_length
        # Adjust predictions array length
        num_predictions = len(dataset) - sequence_length + 1
    else:
        num_predictions = len(dataset)
    
    predictions = np.zeros((num_predictions, 3))
    targets = np.zeros((num_predictions, 3))
    
    with torch.no_grad():
        for i in range(num_predictions):
            if isinstance(model, VanillaRNN):
                # Get sequence data
                X, y = dataset[i]
                X = X.unsqueeze(0).to(device)
                
                # Split features
                q = X[:, :, :3]
                qdot = X[:, :, 3:6]
                qddot = X[:, :, 6:]
                
                # Forward pass
                pred = model(q, qdot, qddot)
                predictions[i] = pred.cpu().numpy()
                targets[i] = y.numpy()
            else:
                # Handle FNN and E2NN
                X, y = dataset[i]
                X = X.unsqueeze(0).to(device)
                
                q = X[:, -1:, :3]
                qdot = X[:, -1:, 3:6]
                qddot = X[:, -1:, 6:]
                
                pred = model(q[:, 0], qdot[:, 0], qddot[:, 0])
                predictions[i] = pred.cpu().numpy()
                targets[i] = y.numpy()
    
    # Calculate MSE for each joint
    mse_per_joint = np.mean((predictions - targets)**2, axis=0)
    avg_mse = np.mean(mse_per_joint)
    
    return predictions, avg_mse

def train_and_evaluate(params):
    # Create output directory with parameters in name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    model_dir = os.path.join(SCRIPT_DIR, f"Trained_models_h{params['hidden_size']}_s{params['sequence_length']}_{timestamp}")
    os.makedirs(model_dir, exist_ok=True)
    
    # Save hyperparameters
    with open(os.path.join(model_dir, 'hyperparameters.json'), 'w') as f:
        json.dump(params, f, indent=4)
    
    # Load and split dataset
    dataset = RobotDataset(params['data_file'], forward_only=True, sequence_length=params['sequence_length'])
    train_size = int(0.8 * len(dataset))
    test_size = len(dataset) - train_size
    train_dataset, test_dataset = torch.utils.data.random_split(dataset, [train_size, test_size])
    
    train_loader = DataLoader(train_dataset, batch_size=params['batch_size'], shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=params['batch_size'], shuffle=False)
    
    # Initialize all models with same hyperparameters
    models = {
        'E2NN': E2NN(hidden_size=params['hidden_size']),
        'VanillaFNN': VanillaFNN(hidden_size=params['hidden_size']),
        'VanillaRNN': VanillaRNN(hidden_size=params['hidden_size'], sequence_length=params['sequence_length'])
    }
    
    # Train all models
    training_histories = {}
    test_losses = {}
    predictions = {}
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    for model_name, model in models.items():
        print(f"\nTraining {model_name}...")
        model = model.to(device)
        optimizer = torch.optim.Adam(model.parameters(), lr=params['learning_rate'])
        criterion = nn.MSELoss()
        
        # Train model
        best_state, train_losses, test_losses_history = train_model(model, train_loader, test_loader, optimizer, criterion, params)
        training_histories[model_name] = {
            'train_loss': train_losses,
            'test_loss': test_losses_history
        }
        
        # Save model
        torch.save(best_state, os.path.join(model_dir, f'{model_name.lower()}_model.pth'))
        
        # Load best model for evaluation
        model.load_state_dict(best_state)
        model.eval()
        
        # Evaluate model
        predictions[model_name], mse = evaluate_model(model, dataset)
        test_losses[model_name] = mse
    
    # Plot training histories
    plt.figure(figsize=(12, 6))
    for model_name, history in training_histories.items():
        plt.plot(history['train_loss'], label=f'{model_name} Train')
        plt.plot(history['test_loss'], label=f'{model_name} Test')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training History Comparison')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(model_dir, 'training_history.png'))
    plt.close()
    
    # Plot predictions comparison
    fig, axes = plt.subplots(3, 1, figsize=(15, 12))
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3']
    
    # Get true torques for the actual prediction range
    if isinstance(model, VanillaRNN):
        true_torques = dataset.tau[model.sequence_length-1:].numpy()
    else:
        true_torques = dataset.tau.numpy()
    
    for i in range(3):
        axes[i].plot(true_torques[:, i], label='True Torque', color='black', linewidth=2)
        
        for model_name, preds in predictions.items():
            mse = np.mean((true_torques[:len(preds), i] - preds[:, i])**2)
            axes[i].plot(preds[:, i], label=f'{model_name} (MSE: {mse:.6f})', alpha=0.7)
        
        axes[i].set_title(f'{joint_names[i]} Torque Predictions')
        axes[i].set_xlabel('Time Step')
        axes[i].set_ylabel('Torque (Nm)')
        axes[i].legend()
        axes[i].grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(model_dir, 'predictions_comparison.png'))
    plt.close()
    
    # Save metrics
    metrics = {}
    for model_name, preds in predictions.items():
        metrics[model_name] = {
            f'joint{i+1}_mse': float(np.mean((true_torques[:len(preds), i] - preds[:, i])**2))
            for i in range(3)
        }
        metrics[model_name]['avg_mse'] = float(np.mean(list(metrics[model_name].values())))
    
    with open(os.path.join(model_dir, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=4)
    
    print("\nTraining completed. Results saved in:", model_dir)
    print("\nTest MSE values:")
    for model_name, model_metrics in metrics.items():
        print(f"\n{model_name}:")
        for metric_name, value in model_metrics.items():
            print(f"  {metric_name}: {value:.6f}")

if __name__ == "__main__":
    # Launch GUI for hyperparameter selection
    gui = HyperparameterGUI()
    params = gui.get_params()
    
    if params:
        print("Starting training with parameters:", params)
        train_and_evaluate(params)
    else:
        print("Training cancelled") 
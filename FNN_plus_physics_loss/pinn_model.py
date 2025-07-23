import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import torch
import torch.nn as nn
from .dynamics import RobotDynamics

class PINN(nn.Module):
    def __init__(self, input_size=9, hidden_size=128, num_layers=4, dropout_rate=0.1):
        """
        Physics-Informed Neural Network for robot dynamics
        Args:
            input_size: Number of input features (positions, velocities, accelerations)
            hidden_size: Number of neurons in hidden layers
            num_layers: Number of hidden layers
            dropout_rate: Dropout rate for regularization
        """
        super(PINN, self).__init__()
        
        # Create robot dynamics model
        # These parameters should be adjusted based on your robot
        self.dynamics = RobotDynamics(
            l1=0.3, l2=0.3, l3=0.2,  # Link lengths in meters
            m1=1.0, m2=1.0, m3=0.5,   # Link masses in kg
            I1=0.1, I2=0.1, I3=0.05   # Link inertias in kg⋅m²
        )
        
        # Neural network layers
        layers = []
        current_size = input_size
        
        # Add hidden layers
        for _ in range(num_layers):
            layers.extend([
                nn.Linear(current_size, hidden_size),
                nn.ReLU(),
                nn.BatchNorm1d(hidden_size),
                nn.Dropout(dropout_rate)
            ])
            current_size = hidden_size
        
        # Output layer (3 torques)
        layers.append(nn.Linear(hidden_size, 3))
        
        self.network = nn.Sequential(*layers)
        
        # Initialize weights using Xavier initialization
        self.apply(self._init_weights)
    
    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            nn.init.xavier_normal_(module.weight)
            if module.bias is not None:
                nn.init.zeros_(module.bias)
    
    def forward(self, x):
        """
        Forward pass through the network
        Args:
            x: Input tensor [batch_size, 9] containing:
               - Joint positions [θ₁, θ₂, θ₃]
               - Joint velocities [θ̇₁, θ̇₂, θ̇₃]
               - Joint accelerations [θ̈₁, θ̈₂, θ̈₃]
        Returns:
            Predicted joint torques [τ₁, τ₂, τ₃]
        """
        return self.network(x)
    
    def compute_losses(self, x, y_true, physics_weight=0.1):
        """
        Compute both data loss and physics-based loss
        Args:
            x: Input tensor [batch_size, 9]
            y_true: True torque values [batch_size, 3]
            physics_weight: Weight for physics loss term (λ)
        Returns:
            total_loss: Combined loss
            data_loss: MSE between predicted and true torques
            physics_loss: Physics constraint violation
        """
        # Split input into positions, velocities, and accelerations
        theta = x[:, :3]
        theta_dot = x[:, 3:6]
        theta_ddot = x[:, 6:]
        
        # Forward pass to get predicted torques
        y_pred = self(x)
        
        # Compute data loss (MSE)
        data_loss = torch.mean((y_pred - y_true)**2)
        
        # Compute physics loss
        phys_loss = self.dynamics.physics_loss(theta, theta_dot, theta_ddot, y_pred)
        
        # Combine losses
        total_loss = data_loss + physics_weight * phys_loss
        
        return total_loss, data_loss, phys_loss
    
    def predict_torques(self, theta, theta_dot, theta_ddot):
        """
        Predict torques for given joint states
        Args:
            theta: Joint positions [batch_size, 3]
            theta_dot: Joint velocities [batch_size, 3]
            theta_ddot: Joint accelerations [batch_size, 3]
        Returns:
            Predicted torques [batch_size, 3]
        """
        x = torch.cat([theta, theta_dot, theta_ddot], dim=1)
        return self(x)
    
    def get_physics_violation(self, x):
        """
        Compute the physics constraint violation for given inputs
        Args:
            x: Input tensor [batch_size, 9]
        Returns:
            Physics constraint violation norm
        """
        theta = x[:, :3]
        theta_dot = x[:, 3:6]
        theta_ddot = x[:, 6:]
        y_pred = self(x)
        
        actual_torques = self.dynamics.forward_dynamics(theta, theta_dot, theta_ddot)
        violation = torch.norm(actual_torques - y_pred, dim=1)
        return violation 
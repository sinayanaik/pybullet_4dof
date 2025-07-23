import torch
import numpy as np

class RobotDynamics:
    def __init__(self, l1, l2, l3, m1, m2, m3, I1, I2, I3, g=9.81):
        """
        Initialize robot parameters
        l1, l2, l3: Link lengths
        m1, m2, m3: Link masses
        I1, I2, I3: Link inertias
        g: Gravitational acceleration
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.m1 = m1
        self.m2 = m2
        self.m3 = m3
        self.I1 = I1
        self.I2 = I2
        self.I3 = I3
        self.g = g

    def mass_matrix(self, theta):
        """Compute the mass-inertia matrix M(θ)"""
        t1, t2, t3 = theta[:, 0], theta[:, 1], theta[:, 2]
        
        # Pre-compute trigonometric terms
        c2 = torch.cos(t2)
        c3 = torch.cos(t3)
        c23 = torch.cos(t2 + t3)
        
        # Mass matrix elements
        m11 = (self.l1**2 * (0.25*self.m1 + self.m2 + self.m3) + 
               self.l2**2 * (0.25*self.m2 + self.m3) + 
               0.25*self.l3**2*self.m3 + 
               self.l1*self.l2*c2*(self.m2 + 2*self.m3) +
               self.l2*self.l3*self.m3*c3 + 
               self.l1*self.l3*self.m3*c23 + 
               self.I1 + self.I2 + self.I3)
        
        m12 = (self.l2**2*(0.25*self.m2 + self.m3) + 
               0.25*self.l3**2*self.m3 +
               self.l1*self.l2*c2*(self.m3 + 0.5*self.m2) +
               self.l2*self.l3*self.m3*c3 +
               0.5*self.l1*self.l3*self.m3*c23 +
               self.I2 + self.I3)
        
        m13 = (0.25*self.l3**2*self.m3 +
               0.5*self.l2*self.l3*self.m3*c3 +
               0.5*self.l1*self.l3*self.m3*c23 +
               self.I3)
        
        m22 = (self.l2**2*(0.25*self.m2 + self.m3) +
               0.25*self.l3**2*self.m3 +
               self.l2*self.l3*self.m3*c3 +
               self.I2 + self.I3)
        
        m23 = (0.25*self.l3**2*self.m3 +
               0.5*self.l2*self.l3*self.m3*c3 +
               self.I3)
        
        m33 = 0.25*self.l3**2*self.m3 + self.I3
        
        # Build the mass matrix for batch processing
        M = torch.zeros(theta.shape[0], 3, 3, device=theta.device)
        M[:, 0, 0] = m11
        M[:, 0, 1] = m12
        M[:, 0, 2] = m13
        M[:, 1, 0] = m12
        M[:, 1, 1] = m22
        M[:, 1, 2] = m23
        M[:, 2, 0] = m13
        M[:, 2, 1] = m23
        M[:, 2, 2] = m33
        
        return M

    def coriolis_matrix(self, theta, theta_dot):
        """Compute the Coriolis and centrifugal forces C(θ,θ̇)"""
        t1, t2, t3 = theta[:, 0], theta[:, 1], theta[:, 2]
        td1, td2, td3 = theta_dot[:, 0], theta_dot[:, 1], theta_dot[:, 2]
        
        # Pre-compute trigonometric terms
        s2 = torch.sin(t2)
        s3 = torch.sin(t3)
        s23 = torch.sin(t2 + t3)
        
        # Coriolis terms
        C1 = (td2**2 * (-0.5*self.l1*self.l2*self.m2*s2 - self.l1*self.l2*self.m3*s2 - 
                        0.5*self.l1*self.l3*self.m3*s23) +
              td3**2 * (-0.5*self.l2*self.l3*self.m3*s3 - 0.5*self.l1*self.l3*self.m3*s23) +
              td1*td2 * (-self.l1*self.l2*self.m2*s2 - 2*self.l1*self.l2*self.m3*s2 - 
                         self.l1*self.l3*self.m3*s23) +
              td2*td3 * (-self.l1*self.l3*self.m3*s23 - self.l2*self.l3*self.m3*s3) +
              td1*td3 * (-self.l2*self.l3*self.m3*s3 - self.l1*self.l3*self.m3*s23))
        
        C2 = (td1**2 * (0.5*self.l1*self.l2*self.m2*s2 + self.l1*self.l2*self.m3*s2 +
                        0.5*self.l1*self.l3*self.m3*s23) +
              td3**2 * (-0.5*self.l2*self.l3*self.m3*s3) +
              td1*td3 * (-self.l2*self.l3*self.m3*s3) +
              td2*td3 * (-self.l2*self.l3*self.m3*s3))
        
        C3 = (td1**2 * (0.5*self.l2*self.l3*self.m3*s3 + 0.5*self.l1*self.l3*self.m3*s23) +
              td2**2 * (0.5*self.l2*self.l3*self.m3*s3) +
              td1*td2 * (self.l2*self.l3*self.m3*s3))
        
        return torch.stack([C1, C2, C3], dim=1)

    def gravity_vector(self, theta):
        """Compute the gravity vector G(θ)"""
        t1, t2, t3 = theta[:, 0], theta[:, 1], theta[:, 2]
        
        # Pre-compute trigonometric terms
        c1 = torch.cos(t1)
        c12 = torch.cos(t1 + t2)
        c123 = torch.cos(t1 + t2 + t3)
        
        # Gravity terms
        G1 = (self.m3*self.g*(self.l1*c1 + 0.5*self.l3*c123 + self.l2*c12) +
              self.m2*self.g*(self.l1*c1 + 0.5*self.l2*c12) +
              0.5*self.m1*self.g*self.l1*c1)
        
        G2 = (self.m3*self.g*(0.5*self.l3*c123 + self.l2*c12) +
              0.5*self.m2*self.g*self.l2*c12)
        
        G3 = 0.5*self.m3*self.g*self.l3*c123
        
        return torch.stack([G1, G2, G3], dim=1)

    def forward_dynamics(self, theta, theta_dot, theta_ddot):
        """
        Compute the required torques given joint positions, velocities, and accelerations
        Returns: τ = M(θ)θ̈ + C(θ,θ̇) + G(θ)
        """
        M = self.mass_matrix(theta)
        C = self.coriolis_matrix(theta, theta_dot)
        G = self.gravity_vector(theta)
        
        # Compute torques (batch matrix multiplication)
        torques = torch.bmm(M, theta_ddot.unsqueeze(2)).squeeze(2) + C + G
        return torques

    def inverse_dynamics(self, theta, theta_dot, torques):
        """
        Compute joint accelerations given positions, velocities, and torques
        Returns: θ̈ = M⁻¹(θ)(τ - C(θ,θ̇) - G(θ))
        """
        M = self.mass_matrix(theta)
        C = self.coriolis_matrix(theta, theta_dot)
        G = self.gravity_vector(theta)
        
        # Solve for accelerations
        b = torques - C - G
        theta_ddot = torch.linalg.solve(M, b.unsqueeze(2)).squeeze(2)
        return theta_ddot

    def physics_loss(self, theta, theta_dot, theta_ddot, predicted_torques):
        """
        Compute the physics-based loss term
        Loss = ||M(θ)θ̈ + C(θ,θ̇) + G(θ) - τ_predicted||₂²
        """
        actual_torques = self.forward_dynamics(theta, theta_dot, theta_ddot)
        return torch.mean((actual_torques - predicted_torques)**2) 
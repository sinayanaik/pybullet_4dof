# Physics-Informed Neural Network for 3-DOF Robot Dynamics

This implementation combines a feedforward neural network with physics-based constraints for predicting joint torques in a 3-DOF robotic arm.

## Mathematical Model

### Loss Function
The total loss function combines data-driven loss with physics-based constraints:

```math
Loss_{PINN} = \frac{1}{N} \sum_{j}^{N} ||f(x_j|\theta) - y_j||_2^2 + \lambda \frac{1}{M} \sum_{i}^{M} ||g(x_i, f(x_i,|\theta))||_2^2
```

where:
- First term: Data loss (MSE between predicted and actual torques)
- Second term: Physics loss (violation of dynamic constraints)
- λ: Weighting factor for physics loss
- N: Number of data samples
- M: Number of physics constraint evaluation points
- θ: Neural network parameters
- f(x|θ): Neural network prediction
- g(x, f): Physics constraints from robot dynamics

### Robot Dynamics
The 3-DOF robotic arm dynamics are described by the following equation:

```math
M(θ)\ddot{θ} + C(θ,\dot{θ}) + G(θ) = τ
```

where:
- M(θ): 3×3 mass-inertia matrix
- C(θ,θ̇): Centrifugal and Coriolis forces
- G(θ): Gravitational forces
- τ: Joint torques
- θ: Joint angles
- θ̇: Joint velocities
- θ̈: Joint accelerations

#### Mass-Inertia Matrix Elements
The complete mass-inertia matrix M is symmetric, with elements:

```math
m_{11} = l_1^2(0.25m_1 + m_2 + m_3) + l_2^2(0.25m_2 + m_3) + 0.25l_3^2m_3 + l_1l_2 \cos(θ_2)(m_2 + 2m_3) + l_2l_3m_3 \cos(θ_3) + l_1l_3m_3 \cos(θ_2 + θ_3) + I_1 + I_2 + I_3
```

```math
m_{12} = l_2^2(0.25m_2 + m_3) + 0.25l_3^2m_3 + l_1l_2 \cos(θ_2)(m_3 + 0.5m_2) + l_2l_3m_3 \cos(θ_3) + 0.5l_1l_3m_3 \cos(θ_2 + θ_3) + I_2 + I_3
```

```math
m_{13} = 0.25l_3^2m_3 + 0.5l_2l_3m_3 \cos(θ_3) + 0.5l_1l_3m_3 \cos(θ_2 + θ_3) + I_3
```

```math
m_{21} = m_{12}  \text{ (symmetric)}
```

```math
m_{22} = l_2^2(0.25m_2 + m_3) + 0.25l_3^2m_3 + l_2l_3m_3 \cos(θ_3) + I_2 + I_3
```

```math
m_{23} = 0.25l_3^2m_3 + 0.5l_2l_3m_3 \cos(θ_3) + I_3
```

```math
m_{31} = m_{13}  \text{ (symmetric)}
```

```math
m_{32} = m_{23}  \text{ (symmetric)}
```

```math
m_{33} = 0.25l_3^2m_3 + I_3
```

#### Centrifugal and Coriolis Forces
The complete Coriolis and centrifugal terms are:

```math
C_1(θ,\dot{θ}) = (\dot{θ_2})^2(-0.5l_1l_2m_2 \sin(θ_2) - l_1l_2m_3 \sin(θ_2) - 0.5l_1l_3m_3 \sin(θ_2 + θ_3)) + (\dot{θ_3})^2(-0.5l_2l_3m_3 \sin(θ_3) - 0.5l_1l_3m_3 \sin(θ_2 + θ_3)) + \dot{θ_1}\dot{θ_2}(-l_1l_2m_2 \sin(θ_2) - 2l_1l_2m_3 \sin(θ_2) - l_1l_3m_3 \sin(θ_2 + θ_3)) + \dot{θ_2}\dot{θ_3}(-l_1l_3m_3 \sin(θ_2 + θ_3) - l_2l_3m_3 \sin(θ_3)) + \dot{θ_1}\dot{θ_3}(-l_2l_3m_3 \sin(θ_3) - l_1l_3m_3 \sin(θ_2 + θ_3))
```

```math
C_2(θ,\dot{θ}) = (\dot{θ_1})^2(0.5l_1l_2m_2 \sin(θ_2) + l_1l_2m_3 \sin(θ_2) + 0.5l_1l_3m_3 \sin(θ_2 + θ_3)) + (\dot{θ_3})^2(-0.5l_2l_3m_3 \sin(θ_3)) + \dot{θ_1}\dot{θ_3}(-l_2l_3m_3 \sin(θ_3)) + \dot{θ_2}\dot{θ_3}(-l_2l_3m_3 \sin(θ_3))
```

```math
C_3(θ,\dot{θ}) = (\dot{θ_1})^2(0.5l_2l_3m_3 \sin(θ_3) + 0.5l_1l_3m_3 \sin(θ_2 + θ_3)) + (\dot{θ_2})^2(0.5l_2l_3m_3 \sin(θ_3)) + \dot{θ_1}\dot{θ_2}(l_2l_3m_3 \sin(θ_3))
```

#### Gravitational Forces
The complete gravitational terms are:

```math
G_1(θ) = m_3g(l_1 \cos(θ_1) + 0.5l_3 \cos(θ_1 + θ_2 + θ_3) + l_2 \cos(θ_1 + θ_2)) + m_2g(l_1 \cos(θ_1) + 0.5l_2 \cos(θ_1 + θ_2)) + 0.5m_1gl_1 \cos(θ_1)
```

```math
G_2(θ) = m_3g(0.5l_3 \cos(θ_1 + θ_2 + θ_3) + l_2 \cos(θ_1 + θ_2)) + 0.5m_2gl_2 \cos(θ_1 + θ_2)
```

```math
G_3(θ) = 0.5m_3gl_3 \cos(θ_1 + θ_2 + θ_3)
```

## Implementation Details

### Network Architecture
- Input Layer: Joint positions, velocities, and accelerations (9 features)
- Hidden Layers: Multiple fully connected layers with ReLU activation
- Output Layer: 3 nodes (predicted torques for each joint)

### Physics Loss Computation
1. Forward pass to predict torques
2. Compute mass matrix M(θ)
3. Compute Coriolis terms C(θ,θ̇)
4. Compute gravitational terms G(θ)
5. Calculate residual: ||M(θ)θ̈ + C(θ,θ̇) + G(θ) - τ_predicted||₂²

### Training Process
1. Sample batch of data points
2. Compute data loss using MSE
3. Compute physics loss using dynamics equations
4. Combine losses with weighting factor λ
5. Update network parameters using Adam optimizer

## Usage
```python
python3 train_pinn.py --epochs 1000 --batch_size 32 --learning_rate 0.001 --physics_weight 0.1
```

## Files
- `train_pinn.py`: Main training script
- `pinn_model.py`: Neural network architecture and physics loss implementation
- `dynamics.py`: Robot dynamics computations
- `predict.py`: Script for making predictions with trained model 
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
The elements of the mass-inertia matrix M are:

```math
m_{11} = l_1^2(0.25m_1 + m_2 + m_3) + l_2^2(0.25m_2 + m_3) + 0.25l_3^2m_3 + l_1l_2 \cos(θ_2)(m_2 + 2m_3) + l_2l_3m_3 \cos(θ_3) + l_1l_3m_3 \cos(θ_2 + θ_3) + I_1 + I_2 + I_3
```

[Additional matrix elements m12 through m33 as shown in equations (3)-(10)]

#### Centrifugal and Coriolis Forces
```math
C_1(θ,\dot{θ}) = (\dot{θ_2})^2(-0.5l_1l_2m_2 \sin(θ_2) - l_1l_2m_3 \sin(θ_2) - 0.5l_1l_3m_3 \sin(θ_2 + θ_3)) + ...
```

[Additional C2 and C3 terms as shown in equations (12)-(13)]

#### Gravitational Forces
```math
G_1(θ) = m_3g(l_1 \cos(θ_1) + 0.5l_3 \cos(θ_1 + θ_2 + θ_3) + l_2 \cos(θ_1 + θ_2)) + ...
```

[Additional G2 and G3 terms as shown in equations (14)-(16)]

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

## Setup and Dependencies

```bash
# Install required packages
pip3 install torch numpy pandas matplotlib scikit-learn
```

## Usage

### Training
```bash
python3 train_pinn.py --epochs 1000 --batch_size 32 --learning_rate 0.001 --physics_weight 0.1 --data_file path/to/your/data.csv
```

### Making Predictions
```bash
python3 predict.py --model_dir models/pinn_model_TIMESTAMP --test_file path/to/test_data.csv
```

### Command Line Arguments

Training arguments:
- `--data_file`: Path to training data CSV file (required)
- `--epochs`: Number of training epochs (default: 1000)
- `--batch_size`: Batch size for training (default: 32)
- `--learning_rate`: Learning rate for optimizer (default: 0.001)
- `--hidden_size`: Number of neurons in hidden layers (default: 128)
- `--num_layers`: Number of hidden layers (default: 4)
- `--dropout_rate`: Dropout rate for regularization (default: 0.1)
- `--physics_weight`: Weight for physics loss term λ (default: 0.1)
- `--print_every`: Print progress every N epochs (default: 10)

Prediction arguments:
- `--model_dir`: Directory containing the trained model (required)
- `--test_file`: Path to test data CSV file (required)
- `--output_dir`: Directory to save prediction results (default: 'predictions')

## Files
- `train_pinn.py`: Main training script
- `pinn_model.py`: Neural network architecture and physics loss implementation
- `dynamics.py`: Robot dynamics computations
- `predict.py`: Script for making predictions with trained model 
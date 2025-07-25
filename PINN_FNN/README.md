# Physics-Informed Neural Network (PINN) for 3-DOF Robot Dynamics

This implementation uses a Physics-Informed Neural Network to learn the dynamics model of a 3-DOF robotic arm. The model combines data-driven learning with physics-based constraints derived from the robot's equations of motion.

## Model Architecture

The PINN model consists of:
1. A feedforward neural network that takes joint angles, velocities, and accelerations as input and predicts joint torques
2. A physics-based loss function that enforces the Euler-Lagrange equations of motion
3. A combined loss function that balances between data fitting and physics constraints

### Network Structure
- Input: 9 features (3 angles, 3 velocities, 3 accelerations)
- Hidden layers: 3 layers with 128 units each and ReLU activation
- Output: 3 torques

### Loss Functions
1. **Data Loss**: MSE between predicted and actual torques
2. **Physics Loss**: MSE between predicted torques and physics-based torque calculations
3. **Total Loss**: Weighted sum of data loss and physics loss

## Files

- `train_model.py`: Main training script
- `predict_torque.py`: Script for testing the trained model
- `README.md`: This documentation

## Usage

1. **Training the Model**
   ```bash
   python3 train_model.py
   ```
   This will:
   - Load the trajectory data
   - Train the PINN model
   - Save the model and training history
   - Generate training loss plots

2. **Testing the Model**
   ```bash
   python3 predict_torque.py
   ```
   This will:
   - Load the latest trained model
   - Make predictions on the test data
   - Calculate error metrics
   - Generate prediction plots

## Model Details

### Physics Components

The physics loss incorporates:
1. Gravitational terms for each link
2. Inertial terms from the mass matrix
3. Simplified dynamics (excluding Coriolis and centrifugal terms)

### Hyperparameters

- Learning rate: 0.001
- Batch size: 32
- Number of epochs: 100
- Physics loss weight: 0.1

## Output Files

The training process creates a timestamped directory containing:
- `best_model.pth`: Trained model weights
- `training_history.npy`: Training loss history
- `training_history.png`: Plot of training losses

The testing process generates:
- `prediction_results.png`: Comparison plots of true vs predicted torques

## Dependencies

- PyTorch
- NumPy
- Pandas
- Matplotlib 
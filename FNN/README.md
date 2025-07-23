# Feedforward Neural Network for Robot Joint Torque Prediction

This implementation provides a neural network model to predict joint torques from joint angles in the 4-DOF robot arm. The model learns the inverse dynamics mapping from joint angles to required torques for trajectory tracking.

## Model Architecture

### Network Structure
- **Input Layer**: 3 neurons (θ₁, θ₂, θ₃)
  - Takes joint angles in radians
  - Input is standardized (zero mean, unit variance)

- **Hidden Layers**: 
  - Layer 1: 64 neurons with ReLU activation
  - Layer 2: 64 neurons with ReLU activation
  - Fully connected architecture

- **Output Layer**: 3 neurons (τ₁, τ₂, τ₃)
  - Predicts torques for all three joints
  - Output is in Newton-meters (Nm)
  - Values are destandardized before return

### Training Configuration
- **Loss Function**: Mean Squared Error (MSE)
- **Optimizer**: Adam
  - Learning Rate: 0.005
  - Beta1: 0.9 (default)
  - Beta2: 0.999 (default)
- **Batch Size**: 32
- **Epochs**: 1000
- **Early Stopping**: None (fixed epochs)
- **Progress Monitoring**: Every 100 epochs

## Implementation Files

### 1. Training Script (`train_torque_model.py`)
```python
# Example usage
python train_torque_model.py
```
Key components:
- Data loading and preprocessing
- Model definition and training loop
- Real-time loss monitoring
- Model and scaler saving
- Performance visualization

### 2. Prediction Script (`predict_torque.py`)
```python
# Example usage
python predict_torque.py
```
Features:
- Model loading utilities
- Prediction functions
- Example inference code
- Batch prediction support

### 3. Model File (`torque_model.pth`)
Saved components:
- Neural network weights and biases
- Input scaler parameters
- Output scaler parameters
- Model architecture information

### 4. Visualization Outputs
- `training_loss.png`: 
  - Shows loss convergence over epochs
  - Helps identify training stability
  - Log scale for better detail

- `prediction_results.png`:
  - Three subplots (one per joint)
  - Actual vs predicted torques
  - MSE metrics for each joint
  - Time-series visualization

## Data Processing Pipeline

### Input Processing
1. Load joint trajectory data
2. Extract forward semi-circle motion
3. Standardize joint angles:
   ```python
   X_scaled = scaler.fit_transform(joint_angles)
   ```

### Output Processing
1. Extract joint torques
2. Standardize torque values
3. Inverse transform predictions:
   ```python
   torques = scaler.inverse_transform(predictions)
   ```

### Data Split
- Uses only forward trajectory
- No explicit train/test split
- Real-time validation through visualization

## Performance Metrics

### Training Metrics
- Loss convergence rate
- Final MSE value
- Training time per epoch

### Prediction Metrics
- Per-joint MSE
- Visual alignment quality
- Prediction time

## Usage Instructions

### 1. Installation
```bash
# Create virtual environment (recommended)
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or
.\venv\Scripts\activate  # Windows

# Install dependencies
pip install torch numpy pandas matplotlib scikit-learn
```

### 2. Training
```bash
# Run training
python train_torque_model.py

# Monitor output
- Watch for loss convergence
- Check generated plots
- Verify model saving
```

### 3. Prediction
```bash
# Single prediction
python predict_torque.py

# Custom angles
angles = np.array([0.5, 0.3, 0.2])
model.predict(angles)
```

### 4. Customization
- Adjust hyperparameters in `train_torque_model.py`:
  ```python
  hidden_size = 64  # Network width
  num_epochs = 500  # Training duration
  learning_rate = 0.005  # Optimization speed
  batch_size = 32  # Training batch size
  ```

## Dependencies

### Required Packages
- PyTorch >= 1.9.0
- NumPy >= 1.19.2
- Pandas >= 1.3.0
- Matplotlib >= 3.3.4
- scikit-learn >= 0.24.2

### Optional Tools
- Jupyter Notebook (for experimentation)
- TensorBoard (for extended monitoring)
- CUDA (for GPU acceleration)

## Troubleshooting

### Common Issues
1. **Loss not converging**
   - Increase learning rate
   - Add more epochs
   - Check data normalization

2. **Overfitting**
   - Reduce network size
   - Add dropout layers
   - Implement early stopping

3. **Poor Predictions**
   - Verify data quality
   - Check scaling parameters
   - Ensure angle ranges match training

## Future Improvements

### Planned Features
1. Cross-validation support
2. Uncertainty estimation
3. Real-time prediction API
4. Extended architecture options

### Optimization Ideas
1. Hyperparameter tuning
2. Architecture search
3. Ensemble methods
4. Online learning support 
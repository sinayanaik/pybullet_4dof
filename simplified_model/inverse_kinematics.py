import numpy as np
from scipy.optimize import minimize

def inverse_kinematics(robot, target_x, target_z):
    """
    Solve inverse kinematics to reach target position
    Returns: joint angles [θ2, θ3, θ4]
    """
    def objective(theta):
        # Get end effector position for given joint angles
        points = robot.forward_kinematics(theta)
        end_x, end_z = points[0, -1], points[1, -1]
        
        # Calculate distance to target
        return np.sqrt((end_x - target_x)**2 + (end_z - target_z)**2)
    
    # Initial guess (vertical configuration)
    theta0 = [0, 0, 0]
    
    # Joint limits from robot
    bounds = robot.joint_limits
    
    # Solve optimization problem
    result = minimize(objective, theta0, method='SLSQP', bounds=bounds)
    
    if result.success:
        return result.x
    else:
        raise ValueError("Could not find valid joint angles for target position") 
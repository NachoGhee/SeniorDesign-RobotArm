import numpy as np
from ik_geo import Robot

# --- 1. POE Matrix Setup ---
h = np.array([
    [0.0, 1.0, 0.0],  # J1
    [1.0, 0.0, 0.0],  # J2
    [1.0, 0.0, 0.0],  # J3
    [0.0, 0.0, 1.0],  # J4
    [1.0, 0.0, 0.0],  # J5
    [0.0, 0.0, 1.0]   # J6
])

# Converted from mm to meters to match the controller translation speeds
p = np.array([
    [0.0, 0.0,      0.0],       # Base to J1
    [0.0, 0.095825, 0.0],       # J1 to J2
    [0.0, 0.0,      0.1778],    # J2 to J3
    [0.0, 0.0,      0.20066],   # J3 to J4 (SolidWorks CAD)
    [0.0, 0.0,      0.1650],    # J4 to J5
    [0.0, 0.0,      0.069527],  # J5 to J6
    [0.0, 0.0,      0.0]        # J6 to Tool
])

try:
    pose_arm = Robot.gen_six_dof(h, p)
    MODEL_READY = True
except Exception as e:
    pose_arm = None
    MODEL_READY = False

class RobotModelNotConfigured(Exception):
    pass

# --- 2. Controller Configuration ---
# Define Cartesian workspace limits in meters [min, max] for X, Y, Z
CARTESIAN_WORKSPACE_BOUNDS_M = [
    [-0.6, 0.6],  # X axis limits (left/right)
    [-0.6, 0.6],  # Y axis limits (forward/back)
    [0.0, 0.8]    # Z axis limits (up/down - prevents hitting the floor)
]

# --- 3. Wrapper Functions ---
def model_configuration_errors():
    """Returns a list of setup errors, or an empty list if ready."""
    if not MODEL_READY:
        return ["IK-Geo POE solver failed to initialize."]
    return []

def encoder_degrees_to_model_radians(angles_deg):
    """Converts ESP32 degrees to solver radians, applying DH offsets."""
    angles_rad = np.radians(angles_deg)
    # Apply the +90 degree offset to J3 (index 2) from your DH table
    angles_rad[2] += np.pi / 2
    return angles_rad

def model_radians_to_encoder_degrees(angles_rad):
    """Converts solver radians back to ESP32 degrees, removing DH offsets."""
    angles_rad_copy = np.copy(angles_rad)
    # Remove the +90 degree offset from J3 (index 2)
    angles_rad_copy[2] -= np.pi / 2
    return np.degrees(angles_rad_copy)

def forward_camera_pose(seed_angles_rad):
    """Calculates the current 4x4 Cartesian transformation matrix."""
    if not MODEL_READY:
        raise RobotModelNotConfigured("Robot model not initialized.")
    
    R, t = pose_arm.forward_kinematics(seed_angles_rad)
    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = t
    return pose

def solve_camera_pose(target_pose, seed_angles_rad):
    """Calculates the closest valid joint angles for a target Cartesian pose."""
    if not MODEL_READY:
        raise RobotModelNotConfigured("Robot model not initialized.")
    
    R_target = target_pose[:3, :3]
    t_target = target_pose[:3, 3]
    
    # Get all possible solutions for the target pose
    solutions = pose_arm.get_ik(R_target, t_target)
    
    if len(solutions) == 0:
        raise ValueError("Target coordinate is outside of the arm's physical reach.")
        
    # Select the solution requiring the least joint movement
    best_solution = None
    min_distance = float('inf')
    for sol in solutions:
        dist = np.linalg.norm(sol - seed_angles_rad)
        if dist < min_distance:
            min_distance = dist
            best_solution = sol
            
    return best_solution
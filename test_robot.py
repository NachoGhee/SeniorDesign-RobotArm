import numpy as np
from ik_geo import Robot

# 1. Define dummy h (6 axes of rotation, 6x3 matrix)
h = np.zeros((6, 3))
h[:, 2] = 1.0  # Set all dummy axes to Z

# 2. Define dummy p (7 link offset vectors, 7x3 matrix)
p = np.zeros((7, 3))
p[1] = [0.0, 0.0, 95.825] # Dummy J1 to J2 offset

print("--- Testing Robot Instantiation ---")
try:
    # Use the specific factory method for a general 6-DOF arm
    robot = Robot.gen_six_dof(h, p)
    print("Success! The POSE arm matrices were accepted by the gen_six_dof solver.")
    
    # Let's see what happens if we ask it to solve for a dummy target
    # Forward kinematics target frame (R=identity, t=dummy point)
    R_target = np.eye(3)
    t_target = np.array([100.0, 100.0, 100.0])
    
    solutions = robot.get_ik(R_target, t_target)
    print(f"Solver ran successfully! Found {len(solutions)} valid joint configurations.")
    
except Exception as e:
    print(f"Initialization Failed: {e}")
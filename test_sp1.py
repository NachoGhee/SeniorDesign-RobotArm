import numpy as np
import ik_geo
import math

# Point directly to the compiled Rust extension submodule
ig = ik_geo.ik_geo
# POSE Arm Denavit-Hartenberg Parameters 
# Format: [a (length), alpha (twist), d (offset), theta_offset]
dh_params = [
    [0.0,   math.radians(-90), 95.825, 0.0],              # J1: Turret Base
    [177.8, 0.0,               0.0,    0.0],              # J2: Shoulder (177.8mm humerus)
    [0.0,   math.radians(90),  0.0,    math.radians(90)], # J3: Elbow (+90 deg offset)
    [0.0,   math.radians(90),  349.15, 0.0],              # J4: Wrist Rotation
    [0.0,   math.radians(-90), 0.0,    0.0],              # J5: Wrist Flexion
    [0.0,   0.0,               69.527, 0.0]               # J6: Secondary Wrist
]

# --- Subproblem 1 Test (J1 Turret Rotation) ---
# Equation: R_k(theta) * p1 = p2
# Solves for theta: the angle required to rotate point p1 to p2 around axis k.

# 1. Define the rotation axis (k). J1 rotates entirely around the base Z-axis.
k = np.array([0.0, 0.0, 1.0])

# 2. Define an initial coordinate (p1) extending out along the X-axis.
p1 = np.array([100.0, 0.0, 0.0])

# 3. Define the target coordinate (p2) rotated exactly 90 degrees onto the Y-axis.
p2 = np.array([0.0, 100.0, 0.0])

# 4. Execute the ik-geo Rust solver for Subproblem 1
solution_exists, theta = ig.sp1(p1, p2, k)

print("--- POSE Arm: Subproblem 1 Kinematics Test ---")
print(f"Kinematic Axis (k): {k}")
print(f"Start Vector (p1):  {p1}")
print(f"Target Vector (p2): {p2}")

if solution_exists:
    # Convert the resulting radians back to degrees for easy reading
    print(f"\nSolver Success! Calculated J1 Turret Angle: {math.degrees(theta):.2f}°")
else:
    print("\nSolver Failed: No valid rotation exists between these vectors.")
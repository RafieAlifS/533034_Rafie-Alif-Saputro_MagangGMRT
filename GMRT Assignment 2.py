import numpy as np

def forward_kinematics(t1, t2, l1, l2):
    theta1 = np.deg2rad(t1)
    theta2 = np.deg2rad(t2)
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

def inverse_kinematics(t1, t2, l1, l2):
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    if abs(cos_theta2) > 1:
        raise ValueError("Target is out of reach")
    
    k1a = l1 + l2 * np.cos(theta2a)
    k2a = l2 * np.sin(theta2a)
    theta1a = np.arctan2(y, x) - np.arctan2(k2a, k1a)

    k1b = l1 + l2 * np.cos(theta2b)
    k2b = l2 * np.sin(theta2b)
    theta1b = np.arctan2(y, x) - np.arctan2(k2b, k1b)

    sol1 = (np.rad2deg(theta1a), np.rad2deg(theta2a))
    sol2 = (np.rad2deg(theta1b), np.rad2deg(theta2b))
    return sol1, sol2

a1 = 34
a2 = 62
angle1 = 40
angle2 = 30

x, y = forward_kinematics(angle1, angle2, a1, a2)
print(f"End effector position: x = {x:.2f}, y = {y:.2f}")

solutions = inverse_kinematics(angle1, angle2, a1, a2)
print("IK solutions (theta1, theta2):")
print(f"Elbow-up:   {solutions[0]}")
print(f"Elbow-down: {solutions[1]}")

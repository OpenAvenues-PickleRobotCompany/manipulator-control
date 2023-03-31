import numpy as np
import matplotlib.pyplot as plt

# Robot Parameters
l1 = 1
l2 = 1

# Target Position
x_target = 2
y_target = 1

# Initial Joint Angles
q0 = np.array([0, 0])

# Joint Limits
q_min = np.array([-np.pi/2, -np.pi/2])
q_max = np.array([np.pi/2, np.pi/2])

# Cost Function
def cost(q):
    # Forward Kinematics
    x = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1])
    y = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1])
    
    # Distance to Target
    dist = np.sqrt((x - x_target)**2 + (y - y_target)**2)
    
    return dist

# Constraint Function
def constraint(q):
    # Joint Limits
    return np.concatenate((q - q_min, q_max - q))

# Optimization
q_star = q0.copy()
alpha = 0.1
tol = 1e-3
while True:
    # Compute Gradient
    grad = np.zeros(2)
    for i in range(2):
        dq = np.zeros(2)
        dq[i] = tol
        grad[i] = (cost(q_star + dq) - cost(q_star - dq)) / (2*tol)
        
    # Update Joint Angles
    q_star = np.clip(q_star - alpha * grad, q_min, q_max)
    
    # Check for Convergence
    if np.linalg.norm(grad) < tol:
        break

# Plot Results
x_star = l1 * np.cos(q_star[0]) + l2 * np.cos(q_star[0] + q_star[1])
y_star = l1 * np.sin(q_star[0]) + l2 * np.sin(q_star[0] + q_star[1])
plt.plot([0, l1*np.cos(q_star[0]), x_star], [0, l1*np.sin(q_star[0]), y_star], '-o')
plt.plot(x_target, y_target, 'rx')
plt.axis('equal')
plt.show()
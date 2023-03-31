import math as m
def forward_kinematics_2R(theta1, theta2, L1, L2):
    x = L1*m.cos(theta1) + L2*m.cos(theta1+theta2)
    y = L1*m.sin(theta1) + L2*m.sin(theta1+theta2)
    phi = theta1 + theta2
    return x, y, phi

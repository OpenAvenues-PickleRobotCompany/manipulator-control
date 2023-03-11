import math as m
def inverse_kinematics_2R(x, y, L1, L2):
    # calculate the distance from the origin to the end effector
    r = m.sqrt(x**2 + y**2)

    # calculate the angle theta2
    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_theta2 = m.sqrt(1 - (cos_theta2)**2)
    theta2 = m.atan2(sin_theta2, cos_theta2)

    # calculate the angle theta1
    alpha = m.atan2(y, x)
    beta = m.atan2(L2 * sin_theta2, L1 + L2 * cos_theta2)
    theta1 = alpha - beta
    return theta1, theta2
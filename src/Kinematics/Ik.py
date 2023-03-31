import math as m

def inverse_kinematics_2R(x, y, L1, L2):
    """
    Computes the inverse kinematics of a 2R manipulator given the end effector position (x, y)
    and the lengths of the two links L1 and L2.

    Arguments:
    x -- the x-coordinate of the end effector position
    y -- the y-coordinate of the end effector position
    L1 -- the length of the first link
    L2 -- the length of the second link

    Returns:
    A tuple (theta1, theta2) containing the angles between the x-axis and the first and second links,
    The angles are returned in the range [0, 2*PI).
    """
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

    # normalize the angles to [0, 2*PI)
    theta1 = theta1 % (2 * m.pi)
    theta2 = theta2 % (2 * m.pi)

    return theta1, theta2

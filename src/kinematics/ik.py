import numpy as np
import math

#Inverse kinematics: given the end effector position, lengths, and alpha (theta1+theta2), calculate theta1, theta2 which satisfies it
class inverse_kinematics_planar:
    def __init__(self, end_effector_pos:float, l1:float, l2:float):
        self.x = end_effector_pos[0]
        self.y = end_effector_pos[1] 
        self.l1 = l1
        self.l2 = l2 

    def compute_angles(self):
        # Check if the desired end-effector position is within the robot's workspace
        eps = 1e-6
        if (self.x ** 2 + self.y ** 2) > (self.l1 + self.l2) ** 2 + eps:
            raise ValueError("Desired end-effector position is outside the robot's workspace")

        # Compute the argument of acos()
        arg = (self.x ** 2 + self.y ** 2 - self.l2 ** 2 - self.l1 ** 2) / (2 * self.l1 * self.l2)

        # Clipping the values for cosine to avoid math domain errors
        # Check if the argument is within the valid range
        if arg > 1.0:
            arg = 1.0
        elif arg < -1.0:
            arg = -1.0

        # Compute theta2 using acos()
        theta2 = math.acos(arg)

        # Compute theta1 using atan2()
        k1 = self.l1 + self.l2 * math.cos(theta2)
        k2 = self.l2 * math.sin(theta2)
        theta1 = math.atan2(self.y, self.x) - math.atan2(k2, k1)

        return theta1, theta2


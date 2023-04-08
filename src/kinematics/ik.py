import numpy as np
import math

#Inverse kinematics: given the end effector position, lengths, and alpha (theta1+theta2), calculate theta1, theta2 which satisfies it
class inverse_kinematics_planar:
    def __init__(self, end_effector_pos:float, l1:float, l2:float):
        self.x = end_effector_pos[0]
        self.z = end_effector_pos[2] 
    
        self.l1 = l1
        self.l2 = l2 

    def compute_angles(self):
        # Check if the desired end-effector position is within the robot's workspace
        eps = 1e-6
        if (self.x ** 2 + self.z ** 2) > (self.l1 + self.l2) ** 2 + eps:
            raise ValueError("Desired end-effector position is outside the robot's workspace")

        # Calculate the elbow-up angles
        d = (self.x ** 2 + self.z ** 2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2 + eps)
        theta2 = np.arctan2(np.sqrt(1 - d ** 2), d)
        theta1 = np.arctan2(self.z, self.x) - np.arctan2(self.l2 * np.sin(theta2), self.l1 + self.l2 * np.cos(theta2) + eps)

        return theta1, theta2



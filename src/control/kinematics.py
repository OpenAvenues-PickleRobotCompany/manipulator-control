import numpy as np
import math

#Forward Kinematics: given theta1, theta2, l1, and l2 for a 2-link planar arm, determine the end effector position
class forward_kinematics_planar:
    def __init__(self, theta1: float, theta2: float,  
                l1:float, l2: float):
        self.theta1=theta1
        self.theta2=theta2

        self.l1=l1
        self.l2=l2

    #Computes positions relative to joint 0 position (x0,y0)
    def compute_positions(self):
        
        x1 = self.l1*np.cos(self.theta1)
        y1 = self.l1*np.sin(self.theta1)
        joint1_pos = (x1,y1,0) 
        
        x2 = x1 + self.l2*np.cos(self.theta1+self.theta2)
        y2 = y1 + self.l2*np.sin(self.theta1+self.theta2)
        end_effector_pos = (x2,y2,0)
        
        return joint1_pos, end_effector_pos


#Inverse kinematics: given the end effector position, lengths, and alpha (theta1+theta2), calculate theta1, theta2 which satisfies it
class inverse_kinematics_planar:
    def __init__(self, end_effector_pos:float, l1:float, l2:float):
        self.x = end_effector_pos[0]
        self.y = end_effector_pos[1] 
        self.l1 = l1
        self.l2 = l2 
    
# Computes angles to make the end effector position possible
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
        numerator = self.y * self.l1 - self.x * self.l2 * np.sin(theta2)
        denominator = self.x * self.l2 * np.cos(theta2) + self.y * self.l1
        theta1 = math.atan2(numerator, denominator)

        return theta1, theta2

        
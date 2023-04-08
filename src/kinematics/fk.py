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

    #Computes positions relative to joint 0 position 
    def compute_positions(self):
        
        x1 = self.l1*np.cos(self.theta1)
        z1 = self.l1*np.sin(self.theta1)
        joint1_pos = (x1,0,z1) 
        
        x2 = x1 + self.l2*np.cos(self.theta1+self.theta2)
        z2 = z1 + self.l2*np.sin(self.theta1+self.theta2)
        end_effector_pos = (x2,0,z2)

        return joint1_pos, end_effector_pos


        
from enum import Enum
import matplotlib.pyplot as plt 
import numpy as np
import math

class DiscretizationMethod(Enum):
    EULER_FORWARD = 'euler_forward'
    EULER_BACKWARD = 'euler_backward'


class P:
    def __init__(self, kp: float):
        self.kp = kp
        
    def compute_command(self, desired_state: float, current_state: float):
        return self.kp * (desired_state - current_state)


class PID:
    def __init__(self, kp: float, kd: float, ki: float, ts: float,discretization_method: DiscretizationMethod):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ts = ts
        self.discretization_method = discretization_method
        
        self.saturation_limits = (-1000,1000)
        
        self.previous_error = 0  
        self.integral_error = 0 


    def compute_command(self, desired_state: float, current_state: float):
        
        error = desired_state - current_state #current error
        
        self.integral_error += error * self.ts #riennman approximation, error*ts = rectangle. smaller ts:closer to continuous time 
            
        deriv_error = (error-self.previous_error) / self.ts 
        
        command = self.kp*error + self.kd*deriv_error + self.ki*self.integral_error
        
        #Antiwindup: If the actuator is saturated (reached its physical limit), the error will keep accumulating. 
        #Prevent this by adjusting command if it hits a saturation limit
        #saturation_limits = (lower_limit, upper_limit)
        if command < self.saturation_limits[0]:
            command = self.saturation_limits[0]
            
        if command > self.saturation_limits[1]:
            command = self.saturation_limits[1] 
        
        self.previous_error = error
        
        return command
    
#Forward Kinematics: given theta1, theta2, l1, and l2 for a 2-link planar arm, determine the end effector position
class forward_kinematics_planar:
    def __init__(self, theta1: float, theta2: float,  
                 l1:float, l2: float):
        
        self.theta1=theta1
        self.theta2=theta2
   
        self.l1=l1
        self.l2=l2
 
    #computes positions relative to joint 0 position (x0,y0)
    def compute_positions(self):
        
        x1 = self.l1*np.cos(self.theta1)
        y1 = self.l1*np.sin(self.theta1)
        joint1_pos = (x1,y1) 
        
        x2 = x1 + self.l2*np.cos(self.theta1+self.theta2)
        y2 = y1 + self.l2*np.sin(self.theta1+self.theta2)
        end_effector_pos = (x2,y2)
        
        return joint1_pos, end_effector_pos
    

#Inverse kinematics: given the end effector position, lengths, and alpha (theta1+theta2), calculate theta1, theta2 which satisfies it
class inverse_kinematics_planar:
    def __init__(self, end_effector_pos:float, l1:float, l2:float, 
                alpha:float):
    
        self.x = end_effector_pos[0]
        self.y = end_effector_pos[1] 
        self.l1 = l1
        self.l2 = l2 
        self.alpha = alpha 
        
    def compute_angles(self): 
        theta2 = math.acos(self.x**2 + self.y**2 - self.l2**2 - self.l1**2 / (2*self.l1*self.l2))
        theta1 = self.alpha - theta2 
        
        return theta1, theta2 
            

    
    
    
if __name__ == '__main__':
    pass

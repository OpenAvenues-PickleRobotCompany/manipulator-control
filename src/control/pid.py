from enum import Enum
import matplotlib.pyplot as plt 
import numpy as np

class DiscretizationMethod(Enum):
    EULER_FORWARD = 'euler_forward'
    EULER_BACKWARD = 'euler_backward' #implicit

class P:
    def __init__(self, kp: float):
        self.kp = kp
        
    def discretize(self):
        pass

    def compute_command(self, desired_state: float, current_state: float):
        return self.kp * (desired_state - current_state)


class PID:
    def __init__(self, kp: float, kd: float, ki: float, ts: float, discretization_method: DiscretizationMethod):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ts = ts
        self.discretization_method = discretization_method

        self.discretize()
        
        self.previous_error = 0  
        self.integral_error = 0 


    def discretize(self):
        if self.discretization_method == DiscretizationMethod.EULER_FORWARD:
            self.kd = self.kd / self.ts
            self.ki = self.ki * self.ts 
            
        
        if self.discretization_method == DiscretizationMethod.EULER_BACKWARD:
            pass
           
    def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state
        
        deriv_error = (error-self.previous_error)/self.ts
        
        self.integral_error += error * self.ts
        
        self.previous_error = error
        
        command = self.kp*error + self.kd*deriv_error + self.ki*self.integral_error

        return command
    
    
    

#using image on slide27, project session 2 as reference
class forward_kinematics_planar:
    def __init__(self, theta1: float, theta2: float, theta3: float, 
                 l1:float, l2: float, l3: float):
        
        self.theta1=theta1
        self.theta2=theta2
        self.theta3=theta3
        
        self.l1=l1
        self.l2=l2
        self.l3=l3
        
        #joint 0=base
        self.joint1_pos=None 
        self.joint2_pos=None
        self.end_effector_pos=None 
    
        self.compute_positions()
        
    #computes positions relative to joint 0 position (x0,y0)
    def compute_positions(self):
        
        x1=self.l1*np.cos(self.theta1)
        y1=self.l1*np.sin(self.theta1)
        self.joint1_pos=(x1,y1) 
        
        x2 = x1 + self.l2*np.cos(self.theta1+self.theta2)
        y2 = y1 + self.l2*np.sin(self.theta1+self.theta2)
        self.joint2_pos=(x2,y2) 
        
        x3 = x2 + self.l3 * np.cos(self.theta1 + self.theta2 + self.theta3)
        y3 = y2 + self.l3 * np.sin(self.theta1 + self.theta2 + self.theta3)
        self.end_effector_pos = (x3,y3) 
        
    
    
    
if __name__ == '__main__':
    pass

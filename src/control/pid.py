from enum import Enum
import matplotlib.pyplot as plt 
import numpy as np
import math

class DiscretizationMethod(Enum):
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
        self.i = 0  #integral error
        self.T_t = .05 #tracking time, how fast integral term will be reset


def compute_command(self, desired_state: float, current_state: float):
    
    error = desired_state - current_state #current error
    
    p = self.kp*error #proportional error
    
    self.i += self.ts * (self.ki*error) #integral term 

    d = (error - self.previous_error) / self.ts #derivative of the error (euler backward: using previous error)
    
    command = p + self.i + d 
    
    if command < self.saturation_limits[0]:
        saturated_command = self.saturation_limits[0]
        e_p = saturated_command - command
        self.i += self.ts * ( self.ki*error + (1/self.T_t)*e_p ) #integral term with backcalculation 
        command = p + self.i + d
        
    if command > self.saturation_limits[1]:
        saturated_command = self.saturation_limits[1] 
        e_p = saturated_command - command
        self.i += self.ts * ( self.ki*error + (1/self.T_t)*e_p ) #integral term with backcalculation 
        command = p + self.i + d
        
    self.previous_error = error
    
    
        
    return command
    

    
if __name__ == '__main__':
    pass

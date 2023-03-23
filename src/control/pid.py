from enum import Enum
import matplotlib.pyplot as plt 
import numpy as np
import math

#test

class P:
    def __init__(self, kp: float):
        self.kp = kp
        
    def compute_command(self, desired_state: float, current_state: float):
        return self.kp * (desired_state - current_state)

class PID:
    def __init__(self, kp: float, kd: float, ki: float, ts: float, sat_limits: tuple):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ts = ts
        
        self.saturation_limits = sat_limits

        self.i=0
        self.previous_error = 0  
        self.T_t = .05 #tracking time, how fast integral term will be reset


    def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state #current error
        
        #proportional error
        p = self.kp*error 

        #integral term 
        self.i += self.ts * (self.ki*error) 
        
        #derivative of the error (euler backward: using previous error)
        d = (error - self.previous_error) / self.ts 
    
        command = p + self.i + d 
            
        saturated_command = max(self.saturation_limits[0], min(command, self.saturation_limits[1]))
        e_p = saturated_command - command
        self.i += ((self.ts /self.T_t)*e_p)
        command = p + self.i + d

        self.previous_error = error

        return command
        

 
from enum import Enum
from Kinematics.Ik import *

class P:
    def __init__(self, kp: float):
        self.kp = kp

    def discretize(self):
        pass

    def compute_command(self, desired_state: float, current_state: float):
        return self.kp * (desired_state - current_state)


class PID:
    def __init__(self, kp: float, ki: float, kd: float, ts: float, max_output=100, min_output=-100):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ts = ts
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0.
        self.last_error = 0.
        self.last_output = 0.
        self.T_t = 1        

    def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state
        ep = desired_state - self.last_output
        self.integral += error * self.ts + (1/self.T_t)*ep
        derivative = (error - self.last_error) / self.ts

        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * derivative

        command = p + i + d
        if command > self.max_output:
            aw_term = self.kp * (command - self.max_output)
            self.integral -= aw_term / self.ki + (1/self.T_t) * ep
            p = self.kp * error
            i = self.ki * self.integral
            d = self.kd * derivative
            command = p + i + d
            if command > self.max_output:
                command = self.max_output

        elif command < self.min_output:
            aw_term = self.kp * (command - self.min_output)
            self.integral -= (aw_term / self.ki) + (1/self.T_t) * ep
            p = self.kp * error
            i = self.ki * self.integral
            d = self.kd * derivative
            command = p + i + d
            if command < self.min_output:
                command = self.min_output

        self.last_error = error

        return command



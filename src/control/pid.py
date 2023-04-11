from enum import Enum

class P:
    def __init__(self, kp: float):
        self.kp = kp

    def discretize(self):
        pass

    def compute_command(self, desired_state: float, current_state: float):
        return self.kp * (desired_state - current_state)
class PID:
    def __init__(self, kp: float, ki: float, kd: float, ts: float, max_output=1000, min_output=-1000):
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
        integral = self.integral + error * self.ts
        derivative = (error - self.last_error) / self.ts

        p = self.kp * error
        i = self.ki * integral
        d = self.kd * derivative

        command = p + i + d
        if command > self.max_output:
            ep = self.max_output - command
            integral += 1/self.T_t *self.ts * ep
            command = p + integral + d
            if command > self.max_output:
                command = self.max_output
        elif command < self.min_output:
            ep = self.min_output - command
            integral += 1/self.T_t *self.ts * ep
            command = p + integral + d
            if command < self.min_output:
                command = self.min_output

        self.last_error = error
        self.integral = integral
        return command


from enum import Enum


class DiscretizationMethod(Enum):
    EULER_FORWARD = 'euler_forward'
    # EULER_IMPLICIT = 'euler_implicit'

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
        self.last_error = 0  #initialize previous error to approximate the error derivative later
        self.integral_error = 0 #initialize this to zero so it accumulates (riemann)


    def discretize(self):
        if self.discretization_method == DiscretizationMethod.EULER_FORWARD:
            self.kp = self.kp / self.ts
            self.kd = self.kd / self.ts
            self.ki = self.ki * self.ts #area approximation by multiplying
           
    def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state
        deriv_error = (error-self.last_error)/self.ts
        self.integral_error += error * self.ts
       
        self.last_error = error


        command = self.kp*error + self.kd*deriv_error + self.ki*self.integral_error
       
        return command




if __name__ == '__main__':
    pass

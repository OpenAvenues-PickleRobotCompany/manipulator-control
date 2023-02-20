from enum import Enum
import math as m

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
    def __init__(self, kp: float, ki: float, kd: float, ts: float, discretization_method: DiscretizationMethod):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ts = ts
        self.max_output = 100
        self.min_output = -100
        self.discretization_method = discretization_method

        self.discretize()

    def discretize(self):
        kp_d = self.kp
        ki_d = self.ki * self.ts
        kd_d = self.kd / self.ts
        pid_d = PID (kp_d, ki_d, kd_d, self.ts)
        return pid_d

    def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state
        p = self.kp * error
        d = self.kd * (error-self.last_error) / self.ts
        i = self.ki * (error+self.last_error)/2 * self.ts
        #antiwind?
        if i > self.max_output:
            i = self.max_output
        elif i < self.min_output:
            i = self.min_output

        self.last_error = error
        return p+d+i


if __name__ == '__main__':
    pass


def forward_kinematics_2R(theta1, theta2, L1, L2):
    x = L1*m.cos(theta1) + L2*m.cos(theta1+theta2)
    y = L1*m.sin(theta1) + L2*m.sin(theta1+theta2)
    phi = theta1 + theta2
    return x, y, phi
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
        self.integral = 0
        self.last_error = 0
        self.discretization_method = discretization_method
        self.Discretization_method()

    def Discretization_method(self):
        self.kp_d = self.kp * self.ts
        self.ki_d = self.ki * self.ts
        self.kd_d = self.kd / self.ts
        

    def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state
        self.integral += error * self.ts
        derivative = (error - self.last_error) / self.ts

        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * derivative

        command = p + i + d

        if command > self.max_output:
            command = self.max_output
        elif command < self.min_output:
            command = self.min_output

        self.last_error = error

        return command

if __name__ == '__main__':
    pass


def forward_kinematics_2R(theta1, theta2, L1, L2):
    x = L1*m.cos(theta1) + L2*m.cos(theta1+theta2)
    y = L1*m.sin(theta1) + L2*m.sin(theta1+theta2)
    phi = theta1 + theta2
    return x, y, phi



def inverse_kinematics_2R(x, y, L1, L2):
    # calculate the distance from the origin to the end effector
    r = m.sqrt(x**2 + y**2)

    # calculate the angle theta2
    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    sin_theta2 = m.sqrt(1 - (cos_theta2)**2)
    theta2 = m.atan2(sin_theta2, cos_theta2)

    # calculate the angle theta1
    alpha = m.atan2(y, x)
    beta = m.atan2(L2 * sin_theta2, L1 + L2 * cos_theta2)
    theta1 = alpha - beta
    return theta1, theta2

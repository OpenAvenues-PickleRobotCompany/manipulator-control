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

    def discretize(self):
        # TODO: implement
        pass

    def compute_command(self, desired_state: float, current_state: float):
        error = desired_state - current_state
        p = self.kp * error
        d = self.kd * error / self.ts
        i = self.ki * error * self.ts
        return p+d+i


if __name__ == '__main__':
    pass
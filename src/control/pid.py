from enum import Enum
import math as m
import pybullet as p
import time 
from typing import Final

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
        self.last_output = 0
        self.T_t = 1
        self.discretization_method = discretization_method
        

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
            command = p + self.integral + d
            if command > self.max_output:
                command = self.max_output

        elif command < self.min_output:
            aw_term = self.kp * (command - self.min_output)
            self.integral -= (aw_term / self.ki) + (1/self.T_t) * ep
            command = self.min_output
            if command < self.min_output:
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



GRAVITY: Final = -9.81
physicsClient = p.connect(p.GUI)

start_pos = [0,0,0]
start_orientation = p.getQuaternionFromEuler([0,0,0])

pendulum_id = p.loadURDF("src/robot/double_pendulum.urdf", start_pos, start_orientation, useFixedBase=True)
p.setGravity(0,0,GRAVITY)

theta1 = 0.0
theta2 = 0.0
p.resetJointState(pendulum_id, 0, theta1)
p.resetJointState(pendulum_id, 1, theta2)

goal = [1,2,0]
kp = 0.1  # Proportional 
ki = 0  # Integral 
kd = 0  # Derivative 
ts = 1/240

pid1 = PID(kp, ki, kd, ts, discretization_method=DiscretizationMethod.EULER_FORWARD)
pid2 = PID(kp, ki, kd, ts, discretization_method=DiscretizationMethod.EULER_FORWARD)

L1 = L2 = 2
theta1_goal, theta2_goal = inverse_kinematics_2R(goal[0], goal[1], L1, L2)

for i in range(1000):
    torque1 = pid1.compute_command(theta1_goal, theta1)
    torque2 = pid2.compute_command(theta2_goal, theta2)
    p.setJointMotorControl2(pendulum_id, 0, p.TORQUE_CONTROL, force=torque1)
    p.setJointMotorControl2(pendulum_id, 1, p.TORQUE_CONTROL, force=torque2)
    theta1, theta2 = p.getJointStates(pendulum_id, [0, 1])[0][:2]
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()
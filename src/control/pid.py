from enum import Enum
import pybullet as p
import time 
from ..Kinematics.Ik import inverse_kinematics_2R
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



GRAVITY = -9.81
physicsClient = p.connect(p.GUI)

start_pos = [0, 0, 0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])

pendulum_id = p.loadURDF("src/robot/double_pendulum_with_saturation.urdf", start_pos, start_orientation, useFixedBase=True)

p.setGravity(0, 0, GRAVITY)

theta1 = 0.0
theta2 = 0.0
p.resetJointState(pendulum_id, 0, theta1)
p.resetJointState(pendulum_id, 1, theta2)

goal = [2, 1, 0]
kp = 10  # Proportional 
ki = 0.1  # Integral 
kd = 0.25  # Derivative 
ts = 1/240

pid1 = PID(kp, ki, kd, ts, discretization_method=DiscretizationMethod.EULER_FORWARD)
pid2 = PID(kp, ki, kd, ts, discretization_method=DiscretizationMethod.EULER_FORWARD)

L1 = L2 = 2
theta1_goal, theta2_goal = inverse_kinematics_2R(goal[0],goal[1],L1,L2)

for i in range(50000):
    theta1 = p.getJointState(pendulum_id, 0)[0]
    theta2 = p.getJointState(pendulum_id, 1)[0]
    torque1 = pid1.compute_command(theta1_goal, theta1)
    torque2 = pid2.compute_command(theta2_goal, theta2)
    p.setJointMotorControl2(pendulum_id, 0, p.TORQUE_CONTROL, force=torque1)
    p.setJointMotorControl2(pendulum_id, 1, p.TORQUE_CONTROL, force=torque2)
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
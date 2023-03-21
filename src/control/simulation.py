import pybullet as p
import time 
from pid import *
from Kinematics import * 

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

pid1 = PID(kp, ki, kd, ts)
pid2 = PID(kp, ki, kd, ts)

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
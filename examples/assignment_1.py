import pybullet as p 
import time 
import numpy as np
import sys
import os
from pathlib import Path

# Assuming your script is in the 'examples' folder
sys.path.append(str(Path(__file__).resolve().parent.parent))

from src.control.pid import P, PID
from src.kinematics.fk import forward_kinematics_planar
from src.kinematics.ik import inverse_kinematics_planar

import argparse

parser = argparse.ArgumentParser(description='PyBullet Simulation')
parser.add_argument('x', type=float, help='X coordinate')
parser.add_argument('y', type=float, help='Y coordinate')
args = parser.parse_args()

if not (args.x and args.y):
    parser.error('X and Y coordinates are required.')
    
# 1) Define the desired end effector position.
# 2) Obtain the current theta1, theta2.
# 3) Use inverse kinematics to obtain the desired theta1, theta2.
# 4) Use PID1, PID2 to obtain torque command based on the desired-current theta values.
# 5) Apply this torque command to the body id from URDF.
def main():
    
    #Initializing physics client, gravity
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81) #sets gravity
    
    #Initializing start position and orientation
    start_pos = [0,0,0]
    start_orientation = p.getQuaternionFromEuler([0,0,0]) #converts roll pitch yaw angles of obj to quaternion
    
    pendulum_id = p.loadURDF("../src/urdfs/double_pendulum_with_saturation.urdf",start_pos, start_orientation, useFixedBase=1) #move one directory up
    p.setTimeStep(1./240.) #updates the simulation every 1/240 seconds (240 times per second)

    # Set up PID controllers
    kp = 20
    kd = 10
    ki = 0.5
    ts = 1./240.

    pid1 = PID(kp, kd, ki, ts, (-1000,1000))
    pid2 = PID(kp, kd, ki, ts, (-1000,1000))

    desired_end_effector_pos = [args.x, args.y, 0] # x, y, z position of the end effector
    
    l1 = 2
    l2 = 2
    ik = inverse_kinematics_planar(desired_end_effector_pos, l1, l2)

    tolerance = 0.01
    max_iterations = 1000000  # Maximum number of iterations
    stable_iterations = 0   # Counter for stable end effector iterations
    required_stable_iterations = 100  # Number of required stable iterations to exit the loop
    iteration = 0
   
    
    while iteration < max_iterations:
        # Get joint states
        current_theta1 = p.getJointState(pendulum_id, 0)[0]
        current_theta2 = p.getJointState(pendulum_id, 1)[0]

        # Solve inverse kinematics for desired end effector position. Returns the desired angles at the joints
        desired_theta1, desired_theta2 = ik.compute_angles()

        # Get torque commands from PID controllers
        torque1 = pid1.compute_command(desired_theta1, current_theta1)
        torque2 = pid2.compute_command(desired_theta2, current_theta2)

        # Apply torque to each joint
        p.setJointMotorControl2(pendulum_id, 0, p.TORQUE_CONTROL, force=torque1)
        p.setJointMotorControl2(pendulum_id, 1, p.TORQUE_CONTROL, force=torque2)

        # Check if end effector is within tolerance of desired position
        fk = forward_kinematics_planar(current_theta1, current_theta2, l1, l2)
        end_effector_pos_current = fk.compute_positions()[1]

        error = np.linalg.norm(np.array(desired_end_effector_pos) - np.array(end_effector_pos_current))

        if error <= tolerance:
            stable_iterations += 1
        else:
            stable_iterations = 0

        if stable_iterations >= required_stable_iterations:
            print("DONE")
            break

        iteration += 1
        p.stepSimulation()

        if iteration >= max_iterations:
            print("Simulation reached maximum number of iterations.")

        p.stepSimulation()

    p.disconnect()
    
main()
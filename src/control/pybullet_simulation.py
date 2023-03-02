import pybullet as p 
import time 
from pid import *
from kinematics import * 
import numpy as np

def main():
    
# 1) Define the desired end effector position.
# 2) Obtain the current theta1, theta2.
# 3) Use inverse kinematics to obtain the desired theta1, theta2.
# 4) Use PID1, PID2 to obtain torque command based on the desired-current theta values.
# 5) Apply this torque command to the body id from URDF.
    
    #Initializing physics client, gravity
    p.setGravity(0, 0, -9.81) #sets gravity
    
    #Initializing start position and orientation
    start_pos = [0,0,0]
    start_orientation = p.getQuaternionFromEuler([0,0,0]) #converts roll pitch yaw angles of obj to quaternion
    
    pendulum_id = p.loadURDF("../robot/double_pendulum_with_saturation.urdf",start_pos, start_orientation) #move one directory up
    p.setTimeStep(1./240.) #updates the simulation every 1/240 seconds (240 times per second)

    # Set up PID controllers
    kp = 10
    kd = 0.5
    ki = 0.5
    ts = 1./240.
    discretization_method = DiscretizationMethod.EULER_BACKWARD
    pid1 = PID(kp, kd, ki, ts, discretization_method)
    pid2 = PID(kp, kd, ki, ts, discretization_method)

    end_effector_pos = [0, 1, 0] # x, y, z position of the end effector
    
    l1 = 1 
    l2 = 1  
    ik = inverse_kinematics_planar(end_effector_pos, l1, l2)

    while True:
        # Get joint states
        #getJointState(bodyUniqueID, jointIndex) = [jointPosition (angle), jointVelocity, jointReactionForces, appliedJointMotorTorque]
        current_theta1 = p.getJointState(pendulum_id, 0)[0]
        current_theta2 = p.getJointState(pendulum_id, 1)[0]

        print("CURRENT ANGLES", current_theta1, current_theta2)
        # Solve inverse kinematics for desired end effector position. Returns the desired angles at the joints
        desired_theta1, desired_theta2 = ik.compute_angles()
        
        print("DESIRED ANGLES",desired_theta1,desired_theta2)
        
        # Get torque commands from PID controllers
        torque1 = pid1.compute_command(desired_theta1, current_theta1)
        torque2 = pid2.compute_command(desired_theta2, current_theta2)
        
        # Apply torque to each joint
        #setJointMotorControl2(bodyUniqueId, jointIndex, controlMode=(position,velocity,torque), force(or torque))
        p.setJointMotorControl2(pendulum_id, 0, p.TORQUE_CONTROL, force=torque1)
        p.setJointMotorControl2(pendulum_id, 1, p.TORQUE_CONTROL, force=torque2)
        
        p.stepSimulation()

    p.disconnect()
    
main()
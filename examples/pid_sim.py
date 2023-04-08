import pybullet as p 
import numpy as np
import os
from src.control.pid import P, PID
from src.kinematics.fk import forward_kinematics_planar
from src.kinematics.ik import inverse_kinematics_planar
import argparse

"""
PyBullet Simulation for Double Pendulum Arm 

1) Obtains desired end effector position from cmd line arguments
2Use inverse kinematics to obtain the desired theta1, theta2.
3) Use PID1, PID2 to obtain torque command based on the desired-current theta values.
4) Apply this torque command to the body ID from URDF.

"""

#Obtain desired end effector position
parser = argparse.ArgumentParser(description='PyBullet Simulation')
parser.add_argument('x', type=float, help='X coordinate')
parser.add_argument('z', type=float, help='Z coordinate')
args = parser.parse_args()


# Construct the absolute path to the URDF file
urdf_path = os.path.join('src', 'urdfs', 'double_pendulum_with_saturation.urdf')

def main():

    # Initialize physics client GUI and camera extrinsics
    physicsClient = p.connect(p.GUI)
    add_axes()
    p.resetDebugVisualizerCamera(5, 180, 0, [0,0,0])
    p.setGravity(0, 0, -9.81) 
    
    # Initializing start position and orientation
    start_pos = [0,0,0]
    start_orientation = p.getQuaternionFromEuler([0,0,0]) 
    pendulum_id = p.loadURDF(urdf_path,start_pos, start_orientation, useFixedBase=1)
    p.setTimeStep(1./240) 

    # Set up PID controllers
    kp = 25
    kd = 10
    ki = 3
    ts = 1./240.
    pid1 = PID(kp, kd, ki, ts, (-1000,1000))
    pid2 = PID(kp, kd, ki, ts, (-1000,1000))
    desired_end_effector_pos = [-args.z,0,args.x] 
    (l1, l2) = (3,3)
    
    # Solve inverse kinematics for desired end effector position. Returns the desired angles at the joints
    ik = inverse_kinematics_planar(desired_end_effector_pos, l1, l2)
    desired_theta1, desired_theta2 = ik.compute_angles()

    tolerance = 0.05
    max_iterations = 100000000  
    stable_iterations = 0   
    required_stable_iterations = 100000  
    iteration = 0
    
    label_created = False
    label_update_frequency = 5000
       
    while iteration < max_iterations:
        # Get joint states
        current_theta1 = p.getJointState(pendulum_id, 0)[0]
        current_theta2 = p.getJointState(pendulum_id, 1)[0]

        # Get torque commands from PID controllers
        torque1 = pid1.compute_command(desired_theta1, current_theta1)
        torque2 = pid2.compute_command(desired_theta2, current_theta2)

        # Apply torque to each joint
        p.setJointMotorControl2(pendulum_id, 0, p.TORQUE_CONTROL, force=torque1)
        p.setJointMotorControl2(pendulum_id, 1, p.TORQUE_CONTROL, force=torque2)

        # Check if end effector is within tolerance of desired position
        fk = forward_kinematics_planar(current_theta1, current_theta2, l1, l2)
        end_effector_pos_current = fk.compute_positions()[1]
        
        #Create label according to frequency 
        if iteration % label_update_frequency == 0:
            if label_created:
                p.removeUserDebugItem(desired_label_id)  
                p.removeUserDebugItem(current_label_id) 

            # Desired position label
            desired_label_text = f"Desired: ({desired_end_effector_pos[2]:.2f}, {-desired_end_effector_pos[0]:.2f})"
            desired_label_id = p.addUserDebugText(
                text=desired_label_text,
                textPosition=[-0.5, 0, 2.5],
                textColorRGB=[0, 0, 1],
                textSize=1.5,
            )

            # Current position label
            current_label_text = f"Current: ({end_effector_pos_current[2]:.2f}, {-end_effector_pos_current[0]:.2f})"
            current_label_id = p.addUserDebugText(
                text=current_label_text,
                textPosition=[-0.5, 0, 2],
                textColorRGB=[0, 0, 1],
                textSize=1.5,
            )

            label_created = True

        #Check if error is in acceptable range for required number of stable iterations
        error = np.linalg.norm(np.array(desired_end_effector_pos) - np.array(end_effector_pos_current))

        if error <= tolerance:
            stable_iterations += 1
        else:
            stable_iterations = 0

        if stable_iterations >= required_stable_iterations:
            print("Convergence Reached.")
            break

        iteration += 1

        if iteration >= max_iterations:
            print("Simulation reached maximum number of iterations.")

        p.stepSimulation()
    p.disconnect()
    
    
#Draws axis lines for the 2D view of the arm
def add_axes(length=1.0):
    p.addUserDebugLine([0, 0, 0], [length, 0, 0], [1, 0, 0], lineWidth=2)
    p.addUserDebugLine([0, 0, 0], [0, 0, length], [0, 0, 1], lineWidth=2)
    p.addUserDebugText("-X", [length, 0, 0], [1, 0, 0], textSize=1.5)
    p.addUserDebugText("Z", [0, 0, length], [0, 0, 1], textSize=1.5)
    
main()
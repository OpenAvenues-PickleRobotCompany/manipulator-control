from typing import Final
import pybullet as p
import time
from src.control.pid import *

GRAVITY: Final = -9.81

physics_client = p.connect(p.GUI)


start_pos = [0,0,0]
start_orientation = p.getQuaternionFromEuler([0,0,0])

pendulum_id = p.loadURDF("src/robot/double_pendulum.urdf", start_pos, start_orientation, useFixedBase=True)
p.setGravity(0,0,GRAVITY)

scourse.robot()
t1, t2 = inverse_kinematics_2R(1,1,0,1) 


for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.) # 1/240 is the default timestep of the simulation engine

p.disconnect()



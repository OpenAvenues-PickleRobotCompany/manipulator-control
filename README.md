# manipulator-control

## What I learned

During my first assignment, I learned about three main topics: PID controllers, forward and inverse kinematics, and the PyBullet library.

PID controllers are a way to control the behavior of a system based on feedback. They use a mathematical formula to adjust the input to the system in response to the output. I learned about the three parts of a PID controller: proportional, integral, and derivative, and how to tune them to get the desired behavior. Additionally, I learned about time discretization and anti-windup techniques to improve the performance of the controller.

Forward and inverse kinematics are a way to calculate the position and orientation of a robot's end effector based on the joint angles. I learned how to use trigonometry and geometry to derive the equations needed for these calculations for a planar 2R robot.

PyBullet is a library that can be used to simulate the behavior of a robot in a virtual environment. I learned how to set up a simulation, load a robot model, and control its joints.

## Issues faced

While working on the assignment, I faced several issues.

The first issue was that I found it difficult to understand the theory behind PID controllers and how to apply it in practice. The mathematical formulas and concepts were new to me, and I had to spend a lot of time studying and experimenting to get a handle on them.

The second issue was that I had difficulty understanding the PyBullet library and how to use it to simulate the robot's behavior. The library had many options and settings, and I had to spend time reading documentation and experimenting with different settings to get the simulation to work as intended.

## Issues remaining to be solved

One issue that remains to be solved is the issue of the arm moving together in the simulation. This may be a result of an error in the kinematics calculations or a problem with the simulation settings, and will require further investigation to resolve. 

## Limits of the current work

The current implementation of the PID controller, kinematics, and simulation is limited in scope and functionality. It is designed for a planar 2R robot, and may not be suitable for more complex scenarios or environments. Additionally, the implementation is not fully optimized, and may require further tuning to achieve optimal performance. There may be other approaches or algorithms that could be explored to improve the implementation.

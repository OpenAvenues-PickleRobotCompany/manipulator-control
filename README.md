# Controlling a 2R Robotic Arm 

For this project, a robot manipulator is directed by a PID controller.


## Usage 

To visualize the arm converging on an input end effector position, download all requirements:

```
$ pip install -r requirements.txt
```

and run the code as following:

``` 
python -m examples.pid_sim [x_position] [z_position]
```

If the desired end effector position has been reached for 100 stable iterations, then the desired position has effectively been reached and the program will exit. Otherwise, the program will exit once the maximum number of iterations is reached (1000000). Since the lengths of the links are both 3 units, the desired end effector position can only be reached if it is at most 6 units away from the origin.

## 1) 2R Manipulator
A manipulator with two rotary joints, also known as revolute joints, is referred to as a 2R manipulator. The two revolute joints enable the robot arm to rotate within two degrees of freedom. The base joint rotates around a fixed axis, while the shoulder joint rotates around an axis that is perpendicular to the first joint. These joints allow the robot arm to move in a two-dimensional plane.

<p align="center">
  <img src="https://user-images.githubusercontent.com/71537050/224106200-a9d1e3ca-1321-4dbd-9417-7389fe0f5a11.png">
</p>
<p align="center">
  Diagram from Chakraborty et al. (2009) showing a 2R manipulator.
</p>

One relevant application of a 2R manipulator is in pick-and-place operations. This involves a robot arm being fixed on a base and having an end effector (some sort of gripper or suction cup) to grab ojects. Since this case is limited to two degrees of freedom, it is more well-suited for situations where items are arranged in a plane (such as a conveyor belt or production line). 

## 2) PID Control
PID (Proportional-Integral-Derivative) is a control algorithm commonly used in engineering applications. It is a feedback loop that continuously adjusts an output based on the difference between a desired setpoint and the measured process variable. The PID controller calculates a command output based on three terms: the proportional term (P), the integral term (I), and the derivative term (D). The proportional term is proportional to the error between the desired setpoint and the measured process variable. The integral term accumulates the error over time and compensates for steady-state errors. The derivative term predicts the future trend of the error and compensates for overshooting or oscillations.  

#### 2a) Time Discretization

Time discretization is necessary for a PID controller to ensure that the controller operates at a specific rate, allowing for stable control of a process or system. The implementation uses Euler backward discretization. This means that we compute the derivative of our error term as current error subtracted by the previous error divided by a time step. The integral term, then, is the accumulation of error multiplied by the timestep (to create a Riemann rectangular approximation). Thus, as the timestep decreases towards zero, we expect the system to look continuous rather than discretized. 

#### 2b) Anti-windup 

Antiwindup is a technique used in control systems to prevent integrators from accumulating error inappropriately; in a PID controller, when a control signal reaches its maximum or minimum limit, the integrator continues to integrate the error. To avoid this, an antiwindup scheme is implemented to reset the integrator when the control signal reaches a saturation limit. 

A backcalculation term is used to estimate the integrator's contribution to the output when the controller is saturated. When the command exceeds the saturation limits, the integral error is recomputed. It is proportional to the error difference between the actual and saturated command, and inversely proportional to the tracking time constant.  This tracking time thus determines how long it takes for the integral term to be reset. In this implementation, the tracking time is set to 0.05; so after 0.05 seconds of continuous saturation, the integral term will be reset. This particular approach is described in detail by Silva et al. (2018). 

## 3) PyBullet
Bullet is a physics simulation engine used for simulating physics-based interactions in virtual environments. It is an open-source software developed by Erwin Coumans at Google. Pybullet provides an easy-to-use interface for interacting with Bullet in Python. For this implementation, a 2R arm is simulated with data inputted by URDF files. At each joint, torque commands are generated by two PID controllers and applied to them to drive it towards our desired end effector state.

- Define the desired end effector position.
- Obtain the current theta1, theta2 (getJointState)
- Use inverse kinematics to obtain the desired theta1, theta2.
- Use PID1, PID2 to obtain torque command based on the desired-current theta values.
- Apply this torque command to the body id from URDF (setJointMotorControl)
- Use forward kinematics to calculate the current end effector position and check if it is close to our desired position.

<p align="center">
  <img src="https://user-images.githubusercontent.com/71537050/230735855-d3da3fa2-1a91-4b1d-be30-570f84a5bfe9.png" alt="image">
</p>
<p align="center">
  PyBullet 2R Simulation. Desired End Effector Position: x=3.0, y=0.0, z=1.50
</p>

## Sources

- Chakraborty, Nilanjan & Akella, Srinivas & Trinkle, J.C. (Jeff). (2009). Complementarity-based Dynamic Simulation for Kinodynamic Motion Planning. 2009 IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS 2009. 787 - 794. 10.1109/IROS.2009.5354274. 

- Lucian R. da Silva, Rodolfo C.C. Flesch, Julio E. Normey-Rico (2018). Analysis of Anti-windup Techniques in PID Control of Processes with Measurement Noise⁎⁎This work was supported by the Brazilian National Council for Scientific and Technological Development (CNPq) under Grants 311024/2015-7 and 305785/2015-0., IFAC-PapersOnLine, Volume 51, Issue 4, 2018, Pages 948-953, ISSN 2405-8963, https://doi.org/10.1016/j.ifacol.2018.06.100.


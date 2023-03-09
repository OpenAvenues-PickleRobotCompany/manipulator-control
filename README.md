# manipulator-control

## Controlling a 2R Robotic Arm 

For this project, a robot manipulator is directed by a PID controller.

### 2R Manipulator
A manipulator with two rotary joints, also known as revolute joints, is referred to as a 2R manipulator. The two revolute joints enable the robot arm to rotate within two degrees of freedom. The base joint rotates around a fixed axis, while the shoulder joint rotates around an axis that is perpendicular to the first joint. These joints allow the robot arm to move in a two-dimensional plane.

<p align="center">
  <img src="https://user-images.githubusercontent.com/71537050/224106200-a9d1e3ca-1321-4dbd-9417-7389fe0f5a11.png">
</p>
<p align="center">
  Diagram from Chakraborty et al.(2009) showing a 2R manipulator.
</p>

One relevant application of a 2R manipulator is in pick-and-place operations. This involves a robot arm being fixed on a base and having an end effector (some sort of gripper or suction cup) to grab ojects. Since this case is limited to two degrees of freedom, it is more well-suited for situations where items are arranged in a plane (such as a conveyor belt or production line). 

### PID Control
PID (Proportional-Integral-Derivative) is a control algorithm commonly used in engineering applications. It is a feedback loop that continuously adjusts an output based on the difference between a desired setpoint and the measured process variable. The PID controller calculates a command output based on three terms: the proportional term (P), the integral term (I), and the derivative term (D). The proportional term is proportional to the error between the desired setpoint and the measured process variable. The integral term accumulates the error over time and compensates for steady-state errors. The derivative term predicts the future trend of the error and compensates for overshooting or oscillations.

### Time Discretization



#### Anti-windup Scheme

Antiwindup is a technique used in control systems to prevent integrators from accumulating error inappropriately; in a PID controller, when a control signal reaches its maximum or minimum limit, the integrator continues to integrate the error. To avoid this, an antiwindup scheme is implemented to reset the integrator when the control signal reaches a saturation limit. 

A backcalculation term is used to estimate the integrator's contribution to the output when the controller is saturate. When the command exceeds the saturation limits, the integral error is recomputed. It is proportional to the error difference between the actual and saturated command, and inversely proportional to the tracking time constant.  This tracking time thus determines how long it takes for the integral term to be reset. In this implementation, the tracking time is set to 0.05; so after 0.05 seconds of continuous saturation, the integral term will be reset.







### Sources

- Chakraborty, Nilanjan & Akella, Srinivas & Trinkle, J.C. (Jeff). (2009). Complementarity-based Dynamic Simulation for Kinodynamic Motion Planning. 2009 IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS 2009. 787 - 794. 10.1109/IROS.2009.5354274. 

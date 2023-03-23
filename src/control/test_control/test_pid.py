from src.control.pid import *
import matplotlib.pyplot as plt
import numpy

def test_proportional():
    null_controller = P(kp=0.)
    for i in range(10):
        assert 0. == null_controller.compute_command(i, 0)

    controller = P(kp=1)
    for i in range(10):
        assert 0. == controller.compute_command(i, i)


    for i in range(10):
        assert i == controller.compute_command(i, 0)


    controller = P(kp=2)
    for i in range(10):
        assert 2*i == controller.compute_command(i, 0)


    assert 2 == controller.compute_command(2, 1)


def test_pid():
    pid = PID(kp=1,kd=1,ki=1,ts=0.1)
    
    #expect zero
    command = pid.compute_command(desired_state=0, current_state=0)
    assert 0 == command

   
    #expect nonzero command to drive current state upwards
    command = pid.compute_command(desired_state=1, current_state=0)
    assert command > 0
    




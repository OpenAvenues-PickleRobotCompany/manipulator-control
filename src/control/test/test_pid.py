from src.control.pid import P,PID,forward_kinematics_2R
import math as m

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
    new_controller = PID(kp = 1., ki = 1., kd = 1. , ts = 1.)
    for i in range(10):
        assert i*3 == new_controller.compute_command(i, 0)
    new_controller_1 = PID(kp = 1., ki = 2., kd = 3. , ts = 0.1)
    for i in range(10):
        assert i*(1+2/0.1+3/0.1) == new_controller_1.compute_command(i, 0)

def test_forward_kinematics_2R():
    x,y,phi = forward_kinematics_2R(m.pi/6, m.pi/3, 1, 2)
    x == 1*m.cos(m.pi/6) + 2*m.cos(m.pi/3 + m.pi/6)
    y == 1*m.sin(m.pi/6) + 2*m.sin(m.pi/3 + m.pi/6)
    phi == m.pi/2
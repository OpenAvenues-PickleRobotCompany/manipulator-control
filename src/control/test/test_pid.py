from src.control.pid import *
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
    new_controller = PID(kp=1., ki=2., kd=3., ts=0.1, discretization_method=DiscretizationMethod.EULER_FORWARD)
    sum_error = 0
    last_error = 0
    for i in range(1, 11):
        error = 10 - i
        sum_error += error
        derivative = (error - last_error) / 0.1
        expected_output = 1 * error + 2 * sum_error * 0.1 + 3 * derivative
        if expected_output > new_controller.max_output:
            expected_output = 100
        if expected_output < new_controller.min_output:
            expected_output = -100
        assert expected_output == new_controller.compute_command(10, i)
        last_error = error


def test_forward_kinematics_2R():
    x,y,phi = forward_kinematics_2R(m.pi/6, m.pi/3, 1, 2)
    x == 1*m.cos(m.pi/6) + 2*m.cos(m.pi/3 + m.pi/6)
    y == 1*m.sin(m.pi/6) + 2*m.sin(m.pi/3 + m.pi/6)
    phi == m.pi/2

def test_inverse_kinematics_2R():
    x,y,phi = forward_kinematics_2R(m.pi/6, m.pi/3, 1, 2)
    m.pi/6, m.pi/3 == inverse_kinematics_2R(x, y, 1, 2)

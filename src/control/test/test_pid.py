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
    def test_proportional(self):
        controller = PID(kp=2, ki=0, kd=0, ts=0.1, discretization_method=DiscretizationMethod.EULER_FORWARD)
        for i in range(10):
            assert 2*i == controller.compute_command(i, 0)

    def test_integral(self):
        controller = PID(kp=0, ki=2, kd=0, ts=0.1, discretization_method=DiscretizationMethod.EULER_FORWARD)
        for i in range(10):
            assert i == controller.compute_command(i, 0)

    def test_derivative(self):
        controller = PID(kp=0, ki=0, kd=2, ts=0.1, discretization_method=DiscretizationMethod.EULER_FORWARD)
        assert 20 == controller.compute_command(0, 10)

    def test_integration_limits(self):
        controller = PID(kp=1, ki=2, kd=0, ts=0.1, discretization_method=DiscretizationMethod.EULER_FORWARD)
        assert controller.max_output == controller.compute_command(0, -100)
        assert controller.min_output == controller.compute_command(0, 100)


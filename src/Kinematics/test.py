from src.Kinematics import Fk, Ik
import math as m


def test_forward_kinematics_2R():
    x, y, phi = Fk.forward_kinematics_2R(m.pi/6, m.pi/3, 1, 2)
    assert abs(x - (1*m.cos(m.pi/6) + 2*m.cos(m.pi/3 + m.pi/6))) < 1e-6
    assert abs(y - (1*m.sin(m.pi/6) + 2*m.sin(m.pi/3 + m.pi/6))) < 1e-6
    assert abs(phi - (m.pi/2)) < 1e-6

import random

def test_inverse_kinematics_2R():
    L1, L2 = 1, 2
    for i in range(10):
        # generate random joint angles
        theta1 = random.uniform(-m.pi, m.pi)
        theta2 = random.uniform(-m.pi, m.pi)
        # compute end effector coordinates
        x, y, phi = Fk.forward_kinematics_2R(theta1, theta2, L1, L2)
        # compute joint angles 
        theta1_inv, theta2_inv = Ik.inverse_kinematics_2R(x, y, L1, L2)
        # compute end effector coordinates 
        x_inv, y_inv, phi_inv = Fk.forward_kinematics_2R(theta1_inv, theta2_inv, L1, L2)
        # check that the resulting end effector coordinates match the original ones
        assert abs(x - x_inv) < 1e-6
        assert abs(y - y_inv) < 1e-6
        assert abs(phi - phi_inv) < 1e-6

from Kinematics import forward_kinematics_2R, inverse_kinematics_2R
import math as m

def test_forward_kinematics_2R():
    x,y,phi = forward_kinematics_2R(m.pi/6, m.pi/3, 1, 2)
    x == 1*m.cos(m.pi/6) + 2*m.cos(m.pi/3 + m.pi/6)
    y == 1*m.sin(m.pi/6) + 2*m.sin(m.pi/3 + m.pi/6)
    phi == m.pi/2

def test_inverse_kinematics_2R():
    x,y,phi = forward_kinematics_2R(m.pi/6, m.pi/3, 1, 2)
    m.pi/6, m.pi/3 == inverse_kinematics_2R(x, y, 1, 2)

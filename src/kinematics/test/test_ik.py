from src.kinematics.ik import inverse_kinematics_planar
from src.kinematics.fk import forward_kinematics_planar
import numpy as np

def test_inverse_kinematics_planar():
    # Test 1: When both angles are 0
    ik = inverse_kinematics_planar((2, 0, 0), 1, 1)
    theta1, theta2 = ik.compute_angles()

    # Compute forward kinematics with the returned angles
    fk = forward_kinematics_planar(theta1, theta2, 1, 1)
    _, end_effector_pos = fk.compute_positions()

    # Check if the computed end_effector_pos is close to the desired position
    assert np.allclose(end_effector_pos, (2, 0, 0), rtol=1e-5, atol=1e-5)

    # Test 2: When both angles are 90 degrees
    ik = inverse_kinematics_planar((0, 0, np.sqrt(2)), 1, 1)
    theta1, theta2 = ik.compute_angles()

    # Compute forward kinematics with the returned angles
    fk = forward_kinematics_planar(theta1, theta2, 1, 1)
    _, end_effector_pos = fk.compute_positions()

    # Check if the computed end_effector_pos is close to the desired position
    assert np.allclose(end_effector_pos, (0, 0, np.sqrt(2)), rtol=1e-5, atol=1e-5)

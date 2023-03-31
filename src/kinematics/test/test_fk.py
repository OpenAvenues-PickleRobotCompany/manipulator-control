from src.kinematics.fk import forward_kinematics_planar
import numpy as np 


# fk = (theta1, theta2, l1, l2)
#np.allclose: returns true if arrays are element-wise equal within  a tolerance 
def test_compute_positions():
    # Test 1: When both angles are 0
    fk = forward_kinematics_planar(0, 0, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (1, 0, 0), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (2, 0, 0), rtol=1e-5, atol=1e-5)

    # Test 2: When both angles are 90 degrees
    fk = forward_kinematics_planar(np.pi / 2, np.pi / 2, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (0, 1, 0), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (-1, 1, 0), rtol=1e-5, atol=1e-5)

    # Test 3: When one angle is 90 degrees and the other is -90 degrees
    fk = forward_kinematics_planar(np.pi / 2, -np.pi / 2, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (0, 1, 0), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (1, 1, 0), rtol=1e-5, atol=1e-5)

    # Test 4: When both angles are 180 degrees
    fk = forward_kinematics_planar(np.pi, np.pi, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (-1, 0, 0), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (0, 0, 0), rtol=1e-5, atol=1e-5)

    # Test 5: When theta1 is 45 degrees and theta2 is 45 degrees
    fk = forward_kinematics_planar(np.pi / 4, np.pi / 4, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (0.7071067811865476, 0.7071067811865476, 0), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (0.7071067811865477, 1.7071067811865475, 0), rtol=1e-5, atol=1e-5)

    # Test 6: When theta1 is 135 degrees and theta2 is -135 degrees
    fk = forward_kinematics_planar(3 * np.pi / 4, -3 * np.pi / 4, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (-0.7071067811865476, 0.7071067811865476, 0), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (0.29289321881345254, 0.7071067811865476, 0), rtol=1e-5, atol=1e-5)

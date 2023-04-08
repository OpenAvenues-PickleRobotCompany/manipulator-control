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
    fk = forward_kinematics_planar(np.pi/2, np.pi/2, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (0, 0,1 ), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (-1, 0, 1 ), rtol=1e-5, atol=1e-5)
    
    # Test 3: When theta1 is 45 degrees and theta2 is 30 degrees
    fk = forward_kinematics_planar(np.pi/4, np.pi/6, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (0.70710678, 0, 0.70710678), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (0.9659258263, 0, 1.673032607), rtol=1e-5, atol=1e-5)
    
    # Test 4: When theta1 is 120 degrees and theta2 is 60 degrees
    fk = forward_kinematics_planar(2*np.pi/3, np.pi/3, 1, 1)
    joint1_pos, end_effector_pos = fk.compute_positions()
    assert np.allclose(joint1_pos, (-0.5, 0, 0.8660254), rtol=1e-5, atol=1e-5)
    assert np.allclose(end_effector_pos, (-1.5, 0, 0.8660254038), rtol=1e-5, atol=1e-5)

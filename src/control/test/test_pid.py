from src.control.pid import P

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
    assert False

import numpy as np

from vrso101.robot.joints import load_robot_spec
from vrso101.robot.kinematics import Kinematics

spec = load_robot_spec()
kin = Kinematics(spec)


def test_jacobian_matches_finite_difference():
    q = np.random.default_rng(0).uniform(spec.lower, spec.upper) * 0.6
    J = kin.jacobian(q)
    eps = 1e-6
    for i in range(5):
        dq = np.zeros(6)
        dq[i] = eps
        dp = (kin.fk(q + dq)[:3, 3] - kin.fk(q - dq)[:3, 3]) / (2 * eps)
        np.testing.assert_allclose(J[:3, i], dp, atol=1e-5)


def test_ik_recovers_reachable_pose():
    rng = np.random.default_rng(1)
    ok = 0
    for _ in range(20):
        q_true = rng.uniform(spec.lower, spec.upper) * 0.5
        q_true[5] = 0
        T = kin.fk(q_true)
        res = kin.ik(T, spec.home, max_iter=200)
        if res.converged:
            ok += 1
            assert np.linalg.norm(kin.fk(res.q)[:3, 3] - T[:3, 3]) < 3e-3
            assert spec.within_limits(res.q)
    assert ok >= 14  # 5 自由度かつ局所解があるので全数成功は要求しない


def test_ik_unreachable_reports_not_converged_and_stays_in_limits():
    T = np.eye(4)
    T[:3, 3] = [3.0, 0, 0]
    res = kin.ik(T, spec.home)
    assert not res.converged
    assert spec.within_limits(res.q)

"""The C++ Kalman filter through the bindings, against a NumPy reference of the same equations."""

import math

import numpy as np
import pytest

from aegislink._core import KalmanFilter, MotionModel


def reference_matrices(model, q, dt):
    """F and Q of the model, state ordered x, y, vx, vy (, ax, ay)."""
    k = 2 if model == MotionModel.CONSTANT_VELOCITY else 3
    f1 = np.array([[1, dt, dt * dt / 2], [0, 1, dt], [0, 0, 1]])[:k, :k]
    # White noise in the highest derivative: Q[i][j] = q dt^p / ((k-1-i)! (k-1-j)! p),
    # p = 2k - 1 - i - j.
    q1 = np.array([[q * dt ** (2 * k - 1 - i - j)
                    / (math.factorial(k - 1 - i) * math.factorial(k - 1 - j) * (2 * k - 1 - i - j))
                    for j in range(k)] for i in range(k)])
    order = [d * 2 + a for a in range(2) for d in range(k)]   # per-axis blocks -> state order
    f = np.zeros((2 * k, 2 * k))
    qm = np.zeros((2 * k, 2 * k))
    for a in range(2):
        idx = order[a * k:(a + 1) * k]
        f[np.ix_(idx, idx)] = f1
        qm[np.ix_(idx, idx)] = q1
    return f, qm


@pytest.mark.parametrize("model", [MotionModel.CONSTANT_VELOCITY,
                                   MotionModel.CONSTANT_ACCELERATION])
def test_matches_numpy_reference(model):
    rng = np.random.default_rng(3)
    q = 0.7
    kf = KalmanFilter(model, q)
    kf.init(2.0, -1.0, (0.5, 0.1, 0.3), 4.0, 1.5)
    n = kf.dim
    x = np.zeros(n)
    x[:2] = (2.0, -1.0)
    p = np.zeros((n, n))
    p[:2, :2] = [[0.5, 0.1], [0.1, 0.3]]
    p[2, 2] = p[3, 3] = 16.0
    if n == 6:
        p[4, 4] = p[5, 5] = 1.5 ** 2
    h = np.zeros((2, n))
    h[0, 0] = h[1, 1] = 1
    for _ in range(200):
        dt = rng.uniform(0.0, 0.5)
        kf.predict(dt)
        f, qm = reference_matrices(model, q, dt)
        x = f @ x
        p = f @ p @ f.T + qm
        z = rng.normal(0, 3, 2)
        r = np.array([[rng.uniform(0.1, 2), 0.0], [0.0, rng.uniform(0.1, 2)]])
        r[0, 1] = r[1, 0] = rng.uniform(-0.05, 0.05)
        nis = kf.update(z[0], z[1], (r[0, 0], r[0, 1], r[1, 1]))
        s = h @ p @ h.T + r
        gain = p @ h.T @ np.linalg.inv(s)
        innovation = z - h @ x
        x = x + gain @ innovation
        a = np.eye(n) - gain @ h
        p = a @ p @ a.T + gain @ r @ gain.T
        assert nis == pytest.approx(innovation @ np.linalg.inv(s) @ innovation, rel=1e-9)
        assert np.allclose(kf.state, x, rtol=1e-9, atol=1e-9)
        assert np.allclose(kf.covariance, p, rtol=1e-9, atol=1e-9)
    pxx, pxy, pyy = kf.position_covariance
    assert (pxx, pxy, pyy) == pytest.approx((p[0, 0], p[0, 1], p[1, 1]))


def test_predicted_and_copy_leave_the_filter_alone():
    kf = KalmanFilter(MotionModel.CONSTANT_VELOCITY, 1.0)
    kf.init(0.0, 0.0, (1.0, 0.0, 1.0), 2.0)
    kf.update(1.0, 1.0, (1.0, 0.0, 1.0))
    before = (kf.state, kf.covariance)
    ahead = kf.predicted(2.0)
    other = kf.copy()
    other.predict(1.0)
    assert (kf.state, kf.covariance) == before
    assert ahead.covariance[0][0] > kf.covariance[0][0]
    assert ahead.model == kf.model and ahead.q == kf.q


def test_bad_arguments_raise_value_error():
    with pytest.raises(ValueError):
        KalmanFilter(MotionModel.CONSTANT_VELOCITY, -1.0)
    kf = KalmanFilter(MotionModel.CONSTANT_VELOCITY, 1.0)
    kf.init(0.0, 0.0, (1.0, 0.0, 1.0), 1.0)
    with pytest.raises(ValueError):
        kf.predict(-1.0)
    with pytest.raises(ValueError):
        kf.predict(math.inf)
    assert kf.update(0.0, 0.0, (-3.0, 0.0, -3.0)) == -1.0

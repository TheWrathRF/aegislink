#include "aegis/kalman.hpp"
#include "test_runner.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

using aegis::KalmanFilter;
using aegis::MotionModel;

static bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

TEST(init_sets_position_and_covariance) {
    KalmanFilter kf(MotionModel::ConstantVelocity, 0.2);
    kf.init(1.0, 2.0, {1.0, 0.5, 4.0}, 2.0);
    CHECK_EQ(kf.dim(), std::size_t{4});
    CHECK(near(kf.state()[0], 1.0) && near(kf.state()[1], 2.0));
    CHECK(near(kf.state()[2], 0.0) && near(kf.state()[3], 0.0));
    CHECK(near(kf.covariance(0, 1), 0.5) && near(kf.covariance(1, 0), 0.5));
    CHECK(near(kf.covariance(2, 2), 4.0) && near(kf.covariance(3, 3), 4.0));
    CHECK(near(kf.covariance(0, 2), 0.0));
}

TEST(constant_velocity_predict_matches_hand_computed) {
    // Per axis: P' = F P F^T + q [dt^3/3 dt^2/2; dt^2/2 dt] with F = [1 dt; 0 1].
    const double q = 0.2, dt = 0.5;
    KalmanFilter kf(MotionModel::ConstantVelocity, q);
    kf.init(1.0, 2.0, {1.0, 0.0, 4.0}, 2.0);
    kf.predict(dt);
    CHECK(near(kf.covariance(0, 0), 1.0 + dt * dt * 4.0 + q * dt * dt * dt / 3));
    CHECK(near(kf.covariance(1, 1), 4.0 + dt * dt * 4.0 + q * dt * dt * dt / 3));
    CHECK(near(kf.covariance(0, 2), dt * 4.0 + q * dt * dt / 2));
    CHECK(near(kf.covariance(2, 0), kf.covariance(0, 2)));
    CHECK(near(kf.covariance(2, 2), 4.0 + q * dt));
    CHECK(near(kf.covariance(0, 1), 0.0) && near(kf.covariance(0, 3), 0.0));
    CHECK(near(kf.state()[0], 1.0) && near(kf.state()[1], 2.0));
}

TEST(update_matches_hand_computed) {
    const double q = 0.2, dt = 0.5;
    KalmanFilter kf(MotionModel::ConstantVelocity, q);
    kf.init(1.0, 2.0, {1.0, 0.0, 4.0}, 2.0);
    kf.predict(dt);
    const double pxx = kf.covariance(0, 0), pxv = kf.covariance(0, 2), pvv = kf.covariance(2, 2);
    const double nis = kf.update(2.0, 2.0, {1.0, 0.0, 1.0});
    // The axes are independent here, so each is a scalar filter: S = Pxx + R, K = P[:, x] / S.
    const double s = pxx + 1.0;
    CHECK(near(nis, 1.0 / s));
    CHECK(near(kf.state()[0], 1.0 + pxx / s));
    CHECK(near(kf.state()[2], pxv / s));
    CHECK(near(kf.state()[1], 2.0) && near(kf.state()[3], 0.0));
    CHECK(near(kf.covariance(0, 0), pxx - pxx * pxx / s));
    CHECK(near(kf.covariance(0, 2), pxv - pxx * pxv / s));
    CHECK(near(kf.covariance(2, 2), pvv - pxv * pxv / s));
}

TEST(update_with_correlated_noise_matches_matrix_formula) {
    KalmanFilter kf(MotionModel::ConstantVelocity, 1.0);
    kf.init(0.0, 0.0, {2.0, 0.5, 1.0}, 1.0);
    const aegis::Cov2 r{1.0, -0.3, 2.0};
    const double nis = kf.update(1.0, -1.0, r);
    // S = P + R = [3 0.2; 0.2 3], position P' = P - P S^-1 P.
    const double s00 = 3.0, s01 = 0.2, s11 = 3.0, det = s00 * s11 - s01 * s01;
    const double i00 = s11 / det, i01 = -s01 / det, i11 = s00 / det;
    CHECK(near(nis, 1.0 * (i00 * 1.0 - i01 * 1.0) - 1.0 * (i01 * 1.0 - i11 * 1.0)));
    const double k00 = 2.0 * i00 + 0.5 * i01, k01 = 2.0 * i01 + 0.5 * i11;
    const double k10 = 0.5 * i00 + 1.0 * i01, k11 = 0.5 * i01 + 1.0 * i11;
    CHECK(near(kf.state()[0], k00 * 1.0 - k01 * 1.0));
    CHECK(near(kf.state()[1], k10 * 1.0 - k11 * 1.0));
    CHECK(near(kf.covariance(0, 0), 2.0 - (k00 * 2.0 + k01 * 0.5)));
    CHECK(near(kf.covariance(0, 1), 0.5 - (k00 * 0.5 + k01 * 1.0)));
    CHECK(near(kf.covariance(1, 1), 1.0 - (k10 * 0.5 + k11 * 1.0)));
}

TEST(constant_acceleration_process_noise_matches_hand_computed) {
    const double q = 0.5, dt = 2.0;
    KalmanFilter kf(MotionModel::ConstantAcceleration, q);
    kf.init(0.0, 0.0, {0.0, 0.0, 0.0}, 0.0, 0.0);
    CHECK_EQ(kf.dim(), std::size_t{6});
    kf.predict(dt);
    CHECK(near(kf.covariance(0, 0), q * 32 / 20));  // dt^5 / 20
    CHECK(near(kf.covariance(0, 2), q * 16 / 8));   // dt^4 / 8
    CHECK(near(kf.covariance(0, 4), q * 8 / 6));    // dt^3 / 6
    CHECK(near(kf.covariance(2, 2), q * 8 / 3));    // dt^3 / 3
    CHECK(near(kf.covariance(2, 4), q * 4 / 2));    // dt^2 / 2
    CHECK(near(kf.covariance(4, 4), q * dt));
    CHECK(near(kf.covariance(1, 1), kf.covariance(0, 0)) && near(kf.covariance(0, 1), 0.0));
}

TEST(constant_acceleration_moves_acceleration_into_position) {
    KalmanFilter kf(MotionModel::ConstantAcceleration, 0.0);
    kf.init(0.0, 0.0, {0.0, 0.0, 0.0}, 0.0, 1.0);
    kf.predict(2.0);
    // Only the acceleration is uncertain: x = a dt^2 / 2, v = a dt.
    CHECK(near(kf.covariance(0, 0), 4.0));
    CHECK(near(kf.covariance(0, 2), 4.0));
    CHECK(near(kf.covariance(2, 2), 4.0));
    CHECK(near(kf.covariance(0, 4), 2.0));
}

TEST(learns_a_constant_velocity) {
    for (const auto model : {MotionModel::ConstantVelocity, MotionModel::ConstantAcceleration}) {
        KalmanFilter kf(model, 0.01);
        kf.init(0.0, 0.0, {1e-4, 0.0, 1e-4}, 5.0, 1.0);
        const double dt = 0.1;
        for (int i = 1; i <= 100; ++i) {
            kf.predict(dt);
            kf.update(3.0 * i * dt, -1.0 * i * dt, {1e-4, 0.0, 1e-4});
        }
        CHECK(near(kf.state()[2], 3.0, 1e-3) && near(kf.state()[3], -1.0, 1e-3));
        const KalmanFilter ahead = kf.predicted(2.0);
        CHECK(near(ahead.state()[0], 30.0 + 6.0, 1e-2) && near(ahead.state()[1], -10.0 - 2.0, 1e-2));
        CHECK(near(kf.state()[0], 30.0, 1e-2));  // predicted() left the filter where it was
        CHECK(ahead.covariance(0, 0) > kf.covariance(0, 0));
    }
}

TEST(covariance_stays_symmetric_and_positive) {
    KalmanFilter kf(MotionModel::ConstantAcceleration, 1e-6);
    kf.init(0.0, 0.0, {1.0, 0.0, 1.0}, 10.0, 3.0);
    for (int i = 0; i < 20000; ++i) {
        kf.predict(1.0 / 30);
        kf.update(0.0, 0.0, {1e-10, 0.0, 1e-10});
    }
    const auto p = kf.position_covariance();
    CHECK(p[0] > 0.0 && p[2] > 0.0 && p[0] * p[2] - p[1] * p[1] > 0.0);
    for (std::size_t i = 0; i < kf.dim(); ++i) {
        CHECK(kf.covariance(i, i) >= 0.0);
        for (std::size_t j = 0; j < kf.dim(); ++j) CHECK(kf.covariance(i, j) == kf.covariance(j, i));
    }
}

TEST(rejects_bad_input) {
    bool threw = false;
    try {
        KalmanFilter kf(MotionModel::ConstantVelocity, -1.0);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
    KalmanFilter kf(MotionModel::ConstantVelocity, 1.0);
    kf.init(5.0, 5.0, {1.0, 0.0, 1.0}, 1.0);
    threw = false;
    try {
        kf.predict(-0.1);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
    threw = false;
    try {
        kf.predict(std::nan(""));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
    // An innovation covariance that is not positive definite changes nothing.
    CHECK(kf.update(0.0, 0.0, {-5.0, 0.0, -5.0}) == -1.0);
    CHECK(kf.update(0.0, 0.0, {std::nan(""), 0.0, 1.0}) == -1.0);
    CHECK(near(kf.state()[0], 5.0) && near(kf.covariance(0, 0), 1.0));
}

TEST(innovations_are_chi_square_with_two_degrees_of_freedom) {
    // A target that really moves by the model: the mean NIS of a consistent filter is 2, and
    // 95 % of the positions lie inside the 95 % ellipse (squared distance <= 5.991).
    const double q = 0.5, dt = 1.0 / 30, r = 0.09;
    std::mt19937_64 rng(7);
    std::normal_distribution<double> unit(0.0, 1.0);
    KalmanFilter kf(MotionModel::ConstantVelocity, q);
    kf.init(0.0, 0.0, {r, 0.0, r}, 10.0);
    double pos[2] = {0.0, 0.0}, vel[2] = {8.0, -3.0};
    double nis_sum = 0.0;
    int inside = 0, steps = 0;
    for (int i = 0; i < 40000; ++i) {
        for (int axis = 0; axis < 2; ++axis) {
            // Exact discretisation of white acceleration noise over dt.
            const double a = unit(rng), b = unit(rng);
            const double dv = std::sqrt(q * dt) * a;
            const double dx = std::sqrt(q * dt * dt * dt) * (a / 2 + b / std::sqrt(12.0));
            pos[axis] += vel[axis] * dt + dx;
            vel[axis] += dv;
        }
        kf.predict(dt);
        const double nis = kf.update(pos[0] + std::sqrt(r) * unit(rng), pos[1] + std::sqrt(r) * unit(rng),
                                     {r, 0.0, r});
        if (i < 1000) continue;  // let the start-up transient pass
        nis_sum += nis;
        const auto p = kf.position_covariance();
        const double ex = pos[0] - kf.state()[0], ey = pos[1] - kf.state()[1];
        const double det = p[0] * p[2] - p[1] * p[1];
        const double d2 = (ex * (p[2] * ex - p[1] * ey) + ey * (p[0] * ey - p[1] * ex)) / det;
        inside += d2 <= 5.991 ? 1 : 0;
        ++steps;
    }
    const double mean_nis = nis_sum / steps;
    const double share = static_cast<double>(inside) / steps;
    CHECK(mean_nis > 1.9 && mean_nis < 2.1);
    CHECK(share > 0.94 && share < 0.96);
}

// A linear Kalman filter for one target moving on flat ground, in metres.
#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace aegis {

enum class MotionModel {
    // State x, y, vx, vy; the acceleration is white noise.
    ConstantVelocity,
    // State x, y, vx, vy, ax, ay; the jerk (change of acceleration) is white noise.
    ConstantAcceleration,
};

// A 2x2 symmetric matrix: xx, xy, yy.
using Cov2 = std::array<double, 3>;

class KalmanFilter {
public:
    // `q` is the spectral density of the white noise that drives the model: m^2/s^3 for the
    // acceleration of the constant-velocity model, m^2/s^5 for the jerk of the other.
    KalmanFilter(MotionModel model, double q);

    // Starts at a measured position with covariance `r`; speed and acceleration start at 0 with
    // standard deviations `speed_std` (m/s) and `accel_std` (m/s^2, ignored without that state).
    void init(double x, double y, const Cov2& r, double speed_std, double accel_std = 0.0);

    // Moves the state `dt` seconds ahead (dt >= 0).
    void predict(double dt);

    // Takes a measured position with covariance `r`. Returns the normalised innovation squared
    // (the measurement's squared Mahalanobis distance from the prediction), or -1 when the
    // innovation covariance is not positive definite and nothing was changed.
    double update(double x, double y, const Cov2& r);

    // A copy moved `dt` seconds ahead; this filter is unchanged.
    KalmanFilter predicted(double dt) const;

    MotionModel model() const { return model_; }
    double q() const { return q_; }
    std::size_t dim() const { return n_; }
    // x, y, vx, vy (, ax, ay).
    const std::vector<double>& state() const { return x_; }
    double covariance(std::size_t row, std::size_t col) const { return p_[row * n_ + col]; }
    Cov2 position_covariance() const { return {p_[0], p_[1], p_[n_ + 1]}; }

private:
    MotionModel model_;
    double q_;
    std::size_t n_;
    std::vector<double> x_;
    std::vector<double> p_;  // n x n, row-major
};

}  // namespace aegis

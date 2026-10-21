#include "aegis/kalman.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aegis {

namespace {

// n x n matrices, row-major. The state has at most 6 entries, so plain loops are fast enough.
using Matrix = std::vector<double>;

Matrix multiply(const Matrix& a, const Matrix& b, std::size_t n) {
    Matrix c(n * n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t k = 0; k < n; ++k) {
            const double aik = a[i * n + k];
            for (std::size_t j = 0; j < n; ++j) c[i * n + j] += aik * b[k * n + j];
        }
    }
    return c;
}

Matrix transpose(const Matrix& a, std::size_t n) {
    Matrix t(n * n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) t[j * n + i] = a[i * n + j];
    }
    return t;
}

void symmetrize(Matrix& a, std::size_t n) {
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double mean = 0.5 * (a[i * n + j] + a[j * n + i]);
            a[i * n + j] = mean;
            a[j * n + i] = mean;
        }
    }
}

// The state is ordered by derivative, then axis: x, y, vx, vy, ax, ay.
std::size_t index(std::size_t derivative, std::size_t axis) { return derivative * 2 + axis; }

std::size_t orders(MotionModel model) { return model == MotionModel::ConstantVelocity ? 2 : 3; }

}  // namespace

KalmanFilter::KalmanFilter(MotionModel model, double q)
    : model_(model), q_(q), n_(2 * orders(model)), x_(n_, 0.0), p_(n_ * n_, 0.0) {
    if (!(q >= 0.0) || !std::isfinite(q)) throw std::invalid_argument("q must be finite and >= 0");
}

void KalmanFilter::init(double x, double y, const Cov2& r, double speed_std, double accel_std) {
    std::fill(x_.begin(), x_.end(), 0.0);
    std::fill(p_.begin(), p_.end(), 0.0);
    x_[0] = x;
    x_[1] = y;
    p_[0] = r[0];
    p_[1] = r[1];
    p_[n_] = r[1];
    p_[n_ + 1] = r[2];
    for (std::size_t axis = 0; axis < 2; ++axis) {
        const std::size_t v = index(1, axis);
        p_[v * n_ + v] = speed_std * speed_std;
        if (n_ == 6) {
            const std::size_t a = index(2, axis);
            p_[a * n_ + a] = accel_std * accel_std;
        }
    }
}

void KalmanFilter::predict(double dt) {
    if (!(dt >= 0.0) || !std::isfinite(dt)) throw std::invalid_argument("dt must be finite and >= 0");
    const std::size_t k = orders(model_);
    // Per axis: f[i][j] moves derivative j into derivative i (dt^(j-i) / (j-i)!), and qa is the
    // covariance that white noise in the highest derivative adds over dt (divided by q).
    double f[3][3] = {{1, dt, dt * dt / 2}, {0, 1, dt}, {0, 0, 1}};
    double qa[3][3] = {};
    const double dt2 = dt * dt, dt3 = dt2 * dt;
    if (k == 2) {
        qa[0][0] = dt3 / 3;
        qa[0][1] = qa[1][0] = dt2 / 2;
        qa[1][1] = dt;
    } else {
        const double dt4 = dt3 * dt, dt5 = dt4 * dt;
        qa[0][0] = dt5 / 20;
        qa[0][1] = qa[1][0] = dt4 / 8;
        qa[0][2] = qa[2][0] = dt3 / 6;
        qa[1][1] = dt3 / 3;
        qa[1][2] = qa[2][1] = dt2 / 2;
        qa[2][2] = dt;
    }

    Matrix fm(n_ * n_, 0.0);
    Matrix qm(n_ * n_, 0.0);
    for (std::size_t axis = 0; axis < 2; ++axis) {
        for (std::size_t i = 0; i < k; ++i) {
            for (std::size_t j = 0; j < k; ++j) {
                fm[index(i, axis) * n_ + index(j, axis)] = f[i][j];
                qm[index(i, axis) * n_ + index(j, axis)] = q_ * qa[i][j];
            }
        }
    }

    std::vector<double> x(n_, 0.0);
    for (std::size_t i = 0; i < n_; ++i) {
        for (std::size_t j = 0; j < n_; ++j) x[i] += fm[i * n_ + j] * x_[j];
    }
    x_ = x;
    p_ = multiply(multiply(fm, p_, n_), transpose(fm, n_), n_);
    for (std::size_t i = 0; i < n_ * n_; ++i) p_[i] += qm[i];
    symmetrize(p_, n_);
}

double KalmanFilter::update(double x, double y, const Cov2& r) {
    // The measurement is the position: H = [I 0], so H P H^T is the top-left 2x2 block of P.
    const double s00 = p_[0] + r[0];
    const double s01 = p_[1] + r[1];
    const double s11 = p_[n_ + 1] + r[2];
    const double det = s00 * s11 - s01 * s01;
    if (!(det > 0.0) || !(s00 > 0.0) || !std::isfinite(det)) return -1.0;
    const double i00 = s11 / det, i01 = -s01 / det, i11 = s00 / det;
    const double y0 = x - x_[0];
    const double y1 = y - x_[1];
    const double nis = y0 * (i00 * y0 + i01 * y1) + y1 * (i01 * y0 + i11 * y1);

    // Gain K = P H^T S^-1 (n x 2).
    std::vector<double> gain(n_ * 2);
    for (std::size_t i = 0; i < n_; ++i) {
        const double p0 = p_[i * n_], p1 = p_[i * n_ + 1];
        gain[i * 2] = p0 * i00 + p1 * i01;
        gain[i * 2 + 1] = p0 * i01 + p1 * i11;
    }
    for (std::size_t i = 0; i < n_; ++i) x_[i] += gain[i * 2] * y0 + gain[i * 2 + 1] * y1;

    // Joseph form, P = (I - KH) P (I - KH)^T + K R K^T: it stays symmetric and positive
    // definite despite rounding, unlike the shorter (I - KH) P.
    Matrix a(n_ * n_, 0.0);
    for (std::size_t i = 0; i < n_; ++i) {
        a[i * n_ + i] = 1.0;
        a[i * n_] -= gain[i * 2];
        a[i * n_ + 1] -= gain[i * 2 + 1];
    }
    Matrix p = multiply(multiply(a, p_, n_), transpose(a, n_), n_);
    for (std::size_t i = 0; i < n_; ++i) {
        const double k0 = gain[i * 2], k1 = gain[i * 2 + 1];
        for (std::size_t j = 0; j < n_; ++j) {
            const double l0 = gain[j * 2], l1 = gain[j * 2 + 1];
            p[i * n_ + j] += k0 * (r[0] * l0 + r[1] * l1) + k1 * (r[1] * l0 + r[2] * l1);
        }
    }
    p_ = p;
    symmetrize(p_, n_);
    return nis;
}

KalmanFilter KalmanFilter::predicted(double dt) const {
    KalmanFilter copy = *this;
    copy.predict(dt);
    return copy;
}

}  // namespace aegis

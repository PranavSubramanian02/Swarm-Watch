#pragma once

// Zero-allocation fixed-size linear algebra and Hamilton quaternion math

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace swarm {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

inline double clampd(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline double wrap_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double norm2() const { return x * x + y * y + z * z; }
    double norm() const { return std::sqrt(norm2()); }
    Vec3 normalized() const {
        double n = norm();
        return n > 1e-12 ? (*this) / n : Vec3{0.0, 0.0, 0.0};
    }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

template <int R, int C>
struct Mat {
    std::array<double, R * C> data{};

    double& operator()(int r, int c) { return data[r * C + c]; }
    double operator()(int r, int c) const { return data[r * C + c]; }

    static Mat<R, C> zero() { return Mat<R, C>{}; }

    static Mat<R, R> identity() {
        static_assert(R == C, "Identity requires square matrix");
        Mat<R, R> m{};
        for (int i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }

    Mat<R, C> operator+(const Mat<R, C>& o) const {
        Mat<R, C> out{};
        for (int i = 0; i < R * C; ++i) out.data[i] = data[i] + o.data[i];
        return out;
    }

    Mat<R, C> operator-(const Mat<R, C>& o) const {
        Mat<R, C> out{};
        for (int i = 0; i < R * C; ++i) out.data[i] = data[i] - o.data[i];
        return out;
    }

    Mat<R, C> operator*(double s) const {
        Mat<R, C> out{};
        for (int i = 0; i < R * C; ++i) out.data[i] = data[i] * s;
        return out;
    }

    Mat<C, R> transpose() const {
        Mat<C, R> out{};
        for (int r = 0; r < R; ++r)
            for (int c = 0; c < C; ++c)
                out(c, r) = (*this)(r, c);
        return out;
    }

    double trace() const {
        static_assert(R == C, "Trace requires square matrix");
        double t = 0.0;
        for (int i = 0; i < R; ++i) t += (*this)(i, i);
        return t;
    }
};

template <int R, int K, int C>
Mat<R, C> mat_mul(const Mat<R, K>& a, const Mat<K, C>& b) {
    Mat<R, C> out{};
    for (int r = 0; r < R; ++r) {
        for (int k = 0; k < K; ++k) {
            double aik = a(r, k);
            if (aik == 0.0) continue;
            for (int c = 0; c < C; ++c) out(r, c) += aik * b(k, c);
        }
    }
    return out;
}

template <int R, int C>
std::array<double, R> mat_vec(const Mat<R, C>& a, const std::array<double, C>& v) {
    std::array<double, R> out{};
    for (int r = 0; r < R; ++r) {
        double s = 0.0;
        for (int c = 0; c < C; ++c) s += a(r, c) * v[c];
        out[r] = s;
    }
    return out;
}

inline Mat<3, 3> skew(const Vec3& v) {
    Mat<3, 3> s{};
    s(0, 1) = -v.z; s(0, 2) = v.y;
    s(1, 0) = v.z;  s(1, 2) = -v.x;
    s(2, 0) = -v.y; s(2, 1) = v.x;
    return s;
}

inline Vec3 mat3_vec(const Mat<3, 3>& m, const Vec3& v) {
    return {
        m(0, 0) * v.x + m(0, 1) * v.y + m(0, 2) * v.z,
        m(1, 0) * v.x + m(1, 1) * v.y + m(1, 2) * v.z,
        m(2, 0) * v.x + m(2, 1) * v.y + m(2, 2) * v.z,
    };
}

// Gauss-Jordan matrix inversion for small positive-definite blocks
template <int N>
bool mat_inv(const Mat<N, N>& a, Mat<N, N>& inv_out) {
    Mat<N, N> m = a;
    inv_out = Mat<N, N>::identity();
    for (int i = 0; i < N; ++i) {
        int pivot = i;
        double best = std::abs(m(i, i));
        for (int r = i + 1; r < N; ++r) {
            if (std::abs(m(r, i)) > best) {
                best = std::abs(m(r, i));
                pivot = r;
            }
        }
        if (best < 1e-12) return false;
        if (pivot != i) {
            for (int c = 0; c < N; ++c) {
                std::swap(m(i, c), m(pivot, c));
                std::swap(inv_out(i, c), inv_out(pivot, c));
            }
        }
        double diag = m(i, i);
        for (int c = 0; c < N; ++c) {
            m(i, c) /= diag;
            inv_out(i, c) /= diag;
        }
        for (int r = 0; r < N; ++r) {
            if (r == i) continue;
            double factor = m(r, i);
            for (int c = 0; c < N; ++c) {
                m(r, c) -= factor * m(i, c);
                inv_out(r, c) -= factor * inv_out(i, c);
            }
        }
    }
    return true;
}

// Unit Hamilton quaternion (w, x, y, z) representing body-to-NED rotation R_nb
struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Quat() = default;
    Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    Quat normalized() const {
        double n = norm();
        if (n < 1e-15) return {1.0, 0.0, 0.0, 0.0};
        double s = (w < 0.0 ? -1.0 : 1.0) / n;
        return {w * s, x * s, y * s, z * s};
    }

    Quat conjugate() const { return {w, -x, -y, -z}; }

    Quat operator*(const Quat& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
        };
    }

    static Quat from_rotvec(const Vec3& dtheta) {
        double angle = dtheta.norm();
        if (angle < 1e-8) {
            return Quat{1.0, 0.5 * dtheta.x, 0.5 * dtheta.y, 0.5 * dtheta.z}.normalized();
        }
        double half = 0.5 * angle;
        double s = std::sin(half) / angle;
        return Quat{std::cos(half), dtheta.x * s, dtheta.y * s, dtheta.z * s}.normalized();
    }

    static Quat from_euler(double roll, double pitch, double yaw) {
        double cr = std::cos(0.5 * roll), sr = std::sin(0.5 * roll);
        double cp = std::cos(0.5 * pitch), sp = std::sin(0.5 * pitch);
        double cy = std::cos(0.5 * yaw), sy = std::sin(0.5 * yaw);
        return Quat{
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        }.normalized();
    }

    Vec3 to_euler() const {
        double sinr = 2.0 * (w * x + y * z);
        double cosr = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr, cosr);
        double sinp = clampd(2.0 * (w * y - z * x), -1.0, 1.0);
        double pitch = std::asin(sinp);
        double siny = 2.0 * (w * z + x * y);
        double cosy = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny, cosy);
        return {roll, pitch, yaw};
    }

    Mat<3, 3> to_rot_matrix() const {
        Mat<3, 3> r{};
        double xx = x * x, yy = y * y, zz = z * z;
        double xy = x * y, xz = x * z, yz = y * z;
        double wx = w * x, wy = w * y, wz = w * z;
        r(0, 0) = 1.0 - 2.0 * (yy + zz); r(0, 1) = 2.0 * (xy - wz);       r(0, 2) = 2.0 * (xz + wy);
        r(1, 0) = 2.0 * (xy + wz);       r(1, 1) = 1.0 - 2.0 * (xx + zz); r(1, 2) = 2.0 * (yz - wx);
        r(2, 0) = 2.0 * (xz - wy);       r(2, 1) = 2.0 * (yz + wx);       r(2, 2) = 1.0 - 2.0 * (xx + yy);
        return r;
    }

    Vec3 rotate(const Vec3& v) const { return mat3_vec(to_rot_matrix(), v); }
    Vec3 rotate_inv(const Vec3& v) const { return mat3_vec(to_rot_matrix().transpose(), v); }
};

inline Vec3 quat_log(const Quat& q_in) {
    Quat q = q_in.normalized();
    Vec3 v{q.x, q.y, q.z};
    double vn = v.norm();
    if (vn < 1e-10) return 2.0 * v;
    double angle = 2.0 * std::atan2(vn, q.w);
    return (angle / vn) * v;
}

}  // namespace swarm

#pragma once

#include <cmath>

namespace sarx {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }

    constexpr Vec3& operator*=(double s) {
        x *= s; y *= s; z *= s;
        return *this;
    }
};

constexpr Vec3 operator+(Vec3 a, const Vec3& b) { return a += b; }
constexpr Vec3 operator-(Vec3 a, const Vec3& b) { return a -= b; }
constexpr Vec3 operator-(const Vec3& v) { return {-v.x, -v.y, -v.z}; }
constexpr Vec3 operator*(Vec3 v, double s) { return v *= s; }
constexpr Vec3 operator*(double s, Vec3 v) { return v *= s; }
constexpr Vec3 operator/(Vec3 v, double s) { return {v.x / s, v.y / s, v.z / s}; }

constexpr double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline double length_squared(const Vec3& v) { return dot(v, v); }
inline double length(const Vec3& v) { return std::sqrt(length_squared(v)); }

inline Vec3 normalized(const Vec3& v) {
    const double len = length(v);
    return len > 1e-12 ? v / len : Vec3{};
}

inline Vec3 rotate_between(const Vec3& from, const Vec3& to, const Vec3& value) {
    const Vec3 a = normalized(from);
    const Vec3 b = normalized(to);
    if (length_squared(a) <= 1e-12 || length_squared(b) <= 1e-12) {
        return value;
    }

    const double c = dot(a, b);
    if (c >= 1.0 - 1e-10) {
        return value;
    }

    if (c <= -1.0 + 1e-10) {
        const Vec3 basis = std::abs(a.x) < 0.8
            ? Vec3{1.0, 0.0, 0.0}
            : Vec3{0.0, 1.0, 0.0};
        const Vec3 axis = normalized(cross(a, basis));
        return (-value) + axis * (2.0 * dot(axis, value));
    }

    const Vec3 axis_raw = cross(a, b);
    const double s = length(axis_raw);
    const Vec3 axis = axis_raw / s;

    return value * c
        + cross(axis, value) * s
        + axis * (dot(axis, value) * (1.0 - c));
}

// Unit quaternion rotation (w + xi + yj + zk). Identity by default.
struct Rotation {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

inline Rotation normalized(const Rotation& q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return n > 1e-12 ? Rotation{q.w / n, q.x / n, q.y / n, q.z / n} : Rotation{};
}

inline Rotation operator*(const Rotation& a, const Rotation& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline Rotation conjugate(const Rotation& q) { return {q.w, -q.x, -q.y, -q.z}; }

inline Vec3 rotate(const Rotation& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = cross(u, v) * 2.0;
    return v + t * q.w + cross(u, t);
}

inline Rotation axis_angle(const Vec3& axis, double radians) {
    const double len = std::sqrt(dot(axis, axis));
    if (len <= 1e-12) return {};
    const double s = std::sin(radians * 0.5) / len;
    return {std::cos(radians * 0.5), axis.x * s, axis.y * s, axis.z * s};
}

inline Rotation slerp(Rotation a, Rotation b, double t) {
    double d = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (d < 0.0) {
        b = {-b.w, -b.x, -b.y, -b.z};
        d = -d;
    }
    if (d > 0.9995) {
        return normalized(Rotation{a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
                                   a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t});
    }
    const double theta = std::acos(d);
    const double s = std::sin(theta);
    const double wa = std::sin((1.0 - t) * theta) / s;
    const double wb = std::sin(t * theta) / s;
    return {a.w * wa + b.w * wb, a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb};
}

inline bool nearly_equal(double a, double b, double eps = 1e-8) {
    return std::abs(a - b) <= eps;
}

inline bool nearly_equal(const Vec3& a, const Vec3& b, double eps = 1e-8) {
    return nearly_equal(a.x, b.x, eps)
        && nearly_equal(a.y, b.y, eps)
        && nearly_equal(a.z, b.z, eps);
}

} // namespace sarx

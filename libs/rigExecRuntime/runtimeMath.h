//
// rigExecRuntime vendored scalar math: vectors.
//
// An operational mirror of the OpenUSD Gf subset the kernels use. Every
// operation performs the SAME floating-point operations in the SAME order
// as its Gf counterpart -- including Gf's quirks (division multiplies by
// a double reciprocal; Normalize divides short vectors by eps rather than
// by their length) -- so results are bit-identical under default FP
// semantics. No fast-math: the runtime target must keep precise behavior
// on every platform or the parity gate fails.
//
// Verified against Gf by testRigExecRuntimeMath (randomized bitwise
// comparison, USD linked into the TEST only -- never here).
//
// Only <algorithm>, <cmath>, <cstdint>: this header must compile with no
// USD include path. The M4 import check enforces it.
//

#ifndef RIGEXEC_RUNTIME_MATH_H
#define RIGEXEC_RUNTIME_MATH_H

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdint>

namespace rigExec {

/// GF_MIN_VECTOR_LENGTH: the short-vector guard of Normalize.
inline constexpr double kRrMinVectorLength = 1e-10;

inline double
RrSqrt(double v)
{
    return std::sqrt(v);
}

inline float
RrSqrt(float v)
{
    return std::sqrt(v);
}

inline double
RrClamp(double value, double min, double max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

inline float
RrClamp(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

struct RrVec2f {
    float _data[2];

    RrVec2f() = default;
    explicit RrVec2f(float s)
    {
        _data[0] = s;
        _data[1] = s;
    }
    RrVec2f(float x, float y)
    {
        _data[0] = x;
        _data[1] = y;
    }

    const float *data() const { return _data; }
    float *data() { return _data; }
    float operator[](size_t i) const { return _data[i]; }
    float &operator[](size_t i) { return _data[i]; }

    bool operator==(const RrVec2f &v) const
    {
        return _data[0] == v._data[0] && _data[1] == v._data[1];
    }
    bool operator!=(const RrVec2f &v) const { return !(*this == v); }

    RrVec2f operator-() const { return RrVec2f(-_data[0], -_data[1]); }

    RrVec2f &operator+=(const RrVec2f &v)
    {
        _data[0] += v._data[0];
        _data[1] += v._data[1];
        return *this;
    }
    RrVec2f &operator-=(const RrVec2f &v)
    {
        _data[0] -= v._data[0];
        _data[1] -= v._data[1];
        return *this;
    }
    RrVec2f &operator*=(double s)
    {
        _data[0] *= s;
        _data[1] *= s;
        return *this;
    }
    RrVec2f &operator/=(double s) { return *this *= (1.0 / s); }

    RrVec2f operator+(const RrVec2f &v) const
    {
        return RrVec2f(*this) += v;
    }
    RrVec2f operator-(const RrVec2f &v) const
    {
        return RrVec2f(*this) -= v;
    }
    RrVec2f operator*(double s) const { return RrVec2f(*this) *= s; }
    RrVec2f operator/(double s) const { return *this * (1.0 / s); }
    friend RrVec2f operator*(double s, const RrVec2f &v) { return v * s; }

    float operator*(const RrVec2f &v) const
    {
        return _data[0] * v[0] + _data[1] * v[1];
    }

    float GetLengthSq() const { return *this * *this; }
    float GetLength() const { return RrSqrt(GetLengthSq()); }
    float Normalize(float eps = float(kRrMinVectorLength))
    {
        float length = GetLength();
        *this /= (length > eps) ? length : eps;
        return length;
    }
    RrVec2f GetNormalized(float eps = float(kRrMinVectorLength)) const
    {
        RrVec2f normalized(*this);
        normalized.Normalize(eps);
        return normalized;
    }
};

struct RrVec2d {
    double _data[2];

    RrVec2d() = default;
    explicit RrVec2d(double s)
    {
        _data[0] = s;
        _data[1] = s;
    }
    RrVec2d(double x, double y)
    {
        _data[0] = x;
        _data[1] = y;
    }

    const double *data() const { return _data; }
    double *data() { return _data; }
    double operator[](size_t i) const { return _data[i]; }
    double &operator[](size_t i) { return _data[i]; }

    bool operator==(const RrVec2d &v) const
    {
        return _data[0] == v._data[0] && _data[1] == v._data[1];
    }
    bool operator!=(const RrVec2d &v) const { return !(*this == v); }

    RrVec2d operator-() const { return RrVec2d(-_data[0], -_data[1]); }

    RrVec2d &operator+=(const RrVec2d &v)
    {
        _data[0] += v._data[0];
        _data[1] += v._data[1];
        return *this;
    }
    RrVec2d &operator-=(const RrVec2d &v)
    {
        _data[0] -= v._data[0];
        _data[1] -= v._data[1];
        return *this;
    }
    RrVec2d &operator*=(double s)
    {
        _data[0] *= s;
        _data[1] *= s;
        return *this;
    }
    RrVec2d &operator/=(double s) { return *this *= (1.0 / s); }

    RrVec2d operator+(const RrVec2d &v) const
    {
        return RrVec2d(*this) += v;
    }
    RrVec2d operator-(const RrVec2d &v) const
    {
        return RrVec2d(*this) -= v;
    }
    RrVec2d operator*(double s) const { return RrVec2d(*this) *= s; }
    RrVec2d operator/(double s) const { return *this * (1.0 / s); }
    friend RrVec2d operator*(double s, const RrVec2d &v) { return v * s; }

    double operator*(const RrVec2d &v) const
    {
        return _data[0] * v[0] + _data[1] * v[1];
    }

    double GetLengthSq() const { return *this * *this; }
    double GetLength() const { return RrSqrt(GetLengthSq()); }
    double Normalize(double eps = kRrMinVectorLength)
    {
        double length = GetLength();
        *this /= (length > eps) ? length : eps;
        return length;
    }
    RrVec2d GetNormalized(double eps = kRrMinVectorLength) const
    {
        RrVec2d normalized(*this);
        normalized.Normalize(eps);
        return normalized;
    }
};

struct RrVec3f {
    float _data[3];

    RrVec3f() = default;
    explicit RrVec3f(float s)
    {
        _data[0] = s;
        _data[1] = s;
        _data[2] = s;
    }
    RrVec3f(float x, float y, float z)
    {
        _data[0] = x;
        _data[1] = y;
        _data[2] = z;
    }

    const float *data() const { return _data; }
    float *data() { return _data; }
    float operator[](size_t i) const { return _data[i]; }
    float &operator[](size_t i) { return _data[i]; }

    bool operator==(const RrVec3f &v) const
    {
        return _data[0] == v._data[0] && _data[1] == v._data[1] &&
               _data[2] == v._data[2];
    }
    bool operator!=(const RrVec3f &v) const { return !(*this == v); }

    RrVec3f operator-() const
    {
        return RrVec3f(-_data[0], -_data[1], -_data[2]);
    }

    RrVec3f &operator+=(const RrVec3f &v)
    {
        _data[0] += v._data[0];
        _data[1] += v._data[1];
        _data[2] += v._data[2];
        return *this;
    }
    RrVec3f &operator-=(const RrVec3f &v)
    {
        _data[0] -= v._data[0];
        _data[1] -= v._data[1];
        _data[2] -= v._data[2];
        return *this;
    }
    RrVec3f &operator*=(double s)
    {
        _data[0] *= s;
        _data[1] *= s;
        _data[2] *= s;
        return *this;
    }
    RrVec3f &operator/=(double s) { return *this *= (1.0 / s); }

    RrVec3f operator+(const RrVec3f &v) const
    {
        return RrVec3f(*this) += v;
    }
    RrVec3f operator-(const RrVec3f &v) const
    {
        return RrVec3f(*this) -= v;
    }
    RrVec3f operator*(double s) const { return RrVec3f(*this) *= s; }
    RrVec3f operator/(double s) const { return *this * (1.0 / s); }
    friend RrVec3f operator*(double s, const RrVec3f &v) { return v * s; }

    float operator*(const RrVec3f &v) const
    {
        return _data[0] * v[0] + _data[1] * v[1] + _data[2] * v[2];
    }

    float GetLengthSq() const { return *this * *this; }
    float GetLength() const { return RrSqrt(GetLengthSq()); }
    float Normalize(float eps = float(kRrMinVectorLength))
    {
        float length = GetLength();
        *this /= (length > eps) ? length : eps;
        return length;
    }
    RrVec3f GetNormalized(float eps = float(kRrMinVectorLength)) const
    {
        RrVec3f normalized(*this);
        normalized.Normalize(eps);
        return normalized;
    }
};

struct RrVec3d {
    double _data[3];

    RrVec3d() = default;
    explicit RrVec3d(double s)
    {
        _data[0] = s;
        _data[1] = s;
        _data[2] = s;
    }
    RrVec3d(double x, double y, double z)
    {
        _data[0] = x;
        _data[1] = y;
        _data[2] = z;
    }

    static RrVec3d XAxis() { return RrVec3d(1.0, 0.0, 0.0); }
    static RrVec3d YAxis() { return RrVec3d(0.0, 1.0, 0.0); }
    static RrVec3d ZAxis() { return RrVec3d(0.0, 0.0, 1.0); }

    const double *data() const { return _data; }
    double *data() { return _data; }
    double operator[](size_t i) const { return _data[i]; }
    double &operator[](size_t i) { return _data[i]; }

    bool operator==(const RrVec3d &v) const
    {
        return _data[0] == v._data[0] && _data[1] == v._data[1] &&
               _data[2] == v._data[2];
    }
    bool operator!=(const RrVec3d &v) const { return !(*this == v); }

    RrVec3d operator-() const
    {
        return RrVec3d(-_data[0], -_data[1], -_data[2]);
    }

    RrVec3d &operator+=(const RrVec3d &v)
    {
        _data[0] += v._data[0];
        _data[1] += v._data[1];
        _data[2] += v._data[2];
        return *this;
    }
    RrVec3d &operator-=(const RrVec3d &v)
    {
        _data[0] -= v._data[0];
        _data[1] -= v._data[1];
        _data[2] -= v._data[2];
        return *this;
    }
    RrVec3d &operator*=(double s)
    {
        _data[0] *= s;
        _data[1] *= s;
        _data[2] *= s;
        return *this;
    }
    RrVec3d &operator/=(double s) { return *this *= (1.0 / s); }

    RrVec3d operator+(const RrVec3d &v) const
    {
        return RrVec3d(*this) += v;
    }
    RrVec3d operator-(const RrVec3d &v) const
    {
        return RrVec3d(*this) -= v;
    }
    RrVec3d operator*(double s) const { return RrVec3d(*this) *= s; }
    RrVec3d operator/(double s) const { return *this * (1.0 / s); }
    friend RrVec3d operator*(double s, const RrVec3d &v) { return v * s; }

    double operator*(const RrVec3d &v) const
    {
        return _data[0] * v[0] + _data[1] * v[1] + _data[2] * v[2];
    }

    double GetLengthSq() const { return *this * *this; }
    double GetLength() const { return RrSqrt(GetLengthSq()); }
    double Normalize(double eps = kRrMinVectorLength)
    {
        double length = GetLength();
        *this /= (length > eps) ? length : eps;
        return length;
    }
    RrVec3d GetNormalized(double eps = kRrMinVectorLength) const
    {
        RrVec3d normalized(*this);
        normalized.Normalize(eps);
        return normalized;
    }
};

struct RrVec4d {
    double _data[4];

    RrVec4d() = default;
    explicit RrVec4d(double s)
    {
        _data[0] = s;
        _data[1] = s;
        _data[2] = s;
        _data[3] = s;
    }
    RrVec4d(double x, double y, double z, double w)
    {
        _data[0] = x;
        _data[1] = y;
        _data[2] = z;
        _data[3] = w;
    }

    const double *data() const { return _data; }
    double *data() { return _data; }
    double operator[](size_t i) const { return _data[i]; }
    double &operator[](size_t i) { return _data[i]; }

    bool operator==(const RrVec4d &v) const
    {
        return _data[0] == v._data[0] && _data[1] == v._data[1] &&
               _data[2] == v._data[2] && _data[3] == v._data[3];
    }
    bool operator!=(const RrVec4d &v) const { return !(*this == v); }

    RrVec4d operator-() const
    {
        return RrVec4d(-_data[0], -_data[1], -_data[2], -_data[3]);
    }

    RrVec4d &operator+=(const RrVec4d &v)
    {
        _data[0] += v._data[0];
        _data[1] += v._data[1];
        _data[2] += v._data[2];
        _data[3] += v._data[3];
        return *this;
    }
    RrVec4d &operator-=(const RrVec4d &v)
    {
        _data[0] -= v._data[0];
        _data[1] -= v._data[1];
        _data[2] -= v._data[2];
        _data[3] -= v._data[3];
        return *this;
    }
    RrVec4d &operator*=(double s)
    {
        _data[0] *= s;
        _data[1] *= s;
        _data[2] *= s;
        _data[3] *= s;
        return *this;
    }
    RrVec4d &operator/=(double s) { return *this *= (1.0 / s); }

    RrVec4d operator+(const RrVec4d &v) const
    {
        return RrVec4d(*this) += v;
    }
    RrVec4d operator-(const RrVec4d &v) const
    {
        return RrVec4d(*this) -= v;
    }
    RrVec4d operator*(double s) const { return RrVec4d(*this) *= s; }
    RrVec4d operator/(double s) const { return *this * (1.0 / s); }
    friend RrVec4d operator*(double s, const RrVec4d &v) { return v * s; }

    double operator*(const RrVec4d &v) const
    {
        return _data[0] * v[0] + _data[1] * v[1] + _data[2] * v[2] +
               _data[3] * v[3];
    }

    double GetLengthSq() const { return *this * *this; }
    double GetLength() const { return RrSqrt(GetLengthSq()); }
    double Normalize(double eps = kRrMinVectorLength)
    {
        double length = GetLength();
        *this /= (length > eps) ? length : eps;
        return length;
    }
    RrVec4d GetNormalized(double eps = kRrMinVectorLength) const
    {
        RrVec4d normalized(*this);
        normalized.Normalize(eps);
        return normalized;
    }
};

struct RrVec3i {
    int32_t _data[3];

    RrVec3i() = default;
    explicit RrVec3i(int32_t s)
    {
        _data[0] = s;
        _data[1] = s;
        _data[2] = s;
    }
    RrVec3i(int32_t x, int32_t y, int32_t z)
    {
        _data[0] = x;
        _data[1] = y;
        _data[2] = z;
    }

    int32_t operator[](size_t i) const { return _data[i]; }
    int32_t &operator[](size_t i) { return _data[i]; }

    bool operator==(const RrVec3i &v) const
    {
        return _data[0] == v._data[0] && _data[1] == v._data[1] &&
               _data[2] == v._data[2];
    }
    bool operator!=(const RrVec3i &v) const { return !(*this == v); }
};

inline float
RrDot(const RrVec2f &a, const RrVec2f &b)
{
    return a * b;
}

inline double
RrDot(const RrVec2d &a, const RrVec2d &b)
{
    return a * b;
}

inline float
RrDot(const RrVec3f &a, const RrVec3f &b)
{
    return a * b;
}

inline double
RrDot(const RrVec3d &a, const RrVec3d &b)
{
    return a * b;
}

inline RrVec3f
RrCross(const RrVec3f &a, const RrVec3f &b)
{
    return RrVec3f(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                   a[0] * b[1] - a[1] * b[0]);
}

inline RrVec3d
RrCross(const RrVec3d &a, const RrVec3d &b)
{
    return RrVec3d(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                   a[0] * b[1] - a[1] * b[0]);
}

inline RrVec3f operator^(const RrVec3f &a, const RrVec3f &b)
{
    return RrCross(a, b);
}

inline RrVec3d operator^(const RrVec3d &a, const RrVec3d &b)
{
    return RrCross(a, b);
}

struct RrRotation;
struct RrQuatd;

inline RrVec3d
RrProject(const RrVec4d &v)
{
    if (v[3] == 1.0) {
        return RrVec3d(v[0], v[1], v[2]);
    } else if (v[3] == 0.0) {
        return RrVec3d(v[0], v[1], v[2]);
    } else {
        return RrVec3d(v[0] / v[3], v[1] / v[3], v[2] / v[3]);
    }
}

struct RrMat3d {
    double _mtx[3][3];

    RrMat3d() = default;
    RrMat3d(double m00, double m01, double m02, double m10, double m11,
            double m12, double m20, double m21, double m22)
    {
        Set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }
    explicit RrMat3d(double s) { SetDiagonal(s); }

    RrMat3d &Set(double m00, double m01, double m02, double m10,
                 double m11, double m12, double m20, double m21,
                 double m22)
    {
        _mtx[0][0] = m00;
        _mtx[0][1] = m01;
        _mtx[0][2] = m02;
        _mtx[1][0] = m10;
        _mtx[1][1] = m11;
        _mtx[1][2] = m12;
        _mtx[2][0] = m20;
        _mtx[2][1] = m21;
        _mtx[2][2] = m22;
        return *this;
    }

    RrMat3d &SetDiagonal(double s)
    {
        _mtx[0][0] = s;
        _mtx[0][1] = 0.0;
        _mtx[0][2] = 0.0;
        _mtx[1][0] = 0.0;
        _mtx[1][1] = s;
        _mtx[1][2] = 0.0;
        _mtx[2][0] = 0.0;
        _mtx[2][1] = 0.0;
        _mtx[2][2] = s;
        return *this;
    }

    RrMat3d &SetIdentity() { return SetDiagonal(1.0); }

    RrMat3d &SetScale(double s) { return SetDiagonal(s); }

    void SetRow(int i, const RrVec3d &v)
    {
        _mtx[i][0] = v[0];
        _mtx[i][1] = v[1];
        _mtx[i][2] = v[2];
    }
    RrVec3d GetRow(int i) const
    {
        return RrVec3d(_mtx[i][0], _mtx[i][1], _mtx[i][2]);
    }
    void SetColumn(int i, const RrVec3d &v)
    {
        _mtx[0][i] = v[0];
        _mtx[1][i] = v[1];
        _mtx[2][i] = v[2];
    }
    RrVec3d GetColumn(int i) const
    {
        return RrVec3d(_mtx[0][i], _mtx[1][i], _mtx[2][i]);
    }

    double *operator[](size_t i) { return _mtx[i]; }
    const double *operator[](size_t i) const { return _mtx[i]; }

    bool operator==(const RrMat3d &m) const
    {
        return (_mtx[0][0] == m._mtx[0][0] &&
                _mtx[0][1] == m._mtx[0][1] &&
                _mtx[0][2] == m._mtx[0][2] &&
                _mtx[1][0] == m._mtx[1][0] &&
                _mtx[1][1] == m._mtx[1][1] &&
                _mtx[1][2] == m._mtx[1][2] &&
                _mtx[2][0] == m._mtx[2][0] &&
                _mtx[2][1] == m._mtx[2][1] &&
                _mtx[2][2] == m._mtx[2][2]);
    }
    bool operator!=(const RrMat3d &m) const { return !(*this == m); }

    RrMat3d &operator*=(const RrMat3d &m)
    {
        RrMat3d tmp = *this;

        _mtx[0][0] = tmp._mtx[0][0] * m._mtx[0][0] +
                     tmp._mtx[0][1] * m._mtx[1][0] +
                     tmp._mtx[0][2] * m._mtx[2][0];

        _mtx[0][1] = tmp._mtx[0][0] * m._mtx[0][1] +
                     tmp._mtx[0][1] * m._mtx[1][1] +
                     tmp._mtx[0][2] * m._mtx[2][1];

        _mtx[0][2] = tmp._mtx[0][0] * m._mtx[0][2] +
                     tmp._mtx[0][1] * m._mtx[1][2] +
                     tmp._mtx[0][2] * m._mtx[2][2];

        _mtx[1][0] = tmp._mtx[1][0] * m._mtx[0][0] +
                     tmp._mtx[1][1] * m._mtx[1][0] +
                     tmp._mtx[1][2] * m._mtx[2][0];

        _mtx[1][1] = tmp._mtx[1][0] * m._mtx[0][1] +
                     tmp._mtx[1][1] * m._mtx[1][1] +
                     tmp._mtx[1][2] * m._mtx[2][1];

        _mtx[1][2] = tmp._mtx[1][0] * m._mtx[0][2] +
                     tmp._mtx[1][1] * m._mtx[1][2] +
                     tmp._mtx[1][2] * m._mtx[2][2];

        _mtx[2][0] = tmp._mtx[2][0] * m._mtx[0][0] +
                     tmp._mtx[2][1] * m._mtx[1][0] +
                     tmp._mtx[2][2] * m._mtx[2][0];

        _mtx[2][1] = tmp._mtx[2][0] * m._mtx[0][1] +
                     tmp._mtx[2][1] * m._mtx[1][1] +
                     tmp._mtx[2][2] * m._mtx[2][1];

        _mtx[2][2] = tmp._mtx[2][0] * m._mtx[0][2] +
                     tmp._mtx[2][1] * m._mtx[1][2] +
                     tmp._mtx[2][2] * m._mtx[2][2];

        return *this;
    }

    RrMat3d &operator*=(double d)
    {
        _mtx[0][0] *= d;
        _mtx[0][1] *= d;
        _mtx[0][2] *= d;
        _mtx[1][0] *= d;
        _mtx[1][1] *= d;
        _mtx[1][2] *= d;
        _mtx[2][0] *= d;
        _mtx[2][1] *= d;
        _mtx[2][2] *= d;
        return *this;
    }

    RrMat3d &operator+=(const RrMat3d &m)
    {
        _mtx[0][0] += m._mtx[0][0];
        _mtx[0][1] += m._mtx[0][1];
        _mtx[0][2] += m._mtx[0][2];
        _mtx[1][0] += m._mtx[1][0];
        _mtx[1][1] += m._mtx[1][1];
        _mtx[1][2] += m._mtx[1][2];
        _mtx[2][0] += m._mtx[2][0];
        _mtx[2][1] += m._mtx[2][1];
        _mtx[2][2] += m._mtx[2][2];
        return *this;
    }

    RrMat3d &operator-=(const RrMat3d &m)
    {
        _mtx[0][0] -= m._mtx[0][0];
        _mtx[0][1] -= m._mtx[0][1];
        _mtx[0][2] -= m._mtx[0][2];
        _mtx[1][0] -= m._mtx[1][0];
        _mtx[1][1] -= m._mtx[1][1];
        _mtx[1][2] -= m._mtx[1][2];
        _mtx[2][0] -= m._mtx[2][0];
        _mtx[2][1] -= m._mtx[2][1];
        _mtx[2][2] -= m._mtx[2][2];
        return *this;
    }

    friend RrMat3d operator*(const RrMat3d &m1, const RrMat3d &m2)
    {
        RrMat3d tmp(m1);
        tmp *= m2;
        return tmp;
    }
    friend RrMat3d operator*(const RrMat3d &m1, double d)
    {
        RrMat3d m = m1;
        return m *= d;
    }
    friend RrMat3d operator*(double d, const RrMat3d &m) { return m * d; }
    friend RrMat3d operator+(const RrMat3d &m1, const RrMat3d &m2)
    {
        RrMat3d tmp(m1);
        tmp += m2;
        return tmp;
    }
    friend RrMat3d operator-(const RrMat3d &m1, const RrMat3d &m2)
    {
        RrMat3d tmp(m1);
        tmp -= m2;
        return tmp;
    }
    friend RrMat3d operator-(const RrMat3d &m)
    {
        return RrMat3d(-m._mtx[0][0], -m._mtx[0][1], -m._mtx[0][2],
                       -m._mtx[1][0], -m._mtx[1][1], -m._mtx[1][2],
                       -m._mtx[2][0], -m._mtx[2][1], -m._mtx[2][2]);
    }

    friend RrVec3d operator*(const RrVec3d &vec, const RrMat3d &m)
    {
        return RrVec3d(
            vec[0] * m._mtx[0][0] + vec[1] * m._mtx[1][0] +
                vec[2] * m._mtx[2][0],
            vec[0] * m._mtx[0][1] + vec[1] * m._mtx[1][1] +
                vec[2] * m._mtx[2][1],
            vec[0] * m._mtx[0][2] + vec[1] * m._mtx[1][2] +
                vec[2] * m._mtx[2][2]);
    }

    friend RrVec3d operator*(const RrMat3d &m, const RrVec3d &vec)
    {
        return RrVec3d(
            vec[0] * m._mtx[0][0] + vec[1] * m._mtx[0][1] +
                vec[2] * m._mtx[0][2],
            vec[0] * m._mtx[1][0] + vec[1] * m._mtx[1][1] +
                vec[2] * m._mtx[1][2],
            vec[0] * m._mtx[2][0] + vec[1] * m._mtx[2][1] +
                vec[2] * m._mtx[2][2]);
    }

    RrMat3d GetTranspose() const
    {
        RrMat3d transpose;
        transpose._mtx[0][0] = _mtx[0][0];
        transpose._mtx[0][1] = _mtx[1][0];
        transpose._mtx[0][2] = _mtx[2][0];
        transpose._mtx[1][0] = _mtx[0][1];
        transpose._mtx[1][1] = _mtx[1][1];
        transpose._mtx[1][2] = _mtx[2][1];
        transpose._mtx[2][0] = _mtx[0][2];
        transpose._mtx[2][1] = _mtx[1][2];
        transpose._mtx[2][2] = _mtx[2][2];
        return transpose;
    }

    double GetDeterminant() const
    {
        return (_mtx[0][0] * _mtx[1][1] * _mtx[2][2] +
                _mtx[0][1] * _mtx[1][2] * _mtx[2][0] +
                _mtx[0][2] * _mtx[1][0] * _mtx[2][1] -
                _mtx[0][0] * _mtx[1][2] * _mtx[2][1] -
                _mtx[0][1] * _mtx[1][0] * _mtx[2][2] -
                _mtx[0][2] * _mtx[1][1] * _mtx[2][0]);
    }

    RrMat3d GetInverse(double *detPtr = nullptr, double eps = 0) const
    {
        double a00, a01, a02, a10, a11, a12, a20, a21, a22;
        double det, rcp;

        a00 = _mtx[0][0];
        a01 = _mtx[0][1];
        a02 = _mtx[0][2];
        a10 = _mtx[1][0];
        a11 = _mtx[1][1];
        a12 = _mtx[1][2];
        a20 = _mtx[2][0];
        a21 = _mtx[2][1];
        a22 = _mtx[2][2];
        det = -(a02 * a11 * a20) + a01 * a12 * a20 + a02 * a10 * a21 -
              a00 * a12 * a21 - a01 * a10 * a22 + a00 * a11 * a22;

        if (detPtr) {
            *detPtr = det;
        }

        RrMat3d inverse;

        if (std::fabs(det) > eps) {
            rcp = 1.0 / det;
            inverse._mtx[0][0] = (-(a12 * a21) + a11 * a22) * rcp;
            inverse._mtx[0][1] = (a02 * a21 - a01 * a22) * rcp;
            inverse._mtx[0][2] = (-(a02 * a11) + a01 * a12) * rcp;
            inverse._mtx[1][0] = (a12 * a20 - a10 * a22) * rcp;
            inverse._mtx[1][1] = (-(a02 * a20) + a00 * a22) * rcp;
            inverse._mtx[1][2] = (a02 * a10 - a00 * a12) * rcp;
            inverse._mtx[2][0] = (-(a11 * a20) + a10 * a21) * rcp;
            inverse._mtx[2][1] = (a01 * a20 - a00 * a21) * rcp;
            inverse._mtx[2][2] = (-(a01 * a10) + a00 * a11) * rcp;
        } else {
            inverse.SetScale(FLT_MAX);
        }

        return inverse;
    }
};

struct RrMat4d {
    double _mtx[4][4];

    RrMat4d() = default;
    RrMat4d(double m00, double m01, double m02, double m03, double m10,
            double m11, double m12, double m13, double m20, double m21,
            double m22, double m23, double m30, double m31, double m32,
            double m33)
    {
        Set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23,
            m30, m31, m32, m33);
    }
    explicit RrMat4d(double s) { SetDiagonal(s); }
    RrMat4d(const RrRotation &rotate, const RrVec3d &translate);
    RrMat4d(const RrMat3d &rotmx, const RrVec3d &translate)
    {
        SetTransform(rotmx, translate);
    }

    RrMat4d &Set(double m00, double m01, double m02, double m03,
                 double m10, double m11, double m12, double m13,
                 double m20, double m21, double m22, double m23,
                 double m30, double m31, double m32, double m33)
    {
        _mtx[0][0] = m00;
        _mtx[0][1] = m01;
        _mtx[0][2] = m02;
        _mtx[0][3] = m03;
        _mtx[1][0] = m10;
        _mtx[1][1] = m11;
        _mtx[1][2] = m12;
        _mtx[1][3] = m13;
        _mtx[2][0] = m20;
        _mtx[2][1] = m21;
        _mtx[2][2] = m22;
        _mtx[2][3] = m23;
        _mtx[3][0] = m30;
        _mtx[3][1] = m31;
        _mtx[3][2] = m32;
        _mtx[3][3] = m33;
        return *this;
    }

    RrMat4d &SetDiagonal(double s)
    {
        _mtx[0][0] = s;
        _mtx[0][1] = 0.0;
        _mtx[0][2] = 0.0;
        _mtx[0][3] = 0.0;
        _mtx[1][0] = 0.0;
        _mtx[1][1] = s;
        _mtx[1][2] = 0.0;
        _mtx[1][3] = 0.0;
        _mtx[2][0] = 0.0;
        _mtx[2][1] = 0.0;
        _mtx[2][2] = s;
        _mtx[2][3] = 0.0;
        _mtx[3][0] = 0.0;
        _mtx[3][1] = 0.0;
        _mtx[3][2] = 0.0;
        _mtx[3][3] = s;
        return *this;
    }

    RrMat4d &SetIdentity() { return SetDiagonal(1.0); }

    RrMat4d &SetScale(double s)
    {
        _mtx[0][0] = s;   _mtx[0][1] = 0.0; _mtx[0][2] = 0.0; _mtx[0][3] = 0.0;
        _mtx[1][0] = 0.0; _mtx[1][1] = s;   _mtx[1][2] = 0.0; _mtx[1][3] = 0.0;
        _mtx[2][0] = 0.0; _mtx[2][1] = 0.0; _mtx[2][2] = s;   _mtx[2][3] = 0.0;
        _mtx[3][0] = 0.0; _mtx[3][1] = 0.0; _mtx[3][2] = 0.0; _mtx[3][3] = 1.0;

        return *this;
    }

    void SetRow(int i, const RrVec4d &v)
    {
        _mtx[i][0] = v[0];
        _mtx[i][1] = v[1];
        _mtx[i][2] = v[2];
        _mtx[i][3] = v[3];
    }
    RrVec4d GetRow(int i) const
    {
        return RrVec4d(_mtx[i][0], _mtx[i][1], _mtx[i][2], _mtx[i][3]);
    }
    void SetColumn(int i, const RrVec4d &v)
    {
        _mtx[0][i] = v[0];
        _mtx[1][i] = v[1];
        _mtx[2][i] = v[2];
        _mtx[3][i] = v[3];
    }
    RrVec4d GetColumn(int i) const
    {
        return RrVec4d(_mtx[0][i], _mtx[1][i], _mtx[2][i], _mtx[3][i]);
    }

    double *operator[](size_t i) { return _mtx[i]; }
    const double *operator[](size_t i) const { return _mtx[i]; }

    bool operator==(const RrMat4d &m) const
    {
        return (_mtx[0][0] == m._mtx[0][0] &&
                _mtx[0][1] == m._mtx[0][1] &&
                _mtx[0][2] == m._mtx[0][2] &&
                _mtx[0][3] == m._mtx[0][3] &&
                _mtx[1][0] == m._mtx[1][0] &&
                _mtx[1][1] == m._mtx[1][1] &&
                _mtx[1][2] == m._mtx[1][2] &&
                _mtx[1][3] == m._mtx[1][3] &&
                _mtx[2][0] == m._mtx[2][0] &&
                _mtx[2][1] == m._mtx[2][1] &&
                _mtx[2][2] == m._mtx[2][2] &&
                _mtx[2][3] == m._mtx[2][3] &&
                _mtx[3][0] == m._mtx[3][0] &&
                _mtx[3][1] == m._mtx[3][1] &&
                _mtx[3][2] == m._mtx[3][2] &&
                _mtx[3][3] == m._mtx[3][3]);
    }
    bool operator!=(const RrMat4d &m) const { return !(*this == m); }

    RrMat4d &operator*=(const RrMat4d &m)
    {
        RrMat4d tmp = *this;

        _mtx[0][0] = tmp._mtx[0][0] * m._mtx[0][0] +
                     tmp._mtx[0][1] * m._mtx[1][0] +
                     tmp._mtx[0][2] * m._mtx[2][0] +
                     tmp._mtx[0][3] * m._mtx[3][0];

        _mtx[0][1] = tmp._mtx[0][0] * m._mtx[0][1] +
                     tmp._mtx[0][1] * m._mtx[1][1] +
                     tmp._mtx[0][2] * m._mtx[2][1] +
                     tmp._mtx[0][3] * m._mtx[3][1];

        _mtx[0][2] = tmp._mtx[0][0] * m._mtx[0][2] +
                     tmp._mtx[0][1] * m._mtx[1][2] +
                     tmp._mtx[0][2] * m._mtx[2][2] +
                     tmp._mtx[0][3] * m._mtx[3][2];

        _mtx[0][3] = tmp._mtx[0][0] * m._mtx[0][3] +
                     tmp._mtx[0][1] * m._mtx[1][3] +
                     tmp._mtx[0][2] * m._mtx[2][3] +
                     tmp._mtx[0][3] * m._mtx[3][3];

        _mtx[1][0] = tmp._mtx[1][0] * m._mtx[0][0] +
                     tmp._mtx[1][1] * m._mtx[1][0] +
                     tmp._mtx[1][2] * m._mtx[2][0] +
                     tmp._mtx[1][3] * m._mtx[3][0];

        _mtx[1][1] = tmp._mtx[1][0] * m._mtx[0][1] +
                     tmp._mtx[1][1] * m._mtx[1][1] +
                     tmp._mtx[1][2] * m._mtx[2][1] +
                     tmp._mtx[1][3] * m._mtx[3][1];

        _mtx[1][2] = tmp._mtx[1][0] * m._mtx[0][2] +
                     tmp._mtx[1][1] * m._mtx[1][2] +
                     tmp._mtx[1][2] * m._mtx[2][2] +
                     tmp._mtx[1][3] * m._mtx[3][2];

        _mtx[1][3] = tmp._mtx[1][0] * m._mtx[0][3] +
                     tmp._mtx[1][1] * m._mtx[1][3] +
                     tmp._mtx[1][2] * m._mtx[2][3] +
                     tmp._mtx[1][3] * m._mtx[3][3];

        _mtx[2][0] = tmp._mtx[2][0] * m._mtx[0][0] +
                     tmp._mtx[2][1] * m._mtx[1][0] +
                     tmp._mtx[2][2] * m._mtx[2][0] +
                     tmp._mtx[2][3] * m._mtx[3][0];

        _mtx[2][1] = tmp._mtx[2][0] * m._mtx[0][1] +
                     tmp._mtx[2][1] * m._mtx[1][1] +
                     tmp._mtx[2][2] * m._mtx[2][1] +
                     tmp._mtx[2][3] * m._mtx[3][1];

        _mtx[2][2] = tmp._mtx[2][0] * m._mtx[0][2] +
                     tmp._mtx[2][1] * m._mtx[1][2] +
                     tmp._mtx[2][2] * m._mtx[2][2] +
                     tmp._mtx[2][3] * m._mtx[3][2];

        _mtx[2][3] = tmp._mtx[2][0] * m._mtx[0][3] +
                     tmp._mtx[2][1] * m._mtx[1][3] +
                     tmp._mtx[2][2] * m._mtx[2][3] +
                     tmp._mtx[2][3] * m._mtx[3][3];

        _mtx[3][0] = tmp._mtx[3][0] * m._mtx[0][0] +
                     tmp._mtx[3][1] * m._mtx[1][0] +
                     tmp._mtx[3][2] * m._mtx[2][0] +
                     tmp._mtx[3][3] * m._mtx[3][0];

        _mtx[3][1] = tmp._mtx[3][0] * m._mtx[0][1] +
                     tmp._mtx[3][1] * m._mtx[1][1] +
                     tmp._mtx[3][2] * m._mtx[2][1] +
                     tmp._mtx[3][3] * m._mtx[3][1];

        _mtx[3][2] = tmp._mtx[3][0] * m._mtx[0][2] +
                     tmp._mtx[3][1] * m._mtx[1][2] +
                     tmp._mtx[3][2] * m._mtx[2][2] +
                     tmp._mtx[3][3] * m._mtx[3][2];

        _mtx[3][3] = tmp._mtx[3][0] * m._mtx[0][3] +
                     tmp._mtx[3][1] * m._mtx[1][3] +
                     tmp._mtx[3][2] * m._mtx[2][3] +
                     tmp._mtx[3][3] * m._mtx[3][3];

        return *this;
    }

    RrMat4d &operator*=(double d)
    {
        _mtx[0][0] *= d;
        _mtx[0][1] *= d;
        _mtx[0][2] *= d;
        _mtx[0][3] *= d;
        _mtx[1][0] *= d;
        _mtx[1][1] *= d;
        _mtx[1][2] *= d;
        _mtx[1][3] *= d;
        _mtx[2][0] *= d;
        _mtx[2][1] *= d;
        _mtx[2][2] *= d;
        _mtx[2][3] *= d;
        _mtx[3][0] *= d;
        _mtx[3][1] *= d;
        _mtx[3][2] *= d;
        _mtx[3][3] *= d;
        return *this;
    }

    RrMat4d &operator+=(const RrMat4d &m)
    {
        _mtx[0][0] += m._mtx[0][0];
        _mtx[0][1] += m._mtx[0][1];
        _mtx[0][2] += m._mtx[0][2];
        _mtx[0][3] += m._mtx[0][3];
        _mtx[1][0] += m._mtx[1][0];
        _mtx[1][1] += m._mtx[1][1];
        _mtx[1][2] += m._mtx[1][2];
        _mtx[1][3] += m._mtx[1][3];
        _mtx[2][0] += m._mtx[2][0];
        _mtx[2][1] += m._mtx[2][1];
        _mtx[2][2] += m._mtx[2][2];
        _mtx[2][3] += m._mtx[2][3];
        _mtx[3][0] += m._mtx[3][0];
        _mtx[3][1] += m._mtx[3][1];
        _mtx[3][2] += m._mtx[3][2];
        _mtx[3][3] += m._mtx[3][3];
        return *this;
    }

    RrMat4d &operator-=(const RrMat4d &m)
    {
        _mtx[0][0] -= m._mtx[0][0];
        _mtx[0][1] -= m._mtx[0][1];
        _mtx[0][2] -= m._mtx[0][2];
        _mtx[0][3] -= m._mtx[0][3];
        _mtx[1][0] -= m._mtx[1][0];
        _mtx[1][1] -= m._mtx[1][1];
        _mtx[1][2] -= m._mtx[1][2];
        _mtx[1][3] -= m._mtx[1][3];
        _mtx[2][0] -= m._mtx[2][0];
        _mtx[2][1] -= m._mtx[2][1];
        _mtx[2][2] -= m._mtx[2][2];
        _mtx[2][3] -= m._mtx[2][3];
        _mtx[3][0] -= m._mtx[3][0];
        _mtx[3][1] -= m._mtx[3][1];
        _mtx[3][2] -= m._mtx[3][2];
        _mtx[3][3] -= m._mtx[3][3];
        return *this;
    }

    friend RrMat4d operator*(const RrMat4d &m1, const RrMat4d &m2)
    {
        RrMat4d tmp(m1);
        tmp *= m2;
        return tmp;
    }
    friend RrMat4d operator*(const RrMat4d &m1, double d)
    {
        RrMat4d m = m1;
        return m *= d;
    }
    friend RrMat4d operator*(double d, const RrMat4d &m) { return m * d; }
    friend RrMat4d operator+(const RrMat4d &m1, const RrMat4d &m2)
    {
        RrMat4d tmp(m1);
        tmp += m2;
        return tmp;
    }
    friend RrMat4d operator-(const RrMat4d &m1, const RrMat4d &m2)
    {
        RrMat4d tmp(m1);
        tmp -= m2;
        return tmp;
    }
    friend RrMat4d operator-(const RrMat4d &m)
    {
        return RrMat4d(
            -m._mtx[0][0], -m._mtx[0][1], -m._mtx[0][2], -m._mtx[0][3],
            -m._mtx[1][0], -m._mtx[1][1], -m._mtx[1][2], -m._mtx[1][3],
            -m._mtx[2][0], -m._mtx[2][1], -m._mtx[2][2], -m._mtx[2][3],
            -m._mtx[3][0], -m._mtx[3][1], -m._mtx[3][2], -m._mtx[3][3]);
    }

    friend RrVec4d operator*(const RrMat4d &m, const RrVec4d &vec)
    {
        return RrVec4d(
            vec[0] * m._mtx[0][0] + vec[1] * m._mtx[0][1] +
                vec[2] * m._mtx[0][2] + vec[3] * m._mtx[0][3],
            vec[0] * m._mtx[1][0] + vec[1] * m._mtx[1][1] +
                vec[2] * m._mtx[1][2] + vec[3] * m._mtx[1][3],
            vec[0] * m._mtx[2][0] + vec[1] * m._mtx[2][1] +
                vec[2] * m._mtx[2][2] + vec[3] * m._mtx[2][3],
            vec[0] * m._mtx[3][0] + vec[1] * m._mtx[3][1] +
                vec[2] * m._mtx[3][2] + vec[3] * m._mtx[3][3]);
    }

    friend RrVec4d operator*(const RrVec4d &vec, const RrMat4d &m)
    {
        return RrVec4d(
            vec[0] * m._mtx[0][0] + vec[1] * m._mtx[1][0] +
                vec[2] * m._mtx[2][0] + vec[3] * m._mtx[3][0],
            vec[0] * m._mtx[0][1] + vec[1] * m._mtx[1][1] +
                vec[2] * m._mtx[2][1] + vec[3] * m._mtx[3][1],
            vec[0] * m._mtx[0][2] + vec[1] * m._mtx[1][2] +
                vec[2] * m._mtx[2][2] + vec[3] * m._mtx[3][2],
            vec[0] * m._mtx[0][3] + vec[1] * m._mtx[1][3] +
                vec[2] * m._mtx[2][3] + vec[3] * m._mtx[3][3]);
    }

    RrMat4d GetTranspose() const
    {
        RrMat4d transpose;
        transpose._mtx[0][0] = _mtx[0][0];
        transpose._mtx[0][1] = _mtx[1][0];
        transpose._mtx[0][2] = _mtx[2][0];
        transpose._mtx[0][3] = _mtx[3][0];
        transpose._mtx[1][0] = _mtx[0][1];
        transpose._mtx[1][1] = _mtx[1][1];
        transpose._mtx[1][2] = _mtx[2][1];
        transpose._mtx[1][3] = _mtx[3][1];
        transpose._mtx[2][0] = _mtx[0][2];
        transpose._mtx[2][1] = _mtx[1][2];
        transpose._mtx[2][2] = _mtx[2][2];
        transpose._mtx[2][3] = _mtx[3][2];
        transpose._mtx[3][0] = _mtx[0][3];
        transpose._mtx[3][1] = _mtx[1][3];
        transpose._mtx[3][2] = _mtx[2][3];
        transpose._mtx[3][3] = _mtx[3][3];
        return transpose;
    }

    double GetDeterminant() const
    {
        return (-_mtx[0][3] * _GetDeterminant3(1, 2, 3, 0, 1, 2) +
                _mtx[1][3] * _GetDeterminant3(0, 2, 3, 0, 1, 2) -
                _mtx[2][3] * _GetDeterminant3(0, 1, 3, 0, 1, 2) +
                _mtx[3][3] * _GetDeterminant3(0, 1, 2, 0, 1, 2));
    }

    double GetDeterminant3() const
    {
        return _GetDeterminant3(0, 1, 2, 0, 1, 2);
    }

    double _GetDeterminant3(size_t row1, size_t row2, size_t row3,
                            size_t col1, size_t col2, size_t col3) const
    {
        return (_mtx[row1][col1] * _mtx[row2][col2] * _mtx[row3][col3] +
                _mtx[row1][col2] * _mtx[row2][col3] * _mtx[row3][col1] +
                _mtx[row1][col3] * _mtx[row2][col1] * _mtx[row3][col2] -
                _mtx[row1][col1] * _mtx[row2][col3] * _mtx[row3][col2] -
                _mtx[row1][col2] * _mtx[row2][col1] * _mtx[row3][col3] -
                _mtx[row1][col3] * _mtx[row2][col2] * _mtx[row3][col1]);
    }

    RrMat4d GetInverse(double *detPtr = nullptr, double eps = 0) const
    {
        double x00, x01, x02, x03;
        double x10, x11, x12, x13;
        double x20, x21, x22, x23;
        double x30, x31, x32, x33;
        double y01, y02, y03, y12, y13, y23;
        double z00, z10, z20, z30;
        double z01, z11, z21, z31;
        double z02, z03, z12, z13, z22, z23, z32, z33;

        // Pickle 1st two columns of matrix into registers
        x00 = _mtx[0][0];
        x01 = _mtx[0][1];
        x10 = _mtx[1][0];
        x11 = _mtx[1][1];
        x20 = _mtx[2][0];
        x21 = _mtx[2][1];
        x30 = _mtx[3][0];
        x31 = _mtx[3][1];

        // Compute all six 2x2 determinants of 1st two columns
        y01 = x00 * x11 - x10 * x01;
        y02 = x00 * x21 - x20 * x01;
        y03 = x00 * x31 - x30 * x01;
        y12 = x10 * x21 - x20 * x11;
        y13 = x10 * x31 - x30 * x11;
        y23 = x20 * x31 - x30 * x21;

        // Pickle 2nd two columns of matrix into registers
        x02 = _mtx[0][2];
        x03 = _mtx[0][3];
        x12 = _mtx[1][2];
        x13 = _mtx[1][3];
        x22 = _mtx[2][2];
        x23 = _mtx[2][3];
        x32 = _mtx[3][2];
        x33 = _mtx[3][3];

        // Compute all 3x3 cofactors for 2nd two columns
        z33 = x02 * y12 - x12 * y02 + x22 * y01;
        z23 = x12 * y03 - x32 * y01 - x02 * y13;
        z13 = x02 * y23 - x22 * y03 + x32 * y02;
        z03 = x22 * y13 - x32 * y12 - x12 * y23;
        z32 = x13 * y02 - x23 * y01 - x03 * y12;
        z22 = x03 * y13 - x13 * y03 + x33 * y01;
        z12 = x23 * y03 - x33 * y02 - x03 * y23;
        z02 = x13 * y23 - x23 * y13 + x33 * y12;

        // Compute all six 2x2 determinants of 2nd two columns
        y01 = x02 * x13 - x12 * x03;
        y02 = x02 * x23 - x22 * x03;
        y03 = x02 * x33 - x32 * x03;
        y12 = x12 * x23 - x22 * x13;
        y13 = x12 * x33 - x32 * x13;
        y23 = x22 * x33 - x32 * x23;

        // Compute all 3x3 cofactors for 1st two columns
        z30 = x11 * y02 - x21 * y01 - x01 * y12;
        z20 = x01 * y13 - x11 * y03 + x31 * y01;
        z10 = x21 * y03 - x31 * y02 - x01 * y23;
        z00 = x11 * y23 - x21 * y13 + x31 * y12;
        z31 = x00 * y12 - x10 * y02 + x20 * y01;
        z21 = x10 * y03 - x30 * y01 - x00 * y13;
        z11 = x00 * y23 - x20 * y03 + x30 * y02;
        z01 = x20 * y13 - x30 * y12 - x10 * y23;

        // Compute 4x4 determinant and its reciprocal.
        double det = x30 * z30 + x20 * z20 + x10 * z10 + x00 * z00;
        if (detPtr) {
            *detPtr = det;
        }

        RrMat4d inverse;

        if (std::fabs(det) > eps) {
            double rcp = 1.0 / det;
            // Multiply all 3x3 cofactors by reciprocal and transpose.
            inverse._mtx[0][0] = z00 * rcp;
            inverse._mtx[0][1] = z10 * rcp;
            inverse._mtx[1][0] = z01 * rcp;
            inverse._mtx[0][2] = z20 * rcp;
            inverse._mtx[2][0] = z02 * rcp;
            inverse._mtx[0][3] = z30 * rcp;
            inverse._mtx[3][0] = z03 * rcp;
            inverse._mtx[1][1] = z11 * rcp;
            inverse._mtx[1][2] = z21 * rcp;
            inverse._mtx[2][1] = z12 * rcp;
            inverse._mtx[1][3] = z31 * rcp;
            inverse._mtx[3][1] = z13 * rcp;
            inverse._mtx[2][2] = z22 * rcp;
            inverse._mtx[2][3] = z32 * rcp;
            inverse._mtx[3][2] = z23 * rcp;
            inverse._mtx[3][3] = z33 * rcp;
        } else {
            inverse.SetScale(FLT_MAX);
        }

        return inverse;
    }

    RrVec3d Transform(const RrVec3d &vec) const
    {
        double w = vec[0] * _mtx[0][3] + vec[1] * _mtx[1][3] +
                   vec[2] * _mtx[2][3] + _mtx[3][3];
        RrVec3d transformed(
            vec[0] * _mtx[0][0] + vec[1] * _mtx[1][0] +
                vec[2] * _mtx[2][0] + _mtx[3][0],
            vec[0] * _mtx[0][1] + vec[1] * _mtx[1][1] +
                vec[2] * _mtx[2][1] + _mtx[3][1],
            vec[0] * _mtx[0][2] + vec[1] * _mtx[1][2] +
                vec[2] * _mtx[2][2] + _mtx[3][2]);
        return transformed / w;
    }

    RrVec3d TransformDir(const RrVec3d &vec) const
    {
        return RrVec3d(vec[0] * _mtx[0][0] + vec[1] * _mtx[1][0] +
                           vec[2] * _mtx[2][0],
                       vec[0] * _mtx[0][1] + vec[1] * _mtx[1][1] +
                           vec[2] * _mtx[2][1],
                       vec[0] * _mtx[0][2] + vec[1] * _mtx[1][2] +
                           vec[2] * _mtx[2][2]);
    }

    RrVec3d TransformAffine(const RrVec3d &vec) const
    {
        return RrVec3d(vec[0] * _mtx[0][0] + vec[1] * _mtx[1][0] +
                           vec[2] * _mtx[2][0] + _mtx[3][0],
                       vec[0] * _mtx[0][1] + vec[1] * _mtx[1][1] +
                           vec[2] * _mtx[2][1] + _mtx[3][1],
                       vec[0] * _mtx[0][2] + vec[1] * _mtx[1][2] +
                           vec[2] * _mtx[2][2] + _mtx[3][2]);
    }


    RrVec3d ExtractTranslation() const
    {
        return RrVec3d(_mtx[3][0], _mtx[3][1], _mtx[3][2]);
    }

    RrMat3d ExtractRotationMatrix() const
    {
        return RrMat3d(_mtx[0][0], _mtx[0][1], _mtx[0][2], _mtx[1][0],
                       _mtx[1][1], _mtx[1][2], _mtx[2][0], _mtx[2][1],
                       _mtx[2][2]);
    }

    RrQuatd ExtractRotationQuat() const;

    RrRotation ExtractRotation() const;

    RrMat4d &SetTranslateOnly(const RrVec3d &t)
    {
        _mtx[3][0] = t[0];
        _mtx[3][1] = t[1];
        _mtx[3][2] = t[2];
        _mtx[3][3] = 1.0;
        return *this;
    }

    void _SetRotateFromQuat(double r, const RrVec3d &i)
    {
        _mtx[0][0] = 1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2]);
        _mtx[0][1] = 2.0 * (i[0] * i[1] + i[2] * r);
        _mtx[0][2] = 2.0 * (i[2] * i[0] - i[1] * r);

        _mtx[1][0] = 2.0 * (i[0] * i[1] - i[2] * r);
        _mtx[1][1] = 1.0 - 2.0 * (i[2] * i[2] + i[0] * i[0]);
        _mtx[1][2] = 2.0 * (i[1] * i[2] + i[0] * r);

        _mtx[2][0] = 2.0 * (i[2] * i[0] + i[1] * r);
        _mtx[2][1] = 2.0 * (i[1] * i[2] - i[0] * r);
        _mtx[2][2] = 1.0 - 2.0 * (i[1] * i[1] + i[0] * i[0]);
    }

    RrMat4d &SetRotateOnly(const RrQuatd &rot);
    RrMat4d &SetRotate(const RrQuatd &rot);
    RrMat4d &SetRotateOnly(const RrRotation &rot);
    RrMat4d &SetRotate(const RrRotation &rot);

    RrMat4d &SetRotate(const RrMat3d &mx3)
    {
        _mtx[0][0] = mx3[0][0];
        _mtx[0][1] = mx3[0][1];
        _mtx[0][2] = mx3[0][2];
        _mtx[0][3] = 0.0;

        _mtx[1][0] = mx3[1][0];
        _mtx[1][1] = mx3[1][1];
        _mtx[1][2] = mx3[1][2];
        _mtx[1][3] = 0.0;

        _mtx[2][0] = mx3[2][0];
        _mtx[2][1] = mx3[2][1];
        _mtx[2][2] = mx3[2][2];
        _mtx[2][3] = 0.0;

        _mtx[3][0] = 0.0;
        _mtx[3][1] = 0.0;
        _mtx[3][2] = 0.0;
        _mtx[3][3] = 1.0;

        return *this;
    }

    RrMat4d &SetRotateOnly(const RrMat3d &mx3)
    {
        _mtx[0][0] = mx3[0][0];
        _mtx[0][1] = mx3[0][1];
        _mtx[0][2] = mx3[0][2];

        _mtx[1][0] = mx3[1][0];
        _mtx[1][1] = mx3[1][1];
        _mtx[1][2] = mx3[1][2];

        _mtx[2][0] = mx3[2][0];
        _mtx[2][1] = mx3[2][1];
        _mtx[2][2] = mx3[2][2];

        return *this;
    }

    RrMat4d &SetTransform(const RrMat3d &rotate, const RrVec3d &translate)
    {
        SetRotate(rotate);
        return SetTranslateOnly(translate);
    }
    RrMat4d &SetTransform(const RrRotation &rotate,
                          const RrVec3d &translate);

    bool Orthonormalize(bool issueWarning = true);

    RrMat4d GetOrthonormalized(bool issueWarning = true) const
    {
        RrMat4d result = *this;
        result.Orthonormalize(issueWarning);
        return result;
    }
};

inline constexpr double kRrMinOrthoTolerance = 1e-6;

inline constexpr double kRrPi = 3.14159265358979323846;

inline double
RrRadiansToDegrees(double radians)
{
    return radians * (180.0 / kRrPi);
}

inline double
RrDegreesToRadians(double degrees)
{
    return degrees * (kRrPi / 180.0);
}

inline void
RrSinCos(double v, double *s, double *c)
{
#if defined(__linux__) && defined(__GLIBC__)
    ::sincos(v, s, c);
#else
    *s = std::sin(v);
    *c = std::cos(v);
#endif
}

inline bool
RrIsClose(double a, double b, double epsilon)
{
    return std::fabs(a - b) < epsilon;
}

inline bool
RrIsClose(const RrVec3d &a, const RrVec3d &b, double tolerance)
{
    RrVec3d delta = a - b;
    return delta.GetLengthSq() <= tolerance * tolerance;
}

inline double
RrSqr(double x)
{
    return x * x;
}

inline bool
RrOrthogonalizeBasis(RrVec3d *tx, RrVec3d *ty, RrVec3d *tz,
                     bool normalize, double eps = kRrMinOrthoTolerance);

struct RrQuatd {
    RrVec3d _imaginary;
    double _real;

    RrQuatd() = default;
    explicit RrQuatd(double real) : _imaginary(0), _real(real) {}
    RrQuatd(double real, const RrVec3d &imaginary)
        : _imaginary(imaginary), _real(real)
    {
    }

    void SetReal(double real) { _real = real; }
    void SetImaginary(const RrVec3d &imaginary) { _imaginary = imaginary; }
    void SetImaginary(double i, double j, double k)
    {
        _imaginary = RrVec3d(i, j, k);
    }

    double GetReal() const { return _real; }
    const RrVec3d &GetImaginary() const { return _imaginary; }

    static RrQuatd GetZero()
    {
        return RrQuatd(0.0, RrVec3d(0.0, 0.0, 0.0));
    }
    static RrQuatd GetIdentity()
    {
        return RrQuatd(1.0, RrVec3d(0.0, 0.0, 0.0));
    }

    double GetLength() const { return RrSqrt(_GetLengthSquared()); }

    RrQuatd GetNormalized(double eps = kRrMinVectorLength) const
    {
        RrQuatd ret(*this);
        ret.Normalize(eps);
        return ret;
    }

    double Normalize(double eps = kRrMinVectorLength)
    {
        double length = GetLength();
        if (length < eps) {
            *this = GetIdentity();
        } else {
            *this /= length;
        }
        return length;
    }

    RrQuatd GetConjugate() const
    {
        return RrQuatd(GetReal(), -GetImaginary());
    }

    RrQuatd GetInverse() const
    {
        return GetConjugate() / _GetLengthSquared();
    }

    RrVec3d Transform(const RrVec3d &point) const
    {
        double tmpDot = RrDot(_imaginary, _imaginary);
        double tmpSqr = _real * _real;
        return (2 * RrDot(_imaginary, point) * _imaginary +
                (tmpSqr - tmpDot) * point +
                2 * _real * RrCross(_imaginary, point)) /
               (tmpSqr + tmpDot);
    }

    RrQuatd operator-() const
    {
        return RrQuatd(-GetReal(), -GetImaginary());
    }

    bool operator==(const RrQuatd &q) const
    {
        return (GetReal() == q.GetReal() &&
                GetImaginary() == q.GetImaginary());
    }
    bool operator!=(const RrQuatd &q) const { return !(*this == q); }

    RrQuatd &operator*=(const RrQuatd &q)
    {
        double r1 = GetReal();
        double r2 = q.GetReal();
        const RrVec3d &i1 = GetImaginary();
        const RrVec3d &i2 = q.GetImaginary();

        double r = r1 * r2 - RrDot(i1, i2);

        RrVec3d i(r1 * i2[0] + r2 * i1[0] +
                      (i1[1] * i2[2] - i1[2] * i2[1]),
                  r1 * i2[1] + r2 * i1[1] +
                      (i1[2] * i2[0] - i1[0] * i2[2]),
                  r1 * i2[2] + r2 * i1[2] +
                      (i1[0] * i2[1] - i1[1] * i2[0]));

        SetReal(r);
        SetImaginary(i);

        return *this;
    }

    RrQuatd &operator*=(double s)
    {
        _real *= s;
        _imaginary *= s;
        return *this;
    }

    RrQuatd &operator/=(double s)
    {
        _real /= s;
        _imaginary /= s;
        return *this;
    }

    RrQuatd &operator+=(const RrQuatd &q)
    {
        _real += q._real;
        _imaginary += q._imaginary;
        return *this;
    }

    RrQuatd &operator-=(const RrQuatd &q)
    {
        _real -= q._real;
        _imaginary -= q._imaginary;
        return *this;
    }

    friend RrQuatd operator+(const RrQuatd &q1, const RrQuatd &q2)
    {
        return RrQuatd(q1) += q2;
    }
    friend RrQuatd operator-(const RrQuatd &q1, const RrQuatd &q2)
    {
        return RrQuatd(q1) -= q2;
    }
    friend RrQuatd operator*(const RrQuatd &q1, const RrQuatd &q2)
    {
        return RrQuatd(q1) *= q2;
    }
    friend RrQuatd operator*(const RrQuatd &q, double s)
    {
        return RrQuatd(q) *= s;
    }
    friend RrQuatd operator*(double s, const RrQuatd &q)
    {
        return RrQuatd(q) *= s;
    }
    friend RrQuatd operator/(const RrQuatd &q, double s)
    {
        return RrQuatd(q) /= s;
    }

    double _GetLengthSquared() const
    {
        return RrDot(_imaginary, _imaginary) + _real * _real;
    }
};

inline RrQuatd
RrSlerp(double alpha, const RrQuatd &q0, const RrQuatd &q1)
{
    double cosTheta = q0.GetImaginary() * q1.GetImaginary() +
                      q0.GetReal() * q1.GetReal();
    bool flip1 = false;

    if (cosTheta < 0.0) {
        cosTheta = -cosTheta;
        flip1 = true;
    }

    double scale0, scale1;

    if (1.0 - cosTheta > 0.00001) {
        double theta = std::acos(cosTheta), sinTheta = std::sin(theta);

        scale0 = std::sin((1.0 - alpha) * theta) / sinTheta;
        scale1 = std::sin(alpha * theta) / sinTheta;
    } else {
        scale0 = 1.0 - alpha;
        scale1 = alpha;
    }

    if (flip1) {
        scale1 = -scale1;
    }

    return scale0 * q0 + scale1 * q1;
}

inline RrQuatd
RrSlerp(const RrQuatd &q0, const RrQuatd &q1, double alpha)
{
    return RrSlerp(alpha, q0, q1);
}

struct RrRotation {
    RrVec3d _axis;
    double _angle;

    RrRotation() {}
    RrRotation(const RrVec3d &axis, double angle)
    {
        SetAxisAngle(axis, angle);
    }
    RrRotation(const RrQuatd &quat) { SetQuat(quat); }
    RrRotation(const RrVec3d &rotateFrom, const RrVec3d &rotateTo)
    {
        SetRotateInto(rotateFrom, rotateTo);
    }

    RrRotation &SetAxisAngle(const RrVec3d &axis, double angle)
    {
        _axis = axis;
        _angle = angle;
        if (!RrIsClose(_axis * _axis, 1.0, 1e-10)) {
            _axis.Normalize();
        }
        return *this;
    }

    RrRotation &SetQuat(const RrQuatd &quat)
    {
        double len = quat.GetImaginary().GetLength();
        if (len > kRrMinVectorLength) {
            double x = std::acos(RrClamp(quat.GetReal(), -1.0, 1.0));
            SetAxisAngle(quat.GetImaginary() / len,
                         2.0 * RrRadiansToDegrees(x));
        } else {
            SetIdentity();
        }
        return *this;
    }

    RrRotation &SetRotateInto(const RrVec3d &rotateFrom,
                              const RrVec3d &rotateTo)
    {
        RrVec3d from = rotateFrom.GetNormalized();
        RrVec3d to = rotateTo.GetNormalized();

        double cos = RrDot(from, to);

        if (cos > 0.9999999) {
            return SetIdentity();
        }

        if (cos < -0.9999999) {
            RrVec3d tmp = RrCross(from, RrVec3d(1.0, 0.0, 0.0));
            if (tmp.GetLength() < 0.00001) {
                tmp = RrCross(from, RrVec3d(0.0, 1.0, 0.0));
            }
            return SetAxisAngle(tmp.GetNormalized(), 180.0);
        }

        RrVec3d axis = RrCross(rotateFrom, rotateTo).GetNormalized();
        return SetAxisAngle(axis, RrRadiansToDegrees(std::acos(cos)));
    }

    RrRotation &SetIdentity()
    {
        _axis = RrVec3d(1.0, 0.0, 0.0);
        _angle = 0.0;
        return *this;
    }

    const RrVec3d &GetAxis() const { return _axis; }
    double GetAngle() const { return _angle; }

    RrQuatd GetQuat() const
    {
        double radians = RrDegreesToRadians(_angle) / 2.0;
        double sinR, cosR;
        RrSinCos(radians, &sinR, &cosR);
        RrVec3d axis = _axis * sinR;
        return RrQuatd(cosR, axis).GetNormalized();
    }

    RrRotation GetInverse() const
    {
        return RrRotation(_axis, -_angle);
    }

    RrVec3d Decompose(const RrVec3d &axis0, const RrVec3d &axis1,
                      const RrVec3d &axis2) const
    {
        RrMat4d mat;
        mat.SetRotate(*this);

        RrVec3d nAxis0 = axis0.GetNormalized();
        RrVec3d nAxis1 = axis1.GetNormalized();
        RrVec3d nAxis2 = axis2.GetNormalized();

        RrMat4d axes(nAxis0[0], nAxis1[0], nAxis2[0], 0, nAxis0[1],
                     nAxis1[1], nAxis2[1], 0, nAxis0[2], nAxis1[2],
                     nAxis2[2], 0, 0, 0, 0, 1);

        RrMat4d m = axes.GetTranspose() * mat * axes;

        int i = 0, j = 1, k = 2;
        double r0, r1, r2;
        double cy = std::sqrt(m[i][i] * m[i][i] + m[j][i] * m[j][i]);
        if (cy > 1e-6) {
            r0 = std::atan2(m[k][j], m[k][k]);
            r1 = std::atan2(-m[k][i], cy);
            r2 = std::atan2(m[j][i], m[i][i]);
        } else {
            r0 = std::atan2(-m[j][k], m[j][j]);
            r1 = std::atan2(-m[k][i], cy);
            r2 = 0;
        }

        RrVec3d axisCross = RrCross(nAxis0, nAxis1);
        double axisHand = RrDot(axisCross, nAxis2);
        if (axisHand >= 0.0) {
            r0 = -r0;
            r1 = -r1;
            r2 = -r2;
        }

        return RrVec3d(RrRadiansToDegrees(r0), RrRadiansToDegrees(r1),
                       RrRadiansToDegrees(r2));
    }

    RrVec3d TransformDir(const RrVec3d &vec) const
    {
        return RrMat4d().SetRotate(*this).TransformDir(vec);
    }
};

inline RrMat4d::RrMat4d(const RrRotation &rotate, const RrVec3d &translate)
{
    SetTransform(rotate, translate);
}

inline RrQuatd
RrMat4d::ExtractRotationQuat() const
{
    int i;

    if (_mtx[0][0] > _mtx[1][1]) {
        i = (_mtx[0][0] > _mtx[2][2] ? 0 : 2);
    } else {
        i = (_mtx[1][1] > _mtx[2][2] ? 1 : 2);
    }

    RrVec3d im;
    double r;

    if (_mtx[0][0] + _mtx[1][1] + _mtx[2][2] > _mtx[i][i]) {
        r = 0.5 * std::sqrt(_mtx[0][0] + _mtx[1][1] + _mtx[2][2] +
                            _mtx[3][3]);
        im = RrVec3d((_mtx[1][2] - _mtx[2][1]) / (4.0 * r),
                     (_mtx[2][0] - _mtx[0][2]) / (4.0 * r),
                     (_mtx[0][1] - _mtx[1][0]) / (4.0 * r));
    } else {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        double q = 0.5 * std::sqrt(_mtx[i][i] - _mtx[j][j] -
                                   _mtx[k][k] + _mtx[3][3]);

        im[i] = q;
        im[j] = (_mtx[i][j] + _mtx[j][i]) / (4 * q);
        im[k] = (_mtx[k][i] + _mtx[i][k]) / (4 * q);
        r = (_mtx[j][k] - _mtx[k][j]) / (4 * q);
    }

    return RrQuatd(RrClamp(r, -1.0, 1.0), im);
}

inline RrRotation
RrMat4d::ExtractRotation() const
{
    return RrRotation(ExtractRotationQuat());
}

inline RrMat4d &
RrMat4d::SetRotateOnly(const RrQuatd &rot)
{
    _SetRotateFromQuat(rot.GetReal(), rot.GetImaginary());
    return *this;
}

inline RrMat4d &
RrMat4d::SetRotate(const RrQuatd &rot)
{
    SetRotateOnly(rot);

    _mtx[0][3] = 0.0;
    _mtx[1][3] = 0.0;
    _mtx[2][3] = 0.0;

    _mtx[3][0] = 0.0;
    _mtx[3][1] = 0.0;
    _mtx[3][2] = 0.0;
    _mtx[3][3] = 1.0;

    return *this;
}

inline RrMat4d &
RrMat4d::SetRotateOnly(const RrRotation &rot)
{
    RrQuatd quat = rot.GetQuat();
    _SetRotateFromQuat(quat.GetReal(), quat.GetImaginary());
    return *this;
}

inline RrMat4d &
RrMat4d::SetRotate(const RrRotation &rot)
{
    SetRotateOnly(rot);

    _mtx[0][3] = 0.0;
    _mtx[1][3] = 0.0;
    _mtx[2][3] = 0.0;

    _mtx[3][0] = 0.0;
    _mtx[3][1] = 0.0;
    _mtx[3][2] = 0.0;
    _mtx[3][3] = 1.0;

    return *this;
}

inline RrMat4d &
RrMat4d::SetTransform(const RrRotation &rotate, const RrVec3d &translate)
{
    SetRotate(rotate);
    return SetTranslateOnly(translate);
}

inline bool
RrOrthogonalizeBasis(RrVec3d *tx, RrVec3d *ty, RrVec3d *tz,
                     bool normalize, double eps)
{
    RrVec3d ax, bx, cx, ay, by, cy, az, bz, cz;

    if (normalize) {
        tx->Normalize();
        ty->Normalize();
        tz->Normalize();
        ax = *tx;
        ay = *ty;
        az = *tz;
    } else {
        ax = *tx;
        ay = *ty;
        az = *tz;
        ax.Normalize();
        ay.Normalize();
        az.Normalize();
    }

    if (RrIsClose(ax, ay, eps) || RrIsClose(ax, az, eps) ||
        RrIsClose(ay, az, eps)) {
        return false;
    }

    const int MAX_ITERS = 20;
    int iter;
    for (iter = 0; iter < MAX_ITERS; ++iter) {
        bx = *tx;
        by = *ty;
        bz = *tz;

        bx -= RrDot(ay, bx) * ay;
        bx -= RrDot(az, bx) * az;

        by -= RrDot(ax, by) * ax;
        by -= RrDot(az, by) * az;

        bz -= RrDot(ax, bz) * ax;
        bz -= RrDot(ay, bz) * ay;

        cx = 0.5 * (*tx + bx);
        cy = 0.5 * (*ty + by);
        cz = 0.5 * (*tz + bz);

        if (normalize) {
            cx.Normalize();
            cy.Normalize();
            cz.Normalize();
        }

        RrVec3d xDiff = *tx - cx;
        RrVec3d yDiff = *ty - cy;
        RrVec3d zDiff = *tz - cz;

        double error = RrDot(xDiff, xDiff) + RrDot(yDiff, yDiff) +
                       RrDot(zDiff, zDiff);

        if (error < RrSqr(eps)) {
            break;
        }

        *tx = cx;
        *ty = cy;
        *tz = cz;

        ax = *tx;
        ay = *ty;
        az = *tz;

        if (!normalize) {
            ax.Normalize();
            ay.Normalize();
            az.Normalize();
        }
    }

    return iter < MAX_ITERS;
}

inline bool
RrMat4d::Orthonormalize(bool issueWarning)
{
    (void)issueWarning;
    RrVec3d r0(_mtx[0][0], _mtx[0][1], _mtx[0][2]);
    RrVec3d r1(_mtx[1][0], _mtx[1][1], _mtx[1][2]);
    RrVec3d r2(_mtx[2][0], _mtx[2][1], _mtx[2][2]);

    bool result = RrOrthogonalizeBasis(&r0, &r1, &r2, true);

    _mtx[0][0] = r0[0];
    _mtx[0][1] = r0[1];
    _mtx[0][2] = r0[2];
    _mtx[1][0] = r1[0];
    _mtx[1][1] = r1[1];
    _mtx[1][2] = r1[2];
    _mtx[2][0] = r2[0];
    _mtx[2][1] = r2[1];
    _mtx[2][2] = r2[2];

    // Divide out any homogeneous coordinate - unless it's zero.
    if (_mtx[3][3] != 1.0 &&
        !RrIsClose(_mtx[3][3], 0.0, kRrMinVectorLength)) {
        _mtx[3][0] /= _mtx[3][3];
        _mtx[3][1] /= _mtx[3][3];
        _mtx[3][2] /= _mtx[3][3];
        _mtx[3][3] = 1.0;
    }

    return result;
}
// ---------------------------------------------------------------------------
// Chunk 11: GfCamera-compatible frustum math
// ---------------------------------------------------------------------------

// Fit-mode policy mirroring GfCamera::FOVDirection.
enum RrCameraFitMode {
    RrCameraFitHorizontal,
    RrCameraFitVertical,
    RrCameraFitFree
};

// Minimal GfCamera-compatible parameter block.
struct RrCamera {
    double focalLength = 50.0;
    double horizontalAperture = 20.955;
    double verticalAperture = 15.2908;
    double horizontalApertureOffset = 0.0;
    double verticalApertureOffset = 0.0;
    double nearDistance = 1.0;
    double farDistance = 100.0;
    RrVec3d up = RrVec3d::ZAxis();
    RrCameraFitMode fitMode = RrCameraFitHorizontal;
};

inline void
RrCameraFrustumWindow(const RrCamera &cam, double aspect,
                      RrVec2d *topLeft, RrVec2d *bottomRight)
{
    RrVec2d top(0.0), bottom(0.0);
    if (cam.fitMode == RrCameraFitVertical) {
        top = RrVec2d(cam.horizontalApertureOffset +
                          aspect * 0.5 * cam.verticalAperture,
                      cam.verticalApertureOffset +
                          0.5 * cam.verticalAperture);
        bottom = RrVec2d(cam.horizontalApertureOffset -
                             aspect * 0.5 * cam.verticalAperture,
                         cam.verticalApertureOffset -
                             0.5 * cam.verticalAperture);
    } else if (cam.fitMode == RrCameraFitHorizontal) {
        top = RrVec2d(cam.horizontalApertureOffset +
                          0.5 * cam.horizontalAperture,
                      cam.verticalApertureOffset +
                          0.5 * cam.verticalAperture / aspect);
        bottom = RrVec2d(cam.horizontalApertureOffset -
                             0.5 * cam.horizontalAperture,
                         cam.verticalApertureOffset -
                             0.5 * cam.verticalAperture / aspect);
    } else {
        top = RrVec2d(cam.horizontalApertureOffset +
                          0.5 * cam.horizontalAperture,
                      cam.verticalApertureOffset +
                          0.5 * cam.verticalAperture);
        bottom = RrVec2d(cam.horizontalApertureOffset -
                             0.5 * cam.horizontalAperture,
                         cam.verticalApertureOffset -
                             0.5 * cam.verticalAperture);
    }
    const double mult =
        -cam.nearDistance / (cam.focalLength * 0.1);
    *topLeft = mult * top;
    *bottomRight = mult * bottom;
}

// GfCamera::SetFromPositionAndTargetUp_vectors direction convention.
inline RrVec3d
RrCameraForward(const RrVec3d &position, const RrVec3d &target)
{
    RrVec3d dir = target - position;
    dir.Normalize();
    return dir;
}

inline void
RrCameraBasis(const RrVec3d &forward, const RrVec3d &up,
              RrVec3d *side, RrVec3d *trueUp)
{
    *side = RrCross(forward, up);
    side->Normalize();
    *trueUp = RrCross(*side, forward);
    trueUp->Normalize();
}

// GfFrustum::ComputeViewFrame look-at matrix convention.
inline RrMat4d
RrFrustumViewFrame(const RrVec3d &position, const RrRotation &rotation)
{
    RrMat4d view;
    view.SetIdentity();
    view.SetRotate(rotation);
    view.SetTranslateOnly(position);
    return view.GetInverse();
}

// UsdGeomXformCommonAPI-compatible look-at orientation.
inline RrRotation
RrLookAtRotation(const RrVec3d &forward, const RrVec3d &up)
{
    RrVec3d side, trueUp;
    RrCameraBasis(forward, up, &side, &trueUp);
    RrMat4d basis;
    basis.SetIdentity();
    basis[0][0] = side[0];
    basis[0][1] = side[1];
    basis[0][2] = side[2];
    basis[1][0] = trueUp[0];
    basis[1][1] = trueUp[1];
    basis[1][2] = trueUp[2];
    basis[2][0] = -forward[0];
    basis[2][1] = -forward[1];
    basis[2][2] = -forward[2];
    return basis.ExtractRotation();
}

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_MATH_H


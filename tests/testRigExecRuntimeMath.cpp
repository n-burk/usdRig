//
// testRigExecRuntimeMath: bitwise equivalence of the vendored runtime math
// against Gf. USD is linked into the TEST only -- never into the runtime.
//
// Every comparison is exact (bitwise, including signed zeros and NaN
// payloads): the runtime must perform the SAME operations in the SAME
// order as Gf. Randomized inputs (fixed seed) plus edge cases.
//

#include "rigExecRuntime/runtimeMath.h"

#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec2d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec4d.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int _failures = 0;
std::string _context;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            ++_failures;                                                    \
            std::printf("FAIL %s:%d [%s]: %s\n", __FILE__, __LINE__,         \
                        _context.c_str(), #cond);                           \
        }                                                                   \
    } while (0)

uint64_t _rngState = 0x243F6A8885A308D3ull;

uint64_t
_NextBits()
{
    uint64_t x = _rngState;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    _rngState = x;
    return x * 0x2545F4914F6CDD1Dull;
}

double
_RandDouble(double lo = -10.0, double hi = 10.0)
{
    const double t =
        (double)(_NextBits() >> 11) * (1.0 / 9007199254740992.0);
    return lo + t * (hi - lo);
}

float
_RandFloat(float lo = -10.0f, float hi = 10.0f)
{
    return float(_RandDouble(lo, hi));
}

bool
_SameBits(double a, double b)
{
    uint64_t ua, ub;
    std::memcpy(&ua, &a, sizeof(ua));
    std::memcpy(&ub, &b, sizeof(ub));
    return ua == ub;
}

bool
_SameBits(float a, float b)
{
    uint32_t ua, ub;
    std::memcpy(&ua, &a, sizeof(ua));
    std::memcpy(&ub, &b, sizeof(ub));
    return ua == ub;
}

bool
_SameVec3d(const GfVec3d &a, const rigExec::RrVec3d &b)
{
    return _SameBits(a[0], b[0]) && _SameBits(a[1], b[1]) &&
           _SameBits(a[2], b[2]);
}

bool
_SameVec3f(const GfVec3f &a, const rigExec::RrVec3f &b)
{
    return _SameBits(a[0], b[0]) && _SameBits(a[1], b[1]) &&
           _SameBits(a[2], b[2]);
}

bool
_SameVec2d(const GfVec2d &a, const rigExec::RrVec2d &b)
{
    return _SameBits(a[0], b[0]) && _SameBits(a[1], b[1]);
}

bool
_SameVec2f(const GfVec2f &a, const rigExec::RrVec2f &b)
{
    return _SameBits(a[0], b[0]) && _SameBits(a[1], b[1]);
}

bool
_SameVec4d(const GfVec4d &a, const rigExec::RrVec4d &b)
{
    return _SameBits(a[0], b[0]) && _SameBits(a[1], b[1]) &&
           _SameBits(a[2], b[2]) && _SameBits(a[3], b[3]);
}

bool
_SameMat3d(const GfMatrix3d &a, const rigExec::RrMat3d &b)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!_SameBits(a[r][c], b[r][c])) {
                return false;
            }
        }
    }
    return true;
}

bool
_SameMat4d(const GfMatrix4d &a, const rigExec::RrMat4d &b)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!_SameBits(a[r][c], b[r][c])) {
                return false;
            }
        }
    }
    return true;
}

bool
_SameQuatd(const GfQuatd &a, const rigExec::RrQuatd &b)
{
    return _SameBits(a.GetReal(), b.GetReal()) &&
           _SameVec3d(a.GetImaginary(), b.GetImaginary());
}

bool
_SameRotation(const GfRotation &a, const rigExec::RrRotation &b)
{
    return _SameVec3d(a.GetAxis(), b.GetAxis()) &&
           _SameBits(a.GetAngle(), b.GetAngle());
}

GfVec3d
_RandGfVec3d()
{
    return GfVec3d(_RandDouble(), _RandDouble(), _RandDouble());
}

rigExec::RrVec3d
_ToRr(const GfVec3d &v)
{
    return rigExec::RrVec3d(v[0], v[1], v[2]);
}

GfVec3f
_RandGfVec3f()
{
    return GfVec3f(_RandFloat(), _RandFloat(), _RandFloat());
}

rigExec::RrVec3f
_ToRr(const GfVec3f &v)
{
    return rigExec::RrVec3f(v[0], v[1], v[2]);
}

GfVec2d
_RandGfVec2d()
{
    return GfVec2d(_RandDouble(), _RandDouble());
}

rigExec::RrVec2d
_ToRr(const GfVec2d &v)
{
    return rigExec::RrVec2d(v[0], v[1]);
}

GfVec2f
_RandGfVec2f()
{
    return GfVec2f(_RandFloat(), _RandFloat());
}

rigExec::RrVec2f
_ToRr(const GfVec2f &v)
{
    return rigExec::RrVec2f(v[0], v[1]);
}

GfVec4d
_RandGfVec4d()
{
    return GfVec4d(_RandDouble(), _RandDouble(), _RandDouble(),
                   _RandDouble());
}

rigExec::RrVec4d
_ToRr(const GfVec4d &v)
{
    return rigExec::RrVec4d(v[0], v[1], v[2], v[3]);
}

GfMatrix3d
_RandGfMat3d()
{
    return GfMatrix3d(_RandDouble(), _RandDouble(), _RandDouble(),
                      _RandDouble(), _RandDouble(), _RandDouble(),
                      _RandDouble(), _RandDouble(), _RandDouble());
}

rigExec::RrMat3d
_ToRr(const GfMatrix3d &m)
{
    return rigExec::RrMat3d(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1],
                            m[1][2], m[2][0], m[2][1], m[2][2]);
}

GfMatrix4d
_RandGfMat4d()
{
    return GfMatrix4d(
        _RandDouble(), _RandDouble(), _RandDouble(), _RandDouble(),
        _RandDouble(), _RandDouble(), _RandDouble(), _RandDouble(),
        _RandDouble(), _RandDouble(), _RandDouble(), _RandDouble(),
        _RandDouble(), _RandDouble(), _RandDouble(), _RandDouble());
}

rigExec::RrMat4d
_ToRr(const GfMatrix4d &m)
{
    return rigExec::RrMat4d(
        m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2],
        m[1][3], m[2][0], m[2][1], m[2][2], m[2][3], m[3][0], m[3][1],
        m[3][2], m[3][3]);
}

GfQuatd
_RandGfQuatd()
{
    return GfQuatd(_RandDouble(), _RandGfVec3d());
}

rigExec::RrQuatd
_ToRr(const GfQuatd &q)
{
    return rigExec::RrQuatd(q.GetReal(), _ToRr(q.GetImaginary()));
}

void
_TestVec3d()
{
    _context = "vec3d";
    for (int i = 0; i < 2000; ++i) {
        const GfVec3d a = _RandGfVec3d();
        const GfVec3d b = _RandGfVec3d();
        const double s = _RandDouble(-3.0, 3.0);
        const rigExec::RrVec3d ra = _ToRr(a);
        const rigExec::RrVec3d rb = _ToRr(b);
        CHECK(_SameVec3d(a + b, ra + rb));
        CHECK(_SameVec3d(a - b, ra - rb));
        CHECK(_SameVec3d(-a, -ra));
        CHECK(_SameVec3d(a * s, ra * s));
        CHECK(_SameVec3d(s * a, s * ra));
        CHECK(_SameBits(a * b, ra * rb));
        CHECK(_SameVec3d(a ^ b, ra ^ rb));
        CHECK(_SameVec3d(GfCross(a, b), rigExec::RrCross(ra, rb)));
        CHECK(_SameBits(GfDot(a, b), rigExec::RrDot(ra, rb)));
        CHECK(_SameBits(a.GetLengthSq(), ra.GetLengthSq()));
        CHECK(_SameBits(a.GetLength(), ra.GetLength()));
        CHECK(_SameVec3d(a.GetNormalized(), ra.GetNormalized()));
        CHECK((a == b) == (ra == rb));
        CHECK((a != b) == (ra != rb));
        {
            GfVec3d ma = a;
            rigExec::RrVec3d mr = ra;
            CHECK(_SameBits(ma.Normalize(), mr.Normalize()));
            CHECK(_SameVec3d(ma, mr));
        }
        if (s != 0.0) {
            CHECK(_SameVec3d(a / s, ra / s));
        }
    }
    // Edges: zeros (eps path), tiny straddling 1e-10, huge, signed zero.
    const std::vector<GfVec3d> edges = {
        GfVec3d(0), GfVec3d(1e-11), GfVec3d(1e-10), GfVec3d(1e-9),
        GfVec3d(1e15, -1e15, 1e-15), GfVec3d(-0.0, 0.0, -1.0)};
    for (const GfVec3d &a : edges) {
        const rigExec::RrVec3d ra = _ToRr(a);
        CHECK(_SameBits(a.GetLengthSq(), ra.GetLengthSq()));
        CHECK(_SameBits(a.GetLength(), ra.GetLength()));
        CHECK(_SameVec3d(a.GetNormalized(), ra.GetNormalized()));
        GfVec3d ma = a;
        rigExec::RrVec3d mr = ra;
        CHECK(_SameBits(ma.Normalize(), mr.Normalize()));
        CHECK(_SameVec3d(ma, mr));
        CHECK(_SameVec3d(a ^ a, ra ^ ra));
    }
}

void
_TestVec3f()
{
    _context = "vec3f";
    for (int i = 0; i < 2000; ++i) {
        const GfVec3f a = _RandGfVec3f();
        const GfVec3f b = _RandGfVec3f();
        const double s = _RandDouble(-3.0, 3.0);
        const rigExec::RrVec3f ra = _ToRr(a);
        const rigExec::RrVec3f rb = _ToRr(b);
        CHECK(_SameVec3f(a + b, ra + rb));
        CHECK(_SameVec3f(a - b, ra - rb));
        CHECK(_SameVec3f(-a, -ra));
        CHECK(_SameVec3f(a * s, ra * s));
        CHECK(_SameVec3f(s * a, s * ra));
        CHECK(_SameBits(a * b, ra * rb));
        CHECK(_SameVec3f(a ^ b, ra ^ rb));
        CHECK(_SameVec3f(GfCross(a, b), rigExec::RrCross(ra, rb)));
        CHECK(_SameBits(GfDot(a, b), rigExec::RrDot(ra, rb)));
        CHECK(_SameBits(a.GetLengthSq(), ra.GetLengthSq()));
        CHECK(_SameBits(a.GetLength(), ra.GetLength()));
        CHECK(_SameVec3f(a.GetNormalized(), ra.GetNormalized()));
        CHECK((a == b) == (ra == rb));
        {
            GfVec3f ma = a;
            rigExec::RrVec3f mr = ra;
            CHECK(_SameBits(ma.Normalize(), mr.Normalize()));
            CHECK(_SameVec3f(ma, mr));
        }
        if (s != 0.0) {
            CHECK(_SameVec3f(a / s, ra / s));
        }
    }
    const std::vector<GfVec3f> edges = {
        GfVec3f(0), GfVec3f(1e-11f), GfVec3f(1e-9f), GfVec3f(1e20f)};
    for (const GfVec3f &a : edges) {
        const rigExec::RrVec3f ra = _ToRr(a);
        CHECK(_SameVec3f(a.GetNormalized(), ra.GetNormalized()));
        GfVec3f ma = a;
        rigExec::RrVec3f mr = ra;
        CHECK(_SameBits(ma.Normalize(), mr.Normalize()));
        CHECK(_SameVec3f(ma, mr));
    }
}

void
_TestVec2()
{
    _context = "vec2";
    for (int i = 0; i < 1000; ++i) {
        const GfVec2d a = _RandGfVec2d();
        const GfVec2d b = _RandGfVec2d();
        const double s = _RandDouble(-3.0, 3.0);
        const rigExec::RrVec2d ra = _ToRr(a);
        const rigExec::RrVec2d rb = _ToRr(b);
        CHECK(_SameVec2d(a + b, ra + rb));
        CHECK(_SameVec2d(a - b, ra - rb));
        CHECK(_SameVec2d(a * s, ra * s));
        CHECK(_SameBits(a * b, ra * rb));
        CHECK(_SameBits(GfDot(a, b), rigExec::RrDot(ra, rb)));
        CHECK(_SameBits(a.GetLength(), ra.GetLength()));
        CHECK(_SameVec2d(a.GetNormalized(), ra.GetNormalized()));
        CHECK((a == b) == (ra == rb));
        if (s != 0.0) {
            CHECK(_SameVec2d(a / s, ra / s));
        }
        const GfVec2f fa = _RandGfVec2f();
        const GfVec2f fb = _RandGfVec2f();
        const rigExec::RrVec2f fra = _ToRr(fa);
        const rigExec::RrVec2f frb = _ToRr(fb);
        CHECK(_SameVec2f(fa + fb, fra + frb));
        CHECK(_SameVec2f(fa - fb, fra - frb));
        CHECK(_SameVec2f(fa * s, fra * s));
        CHECK(_SameBits(fa * fb, fra * frb));
        CHECK(_SameBits(fa.GetLength(), fra.GetLength()));
        CHECK(_SameVec2f(fa.GetNormalized(), fra.GetNormalized()));
    }
    const GfVec2d z(0);
    const rigExec::RrVec2d rz = _ToRr(z);
    CHECK(_SameVec2d(z.GetNormalized(), rz.GetNormalized()));
    const GfVec2f fz(0);
    const rigExec::RrVec2f frz = _ToRr(fz);
    CHECK(_SameVec2f(fz.GetNormalized(), frz.GetNormalized()));
}

void
_TestVec4d()
{
    _context = "vec4d";
    for (int i = 0; i < 1000; ++i) {
        const GfVec4d a = _RandGfVec4d();
        const GfVec4d b = _RandGfVec4d();
        const double s = _RandDouble(-3.0, 3.0);
        const rigExec::RrVec4d ra = _ToRr(a);
        const rigExec::RrVec4d rb = _ToRr(b);
        CHECK(_SameVec4d(a + b, ra + rb));
        CHECK(_SameVec4d(a - b, ra - rb));
        CHECK(_SameVec4d(a * s, ra * s));
        CHECK(_SameBits(a * b, ra * rb));
        CHECK(_SameBits(a.GetLength(), ra.GetLength()));
        CHECK(_SameVec4d(a.GetNormalized(), ra.GetNormalized()));
        CHECK((a == b) == (ra == rb));
        if (s != 0.0) {
            CHECK(_SameVec4d(a / s, ra / s));
        }
    }
}

void
_TestFree()
{
    _context = "free";
    for (int i = 0; i < 2000; ++i) {
        const double v = _RandDouble(-5.0, 5.0);
        const double lo = _RandDouble(-5.0, 0.0);
        const double hi = _RandDouble(0.0, 5.0);
        CHECK(_SameBits(GfClamp(v, lo, hi),
                        rigExec::RrClamp(v, lo, hi)));
        CHECK(_SameBits(GfSqrt(std::fabs(v)),
                        rigExec::RrSqrt(std::fabs(v))));
        const float fv = _RandFloat(-5.0f, 5.0f);
        CHECK(_SameBits(GfClamp(fv, (float)lo, (float)hi),
                        rigExec::RrClamp(fv, (float)lo, (float)hi)));
        CHECK(_SameBits(GfSqrt(std::fabs(fv)),
                        rigExec::RrSqrt(std::fabs(fv))));
        CHECK(rigExec::RrIsClose(v, lo, 0.5) ==
              GfIsClose(v, lo, 0.5));
        const GfVec3d a = _RandGfVec3d();
        const GfVec3d b = _RandGfVec3d();
        CHECK(rigExec::RrIsClose(_ToRr(a), _ToRr(b), 0.5) ==
              GfIsClose(a, b, 0.5));
        CHECK(_SameBits(GfSqr(v), rigExec::RrSqr(v)));
        CHECK(_SameBits(GfRadiansToDegrees(v),
                        rigExec::RrRadiansToDegrees(v)));
        CHECK(_SameBits(GfDegreesToRadians(v),
                        rigExec::RrDegreesToRadians(v)));
        double gs, gc, rs, rc;
        GfSinCos(v, &gs, &gc);
        rigExec::RrSinCos(v, &rs, &rc);
        CHECK(_SameBits(gs, rs));
        CHECK(_SameBits(gc, rc));
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(_SameBits(GfClamp(nan, -1.0, 1.0),
                    rigExec::RrClamp(nan, -1.0, 1.0)));
}

void
_TestMat3d()
{
    _context = "mat3d";
    for (int i = 0; i < 1000; ++i) {
        const GfMatrix3d a = _RandGfMat3d();
        const GfMatrix3d b = _RandGfMat3d();
        const double s = _RandDouble(-3.0, 3.0);
        const rigExec::RrMat3d ra = _ToRr(a);
        const rigExec::RrMat3d rb = _ToRr(b);
        CHECK(_SameMat3d(a * b, ra * rb));
        CHECK(_SameMat3d(a * s, ra * s));
        CHECK(_SameMat3d(s * a, s * ra));
        CHECK(_SameMat3d(a + b, ra + rb));
        CHECK(_SameMat3d(a - b, ra - rb));
        CHECK(_SameMat3d(-a, -ra));
        CHECK(_SameMat3d(a.GetTranspose(), ra.GetTranspose()));
        CHECK(_SameBits(a.GetDeterminant(), ra.GetDeterminant()));
        CHECK(_SameMat3d(a.GetInverse(), ra.GetInverse()));
        CHECK((a == b) == (ra == rb));
        for (int r = 0; r < 3; ++r) {
            CHECK(_SameVec3d(a.GetRow(r), ra.GetRow(r)));
            CHECK(_SameVec3d(a.GetColumn(r), ra.GetColumn(r)));
        }
        {
            GfMatrix3d g = a;
            rigExec::RrMat3d rr = ra;
            const GfVec3d v = _RandGfVec3d();
            g.SetRow(1, v);
            rr.SetRow(1, _ToRr(v));
            g.SetColumn(2, v);
            rr.SetColumn(2, _ToRr(v));
            CHECK(_SameMat3d(g, rr));
        }
    }
    // Singular: all zeros, all ones, duplicated rows.
    const std::vector<GfMatrix3d> singulars = {
        GfMatrix3d(0.0), GfMatrix3d(1, 1, 1, 1, 1, 1, 1, 1, 1),
        GfMatrix3d(1, 2, 3, 1, 2, 3, 4, 5, 6)};
    for (const GfMatrix3d &a : singulars) {
        CHECK(_SameMat3d(a.GetInverse(), _ToRr(a).GetInverse()));
        CHECK(_SameBits(a.GetDeterminant(),
                        _ToRr(a).GetDeterminant()));
    }
}

void
_TestMat4d()
{
    _context = "mat4d";
    for (int i = 0; i < 1000; ++i) {
        const GfMatrix4d a = _RandGfMat4d();
        const GfMatrix4d b = _RandGfMat4d();
        const double s = _RandDouble(-3.0, 3.0);
        const rigExec::RrMat4d ra = _ToRr(a);
        const rigExec::RrMat4d rb = _ToRr(b);
        CHECK(_SameMat4d(a * b, ra * rb));
        CHECK(_SameMat4d(a * s, ra * s));
        CHECK(_SameMat4d(a + b, ra + rb));
        CHECK(_SameMat4d(a - b, ra - rb));
        CHECK(_SameMat4d(-a, -ra));
        CHECK(_SameMat4d(a.GetTranspose(), ra.GetTranspose()));
        CHECK(_SameBits(a.GetDeterminant(), ra.GetDeterminant()));
        CHECK(_SameBits(a.GetDeterminant3(), ra.GetDeterminant3()));
        CHECK(_SameMat4d(a.GetInverse(), ra.GetInverse()));
        CHECK((a == b) == (ra == rb));
        for (int r = 0; r < 4; ++r) {
            CHECK(_SameVec4d(a.GetRow(r), ra.GetRow(r)));
            CHECK(_SameVec4d(a.GetColumn(r), ra.GetColumn(r)));
        }
        {
            GfMatrix4d g = a;
            rigExec::RrMat4d rr = ra;
            const GfVec4d w = _RandGfVec4d();
            g.SetRow(1, w);
            rr.SetRow(1, _ToRr(w));
            g.SetColumn(2, w);
            rr.SetColumn(2, _ToRr(w));
            CHECK(_SameMat4d(g, rr));
        }
        const GfVec3d v = _RandGfVec3d();
        const GfVec4d w = _RandGfVec4d();
        CHECK(_SameVec3d(a.Transform(v), ra.Transform(_ToRr(v))));
        CHECK(_SameVec3d(a.TransformDir(v), ra.TransformDir(_ToRr(v))));
        CHECK(_SameVec3d(a.TransformAffine(v),
                         ra.TransformAffine(_ToRr(v))));
        CHECK(_SameVec4d(a * w, ra * _ToRr(w)));
        CHECK(_SameVec4d(w * a, _ToRr(w) * ra));
        CHECK(_SameVec4d(a * w, ra * _ToRr(w)));
        CHECK(_SameVec3d(a.ExtractTranslation(),
                         ra.ExtractTranslation()));
        CHECK(_SameMat3d(a.ExtractRotationMatrix(),
                         ra.ExtractRotationMatrix()));
        CHECK(_SameQuatd(a.ExtractRotationQuat(),
                         ra.ExtractRotationQuat()));
        CHECK(_SameRotation(a.ExtractRotation(), ra.ExtractRotation()));
        for (int row = 0; row < 4; ++row) {
            CHECK(_SameVec4d(a.GetRow(row), ra.GetRow(row)));
        }
        {
            GfMatrix4d ma = a;
            rigExec::RrMat4d mr = ra;
            ma.SetRow(2, w);
            mr.SetRow(2, _ToRr(w));
            CHECK(_SameMat4d(ma, mr));
            ma.SetTranslateOnly(v);
            mr.SetTranslateOnly(_ToRr(v));
            CHECK(_SameMat4d(ma, mr));
            ma.SetIdentity();
            mr.SetIdentity();
            CHECK(_SameMat4d(ma, mr));
            ma.SetScale(2.5);
            mr.SetScale(2.5);
            CHECK(_SameMat4d(ma, mr));
        }
        {
            const GfQuatd q = _RandGfQuatd().GetNormalized();
            GfMatrix4d ma = a;
            rigExec::RrMat4d mr = ra;
            ma.SetRotate(q);
            mr.SetRotate(_ToRr(q));
            CHECK(_SameMat4d(ma, mr));
            ma.SetRotateOnly(q);
            mr.SetRotateOnly(_ToRr(q));
            CHECK(_SameMat4d(ma, mr));
        }
        {
            const GfMatrix3d m3 = _RandGfMat3d();
            GfMatrix4d ma = a;
            rigExec::RrMat4d mr = ra;
            ma.SetRotate(m3);
            mr.SetRotate(_ToRr(m3));
            CHECK(_SameMat4d(ma, mr));
            ma.SetRotateOnly(m3);
            mr.SetRotateOnly(_ToRr(m3));
            CHECK(_SameMat4d(ma, mr));
        }
        {
            GfMatrix4d ma = a;
            rigExec::RrMat4d mr = ra;
            CHECK(ma.Orthonormalize(false) ==
                  mr.Orthonormalize(false));
            CHECK(_SameMat4d(ma, mr));
        }
    }
    const std::vector<GfMatrix4d> singulars = {
        GfMatrix4d(0.0),
        GfMatrix4d(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1)};
    for (const GfMatrix4d &a : singulars) {
        CHECK(_SameMat4d(a.GetInverse(), _ToRr(a).GetInverse()));
        CHECK(_SameBits(a.GetDeterminant(),
                        _ToRr(a).GetDeterminant()));
        CHECK(_SameQuatd(a.ExtractRotationQuat(),
                         _ToRr(a).ExtractRotationQuat()));
    }
    // Projective Transform (w != 0/1) and the identity fast path.
    for (int i = 0; i < 200; ++i) {
        GfMatrix4d a = _RandGfMat4d();
        a[0][3] = _RandDouble();
        a[3][3] = _RandDouble(-2.0, 2.0);
        const GfVec3d v = _RandGfVec3d();
        CHECK(_SameVec3d(a.Transform(v), _ToRr(a).Transform(_ToRr(v))));
    }
    {
        const GfMatrix4d a(1.0);
        const GfVec3d v = _RandGfVec3d();
        CHECK(_SameVec3d(a.Transform(v), _ToRr(a).Transform(_ToRr(v))));
        CHECK(_SameVec3d(a.TransformDir(v),
                         _ToRr(a).TransformDir(_ToRr(v))));
    }
}

void
_TestQuatd()
{
    _context = "quatd";
    for (int i = 0; i < 2000; ++i) {
        const GfQuatd a = _RandGfQuatd();
        const GfQuatd b = _RandGfQuatd();
        const double s = _RandDouble(-3.0, 3.0);
        const rigExec::RrQuatd ra = _ToRr(a);
        const rigExec::RrQuatd rb = _ToRr(b);
        CHECK(_SameQuatd(a + b, ra + rb));
        CHECK(_SameQuatd(a - b, ra - rb));
        CHECK(_SameQuatd(a * b, ra * rb));
        CHECK(_SameQuatd(a * s, ra * s));
        CHECK(_SameQuatd(s * a, s * ra));
        CHECK(_SameQuatd(-a, -ra));
        CHECK(_SameBits(a.GetLength(), ra.GetLength()));
        CHECK(_SameQuatd(a.GetNormalized(), ra.GetNormalized()));
        CHECK(_SameQuatd(a.GetConjugate(), ra.GetConjugate()));
        CHECK(_SameQuatd(a.GetInverse(), ra.GetInverse()));
        CHECK((a == b) == (ra == rb));
        {
            GfQuatd ma = a;
            rigExec::RrQuatd mr = ra;
            CHECK(_SameBits(ma.Normalize(), mr.Normalize()));
            CHECK(_SameQuatd(ma, mr));
        }
        if (s != 0.0) {
            CHECK(_SameQuatd(a / s, ra / s));
        }
        const GfVec3d v = _RandGfVec3d();
        CHECK(_SameVec3d(a.Transform(v), ra.Transform(_ToRr(v))));
        const double alpha = _RandDouble(0.0, 1.0);
        CHECK(_SameQuatd(GfSlerp(alpha, a.GetNormalized(),
                                 b.GetNormalized()),
                         rigExec::RrSlerp(alpha, ra.GetNormalized(),
                                          rb.GetNormalized())));
    }
    // Short quaternions normalize to identity.
    const std::vector<GfQuatd> shorts = {
        GfQuatd(0.0, GfVec3d(0)), GfQuatd(1e-12, GfVec3d(1e-12))};
    for (const GfQuatd &a : shorts) {
        CHECK(_SameQuatd(a.GetNormalized(), _ToRr(a).GetNormalized()));
        GfQuatd ma = a;
        rigExec::RrQuatd mr = _ToRr(a);
        CHECK(_SameBits(ma.Normalize(), mr.Normalize()));
        CHECK(_SameQuatd(ma, mr));
    }
}

void
_TestRotation()
{
    _context = "rotation";
    for (int i = 0; i < 1000; ++i) {
        const GfVec3d axis = _RandGfVec3d();
        const double angle = _RandDouble(-720.0, 720.0);
        const GfRotation a(axis, angle);
        const rigExec::RrRotation ra(_ToRr(axis), angle);
        CHECK(_SameRotation(a, ra));
        CHECK(_SameQuatd(a.GetQuat(), ra.GetQuat()));
        CHECK(_SameRotation(a.GetInverse(), ra.GetInverse()));
        CHECK(_SameVec3d(a.GetAxis(), ra.GetAxis()));
        CHECK(_SameBits(a.GetAngle(), ra.GetAngle()));
        const GfVec3d v = _RandGfVec3d();
        CHECK(_SameVec3d(a.TransformDir(v), ra.TransformDir(_ToRr(v))));
        const GfQuatd q = _RandGfQuatd();
        const GfRotation b(q);
        const rigExec::RrRotation rb(_ToRr(q));
        CHECK(_SameRotation(b, rb));
        const GfVec3d from = _RandGfVec3d();
        const GfVec3d to = _RandGfVec3d();
        const GfRotation c(from, to);
        const rigExec::RrRotation rc(_ToRr(from), _ToRr(to));
        CHECK(_SameRotation(c, rc));
        const GfVec3d e0 = _RandGfVec3d().GetNormalized();
        GfVec3d e1 = _RandGfVec3d();
        e1 -= e0 * (e0 * e1);
        e1.Normalize();
        const GfVec3d e2 = GfCross(e0, e1);
        CHECK(_SameVec3d(a.Decompose(e0, e1, e2),
                         ra.Decompose(_ToRr(e0), _ToRr(e1), _ToRr(e2))));
    }
    // Parallel/opposite SetRotateInto branches, identity, half turns.
    const GfVec3d x(1, 0, 0);
    const std::vector<std::pair<GfVec3d, GfVec3d>> pairs = {
        {x, x},
        {x, GfVec3d(1.00000001, 0, 0)},
        {x, -x},
        {x, GfVec3d(-1.00000001, 0, 0)},
        {GfVec3d(0), GfVec3d(0, 1, 0)},
    };
    for (const auto &p : pairs) {
        const GfRotation a(p.first, p.second);
        const rigExec::RrRotation ra(_ToRr(p.first), _ToRr(p.second));
        CHECK(_SameRotation(a, ra));
    }
    for (double angle : {0.0, 90.0, 180.0, 270.0, 360.0, -180.0}) {
        const GfRotation a(x, angle);
        const rigExec::RrRotation ra(_ToRr(x), angle);
        CHECK(_SameQuatd(a.GetQuat(), ra.GetQuat()));
        CHECK(_SameVec3d(
            a.Decompose(GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
                        GfVec3d(0, 0, 1)),
            ra.Decompose(rigExec::RrVec3d(1, 0, 0),
                         rigExec::RrVec3d(0, 1, 0),
                         rigExec::RrVec3d(0, 0, 1))));
    }
}

}  // namespace

int
main()
{
    _TestVec3d();
    _TestVec3f();
    _TestVec2();
    _TestVec4d();
    _TestFree();
    _TestMat3d();
    _TestMat4d();
    _TestQuatd();
    _TestRotation();
    if (_failures) {
        std::printf("testRigExecRuntimeMath: %d failures\n", _failures);
        return 1;
    }
    std::printf("testRigExecRuntimeMath: all tests passed\n");
    return 0;
}

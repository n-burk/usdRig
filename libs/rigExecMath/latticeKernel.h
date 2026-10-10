// Method reference: Sederberg and Parry (1986), https://doi.org/10.1145/15886.15903
//
// Tensor-product Bernstein lattice kernel (RigExecLatticeMover, spec §7.5),
// and the regular-grid cubic displacement lattice (rigExec:evaluation
// "regularGrid"; basis and edge references: docs/references.md).
// Header-only and USD-free: the USD evaluators (GfVec3f) and the zero-USD
// runtime (RrVec3f) instantiate one definition.
#ifndef RIGEXEC_MATH_LATTICE_KERNEL_H
#define RIGEXEC_MATH_LATTICE_KERNEL_H

#include "surfaceSnapKernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <string_view>
#include <vector>

namespace rigExec {

/// The Bernstein polynomial of \p index and \p degree at \p t: one axis
/// factor of a lattice point's weight.
inline double
RigExecLatticeBernstein(int degree, int index, double t)
{
    double coefficient = 1.0;
    for (int k = 0; k < index; ++k) {
        coefficient *= double(degree - k) / double(index - k);
    }
    return coefficient * std::pow(t, index) *
           std::pow(1.0 - t, degree - index);
}

/// A lattice bind: every rest point's Bernstein factors along the three cage
/// axes. It depends on the rest points, the rest cage's bound and the
/// divisions only -- never on the posed cage -- so its owner builds it once
/// per bind and a frame only accumulates the cage deltas through it.
///
/// Every factor is +0, positive, +inf or NaN (bind coordinates are clamped
/// into [0, 1]; a degree whose binomial overflows gives +inf, and NaN where
/// that meets a zero power), and only each axis's nonzero span is stored: a
/// point outside the cage's bound has a single nonzero factor on each axis
/// it is clamped on.
struct RigExecLatticeBasis {
    int divisions[3] = {0, 0, 0};
    /// Six per point: [begin, end) of the nonzero factor indices along x,
    /// then y, then z. Every factor outside a span is exactly +0.
    std::vector<int> ranges;
    /// Each point's in-span factors, x then y then z, points in order.
    std::vector<double> factors;
    /// Every factor is at most 1e100 (so neither +inf nor NaN): Bernstein
    /// values are at most 1 unless a huge degree overflows the binomial.
    /// Then no product of three overflows and a product with a +0 factor
    /// is +0.
    bool bounded = true;
};

/// The rest cage's bound, which defines the bind space: \p lo its minimum
/// corner and \p size its extent. False when an extent is not positive (the
/// kernel then passes through). \p count must be nonzero.
template <class Point>
bool
RigExecLatticeBindBox(const Point *restCage, size_t count, Point *lo,
                      Point *size)
{
    *lo = restCage[0];
    Point hi = restCage[0];
    for (size_t i = 0; i < count; ++i) {
        for (int a = 0; a < 3; ++a) {
            (*lo)[a] = std::min((*lo)[a], restCage[i][a]);
            hi[a] = std::max(hi[a], restCage[i][a]);
        }
    }
    *size = hi - *lo;
    return !((*size)[0] <= 0 || (*size)[1] <= 0 || (*size)[2] <= 0);
}

/// One rest point's Bernstein factors along the three cage axes: \p factors
/// receives \p divisions[0] of them, then [1], then [2], and \p range the
/// [begin, end) of each axis's nonzero ones. True when every factor is at
/// most 1e100 (RigExecLatticeBasis::bounded). The retained basis and the
/// streaming kernel both take their factors from here, so they hold the
/// same values: RigExecLatticeBernstein of the same float bind coordinate.
template <class Point>
bool
RigExecLatticePointFactors(const Point &restPoint, const Point &lo,
                           const Point &size, const int divisions[3],
                           double *factors, int *range)
{
    bool bounded = true;
    for (int a = 0; a < 3; ++a) {
        // Bind coordinate from the REST point, clamped into the cage.
        const float t = std::min(
            1.0f, std::max(0.0f, (restPoint[a] - lo[a]) / size[a]));
        const int d = divisions[a];
        int begin = d, end = 0;
        for (int k = 0; k < d; ++k) {
            const double f = RigExecLatticeBernstein(d - 1, k, t);
            factors[k] = f;
            if (f != 0.0) {
                begin = std::min(begin, k);
                end = k + 1;
            }
            if (!(f <= 1e100)) {
                bounded = false;
            }
        }
        if (begin >= end) {
            begin = end = 0;
        }
        range[2 * a] = begin;
        range[2 * a + 1] = end;
        factors += d;
    }
    return bounded;
}

/// Builds the bind of \p count rest points inside the box (\p lo, \p size)
/// at \p dx x \p dy x \p dz divisions: RigExecLatticePointFactors per point,
/// a pure function of the inputs, so a retained basis holds the values a
/// fresh one would.
template <class Point>
void
RigExecBuildLatticeBasis(const Point *restPoints, size_t count,
                         const Point &lo, const Point &size, int dx, int dy,
                         int dz, RigExecLatticeBasis *basis)
{
    const int divisions[3] = {dx, dy, dz};
    for (int a = 0; a < 3; ++a) {
        basis->divisions[a] = divisions[a];
    }
    basis->ranges.assign(count * 6, 0);
    basis->factors.clear();
    basis->bounded = true;
    std::vector<double> scratch(size_t(dx) + size_t(dy) + size_t(dz));
    for (size_t i = 0; i < count; ++i) {
        int *r = &basis->ranges[i * 6];
        if (!RigExecLatticePointFactors(restPoints[i], lo, size, divisions,
                                        scratch.data(), r)) {
            basis->bounded = false;
        }
        const double *axis = scratch.data();
        for (int a = 0; a < 3; ++a) {
            basis->factors.insert(basis->factors.end(), axis + r[2 * a],
                                  axis + r[2 * a + 1]);
            axis += divisions[a];
        }
    }
    basis->factors.shrink_to_fit();
}

/// Whether every cage delta is finite: with bounded factors, the condition
/// for visiting the nonzero spans alone.
template <class Point>
bool
RigExecLatticeDeltasFinite(const Point *cageDeltas, size_t cageCount)
{
    for (size_t k = 0; k < cageCount; ++k) {
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(cageDeltas[k][a])) {
                return false;
            }
        }
    }
    return true;
}

/// One point's displacement: the cage deltas (posed minus rest, x-fastest)
/// weighed by the point's factors. \p r holds its six span bounds and \p fa,
/// \p fb, \p fc its factors from each span's begin. \p skipZeroTerms visits
/// the spans alone; otherwise every term is visited, with the factors
/// outside the spans as the +0 they are.
///
/// The two visits give the same bits when the point's factors are bounded
/// and every delta is finite: a term with a +0 factor then weighs +0 (no
/// factor is +inf, so there is no inf * 0), so it adds delta * +0 = +-0,
/// and adding +-0 leaves the sum as it was -- the sum starts at +0, and
/// round-to-nearest addition never makes it -0. That assumes the default
/// floating-point environment, neither flush-to-zero nor
/// denormals-are-zero: FTZ alone flushes a sum whose value is a negative
/// subnormal to -0, which a skipped +0 term would have made +0.
template <class Point>
Point
RigExecLatticePointDelta(const Point *cageDeltas, int dx, int dy, int dz,
                         const int *r, const double *fa, const double *fb,
                         const double *fc, bool skipZeroTerms)
{
    Point delta(0.0f);
    if (skipZeroTerms) {
        for (int c = r[4]; c < r[5]; ++c) {
            for (int b = r[2]; b < r[3]; ++b) {
                for (int a = r[0]; a < r[1]; ++a) {
                    delta += cageDeltas[(c * dy + b) * dx + a] *
                             float(fa[a - r[0]] * fb[b - r[2]] *
                                   fc[c - r[4]]);
                }
            }
        }
    } else {
        for (int c = 0; c < dz; ++c) {
            const double vc = c >= r[4] && c < r[5] ? fc[c - r[4]] : 0.0;
            for (int b = 0; b < dy; ++b) {
                const double vb = b >= r[2] && b < r[3] ? fb[b - r[2]] : 0.0;
                for (int a = 0; a < dx; ++a) {
                    const double va =
                        a >= r[0] && a < r[1] ? fa[a - r[0]] : 0.0;
                    delta += cageDeltas[(c * dy + b) * dx + a] *
                             float(va * vb * vc);
                }
            }
        }
    }
    return delta;
}

/// RigExecApplyLatticeBasis over points [begin, end) of \p count points, the
/// group's own buffer \p out holding point begin + k at k: the factor cursor
/// is advanced past the points before \p begin, and each point visits the
/// same terms in the same order with the same skipZeroTerms decision, so its
/// bits are the whole call's.
template <class Point>
void
RigExecApplyLatticeBasisGroup(Point *out, size_t count, size_t begin,
                              size_t end, const RigExecLatticeBasis &basis,
                              const Point *cageDeltas)
{
    if (basis.ranges.size() != count * 6) {
        return;  // a basis for other points: pass through
    }
    end = std::min(end, count);
    if (begin >= end) {
        return;
    }
    const int dx = basis.divisions[0], dy = basis.divisions[1],
              dz = basis.divisions[2];
    const size_t cageCount = size_t(dx) * size_t(dy) * size_t(dz);
    // Revision-wide, as the whole call decides it.
    const bool skipZeroTerms =
        basis.bounded && RigExecLatticeDeltasFinite(cageDeltas, cageCount);
    const double *next = basis.factors.data();
    for (size_t i = 0; i < begin; ++i) {
        const int *r = &basis.ranges[i * 6];
        next += (r[1] - r[0]) + (r[3] - r[2]) + (r[5] - r[4]);
    }
    for (size_t i = begin; i < end; ++i) {
        const int *r = &basis.ranges[i * 6];
        const double *fa = next;
        const double *fb = fa + (r[1] - r[0]);
        const double *fc = fb + (r[3] - r[2]);
        next = fc + (r[5] - r[4]);
        out[i - begin] += RigExecLatticePointDelta(cageDeltas, dx, dy, dz, r,
                                                   fa, fb, fc, skipZeroTerms);
    }
}

/// RigExecApplyLatticeBasis over points [begin, end) of the \p count points
/// at \p points, indexed absolutely (RigExecApplyLatticeBasisGroup).
template <class Point>
void
RigExecApplyLatticeBasisRange(Point *points, size_t count, size_t begin,
                              size_t end, const RigExecLatticeBasis &basis,
                              const Point *cageDeltas)
{
    end = std::min(end, count);
    if (begin >= end) {
        return;
    }
    RigExecApplyLatticeBasisGroup(points + begin, count, begin, end, basis,
                                  cageDeltas);
}

/// Adds the posed cage's displacement to \p points through \p basis, which
/// was built for these \p count points. \p cageDeltas are posed minus rest,
/// x-fastest. Bit-identical to visiting every term: the spans alone are
/// visited only when every factor is bounded and every delta finite
/// (RigExecLatticePointDelta).
template <class Point>
void
RigExecApplyLatticeBasis(Point *points, size_t count,
                         const RigExecLatticeBasis &basis,
                         const Point *cageDeltas)
{
    RigExecApplyLatticeBasisRange(points, count, 0, count, basis, cageDeltas);
}

/// RigExecApplyLatticeStreaming over points [begin, end), the group's own
/// buffer \p out holding point begin + k at k and \p restPoints indexed
/// absolutely: every point's factors and decision are its own, so each
/// point's bits are the whole call's.
template <class Point>
void
RigExecApplyLatticeStreamingGroup(Point *out, size_t begin, size_t end,
                                  const Point *restPoints, const Point &lo,
                                  const Point &size, int dx, int dy, int dz,
                                  const Point *cageDeltas)
{
    if (begin >= end) {
        return;
    }
    const int divisions[3] = {dx, dy, dz};
    const bool finite = RigExecLatticeDeltasFinite(
        cageDeltas, size_t(dx) * size_t(dy) * size_t(dz));
    std::vector<double> scratch(size_t(dx) + size_t(dy) + size_t(dz));
    int r[6];
    for (size_t i = begin; i < end; ++i) {
        const bool bounded = RigExecLatticePointFactors(
            restPoints[i], lo, size, divisions, scratch.data(), r);
        const double *fa = scratch.data() + r[0];
        const double *fb = scratch.data() + dx + r[2];
        const double *fc = scratch.data() + dx + dy + r[4];
        out[i - begin] += RigExecLatticePointDelta(cageDeltas, dx, dy, dz, r,
                                                   fa, fb, fc,
                                                   bounded && finite);
    }
}

/// RigExecApplyLatticeStreaming over points [begin, end) of \p points and
/// \p restPoints, indexed absolutely (RigExecApplyLatticeStreamingGroup).
template <class Point>
void
RigExecApplyLatticeStreamingRange(Point *points, size_t begin, size_t end,
                                  const Point *restPoints, const Point &lo,
                                  const Point &size, int dx, int dy, int dz,
                                  const Point *cageDeltas)
{
    if (begin >= end) {
        return;
    }
    RigExecApplyLatticeStreamingGroup(points + begin, begin, end, restPoints,
                                      lo, size, dx, dy, dz, cageDeltas);
}

/// The kernel without a retained basis: each point's factors go through a
/// scratch of (dx + dy + dz) doubles and are used at once, so its memory
/// does not grow with the point count. The same factors, terms and order as
/// RigExecApplyLatticeBasis, hence the same bits. A point decides on its own
/// factors whether to visit its spans alone: where the whole basis would be
/// unbounded but this point is not, both visits give the point the same
/// bits (RigExecLatticePointDelta).
template <class Point>
void
RigExecApplyLatticeStreaming(Point *points, size_t count,
                             const Point *restPoints, const Point &lo,
                             const Point &size, int dx, int dy, int dz,
                             const Point *cageDeltas)
{
    RigExecApplyLatticeStreamingRange(points, 0, count, restPoints, lo, size,
                                      dx, dy, dz, cageDeltas);
}

/// A retained bind: the basis with the raw inputs it was built from.
/// Immutable once built, so revisions and frozen clones hold it through
/// shared_ptr<const>: RigExecSurfaceKernelCache::LatticeBasis builds a
/// revision's own, and RigExecLatticeBindSharing hands revisions whose binds
/// are equal one instance.
template <class Point>
struct RigExecLatticeBind {
    static_assert(sizeof(Point) == 3 * sizeof(float),
                  "three packed floats per point");
    std::vector<Point> rest;
    Point lo, size;
    int divisions[3] = {0, 0, 0};
    RigExecLatticeBasis value;

    /// What a bind of \p count points at these divisions retains at most
    /// (allocation overhead aside): the rest copy, six span bounds and every
    /// factor of every point. In double, so no product wraps.
    static double Bytes(size_t count, int dx, int dy, int dz)
    {
        return double(count) *
               (double(sizeof(Point) + 6 * sizeof(int)) +
                double(sizeof(double)) *
                    (double(dx) + double(dy) + double(dz)));
    }

    /// Whether this bind was built from these inputs, comparing every
    /// scalar's raw bits: a signed zero or a NaN payload is another bind.
    /// \p restKnownEqual (the caller's content version of \p restPoints is
    /// the one the bind was built from) skips only the rest-point compare.
    bool Matches(const Point *restPoints, size_t count, const Point &bindLo,
                 const Point &bindSize, int dx, int dy, int dz,
                 bool restKnownEqual = false) const
    {
        if (divisions[0] != dx || divisions[1] != dy || divisions[2] != dz ||
            rest.size() != count) {
            return false;
        }
        for (int axis = 0; axis < 3; ++axis) {
            const auto a = lo[axis], b = bindLo[axis];
            const auto c = size[axis], d = bindSize[axis];
            if (std::memcmp(&a, &b, sizeof(a)) ||
                std::memcmp(&c, &d, sizeof(c))) {
                return false;
            }
        }
        return restKnownEqual || !count ||
               !std::memcmp(rest.data(), restPoints, count * sizeof(Point));
    }

    bool Matches(const RigExecLatticeBind &other) const
    {
        return Matches(other.rest.data(), other.rest.size(), other.lo,
                       other.size, other.divisions[0], other.divisions[1],
                       other.divisions[2]);
    }

    void Build(const Point *restPoints, size_t count, const Point &bindLo,
               const Point &bindSize, int dx, int dy, int dz)
    {
        if (count) {
            rest.assign(restPoints, restPoints + count);
        }
        lo = bindLo;
        size = bindSize;
        divisions[0] = dx;
        divisions[1] = dy;
        divisions[2] = dz;
        RigExecBuildLatticeBasis(restPoints, count, bindLo, bindSize, dx, dy,
                                 dz, &value);
    }
};

/// Hands caches whose retained lattice binds are equal one instance, so
/// revisions with one bind hold one basis. The thread that runs the
/// caches' program offers each cache once, in a fixed order, while none of
/// its steps runs: it replaces the caches' own slots, which otherwise only
/// their revision's step writes. The binds are immutable and the answer is
/// the bind's either way. A bind already shared compares by pointer, so a
/// run that built none compares no points.
template <class Point>
class RigExecLatticeBindSharing {
public:
    template <class Cache>
    void Offer(Cache *cache)
    {
        const std::shared_ptr<const RigExecLatticeBind<Point>> &bind =
            cache->RetainedLatticeBind();
        if (!bind) {
            return;
        }
        for (const auto &held : _distinct) {
            if (held == bind) {
                return;
            }
        }
        for (const auto &held : _distinct) {
            if (held->Matches(*bind)) {
                cache->ShareLatticeBind(held);
                return;
            }
        }
        _distinct.push_back(bind);
    }

private:
    std::vector<std::shared_ptr<const RigExecLatticeBind<Point>>> _distinct;
};

/// RigExecApplyLatticeKernel's decisions over \p count points before it
/// reads a point: the cage description, the bind box (\p lo, \p size) and,
/// when \p cageDeltas is given, the posed-minus-rest cage deltas. False when
/// the kernel passes every point through.
template <class Point>
bool
RigExecLatticeKernelSetup(size_t count, const Point *restPoints,
                          size_t restPointsSize, const Point *restCage,
                          size_t restCageSize, const Point *posedCage,
                          size_t posedCageSize, int dx, int dy, int dz,
                          Point *lo, Point *size,
                          std::vector<Point> *cageDeltas)
{
    const size_t cageCount = size_t(dx) * size_t(dy) * size_t(dz);
    if (count == 0 || restPointsSize != count || restCageSize != cageCount ||
        posedCageSize != cageCount || dx < 2 || dy < 2 || dz < 2) {
        return false;  // invalid cage description: pass through
    }
    if ((restPointsSize > 0 && !restPoints) ||
        (restCageSize > 0 && !restCage) ||
        (posedCageSize > 0 && !posedCage)) {
        return false;  // a null range with a nonzero size: pass through
    }
    if (!RigExecLatticeBindBox(restCage, restCageSize, lo, size)) {
        return false;
    }
    if (cageDeltas) {
        // Cage deltas preserve identity when the cage is at rest.
        cageDeltas->resize(cageCount);
        for (size_t i = 0; i < cageCount; ++i) {
            (*cageDeltas)[i] = posedCage[i] - restCage[i];
        }
    }
    return true;
}

/// RigExecApplyLatticeKernel over points [begin, end) of \p points (the whole
/// array, holding the entering points there): the same pass-through checks
/// over the whole size, the same bind box and cage deltas, and \p basis (or,
/// when null, the per-point evaluation the null-cache path performs), so each
/// point's bits are the whole kernel's. Points outside the range are not
/// touched.
template <class Point>
void
RigExecApplyLatticeKernelRange(
    std::vector<Point> *points, size_t begin, size_t end,
    const Point *restPoints, size_t restPointsSize, const Point *restCage,
    size_t restCageSize, const Point *posedCage, size_t posedCageSize, int dx,
    int dy, int dz, const RigExecLatticeBasis *basis)
{
    Point lo, size;
    std::vector<Point> cageDeltas;
    if (!RigExecLatticeKernelSetup(points->size(), restPoints, restPointsSize,
                                   restCage, restCageSize, posedCage,
                                   posedCageSize, dx, dy, dz, &lo, &size,
                                   &cageDeltas)) {
        return;
    }
    end = std::min(end, points->size());
    if (basis) {
        RigExecApplyLatticeBasisRange(points->data(), points->size(), begin,
                                      end, *basis, cageDeltas.data());
    } else {
        RigExecApplyLatticeStreamingRange(points->data(), begin, end,
                                          restPoints, lo, size, dx, dy, dz,
                                          cageDeltas.data());
    }
}

/// RigExecApplyLatticeKernelRange with the group's points in their own
/// buffers: \p in[k] / \p out[k] are point begin + k of \p count; every
/// whole-array check and the factor cursor are the whole call's. When the
/// setup refuses (an invalid cage), copies \p in to \p out and returns.
template <class Point>
void
RigExecApplyLatticeKernelGroup(
    const Point *in, Point *out, size_t count, size_t begin, size_t end,
    const Point *restPoints, size_t restPointsSize, const Point *restCage,
    size_t restCageSize, const Point *posedCage, size_t posedCageSize, int dx,
    int dy, int dz, const RigExecLatticeBasis *basis)
{
    end = std::min(end, count);
    if (begin >= end) {
        return;
    }
    // The entering points first, as the whole kernel runs on a copy of
    // them: every pass-through below leaves them, and the deformation adds
    // each point's displacement to its own.
    std::copy(in, in + (end - begin), out);
    Point lo, size;
    std::vector<Point> cageDeltas;
    if (!RigExecLatticeKernelSetup(count, restPoints, restPointsSize,
                                   restCage, restCageSize, posedCage,
                                   posedCageSize, dx, dy, dz, &lo, &size,
                                   &cageDeltas)) {
        return;
    }
    if (basis) {
        RigExecApplyLatticeBasisGroup(out, count, begin, end, *basis,
                                      cageDeltas.data());
    } else {
        RigExecApplyLatticeStreamingGroup(out, begin, end, restPoints, lo,
                                          size, dx, dy, dz,
                                          cageDeltas.data());
    }
}

/// p'(u,v,w) = p + sum_abc B_a(u) B_b(v) B_c(w) (posed - rest)_abc over a
/// dx x dy x dz cage in x-fastest order. Bind coordinates derive from each
/// rest point normalized into the rest cage's bound and clamped to it.
/// Identity when the cage is at rest; an invalid cage passes through.
///
/// \p cache, when set, is the caller's own per-revision kernel cache
/// (RigExecSurfaceKernelCache): it retains the basis while the rest points,
/// the bound and the divisions hold their bits, within its budget. Null,
/// or a bind over the budget, streams each point's factors instead. Both
/// answer the same bits.
template <class Point, class Cache>
void
RigExecApplyLatticeKernel(std::vector<Point> *points, const Point *restPoints,
                          size_t restPointsSize, const Point *restCage,
                          size_t restCageSize, const Point *posedCage,
                          size_t posedCageSize, int dx, int dy, int dz,
                          Cache *cache)
{
    Point lo, size;
    std::vector<Point> cageDeltas;
    if (!RigExecLatticeKernelSetup(points->size(), restPoints, restPointsSize,
                                   restCage, restCageSize, posedCage,
                                   posedCageSize, dx, dy, dz, &lo, &size,
                                   &cageDeltas)) {
        return;
    }
    const RigExecLatticeBasis *basis =
        cache ? cache->LatticeBasis(restPoints, restPointsSize, lo, size, dx,
                                    dy, dz)
              : nullptr;
    if (basis) {
        RigExecApplyLatticeBasis(points->data(), points->size(), *basis,
                                 cageDeltas.data());
    } else {
        RigExecApplyLatticeStreaming(points->data(), points->size(),
                                     restPoints, lo, size, dx, dy, dz,
                                     cageDeltas.data());
    }
}

// The regular-grid evaluation. Settings parse from a view, so token text
// parses without a temporary or a registry lookup.
enum class RigExecLatticeInterpolation { Linear, Cardinal, BSpline, CatmullRom };
inline bool RigExecLatticeInterpolationFromString(std::string_view name,RigExecLatticeInterpolation *out) {
    if(name=="linear")*out=RigExecLatticeInterpolation::Linear;
    else if(name=="cardinal")*out=RigExecLatticeInterpolation::Cardinal;
    else if(name=="bspline")*out=RigExecLatticeInterpolation::BSpline;
    else if(name=="catmullRom")*out=RigExecLatticeInterpolation::CatmullRom;
    else return false;
    return true;
}
/// The row-vector maps between the target's point space and the lattice's,
/// from the cage and target frames: "local" relates the two frames, "common"
/// reads both point sets in one shared space.
template<class M> bool RigExecLatticeCoordinateMaps(std::string_view space,const M &cage,const M &target,
    M *targetToLattice,M *latticeToTarget,M *cageToLattice) {
    if(!RigExecSurfaceSnapValidMatrix(cage) || !RigExecSurfaceSnapValidMatrix(target))return false;
    *cageToLattice=M(1.0);
    if(space=="local") {*targetToLattice=target*RigExecSurfaceSnapAffineInverse(cage);*latticeToTarget=cage*RigExecSurfaceSnapAffineInverse(target);}
    else if(space=="common") {*targetToLattice=RigExecSurfaceSnapAffineInverse(cage);*cageToLattice=RigExecSurfaceSnapAffineInverse(cage);*latticeToTarget=cage;}
    else return false;
    return true;
}
/// The regular-grid settings. regularGrid false is the legacy Bernstein
/// evaluation, whatever the other fields hold.
struct RigExecLatticeSettings {
    bool regularGrid=false;
    std::array<RigExecLatticeInterpolation,3> interpolation={RigExecLatticeInterpolation::BSpline,RigExecLatticeInterpolation::BSpline,RigExecLatticeInterpolation::BSpline};
    std::array<float,3> origin={-.5f,-.5f,-.5f},spacing={1,1,1};
    float strength=1;
    std::vector<float> mask;
    bool operator==(const RigExecLatticeSettings &o) const {
        return regularGrid==o.regularGrid && interpolation==o.interpolation && origin==o.origin &&
            spacing==o.spacing && strength==o.strength && mask==o.mask;
    }
    bool operator!=(const RigExecLatticeSettings &o) const { return !(*this==o); }
};
namespace latticeDetail {
inline std::array<float,4> Basis(float t,RigExecLatticeInterpolation mode) {
    const float t2=t*t,t3=t2*t;
    if(mode==RigExecLatticeInterpolation::Linear)return {0,1-t,t,0};
    if(mode==RigExecLatticeInterpolation::BSpline)
        return {(1-3*t+3*t2-t3)/6,(4-6*t2+3*t3)/6,(1+3*t+3*t2-3*t3)/6,t3/6};
    const float c=mode==RigExecLatticeInterpolation::Cardinal?.71f:.5f;
    return {-c*t3+2*c*t2-c*t,(2-c)*t3+(c-3)*t2+1,(c-2)*t3+(3-2*c)*t2+c*t,c*t3-c*t2};
}
}
/// Displaces \p points by the regular grid's posed node offsets: a node's
/// offset is its posed position (in lattice space through \p cageToLattice)
/// less its grid position, mapped back by \p latticeToTarget, and a point
/// reads the cubic blend of the 4x4x4 nodes around its lattice coordinate.
/// Fails atomically, leaving \p points unchanged.
template<class V,class D,class M,class Div>
bool RigExecApplyLatticeGridKernel(std::vector<V> *points,const std::vector<V> &posed,
    const Div &divisions,const RigExecLatticeSettings &s,const M &targetToLattice,
    const M &latticeToTarget,const M &cageToLattice) {
    if(!points || !s.regularGrid || !std::isfinite(s.strength) ||
       (!s.mask.empty() && s.mask.size()!=points->size()) ||
       !RigExecSurfaceSnapValidMatrix(targetToLattice) || !RigExecSurfaceSnapValidMatrix(latticeToTarget) ||
       !RigExecSurfaceSnapValidMatrix(cageToLattice))return false;
    for(float w:s.mask)if(!std::isfinite(w) || w<0 || w>1)return false;
    size_t count=1;
    for(int axis=0;axis<3;++axis) {
        if(divisions[axis]<1 || size_t(divisions[axis])>std::numeric_limits<size_t>::max()/count ||
            !std::isfinite(s.origin[axis]) || !std::isfinite(s.spacing[axis]) ||
            (divisions[axis]>1 && s.spacing[axis]==0))return false;
        count*=size_t(divisions[axis]);
    }
    if(posed.size()!=count)return false;
    std::vector<V> deltas;deltas.reserve(count);
    for(int k=0;k<divisions[2];++k)for(int j=0;j<divisions[1];++j)for(int i=0;i<divisions[0];++i) {
        const auto &p=posed[(size_t(k)*divisions[1]+j)*divisions[0]+i];
        if(!surfaceSnapDetail::Finite(p))return false;
        const D local=cageToLattice.TransformAffine(D(p[0],p[1],p[2]));
        const D delta=local-D(s.origin[0]+i*s.spacing[0],s.origin[1]+j*s.spacing[1],s.origin[2]+k*s.spacing[2]);
        const D mapped=latticeToTarget.TransformDir(delta);
        deltas.emplace_back(float(mapped[0]),float(mapped[1]),float(mapped[2]));
        if(!surfaceSnapDetail::Finite(deltas.back()))return false;
    }
    auto candidate=*points;
    for(size_t i=0;i<candidate.size();++i) {
        const float weight=s.strength*(s.mask.empty()?1:s.mask[i]);if(weight==0)continue;
        const auto &p=candidate[i];if(!surfaceSnapDetail::Finite(p))return false;
        const D local=targetToLattice.TransformAffine(D(p[0],p[1],p[2]));
        std::array<std::array<float,4>,3> weights;std::array<int,3> lower;
        for(int axis=0;axis<3;++axis) {
            if(divisions[axis]==1) {weights[axis]={0,1,0,0};lower[axis]=0;continue;}
            const float coordinate=(float(local[axis])-s.origin[axis])/s.spacing[axis];
            if(!std::isfinite(coordinate))return false;
            // Beyond the supported range all four clamped indices coincide;
            // reducing the integer prevents overflow without clamping the query.
            if(coordinate<-2) {lower[axis]=-2;weights[axis]={0,1,0,0};}
            else if(coordinate>float(divisions[axis]+1)) {lower[axis]=divisions[axis];weights[axis]={0,1,0,0};}
            else {lower[axis]=int(std::floor(coordinate));weights[axis]=latticeDetail::Basis(coordinate-lower[axis],s.interpolation[axis]);}
        }
        V delta(0);
        for(int k=0;k<4;++k)for(int j=0;j<4;++j)for(int a=0;a<4;++a) {
            const int x=std::clamp(lower[0]+a-1,0,divisions[0]-1),y=std::clamp(lower[1]+j-1,0,divisions[1]-1),z=std::clamp(lower[2]+k-1,0,divisions[2]-1);
            const float w=weight*weights[2][k]*weights[1][j]*weights[0][a];
            delta+=deltas[(size_t(z)*divisions[1]+y)*divisions[0]+x]*w;
        }
        candidate[i]+=delta;if(!surfaceSnapDetail::Finite(candidate[i]))return false;
    }
    *points=std::move(candidate);return true;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_LATTICE_KERNEL_H

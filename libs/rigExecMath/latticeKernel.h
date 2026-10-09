// Method reference: Sederberg and Parry (1986), https://doi.org/10.1145/15886.15903
//
// Tensor-product Bernstein lattice kernel (RigExecLatticeMover, spec §7.5).
// Header-only and USD-free: the USD evaluators (GfVec3f) and the zero-USD
// runtime (RrVec3f) instantiate one definition.
#ifndef RIGEXEC_MATH_LATTICE_KERNEL_H
#define RIGEXEC_MATH_LATTICE_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
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
/// Every factor is +0, positive or NaN (bind coordinates are clamped into
/// [0, 1]), and only each axis's nonzero span is stored: a point outside the
/// cage's bound has a single nonzero factor on each axis it is clamped on.
struct RigExecLatticeBasis {
    int divisions[3] = {0, 0, 0};
    /// Six per point: [begin, end) of the nonzero factor indices along x,
    /// then y, then z. Every factor outside a span is exactly +0.
    std::vector<int> ranges;
    /// Each point's in-span factors, x then y then z, points in order.
    std::vector<double> factors;
    /// Every factor is at most 1e100 (Bernstein values are at most 1 unless
    /// a huge degree overflows the binomial), so no product of three
    /// overflows and a product with a +0 factor is +0.
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

/// Builds the bind of \p count rest points inside the box (\p lo, \p size)
/// at \p dx x \p dy x \p dz divisions. Each factor is RigExecLatticeBernstein
/// of the point's float bind coordinate: a pure function of the inputs, so a
/// retained basis holds the values a fresh one would.
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
    std::vector<double> axis(size_t(std::max(dx, std::max(dy, dz))));
    for (size_t i = 0; i < count; ++i) {
        for (int a = 0; a < 3; ++a) {
            // Bind coordinate from the REST point, clamped into the cage.
            const float t = std::min(
                1.0f, std::max(0.0f, (restPoints[i][a] - lo[a]) / size[a]));
            const int d = divisions[a];
            int begin = d, end = 0;
            for (int k = 0; k < d; ++k) {
                const double f = RigExecLatticeBernstein(d - 1, k, t);
                axis[size_t(k)] = f;
                if (f != 0.0) {
                    begin = std::min(begin, k);
                    end = k + 1;
                }
                if (!(f <= 1e100)) {
                    basis->bounded = false;
                }
            }
            if (begin >= end) {
                begin = end = 0;
            }
            basis->ranges[i * 6 + size_t(2 * a)] = begin;
            basis->ranges[i * 6 + size_t(2 * a + 1)] = end;
            basis->factors.insert(basis->factors.end(), axis.begin() + begin,
                                  axis.begin() + end);
        }
    }
    basis->factors.shrink_to_fit();
}

/// Adds the posed cage's displacement to \p points through \p basis, which
/// was built for these \p count points. \p cageDeltas are posed minus rest,
/// x-fastest.
///
/// Bit-identical to visiting every term: with bounded factors a term with a
/// +0 factor weighs +0, so it adds delta * +0 = +-0 when the delta is
/// finite, and adding +-0 leaves the sum as it was -- the sum starts at +0,
/// and round-to-nearest addition never makes it -0. Otherwise every term is
/// visited, with the unstored factors as the +0 they are.
template <class Point>
void
RigExecApplyLatticeBasis(Point *points, size_t count,
                         const RigExecLatticeBasis &basis,
                         const Point *cageDeltas)
{
    if (basis.ranges.size() != count * 6) {
        return;  // a basis for other points: pass through
    }
    const int dx = basis.divisions[0], dy = basis.divisions[1],
              dz = basis.divisions[2];
    const size_t cageCount = size_t(dx) * size_t(dy) * size_t(dz);
    bool skipZeroTerms = basis.bounded;
    for (size_t k = 0; k < cageCount && skipZeroTerms; ++k) {
        for (int a = 0; a < 3; ++a) {
            skipZeroTerms = skipZeroTerms && std::isfinite(cageDeltas[k][a]);
        }
    }
    const double *next = basis.factors.data();
    for (size_t i = 0; i < count; ++i) {
        const int *r = &basis.ranges[i * 6];
        const double *fa = next;
        const double *fb = fa + (r[1] - r[0]);
        const double *fc = fb + (r[3] - r[2]);
        next = fc + (r[5] - r[4]);
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
                    const double vb =
                        b >= r[2] && b < r[3] ? fb[b - r[2]] : 0.0;
                    for (int a = 0; a < dx; ++a) {
                        const double va =
                            a >= r[0] && a < r[1] ? fa[a - r[0]] : 0.0;
                        delta += cageDeltas[(c * dy + b) * dx + a] *
                                 float(va * vb * vc);
                    }
                }
            }
        }
        points[i] += delta;
    }
}

/// p'(u,v,w) = p + sum_abc B_a(u) B_b(v) B_c(w) (posed - rest)_abc over a
/// dx x dy x dz cage in x-fastest order. Bind coordinates derive from each
/// rest point normalized into the rest cage's bound and clamped to it.
/// Identity when the cage is at rest; an invalid cage passes through.
///
/// \p cache, when set, is the caller's own per-revision kernel cache
/// (RigExecSurfaceKernelCache): it retains the basis while the rest points,
/// the bound and the divisions hold their bits. Null builds the basis for
/// this call only. Both answer the same bits.
template <class Point, class Cache>
void
RigExecApplyLatticeKernel(std::vector<Point> *points, const Point *restPoints,
                          size_t restPointsSize, const Point *restCage,
                          size_t restCageSize, const Point *posedCage,
                          size_t posedCageSize, int dx, int dy, int dz,
                          Cache *cache)
{
    const size_t cageCount = size_t(dx) * size_t(dy) * size_t(dz);
    if (points->empty() || restPointsSize != points->size() ||
        restCageSize != cageCount || posedCageSize != cageCount || dx < 2 ||
        dy < 2 || dz < 2) {
        return;  // invalid cage description: pass through
    }
    if ((restPointsSize > 0 && !restPoints) ||
        (restCageSize > 0 && !restCage) ||
        (posedCageSize > 0 && !posedCage)) {
        return;  // a null range with a nonzero size: pass through
    }
    Point lo, size;
    if (!RigExecLatticeBindBox(restCage, restCageSize, &lo, &size)) {
        return;
    }
    // Cage deltas preserve identity when the cage is at rest.
    std::vector<Point> cageDeltas(cageCount);
    for (size_t i = 0; i < cageCount; ++i) {
        cageDeltas[i] = posedCage[i] - restCage[i];
    }
    RigExecLatticeBasis built;
    const RigExecLatticeBasis *basis = &built;
    if (cache) {
        basis = cache->LatticeBasis(restPoints, restPointsSize, lo, size, dx,
                                    dy, dz);
    } else {
        RigExecBuildLatticeBasis(restPoints, restPointsSize, lo, size, dx, dy,
                                 dz, &built);
    }
    if (basis) {
        RigExecApplyLatticeBasis(points->data(), points->size(), *basis,
                                 cageDeltas.data());
    }
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_LATTICE_KERNEL_H

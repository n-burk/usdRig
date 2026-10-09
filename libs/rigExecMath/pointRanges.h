// Point ranges of a range-pipelined point chain: how Build cuts a chain into
// ranges and how a run clips and publishes one. Header-only and USD-free:
// the native program and the zero-USD runtime include one definition.
#ifndef RIGEXEC_MATH_POINT_RANGES_H
#define RIGEXEC_MATH_POINT_RANGES_H

#include <algorithm>
#include <cstddef>
#include <cstring>

namespace rigExec {

/// How many ranges a point chain of \p points points is cut into at Build:
/// about \p target points each, at most \p cap. 1 means the chain is not
/// range-pipelined.
inline size_t
RigExecPointRangeCount(size_t points, size_t target, size_t cap)
{
    target = std::max<size_t>(target, 1);
    cap = std::max<size_t>(cap, 1);
    return std::max<size_t>(1, std::min((points + target - 1) / target, cap));
}

/// Bound \p k of \p points points cut into \p ranges equal ranges, the last
/// one shorter: min(points, k * ceil(points / ranges)).
inline size_t
RigExecPointRangeBound(size_t points, size_t ranges, size_t k)
{
    const size_t size = ranges ? (points + ranges - 1) / ranges : points;
    return std::min(points, k * size);
}

/// A Build range [buildBegin, buildEnd) over the \p count points a run
/// applies to: clipped to \p count, the last range running to \p count, so a
/// partition covers [0, count) exactly once whatever the count. A per-point
/// kernel gives the same bits over any partition.
inline void
RigExecPointRangeAt(int buildBegin, int buildEnd, bool last, size_t count,
                    size_t *begin, size_t *end)
{
    *begin = std::min(size_t(std::max(buildBegin, 0)), count);
    *end = last ? count : std::min(size_t(std::max(buildEnd, 0)), count);
    if (*end < *begin) *end = *begin;
}

/// Makes out[0, n) hold in[0, n), copying only the 1024-point blocks whose
/// bytes differ (memcmp: NaN payloads and signed zeros count), and returns
/// whether any did. \p in and \p out do not overlap; Point is trivially
/// copyable and padding-free.
template <class Point>
bool
RigExecCopyMovedRange(const Point *in, Point *out, size_t n)
{
    constexpr size_t kBlock = 1024;
    bool moved = false;
    for (size_t at = 0; at < n; at += kBlock) {
        const size_t m = std::min(kBlock, n - at);
        if (std::memcmp(in + at, out + at, m * sizeof(Point)) != 0) {
            std::memcpy(out + at, in + at, m * sizeof(Point));
            moved = true;
        }
    }
    return moved;
}

}  // namespace rigExec

#endif

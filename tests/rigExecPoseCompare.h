//
// One definition of "the same published generation", for the tests.
//
// The baked-mode suite grew three comparisons with three different
// coverages -- one that omitted the provider transforms, one that omitted
// every scalar, one that omitted the diagnostics -- and none of them matched
// RigExecComparePoses, which is the comparison the parity mode actually
// makes. Three coverages means three different answers to "did the two paths
// publish the same generation", and the one a failing test happened to call
// decided whether a defect was seen at all.
//
// So: one set of functions, mirroring RigExecComparePoses domain for domain
// -- every published map including the provider transforms and the two
// weight domains, the compared scalars, and the diagnostics in order. A
// domain added to the comparator is added here, and a test that wants less
// than the whole generation says so by calling the narrower function rather
// than by having the broader one quietly cover less.
//
// Reports through a caller-owned failure counter rather than a global, so a
// suite keeps its own `failures` and its own CHECK macro.
//
#ifndef RIGEXEC_TESTS_POSE_COMPARE_H
#define RIGEXEC_TESTS_POSE_COMPARE_H

#include "rigExec/rigEvaluator.h"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExecTest {

/// Exact equality on one published frame: the flags and every point.
///
/// Exact, not a tolerance, for the reason the whole baked mode is: "close"
/// between two implementations of one rig is a second rig.
inline bool
SameFrame(const rigExec::RigExecPointFrame &a,
          const rigExec::RigExecPointFrame &b)
{
    return a.flags == b.flags && a.points == b.points;
}

/// One published map domain, key by key in BOTH directions.
///
/// A key only \p reference has and a key only \p baked has are each a
/// failure: a program that publishes half the joints agrees perfectly about
/// the half it published.
template <class Map, class Equal>
void
CompareMaps(int *failures, const char *what, const std::string &where,
            const Map &reference, const Map &baked, Equal equal)
{
    for (const auto &[path, value] : reference) {
        const auto found = baked.find(path);
        if (found == baked.end()) {
            ++*failures;
            std::printf("FAIL %s: baked mode published no %s for %s\n",
                        where.c_str(), what, path.GetText());
        } else if (!equal(value, found->second)) {
            ++*failures;
            std::printf("FAIL %s: %s differs at %s\n", where.c_str(), what,
                        path.GetText());
        }
    }
    for (const auto &[path, value] : baked) {
        if (!reference.count(path)) {
            ++*failures;
            std::printf("FAIL %s: baked mode published an extra %s at %s\n",
                        where.c_str(), what, path.GetText());
        }
    }
}

/// Every published MAP domain RigExecComparePoses compares.
///
/// The weight domains are empty on both paths while the features that fill
/// them refuse the bake; they are compared anyway, for the same reason the
/// comparator compares them -- so the first generation a weight object bakes
/// is measured instead of waved through.
inline void
CompareEveryMap(int *failures, const std::string &where,
                const rigExec::RigExecRigPose &reference,
                const rigExec::RigExecRigPose &baked)
{
    const auto sameMatrix = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        return a == b;
    };
    CompareMaps(failures, "base joint frame", where, reference.jointFramesBase,
                baked.jointFramesBase, SameFrame);
    CompareMaps(failures, "final joint frame", where,
                reference.jointFramesFinal, baked.jointFramesFinal, SameFrame);
    CompareMaps(failures, "joint matrix", where, reference.jointMatricesFinal,
                baked.jointMatricesFinal, sameMatrix);
    CompareMaps(failures, "control frame", where, reference.controlFrames,
                baked.controlFrames, SameFrame);
    CompareMaps(failures, "provider transform", where, reference.providerXforms,
                baked.providerXforms, sameMatrix);
    CompareMaps(failures, "provider base transform", where,
                reference.providerBaseXforms, baked.providerBaseXforms,
                sameMatrix);
    CompareMaps(failures, "moved property", where, reference.movedProperties,
                baked.movedProperties,
                [](const VtValue &a, const VtValue &b) { return a == b; });
    CompareMaps(failures, "solver frames", where, reference.solverFrames,
                baked.solverFrames,
                [](const std::vector<rigExec::RigExecPointFrame> &a,
                   const std::vector<rigExec::RigExecPointFrame> &b) {
                    return a.size() == b.size() &&
                           std::equal(a.begin(), a.end(), b.begin(),
                                      SameFrame);
                });
    CompareMaps(failures, "weight field", where, reference.weightFields,
                baked.weightFields,
                [](const rigExec::RigExecResolvedWeightField &a,
                   const rigExec::RigExecResolvedWeightField &b) {
                    return a.target == b.target && a.weights == b.weights;
                });
    CompareMaps(failures, "weight frame", where, reference.weightFrames,
                baked.weightFrames, sameMatrix);
}

/// The published SCALARS of a generation, the diagnostics included.
///
/// A path that lands on the right points while reporting different work is
/// still a second rig, and the difference is exactly the shape a map
/// comparison cannot see. The diagnostics are compared in ORDER, because the
/// order is the walk order and a consumer reading "MoverFailed X" after
/// "constraint Y passed through" is being told a sequence.
///
/// The mover-graph counters are a property of how WARM the evaluator is, not
/// of the frame -- a graph built this generation reports the revisions it
/// created -- so this belongs beside a comparison of two evaluators that
/// have answered the same generations, and not beside one that compares a
/// fresh evaluator with a running one.
inline void
CompareGenerationScalars(int *failures, const std::string &where,
                         const rigExec::RigExecRigPose &reference,
                         const rigExec::RigExecRigPose &baked)
{
    const auto count = [failures, &where](const char *what, size_t a,
                                          size_t b) {
        if (a != b) {
            ++*failures;
            std::printf("FAIL %s: %s is %zu dynamically and %zu baked\n",
                        where.c_str(), what, a, b);
        }
    };
    count("mover graph revisions created", reference.moverGraphRevisionsCreated,
          baked.moverGraphRevisionsCreated);
    count("mover graph revisions executed",
          reference.moverGraphRevisionsExecuted,
          baked.moverGraphRevisionsExecuted);
    count("mover graph schedules built", reference.moverGraphSchedulesBuilt,
          baked.moverGraphSchedulesBuilt);
    count("solver override rounds", reference.solverOverrideRounds,
          baked.solverOverrideRounds);
    if (reference.solverOverridesConverged != baked.solverOverridesConverged) {
        ++*failures;
        std::printf("FAIL %s: solver overrides converged is %s dynamically "
                    "and %s baked\n", where.c_str(),
                    reference.solverOverridesConverged ? "true" : "false",
                    baked.solverOverridesConverged ? "true" : "false");
    }
    if (reference.diagnostics != baked.diagnostics) {
        ++*failures;
        std::printf("FAIL %s: diagnostics differ (%zu vs %zu)\n",
                    where.c_str(), reference.diagnostics.size(),
                    baked.diagnostics.size());
        for (size_t i = 0;
             i < std::max(reference.diagnostics.size(),
                          baked.diagnostics.size()); ++i) {
            const char *a = i < reference.diagnostics.size()
                                ? reference.diagnostics[i].c_str() : "<none>";
            const char *b = i < baked.diagnostics.size()
                                ? baked.diagnostics[i].c_str() : "<none>";
            if (std::string(a) != b) std::printf("    %s\n  vs%s\n", a, b);
        }
    }
}

/// The whole published generation: every map, then every compared scalar.
///
/// The full surface of RigExecComparePoses, for a caller whose two
/// evaluators are at the same point in their lives.
inline void
ComparePose(int *failures, const std::string &where,
            const rigExec::RigExecRigPose &reference,
            const rigExec::RigExecRigPose &baked)
{
    CompareEveryMap(failures, where, reference, baked);
    CompareGenerationScalars(failures, where, reference, baked);
}

}  // namespace rigExecTest

#endif  // RIGEXEC_TESTS_POSE_COMPARE_H

// Shared exact published-value comparisons mirror RigExecComparePoses.
// History tests compare maps and ordered diagnostics; executor work traces
// are checked separately because cached and fresh runs can execute differently.
// Helpers report through the caller-owned failure counter.
#ifndef RIGEXEC_TESTS_POSE_COMPARE_H
#define RIGEXEC_TESTS_POSE_COMPARE_H

#include "rigExec/rigEvaluator.h"

#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cstdio>
#include <functional>
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
            std::printf("FAIL %s: actual pose published no %s for %s\n",
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
            std::printf("FAIL %s: actual pose published an extra %s at %s\n",
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
/// Compare ordered published diagnostics. Work counts are inspected through
/// the actual executor trace by scheduling tests and can differ with cache history.
inline void
CompareGenerationScalars(int *failures, const std::string &where,
                         const rigExec::RigExecRigPose &reference,
                         const rigExec::RigExecRigPose &baked)
{
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

/// One evaluator input: a time and the interactive overrides standing on it.
struct EvaluationState {
    UsdTimeCode time = UsdTimeCode::Default();
    std::vector<rigExec::RigExecValueOverride> overrides;
};

/// A generation depends only on its own inputs: \p after evaluated right
/// after \p before on one evaluator publishes exactly what a fresh evaluator
/// publishes for \p after alone. A step that read a value the previous
/// generation left -- a blend reading last run's input aggregate -- differs
/// here. Only the maps are compared, because the counters depend on how warm
/// the evaluator is. \p make builds one stage per evaluator; \p before must
/// publish different solver frames from \p after, or the check is vacuous.
inline void
CheckHistoryIndependent(int *failures, const std::string &what,
                        const std::function<UsdStageRefPtr()> &make,
                        const SdfPath &rigPath,
                        const EvaluationState &before,
                        const EvaluationState &after)
{
    const std::string where = what + " (history independence)";
    const UsdStageRefPtr warmStage = make();
    const UsdStageRefPtr freshStage = make();
    if (!warmStage || !freshStage) {
        ++*failures;
        std::printf("FAIL %s: the fixture does not open\n", where.c_str());
        return;
    }
    rigExec::RigExecRigEvaluator warm(warmStage, rigPath);
    rigExec::RigExecRigEvaluator fresh(freshStage, rigPath);
    std::vector<std::string> errors;
    if (!warm.Compile(&errors) || !fresh.Compile(&errors)) {
        ++*failures;
        std::printf("FAIL %s: the fixture does not compile\n",
                    where.c_str());
        return;
    }
    const auto evaluate = [](rigExec::RigExecRigEvaluator &evaluator,
                             const EvaluationState &state) {
        if (state.overrides.empty()) {
            evaluator.ClearInteractiveOverrides();
        } else {
            evaluator.SetInteractiveOverrides(state.overrides);
        }
        return evaluator.Evaluate(state.time);
    };
    const rigExec::RigExecRigPose first = evaluate(warm, before);
    const rigExec::RigExecRigPose second = evaluate(warm, after);
    const rigExec::RigExecRigPose alone = evaluate(fresh, after);
    if (!first.valid || !second.valid || !alone.valid) {
        ++*failures;
        std::printf("FAIL %s: a generation is invalid\n", where.c_str());
        return;
    }
    if (first.solverFrames == second.solverFrames) {
        ++*failures;
        std::printf("FAIL %s: the two states publish the same solver "
                    "frames\n", where.c_str());
    }
    CompareEveryMap(failures, where + " after a previous generation", alone,
                    second);
    if (warm.GetBakedGenerationCount() != 2 ||
                  fresh.GetBakedGenerationCount() != 1) {
        ++*failures;
        std::printf("FAIL %s: %zu of 2 and %zu of 1 generation(s) came from "
                    "the program\n", where.c_str(),
                    warm.GetBakedGenerationCount(),
                    fresh.GetBakedGenerationCount());
    }
}

}  // namespace rigExecTest

#endif  // RIGEXEC_TESTS_POSE_COMPARE_H

// Live versus frame-cache worker parity for an operator's own tests: one
// warming job through the production step runner must reproduce live
// evaluation of the same time bit for bit.
#ifndef RIGEXEC_TEST_FROZEN_PARITY_H
#define RIGEXEC_TEST_FROZEN_PARITY_H

#include "rigExec/backgroundScheduler.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"

#include <memory>
#include <string>

/// Freezes \p evaluator's program, samples \p time's inputs, runs one
/// warming job and compares its pose with a live evaluation at \p time.
/// False with the reason: a refused freeze or sample, a declined job, or
/// the first parity mismatches.
inline bool
RigExecFrozenMatchesLive(rigExec::RigExecRigEvaluator *evaluator,
                         const PXR_NS::SdfPath &rig, PXR_NS::UsdTimeCode time,
                         std::string *why)
{
    using namespace rigExec;
    std::string error;
    if (!evaluator->Evaluate(time).valid) {
        *why = "the live evaluation is invalid";
        return false;
    }
    std::shared_ptr<const RigExecFrozenProgram> frozen;
    if (!RigExecFreezeProgram(*evaluator, &frozen, &error) || !frozen) {
        *why = "freeze refused: " + error;
        return false;
    }
    RigExecFrameInputs inputs;
    if (!RigExecSampleFrameInputs(*evaluator, time, {}, &inputs, &error)) {
        *why = "sampling refused: " + error;
        return false;
    }
    RigExecBackgroundScheduler scheduler(0);
    RigExecFrozenEvalContext context;
    context.epochDigest = evaluator->GetBindingEpochDigest();
    context.generation = scheduler.CurrentGeneration(rig);
    context.slotCount = evaluator->GetBakedProgram()->GetProviderCount();
    context.varyingInputCount = inputs.values.size();
    if (evaluator->GetPublishWeightFields()) {
        context.flags |= kRigExecFrozenPublishWeightFields;
    }
    if (evaluator->GetSolverGuidesEnabled()) {
        context.flags |= kRigExecFrozenSolverGuidesEnabled;
    }
    context.frozen = frozen.get();
    const RigExecRigPose warmed = RigExecEvaluateFrozen(
        context, inputs, RigExecMakeProductionStepRunner(), &scheduler, rig);
    if (!warmed.valid) {
        *why = "the warming job was declined";
        return false;
    }
    const RigExecRigPose live = evaluator->Evaluate(time);
    RigExecRigPose diff;
    RigExecComparePoses(live, warmed, &diff);
    if (diff.comparisonMismatches != 0 || !diff.diagnostics.empty()) {
        *why = std::to_string(diff.comparisonMismatches) + " mismatch(es)";
        for (const std::string &line : diff.diagnostics) {
            *why += "; " + line;
        }
        return false;
    }
    return true;
}

#endif

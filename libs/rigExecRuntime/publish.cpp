//
// rigExecRuntime epilogue (M2 framework).
//
// Ports of RigExecBakedPublishPose (bakedPose.cpp) and
// RigExecBakedPublishGeometry (bakedGeometry.cpp): step diagnostics in
// program order, fallback-joint lines, provider xforms, joint/control
// publication, weight fields, adjusters and moved properties. Guides
// are skipped: the runtime carries no tap request, which is the
// guides-disabled shape the baked path mirrors.
//

#include "rigExecRuntime/store.h"

#include <algorithm>
#include <utility>

namespace rigExec {

bool
RrPublishPose(RrProgram *program,
              std::vector<std::string> *poseDiagnostics,
              std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireConstants &constants = *program->constants;
    const std::vector<RigExecWireStep> &steps = *program->steps;

    store.providerXforms.clear();
    store.providerBaseXforms.clear();
    store.jointMatricesFinal.clear();
    store.jointFramesBase.clear();
    store.jointFramesFinal.clear();
    store.controlFrames.clear();

    for (const RigExecWireStep &step : steps) {
        if (step.kind == RigExecWireStepKind::PoseInterpolator) {
            continue;
        }
        switch (step.kind) {
        case RigExecWireStepKind::InfluenceFold:
        case RigExecWireStepKind::RevisionStatic:
        case RigExecWireStepKind::RevisionChunk:
        case RigExecWireStepKind::RevisionFuse:
        case RigExecWireStepKind::ChainStatus:
        case RigExecWireStepKind::Derived:
            continue;
        default:
            break;
        }
        const RrStepOutput &output =
            store.stepOutputs[size_t(&step - steps.data())];
        for (const std::string &diagnostic : output.diagnostics) {
            poseDiagnostics->push_back(diagnostic);
        }
    }

    // Fallback joints: one line per failing writer, in walk order,
    // stable-sorted by joint path.
    std::vector<std::pair<uint32_t, std::pair<uint32_t, int>>>
        fallbackJoints;
    std::map<uint32_t, uint32_t> lastPublishingWriter;
    for (const RigExecWireWalkStep &walk : program->poses->walkSteps) {
        if (!walk.solverBatch) {
            continue;
        }
        for (int si : walk.batchSolvers) {
            const RigExecWireSolver &s =
                program->poses->solvers[size_t(si)];
            for (size_t k = 0; k < s.outputs.size(); ++k) {
                const uint32_t jointPath =
                    meta.paths[size_t(s.outputs[k].first)];
                if (k < store.solverOutPresent[size_t(si)].size() &&
                    store.solverOutPresent[size_t(si)][k]) {
                    lastPublishingWriter[jointPath] = s.path;
                    continue;
                }
                int element = -1;
                const auto binding =
                    program->jointBindingIndex.find(jointPath);
                if (binding != program->jointBindingIndex.end()) {
                    const size_t row = binding->second;
                    const std::vector<uint32_t> &writers =
                        program->poses
                            ->jointBindingSolvers[row];
                    for (size_t w = 0; w < writers.size(); ++w) {
                        if (writers[w] == s.path) {
                            element = program->poses
                                          ->jointBindingElements[row][w];
                            break;
                        }
                    }
                }
                fallbackJoints.push_back(
                    {jointPath, {s.path, element}});
            }
        }
    }
    std::stable_sort(
        fallbackJoints.begin(), fallbackJoints.end(),
        [&](const std::pair<uint32_t, std::pair<uint32_t, int>> &a,
            const std::pair<uint32_t, std::pair<uint32_t, int>> &b) {
            return program->TextOrEmpty(a.first) <
                   program->TextOrEmpty(b.first);
        });
    for (const auto &entry : fallbackJoints) {
        const auto kept = lastPublishingWriter.find(entry.first);
        poseDiagnostics->push_back(
            "solver " + program->TextOrEmpty(entry.second.first) +
            " published no element " + std::to_string(entry.second.second) +
            " for joint " + program->TextOrEmpty(entry.first) + "; " +
            (kept == lastPublishingWriter.end()
                 ? std::string("joint fell back to its rest chain")
                 : "the joint keeps the frame " +
                       program->TextOrEmpty(kept->second) + " left"));
    }

    // Plain Xformables a constraint targets, in slot order.
    for (size_t k = 0; k < meta.xformSlots.size(); ++k) {
        const size_t slot = size_t(meta.xformSlots[k]);
        const RrPointFrame &frame = store.fin[size_t(store.finLast[slot])];
        if (!RrFrameUsable(frame)) {
            poseDiagnostics->push_back(
                "constraint target " +
                program->TextOrEmpty(meta.paths[slot]) +
                " has an invalid final frame; transform omitted");
            continue;
        }
        RrMat4d revised;
        revised.SetIdentity();
        if (RrPointsToMatrix(RrIdentityLandmarks(), frame.points,
                             &revised)) {
            store.providerXforms[meta.paths[slot]] = revised;
            store.providerBaseXforms[meta.paths[slot]] =
                store.xformBase[k];
        }
    }

    // Joint publication, decided first so a frame that cannot publish
    // returns before a single key is inserted.
    store.jointMatrixPublished.assign(meta.jointSlots.size(), 0);
    for (size_t k = 0; k < meta.jointSlots.size(); ++k) {
        const size_t slot = size_t(meta.jointSlots[k]);
        const RrPointFrame &finalFrame =
            store.fin[size_t(store.finLast[slot])];
        if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {
            if (!RrFrameUsable(RrWireToFrame(constants.restFrames[slot])) ||
                !RrFrameUsable(finalFrame)) {
                if (error) {
                    *error = "joint " +
                             program->TextOrEmpty(meta.jointPaths[k]) +
                             " has an unusable rest or final frame";
                }
                return false;
            }
            store.jointMatrixPublished[k] = 1;
        } else {
            store.jointMatrixPublished[k] = 0;
            poseDiagnostics->push_back(
                "joint " + program->TextOrEmpty(meta.jointPaths[k]) +
                " has a degenerate final frame; matrix omitted");
        }
    }
    for (int index : meta.jointPublishOrder) {
        const size_t k = size_t(index);
        const size_t slot = size_t(meta.jointSlots[k]);
        store.jointFramesBase[meta.jointPaths[k]] =
            store.base[size_t(store.baseLast[slot])];
        store.jointFramesFinal[meta.jointPaths[k]] =
            store.fin[size_t(store.finLast[slot])];
        if (store.jointMatrixPublished[k]) {
            store.jointMatricesFinal[meta.jointPaths[k]] =
                store.finalMatrix[slot];
        }
    }
    for (int index : meta.controlPublishOrder) {
        const size_t k = size_t(index);
        const size_t slot = size_t(meta.controlSlots[k]);
        store.controlFrames[meta.controlPaths[k]] =
            store.fin[size_t(store.finLast[slot])];
    }

    for (const RigExecWireStep &step : steps) {
        if (step.kind != RigExecWireStepKind::PoseInterpolator) {
            continue;
        }
        const RrStepOutput &output =
            store.stepOutputs[size_t(&step - steps.data())];
        for (const std::string &diagnostic : output.diagnostics) {
            poseDiagnostics->push_back(diagnostic);
        }
    }
    return true;
}

void
RrPublishGeometry(RrProgram *program,
                  const RigExecWireFrameInputs &record,
                  std::vector<std::string> *poseDiagnostics)
{
    RrStore &store = program->store;
    const std::vector<RigExecWireStep> &steps = *program->steps;
    const RigExecWireDomainGeometry &geo = *program->geometry;

    store.movedProperties.clear();
    store.weightFields.clear();

    const auto publishAdjuster = [&](size_t chain, size_t revision) {
        const RigExecWireRevision &wire =
            geo.chains[chain].revisions[revision];
        const RrRevisionPublish &publish =
            store.revisionPublish[size_t(
                geo.chainRevisionBegin[chain] + int(revision))];
        // CurvenetAdjuster is op 11 in RigExecRevisionOp order.
        if (wire.op != 11 || publish.resultStatus != "ok") {
            return;
        }
        RrMat4d netToAsset;
        netToAsset.SetIdentity();
        if (chain < record.revisionAdjusters.size() &&
            revision < record.revisionAdjusters[chain].size() &&
            record.revisionAdjusterHave[chain][revision]) {
            const RigExecWireMatrix4d &wireMatrix =
                record.revisionAdjusters[chain][revision];
            for (size_t r = 0; r < 4; ++r) {
                for (size_t c = 0; c < 4; ++c) {
                    netToAsset[r][c] = wireMatrix[r * 4 + c];
                }
            }
        }
        for (size_t j = 0;
             j < std::min(publish.controlFrames.size(),
                          publish.adjusterPaths.size());
             ++j) {
            store.controlFrames[publish.adjusterPaths[j]] =
                RrFrameFromMatrix(publish.controlFrames[j] * netToAsset);
        }
    };

    for (const RigExecWireStep &step : steps) {
        const RrStepOutput &output =
            store.stepOutputs[size_t(&step - steps.data())];
        if (step.kind == RigExecWireStepKind::RevisionStatic) {
            for (const std::string &diagnostic : output.diagnostics) {
                poseDiagnostics->push_back(diagnostic);
            }
            const auto &entry =
                geo.revisionIndex[size_t(step.object)];
            const size_t chain = size_t(entry.first);
            if (!store.chainPublish[chain].haveBase) {
                continue;
            }
            const RrRevisionPublish &publish =
                store.revisionPublish[size_t(step.object)];
            if (!publish.weightFieldPublished) {
                continue;
            }
            const RigExecWireRevision &wire =
                geo.chains[chain]
                    .revisions[size_t(entry.second)];
            RrWeightFieldPublish field;
            field.target = publish.weightFieldTarget;
            field.weights = publish.weightField;
            store.weightFields[geo.weightObjects[size_t(wire.weightObject)]
                                   .path] = std::move(field);
        } else if (step.kind == RigExecWireStepKind::ChainStatus) {
            for (const std::string &diagnostic : output.diagnostics) {
                poseDiagnostics->push_back(diagnostic);
            }
            const size_t chain = size_t(step.object);
            if (!store.chainPublish[chain].haveBase) {
                continue;
            }
            for (size_t r = 0;
                 r < geo.chains[chain].revisions.size(); ++r) {
                publishAdjuster(chain, r);
            }
            store.movedProperties[store.chainPublish[chain].target] =
                store.chainPublish[chain].result;
        } else if (step.kind == RigExecWireStepKind::Derived) {
            for (const std::string &diagnostic : output.diagnostics) {
                poseDiagnostics->push_back(diagnostic);
            }
            const auto &entry = geo.derivedIndex[size_t(step.object)];
            const size_t chain = size_t(entry.first);
            const RrDerivedPublish &derived =
                store.derivedPublish[size_t(step.object)];
            if (store.chainPublish[chain].haveBase && derived.haveBase) {
                store.movedProperties[derived.target] = derived.result;
            }
        } else {
            switch (step.kind) {
            case RigExecWireStepKind::InfluenceFold:
            case RigExecWireStepKind::RevisionChunk:
            case RigExecWireStepKind::RevisionFuse:
                for (const std::string &diagnostic : output.diagnostics) {
                    poseDiagnostics->push_back(diagnostic);
                }
                break;
            default:
                break;
            }
        }
    }
}

}  // namespace rigExec

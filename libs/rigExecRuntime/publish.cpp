// rigExecRuntime epilogue (M2 framework).
// Ports of RigExecBakedPublishPose (bakedPose.cpp) and
// RigExecBakedPublishGeometry (bakedGeometry.cpp): step diagnostics in
// program order, fallback-joint lines, provider xforms, joint/control
// publication, weight fields and moved properties. Guides
// are skipped: the runtime carries no tap request, which is the
// guides-disabled shape the baked path mirrors.
#include "rigExecRuntime/poseInternal.h"
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
    const std::vector<RigExecWireStep> &steps = *program->steps;

    std::vector<char> aliveSolver(program->poses->solvers.size(), 0);
    for (const RigExecWireStep &step : steps)
        if (step.kind == RigExecWireStepKind::Solve)
            aliveSolver[size_t(step.object)] = 1;

    store.providerXforms.clear();
    store.providerBaseXforms.clear();
    store.jointMatricesFinal.clear();
    store.jointFramesBase.clear();
    store.jointFramesFinal.clear();
    store.controlFrames.clear();

    // Property reporting follows its immutable chain/part inventory,
    // independently of the canonical execution graph's interleaving.
    std::vector<size_t> propertyDiagnostics;
    for(size_t i=0;i<steps.size();++i)
        if(steps[i].kind==RigExecWireStepKind::PropertyRevision &&
           (!store.stepOutputs[i].diagnostics.empty() ||
            (i<store.headLines.size() && !store.headLines[i].empty())))
            propertyDiagnostics.push_back(i);
    std::sort(propertyDiagnostics.begin(),propertyDiagnostics.end(),
        [&](size_t a,size_t b) {
            return std::make_pair(steps[a].object,steps[a].part)<
                   std::make_pair(steps[b].object,steps[b].part);
        });
    for(size_t i:propertyDiagnostics) {
        poseDiagnostics->insert(poseDiagnostics->end(),
            store.stepOutputs[i].diagnostics.begin(),store.stepOutputs[i].diagnostics.end());
        if(i<store.headLines.size())
            poseDiagnostics->insert(poseDiagnostics->end(),
                store.headLines[i].begin(),store.headLines[i].end());
    }

    for (const RigExecWireStep &step : steps) {
        if(step.kind==RigExecWireStepKind::PropertyRevision)continue;
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
            if (!aliveSolver[size_t(si)]) continue;
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
                            ->jointBindingSolvers[row].v;
                    for (size_t w = 0; w < writers.size(); ++w) {
                        if (writers[w] == s.path) {
                            element = program->poses
                                          ->jointBindingElements[row].v[w];
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
    // returns before a single key is inserted. The rest tested is this
    // run's composed one, which a recompose can make unusable.
    store.jointMatrixPublished.assign(meta.jointSlots.size(), 0);
    for (size_t k = 0; k < meta.jointSlots.size(); ++k) {
        const size_t slot = size_t(meta.jointSlots[k]);
        const RrPointFrame &finalFrame =
            store.fin[size_t(store.finLast[slot])];
        if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {
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
    // Pose outputs publish after property chains, as the native epilogue does.
    // The current typed weight also includes disabled or failed solve zeros.
    for (size_t k = 0; k < program->poses->poseWeightPaths.size(); ++k) {
        RrPropertyValue value;
        value.tag = RrPropertyValue::Tag::Float;
        value.f32 = store.poseWeights[k];
        store.propertyResults[program->poses->poseWeightPaths[k]] = value;
    }
    return true;
}

void
RrPublishGeometry(RrProgram *program,
                  std::vector<std::string> *poseDiagnostics)
{
    RrStore &store = program->store;
    const std::vector<RigExecWireStep> &steps = *program->steps;
    const RigExecWireDomainGeometry &geo = *program->geometry;

    store.movedProperties.clear();
    store.movedMatrices.clear();
    store.weightFields.clear();

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
            field.weights = publish.weightField.Read();
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
            store.movedProperties[store.chainPublish[chain].target] =
                store.chainPublish[chain].result.Read();
        } else if (step.kind == RigExecWireStepKind::Derived) {
            for (const std::string &diagnostic : output.diagnostics) {
                poseDiagnostics->push_back(diagnostic);
            }
            const auto &entry = geo.derivedIndex[size_t(step.object)];
            const size_t chain = size_t(entry.first);
            const RrDerivedPublish &derived =
                store.derivedPublish[size_t(step.object)];
            if (store.chainPublish[chain].haveBase && derived.haveBase) {
                if (!derived.matrixTarget) {
                    store.movedProperties[derived.target] = derived.result.Read();
                } else if (derived.haveMatrix) {
                    store.movedMatrices[derived.target] = derived.matrix;
                }
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

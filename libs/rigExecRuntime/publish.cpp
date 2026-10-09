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
#include <cstddef>
#include <utility>

namespace rigExec {

namespace {

using RrEpilogue = RrProgram::EpilogueIndex;

// The step-output blocks, as RigExecBakedStepLines names them.
enum class RrStepLines { Property, Walk, Interpolator, Geometry };

bool
_RrHoldsOutput(const RrStore &store, size_t i)
{
    return (i < store.stepOutputs.size() &&
            !store.stepOutputs[i].diagnostics.empty()) ||
           (i < store.headLines.size() && !store.headLines[i].empty());
}

// The geometry steps whose diagnostics the geometry epilogue publishes;
// the pose epilogue's walk skips them.
bool
_RrPublishedGeometryKind(RigExecWireStepKind kind)
{
    switch (kind) {
    case RigExecWireStepKind::InfluenceFold:
    case RigExecWireStepKind::RevisionStatic:
    case RigExecWireStepKind::RevisionChunk:
    case RigExecWireStepKind::RevisionFuse:
    case RigExecWireStepKind::ChainStatus:
    case RigExecWireStepKind::Derived:
        return true;
    default:
        return false;
    }
}

size_t
_RrSolverCount(const RrProgram &program)
{
    return program.poses ? program.poses->solvers.size() : 0;
}

void
_RrEpilogueLists(const RrProgram &program, std::vector<char> *aliveSolver,
                 std::vector<uint32_t> *geometrySteps)
{
    const std::vector<RigExecWireStep> &steps = *program.steps;
    aliveSolver->assign(_RrSolverCount(program), 0);
    geometrySteps->clear();
    for (uint32_t i = 0; i < steps.size(); ++i) {
        const RigExecWireStepKind kind = steps[i].kind;
        if (kind == RigExecWireStepKind::Solve && steps[i].object >= 0 &&
            size_t(steps[i].object) < aliveSolver->size())
            (*aliveSolver)[size_t(steps[i].object)] = 1;
        if (kind == RigExecWireStepKind::RevisionStatic ||
            kind == RigExecWireStepKind::ChainStatus ||
            kind == RigExecWireStepKind::Derived)
            geometrySteps->push_back(i);
    }
}

void
_RrEnsureEpilogueIndex(RrProgram *program)
{
    const RrEpilogue &E = program->epilogue;
    if (E.held.holding.size() != program->steps->size() ||
        E.aliveSolver.size() != _RrSolverCount(*program))
        RrIndexEpilogue(program);
}

// One block's lines from the step indices \p visit hands over, which must
// come in step order.
template <class Visit>
void
_RrAppendBlock(const RrProgram &program, RrStepLines block,
               const Visit &visit, std::vector<std::string> *out)
{
    const RrStore &store = program.store;
    const std::vector<RigExecWireStep> &steps = *program.steps;
    const auto diagnostics = [&](size_t i) {
        out->insert(out->end(), store.stepOutputs[i].diagnostics.begin(),
                    store.stepOutputs[i].diagnostics.end());
    };
    switch (block) {
    case RrStepLines::Property: {
        // Property reporting follows its immutable chain/part inventory,
        // independently of the canonical execution graph's interleaving.
        std::vector<size_t> property;
        visit([&](size_t i) {
            if (steps[i].kind == RigExecWireStepKind::PropertyRevision &&
                _RrHoldsOutput(store, i))
                property.push_back(i);
        });
        std::sort(property.begin(), property.end(), [&](size_t a, size_t b) {
            return std::make_pair(steps[a].object, steps[a].part) <
                   std::make_pair(steps[b].object, steps[b].part);
        });
        for (size_t i : property) {
            diagnostics(i);
            if (i < store.headLines.size())
                out->insert(out->end(), store.headLines[i].begin(),
                            store.headLines[i].end());
        }
        return;
    }
    case RrStepLines::Walk:
        visit([&](size_t i) {
            const RigExecWireStepKind kind = steps[i].kind;
            if (kind != RigExecWireStepKind::PropertyRevision &&
                kind != RigExecWireStepKind::PoseInterpolator &&
                !_RrPublishedGeometryKind(kind))
                diagnostics(i);
        });
        return;
    case RrStepLines::Interpolator:
        visit([&](size_t i) {
            if (steps[i].kind == RigExecWireStepKind::PoseInterpolator)
                diagnostics(i);
        });
        return;
    case RrStepLines::Geometry:
        visit([&](size_t i) {
            if (_RrPublishedGeometryKind(steps[i].kind)) diagnostics(i);
        });
        return;
    }
}

// Appends \p block from the held steps; under `epilogue.verify` also from
// every step, counting a difference.
void
_RrAppendStepLines(RrProgram *program, RrStepLines block,
                   std::vector<std::string> *out)
{
    RrEpilogue &E = program->epilogue;
    const std::vector<uint32_t> &held = E.held.Ascending();
    const size_t begin = out->size();
    _RrAppendBlock(*program, block, [&](const auto &each) {
        for (const uint32_t i : held) each(size_t(i));
    }, out);
    if (!E.verify) {
        return;
    }
    std::vector<std::string> swept;
    _RrAppendBlock(*program, block, [&](const auto &each) {
        for (size_t i = 0; i < program->steps->size(); ++i) each(i);
    }, &swept);
    if (swept.size() != out->size() - begin ||
        !std::equal(swept.begin(), swept.end(),
                    out->begin() + std::ptrdiff_t(begin))) {
        ++E.mismatches;
    }
}

} // namespace

void
RrIndexEpilogue(RrProgram *program)
{
    RrEpilogue &E = program->epilogue;
    _RrEpilogueLists(*program, &E.aliveSolver, &E.geometrySteps);
    const size_t count = program->steps->size();
    E.held.Reset(count);
    for (uint32_t i = 0; i < count; ++i)
        E.held.Note(i, _RrHoldsOutput(program->store, i));
}

void
RrFoldHeldSteps(RrProgram *program)
{
    RrEpilogue &E = program->epilogue;
    if (E.held.holding.size() != program->steps->size()) {
        RrIndexEpilogue(program);
        return;
    }
    const std::vector<char> &ran = program->store.opExecution.ran;
    for (uint32_t c = 0; c < program->opGraph.ops.size() && c < ran.size(); ++c) {
        if (!ran[c]) continue;
        const uint32_t i = program->opGraph.ops[c].originalIndex;
        E.held.Note(i, _RrHoldsOutput(program->store, i));
    }
}

bool
RrPublishPose(RrProgram *program,
              std::vector<std::string> *poseDiagnostics,
              std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireSlotMeta &meta = *program->slotMeta;

    _RrEnsureEpilogueIndex(program);
    if (program->epilogue.verify) {
        std::vector<char> aliveSolver;
        std::vector<uint32_t> geometrySteps;
        _RrEpilogueLists(*program, &aliveSolver, &geometrySteps);
        bool same = aliveSolver == program->epilogue.aliveSolver &&
                    geometrySteps == program->epilogue.geometrySteps;
        for (size_t i = 0; same && i < program->steps->size(); ++i)
            same = (program->epilogue.held.holding[i] != 0) ==
                   _RrHoldsOutput(store, i);
        if (!same) ++program->epilogue.mismatches;
    }
    const std::vector<char> &aliveSolver = program->epilogue.aliveSolver;

    store.providerXforms.clear();
    store.providerBaseXforms.clear();
    store.jointMatricesFinal.clear();
    store.jointFramesBase.clear();
    store.jointFramesFinal.clear();
    store.controlFrames.clear();

    _RrAppendStepLines(program, RrStepLines::Property, poseDiagnostics);
    _RrAppendStepLines(program, RrStepLines::Walk, poseDiagnostics);

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

    _RrAppendStepLines(program, RrStepLines::Interpolator, poseDiagnostics);
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

    _RrEnsureEpilogueIndex(program);
    _RrAppendStepLines(program, RrStepLines::Geometry, poseDiagnostics);
    // The publications in program order: the maps are keyed, so the order
    // shows only where two steps name one key, and the later step's stands.
    for (const uint32_t index : program->epilogue.geometrySteps) {
        const RigExecWireStep &step = steps[index];
        if (step.kind == RigExecWireStepKind::RevisionStatic) {
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
            const size_t chain = size_t(step.object);
            if (!store.chainPublish[chain].haveBase) {
                continue;
            }
            store.movedProperties[store.chainPublish[chain].target] =
                store.chainPublish[chain].result.Read();
        } else if (step.kind == RigExecWireStepKind::Derived) {
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
        }
    }
}

}  // namespace rigExec

// rigExecRuntime step labels: RigExecBakedStepKindName and StepLabel
// (bakedSchedule.cpp) over the tables, so a file needs no label text.
#include "rigExecRuntime/labels.h"
#include "rigExecRuntime/store.h"

namespace rigExec {

namespace {

const char *
_RrStepKindName(RigExecWireStepKind kind)
{
    switch (kind) {
    case RigExecWireStepKind::ComposeSubtree: return "ComposeSubtree";
    case RigExecWireStepKind::Solve: return "Solve";
    case RigExecWireStepKind::SolverCommit: return "SolverCommit";
    case RigExecWireStepKind::Constraint: return "Constraint";
    case RigExecWireStepKind::CommitDelta: return "CommitDelta";
    case RigExecWireStepKind::PropagateChunk: return "PropagateChunk";
    case RigExecWireStepKind::CommitApply: return "CommitApply";
    case RigExecWireStepKind::ProviderMatrix: return "ProviderMatrix";
    case RigExecWireStepKind::SnapshotFinals: return "SnapshotFinals";
    case RigExecWireStepKind::PoseInterpolator: return "PoseInterpolator";
    case RigExecWireStepKind::VolumePlacements: return "VolumePlacements";
    case RigExecWireStepKind::WeightPacket: return "WeightPacket";
    case RigExecWireStepKind::InfluenceFold: return "InfluenceFold";
    case RigExecWireStepKind::RevisionStatic: return "RevisionStatic";
    case RigExecWireStepKind::RevisionChunk: return "RevisionChunk";
    case RigExecWireStepKind::RevisionFuse: return "RevisionFuse";
    case RigExecWireStepKind::ChainStatus: return "ChainStatus";
    case RigExecWireStepKind::Derived: return "Derived";
    }
    return "unknown";
}

template <class Table>
bool
_RrHas(const Table &table, int64_t index)
{
    return index >= 0 && uint64_t(index) < uint64_t(table.size());
}

// The text after the kind: the path of the object \p step works on, in
// StepLabel's words. False when an index is past its table.
bool
_RrStepObject(const RrProgram &program, const RigExecWireStep &step,
              std::string *out)
{
    const RigExecWireSlotMeta *meta = program.slotMeta;
    const RigExecWireDomainPose *poses = program.poses;
    const RigExecWireDomainGeometry *geometry = program.geometry;
    const int32_t object = step.object;
    switch (step.kind) {
    case RigExecWireStepKind::ComposeSubtree: {
        if (!poses || !meta || !_RrHas(poses->composeGroups, object)) {
            return false;
        }
        const int32_t begin = poses->composeGroups[size_t(object)].begin;
        if (!_RrHas(meta->paths, begin)) {
            return false;
        }
        *out = program.TextOrEmpty(meta->paths[size_t(begin)]);
        return true;
    }
    case RigExecWireStepKind::Solve:
        if (!poses || !_RrHas(poses->solvers, object)) {
            return false;
        }
        *out = program.TextOrEmpty(poses->solvers[size_t(object)].path);
        return true;
    case RigExecWireStepKind::SolverCommit:
    case RigExecWireStepKind::Constraint:
    case RigExecWireStepKind::CommitDelta:
    case RigExecWireStepKind::PropagateChunk:
    case RigExecWireStepKind::CommitApply: {
        if (!poses || !_RrHas(poses->commits, object)) {
            return false;
        }
        // An empty mover path is a batch of commits with no one mover.
        const std::string mover =
            program.TextOrEmpty(poses->commits[size_t(object)].moverPath);
        *out = mover.empty() ? "batch " + std::to_string(object) : mover;
        return true;
    }
    case RigExecWireStepKind::ProviderMatrix:
        if (!meta || !_RrHas(meta->paths, object)) {
            return false;
        }
        *out = program.TextOrEmpty(meta->paths[size_t(object)]) +
               (step.part ? " final" : " base");
        return true;
    case RigExecWireStepKind::SnapshotFinals:
        *out = "every provider";
        return true;
    case RigExecWireStepKind::PoseInterpolator:
        if (!poses || !_RrHas(poses->poseInterpolators, object)) {
            return false;
        }
        *out = program.TextOrEmpty(
            poses->poseInterpolators[size_t(object)].path);
        return true;
    case RigExecWireStepKind::VolumePlacements:
        *out = "every volume weight";
        return true;
    case RigExecWireStepKind::WeightPacket:
        if (!geometry || !_RrHas(geometry->weightObjects, object)) {
            return false;
        }
        *out = program.TextOrEmpty(
            geometry->weightObjects[size_t(object)].path);
        return true;
    case RigExecWireStepKind::InfluenceFold:
    case RigExecWireStepKind::RevisionStatic:
    case RigExecWireStepKind::RevisionChunk:
    case RigExecWireStepKind::RevisionFuse: {
        if (!geometry || !_RrHas(geometry->revisionIndex, object)) {
            return false;
        }
        const auto &entry = geometry->revisionIndex[size_t(object)];
        if (!_RrHas(geometry->chains, entry.first) ||
            !_RrHas(geometry->chains[size_t(entry.first)].revisions,
                    entry.second)) {
            return false;
        }
        *out = program.TextOrEmpty(geometry->chains[size_t(entry.first)]
                                       .revisions[size_t(entry.second)]
                                       .moverPath);
        return true;
    }
    case RigExecWireStepKind::ChainStatus:
        if (!geometry || !_RrHas(geometry->chains, object)) {
            return false;
        }
        *out = program.TextOrEmpty(geometry->chains[size_t(object)].target);
        return true;
    case RigExecWireStepKind::Derived: {
        if (!geometry || !_RrHas(geometry->derivedIndex, object)) {
            return false;
        }
        const auto &entry = geometry->derivedIndex[size_t(object)];
        if (!_RrHas(geometry->chains, entry.first) ||
            !_RrHas(geometry->chains[size_t(entry.first)].derived,
                    entry.second)) {
            return false;
        }
        *out = program.TextOrEmpty(geometry->chains[size_t(entry.first)]
                                       .derived[size_t(entry.second)]
                                       .target);
        return true;
    }
    }
    out->clear();
    return true;
}

}  // namespace

std::string
RrStepLabel(const RrProgram &program, size_t step)
{
    if (!program.steps || step >= program.steps->size()) {
        return std::to_string(step);
    }
    const RigExecWireStep &wire = (*program.steps)[step];
    std::string object;
    if (!_RrStepObject(program, wire, &object)) {
        object = std::to_string(wire.object);
    }
    return std::string(_RrStepKindName(wire.kind)) + " " + object;
}

}  // namespace rigExec

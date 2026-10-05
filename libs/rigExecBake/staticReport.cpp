// .rigexec time-variance audit.
#include "rigExecBake/staticReport.h"
#include "rigExecBake/capture.h"
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/revisionReads.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/container.h"

#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <functional>
#include <set>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

using _Entries = std::set<std::pair<std::string, std::string>>;

// Whether \p prim's own transform might vary: an op that might vary, or one
// holding a single time sample (which reads back at a numeric time only, as
// RigExecBakedAnimatedOrConnected counts it).
bool
_TransformAnimated(const UsdPrim &prim)
{
    const UsdGeomXformable xformable(prim);
    if (!xformable) {
        return false;
    }
    if (xformable.TransformMightBeTimeVarying()) {
        return true;
    }
    bool resets = false;
    for (const UsdGeomXformOp &op : xformable.GetOrderedXformOps(&resets)) {
        if (op.GetAttr().GetNumTimeSamples() > 0) {
            return true;
        }
    }
    return false;
}

// A transform read relative to \p root (UsdGeomXformCache::
// ComputeRelativeTransform): the prim's own and every ancestor's up to the
// root, exclusive, or up to the pseudo-root when the root is not above it.
void
_Transform(const UsdStageRefPtr &stage, const UsdPrim &root,
           const SdfPath &path, const std::string &field, _Entries *entries)
{
    for (UsdPrim p = stage->GetPrimAtPath(path);
         p && !p.IsPseudoRoot() && p != root; p = p.GetParent()) {
        if (_TransformAnimated(p)) {
            entries->emplace(field, p.GetPath().GetString());
        }
    }
}

void
_Attribute(const UsdAttribute &attribute, const std::string &field,
           _Entries *entries)
{
    if (RigExecBakedAnimatedOrConnected(attribute)) {
        entries->emplace(field, attribute.GetPath().GetString());
    }
}

void
_Attribute(const UsdStageRefPtr &stage, const SdfPath &path,
           const std::string &field, _Entries *entries)
{
    if (!path.IsEmpty() && path.IsPropertyPath()) {
        _Attribute(stage->GetAttributeAtPath(path), field, entries);
    }
}

// Whether the computed section evaluates the attribute at a path over the
// input slots (a connection-following scalar the assembly reads), rather
// than the record replaying it.
using _SlotDriven = std::function<bool(const SdfPath &)>;

// A revision's stage-routed dense blend-sample points, the shapes of its
// sparse samples (read per run when the cache refuses one, which a
// connection makes it do), the binding arrays its assembly reads as static
// path reads, and every other path read its assembly makes at the time
// that no input slot recomputes (structural tokens, lattice divisions, a
// wire's dropoff and frames, extent widths, a per-run skin layout).
void
_Revision(const RigExecBakedProgramImpl &program, double time,
          const _SlotDriven &slotDriven,
          const RigExecBakedProgramImpl::GeomRevision &revision,
          _Entries *entries)
{
    const UsdStageRefPtr &stage = program.stage;
    for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
         revision.blendChannels) {
        for (const auto &sample : channel.samples) {
            if (sample.blendShape.IsEmpty()) {
                if (sample.phase.IsBase()) {
                    _Attribute(sample.points,
                               "blend sample points " +
                                   sample.samplePath.GetString(),
                               entries);
                }
                continue;
            }
            const UsdPrim shape =
                stage->GetPrimAtPath(sample.blendShape.GetPrimPath());
            if (!shape) {
                continue;
            }
            const std::string field =
                "blend sample layout " + sample.samplePath.GetString();
            for (const char *name : {"offsets", "pointIndices"}) {
                _Attribute(shape.GetAttribute(TfToken(name)), field, entries);
            }
        }
    }
    const RigExecRevisionBinding &binding = revision.binding;
    const std::string field = "revision binding " +
                              revision.moverPath.GetString();
    const std::set<SdfPath> bindingPaths = {
        binding.cagePoints,       binding.surfacePoints,
        binding.bindCoords,       binding.driverCurvePoints,
        binding.driverCurveOrder, binding.driverCurveKnots,
        binding.topologyCounts,   binding.topologyIndices};
    for (const SdfPath &path : bindingPaths) {
        _Attribute(stage, path, field, entries);
    }
    const std::string readField = "revision read " +
                                  revision.moverPath.GetString();
    RigExecBakeEnumerateRevisionReads(
        program, revision, time, [&](RigExecBakeRevisionRead &&read) {
            // A read at Default sees no time sample.
            if (read.rest || bindingPaths.count(read.path) ||
                !read.path.IsPropertyPath()) {
                return;
            }
            const UsdAttribute attribute =
                stage->GetAttributeAtPath(read.path);
            if (RigExecBakedAnimatedOrConnected(attribute) &&
                !slotDriven(read.path)) {
                entries->emplace(readField, read.path.GetString());
            }
        });
}

}  // namespace

bool
RigExecBakeStaticReport(RigExecRigEvaluator &evaluator,
                        std::vector<RigExecBakeStaticEntry> *entries,
                        std::string *error)
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    if (!entries) {
        return Fail("no entries to report into");
    }
    entries->clear();
    if (!evaluator.GetBakedProgram()) {
        std::vector<std::string> compileErrors;
        if (!evaluator.Compile(&compileErrors)) {
            std::string joined = "compile failed";
            for (const std::string &message : compileErrors) {
                joined += "\n  " + message;
            }
            return Fail(joined);
        }
    }
    const RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    if (!baked) {
        return Fail("no baked program stands: the evaluator is not in a "
                    "baked mode, or the epoch does not bake");
    }
    const RigExecBakedProgramImpl &B = baked->GetStepGraph();
    const UsdStageRefPtr &stage = B.stage;
    const double probe = RigExecBakedProbeTime(stage).GetValue();
    _Entries found;

    // The captures read the standing program and the stage; neither
    // evaluates. The computed section names the reads the slots evaluate
    // and the oracle facts of the objects the runtime resolves.
    RigExecBinaryWriter writer;
    std::string why;
    RigExecBakeCapture capture(evaluator, &writer, &why);
    if (!capture.Valid()) {
        return Fail(why);
    }
    RigExecBakeComputedCapture computed(evaluator, capture, probe, &writer,
                                        &why);
    if (!computed.Valid()) {
        return Fail(why);
    }
    std::set<uint32_t> slotPaths;
    for (const RigExecWirePathScalarRead &read :
         computed.GetComputed().pathScalarReads) {
        slotPaths.insert(read.path);
    }
    // The writer interns: a path the section names has its id already.
    const _SlotDriven slotDriven = [&](const SdfPath &path) {
        return slotPaths.count(writer.AddString(path.GetString())) != 0;
    };

    // Transforms the prologue reads off the stage.
    for (const int slot : B.xformSlots) {
        const SdfPath &path = B.paths[size_t(slot)];
        _Transform(stage, B.assetRoot, path, "xform base " + path.GetString(),
                   &found);
    }
    for (const RigExecBakedProgramImpl::NativeXformSource &source :
         B.nativeSources) {
        _Transform(stage, B.assetRoot, source.path,
                   "native frame " + source.path.GetString(), &found);
    }
    for (const SdfPath &path : B.deltaBasePaths) {
        _Transform(stage, B.assetRoot, path, "delta base " + path.GetString(),
                   &found);
    }

    // Constraint tables, read raw at the frame's time.
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        if (!arrays.prim) {
            continue;
        }
        const std::string field =
            "constraint arrays " + arrays.prim.GetPath().GetString();
        std::vector<const char *> names = {"inputs:sourceWeights"};
        if (arrays.parentOffsets) {
            names.push_back("inputs:translationOffsets");
            names.push_back("inputs:rotationOffsets");
        }
        if (arrays.readPole) {
            names.push_back("inputs:poleVectorWeights");
        }
        for (const char *name : names) {
            _Attribute(arrays.prim.GetAttribute(TfToken(name)), field,
                       &found);
        }
    }

    // Ribbon driver points.
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        if (solver.ribbonPointsVarying) {
            found.emplace("ribbon points " + solver.path.GetString(),
                          solver.ribbonPointsPath.GetString());
        }
    }

    // Point chains: authored bases, blend-sample points, binding arrays.
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (chain.baseQuery.IsValid()) {
            _Attribute(chain.baseQuery.GetAttribute(),
                       "chain base " + chain.target.GetString(), &found);
        }
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            _Revision(B, probe, slotDriven, revision, &found);
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (derived.baseQuery.IsValid()) {
                _Attribute(derived.baseQuery.GetAttribute(),
                           "derived base " + derived.target.GetString(),
                           &found);
            }
            _Revision(B, probe, slotDriven, derived.revision, &found);
        }
    }

    // The arrays a WeightPacket step gathers.
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        const std::string field =
            "weight object " + object.path.GetString() + " gather";
        for (const std::vector<UsdAttribute> *attributes :
             {&object.combineTargetPoints, &object.targetPoints,
              &object.samplePoints, &object.curvePoints}) {
            for (const UsdAttribute &attribute : *attributes) {
                _Attribute(attribute, field, &found);
            }
        }
    }

    // The oracle facts of the objects the runtime resolves.
    for (const RigExecBakeTimeVaryingFact &fact :
         computed.GetTimeVaryingFacts()) {
        found.emplace("weight object " + fact.object + " oracle",
                      fact.attribute);
    }

    entries->reserve(found.size());
    for (const auto &[field, source] : found) {
        entries->push_back(RigExecBakeStaticEntry{field, source});
    }
    return true;
}

}  // namespace rigExec

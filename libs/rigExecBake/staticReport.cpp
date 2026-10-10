// .rigexec time-variance audit.
#include "rigExecBake/staticReport.h"
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/pathTable.h"
#include "rigExecBake/revisionReads.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"

#include "rigExecRuntime/stageArrayInputs.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
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

// Whether the file evaluates the attribute at a path over the input slots
// (a connection-following scalar the assembly reads, a read row), rather
// than holding its value at the bake time.
using _SlotDriven = std::function<bool(const SdfPath &)>;

// Whether \p binding can answer a run: one of its candidate chains has a
// base value to read. A chain whose target holds no value at all never
// reads a base, so a binding of such chains alone always falls through to
// its tail, which live baked reads off the stage on every run and playback
// reads as the points the bake held. A base that is missing at some times
// only is time-varying, and the report names it as a chain base.
bool
_CanAnswer(const RigExecBakedProgramImpl &program,
           const RigExecBakedPointsBinding &binding)
{
    if (binding.id < 0) {
        return false;
    }
    for (const RigExecBakedPointVersion &candidate : binding.candidates) {
        if (candidate.chain < 0 ||
            size_t(candidate.chain) >= program.chains.size()) {
            continue;
        }
        const UsdAttributeQuery &base =
            program.chains[size_t(candidate.chain)].baseQuery;
        if (base.IsValid() && base.HasValue()) {
            return true;
        }
    }
    return false;
}

// A revision's stage-routed dense blend-sample points (a base-phase
// sample's, and a phased one's whose binding can never answer), the shapes
// of its sparse samples (read per run when the cache refuses one, which a
// connection makes it do), the binding arrays its assembly reads as static
// path reads, and every other path read its assembly makes at the time
// that no input slot recomputes (structural tokens, lattice divisions, a
// wire's dropoff and frames, extent widths, a per-run skin layout).
void
_Revision(const RigExecBakedProgramImpl &program, double time,
          const _SlotDriven &slotDriven,
          const RigExecBakedProgramImpl::GeomRevision &revision,
          _Entries *entries, size_t chainIndex, size_t revisionIndex,
          bool derived)
{
    const UsdStageRefPtr &stage = program.stage;
    for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
         revision.blendChannels) {
        for (const auto &sample : channel.samples) {
            if (sample.blendShape.IsEmpty()) {
                if (sample.phase.IsBase() ||
                    !_CanAnswer(program, sample.pointBinding)) {
                    if (!slotDriven(sample.points.GetPath())) {
                        _Attribute(sample.points,
                                   "blend sample points " + sample.samplePath.GetString(),
                                   entries);
                    }
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
        if (!slotDriven(path)) _Attribute(stage, path, field, entries);
    }
    const std::string readField = "revision read " +
                                  revision.moverPath.GetString();
    RigExecBakeEnumerateRevisionReads(
        program, chainIndex, revisionIndex, derived, time,
        [&](RigExecBakeRevisionRead &&read) {
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
        return Fail("no compiled program stands for this epoch");
    }
    const RigExecBakedProgramImpl &B = baked->GetStepGraph();
    const UsdStageRefPtr &stage = B.stage;
    const double probe = RigExecBakedProbeTime(stage).GetValue();
    _Entries found;

    // The report is of the authored epoch, as a bake is.
    if (evaluator.HasInteractiveOverrides()) {
        return Fail("cannot bake with interactive overrides standing");
    }
    // The input collection reads the standing program and the stage; it
    // never evaluates. It names the reads the slots evaluate and the
    // oracle facts of the objects the runtime resolves.
    RigExecBakePathTable paths;
    std::string why;
    RigExecBakeComputedCapture computed(evaluator, probe, &paths, &why);
    if (!computed.Valid()) {
        return Fail(why);
    }
    std::set<std::string> slotPaths;
    for (const RigExecBakePathScalarRead &read :
         computed.GetInputs().pathScalarReads) {
        slotPaths.insert(paths.Text(read.path));
    }
    // Use the same consumer inventory as the stage sampler. This small
    // wire view needs no constants, geometry payloads or serialization.
    const auto &capture = computed.GetInputs();
    fb::RigExecWireFile view;
    view.inputs = capture.inputs;
    view.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    view.geometry->weightObjects = capture.weightObjects;
    for (const auto &row : capture.arrayRows) {
        fb::RigExecWirePathRead wire;
        wire.rest = row.rest;
        wire.read = std::make_unique<fb::RigExecWireInput>(row.read);
        view.geometry->pathReads.push_back(std::move(wire));
    }
    // Dense sample reads use the same walk inventory as path reads.
    for (const auto &sample : capture.blendPoints) {
        fb::RigExecWirePathRead wire;
        wire.rest = false;
        wire.read = std::make_unique<fb::RigExecWireInput>(sample.read);
        view.geometry->pathReads.push_back(std::move(wire));
    }
    for (int32_t slot : capture.chainBaseSlots) {
        fb::RigExecWireChain chain;
        chain.baseSlot = slot;
        view.geometry->chains.push_back(std::move(chain));
    }
    for (const auto &layout : capture.layoutSlots) {
        fb::RigExecWireRevision revision;
        revision.jointIndicesSlot = int32_t(layout.indices);
        revision.jointWeightsSlot = int32_t(layout.weights);
        fb::RigExecWireChain chain;
        chain.revisions.push_back(std::move(revision));
        view.geometry->chains.push_back(std::move(chain));
    }
    for (size_t slot : RigExecStageArraySlots(view)) {
        slotPaths.insert(paths.Text(capture.inputs[slot].name()));
    }
    const _SlotDriven slotDriven = [&](const SdfPath &path) {
        return slotPaths.count(path.GetString()) != 0;
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
    for (size_t chainIndex = 0; chainIndex < B.chains.size(); ++chainIndex) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[chainIndex];
        if (chain.baseQuery.IsValid() &&
            !slotDriven(chain.baseQuery.GetAttribute().GetPath())) {
            _Attribute(chain.baseQuery.GetAttribute(),
                       "chain base " + chain.target.GetString(), &found);
        }
        for (size_t revisionIndex = 0; revisionIndex < chain.revisions.size();
             ++revisionIndex) {
            _Revision(B, probe, slotDriven, chain.revisions[revisionIndex],
                      &found, chainIndex, revisionIndex, false);
        }
        for (size_t revisionIndex = 0; revisionIndex < chain.derived.size();
             ++revisionIndex) {
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[revisionIndex];
            if (derived.baseQuery.IsValid()) {
                _Attribute(derived.baseQuery.GetAttribute(),
                           "derived base " + derived.target.GetString(),
                           &found);
            }
            _Revision(B, probe, slotDriven, derived.revision, &found,
                      chainIndex, revisionIndex, true);
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
                if (!slotDriven(attribute.GetPath())) {
                    _Attribute(attribute, field, &found);
                }
            }
        }
    }

    // The oracle facts of the objects the runtime resolves.
    for (const RigExecBakeTimeVaryingFact &fact :
         computed.GetTimeVaryingFacts()) {
        if (!slotDriven(SdfPath(fact.attribute))) {
            found.emplace("weight object " + fact.object + " oracle",
                          fact.attribute);
        }
    }

    entries->reserve(found.size());
    for (const auto &[field, source] : found) {
        entries->push_back(RigExecBakeStaticEntry{field, source});
    }
    return true;
}

}  // namespace rigExec

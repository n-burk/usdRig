// .rigexec path-read enumeration.
#include "rigExecBake/revisionReads.h"
#include "rigExecBake/capture.h"
#include "rigExec/moverGraph.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <limits>
#include <map>
#include <set>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

// The names and fallbacks the recording sites use, spelled as they spell
// them.
TF_DEFINE_PRIVATE_TOKENS(
    _readTokens,
    ((enabled, "inputs:enabled"))
    ((weightBlend, "rigExec:weightBlend"))
    ((pointFrame, "rigExec:pointFrame"))
    ((driverDeltaFrame, "rigExec:driverDeltaFrame"))
    ((jointIndices, "rigExec:jointIndices"))
    ((jointWeights, "rigExec:jointWeights"))
    ((elementSize, "rigExec:elementSize"))
    ((skinningMethod, "rigExec:skinningMethod"))
    ((classicLinear, "classicLinear"))
    ((divisions, "rigExec:divisions"))
    ((restPoints, "inputs:restPoints"))
    ((iterations, "inputs:iterations"))
    ((step, "inputs:step"))
    ((pinBorders, "inputs:pinBorders"))
    ((distanceWeight, "inputs:distanceWeight"))
    ((displacement, "inputs:displacement"))
    ((topology, "inputs:topology"))
    ((cloth, "cloth"))
    ((pinPoints, "inputs:pinPoints"))
    ((neighborDistance, "inputs:neighborDistance"))
    ((restLengthScale, "inputs:restLengthScale"))
    ((stretchStiffness, "inputs:stretchStiffness"))
    ((compressionStiffness, "inputs:compressionStiffness"))
    ((bendStiffness, "inputs:bendStiffness"))
    ((maxDisplacement, "inputs:maxDisplacement"))
    ((tangentPlaneCollisions, "inputs:tangentPlaneCollisions"))
    ((tangentPlaneInset, "inputs:tangentPlaneInset"))
    ((wrinkleScale, "inputs:wrinkleScale"))
    ((smoothingIterations, "inputs:smoothingIterations"))
    ((driverWeights, "inputs:driverWeights"))
    ((driverBaseWeights, "inputs:driverBaseWeights"))
    ((rest, "rest"))
    ((local, "local"))
    ((dropoffDistance, "inputs:dropoffDistance"))
    ((rayOrigin, "rigExec:rayOrigin"))
    ((rayDirection, "rigExec:rayDirection"))
    ((rayUp, "rigExec:rayUp"))
    ((shaderOffset, "rigExec:shaderOffset"))
    ((projectionMode, "rigExec:projectionMode"))
    ((material, "material"))
    ((combineWeight, "RigExecCombineWeight"))
    ((sphereWeight, "RigExecSphereWeight"))
    ((planeWeight, "RigExecPlaneWeight"))
    ((curveWeight, "RigExecCurveWeight"))
);

namespace rigExec {
namespace {

// The sites read through the generation's resolved inputs, and so does the
// enumeration: a connection walk that reaches a property chain's target
// takes the chain's result, as the run's did. A key the overlay holds
// itself, at a site that consults it, is one the recorder skips; it is
// emitted marked overlaid.
struct _Enumeration {
    const RigExecResolvedInputs *resolved = nullptr;
    UsdTimeCode time;
    const RigExecBakeRevisionReadSink *sink = nullptr;

    void Emit(const SdfPath &path, UsdTimeCode at, bool resolved,
              VtValue value, bool overlaid = false) const
    {
        if (path.IsEmpty()) {
            return;
        }
        RigExecBakeRevisionRead read;
        read.path = path;
        read.rest = at == UsdTimeCode::Default();
        read.resolved = resolved;
        read.value = std::move(value);
        read.overlaid = overlaid;
        (*sink)(std::move(read));
    }

    // RigExecRecordStageRead's skip: the overlay holds the key itself.
    bool Holds(const SdfPath &path) const
    {
        return resolved && resolved->Find(path) != nullptr;
    }

    // RigExecRecordStageRead over a missing attribute: known absent, keyed
    // rest whatever the read's time.
    void Missing(const SdfPath &path) const
    {
        Emit(path, UsdTimeCode::Default(), false, VtValue());
    }
};

// _RecordedInput: a scalar on the mover, read through the resolved inputs
// (falling back to the raw value, then to the fallback) and recorded as
// following connections.
template <class T>
void
_ResolvedInput(const _Enumeration &E, const UsdPrim &prim,
               const TfToken &name, T fallback, UsdTimeCode at)
{
    const SdfPath key = prim.GetPath().AppendProperty(name);
    const UsdAttribute a = prim.GetAttribute(name);
    if (!a) {
        E.Missing(key);
        return;
    }
    T value = fallback;
    if (!E.resolved->GetAttribute(a, at, &value)) {
        a.Get(&value, at);
    }
    E.Emit(key, at, /*resolved=*/true, VtValue(value), E.Holds(key));
}

// The skin's skinningMethod and elementSize and the wire's driver weights:
// connection-following when the resolved read answers, a raw read
// otherwise.
template <class T>
void
_ResolvedOrRaw(const _Enumeration &E, const UsdPrim &prim,
               const TfToken &name, T fallback, UsdTimeCode at)
{
    const UsdAttribute a = prim.GetAttribute(name);
    if (!a) {
        E.Missing(prim.GetPath().AppendProperty(name));
        return;
    }
    T value = fallback;
    if (E.resolved->GetAttribute(a, at, &value)) {
        E.Emit(a.GetPath(), at, /*resolved=*/true, VtValue(value),
               E.Holds(a.GetPath()));
        return;
    }
    a.Get(&value, at);
    E.Emit(a.GetPath(), at, /*resolved=*/false, VtValue(value));
}

// _Enabled: both arms record as connection-following; only the resolved
// arm skips a key the overlay holds.
void
_Enabled(const _Enumeration &E, const UsdPrim &prim, UsdTimeCode at)
{
    const UsdAttribute a = prim.GetAttribute(_readTokens->enabled);
    if (!a) {
        E.Missing(prim.GetPath().AppendProperty(_readTokens->enabled));
        return;
    }
    bool enabled = true;
    const bool walked = E.resolved->GetAttribute(a, at, &enabled);
    if (!walked) {
        a.Get(&enabled, at);
    }
    E.Emit(a.GetPath(), at, /*resolved=*/true, VtValue(enabled),
           walked && E.Holds(a.GetPath()));
}

// A raw read of a mover attribute (_RecordedToken, the lattice's divisions,
// the wire's dropoff).
template <class T>
void
_RawAttribute(const _Enumeration &E, const UsdPrim &prim,
              const TfToken &name, T fallback, UsdTimeCode at)
{
    if (const UsdAttribute a = prim.GetAttribute(name)) {
        T value = fallback;
        a.Get(&value, at);
        E.Emit(a.GetPath(), at, /*resolved=*/false, VtValue(value));
    } else {
        E.Missing(prim.GetPath().AppendProperty(name));
    }
}

// A raw read of a binding path through RigExecRecordStageRead (the wire's
// rest curve, order, knots and bind coordinates): known absent, keyed rest,
// when nothing stands there. With \p overlay the site first takes the
// overlay's value at the path, unrecorded (the wire's bind coordinates).
template <class T>
void
_RawPath(const _Enumeration &E, const UsdStageRefPtr &stage,
         const SdfPath &path, T fallback, UsdTimeCode at,
         bool overlay = false)
{
    if (path.IsEmpty()) {
        return;
    }
    T held;
    const bool overlaid = overlay && E.resolved->Get(path, &held);
    if (const UsdAttribute a = stage->GetAttributeAtPath(path)) {
        T value = fallback;
        a.Get(&value, at);
        E.Emit(a.GetPath(), at, /*resolved=*/false, VtValue(value),
               overlaid);
    } else if (overlaid) {
        E.Emit(path, UsdTimeCode::Default(), /*resolved=*/false, VtValue(),
               /*overlaid=*/true);
    } else {
        E.Missing(path);
    }
}

// The recording _Array: a typed array at an exact path, keyed by the path
// at the read's own rest flag, absent or not. With \p overlay (every site
// but the lattice's bind-time cage) the site first takes the overlay's
// array at the path, unrecorded.
template <class T>
void
_Array(const _Enumeration &E, const UsdPrim &prim, const SdfPath &path,
       UsdTimeCode at, bool overlay = true)
{
    if (path.IsEmpty()) {
        return;
    }
    VtArray<T> held;
    const bool overlaid = overlay && E.resolved->Get(path, &held);
    if (const UsdAttribute a = prim.GetStage()->GetAttributeAtPath(path)) {
        VtArray<T> value;
        a.Get(&value, at);
        E.Emit(path, at, /*resolved=*/false, VtValue(value), overlaid);
    } else {
        E.Emit(path, at, /*resolved=*/false, VtValue(), overlaid);
    }
}

void
_Topology(const _Enumeration &E, const UsdPrim &prim,
          const RigExecRevisionBinding &binding)
{
    _Array<int>(E, prim, binding.topologyCounts, E.time);
    _Array<int>(E, prim, binding.topologyIndices, E.time);
}

// RigExecReadProjectorTarget, the whole read of a derived matrix target.
void
_Projector(const _Enumeration &E, const UsdPrim &prim, RigExecRevisionOp op,
           const RigExecRevisionBinding &binding)
{
    const UsdTimeCode at = E.time;
    if (op == RigExecRevisionOp::ShaderDials) {
        const UsdStageRefPtr stage = prim.GetStage();
        for (const SdfPath &dial : binding.shaderDials) {
            const UsdAttribute a = stage->GetAttributeAtPath(dial);
            if (!a) {
                E.Missing(dial);
                continue;
            }
            double value = 0.0;
            if (a.GetTypeName() == SdfValueTypeNames->Float) {
                float asFloat = 0.0f;
                if (!E.resolved->GetAttribute(a, at, &asFloat)) {
                    a.Get(&asFloat, at);
                }
                value = double(asFloat);
            } else if (!E.resolved->GetAttribute(a, at, &value)) {
                a.Get(&value, at);
            }
            E.Emit(dial, at, /*resolved=*/true, VtValue(value),
                   E.Holds(dial));
        }
        return;
    }
    _ResolvedInput(E, prim, _readTokens->rayOrigin, GfVec3d(0.0, 0.0, 0.0),
                   at);
    _ResolvedInput(E, prim, _readTokens->rayDirection,
                   GfVec3d(0.0, 0.0, 1.0), at);
    _ResolvedInput(E, prim, _readTokens->rayUp, GfVec3d(0.0, 1.0, 0.0), at);
    _ResolvedInput(E, prim, _readTokens->shaderOffset, GfMatrix4d(1.0), at);
    _RawAttribute(E, prim, _readTokens->projectionMode,
                  _readTokens->material, at);
    _Topology(E, prim, binding);
}

// The generation's overlay as the run's steps read through it: the
// property chains' results (a bake stands no interactive override; the pose
// weights join only when the pose is published, after every step).
RigExecResolvedInputs
_RunOverlay(const RigExecBakedProgramImpl &program)
{
    RigExecResolvedInputs overlay;
    for (const auto &[path, value] : program.propertyResults) {
        overlay.SetProperty(path, value);
    }
    return overlay;
}

// The assembly of one revision (moverGraph.cpp: RigExecAssembleMatrix-,
// RigExecAssembleSkin- and RigExecAssembleParameters), every arm, past
// every early return an input can take, over \p overlay.
void
_Revision(const RigExecBakedProgramImpl &program,
          const RigExecResolvedInputs &overlay, _Enumeration E,
          const RigExecBakedProgramImpl::GeomRevision &revision)
{
    const UsdPrim &prim = revision.moverPrim;
    // Every site keys off the mover prim, and none records without one.
    if (!prim) {
        return;
    }
    const RigExecRevisionBinding &binding = revision.binding;
    const RigExecRevisionOp op = revision.op;
    const UsdTimeCode at = E.time;
    const UsdTimeCode rest = UsdTimeCode::Default();
    E.resolved = &overlay;
    // A derived matrix target reads through the generation's overlay alone.
    if (RigExecIsDerivedMatrixOp(op)) {
        _Projector(E, prim, op, binding);
        return;
    }
    // A revision that declares read phases reads through its own overlay:
    // the generation's plus what each point binding resolves to
    // (RigExecBakedOverlayPointReads, bakedGeometry.cpp).
    RigExecResolvedInputs phased;
    if (!binding.phases.empty()) {
        phased = overlay;
        for (const RigExecBakedPointsBinding &bound :
             revision.pointBindings) {
            const GfVec3f *points = nullptr;
            size_t count = 0;
            if (RigExecBakedResolvePoints(program, bound, &points, &count)) {
                phased.SetProperty(
                    bound.input, VtValue(VtVec3fArray(points, points + count)));
            }
        }
        E.resolved = &phased;
    }
    if (op == RigExecRevisionOp::Matrix) {
        _RawAttribute(E, prim, _readTokens->weightBlend, TfToken(), at);
        _RawAttribute(E, prim, _readTokens->pointFrame, TfToken(), at);
        _Enabled(E, prim, at);
        return;
    }
    if (op == RigExecRevisionOp::Skin) {
        _Enabled(E, prim, at);
        _ResolvedOrRaw(E, prim, _readTokens->skinningMethod,
                       _readTokens->classicLinear, at);
        // The per-frame layout, read when the topology cache refuses the
        // mover.
        _Array<int>(E, prim,
                    prim.GetPath().AppendProperty(_readTokens->jointIndices),
                    at);
        _Array<float>(E, prim,
                      prim.GetPath().AppendProperty(_readTokens->jointWeights),
                      at);
        _ResolvedOrRaw(E, prim, _readTokens->elementSize, 1, at);
        return;
    }
    // Derived maintenance has no authored mover and reads no enable.
    if (op != RigExecRevisionOp::RecomputeNormals &&
        op != RigExecRevisionOp::RecomputeExtent) {
        _Enabled(E, prim, at);
    }
    switch (op) {
    case RigExecRevisionOp::BlendShape:
        // Read only in the surfaceFrame delta space.
        _Topology(E, prim, binding);
        break;
    case RigExecRevisionOp::Smooth:
        _Topology(E, prim, binding);
        break;
    case RigExecRevisionOp::SurfaceProject:
        _Array<GfVec3f>(E, prim, binding.surfacePoints, at);
        _Topology(E, prim, binding);
        break;
    case RigExecRevisionOp::DeltaMush:
        _Array<GfVec3f>(
            E, prim, prim.GetPath().AppendProperty(_readTokens->restPoints),
            rest);
        _Topology(E, prim, binding);
        _ResolvedInput(E, prim, _readTokens->iterations, 10, at);
        _ResolvedInput(E, prim, _readTokens->step, 0.5f, at);
        _ResolvedInput(E, prim, _readTokens->pinBorders, true, at);
        _ResolvedInput(E, prim, _readTokens->distanceWeight, 0.0f, at);
        _ResolvedInput(E, prim, _readTokens->displacement, 1.0f, at);
        break;
    case RigExecRevisionOp::Wrinkle:
        _Array<GfVec3f>(
            E, prim, prim.GetPath().AppendProperty(_readTokens->restPoints),
            rest);
        _Topology(E, prim, binding);
        _ResolvedInput(E, prim, _readTokens->topology, _readTokens->cloth,
                       rest);
        // Past the topology token's check: an unknown token stops there.
        _Array<int>(E, prim,
                    prim.GetPath().AppendProperty(_readTokens->pinPoints),
                    rest);
        _ResolvedInput(E, prim, _readTokens->iterations, 80, at);
        _ResolvedInput(E, prim, _readTokens->neighborDistance, 2, at);
        _ResolvedInput(E, prim, _readTokens->restLengthScale, 1.0f, at);
        _ResolvedInput(E, prim, _readTokens->stretchStiffness, 1.0f, at);
        _ResolvedInput(E, prim, _readTokens->compressionStiffness, 1.0f, at);
        _ResolvedInput(E, prim, _readTokens->bendStiffness, 0.1f, at);
        _ResolvedInput(E, prim, _readTokens->maxDisplacement, 0.2f, at);
        _ResolvedInput(E, prim, _readTokens->pinBorders, true, at);
        _ResolvedInput(E, prim, _readTokens->tangentPlaneCollisions, true,
                       at);
        _ResolvedInput(E, prim, _readTokens->tangentPlaneInset, 0.0f, at);
        _ResolvedInput(E, prim, _readTokens->wrinkleScale, 1.0f, at);
        _ResolvedInput(E, prim, _readTokens->smoothingIterations, 0, at);
        break;
    case RigExecRevisionOp::Lattice:
        // The bind-time cage reads past the overlay; the live one through
        // it. Both record.
        _Array<GfVec3f>(E, prim, binding.cagePoints, rest,
                        /*overlay=*/false);
        _Array<GfVec3f>(E, prim, binding.cagePoints, at);
        _RawAttribute(E, prim, _readTokens->divisions, GfVec3i(0, 0, 0), at);
        break;
    case RigExecRevisionOp::Ribbon:
        _Array<GfVec2f>(E, prim, binding.bindCoords, at);
        break;
    case RigExecRevisionOp::Wire: {
        const UsdStageRefPtr stage = prim.GetStage();
        _RawPath(E, stage, binding.driverCurvePoints, VtVec3fArray(), rest);
        // Transform-driven control points read the drivers' weights and
        // frames; a curve-driven wire reads the curve live instead.
        if (binding.driverTransformCount > 0) {
            _ResolvedOrRaw(E, prim, _readTokens->driverWeights,
                           VtFloatArray(), at);
            _ResolvedOrRaw(E, prim, _readTokens->driverBaseWeights,
                           VtFloatArray(), at);
            _RawAttribute(E, prim, _readTokens->pointFrame,
                          _readTokens->rest, at);
            _RawAttribute(E, prim, _readTokens->driverDeltaFrame,
                          _readTokens->local, at);
        } else {
            _Array<GfVec3f>(E, prim, binding.driverCurvePoints, at);
        }
        _RawPath(E, stage, binding.driverCurveOrder, VtIntArray(), rest);
        _RawPath(E, stage, binding.driverCurveKnots, VtDoubleArray(), rest);
        _RawAttribute(E, prim, _readTokens->dropoffDistance, 0.0f, at);
        _RawPath(E, stage, binding.bindCoords, VtArray<GfVec2f>(), at,
                 /*overlay=*/true);
        break;
    }
    case RigExecRevisionOp::RecomputeNormals:
        _Topology(E, prim, binding);
        break;
    case RigExecRevisionOp::RecomputeExtent:
        _Topology(E, prim, binding);
        _Array<float>(E, prim, binding.widths, at);
        break;
    case RigExecRevisionOp::VolumeCorrect:
    case RigExecRevisionOp::EmitGuidePoints:
    case RigExecRevisionOp::External:
        // Nothing past the enable: a plugin's own reads are its payload's.
        break;
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
    case RigExecRevisionOp::SurfaceProjector:
    case RigExecRevisionOp::ShaderDials:
        break;  // handled above
    }
}

// RigExecBakedWeightPacket's gathers, over \p overlay: recorded only when
// the resolved read answers, so a failed read leaves no key (the runtime's
// fallback to a chain base depends on that).
void
_Weight(const RigExecResolvedInputs &overlay, _Enumeration E,
        const RigExecBakedProgramImpl::WeightObject &weight)
{
    E.resolved = &overlay;
    const auto gather = [&E](const std::vector<UsdAttribute> &attributes) {
        for (const UsdAttribute &a : attributes) {
            VtVec3fArray value;
            if (E.resolved->GetAttribute(a, E.time, &value)) {
                E.Emit(a.GetPath(), E.time, /*resolved=*/true,
                       VtValue(value), E.Holds(a.GetPath()));
            }
        }
    };
    if (weight.type == _readTokens->combineWeight) {
        gather(weight.combineTargetPoints);
    } else if (weight.type == _readTokens->sphereWeight ||
               weight.type == _readTokens->planeWeight ||
               weight.type == _readTokens->curveWeight) {
        // Gathered only once the volume can build, which its inputs decide.
        gather(weight.targetPoints);
        gather(weight.samplePoints);
        if (weight.type == _readTokens->curveWeight) {
            gather(weight.curvePoints);
        }
    }
}

}  // namespace

void
RigExecBakeEnumerateRevisionReads(
    const RigExecBakedProgramImpl &program,
    const RigExecBakedProgramImpl::GeomRevision &revision, double time,
    const RigExecBakeRevisionReadSink &sink)
{
    _Enumeration E;
    E.time = UsdTimeCode(time);
    E.sink = &sink;
    _Revision(program, _RunOverlay(program), E, revision);
}

void
RigExecBakeEnumerateWeightReads(const RigExecBakedProgramImpl &program,
                                size_t object, double time,
                                const RigExecBakeRevisionReadSink &sink)
{
    if (object >= program.weightObjects.size()) {
        return;
    }
    _Enumeration E;
    E.time = UsdTimeCode(time);
    E.sink = &sink;
    _Weight(_RunOverlay(program), E, program.weightObjects[object]);
}

void
RigExecBakeEnumerateProgramReads(const RigExecBakedProgramImpl &program,
                                 double time,
                                 std::vector<RigExecBakeRevisionRead> *reads)
{
    const RigExecBakeRevisionReadSink sink =
        [reads](RigExecBakeRevisionRead &&read) {
            reads->push_back(std::move(read));
        };
    const RigExecResolvedInputs overlay = _RunOverlay(program);
    _Enumeration E;
    E.time = UsdTimeCode(time);
    E.sink = &sink;
    for (const RigExecBakedProgramImpl::GeomChain &chain : program.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            _Revision(program, overlay, E, revision);
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            _Revision(program, overlay, E, derived.revision);
        }
    }
    for (const RigExecBakedProgramImpl::WeightObject &weight :
         program.weightObjects) {
        _Weight(overlay, E, weight);
    }
}

bool
RigExecBakeCheckPathReads(
    const std::vector<RigExecWirePathRead> &recorded,
    const std::vector<RigExecBakeRevisionRead> &enumerated,
    const std::function<bool(uint32_t, std::string *)> &text,
    RigExecBakePathReadCheck *check, std::string *error,
    bool acceptUnenumerated)
{
    auto Fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    std::map<std::pair<std::string, bool>, std::vector<size_t>> keys;
    for (size_t i = 0; i < enumerated.size(); ++i) {
        keys[std::make_pair(enumerated[i].path.GetString(),
                            enumerated[i].rest)]
            .push_back(i);
    }
    if (check) {
        check->recorded = recorded.size();
        check->enumerated = keys.size();
        check->unenumerated = 0;
    }
    // An id no string table hands out, for an enumerated token whose text
    // is not the recorded one.
    constexpr uint32_t noToken = std::numeric_limits<uint32_t>::max();
    for (const RigExecWirePathRead &read : recorded) {
        std::string path;
        if (!text(read.path, &path)) {
            return Fail("a recorded path read names string " +
                        std::to_string(read.path) +
                        ", which the string table lacks");
        }
        const std::string label = "path read " + path + " (" +
                                  (read.wasDefault ? "rest" : "live") + ")";
        const auto found =
            keys.find(std::make_pair(path, read.wasDefault != 0));
        if (found == keys.end()) {
            if (acceptUnenumerated) {
                if (check) {
                    ++check->unenumerated;
                }
                continue;
            }
            return Fail(label +
                        " recorded by the assembly but not enumerated");
        }
        // Tokens compare by text: the enumerated token takes the recorded
        // id exactly when its text is the recorded text.
        std::string recordedToken;
        const bool haveToken =
            read.value.tag == RigExecWirePathValue::Tag::Token &&
            text(read.value.token, &recordedToken);
        const auto intern = [&](const std::string &token) {
            return haveToken && token == recordedToken ? read.value.token
                                                       : noToken;
        };
        bool sameValue = false;
        bool sameRoute = false;
        for (const size_t index : found->second) {
            const RigExecBakeRevisionRead &candidate = enumerated[index];
            RigExecWirePathValue encoded;
            if (!RigExecBakeEncodePathValue(candidate.value, intern,
                                            &encoded)) {
                return Fail(label +
                            " enumerated with an unencodable value of type " +
                            candidate.value.GetTypeName());
            }
            if (!RigExecBakeSamePathValue(encoded, read.value)) {
                continue;
            }
            sameValue = true;
            if (!read.forceFrame || candidate.resolved) {
                sameRoute = true;
                break;
            }
        }
        if (!sameValue) {
            return Fail(label + " enumerated with another value");
        }
        if (!sameRoute) {
            return Fail(label + " recorded as connection-following but "
                                "enumerated as a raw read");
        }
    }
    return true;
}

bool
RigExecBakeCompletePathReads(
    const std::vector<RigExecBakeRevisionRead> &enumerated,
    const std::function<uint32_t(const std::string &)> &intern,
    std::vector<RigExecWirePathRead> *record, std::string *error)
{
    std::set<std::pair<uint32_t, uint8_t>> keys;
    for (const RigExecWirePathRead &read : *record) {
        keys.emplace(read.path, read.wasDefault);
    }
    for (const RigExecBakeRevisionRead &candidate : enumerated) {
        if (candidate.overlaid) {
            continue;
        }
        RigExecWirePathRead read;
        read.path = intern(candidate.path.GetString());
        read.wasDefault = candidate.rest ? uint8_t(1) : uint8_t(0);
        if (!keys.emplace(read.path, read.wasDefault).second) {
            continue;
        }
        read.forceFrame = candidate.resolved ? uint8_t(1) : uint8_t(0);
        if (!RigExecBakeEncodePathValue(candidate.value, intern,
                                        &read.value)) {
            if (error) {
                *error = "path read " + candidate.path.GetString() +
                         " enumerated with an unencodable value of type " +
                         candidate.value.GetTypeName();
            }
            return false;
        }
        record->push_back(std::move(read));
    }
    return true;
}

}  // namespace rigExec

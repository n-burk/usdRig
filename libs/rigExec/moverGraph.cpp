//
// RigExec compiled mover graph (spec §7.2). See moverGraph.h.
//
#include "moverGraph.h"
#include "curvenetAdjuster.h"

#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/hash.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/exec/vdf/connectorSpecs.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/dataManagerVector.h"
#include "pxr/exec/vdf/executor.h"
#include "pxr/exec/vdf/inputVector.h"
#include "pxr/exec/vdf/input.h"
#include "pxr/exec/vdf/mask.h"
#include "pxr/exec/vdf/node.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/exec/vdf/readWriteIterator.h"
#include "pxr/exec/vdf/pullBasedExecutorEngine.h"
#include "pxr/exec/vdf/request.h"
#include "pxr/exec/vdf/schedule.h"
#include "pxr/exec/vdf/scheduler.h"
#include "pxr/exec/vdf/tokens.h"

#include <algorithm>
#include <cmath>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((previous, "previous"))
    ((parameters, "parameters"))
    ((status, "status"))
    ((out, "out"))
);

namespace rigExec {

namespace {

// One revision of an exact native point3f[] target.
//
// The connector shape is VDF's own idiom for this: `previous` is a READWRITE
// connector associated with `out`, so an unmodified revision passes its input
// straight through (SetOutputToReferenceInput) with no copy, and a modified one
// writes in place. That is precisely the pass-through contract disabled and
// failed movers rely on (spec §6.6).
class _RevisionNode final : public VdfNode
{
public:
    _RevisionNode(VdfNetwork *network, RigExecRevisionOp op,
                  size_t *executionCount)
        : VdfNode(
              network,
              VdfInputSpecs()
                  .ReadConnector<RigExecMoverParameters>(_tokens->parameters)
                  .ReadConnector<RigExecMoverStatus>(_tokens->status)
                  .ReadWriteConnector<GfVec3f>(_tokens->previous, _tokens->out),
              VdfOutputSpecs()
                  .Connector<GfVec3f>(_tokens->out))
        , _op(op)
        , _executionCount(executionCount)
    {
    }

    void Compute(const VdfContext &ctx) const override;
    const RigExecMoverStatus &GetStatus() const { return _resultStatus; }
    const std::vector<GfMatrix4d> &GetControlFrames() const { return _controlFrames; }

private:
    void _ComputeMatrix(const VdfContext &ctx) const;
    void _ComputeBlendShape(const VdfContext &ctx) const;
    void _ComputeRecomputed(const VdfContext &ctx,
                            const TfToken &expectedKind) const;

    mutable RigExecMoverStatus _resultStatus;
    mutable std::vector<GfMatrix4d> _controlFrames;
    RigExecRevisionOp _op;
    size_t *const _executionCount;
};

bool
_StatusAllowsApply(const VdfContext &ctx)
{
    const RigExecMoverStatus *status =
        ctx.GetInputValuePtr<RigExecMoverStatus>(_tokens->status);
    return status && status->AllowsApply();
}

// Shared scratch-collect / kernel / write-back body for the point3f[] ops
// (spec §6.5: transient scratch is released before the callback returns).
// Peer of _EvaluateScratchKernel in moverKernels.cpp, with the write-back
// changed from Allocate to in-place through the READWRITE connector.
template <typename Kernel>
void
_RunScratchKernel(const VdfContext &ctx, const TfToken &expectedKind,
                  RigExecMoverStatus *resultStatus, Kernel &&kernel)
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    auto passThrough = [&ctx, resultStatus]() {
        if (_StatusAllowsApply(ctx)) resultStatus->state = TfToken("moverFailed");
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != expectedKind) {
        passThrough();
        return;
    }

    std::vector<GfVec3f> scratch;
    {
        VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
        scratch.reserve(previous.ComputeSize());
        for (; !previous.IsAtEnd(); ++previous) {
            scratch.push_back(*previous);
        }
    }
    const std::vector<GfVec3f> preceding = scratch;
    if (!kernel(*params, &scratch)) {
        passThrough();
        return;
    }
    if (scratch.size() != preceding.size()) {
        passThrough();
        return;
    }
    std::vector<float> envelope;
    if (!params->weights.ResolveAll(scratch.size(), &envelope)) {
        passThrough();
        return;
    }
    for (size_t i = 0; i < scratch.size(); ++i) {
        scratch[i] = RigExecBlendEnvelope(
            preceding[i], scratch[i], envelope[i]);
    }

    VdfReadWriteIterator<GfVec3f> out(ctx, _tokens->previous);
    size_t i = 0;
    for (; !out.IsAtEnd() && i < scratch.size(); ++out, ++i) {
        *out = scratch[i];
    }
}

void
_RevisionNode::Compute(const VdfContext &ctx) const
{
    ++*_executionCount;
    const auto *status = ctx.GetInputValuePtr<RigExecMoverStatus>(_tokens->status);
    _resultStatus = status ? *status : RigExecMoverStatus{TfToken("moverFailed"), {}};
    _controlFrames.clear();
    switch (_op) {
    case RigExecRevisionOp::Matrix:
        _ComputeMatrix(ctx);
        return;
    case RigExecRevisionOp::BlendShape:
        _ComputeBlendShape(ctx);
        return;
    case RigExecRevisionOp::VolumeCorrect:
        _RunScratchKernel(
            ctx, TfToken("volumeCorrect"), &_resultStatus,
            [](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                RigExecApplyVolumeCorrect(pts, p.referenceVolume, p.strength);
                return true;
            });
        return;
    case RigExecRevisionOp::Smooth:
        _RunScratchKernel(
            ctx, TfToken("smooth"), &_resultStatus,
            [](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                RigExecApplyLaplacianSmooth(
                    pts, p.topologyCounts, p.topologyIndices, p.strength);
                return true;
            });
        return;
    case RigExecRevisionOp::Lattice:
        _RunScratchKernel(
            ctx, TfToken("lattice"), &_resultStatus,
            [](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                if (p.restPoints.size() != pts->size()) {
                    return false;  // cardinality mismatch fails atomically
                }
                RigExecApplyLattice(
                    pts, p.restPoints, p.auxPoints, p.auxPointsB, p.divisions);
                return true;
            });
        return;
    case RigExecRevisionOp::SurfaceProject:
        _RunScratchKernel(
            ctx, TfToken("surfaceProject"), &_resultStatus,
            [](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                RigExecApplySurfaceProject(
                    pts, p.auxPoints, p.topologyCounts, p.topologyIndices,
                    p.strength);
                return true;
            });
        return;
    case RigExecRevisionOp::EmitGuidePoints:
        _RunScratchKernel(
            ctx, TfToken("emitGuidePoints"), &_resultStatus,
            [](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                if (p.frames.GetSize() != pts->size()) {
                    return false;
                }
                for (size_t i = 0; i < pts->size(); ++i) {
                    (*pts)[i] = GfVec3f(p.frames.frames[i].Origin());
                }
                return true;
            });
        return;
    case RigExecRevisionOp::Ribbon:
        _RunScratchKernel(
            ctx, TfToken("ribbon"), &_resultStatus,
            [](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                if (p.bindCoords.size() != pts->size()) {
                    return false;
                }
                // Rest-relative rigid transport: per-sample maps from the
                // aggregate's rest frames to its posed frames (spec §7.5).
                const size_t n = p.frames.GetSize();
                if (n < 2) {
                    return false;
                }
                std::vector<GfMatrix4d> maps(n);
                for (size_t k = 0; k < n; ++k) {
                    if (!RigExecPointsToMatrix(
                            p.frames.rests[k], p.frames.frames[k].points,
                            &maps[k])) {
                        maps[k].SetIdentity();
                    }
                }
                for (size_t i = 0; i < pts->size(); ++i) {
                    const float u =
                        std::min(1.0f, std::max(0.0f, p.bindCoords[i][0]));
                    const float s = u * float(n - 1);
                    const size_t k = std::min(n - 2, size_t(s));
                    const float t = s - float(k);
                    const GfVec3d a =
                        maps[k].TransformAffine(GfVec3d((*pts)[i]));
                    const GfVec3d b =
                        maps[k + 1].TransformAffine(GfVec3d((*pts)[i]));
                    (*pts)[i] = GfVec3f(a + (b - a) * double(t));
                }
                return true;
            });
        return;
    case RigExecRevisionOp::CurvenetAdjuster:
        _RunScratchKernel(ctx, TfToken("curvenetAdjuster"), &_resultStatus,
            [this](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                const auto preceding = *pts;
                if (!RigExecApplyCurvenetAdjustments(pts, p.restPoints,
                    p.topologyIndices, p.curvenetAdjustmentBasis,
                    p.curvenetAdjustments, &_controlFrames)) return false;
                for (size_t i = 0; i < _controlFrames.size(); ++i) {
                    const int point = p.curvenetAdjustments[i].pointIndex;
                    const float weight = p.weights.Resolve(point, pts->size());
                    _controlFrames[i].SetTranslateOnly(GfVec3d(
                        preceding[point] + ((*pts)[point]-preceding[point])*weight));
                }
                return true;
            });
        return;
    case RigExecRevisionOp::Curvenet:
        _RunScratchKernel(
            ctx, TfToken("curvenet"), &_resultStatus,
            [](const RigExecMoverParameters &p, std::vector<GfVec3f> *pts) {
                if (!p.curvenetBinding) {
                    return false;
                }
                // The incoming points ARE the rest surface (§5): a curvenet
                // layered on top of skinning deforms from the skinned shape,
                // and when it is first in the chain they are the projection
                // pose and the solve takes its fast path.
                std::vector<GfVec3f> solved;
                std::string error;
                if (!RigExecEvaluateProfileMover(*p.curvenetBinding,
                                                 p.auxPoints, *pts, p.strength,
                                                 &solved, &error)) {
                    return false;
                }
                if (solved.size() != pts->size()) {
                    return false;
                }
                pts->swap(solved);
                return true;
            });
        return;
    case RigExecRevisionOp::RecomputeNormals:
        _ComputeRecomputed(ctx, TfToken("recomputeNormals"));
        return;
    case RigExecRevisionOp::RecomputeExtent:
        _ComputeRecomputed(ctx, TfToken("recomputeExtent"));
        return;
    }
    // No runtime dispatch beyond the frozen operation set: an unhandled op is
    // a build error, not a silently skipped revision.
    ctx.SetOutputToReferenceInput(_tokens->previous);
}

// normal3f[] and float3[] hosts: derived values recomputed from the final
// same-generation points rather than from the preceding revision (spec §7.6).
void
_RevisionNode::_ComputeRecomputed(const VdfContext &ctx,
                                  const TfToken &expectedKind) const
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    auto passThrough = [&ctx, this]() {
        if (_StatusAllowsApply(ctx)) _resultStatus.state = TfToken("moverFailed");
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != expectedKind) {
        passThrough();
        return;
    }

    const std::vector<GfVec3f> values =
        expectedKind == "recomputeNormals"
            ? RigExecComputeVertexNormals(params->auxPoints,
                                          params->topologyCounts,
                                          params->topologyIndices)
            : RigExecComputeExtent(params->auxPoints, params->widths);
    if (values.empty() ||
        (expectedKind == "recomputeExtent" && values.size() != 2)) {
        passThrough();
        return;
    }

    // The derived property keeps its authored cardinality: writing in place
    // through the READWRITE connector cannot resize it, so a recomputation
    // that disagrees fails the application rather than truncating.
    VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
    if (previous.ComputeSize() != values.size()) {
        passThrough();
        return;
    }
    std::vector<GfVec3f> preceding;
    preceding.reserve(values.size());
    for (; !previous.IsAtEnd(); ++previous) {
        preceding.push_back(*previous);
    }
    std::vector<float> envelope;
    if (!params->weights.ResolveAll(values.size(), &envelope)) {
        passThrough();
        return;
    }

    VdfReadWriteIterator<GfVec3f> out(ctx, _tokens->previous);
    size_t i = 0;
    for (; !out.IsAtEnd() && i < values.size(); ++out, ++i) {
        *out = RigExecBlendEnvelope(
            preceding[i], values[i], envelope[i]);
    }
}

// Blend deltas are masked per element and added to the preceding revision.
void
_RevisionNode::_ComputeBlendShape(const VdfContext &ctx) const
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
    const size_t count = previous.ComputeSize();

    auto passThrough = [&ctx, this]() {
        if (_StatusAllowsApply(ctx)) _resultStatus.state = TfToken("moverFailed");
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != "blendShape") {
        passThrough();
        return;
    }
    if (params->blendDeltas.size() != count) {
        passThrough();
        return;
    }
    // Resolve the common envelope up front so a cardinality failure passes
    // through before any element is written (the in-place write cannot be
    // rolled back).
    std::vector<float> envelope;
    if (!params->weights.ResolveAll(count, &envelope)) {
        passThrough();
        return;
    }

    std::vector<GfVec3f> transported;
    const std::vector<GfVec3f> *deltas = &params->blendDeltas;
    if (params->blendSurfaceFrame) {
        std::vector<GfVec3f> posed;
        posed.reserve(count);
        for (; !previous.IsAtEnd(); ++previous) posed.push_back(*previous);
        if (!RigExecTransportSurfaceOffsets(params->restPoints, posed,
                params->topologyCounts, params->topologyIndices,
                params->blendDeltas, &transported)) {
            passThrough();
            return;
        }
        deltas = &transported;
    }

    VdfReadWriteIterator<GfVec3f> out(ctx, _tokens->previous);
    size_t i = 0;
    for (; !out.IsAtEnd() && i < count; ++out, ++i) {
        const GfVec3f preceding = *out;
        *out = RigExecBlendEnvelope(
            preceding, preceding + (*deltas)[i], envelope[i]);
    }
}

// Ported unchanged from the generated-application kernel: the callback body was
// already written against VdfContext, so moving from a registered attribute
// expression to a node's Compute is a change of binding, not of math.
void
_RevisionNode::_ComputeMatrix(const VdfContext &ctx) const
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
    const size_t count = previous.ComputeSize();

    auto passThrough = [&ctx, this]() {
        if (_StatusAllowsApply(ctx)) _resultStatus.state = TfToken("moverFailed");
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != "matrix") {
        passThrough();
        return;
    }
    // Resolve the common envelope first so a cardinality failure passes
    // through before any output is written.
    std::vector<float> weights(count);
    if (!params->weights.ResolveAll(count, &weights)) {
        passThrough();
        return;
    }
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);

    // The revision writes in place. Unlike the generated-application kernel --
    // which allocated a fresh output buffer because its expression output had
    // no associated input -- a READWRITE connector hands the input buffer
    // straight through as the output, so constructing the iterator on
    // `previous` is what grants write access. Allocating here instead would
    // fail ("output cannot hold a boxed value") and silently pass through.
    VdfReadWriteIterator<GfVec3f> out(ctx, _tokens->previous);
    if (useSimd) {
        std::vector<GfVec3f> scratch;
        scratch.reserve(count);
        for (; !previous.IsAtEnd(); ++previous) {
            scratch.push_back(*previous);
        }
        RigExecApplyWeightedMatrixSimd(
            scratch.data(), scratch.data(), weights.data(), count,
            params->transform);
        size_t i = 0;
        for (; !out.IsAtEnd(); ++out, ++i) {
            *out = scratch[i];
        }
    } else {
        // Element i is written only after it is read, so in-place is safe.
        size_t i = 0;
        for (; !out.IsAtEnd(); ++out, ++i) {
            *out = GfVec3f(RigExecApplyWeightedMatrix(
                GfVec3d(*out), params->transform, weights[i]));
        }
    }
}

}  // namespace

bool
RigExecRevisionBinding::operator==(const RigExecRevisionBinding &o) const
{
    return moverPath == o.moverPath && target == o.target &&
           transform == o.transform && weightObject == o.weightObject &&
           base == o.base && topologyCounts == o.topologyCounts &&
           topologyIndices == o.topologyIndices &&
           cagePoints == o.cagePoints && surfacePoints == o.surfacePoints &&
           bindCoords == o.bindCoords && driverFrames == o.driverFrames &&
           widths == o.widths && curvenet == o.curvenet &&
           curvenetPoints == o.curvenetPoints &&
           blendInputs == o.blendInputs && blendSamples == o.blendSamples &&
           phases == o.phases && transformPhase == o.transformPhase;
}

std::optional<RigExecRevisionOp>
RigExecRevisionOpForSchema(const TfToken &schemaType, const TfToken &curveMode)
{
    if (schemaType == "RigExecMatrixMover") {
        return RigExecRevisionOp::Matrix;
    }
    if (schemaType == "RigExecBlendShapeMover") {
        return RigExecRevisionOp::BlendShape;
    }
    if (schemaType == "RigExecVolumeCorrectMover") {
        return RigExecRevisionOp::VolumeCorrect;
    }
    if (schemaType == "RigExecSmoothMover") {
        return RigExecRevisionOp::Smooth;
    }
    if (schemaType == "RigExecLatticeMover") {
        return RigExecRevisionOp::Lattice;
    }
    if (schemaType == "RigExecSurfaceMover") {
        return RigExecRevisionOp::SurfaceProject;
    }
    if (schemaType == "RigExecCurvenetMover") {
        return RigExecRevisionOp::Curvenet;
    }
    if (schemaType == "RigExecCurvenetAdjusterMover") {
        return RigExecRevisionOp::CurvenetAdjuster;
    }
    if (schemaType == "RigExecCurveMover") {
        // The curve mover's frozen signature branches on its authored mode.
        return curveMode == "emitGuidePoints"
            ? RigExecRevisionOp::EmitGuidePoints
            : RigExecRevisionOp::Ribbon;
    }
    return std::nullopt;
}

std::shared_ptr<const RigExecProfileMoverBinding>
RigExecCurvenetBindCache::Resolve(
    const SdfPath &mover, const SdfPath &target, size_t digest,
    const std::function<bool(RigExecProfileMoverBinding *, std::string *)>
        &build,
    std::string *error)
{
    _Entry &entry = _entries[{mover, target}];
    if (entry.digest == digest && (entry.binding || !entry.error.empty())) {
        if (!entry.binding && error) {
            *error = entry.error;
        }
        return entry.binding;
    }

    entry.digest = digest;
    entry.binding.reset();
    entry.error.clear();
    auto built = std::make_shared<RigExecProfileMoverBinding>();
    std::string reason;
    if (!build(built.get(), &reason)) {
        // Remembered, so a rig with an unbindable curvenet reports once
        // instead of re-cutting the mesh on every frame.
        entry.error = reason.empty() ? "curvenet bind failed" : reason;
        if (error) {
            *error = entry.error;
        }
        return nullptr;
    }
    // Reported once per (re)bind, not per frame: the cache exists precisely
    // so this happens on a layout change and nowhere else.
    const RigExecCutMeshReport &cut = built->report;
    char summary[320];
    std::snprintf(
        summary, sizeof(summary),
        "curvenet bind %s -> %s: %d cut faces, %d samples, %d cracks, "
        "%d traced, %d unknowns, %zu factor nonzeros",
        mover.GetString().c_str(), target.GetString().c_str(),
        cut.cutFaceCount, cut.sampleCount, cut.crackCount, cut.tracedSegments,
        built->cutMesh.unknownCount,
        built->solver ? built->solver->GetFactorNonzeros() : size_t(0));
    _pending.push_back(summary);
    for (const std::string &warning : cut.warnings) {
        _pending.push_back("curvenet bind " + mover.GetString() + ": " +
                           warning);
    }

    entry.binding = std::move(built);
    return entry.binding;
}

std::vector<std::string>
RigExecCurvenetBindCache::TakeDiagnostics()
{
    std::vector<std::string> out;
    out.swap(_pending);
    return out;
}


std::string
RigExecReadPhase::GetAsString() const
{
    switch (kind) {
    case RigExecReadPhaseKind::Base:      return "base";
    case RigExecReadPhaseKind::Preceding: return "preceding";
    case RigExecReadPhaseKind::Final:     return "final";
    case RigExecReadPhaseKind::AtPrim:    return prim.GetString();
    }
    return "base";
}

bool
RigExecParseReadPhase(
    const std::string &authored, RigExecReadPhase *phase, std::string *error)
{
    if (!phase) {
        return false;
    }
    if (authored.empty() || authored == "base") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Base, SdfPath()};
        return true;
    }
    if (authored == "preceding") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Preceding, SdfPath()};
        return true;
    }
    if (authored == "final") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Final, SdfPath()};
        return true;
    }
    // Anything else must be an absolute prim path. A relative path would have
    // to be resolved against something, and there are two equally plausible
    // somethings here (the mover, the target), so it is rejected rather than
    // guessed.
    if (!SdfPath::IsValidPathString(authored)) {
        if (error) {
            *error = "'" + authored +
                     "' is not base, preceding, final, or a valid prim path";
        }
        return false;
    }
    const SdfPath path(authored);
    if (!path.IsAbsolutePath() || !path.IsPrimPath()) {
        if (error) {
            *error = "'" + authored +
                     "' must be an ABSOLUTE prim path (or base, preceding, "
                     "final)";
        }
        return false;
    }
    *phase = RigExecReadPhase{RigExecReadPhaseKind::AtPrim, path};
    return true;
}

bool
RigExecResolveReadPhase(
    const UsdObject &property,
    const char *legacyAttribute,
    RigExecReadPhase *phase,
    std::string *error)
{
    if (!phase) {
        return false;
    }
    *phase = RigExecReadPhase();
    if (!property.IsValid()) {
        return true;
    }

    // Metadata on the property itself wins: it is the most specific place the
    // phase can be said, and the only one that works for an input with no
    // schema attribute of its own.
    std::string authored;
    if (property.GetMetadata(TfToken(RigExecReadPhaseMetadataName),
                             &authored) &&
        !authored.empty()) {
        std::string why;
        if (!RigExecParseReadPhase(authored, phase, &why)) {
            if (error) {
                *error = property.GetPath().GetString() + ": " +
                         RigExecReadPhaseMetadataName + " " + why;
            }
            return false;
        }
        return true;
    }

    // Then the role-named schema attribute, so every asset authored before the
    // metadata existed keeps meaning exactly what it meant.
    if (legacyAttribute) {
        const UsdPrim owner = property.GetPrim();
        if (const UsdAttribute a =
                owner.GetAttribute(TfToken(legacyAttribute))) {
            TfToken value;
            if (a.Get(&value) && !value.IsEmpty()) {
                std::string why;
                if (!RigExecParseReadPhase(value.GetString(), phase, &why)) {
                    if (error) {
                        *error = owner.GetPath().GetString() + ": " +
                                 legacyAttribute + " " + why;
                    }
                    return false;
                }
                return true;
            }
        }
    }
    return true;
}

void
RigExecChainSnapshots::Record(
    const SdfPath &target, const SdfPath &afterMover, const VtValue &value)
{
    _chains[target].revisions.emplace_back(afterMover, value);
}

void
RigExecChainSnapshots::RecordFinal(const SdfPath &target, const VtValue &value)
{
    _Chain &chain = _chains[target];
    chain.final = value;
    chain.hasFinal = true;
}

const VtValue *
RigExecChainSnapshots::Lookup(
    const SdfPath &target, const RigExecReadPhase &phase,
    const SdfPath &readerMover) const
{
    const auto it = _chains.find(target);
    if (it == _chains.end()) {
        return nullptr;
    }
    const _Chain &chain = it->second;

    switch (phase.kind) {
    case RigExecReadPhaseKind::Base:
        // The stage answers this one; nothing is recorded for it.
        return nullptr;

    case RigExecReadPhaseKind::Final:
        return chain.hasFinal ? &chain.final : nullptr;

    case RigExecReadPhaseKind::Preceding: {
        // The value going INTO the reader. Absent when the reader is the
        // chain's first revision, which is the authored base -- so returning
        // null correctly sends the caller to the stage.
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            if (chain.revisions[i].first == readerMover) {
                return i == 0 ? nullptr : &chain.revisions[i - 1].second;
            }
        }
        // The reader does not write this chain at all, so "preceding" has no
        // position to be relative to.
        return nullptr;
    }

    case RigExecReadPhaseKind::AtPrim: {
        // The LAST revision at or beneath the named prim. For a mover that is
        // the mover itself; for a Scope it is whatever ran last inside it,
        // which is what post-order makes that Scope mean.
        const VtValue *found = nullptr;
        for (const auto &[mover, value] : chain.revisions) {
            if (mover == phase.prim || mover.HasPrefix(phase.prim)) {
                found = &value;
            }
        }
        return found;
    }
    }
    return nullptr;
}

namespace {

SdfPathVector
_Targets(const UsdPrim &prim, const char *rel)
{
    SdfPathVector targets;
    if (const UsdRelationship r = prim.GetRelationship(TfToken(rel))) {
        r.GetTargets(&targets);
    }
    return targets;
}

TfToken
_Token(const UsdPrim &prim, const char *attr, const char *fallback)
{
    TfToken value(fallback);
    if (const UsdAttribute a = prim.GetAttribute(TfToken(attr))) {
        a.Get(&value);
    }
    return value;
}

// A PointBased prim binds to its .points property by the standard rule; an
// exact property path is already canonical.
SdfPath
_PointsOf(const SdfPath &path)
{
    return path.IsPrimPath() ? path.AppendProperty(TfToken("points")) : path;
}

// Every scalar mover input consults the generation's resolved property set
// before the authored stage. This is what lets a property-domain mover drive
// another mover's common envelope without a second evaluation model.
float
_Float(const UsdPrim &prim, const char *attr, float fallback,
       UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    float value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(TfToken(attr))) {
        if (resolved && resolved->GetAttribute(a, time, &value)) {
            return value;
        }
        a.Get(&value, time);
    }
    return value;
}

}  // namespace

RigExecRevisionBinding
RigExecResolveRevisionBinding(
    const UsdPrim &moverPrim,
    const SdfPath &target,
    const std::map<SdfPath, SdfPath> &frameChainHeads)
{
    RigExecRevisionBinding binding;
    if (!moverPrim) {
        return binding;
    }
    binding.moverPath = moverPrim.GetPath();
    binding.target = target;

    const TfToken schemaType = moverPrim.GetTypeName();
    const SdfPath ownerPath = target.GetPrimPath();

    // A phase declared on the relationship that NAMES an input governs that
    // input. Recorded against the exact property path the phase applies to,
    // so the assembler's read of that path is what consults it.
    auto phaseFor = [&moverPrim](const char *rel, const char *legacyAttr) {
        RigExecReadPhase phase;
        if (const UsdRelationship r = moverPrim.GetRelationship(TfToken(rel))) {
            RigExecResolveReadPhase(r, legacyAttr, &phase, nullptr);
        } else if (legacyAttr) {
            // No relationship to hang metadata on, but the legacy attribute
            // may still be authored.
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(TfToken(legacyAttr))) {
                RigExecResolveReadPhase(a, legacyAttr, &phase, nullptr);
            }
        }
        return phase;
    };

    if (schemaType == "RigExecMatrixMover") {
        // "final" binds the provider's frame-chain head instead of the
        // provider itself; every other phase binds the provider (spec §12.1).
        const SdfPathVector transforms = _Targets(moverPrim, "rigExec:transform");
        SdfPath provider = transforms.empty() ? SdfPath() : transforms[0];
        binding.transformPhase =
            phaseFor("rigExec:transform", "rigExec:transformReadPhase");
        if (binding.transformPhase.kind == RigExecReadPhaseKind::Final) {
            const auto it = frameChainHeads.find(provider);
            if (it != frameChainHeads.end()) {
                provider = it->second;
            }
        }
        binding.transform = provider;
    } else if (schemaType == "RigExecBlendShapeMover") {
        if (moverPrim.GetStage()->GetPrimAtPath(ownerPath).IsA<UsdGeomMesh>()) {
            binding.topologyCounts = ownerPath.AppendProperty(TfToken("faceVertexCounts"));
            binding.topologyIndices = ownerPath.AppendProperty(TfToken("faceVertexIndices"));
        }
        binding.blendInputs = _Targets(moverPrim, "rigExec:blendInputs");
        std::sort(binding.blendInputs.begin(), binding.blendInputs.end());
        for (const SdfPath &input : binding.blendInputs) {
            const UsdPrim channel = moverPrim.GetStage()->GetPrimAtPath(input);
            for (const SdfPath &samplePath : _Targets(channel, "rigExec:samples")) {
                const UsdPrim sample = moverPrim.GetStage()->GetPrimAtPath(samplePath);
                const SdfPathVector points = _Targets(sample, "rigExec:targetPoints");
                if (points.size() != 1) continue; // compile validates cardinality
                RigExecReadPhase phase;
                std::string error;
                RigExecResolveReadPhase(
                    sample.GetRelationship(TfToken("rigExec:targetPoints")),
                    "rigExec:pointsReadPhase", &phase, &error);
                binding.blendSamples[input].push_back({samplePath, _PointsOf(points[0]), phase});
            }
        }
        binding.base = target;
    } else if (schemaType == "RigExecVolumeCorrectMover") {
        binding.base = target;
    } else if (schemaType == "RigExecSmoothMover") {
        binding.topologyCounts =
            ownerPath.AppendProperty(TfToken("faceVertexCounts"));
        binding.topologyIndices =
            ownerPath.AppendProperty(TfToken("faceVertexIndices"));
    } else if (schemaType == "RigExecLatticeMover") {
        binding.base = target;
        const SdfPathVector cages = _Targets(moverPrim, "rigExec:cage");
        if (!cages.empty()) {
            binding.cagePoints = _PointsOf(cages[0]);
            const RigExecReadPhase phase =
                phaseFor("rigExec:cage", "rigExec:cageReadPhase");
            if (!phase.IsBase()) {
                binding.phases[binding.cagePoints] = phase;
            }
        }
    } else if (schemaType == "RigExecSurfaceMover") {
        const SdfPathVector surfaces = _Targets(moverPrim, "rigExec:surface");
        if (!surfaces.empty()) {
            const SdfPath surfacePrim = surfaces[0].GetPrimPath();
            binding.surfacePoints =
                surfacePrim.AppendProperty(TfToken("points"));
            const RigExecReadPhase phase =
                phaseFor("rigExec:surface", "rigExec:surfaceReadPhase");
            if (!phase.IsBase()) {
                binding.phases[binding.surfacePoints] = phase;
            }
            binding.topologyCounts =
                surfacePrim.AppendProperty(TfToken("faceVertexCounts"));
            binding.topologyIndices =
                surfacePrim.AppendProperty(TfToken("faceVertexIndices"));
        }
    } else if (schemaType == "RigExecCurvenetAdjusterMover") {
        binding.base = target;
        binding.curvenet = target.GetPrimPath();
    } else if (schemaType == "RigExecCurvenetMover") {
        // The Profile Mover reads the target's own topology to cut it, and
        // its authored base points are the projection pose the cut is
        // computed against (§4.1).
        binding.base = target;
        binding.topologyCounts =
            ownerPath.AppendProperty(TfToken("faceVertexCounts"));
        binding.topologyIndices =
            ownerPath.AppendProperty(TfToken("faceVertexIndices"));
        const SdfPathVector nets = _Targets(moverPrim, "rigExec:curvenet");
        if (!nets.empty()) {
            binding.curvenet = nets[0].GetPrimPath();
            binding.curvenetPoints =
                binding.curvenet.AppendProperty(TfToken("points"));
            const RigExecReadPhase phase =
                phaseFor("rigExec:curvenet", nullptr);
            if (!phase.IsBase()) {
                binding.phases[binding.curvenetPoints] = phase;
            }
        }
    } else if (schemaType == "RigExecCurveMover") {
        const SdfPathVector binds = _Targets(moverPrim, "rigExec:bindCoordinates");
        if (!binds.empty()) {
            binding.bindCoords = binds[0];
        }
        const SdfPathVector frames = _Targets(moverPrim, "rigExec:driverFrames");
        if (!frames.empty()) {
            binding.driverFrames = frames[0];
        }
        if (!binding.bindCoords.IsEmpty()) {
            const RigExecReadPhase phase =
                phaseFor("rigExec:bindCoordinates", nullptr);
            if (!phase.IsBase()) {
                binding.phases[binding.bindCoords] = phase;
            }
        }
    }

    // Every point-chain mover may narrow its application with a weight object.
    const SdfPathVector weights = _Targets(moverPrim, "rigExec:weightObject");
    if (!weights.empty()) {
        binding.weightObject = weights[0];
    }

    return binding;
}

RigExecMoverParameters
RigExecAssembleMatrixParameters(
    const UsdPrim &moverPrim,
    const GfMatrix4d *transform,
    const RigExecWeightPacket *weights,
    UsdTimeCode time,
    const RigExecResolvedInputs *resolved)
{
    RigExecMoverParameters params;
    params.kind = TfToken("matrix");

    bool enabled = true;
    if (moverPrim) {
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("inputs:enabled"))) {
            if (!resolved ||
                !resolved->GetAttribute(a, time, &enabled)) {
                a.Get(&enabled, time);
            }
        }
    }
    params.enabled = enabled;
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }

    if (!transform) {
        return params;  // MoverFailed
    }
    params.weights = weights
        ? *weights
        : RigExecWeightPacket::Constant(_Float(
              moverPrim, "inputs:defaultWeight", 1.0f, time, resolved));
    if (!params.weights.valid) {
        return params;  // invalid common envelope => MoverFailed
    }
    // The matrix must be finite and affine (spec §7.4).
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite((*transform)[i][j])) {
                return params;
            }
        }
    }
    if ((*transform)[0][3] != 0 || (*transform)[1][3] != 0 ||
        (*transform)[2][3] != 0 || (*transform)[3][3] != 1) {
        return params;
    }
    params.transform = *transform;
    params.valid = true;
    return params;
}

RigExecMoverStatus
RigExecStatusForParameters(
    const RigExecMoverParameters &parameters, const SdfPath &moverPath)
{
    RigExecMoverStatus status;
    if (!parameters.enabled) {
        status.state = TfToken("disabled");
    } else if (parameters.valid) {
        status.state = TfToken("ok");
    } else {
        status.state = TfToken("moverFailed");
        // First bad canonical public address (spec §6.6): v0.1 reports the
        // failed mover's own path; per-input attribution is future work.
        status.firstBadAddress = moverPath.GetString();
    }
    return status;
}

namespace {

// Reads a typed array from an exact property path on the mover's stage.
//
// \p resolved is null for BIND-TIME reads and only for those. A rest cage, a
// rest curvenet, a bind-time topology: those are the authored neutral pose the
// deformation is measured against, so a read phase has nothing to say about
// them -- and serving one the same phased value as the live read makes the two
// operands equal and the whole deformation an identity.
template <typename T>
std::vector<T>
_Array(const UsdPrim &moverPrim, const SdfPath &path, UsdTimeCode time,
       const RigExecResolvedInputs *resolved = nullptr)
{
    std::vector<T> out;
    if (path.IsEmpty() || !moverPrim) {
        return out;
    }
    VtArray<T> value;
    if (resolved && resolved->Get(path, &value)) {
        out.assign(value.begin(), value.end());
        return out;
    }
    if (const UsdAttribute a =
            moverPrim.GetStage()->GetAttributeAtPath(path)) {
        // At the EVALUATED time, not Default: the kernels read these same
        // inputs through exec computeValue at the current time, so an
        // animated cage/surface/topology would otherwise silently diverge.
        a.Get(&value, time);
    }
    out.assign(value.begin(), value.end());
    return out;
}

bool
_Enabled(const UsdPrim &prim, UsdTimeCode time,
         const RigExecResolvedInputs *resolved)
{
    bool enabled = true;
    if (prim) {
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("inputs:enabled"))) {
            if (resolved && resolved->GetAttribute(a, time, &enabled)) {
                return enabled;
            }
            a.Get(&enabled, time);
        }
    }
    return enabled;
}

}  // namespace

bool
RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const std::vector<GfVec3f> &base,
    std::vector<GfVec3f> *deltas)
{
    if (!deltas || base.empty()) {
        return false;
    }
    deltas->assign(base.size(), GfVec3f(0));

    for (const RigExecBlendChannel &channel : channels) {
        if (channel.samples.empty() || !std::isfinite(channel.weight)) {
            return false;  // structural error: fails atomically
        }
        for (size_t k = 0; k < channel.samples.size(); ++k) {
            const RigExecBlendSampleData &sample = channel.samples[k];
            if (!std::isfinite(sample.activation) ||
                sample.activation <= 0 ||
                sample.points.size() != base.size() ||
                (k > 0 && sample.activation ==
                              channel.samples[k - 1].activation)) {
                return false;
            }
        }
        // The channel weight is clamped into the authored activation range;
        // an implicit zero-delta sample sits at activation 0.
        const float w = std::min(std::max(channel.weight, 0.0f),
                                 channel.samples.back().activation);
        if (w == 0.0f) {
            continue;
        }
        size_t hi = 0;
        while (hi < channel.samples.size() &&
               channel.samples[hi].activation < w) {
            ++hi;
        }
        if (hi >= channel.samples.size()) {
            hi = channel.samples.size() - 1;
        }
        const float aHi = channel.samples[hi].activation;
        const float aLo = hi > 0 ? channel.samples[hi - 1].activation : 0.0f;
        const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
        const std::vector<GfVec3f> *lo =
            hi > 0 ? &channel.samples[hi - 1].points : nullptr;
        const std::vector<GfVec3f> &hiPts = channel.samples[hi].points;
        for (size_t i = 0; i < base.size(); ++i) {
            const GfVec3f dHi = hiPts[i] - base[i];
            const GfVec3f dLo = lo ? (*lo)[i] - base[i] : GfVec3f(0);
            (*deltas)[i] += dLo + (dHi - dLo) * t;
        }
    }
    return true;
}

RigExecMoverParameters
RigExecAssembleParameters(
    const UsdPrim &moverPrim,
    RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecProviderValues &values,
    UsdTimeCode time)
{
    if (op == RigExecRevisionOp::Matrix) {
        return RigExecAssembleMatrixParameters(
            moverPrim, values.transform, values.weights, time,
            values.resolved);
    }
    if (op == RigExecRevisionOp::CurvenetAdjuster) {
        return RigExecAssembleCurvenetAdjusterParameters(
            moverPrim, binding.target, values.weights, time, values.resolved);
    }

    RigExecMoverParameters params;
    const bool synthesizedDerived =
        op == RigExecRevisionOp::RecomputeNormals ||
        op == RigExecRevisionOp::RecomputeExtent;
    // Derived maintenance has no authored mover. The owning gprim is only a
    // convenient topology/property source and must not accidentally acquire
    // mover semantics from custom attributes with familiar names.
    params.enabled = synthesizedDerived
        ? true
        : _Enabled(moverPrim, time, values.resolved);

    switch (op) {
    case RigExecRevisionOp::BlendShape:
        params.kind = TfToken("blendShape");
        break;
    case RigExecRevisionOp::VolumeCorrect:
        params.kind = TfToken("volumeCorrect");
        break;
    case RigExecRevisionOp::Smooth:
        params.kind = TfToken("smooth");
        break;
    case RigExecRevisionOp::Lattice:
        params.kind = TfToken("lattice");
        break;
    case RigExecRevisionOp::SurfaceProject:
        params.kind = TfToken("surfaceProject");
        break;
    case RigExecRevisionOp::Ribbon:
        params.kind = TfToken("ribbon");
        break;
    case RigExecRevisionOp::EmitGuidePoints:
        params.kind = TfToken("emitGuidePoints");
        break;
    case RigExecRevisionOp::Curvenet:
        params.kind = TfToken("curvenet");
        break;
    case RigExecRevisionOp::RecomputeNormals:
        params.kind = TfToken("recomputeNormals");
        break;
    case RigExecRevisionOp::RecomputeExtent:
        params.kind = TfToken("recomputeExtent");
        break;
    case RigExecRevisionOp::Matrix:
        break;  // handled above
    case RigExecRevisionOp::CurvenetAdjuster:
        break;  // handled above
    }

    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }

    // One envelope contract for every operation. A bound object is the total
    // field and supersedes the scalar fallback; without one, synthesize the
    // normalized constant packet supplied by RigExecMoverAPI.
    params.weights = synthesizedDerived
        ? RigExecWeightPacket::Constant(1.0f)
        : (values.weights
               ? *values.weights
               : RigExecWeightPacket::Constant(_Float(
                     moverPrim, "inputs:defaultWeight", 1.0f, time,
                     values.resolved)));
    if (!params.weights.valid) {
        return params;  // MoverFailed, preserving the preceding revision
    }

    switch (op) {
    case RigExecRevisionOp::BlendShape: {
        params.blendDeltas = values.blendDeltas;
        const TfToken space = _Token(moverPrim, "rigExec:deltaSpace", "target");
        if (space != "target" && space != "surfaceFrame") break;
        params.blendSurfaceFrame = space == "surfaceFrame";
        if (params.blendSurfaceFrame) {
            params.restPoints = values.basePoints;
            params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
            params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
            if (params.topologyCounts.empty()) break;
        }
        params.valid = !params.blendDeltas.empty();
        break;
    }

    case RigExecRevisionOp::VolumeCorrect:
        params.strength = 1.0f;
        // The correction reference is the bound volume of the authored base.
        if (!values.basePoints.empty()) {
            params.referenceVolume = RigExecBoundVolume(
                values.basePoints.data(), values.basePoints.size());
            params.valid = true;
        }
        break;

    case RigExecRevisionOp::Smooth:
        params.strength = 1.0f;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
        params.valid = !params.topologyCounts.empty();
        break;

    case RigExecRevisionOp::Curvenet: {
        params.strength = 1.0f;
        const UsdPrim netPrim =
            binding.curvenet.IsEmpty()
                ? UsdPrim()
                : moverPrim.GetStage()->GetPrimAtPath(binding.curvenet);
        if (!netPrim) {
            break;  // no curvenet: MoverFailed pass-through
        }

        // The projection pose is the curvenet and the surface as AUTHORED --
        // Default time on both. Everything the cut depends on is read here,
        // and its digest is what decides whether the cache still applies.
        const std::vector<GfVec3f> restNet = _Array<GfVec3f>(moverPrim, binding.curvenetPoints, UsdTimeCode::Default(), /*resolved=*/nullptr);
        std::vector<int> splineIndices;
        if (const UsdAttribute a =
                netPrim.GetAttribute(TfToken("rigExec:splineIndices"))) {
            VtIntArray value;
            a.Get(&value, UsdTimeCode::Default());
            splineIndices.assign(value.begin(), value.end());
        }
        int samplesPerSpline = 5;
        if (const UsdAttribute a =
                netPrim.GetAttribute(TfToken("rigExec:samplesPerSpline"))) {
            a.Get(&samplesPerSpline);
        }
        const TfToken basisToken = _Token(netPrim, "rigExec:basis", "bezier");
        params.topologyCounts =
            _Array<int>(moverPrim, binding.topologyCounts, UsdTimeCode::Default(), /*resolved=*/nullptr);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, UsdTimeCode::Default(), /*resolved=*/nullptr);
        // The projection surface is the target's points at DEFAULT, not at
        // the evaluated time. It is a fixed neutral pose by definition (§4.1
        // cuts against it once), and reading the animated value instead would
        // put a per-frame quantity in the bind digest -- re-cutting the mesh
        // and re-factorizing its Laplacian on every frame, while also making
        // the cut mean something different at each one.
        params.restPoints = _Array<GfVec3f>(moverPrim, binding.base, UsdTimeCode::Default(), /*resolved=*/nullptr);
        if (restNet.empty() || splineIndices.empty() ||
            params.topologyCounts.empty() || params.restPoints.empty()) {
            break;
        }

        // The posed net: the evaluator hands over the result of the
        // curvenet's own mover chain when it has one, so knots articulated by
        // ordinary movers arrive already posed. Otherwise the authored value
        // at this time, which is what a keyframed or sculpted net gives.
        params.auxPoints = values.curvenetPoints.empty()
                               ? _Array<GfVec3f>(moverPrim,
                                                 binding.curvenetPoints, time)
                               : values.curvenetPoints;
        if (params.auxPoints.size() != restNet.size()) {
            break;  // the pool changed shape under the bind
        }

        size_t digest = TfHash()(basisToken);
        digest = TfHash::Combine(digest, samplesPerSpline);
        for (int index : splineIndices) {
            digest = TfHash::Combine(digest, index);
        }
        for (const GfVec3f &p : restNet) {
            digest = TfHash::Combine(digest, p[0], p[1], p[2]);
        }
        for (const GfVec3f &p : params.restPoints) {
            digest = TfHash::Combine(digest, p[0], p[1], p[2]);
        }
        for (int c : params.topologyCounts) {
            digest = TfHash::Combine(digest, c);
        }
        for (int i : params.topologyIndices) {
            digest = TfHash::Combine(digest, i);
        }

        auto build = [&](RigExecProfileMoverBinding *out, std::string *why) {
            RigExecCurvenetTopology topology;
            const RigExecCurvenetBasis basis =
                (basisToken == "catmullRom")
                    ? RigExecCurvenetBasis::CatmullRom
                    : RigExecCurvenetBasis::Bezier;
            if (!RigExecBuildCurvenetTopology(splineIndices, restNet.size(),
                                              basis, restNet, nullptr,
                                              &topology, why)) {
                return false;
            }
            return RigExecBindProfileMover(
                topology, restNet, params.restPoints, params.topologyCounts,
                params.topologyIndices, samplesPerSpline, out, why);
        };

        std::string reason;
        if (values.curvenetCache) {
            params.curvenetBinding = values.curvenetCache->Resolve(
                binding.moverPath, binding.target, digest, build, &reason);
        } else {
            auto fresh = std::make_shared<RigExecProfileMoverBinding>();
            if (build(fresh.get(), &reason)) {
                params.curvenetBinding = fresh;
            }
        }
        params.valid = params.curvenetBinding != nullptr;
        break;
    }

    case RigExecRevisionOp::Lattice: {
        params.restPoints = values.basePoints;
        // Operand order matters and is not symmetric: the shared applier is
        // RigExecApplyLattice(points, restPoints, restCage, posedCage, divs),
        // so auxPoints is the BIND-TIME cage and auxPointsB the live one --
        // matching _BuildLatticeMoverParameters. Reversing them is invisible
        // at rest (the two cages are equal) and inverts the deformation as
        // soon as the cage moves.
        // The rest cage is the cage at Default time, read directly. It used to
        // be a compiler-authored capture in rigExec:restCagePoints, but that
        // capture was itself only `a.Get(&v, UsdTimeCode::Default())` on this
        // same attribute -- so reading it here is identical and needs nothing
        // authored. The live cage is the same attribute at the evaluated time.
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.cagePoints, UsdTimeCode::Default(), /*resolved=*/nullptr);
        params.auxPointsB = _Array<GfVec3f>(moverPrim, binding.cagePoints, time, values.resolved);
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("rigExec:divisions"))) {
            a.Get(&params.divisions, time);
        }
        // Same cardinality contract as the kernel: a cage that does not match
        // the declared lattice resolution fails atomically instead of
        // indexing garbage.
        const size_t cageCount = size_t(params.divisions[0]) *
                                 size_t(params.divisions[1]) *
                                 size_t(params.divisions[2]);
        params.valid = params.divisions[0] >= 2 && params.divisions[1] >= 2 &&
                       params.divisions[2] >= 2 &&
                       params.auxPoints.size() == cageCount &&
                       params.auxPointsB.size() == cageCount &&
                       !params.restPoints.empty();
        break;
    }

    case RigExecRevisionOp::SurfaceProject:
        // Fixed at full, matching _BuildSurfaceMoverParameters: v0.1
        // attach/project maps fully. RigExecSurfaceMover declares no
        // inputs:strength, so reading one here silently applied a 0.5
        // default and projected half way.
        params.strength = 1.0f;
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.surfacePoints, time, values.resolved);
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
        params.valid =
            !params.auxPoints.empty() && !params.topologyCounts.empty();
        break;

    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::EmitGuidePoints: {
        const RigExecPointFrameArray *frames = values.driverFrames;
        if (!frames || frames->IsEmpty() ||
            frames->rests.size() != frames->GetSize()) {
            break;  // MoverFailed
        }
        params.frames = *frames;
        if (op == RigExecRevisionOp::Ribbon) {
            params.bindCoords =
                _Array<GfVec2f>(moverPrim, binding.bindCoords, time, values.resolved);
            params.valid = !params.bindCoords.empty();
        } else {
            params.valid = true;
        }
        break;
    }

    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        // Derived maintenance reads the final same-generation points, which
        // the caller supplies as the base value for this revision.
        params.auxPoints = values.basePoints;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
        // Authored widths widen the extent bounds; omitting them silently
        // under-reports the bound of a curves/points gprim.
        if (op == RigExecRevisionOp::RecomputeExtent) {
            params.widths = _Array<float>(moverPrim, binding.widths, time, values.resolved);
        }
        // Vertex normals need the adjacency; without it the kernel reports
        // MoverFailed rather than emitting garbage normals. Extent needs only
        // the points (matches _BuildRecomputeNormals/ExtentParameters).
        params.valid =
            !params.auxPoints.empty() &&
            (op == RigExecRevisionOp::RecomputeExtent ||
             !params.topologyCounts.empty());
        break;

    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::CurvenetAdjuster:
        break;
    }

    return params;
}

struct RigExecMoverGraph::_Runtime {
    struct _RevisionSources {
        VdfInputVector<RigExecMoverParameters> *parameters;
        VdfInputVector<RigExecMoverStatus> *status;
        VdfMaskedOutput previous;
    };

    VdfExecutor<VdfPullBasedExecutorEngine,
        VdfDataManagerVector<VdfDataManagerDeallocationMode::Immediate>> executor;
    std::unique_ptr<VdfSchedule> schedule;
    std::map<VdfMaskedOutput, VdfInputVector<GfVec3f> *> sources;
    std::map<VdfMaskedOutput, _RevisionSources> revisions;
    VdfMaskedOutputVector outputs;
    VdfMaskedOutputVector dirty;
    size_t executionCount = 0;
    size_t scheduleBuildCount = 0;

    void TopologyChanged() {
        schedule.reset();
        executor.InvalidateTopologicalState();
    }
};

RigExecMoverGraph::RigExecMoverGraph() : _runtime(new _Runtime) {}
RigExecMoverGraph::~RigExecMoverGraph() = default;

VdfMaskedOutput
RigExecMoverGraph::AddPointSource(
    const SdfPath &target, const VtVec3fArray &points)
{
    const size_t count = points.size();
    VdfInputVector<GfVec3f> *const source =
        new VdfInputVector<GfVec3f>(&_network, count);
    for (size_t i = 0; i < count; ++i) {
        source->SetValue(i, points[i]);
    }
    TF_UNUSED(target);
    const VdfMaskedOutput output(
        source->GetOutput(), VdfMask::AllOnes(count ? count : 1));
    _runtime->sources.emplace(output, source);
    _runtime->outputs.push_back(output);
    _runtime->TopologyChanged();
    return output;
}

VdfMaskedOutput
RigExecMoverGraph::AddRevision(
    RigExecRevisionOp op,
    const VdfMaskedOutput &previous,
    const RigExecMoverParameters &parameters,
    const RigExecMoverStatus &status)
{
    // The packet and status are mutable graph inputs. The evaluator refreshes
    // them after scene edits without replacing any of the connected nodes.
    VdfInputVector<RigExecMoverParameters> *const paramSource =
        new VdfInputVector<RigExecMoverParameters>(&_network, 1);
    paramSource->SetValue(0, parameters);

    VdfInputVector<RigExecMoverStatus> *const statusSource =
        new VdfInputVector<RigExecMoverStatus>(&_network, 1);
    statusSource->SetValue(0, status);

    _RevisionNode *const revision =
        new _RevisionNode(&_network, op, &_runtime->executionCount);

    const VdfMask one = VdfMask::AllOnes(1);
    _network.Connect(
        paramSource->GetOutput(), revision, _tokens->parameters, one);
    _network.Connect(
        statusSource->GetOutput(), revision, _tokens->status, one);
    _network.Connect(
        previous.GetOutput(), revision, _tokens->previous, previous.GetMask());

    ++_revisionCount;
    const VdfMaskedOutput output(revision->GetOutput(_tokens->out),
                                 previous.GetMask());
    _runtime->revisions.emplace(
        output, _Runtime::_RevisionSources{paramSource, statusSource, previous});
    _runtime->outputs.push_back(output);
    _runtime->TopologyChanged();
    return output;
}

bool
RigExecMoverGraph::UpdatePointSource(
    const VdfMaskedOutput &source, const VtVec3fArray &points)
{
    const auto it = _runtime->sources.find(source);
    if (it == _runtime->sources.end() ||
        it->second->GetSize() != points.size()) {
        return false;
    }
    bool changed = false;
    for (size_t i = 0; i < points.size(); ++i) {
        if (!it->second->IsValueEqual(i, points[i])) {
            it->second->SetValue(i, points[i]);
            changed = true;
        }
    }
    if (changed) {
        // Nonlocal kernels (smooth, surface, volume) require the full input
        // when any point changes. Do not propagate an element mask through
        // them until their individual dependency masks are implemented.
        _runtime->dirty.push_back(source);
    }
    return true;
}

bool
RigExecMoverGraph::UpdateRevision(
    const VdfMaskedOutput &revision,
    const RigExecMoverParameters &parameters,
    const RigExecMoverStatus &status)
{
    const auto it = _runtime->revisions.find(revision);
    if (it == _runtime->revisions.end()) {
        return false;
    }
    const auto &sources = it->second;
    const VdfMask one = VdfMask::AllOnes(1);
    if (!sources.parameters->IsValueEqual(0, parameters)) {
        sources.parameters->SetValue(0, parameters);
        _runtime->dirty.emplace_back(sources.parameters->GetOutput(), one);
    }
    if (!sources.status->IsValueEqual(0, status)) {
        sources.status->SetValue(0, status);
        _runtime->dirty.emplace_back(sources.status->GetOutput(), one);
    }
    return true;
}

bool
RigExecMoverGraph::ReconnectRevision(
    const VdfMaskedOutput &revision, const VdfMaskedOutput &previous)
{
    const auto it = _runtime->revisions.find(revision);
    if (it == _runtime->revisions.end() || revision == previous ||
        revision.GetMask() != previous.GetMask() ||
        (!_runtime->sources.count(previous) &&
         !_runtime->revisions.count(previous))) return false;
    if (it->second.previous == previous) return true;
    // Invalidate on the old topology before disconnecting; downstream cached
    // values must not survive a changed predecessor with an equal packet.
    _runtime->dirty.push_back(revision);
    _runtime->executor.InvalidateValues(_runtime->dirty);
    _runtime->dirty.clear();
    VdfNode *const node = &revision.GetOutput()->GetNode();
    VdfInput *const input = node->GetInput(_tokens->previous);
    while (input->GetNumConnections()) {
        _network.Disconnect(&input->GetNonConstConnection(0));
    }
    _network.Connect(previous, node, _tokens->previous);
    it->second.previous = previous;
    _runtime->dirty.push_back(revision);
    _runtime->TopologyChanged();
    return true;
}

bool
RigExecMoverGraph::RemoveRevision(const VdfMaskedOutput &revision)
{
    const auto it = _runtime->revisions.find(revision);
    if (it == _runtime->revisions.end()) return false;
    _runtime->dirty.push_back(revision);
    _runtime->executor.InvalidateValues(_runtime->dirty);
    _runtime->dirty.clear();
    VdfNode *const node = &revision.GetOutput()->GetNode();
    VdfNode *const parameters = it->second.parameters;
    VdfNode *const status = it->second.status;
    _runtime->TopologyChanged();
    for (VdfOutput *output : {revision.GetOutput(),
            it->second.parameters->GetOutput(), it->second.status->GetOutput()}) {
        _runtime->executor.ClearDataForOutput(
            output->GetId(), output->GetNode().GetId());
    }
    _runtime->outputs.erase(std::remove(_runtime->outputs.begin(),
        _runtime->outputs.end(), revision), _runtime->outputs.end());
    _runtime->revisions.erase(it);
    _network.DisconnectAndDelete(node);
    _network.DisconnectAndDelete(parameters);
    _network.DisconnectAndDelete(status);
    --_revisionCount;
    return true;
}

VtVec3fArray
RigExecMoverGraph::Evaluate(const VdfMaskedOutput &output) const
{
    if (_runtime->sources.find(output) == _runtime->sources.end() &&
        _runtime->revisions.find(output) == _runtime->revisions.end()) {
        return {};
    }
    if (!_runtime->schedule) {
        _runtime->schedule = std::make_unique<VdfSchedule>();
        // Requesting the checkpoints preserves them across READWRITE buffer
        // passing. The compute request below still pulls only this output's
        // dependencies, so unrelated branches remain untouched.
        VdfScheduler::Schedule(
            VdfRequest(_runtime->outputs), _runtime->schedule.get(), true);
        _runtime->executor.Resize(_network);
        ++_runtime->scheduleBuildCount;
    }
    if (!_runtime->dirty.empty()) {
        _runtime->executor.InvalidateValues(_runtime->dirty);
        _runtime->dirty.clear();
    }
    _runtime->executor.Run(*_runtime->schedule, VdfRequest(output));

    const VdfVector *const value = _runtime->executor.GetOutputValue(
        *output.GetOutput(), output.GetMask());
    if (!value) {
        return {};
    }
    VdfVector::ReadAccessor<GfVec3f> values =
        value->GetReadAccessor<GfVec3f>();

    VtVec3fArray result(values.GetNumValues());
    for (size_t i = 0; i < values.GetNumValues(); ++i) {
        result[i] = values[i];
    }
    return result;
}

size_t
RigExecMoverGraph::GetRevisionExecutionCount() const
{
    return _runtime->executionCount;
}

std::vector<GfMatrix4d>
RigExecMoverGraph::GetRevisionControlFrames(const VdfMaskedOutput &revision) const
{
    const auto it = _runtime->revisions.find(revision);
    return it == _runtime->revisions.end() ? std::vector<GfMatrix4d>()
        : static_cast<const _RevisionNode &>(revision.GetOutput()->GetNode()).GetControlFrames();
}

RigExecMoverStatus
RigExecMoverGraph::GetRevisionStatus(const VdfMaskedOutput &revision) const
{
    if (!_runtime->revisions.count(revision)) return {};
    return static_cast<const _RevisionNode &>(revision.GetOutput()->GetNode()).GetStatus();
}

size_t
RigExecMoverGraph::GetScheduleBuildCount() const
{
    return _runtime->scheduleBuildCount;
}

}  // namespace rigExec

//
// RigExec Hydra publication bridge implementation.
//
#include "bridge.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <algorithm>
#include <cmath>

namespace rigExec {

namespace {

// The rigid ASSET-space placement matrix of one posed frame, or nothing
// when the frame cannot supply one.
//
// Guides are rigid: the frame's scale/shear is stripped so the authored
// guide dimensions (a joint's length/radius, a control's per-axis scale)
// are the SOLE dimensional scale. An affine basis would apply the bone
// scale a second time -- to the cone height and base offset for a joint,
// to the whole shape for a control.
//
// Shared by the joint/solver payload and the control guides so the two
// cannot drift apart: a guide drawn from a frame one of them rejects and
// the other accepts would be a difference nobody could see coming.
bool
_RigidGuideMatrix(const RigExecPointFrame &frame, GfMatrix4d *result)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    static const RigExecPointFrame identity;
    GfMatrix4d m(1.0);
    if (!RigExecPointsToMatrix(identity.points, frame.points, &m)) {
        return false;
    }
    // A basis that cannot be orthonormalized (collapsed/collinear axes)
    // draws nothing rather than publishing a broken placement.
    if (!m.Orthonormalize(/* issueWarning = */ false)) {
        return false;
    }
    // Proper rigid rotation: a reflected frame keeps its +X aim but has
    // its Z basis flipped so the determinant is positive.
    if (m.GetDeterminant3() < 0) {
        for (int c = 0; c < 3; ++c) {
            m[2][c] = -m[2][c];
        }
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(m[r][c])) {
                return false;
            }
        }
    }
    *result = m;
    return true;
}

// One guide element from a posed frame: rig-space placement matrix, cone
// length, and primitive radius. Joints use the authored guide:length
// exactly (Ir contract: zero draws no cone); solver elements — which have
// no authored length — fall back to the frame's aim landmark distance.
//
// A non-positive radius draws nothing at all. Unlike length, where zero
// legitimately means "sphere but no bone", a zero-radius guide is invisible
// either way, so publishing it would only cost the viewer geometry it
// cannot see.
bool
_AppendGuideFrame(
    const RigExecPointFrame &frame, double authoredLength,
    double authoredRadius, bool useAimFallback,
    RigExecPublishedPrim *published)
{
    GfMatrix4d m(1.0);
    if (!_RigidGuideMatrix(frame, &m)) {
        return false;
    }
    double length = authoredLength;
    if (length <= 0.0) {
        length = useAimFallback
            ? (frame.X() - frame.Origin()).GetLength() : 0.0;
    }
    if (!std::isfinite(length)) {
        return false;
    }
    if (!std::isfinite(authoredRadius) || authoredRadius <= 0.0) {
        return false;
    }
    published->guideFrames.push_back(m);
    published->guideLengths.push_back(length);
    published->guideRadii.push_back(authoredRadius);
    return true;
}

// The prim's resolved render purpose, which is what BBoxCache classifies
// its extent by -- so the guide it draws is stamped with the same value.
TfToken
_ReadGuidePurpose(const UsdPrim &prim)
{
    if (const UsdGeomImageable imageable = UsdGeomImageable(prim)) {
        const TfToken purpose = imageable.ComputePurpose();
        if (!purpose.IsEmpty()) {
            return purpose;
        }
    }
    return UsdGeomTokens->default_;
}

void
_ReadGuideStyle(
    const UsdPrim &prim, UsdTimeCode time, RigExecPublishedPrim *published)
{
    if (!prim) {
        return;
    }
    published->guidePurpose = _ReadGuidePurpose(prim);
    if (UsdAttribute a = prim.GetAttribute(TfToken("guide:displayColor"))) {
        a.Get(&published->guideColor, time);
    }
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("guide:displayOpacity"))) {
        a.Get(&published->guideOpacity, time);
    }
}

// ---------------------------------------------------------------------------
// Influence volume guides (spec §4.1 volumetric extension, drawn side).
//
// A weight volume is otherwise invisible: it owns no points, and its effect
// is only legible once geometry has already moved -- by which time the
// artist is reading a deformation, not the region that caused it. What is
// actually being PLACED is the pair of surfaces the falloff band is defined
// by, so those are what is drawn: the falloffMin iso-surface (fully ON) and
// the falloffMax iso-surface (fully OFF).
//
// Everything here is built in the volume's own rigid local space, the SAME
// space the field is measured in (RigExecRigPose::weightFrames is the very
// matrix the kernels invert). That is what makes the drawing and the field
// agree by construction rather than by review -- and it is why the schema
// insists shape comes from floats and never from the transform: a scale
// baked into the placement would deform the field without deforming this
// guide, and the artist would be painting with a shape they cannot see.
// ---------------------------------------------------------------------------

// std::acos(-1) rather than M_PI, which needs _USE_MATH_DEFINES on MSVC.
const double _kTwoPi = 2.0 * std::acos(-1.0);

// Ring resolution for the round guides, matching the control guides' 32 so
// a sphere weight and a sphere control read as the same drawing.
constexpr int _kVolumeRingSegments = 32;

// Segments around a curve weight's tube. Deliberately coarser: a curve
// guide draws one of these per polyline vertex, so the vertex count is the
// product of the two, and eight already reads as round at guide scale.
constexpr int _kVolumeTubeSegments = 8;

// Appends one closed ring about \p center in the plane spanned by \p axisA
// and \p axisB, whose LENGTHS are the ring's semi-axes.
//
// The first point is repeated at the end: linear nonperiodic curves have no
// wrap, so restating the start vertex is what closes the ring. That costs
// one vertex and avoids the linear-periodic path entirely -- the same trade
// the control guides make.
void
_AppendVolumeRing(
    const GfVec3f &center, const GfVec3f &axisA, const GfVec3f &axisB,
    int segments, VtVec3fArray *points)
{
    for (int i = 0; i <= segments; ++i) {
        const double theta =
            _kTwoPi * double(i % segments) / double(segments);
        points->push_back(center +
                          float(std::cos(theta)) * axisA +
                          float(std::sin(theta)) * axisB);
    }
}

// Some unit vector perpendicular to \p t, chosen to stay well conditioned:
// crossing with the world axis LEAST aligned with t keeps the result far
// from zero length for every input.
GfVec3f
_PerpendicularTo(const GfVec3f &t)
{
    const GfVec3f seed = std::abs(t[0]) < 0.9f ? GfVec3f(1, 0, 0)
                                               : GfVec3f(0, 1, 0);
    GfVec3f u = GfCross(t, seed);
    u.Normalize();
    return u;
}

// The volume's rigid ASSET-space placement, or nothing when it cannot
// supply one.
//
// RemoveScaleShear() for exactly the reason _ResolveVolumeWeights applies
// it to the same matrix: inputs:scaleX/Y/Z is the sole authority on
// anisotropy, so a scale that leaked in through the transform would size
// the field and this guide differently.
bool
_RigidVolumeMatrix(const GfMatrix4d &placement, GfMatrix4d *result)
{
    GfMatrix4d rigid = placement.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        return false;
    }
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (!std::isfinite(rigid[r][c])) {
                return false;
            }
        }
    }
    *result = rigid;
    return true;
}

// The authored points of a curve weight's rigExec:curve target, accepting
// either an exact point3f[] property or a prim canonicalizing to .points --
// the same two spellings _ReadTargetPoints accepts in the evaluator, so a
// curve that drives a field is a curve this can draw.
bool
_ReadCurveGuidePoints(
    const UsdPrim &prim, UsdTimeCode time, VtVec3fArray *points)
{
    const UsdRelationship rel = prim.GetRelationship(TfToken("rigExec:curve"));
    if (!rel) {
        return false;
    }
    SdfPathVector targets;
    rel.GetTargets(&targets);
    if (targets.size() != 1) {
        return false;
    }
    const SdfPath &target = targets.front();
    const UsdPrim curvePrim =
        prim.GetStage()->GetPrimAtPath(target.GetPrimPath());
    if (!curvePrim) {
        return false;
    }
    const TfToken name = target.IsPropertyPath() ? target.GetNameToken()
                                                 : TfToken("points");
    const UsdAttribute attr = curvePrim.GetAttribute(name);
    return attr && attr.Get(points, time) && points->size() >= 2;
}

// One sphere iso-surface at local radius \p radius.
//
// The points are the UNIT wire sphere and the radius (times the per-axis
// divisors, which is what makes the iso-surface an ellipsoid) rides in the
// transform -- so the wire and implicit draw modes are sized identically
// and the radius can never be applied twice.
void
_AppendSphereVolumeGuide(
    double radius, const GfVec3d &axisScale, bool wire,
    const GfMatrix4d &rigidToAsset, double wireWidth,
    std::vector<RigExecVolumeGuideElement> *elements)
{
    if (!std::isfinite(radius) || radius <= 0.0) {
        // A zero- or negative-radius iso-surface is a point or nothing at
        // all. falloffMin = 0 is the schema DEFAULT, so this is the
        // ordinary case for the inner surface, not an error: the volume
        // simply draws its outer boundary alone.
        return;
    }
    GfMatrix4d scale(1.0);
    scale.SetScale(GfVec3d(radius * axisScale[0], radius * axisScale[1],
                           radius * axisScale[2]));

    RigExecVolumeGuideElement element;
    element.xform = scale * rigidToAsset;
    if (!wire) {
        // Hydra's own implicit, exactly as the joint and control guides
        // use it. Unit radius, set explicitly by the scene index.
        element.primType = HdPrimTypeTokens->sphere;
        elements->push_back(std::move(element));
        return;
    }
    element.primType = HdPrimTypeTokens->basisCurves;
    element.wireWidth = wireWidth;
    // Three orthogonal great circles: the classic wire sphere.
    static const GfVec3f kX(1, 0, 0), kY(0, 1, 0), kZ(0, 0, 1);
    static const GfVec3f kOrigin(0, 0, 0);
    _AppendVolumeRing(kOrigin, kX, kZ, _kVolumeRingSegments, &element.points);
    _AppendVolumeRing(kOrigin, kX, kY, _kVolumeRingSegments, &element.points);
    _AppendVolumeRing(kOrigin, kY, kZ, _kVolumeRingSegments, &element.points);
    element.counts = VtIntArray{_kVolumeRingSegments + 1,
                                _kVolumeRingSegments + 1,
                                _kVolumeRingSegments + 1};
    elements->push_back(std::move(element));
}

// One plane iso-surface: the rectangle at signed offset \p distance along
// the local rigExec:planeAxis, sized by the in-plane half-extents
// \p extentU and \p extentV.
//
// The size comes from inputs:extentU/extentV and from NOTHING ELSE. This
// used to derive a half-size from the falloff band (max(1, |min|, |max|)),
// which meant scrubbing the band resized the drawn square as well as
// sliding it -- a rigger dragging falloffMax saw the plane grow instead of
// the two surfaces separate. The band and the extents are different
// directions and different quantities: the band is a distance ALONG the
// axis and is the only thing \p distance carries, the extents are the size
// ACROSS it. Keeping them apart is bug (2) of this change.
//
// BOUNDED vs UNBOUNDED has to be readable at a glance, because it decides
// whether the rectangle is the field's edge or just a label. Bounded draws
// the closed rectangle alone: what you see is exactly what the field
// covers. Unbounded adds four short outward ticks at the edge midpoints --
// the arrowless "continues this way" mark, four extra segments and no
// extra draw decisions, and it survives being drawn at any scale because
// the ticks are a fraction of the extent rather than a fixed length.
// Filled corner flares or a fade would need a second material and a
// gradient the guide protocol does not carry.
//
// Note the per-axis divisors are deliberately absent: the plane kernel
// returns BEFORE inputs:scaleX/Y/Z is folded into its matrix, so a scaled
// plane guide would draw a band the field does not have.
void
_AppendPlaneVolumeGuide(
    double distance, int axis, double extentU, double extentV, bool bounded,
    bool wire, const GfMatrix4d &rigidToAsset, double wireWidth,
    std::vector<RigExecVolumeGuideElement> *elements)
{
    if (!std::isfinite(distance) || !std::isfinite(extentU) ||
        !std::isfinite(extentV) || extentU <= 0.0 || extentV <= 0.0) {
        // The same values the bounded kernel rejects, and for the same
        // reason: there is no such rectangle. An unbounded plane with a
        // degenerate extent still HAS a field, it just has no square worth
        // drawing, so this draws nothing rather than guessing a size.
        return;
    }
    GfVec3f center(0.0f);
    center[axis] = float(distance);
    GfVec3f u(0.0f), v(0.0f);
    u[(axis + 1) % 3] = float(extentU);
    v[(axis + 2) % 3] = float(extentV);

    RigExecVolumeGuideElement element;
    // The dimensions live in the points here, not in the transform: a
    // plane has no implicit Hydra form whose schema could carry them.
    element.xform = rigidToAsset;
    const GfVec3f corners[4] = {center - u - v, center - u + v,
                                center + u + v, center + u - v};
    if (wire) {
        element.primType = HdPrimTypeTokens->basisCurves;
        element.wireWidth = wireWidth;
        for (int i = 0; i < 5; ++i) {
            element.points.push_back(corners[i % 4]);
        }
        element.counts = VtIntArray{5};
    } else {
        element.primType = HdPrimTypeTokens->mesh;
        for (const GfVec3f &corner : corners) {
            element.points.push_back(corner);
        }
        element.counts = VtIntArray{4};
        element.indices = VtIntArray{0, 1, 2, 3};
    }
    elements->push_back(std::move(element));

    if (bounded) {
        return;
    }
    // The ticks are their own element rather than extra curves on the
    // rectangle: in `geometry` draw mode the rectangle is a mesh, and a
    // mesh cannot carry them. One element type per element keeps both
    // draw modes on the same code.
    RigExecVolumeGuideElement ticks;
    ticks.xform = rigidToAsset;
    ticks.primType = HdPrimTypeTokens->basisCurves;
    ticks.wireWidth = wireWidth;
    // Each half-extent vector reaches the midpoint of one edge AND points
    // outward from it, so one array does both jobs.
    const GfVec3f outward[4] = {u, -u, v, -v};
    for (const GfVec3f &out : outward) {
        const GfVec3f base = center + out;
        ticks.points.push_back(base);
        ticks.points.push_back(base + out * 0.25f);
        ticks.counts.push_back(2);
    }
    elements->push_back(std::move(ticks));
}

// One curve iso-surface: the tube of local radius \p radius about the
// polyline through \p localCurve.
//
// Drawn as rings at every polyline vertex plus four longitudinal rails
// joining them -- a ladder rather than a swept solid. That is enough to
// read the tube's radius and its path at a glance, costs no surfacing
// decisions (a real sweep needs a rotation-minimizing frame to avoid
// twisting), and stays a curve set in both draw modes.
//
// The polyline is already in the DIVIDED local space, so the tube is round
// here and the per-axis divisors turn it elliptical through the transform
// -- the same order the kernel folds them in.
void
_AppendCurveVolumeGuide(
    double radius, const VtVec3fArray &localCurve, const GfMatrix4d &toAsset,
    double wireWidth, std::vector<RigExecVolumeGuideElement> *elements)
{
    if (!std::isfinite(radius) || radius <= 0.0 || localCurve.size() < 2) {
        return;
    }
    const size_t vertices = localCurve.size();
    // Per-vertex frames, from the average of the adjoining segment
    // directions so the tube does not kink at an interior vertex.
    std::vector<GfVec3f> ringU(vertices), ringV(vertices);
    for (size_t k = 0; k < vertices; ++k) {
        GfVec3f tangent(0.0f);
        if (k + 1 < vertices) {
            tangent += localCurve[k + 1] - localCurve[k];
        }
        if (k > 0) {
            tangent += localCurve[k] - localCurve[k - 1];
        }
        if (tangent.GetLength() < 1e-9f) {
            // Coincident neighbours: any frame is as good as any other,
            // and refusing to draw would punch a hole in the tube.
            tangent = GfVec3f(1, 0, 0);
        }
        tangent.Normalize();
        ringU[k] = float(radius) * _PerpendicularTo(tangent);
        GfVec3f w = GfCross(tangent, ringU[k]);
        w.Normalize();
        ringV[k] = float(radius) * w;
    }

    RigExecVolumeGuideElement element;
    element.xform = toAsset;
    element.primType = HdPrimTypeTokens->basisCurves;
    element.wireWidth = wireWidth;
    for (size_t k = 0; k < vertices; ++k) {
        _AppendVolumeRing(localCurve[k], ringU[k], ringV[k],
                          _kVolumeTubeSegments, &element.points);
        element.counts.push_back(_kVolumeTubeSegments + 1);
    }
    // Four rails, at the quarter turns of each ring.
    for (int quarter = 0; quarter < 4; ++quarter) {
        const double theta = _kTwoPi * double(quarter) / 4.0;
        const float c = float(std::cos(theta)), s = float(std::sin(theta));
        for (size_t k = 0; k < vertices; ++k) {
            element.points.push_back(localCurve[k] + c * ringU[k] +
                                     s * ringV[k]);
        }
        element.counts.push_back(int(vertices));
    }
    elements->push_back(std::move(element));
}

}  // namespace

RigExecImagingBridge::RigExecImagingBridge(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
    : RigExecImagingBridge(
          stage, rigPath, std::make_shared<RigExecSnapshotStore>())
{
}

RigExecImagingBridge::RigExecImagingBridge(
    const UsdStageRefPtr &stage, const SdfPath &rigPath,
    std::shared_ptr<RigExecSnapshotStore> store)
    : _stage(stage)
    , _rigPath(rigPath)
    , _evaluator(std::make_unique<RigExecRigEvaluator>(stage, rigPath))
    , _store(std::move(store))
{
}

bool
RigExecImagingBridge::Compile(std::vector<std::string> *errors)
{
    return _evaluator->Compile(errors);
}

SdfPath
RigExecImagingBridge::GetGeneratedScope() const
{
    return _rigPath.AppendChild(TfToken("__RigExecGenerated"));
}

// Constraint-driven transforms, published onto the prim itself so parented
// geometry rides along. Base and revision travel together: the scene index
// sits downstream of Hydra's flatten and turns the pair into a world-space
// delta (see RigExecPublishedPrim::xformBase).
void
RigExecImagingBridge::_FillProviderXforms(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    for (const auto &[providerPath, matrix] : pose.providerXforms) {
        const auto baseIt = pose.providerBaseXforms.find(providerPath);
        if (baseIt == pose.providerBaseXforms.end()) {
            // The evaluator writes both or neither; a revision without its
            // base cannot be turned into a delta, so publishing it would move
            // the prim by an arbitrary amount.
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[providerPath];
        published.xform = matrix;
        published.xformBase = baseIt->second;
        published.hasXform = true;
        snapshot->hasDrivenXforms = true;
    }
}

void
RigExecImagingBridge::_FillGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    // Guide frames are asset-space; the consumer needs to know which prim
    // that space is anchored to in order to place them.
    snapshot->assetRoot = _rigPath.GetParentPath();

    for (const auto &[jointPath, frame] : pose.jointFramesFinal) {
        const UsdPrim prim = _stage->GetPrimAtPath(jointPath);
        double authoredLength = 0;
        // Schema default, so an unauthored joint keeps Hydra's own fallback
        // radius and every existing rig looks exactly as it did.
        double authoredRadius = 1.0;
        if (prim) {
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("guide:length"))) {
                a.Get(&authoredLength, pose.time);
            }
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("guide:radius"))) {
                a.Get(&authoredRadius, pose.time);
            }
        }
        RigExecPublishedPrim &published = snapshot->prims[jointPath];
        if (_AppendGuideFrame(frame, authoredLength, authoredRadius,
                              /* useAimFallback = */ false, &published)) {
            published.assetRoot = _rigPath.GetParentPath();
            published.hasGuides = true;
            _ReadGuideStyle(prim, pose.time, &published);
        }
    }
    for (const auto &[solverPath, frames] : pose.solverFrames) {
        RigExecPublishedPrim &published = snapshot->prims[solverPath];
        // An element has no prim of its own, but the SOLVER does, and it
        // is already where the guide colour and opacity are read from --
        // so the radius comes off it too, exactly as a joint's does. The
        // schema default is 1.0, so a rig that authors nothing draws what
        // it always drew.
        const UsdPrim solverPrim = _stage->GetPrimAtPath(solverPath);
        double authoredRadius = 1.0;
        if (solverPrim) {
            if (UsdAttribute a =
                    solverPrim.GetAttribute(TfToken("guide:radius"))) {
                a.Get(&authoredRadius, pose.time);
            }
        }
        bool any = false;
        for (const RigExecPointFrame &frame : frames) {
            any = _AppendGuideFrame(frame, 0.0, authoredRadius,
                                    /* useAimFallback = */ true,
                                    &published) ||
                  any;
        }
        if (any) {
            published.assetRoot = _rigPath.GetParentPath();
            published.hasGuides = true;
            _ReadGuideStyle(solverPrim, pose.time, &published);
        } else if (!published.hasPoints && !published.hasNormals &&
                   !published.hasExtent && !published.hasXform) {
            snapshot->prims.erase(solverPath);
        }
    }
}

// Controls draw one synthesized shape each at their posed frame
// (spec §10.3 extension): shape, draw mode, and per-axis scale are
// authored on the control, and the results scene index turns the published
// payload into the child prim.
void
RigExecImagingBridge::_FillControlGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    // Set here as well as in _FillGuides: control guide frames are
    // asset-space too, and a rig can publish these and no joint guides at
    // all (every joint frame degenerate, say), in which case this is the
    // only place the anchor gets recorded.
    snapshot->assetRoot = _rigPath.GetParentPath();

    for (const auto &[controlPath, frame] : pose.controlFrames) {
        GfMatrix4d placement(1.0);
        if (!_RigidGuideMatrix(frame, &placement)) {
            continue;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(controlPath);
        // The schema fallbacks, restated. Normally GetAttribute resolves
        // them for us, but a stage composed without the codeless schema
        // plugin registered has no fallback to find, and an empty shape
        // token names no shape at all -- so a rig would silently stop
        // drawing control guides rather than draw the documented default.
        TfToken shape("circle");
        TfToken drawMode("wire");
        GfVec3d scale(1.0, 1.0, 1.0);
        double wireWidth = 0.05;
        if (prim) {
            if (UsdAttribute a = prim.GetAttribute(TfToken("guide:shape"))) {
                a.Get(&shape, pose.time);
            }
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("guide:drawMode"))) {
                a.Get(&drawMode, pose.time);
            }
            static const TfToken scaleAttrs[3] = {
                TfToken("guide:scaleX"), TfToken("guide:scaleY"),
                TfToken("guide:scaleZ")};
            for (int axis = 0; axis < 3; ++axis) {
                if (UsdAttribute a = prim.GetAttribute(scaleAttrs[axis])) {
                    a.Get(&scale[axis], pose.time);
                }
            }
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("guide:wireWidth"))) {
                a.Get(&wireWidth, pose.time);
            }
        }
        // Unlike the scale, a non-positive width is NOT a reason to draw
        // nothing: it selects the hairline fallback, which is what wire
        // guides did before the width existed. Non-finite collapses to the
        // same thing rather than reaching the curves as a NaN.
        if (!std::isfinite(wireWidth) || wireWidth <= 0.0) {
            wireWidth = 0.0;
        }
        // A non-finite or non-positive scale on ANY axis draws nothing,
        // mirroring the joint guide:radius rule: a flattened shape is
        // invisible from most angles and degenerate from the rest, so
        // publishing it would only cost the viewer geometry it cannot
        // meaningfully see.
        if (!std::isfinite(scale[0]) || !std::isfinite(scale[1]) ||
            !std::isfinite(scale[2]) || scale[0] <= 0.0 ||
            scale[1] <= 0.0 || scale[2] <= 0.0) {
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[controlPath];
        published.assetRoot = _rigPath.GetParentPath();
        published.hasControlGuide = true;
        published.controlGuideFrame = placement;
        published.controlGuideShape = shape;
        published.controlGuideDrawMode = drawMode;
        published.controlGuideScale = scale;
        published.controlGuideWireWidth = wireWidth;
        _ReadGuideStyle(prim, pose.time, &published);
    }
}


// Every placed influence volume draws its falloffMin and falloffMax
// iso-surfaces (spec §4.1 volumetric extension, drawn side).
//
// Driven by pose.weightFrames rather than by a stage traversal: that map
// holds exactly the volumes the current epoch RESOLVED a placement for, so
// the set that draws is the set that computes. A volume nothing consumes
// has no resolved frame and correctly draws nothing -- there is no field to
// visualise the region of.
void
RigExecImagingBridge::_FillVolumeGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    // Set here too: a rig may publish these and no joint or control
    // guides at all, in which case this is the only place the asset-space
    // anchor gets recorded (see _FillGuides).
    snapshot->assetRoot = _rigPath.GetParentPath();

    static const TfToken kWire("wire");
    static const TfToken kGeometry("geometry");

    for (const auto &[weightPath, placement] : pose.weightFrames) {
        const UsdPrim prim = _stage->GetPrimAtPath(weightPath);
        if (!prim) {
            continue;
        }
        // The schema fallbacks, restated for the same reason
        // _FillControlGuides restates its own: a stage composed without
        // the codeless schema plugin registered has no fallback to
        // resolve, and a volume would silently stop drawing rather than
        // draw the documented default.
        TfToken drawMode(kWire);
        if (UsdAttribute a = prim.GetAttribute(TfToken("guide:drawMode"))) {
            a.Get(&drawMode, pose.time);
        }
        if (drawMode != kWire && drawMode != kGeometry) {
            // `none` -- and any unrecognized token, which has to mean
            // something definite and means the same thing here as an
            // unknown control shape does: nothing is drawn.
            continue;
        }
        const bool wire = drawMode == kWire;

        GfMatrix4d rigid(1.0);
        if (!_RigidVolumeMatrix(placement, &rigid)) {
            continue;
        }

        auto readFloat = [&prim, &pose](const char *name, float fallback) {
            float value = fallback;
            if (UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                a.Get(&value, pose.time);
            }
            return value;
        };
        const double falloffMin = readFloat("inputs:falloffMin", 0.0f);
        const double falloffMax = readFloat("inputs:falloffMax", 1.0f);
        if (!std::isfinite(falloffMin) || !std::isfinite(falloffMax)) {
            continue;
        }
        double wireWidth = 0.05;
        if (UsdAttribute a = prim.GetAttribute(TfToken("guide:wireWidth"))) {
            a.Get(&wireWidth, pose.time);
        }
        // A non-positive width is not a reason to draw nothing: it
        // selects the hairline fallback, exactly as it does for a control.
        if (!wire || !std::isfinite(wireWidth) || wireWidth <= 0.0) {
            wireWidth = 0.0;
        }

        const TfToken typeName = prim.GetTypeName();
        std::vector<RigExecVolumeGuideElement> elements;

        if (typeName == "RigExecPlaneWeight") {
            TfToken axisToken("y");
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:planeAxis"))) {
                a.Get(&axisToken, pose.time);
            }
            const int axis = axisToken == "x" ? 0
                : (axisToken == "y" ? 1 : (axisToken == "z" ? 2 : -1));
            if (axis < 0) {
                continue;  // the kernel rejects it too
            }
            // Size from the authored extents ONLY. The band moves the two
            // surfaces apart along the axis; it must not change their
            // size, which is what the old max(1, |min|, |max|) half-size
            // did on every scrub.
            const double extentU = readFloat("inputs:extentU", 1.0f);
            const double extentV = readFloat("inputs:extentV", 1.0f);
            TfToken boundsToken("unbounded");
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:planeBounds"))) {
                a.Get(&boundsToken, pose.time);
            }
            if (boundsToken != "unbounded" && boundsToken != "bounded") {
                continue;  // the kernel rejects it too
            }
            const bool bounded = boundsToken == "bounded";
            for (const double distance : {falloffMin, falloffMax}) {
                _AppendPlaneVolumeGuide(distance, axis, extentU, extentV,
                                        bounded, wire, rigid, wireWidth,
                                        &elements);
            }
        } else {
            // Sphere and curve share the per-axis divisors, and reject the
            // same values the kernel rejects -- a volume whose packet is
            // invalid must not draw a shape suggesting it is fine.
            const GfVec3d axisScale(readFloat("inputs:scaleX", 1.0f),
                                    readFloat("inputs:scaleY", 1.0f),
                                    readFloat("inputs:scaleZ", 1.0f));
            bool scaleOk = true;
            for (int i = 0; i < 3; ++i) {
                if (!std::isfinite(axisScale[i]) || axisScale[i] <= 0.0) {
                    scaleOk = false;
                }
            }
            if (!scaleOk) {
                continue;
            }
            if (typeName == "RigExecSphereWeight") {
                for (const double radius : {falloffMin, falloffMax}) {
                    _AppendSphereVolumeGuide(radius, axisScale, wire, rigid,
                                             wireWidth, &elements);
                }
            } else if (typeName == "RigExecCurveWeight") {
                VtVec3fArray curvePoints;
                if (!_ReadCurveGuidePoints(prim, pose.time, &curvePoints)) {
                    continue;
                }
                // Into the DIVIDED local space the kernel measures in:
                // rigid^-1 then the per-axis divisors, so the drawn tube
                // is round exactly where the field's is.
                GfMatrix4d divide(1.0);
                divide.SetScale(GfVec3d(1.0 / axisScale[0],
                                        1.0 / axisScale[1],
                                        1.0 / axisScale[2]));
                const GfMatrix4d toLocal = rigid.GetInverse() * divide;
                VtVec3fArray localCurve(curvePoints.size());
                for (size_t k = 0; k < curvePoints.size(); ++k) {
                    localCurve[k] =
                        GfVec3f(toLocal.Transform(GfVec3d(curvePoints[k])));
                }
                GfMatrix4d scale(1.0);
                scale.SetScale(axisScale);
                const GfMatrix4d toAsset = scale * rigid;
                for (const double radius : {falloffMin, falloffMax}) {
                    _AppendCurveVolumeGuide(radius, localCurve, toAsset,
                                            wireWidth, &elements);
                }
            } else {
                continue;  // not a shape this version draws
            }
        }

        if (elements.empty()) {
            continue;
        }
        // Composed with the asset root by the scene index, like every
        // other guide: these frames carry no stage placement.
        RigExecPublishedPrim &published = snapshot->prims[weightPath];
        published.assetRoot = _rigPath.GetParentPath();
        published.hasVolumeGuides = true;
        published.volumeGuides = std::move(elements);
        _ReadGuideStyle(prim, pose.time, &published);
    }
}

// Paints ONE selected weight object's resolved field onto the geometry it
// weights (spec §10.3 influence-overlay extension).
//
// The field is taken from pose.weightFields -- the packet a mover actually
// consumed -- rather than recomputed, so what a rigger sees is exactly what
// deformed the geometry. A re-derivation could drift, and it would drift
// silently: a plausible-looking gradient over the wrong region is
// indistinguishable from a correct one by eye, which is the whole reason
// the visualisation exists.
void
RigExecImagingBridge::_FillWeightOverlay(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    if (_weightOverlay.IsEmpty()) {
        return;  // off: an ordinary render is untouched
    }
    const auto it = pose.weightFields.find(_weightOverlay);
    if (it == pose.weightFields.end() || it->second.weights.empty()) {
        return;
    }
    // The field carries its own geometry target: the selection names a
    // WEIGHT OBJECT, and which prim that paints is the evaluator's answer,
    // not the caller's.
    const SdfPath geomPath = it->second.target.GetPrimPath();
    if (geomPath.IsEmpty() || geomPath.IsAbsoluteRootPath()) {
        return;
    }
    RigExecPublishedPrim &published = snapshot->prims[geomPath];
    published.hasWeightOverlay = true;
    published.weightOverlay =
        VtFloatArray(it->second.weights.begin(), it->second.weights.end());
}

// Stamps WHICH STAGE and WHICH TIME a generation describes onto it.
//
// The store is process-global, so a consumer that finds a prim by path
// alone -- the compute-extent callback is the one USD calls with a stage
// and time of its own choosing -- would otherwise accept whatever rig
// published last, for whatever frame.
void
RigExecImagingBridge::_StampGeneration(
    UsdTimeCode time, RigExecImagingSnapshot *snapshot) const
{
    snapshot->stage = UsdStageWeakPtr(_stage);
    snapshot->sampleTimeIsDefault = time.IsDefault();
    snapshot->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();
}

bool
RigExecImagingBridge::EvaluateAndPublish(UsdTimeCode time)
{
    const PublishResult result = EvaluateAndPublishResult(time);
    // Deliver the notices even when evaluation failed. A failed generation
    // still CLEARS the previous one, and that clearing is only worth
    // anything if it reaches the observers -- returning early here was the
    // second half of the stale-pose bug: the store had been corrected and
    // nobody was told.
    if (_binding && result.epoch) {
        _binding->SetBindingEpoch(result.epoch);
    }
    if (_results && !result.dirtied.empty()) {
        _results->NotifyGenerationPublished(result.dirtied);
    }
    return result.ok;
}

RigExecImagingBridge::PublishResult
RigExecImagingBridge::EvaluateAndPublishResult(UsdTimeCode time)
{
    PublishResult result;
    // 1. Evaluation always completes before publication (spec §8.2).
    const RigExecRigPose pose = _evaluator->Evaluate(time);
    if (!pose.valid) {
        // A rig that cannot evaluate STOPS DRIVING THE SCENE.
        //
        // Returning here without publishing used to leave the last good
        // generation current, and every consumer kept serving it: disconnect
        // a constraint's rigExec:moves and the recompile fails ("Mover has no
        // moves targets"), so the mesh under the driven Xform stayed exactly
        // where the constraint had put it -- forever, through frame changes
        // and further edits.
        //
        // Nothing downstream could correct that. The results scene index
        // handles a driven transform disappearing, and the store diffs a prim
        // that left the generation as structural -- but neither runs, because
        // there was no publication for them to run on.
        //
        // Clearing is the rule the evaluator already applies to a
        // non-converged generation: an unevaluated result is recoverable, a
        // plausible wrong one is not. The scene falls back to what the stage
        // authors, which reads as "the rig is not running" instead of being
        // invisibly stale.
        result.dirtied = _store->Publish(nullptr);
        // The next good generation has to re-announce its epoch: the binding
        // index is still holding one whose published prim set no longer
        // exists, and an unchanged digest would suppress the replacement.
        _publishedEpochDigest = 0;
        return result;
    }

    // 2. Build the complete immutable generation from the exact native
    // results: points and normals as flat primvars, extent as min/max
    // (spec §10.2). Only standard data crosses this boundary.
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->generation = ++_generation;
    _StampGeneration(time, snapshot.get());
    for (const auto &[propertyPath, value] : pose.movedProperties) {
        const SdfPath primPath = propertyPath.GetPrimPath();
        const TfToken property = propertyPath.GetNameToken();
        // Only geometry crosses into imaging. movedProperties also carries
        // the property-domain math mover results (a float dial, a vector
        // offset, a matrix), which have no Hydra representation -- and
        // indexing snapshot->prims for one of those would MINT an empty
        // published prim for its owner, which then enters the binding epoch
        // below and makes the scene index resolve a prim RigExec publishes
        // nothing for. Look the entry up only once there is something to
        // put in it.
        const bool isGeometry =
            value.IsHolding<VtVec3fArray>() &&
            (property == "points" || property == "normals" ||
             property == "extent");
        if (!isGeometry) {
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[primPath];
        if (property == "points") {
            published.hasPoints = true;
            published.points = value.UncheckedGet<VtVec3fArray>();
        } else if (property == "normals") {
            published.hasNormals = true;
            published.normals = value.UncheckedGet<VtVec3fArray>();
        } else {
            const auto extent = value.UncheckedGet<VtVec3fArray>();
            if (extent.size() == 2) {
                published.hasExtent = true;
                published.extentMin = GfVec3d(extent[0]);
                published.extentMax = GfVec3d(extent[1]);
            }
        }
    }
    // Joints and aggregate solvers publish guide payloads (drawn by the
    // results scene index like OpenExec's IrJointScope guides).
    _FillProviderXforms(pose, snapshot.get());
    _FillGuides(pose, snapshot.get());
    _FillControlGuides(pose, snapshot.get());
    _FillVolumeGuides(pose, snapshot.get());
    _FillWeightOverlay(pose, snapshot.get());

    // 3. A structural recompile publishes a replacement binding epoch
    // before value notices (spec §10.4).
    const size_t epochDigest = _evaluator->GetBindingEpochDigest();
    if (epochDigest != _publishedEpochDigest) {
        auto epoch = std::make_shared<
            RigExecBindingResolvingSceneIndex::BindingEpoch>();
        epoch->id = epochDigest;
        for (const auto &[primPath, published] : snapshot->prims) {
            epoch->publishedPrims.insert(primPath);
        }
        result.epoch = std::move(epoch);
        _publishedEpochDigest = epochDigest;
    }

    // 4. Atomic snapshot swap; the caller sends the coalesced precise
    // dirtied notices from the notice owner (spec §8.2, §10.4).
    result.dirtied = _store->Publish(std::move(snapshot));
    result.ok = true;
    return result;
}

bool
RigExecImagingBridge::PreflightMotionProfile(
    const std::vector<float> &shutterOffsets, MotionBlurSupport support,
    std::string *whyNot)
{
    if (shutterOffsets.empty()) {
        if (whyNot) {
            *whyNot = "render preflight requires at least one explicit "
                      "shutter offset";
        }
        return false;
    }
    // motionBlurSupport = false permits one sample (spec §10.3.1); the
    // capability bit never supplies offsets, and an absent bit leaves the
    // application's explicit render profile authoritative.
    if (support == MotionBlurSupport::False && shutterOffsets.size() > 1) {
        if (whyNot) {
            *whyNot = "renderer advertises motionBlurSupport = false; a "
                      "multi-sample motion profile fails preflight";
        }
        return false;
    }
    return true;
}

RigExecImagingBridge::PublishResult
RigExecImagingBridge::EvaluateAndPublishSamples(
    UsdTimeCode baseTime, const std::vector<float> &shutterOffsets,
    MotionBlurSupport support)
{
    PublishResult result;
    // Preflight fails deterministically before any evaluation
    // (spec §10.5): capability mismatch or an invalid profile never
    // computes and never publishes.
    if (!PreflightMotionProfile(shutterOffsets, support) ||
        baseTime.IsDefault()) {
        return result;
    }
    // The offset nearest zero supplies the primary (non-sampled) outputs.
    size_t baseIndex = 0;
    for (size_t i = 1; i < shutterOffsets.size(); ++i) {
        if (std::abs(shutterOffsets[i]) <
            std::abs(shutterOffsets[baseIndex])) {
            baseIndex = i;
        }
    }

    // Evaluation of every required offset completes before anything
    // publishes (spec 8.2, 10.5): all samples are captured, then one
    // complete immutable generation swaps in.
    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    std::map<SdfPath, std::vector<VtVec3fArray>> pointsPerPrim;
    for (size_t i = 0; i < shutterOffsets.size(); ++i) {
        const UsdTimeCode sampleTime(
            baseTime.GetValue() + double(shutterOffsets[i]));
        const RigExecRigPose pose = _evaluator->Evaluate(sampleTime);
        if (!pose.valid) {
            return result;  // an incomplete sample set never publishes
        }
        if (i == baseIndex) {
            // Guides are single-sampled at the base offset.
            _FillProviderXforms(pose, snapshot.get());
            _FillGuides(pose, snapshot.get());
            _FillControlGuides(pose, snapshot.get());
            _FillVolumeGuides(pose, snapshot.get());
            _FillWeightOverlay(pose, snapshot.get());
        }
        for (const auto &[propertyPath, value] : pose.movedProperties) {
            const SdfPath primPath = propertyPath.GetPrimPath();
            const TfToken property = propertyPath.GetNameToken();
            // Geometry only, and checked before snapshot->prims is indexed:
            // see EvaluateAndPublishResult for why minting an entry for a
            // property-domain result is wrong.
            const bool isGeometry =
                value.IsHolding<VtVec3fArray>() &&
                (property == "points" || property == "normals" ||
                 property == "extent");
            if (!isGeometry) {
                continue;
            }
            if (property == "points") {
                pointsPerPrim[primPath].push_back(
                    value.UncheckedGet<VtVec3fArray>());
            }
            if (i == baseIndex) {
                RigExecPublishedPrim &published = snapshot->prims[primPath];
                if (property == "points") {
                    published.hasPoints = true;
                    published.points = value.UncheckedGet<VtVec3fArray>();
                } else if (property == "normals") {
                    published.hasNormals = true;
                    published.normals = value.UncheckedGet<VtVec3fArray>();
                } else {
                    const auto extent =
                        value.UncheckedGet<VtVec3fArray>();
                    if (extent.size() == 2) {
                        published.hasExtent = true;
                        published.extentMin = GfVec3d(extent[0]);
                        published.extentMax = GfVec3d(extent[1]);
                    }
                }
            }
        }
    }
    for (auto &[primPath, samples] : pointsPerPrim) {
        if (samples.size() != shutterOffsets.size()) {
            return result;  // missing required samples fail preflight
        }
        RigExecPublishedPrim &published = snapshot->prims[primPath];
        published.sampleOffsets = shutterOffsets;
        published.pointsSamples = std::move(samples);
    }
    snapshot->generation = ++_generation;
    // The sampled path is identified by its BASE time: that is the frame a
    // consumer asks about, and the offsets are relative to it.
    _StampGeneration(baseTime, snapshot.get());

    const size_t epochDigest = _evaluator->GetBindingEpochDigest();
    if (epochDigest != _publishedEpochDigest) {
        auto epoch = std::make_shared<
            RigExecBindingResolvingSceneIndex::BindingEpoch>();
        epoch->id = epochDigest;
        for (const auto &[primPath, published] : snapshot->prims) {
            epoch->publishedPrims.insert(primPath);
        }
        result.epoch = std::move(epoch);
        _publishedEpochDigest = epochDigest;
    }
    result.dirtied = _store->Publish(std::move(snapshot));
    result.ok = true;
    return result;
}

}  // namespace rigExec

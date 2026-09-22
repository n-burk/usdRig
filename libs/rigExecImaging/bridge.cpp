//
// RigExec Hydra publication bridge implementation.
//
#include "bridge.h"

#include "warmIndex.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecMath/curvenet.h"
#include "rigExec/frameCacheSparsity.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/tokens.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <set>

namespace rigExec {

namespace {

// The rigid ASSET-space placement matrix of one posed frame, or nothing
// when the frame cannot supply one.
//
// Guides are placed by a rigid matrix: scale/shear never leaks into the
// placement itself. Joint link lengths come from child origins, solver
// dimensions come from their frame landmarks, and controls ask for the
// removed basis magnitudes separately and fold those into their positive
// authored guide:scaleX/Y/Z multipliers.
//
// Shared by the joint/solver payload and the control guides so the two
// cannot drift apart: a guide drawn from a frame one of them rejects and
// the other accepts would be a difference nobody could see coming.
bool
_RigidGuideMatrix(
    const RigExecPointFrame &frame, GfMatrix4d *result,
    GfVec3d *axisMagnitudes = nullptr)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    static const RigExecPointFrame identity;
    GfMatrix4d m(1.0);
    if (!RigExecPointsToMatrix(identity.points, frame.points, &m)) {
        return false;
    }
    if (axisMagnitudes) {
        for (int axis = 0; axis < 3; ++axis) {
            const double magnitude = m.GetRow3(axis).GetLength();
            if (!std::isfinite(magnitude) || magnitude <= 0.0) {
                return false;
            }
            (*axisMagnitudes)[axis] = magnitude;
        }
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
// length, primitive radius, and whether it owns an origin sphere. Solver
// elements derive length from the frame's aim landmark distance. Joint
// elements pass child-derived link frames and lengths directly.
//
// A non-positive radius draws nothing at all. Unlike length, where zero
// legitimately means "sphere but no bone", a zero-radius guide is invisible
// either way, so publishing it would only cost the viewer geometry it
// cannot see.
bool
_AppendGuideFrame(
    const RigExecPointFrame &frame, double length, double authoredRadius,
    bool useAimFallback, bool drawSphere,
    RigExecPublishedPrim *published)
{
    GfMatrix4d m(1.0);
    if (!_RigidGuideMatrix(frame, &m)) {
        return false;
    }
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
    published->guideDrawSpheres.push_back(drawSphere);
    return true;
}

// Builds the rigid frame and length of the link from parent to child. The
// cone is rotationally symmetric about X, so preserving the parent's Y axis
// where possible gives a stable frame without assigning semantic roll to the
// hierarchy edge.
bool
_JointLinkFrame(
    const RigExecPointFrame &parent, const RigExecPointFrame &child,
    GfMatrix4d *frame, double *length)
{
    GfMatrix4d parentMatrix(1.0), childMatrix(1.0);
    if (!_RigidGuideMatrix(parent, &parentMatrix) ||
        !_RigidGuideMatrix(child, &childMatrix)) {
        return false;
    }
    GfVec3d x = childMatrix.ExtractTranslation() -
                parentMatrix.ExtractTranslation();
    *length = x.GetLength();
    if (!std::isfinite(*length) || *length <= 1e-12) {
        return false;
    }
    x /= *length;

    GfVec3d y = parentMatrix.GetRow3(1);
    y -= GfDot(y, x) * x;
    if (y.GetLength() <= 1e-12) {
        y = parentMatrix.GetRow3(2);
        y -= GfDot(y, x) * x;
    }
    if (y.GetLength() <= 1e-12) {
        const GfVec3d seed = std::abs(x[0]) < 0.9
            ? GfVec3d(1, 0, 0) : GfVec3d(0, 1, 0);
        y = GfCross(seed, x);
    }
    y.Normalize();
    GfVec3d z = GfCross(x, y);
    z.Normalize();
    y = GfCross(z, x);
    y.Normalize();

    *frame = parentMatrix;
    for (int column = 0; column < 3; ++column) {
        (*frame)[0][column] = x[column];
        (*frame)[1][column] = y[column];
        (*frame)[2][column] = z[column];
    }
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

// The value the FIRST connection source of \p attr holds at the pose's
// time, or nothing when the attribute has no connection or the source
// cannot supply one (dangling path, a relationship, no value).
//
// UsdAttribute::Get() does NOT follow connections: they are a
// shading-graph concept, not value resolution. So a rig that authored
// `guide:displayOpacity.connect = </limb_params.avars:ikfk>` compiled,
// drew nothing different, and gave no hint why. This is the read-through.
//
// A property-mover result for the source wins over its authored value,
// exactly as the volume guides read their driven dimensions: a dial that
// is itself computed must fade the guide by what it computed.
bool
_ReadConnectedValue(
    const UsdAttribute &attr, const RigExecRigPose &pose, VtValue *held)
{
    SdfPathVector connections;
    if (!attr.GetConnections(&connections) || connections.empty()) {
        return false;
    }
    const SdfPath &sourcePath = connections.front();
    if (!sourcePath.IsPropertyPath()) {
        return false;
    }
    const UsdAttribute source =
        attr.GetStage()->GetAttributeAtPath(sourcePath);
    if (!source) {
        return false;
    }
    const auto moved = pose.movedProperties.find(sourcePath);
    if (moved != pose.movedProperties.end() && !moved->second.IsEmpty()) {
        *held = moved->second;
        return true;
    }
    return source.Get(held, pose.time);
}

// The scalar a float OR double VtValue holds -- avars are double, the
// biped's ikfk dial is float, and a guide must accept either -- or
// nothing when it holds neither or the value is not finite.
bool
_HeldScalar(const VtValue &held, double *value)
{
    if (held.IsHolding<float>()) {
        *value = held.UncheckedGet<float>();
    } else if (held.IsHolding<double>()) {
        *value = held.UncheckedGet<double>();
    } else {
        return false;
    }
    return std::isfinite(*value);
}

// The schema fallback for guide:displayOpacityMin, restated for the same
// reason _FillControlGuides restates its own: a stage composed without the
// codeless schema plugin has no fallback to resolve.
constexpr double _kDefaultGuideOpacityMin = 0.15;

void
_ReadGuideStyle(
    const UsdPrim &prim, const RigExecRigPose &pose,
    RigExecPublishedPrim *published)
{
    if (!prim) {
        return;
    }
    const UsdTimeCode time = pose.time;
    published->guidePurpose = _ReadGuidePurpose(prim);
    if (UsdAttribute a = prim.GetAttribute(TfToken("guide:displayColor"))) {
        VtValue held;
        if (_ReadConnectedValue(a, pose, &held) &&
            held.IsHolding<GfVec3f>()) {
            published->guideColor = held.UncheckedGet<GfVec3f>();
        } else {
            a.Get(&published->guideColor, time);
        }
    }
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("guide:displayOpacity"))) {
        VtValue held;
        double driven = 0.0;
        if (_ReadConnectedValue(a, pose, &held) &&
            _HeldScalar(held, &driven)) {
            // A driven opacity is a dial's value, and a dial parks at its
            // end stops: without the complement one switch could not fade
            // two control sets in opposite directions, and without the
            // floor the inactive set would vanish -- unfindable, so
            // unswitchable-back. Both apply ONLY to a connected value; an
            // unconnected attribute draws exactly what it says.
            bool invert = false;
            if (UsdAttribute i = prim.GetAttribute(
                    TfToken("guide:displayOpacityInvert"))) {
                i.Get(&invert, time);
            }
            double floor = _kDefaultGuideOpacityMin;
            if (UsdAttribute m = prim.GetAttribute(
                    TfToken("guide:displayOpacityMin"))) {
                VtValue heldMin;
                double authoredMin = 0.0;
                if (m.Get(&heldMin, time) &&
                    _HeldScalar(heldMin, &authoredMin)) {
                    floor = authoredMin;
                }
            }
            floor = std::min(1.0, std::max(0.0, floor));
            double opacity = invert ? 1.0 - driven : driven;
            opacity = std::min(1.0, std::max(floor, opacity));
            published->guideOpacity = static_cast<float>(opacity);
        } else {
            a.Get(&published->guideOpacity, time);
        }
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
        // A rectangle has no thickness. Make both sides visible, unlike the
        // closed curve tube whose outward winding is sufficient.
        element.doubleSided = true;
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
// Wire mode draws rings at every polyline vertex plus four longitudinal
// rails. Geometry mode uses the same transported rings as shared vertices
// of a closed manifold: outward-wound side quads and oppositely-wound end
// caps. Its normals are face-varying so the caps remain flat without
// splitting the ring vertices and turning the topology back into an open
// surface.
//
// The polyline is already in the DIVIDED local space, so the tube is round
// here and the per-axis divisors turn it elliptical through the transform
// -- the same order the kernel folds them in.
void
_AppendCurveVolumeGuide(
    double radius, const VtVec3fArray &localCurve, const GfMatrix4d &toAsset,
    bool wire, double wireWidth,
    std::vector<RigExecVolumeGuideElement> *elements)
{
    if (!std::isfinite(radius) || radius <= 0.0 || localCurve.size() < 2) {
        return;
    }

    // Consecutive coincident points define no segment and would create a
    // strip of zero-area quads. Removing only those points preserves the
    // polyline locus (and therefore the field) while keeping the guide a
    // valid surface. A non-finite curve is not drawable at all.
    std::vector<GfVec3f> curve;
    curve.reserve(localCurve.size());
    for (const GfVec3f &p : localCurve) {
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
            !std::isfinite(p[2])) {
            return;
        }
        if (curve.empty() || (p - curve.back()).GetLength() > 1e-6f) {
            curve.push_back(p);
        }
    }
    if (curve.size() < 2) {
        return;
    }

    const size_t vertices = curve.size();
    std::vector<GfVec3f> segmentTangents(vertices - 1);
    for (size_t k = 0; k + 1 < vertices; ++k) {
        segmentTangents[k] = curve[k + 1] - curve[k];
        segmentTangents[k].Normalize();
    }

    // Rotation-minimizing per-vertex frames. Projecting the preceding U
    // axis onto the new tangent plane transports it continuously around a
    // bend instead of independently choosing a world axis at every knot,
    // which can flip a mesh strip by 180 degrees.
    std::vector<GfVec3f> tangents(vertices), ringU(vertices), ringV(vertices);
    for (size_t k = 0; k < vertices; ++k) {
        if (k == 0) {
            tangents[k] = segmentTangents.front();
        } else if (k + 1 == vertices) {
            tangents[k] = segmentTangents.back();
        } else {
            tangents[k] = segmentTangents[k - 1] + segmentTangents[k];
            if (tangents[k].GetLength() < 1e-6f) {
                // A 180-degree turn has no unique bisector. The outgoing
                // segment is at least a valid ring plane.
                tangents[k] = segmentTangents[k];
            }
            tangents[k].Normalize();
        }
        GfVec3f u = k == 0
            ? _PerpendicularTo(tangents[k])
            : ringU[k - 1] -
                  GfDot(ringU[k - 1], tangents[k]) * tangents[k];
        if (u.GetLength() < 1e-6f) {
            u = _PerpendicularTo(tangents[k]);
        } else {
            u.Normalize();
        }
        if (k > 0 && GfDot(u, ringU[k - 1]) < 0.0f) {
            u = -u;
        }
        GfVec3f v = GfCross(tangents[k], u);
        v.Normalize();
        ringU[k] = u;
        ringV[k] = v;
    }

    RigExecVolumeGuideElement element;
    element.xform = toAsset;
    if (wire) {
        element.primType = HdPrimTypeTokens->basisCurves;
        element.wireWidth = wireWidth;
        for (size_t k = 0; k < vertices; ++k) {
            _AppendVolumeRing(curve[k], float(radius) * ringU[k],
                              float(radius) * ringV[k],
                              _kVolumeTubeSegments, &element.points);
            element.counts.push_back(_kVolumeTubeSegments + 1);
        }
        // Four rails, at the quarter turns of each ring.
        for (int quarter = 0; quarter < 4; ++quarter) {
            const double theta = _kTwoPi * double(quarter) / 4.0;
            const float c = float(std::cos(theta));
            const float s = float(std::sin(theta));
            for (size_t k = 0; k < vertices; ++k) {
                element.points.push_back(
                    curve[k] + float(radius) *
                        (c * ringU[k] + s * ringV[k]));
            }
            element.counts.push_back(int(vertices));
        }
        elements->push_back(std::move(element));
        return;
    }

    element.primType = HdPrimTypeTokens->mesh;
    element.normalsInterpolation = TfToken("faceVarying");
    element.points.reserve(vertices * _kVolumeTubeSegments);
    for (size_t k = 0; k < vertices; ++k) {
        for (int j = 0; j < _kVolumeTubeSegments; ++j) {
            const double theta = _kTwoPi * double(j) /
                                 double(_kVolumeTubeSegments);
            const GfVec3f radial =
                float(std::cos(theta)) * ringU[k] +
                float(std::sin(theta)) * ringV[k];
            element.points.push_back(curve[k] + float(radius) * radial);
        }
    }

    // Append one polygon plus a flat, outward normal at every face corner.
    // Face-varying normals keep the octagonal side facets honest and leave
    // a hard edge where each cap meets the tube.
    auto appendFace = [&element](const std::vector<int> &face) {
        element.counts.push_back(static_cast<int>(face.size()));
        element.indices.insert(
            element.indices.end(), face.begin(), face.end());
        GfVec3f normal(0.0f);
        const GfVec3f origin = element.points[face.front()];
        for (size_t i = 1; i + 1 < face.size(); ++i) {
            normal += GfCross(element.points[face[i]] - origin,
                              element.points[face[i + 1]] - origin);
        }
        if (normal.GetLength() > 1e-9f) {
            normal.Normalize();
        }
        for (size_t i = 0; i < face.size(); ++i) {
            element.normals.push_back(normal);
        }
    };

    // Side quads. With V = tangent x U, increasing the ring index and then
    // advancing along the curve winds the face outward.
    for (size_t k = 0; k + 1 < vertices; ++k) {
        for (int j = 0; j < _kVolumeTubeSegments; ++j) {
            const int next = (j + 1) % _kVolumeTubeSegments;
            const int a = int(k * _kVolumeTubeSegments) + j;
            const int b = int(k * _kVolumeTubeSegments) + next;
            const int c = int((k + 1) * _kVolumeTubeSegments) + next;
            const int d = int((k + 1) * _kVolumeTubeSegments) + j;
            appendFace({a, b, c, d});
        }
    }

    // The start cap runs opposite the ring; the end cap follows it. Their
    // boundary edges therefore oppose the adjacent side edges, making the
    // complete face set a consistently oriented closed manifold.
    std::vector<int> startCap, endCap;
    startCap.reserve(_kVolumeTubeSegments);
    endCap.reserve(_kVolumeTubeSegments);
    for (int j = 0; j < _kVolumeTubeSegments; ++j) {
        startCap.push_back(_kVolumeTubeSegments - 1 - j);
        endCap.push_back(int((vertices - 1) * _kVolumeTubeSegments) + j);
    }
    appendFace(startCap);
    appendFace(endCap);
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
    , _frameCache(std::make_shared<RigExecFrameCache>())
{
    // No overlay is selected yet, so nothing consumes the per-point influence
    // field. SetWeightOverlay turns it back on the moment one is.
    _evaluator->SetPublishWeightFields(false);
    // Viewport profiling: the evaluator's own phases plus the imaging
    // layer's (see EvaluateAndPublishResult and the registry), dumped with
    // RigExecImaging_WriteProfileSummary. Off by default; recording changes
    // no evaluated value.
    if (TfGetenvBool("RIGEXEC_IMAGING_PROFILE", false)) {
        _evaluator->SetProfilingEnabled(true);
    }
}

bool
RigExecImagingBridge::Compile(std::vector<std::string> *errors)
{
    InvalidateGuideCaches();
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
    // Flattening stamps resetXformStack=true on every output, so the results
    // index cannot recover authored reset boundaries from Hydra. Capture
    // them while the bridge still owns the source stage, under driven roots.
    std::set<SdfPath> scanned;
    for (const auto &[providerPath, matrix] : pose.providerXforms) {
        bool covered = false;
        for (const auto &root : scanned) covered = covered || providerPath.HasPrefix(root);
        if (covered) continue;
        const auto root = _stage->GetPrimAtPath(providerPath);
        if (!root) continue;
        scanned.insert(providerPath);
        for (const auto &prim : UsdPrimRange(root)) {
            const UsdGeomXformable xform(prim);
            if (xform && xform.GetResetXformStack()) snapshot->xformResetPaths.insert(prim.GetPath());
        }
    }
}

void
RigExecImagingBridge::InvalidateGuideCaches()
{
    _guideInputs.clear();
    _jointChildren.clear();
    _jointChildrenValid = false;
    _controlSpaceJoints.clear();
}

bool
RigExecImagingBridge::_IsControlSpaceJoint(const SdfPath &path) const
{
    auto found = _controlSpaceJoints.find(path);
    if (found == _controlSpaceJoints.end()) {
        bool nested = false;
        // Any control above it through a chain of controls and joints: a
        // pivot can nest under another pivot (a follow above a compression).
        static const TfToken kControl("RigExecControl");
        static const TfToken kJoint("RigExecJoint");
        if (const UsdPrim prim = _stage->GetPrimAtPath(path)) {
            for (UsdPrim up = prim.GetParent(); up; up = up.GetParent()) {
                const TfToken &type = up.GetTypeName();
                if (type == kControl) {
                    nested = true;
                    break;
                }
                if (type != kJoint) {
                    break;
                }
            }
        }
        found = _controlSpaceJoints.emplace(path, nested).first;
    }
    return found->second;
}

void
RigExecImagingBridge::_SyncGuideCaches() const
{
    // Every stage edit the evaluator saw -- through the registry or not;
    // a bridge driven directly (tests, headless tools) gets no registry
    // notice forwarding, and must not publish stale styling for it.
    const uint64_t serial = _evaluator->GetStageEditSerial();
    if (serial != _guideCacheSerial) {
        _guideInputs.clear();
        _jointChildren.clear();
        _jointChildrenValid = false;
        _controlSpaceJoints.clear();
        _guideCacheSerial = serial;
    }
}

namespace {

// Can this attribute's value differ between two generations with no stage
// notice in between? Connected (a dial drives it) or time-varying.
bool
_GuideAttrIsLive(const UsdAttribute &attribute)
{
    return attribute &&
           (attribute.HasAuthoredConnections() ||
            attribute.ValueMightBeTimeVarying());
}

}  // namespace

RigExecImagingBridge::_GuideInputs &
RigExecImagingBridge::_GuideInputsFor(const SdfPath &path) const
{
    auto found = _guideInputs.find(path);
    if (found == _guideInputs.end()) {
        found = _guideInputs.emplace(path, _GuideInputs()).first;
        found->second.prim = _stage->GetPrimAtPath(path);
    }
    return found->second;
}

double
RigExecImagingBridge::_GuideRadius(_GuideInputs &inputs,
                                   UsdTimeCode time) const
{
    // Schema default, so an unauthored joint keeps Hydra's own fallback
    // radius and every existing rig looks exactly as it did.
    if (!inputs.prim) {
        return 1.0;
    }
    if (!inputs.radiusReady || inputs.radiusLive) {
        double radius = 1.0;
        const UsdAttribute a =
            inputs.prim.GetAttribute(TfToken("guide:radius"));
        if (a) {
            a.Get(&radius, time);
        }
        inputs.radius = radius;
        inputs.radiusLive = _GuideAttrIsLive(a);
        inputs.radiusReady = true;
    }
    return inputs.radius;
}

void
RigExecImagingBridge::_ReadGuideStyleCached(
    _GuideInputs &inputs, const RigExecRigPose &pose,
    RigExecPublishedPrim *published) const
{
    const UsdPrim &prim = inputs.prim;
    if (!prim) {
        return;
    }
    const UsdTimeCode time = pose.time;
    if (!inputs.styleReady) {
        inputs.styleReady = true;
        inputs.purpose = _ReadGuidePurpose(prim);
        inputs.colorAttr = prim.GetAttribute(TfToken("guide:displayColor"));
        inputs.colorLive = _GuideAttrIsLive(inputs.colorAttr);
        if (inputs.colorAttr && !inputs.colorLive) {
            inputs.hasColor = inputs.colorAttr.Get(&inputs.color, time);
        }
        inputs.opacityAttr =
            prim.GetAttribute(TfToken("guide:displayOpacity"));
        inputs.opacityLive = _GuideAttrIsLive(inputs.opacityAttr);
        if (inputs.opacityAttr && !inputs.opacityLive) {
            inputs.hasOpacity =
                inputs.opacityAttr.Get(&inputs.opacity, time);
        }
    }
    published->guidePurpose = inputs.purpose;
    if (inputs.colorAttr) {
        if (inputs.colorLive) {
            VtValue held;
            if (_ReadConnectedValue(inputs.colorAttr, pose, &held) &&
                held.IsHolding<GfVec3f>()) {
                published->guideColor = held.UncheckedGet<GfVec3f>();
            } else {
                inputs.colorAttr.Get(&published->guideColor, time);
            }
        } else if (inputs.hasColor) {
            published->guideColor = inputs.color;
        }
    }
    if (inputs.opacityAttr) {
        if (inputs.opacityLive) {
            // The live half is _ReadGuideStyle's opacity rule verbatim
            // (connected dial, invert, floor): only the static half is new.
            RigExecPublishedPrim scratch;
            _ReadGuideStyle(prim, pose, &scratch);
            published->guideOpacity = scratch.guideOpacity;
        } else if (inputs.hasOpacity) {
            published->guideOpacity = inputs.opacity;
        }
    }
}

const std::map<SdfPath, std::vector<SdfPath>> &
RigExecImagingBridge::_JointChildren(const RigExecRigPose &pose) const
{
    // Namespace nesting is joint hierarchy, which only a recompile can
    // change -- so the map is rebuilt when the binding epoch or the joint
    // count moves, not on every generation.
    const size_t key = _evaluator->GetBindingEpochDigest() ^
                       (pose.jointFramesFinal.size() * 0x9E3779B97F4A7C15ull);
    if (_jointChildrenValid && key == _jointChildrenKey) {
        return _jointChildren;
    }
    _jointChildren.clear();
    for (const auto &[childPath, childFrame] : pose.jointFramesFinal) {
        for (SdfPath ancestor = childPath.GetParentPath();
             !ancestor.IsEmpty() && ancestor != _rigPath;
             ancestor = ancestor.GetParentPath()) {
            if (pose.jointFramesFinal.count(ancestor)) {
                _jointChildren[ancestor].push_back(childPath);
                break;
            }
        }
    }
    _jointChildrenKey = key;
    _jointChildrenValid = true;
    return _jointChildren;
}

void
RigExecImagingBridge::_FillGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    _SyncGuideCaches();
    // Guide frames are asset-space; the consumer needs to know which prim
    // that space is anchored to in order to place them.
    snapshot->assetRoot = _rigPath.GetParentPath();

    // Namespace nesting is joint hierarchy. Map every joint to its nearest
    // joint ancestor; scopes or other grouping prims between them do not
    // interrupt the same NamespaceAncestor relationship evaluation uses.
    // Cached per binding epoch (see _JointChildren).
    const std::map<SdfPath, std::vector<SdfPath>> &jointChildren =
        _JointChildren(pose);

    for (const auto &[jointPath, frame] : pose.jointFramesFinal) {
        _GuideInputs &inputs = _GuideInputsFor(jointPath);
        const double authoredRadius = _GuideRadius(inputs, pose.time);
        RigExecPublishedPrim &published = snapshot->prims[jointPath];
        bool any = false;
        const auto children = jointChildren.find(jointPath);
        if (children != jointChildren.end()) {
            for (const SdfPath &childPath : children->second) {
                const auto child = pose.jointFramesFinal.find(childPath);
                if (child == pose.jointFramesFinal.end()) {
                    continue;
                }
                GfMatrix4d linkMatrix(1.0);
                double linkLength = 0.0;
                if (!_JointLinkFrame(frame, child->second, &linkMatrix,
                                     &linkLength)) {
                    continue;
                }
                RigExecPointFrame linkFrame;
                static const std::array<GfVec3d, 4> identity = {
                    GfVec3d(0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
                    GfVec3d(0, 0, 1)};
                for (size_t i = 0; i < identity.size(); ++i) {
                    linkFrame.points[i] = linkMatrix.Transform(identity[i]);
                }
                any = _AppendGuideFrame(
                          linkFrame, linkLength, authoredRadius,
                          /* useAimFallback = */ false,
                          /* drawSphere = */ !any, &published) || any;
            }
        }
        // A leaf, or a joint whose children are all coincident/invalid, still
        // owns its selectable origin sphere.
        if (!any) {
            any = _AppendGuideFrame(
                frame, 0.0, authoredRadius,
                /* useAimFallback = */ false,
                /* drawSphere = */ true, &published);
        }
        if (any) {
            published.assetRoot = _rigPath.GetParentPath();
            published.hasGuides = true;
            _ReadGuideStyleCached(inputs, pose, &published);
        }
    }
    for (const auto &[solverPath, frames] : pose.solverFrames) {
        RigExecPublishedPrim &published = snapshot->prims[solverPath];
        // An element has no prim of its own, but the SOLVER does, and it
        // is already where the guide colour and opacity are read from --
        // so the radius comes off it too, exactly as a joint's does. The
        // schema default is 1.0, so a rig that authors nothing draws what
        // it always drew.
        _GuideInputs &solverInputs = _GuideInputsFor(solverPath);
        const double authoredRadius = _GuideRadius(solverInputs, pose.time);
        bool any = false;
        for (const RigExecPointFrame &frame : frames) {
            any = _AppendGuideFrame(frame, 0.0, authoredRadius,
                                    /* useAimFallback = */ true,
                                    /* drawSphere = */ true,
                                    &published) ||
                  any;
        }
        if (any) {
            published.assetRoot = _rigPath.GetParentPath();
            published.hasGuides = true;
            _ReadGuideStyleCached(solverInputs, pose, &published);
        } else if (!published.hasPoints && !published.hasNormals &&
                   !published.hasExtent && !published.hasXform) {
            snapshot->prims.erase(solverPath);
        }
    }
}

// Controls draw one synthesized shape each at their posed frame
// (spec §10.3 extension): shape, draw mode, and a positive per-axis draw
// multiplier are authored on the control. Evaluated frame-axis magnitudes
// multiply that authored size while the results scene index places the shape
// with the rigidized frame.
void
RigExecImagingBridge::_FillControlGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    _SyncGuideCaches();
    // Set here as well as in _FillGuides: control guide frames are
    // asset-space too, and a rig can publish these and no joint guides at
    // all (every joint frame degenerate, say), in which case this is the
    // only place the anchor gets recorded.
    snapshot->assetRoot = _rigPath.GetParentPath();

    for (const auto &[controlPath, frame] : pose.controlFrames) {
        GfMatrix4d evaluated(1.0);
        static const RigExecPointFrame identity;
        if (frame.IsValid() && RigExecPointsToMatrix(identity.points, frame.points, &evaluated)) {
            auto &published = snapshot->prims[controlPath];
            published.assetRoot = _rigPath.GetParentPath();
            published.hasControlFrame = true;
            published.controlFrame = evaluated;
        }
        GfMatrix4d placement(1.0);
        GfVec3d evaluatedScale(1.0);
        if (!_RigidGuideMatrix(frame, &placement, &evaluatedScale)) {
            continue;
        }
        _GuideInputs &inputs = _GuideInputsFor(controlPath);
        const UsdPrim &prim = inputs.prim;
        if (!inputs.controlReady || inputs.controlLive) {
            // The schema fallbacks, restated. Normally GetAttribute resolves
            // them for us, but a stage composed without the codeless schema
            // plugin registered has no fallback to find, and an empty shape
            // token names no shape at all -- so a rig would silently stop
            // drawing control guides rather than draw the documented default.
            inputs.shape = TfToken("circle");
            inputs.drawMode = TfToken("wire");
            inputs.scale = GfVec3d(1.0, 1.0, 1.0);
            inputs.wireWidth = 0.05;
            inputs.offset = GfVec3d(0.0);
            bool live = false;
            if (prim) {
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("guide:shape"))) {
                    a.Get(&inputs.shape, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("guide:drawMode"))) {
                    a.Get(&inputs.drawMode, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                static const TfToken scaleAttrs[3] = {
                    TfToken("guide:scaleX"), TfToken("guide:scaleY"),
                    TfToken("guide:scaleZ")};
                for (int axis = 0; axis < 3; ++axis) {
                    if (UsdAttribute a =
                            prim.GetAttribute(scaleAttrs[axis])) {
                        a.Get(&inputs.scale[axis], pose.time);
                        live = live || _GuideAttrIsLive(a);
                    }
                }
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("guide:wireWidth"))) {
                    a.Get(&inputs.wireWidth, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("guide:offset"))) {
                    a.Get(&inputs.offset, pose.time);
                    live = live || _GuideAttrIsLive(a);
                }
            }
            inputs.controlLive = live;
            inputs.controlReady = true;
        }
        TfToken shape = inputs.shape;
        TfToken drawMode = inputs.drawMode;
        GfVec3d authoredScale = inputs.scale;
        double wireWidth = inputs.wireWidth;
        // Unlike the scale, a non-positive width is NOT a reason to draw
        // nothing: it selects the hairline fallback, which is what wire
        // guides did before the width existed. Non-finite collapses to the
        // same thing rather than reaching the curves as a NaN.
        if (!std::isfinite(wireWidth) || wireWidth <= 0.0) {
            wireWidth = 0.0;
        }
        // A non-finite or non-positive AUTHORED multiplier on ANY axis draws
        // nothing,
        // mirroring the joint guide:radius rule: a flattened shape is
        // invisible from most angles and degenerate from the rest, so
        // publishing it would only cost the viewer geometry it cannot
        // meaningfully see.
        GfVec3d effectiveScale(1.0);
        bool scaleOk = true;
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(authoredScale[axis]) ||
                authoredScale[axis] <= 0.0) {
                scaleOk = false;
                break;
            }
            effectiveScale[axis] =
                authoredScale[axis] * evaluatedScale[axis];
            if (!std::isfinite(effectiveScale[axis]) ||
                effectiveScale[axis] <= 0.0) {
                scaleOk = false;
                break;
            }
        }
        if (!scaleOk) {
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[controlPath];
        published.assetRoot = _rigPath.GetParentPath();
        published.hasControlGuide = true;
        // guide:offset moves the drawn shape in the control's local frame,
        // carried by the evaluated scale; the pivot stays where it is.
        GfMatrix4d guideFrame = placement;
        if (inputs.offset != GfVec3d(0.0) &&
            std::isfinite(inputs.offset[0]) &&
            std::isfinite(inputs.offset[1]) &&
            std::isfinite(inputs.offset[2])) {
            const GfVec3d local(inputs.offset[0] * evaluatedScale[0],
                                inputs.offset[1] * evaluatedScale[1],
                                inputs.offset[2] * evaluatedScale[2]);
            guideFrame = GfMatrix4d(1.0).SetTranslate(local) * placement;
        }
        published.controlGuideFrame = guideFrame;
        published.controlGuideShape = shape;
        published.controlGuideDrawMode = drawMode;
        published.controlGuideScale = effectiveScale;
        published.controlGuideWireWidth = wireWidth;
        _ReadGuideStyleCached(inputs, pose, &published);
    }

    // The evaluated frames of the PIVOTS nested in a control hierarchy
    // (a RigExecJoint whose parent is a control: a follow, compression or
    // drag pivot). A constraint writes those frames, so a viewer composing
    // them from authored values gets the rest pose, and the controls
    // nested under them would have no trustworthy parent frame. Published
    // beside the controls' own frames so a manipulator can read the
    // evaluator's answer for them. Only these joints: every other joint's
    // frame is not something a control composes against.
    static const RigExecPointFrame identityFrame;
    for (const auto &[jointPath, frame] : pose.jointFramesFinal) {
        if (!frame.IsValid() || !_IsControlSpaceJoint(jointPath)) {
            continue;
        }
        GfMatrix4d evaluated(1.0);
        if (RigExecPointsToMatrix(identityFrame.points, frame.points,
                                  &evaluated)) {
            auto &published = snapshot->prims[jointPath];
            published.assetRoot = _rigPath.GetParentPath();
            published.hasControlFrame = true;
            published.controlFrame = evaluated;
        }
    }
}


// Every placed influence volume draws its falloffMin and falloffMax
// iso-surfaces (spec §4.1 volumetric extension, drawn side).
//
// Driven by pose.weightFrames rather than by a second stage traversal: the
// evaluator discovers every placed volume beneath the rig and resolves its
// frame whether or not a mover consumes the field. That makes the schema's
// selectable/framable guide contract useful during authoring, before the
// volume is wired, while keeping drawing on the exact same placement path the
// field uses once it is consumed.
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
                // Property movers can drive volume dimensions. The field
                // consumes these evaluated values, so its iso-surfaces must
                // use the same values instead of the authored source.
                const auto moved = pose.movedProperties.find(a.GetPath());
                if (moved != pose.movedProperties.end() &&
                    moved->second.IsHolding<float>()) {
                    return moved->second.UncheckedGet<float>();
                }
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
                                            wire, wireWidth, &elements);
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
        _ReadGuideStyle(prim, pose, &published);
    }
}

void
RigExecImagingBridge::_FillCurvenetGuides(
    const RigExecRigPose &pose, RigExecImagingSnapshot *snapshot) const
{
    const SdfPath assetPath = _rigPath.GetParentPath();
    const UsdPrim asset = _stage->GetPrimAtPath(assetPath);
    if (!asset) return;
    UsdGeomXformCache xforms(pose.time);
    for (const UsdPrim &prim : UsdPrimRange(asset)) {
        if (prim.GetTypeName() != "RigExecCurvenet") continue;
        const UsdAttribute pointsAttr = prim.GetAttribute(TfToken("points"));
        VtVec3fArray points;
        const auto moved = pose.movedProperties.find(pointsAttr.GetPath());
        if (moved != pose.movedProperties.end() &&
            moved->second.IsHolding<VtVec3fArray>()) {
            points = moved->second.UncheckedGet<VtVec3fArray>();
        } else if (!pointsAttr.Get(&points, pose.time)) {
            continue;
        }
        VtIntArray indices;
        if (!prim.GetAttribute(TfToken("rigExec:splineIndices"))
                 .Get(&indices, pose.time) || indices.empty()) continue;
        TfToken basis("bezier");
        prim.GetAttribute(TfToken("rigExec:basis")).Get(&basis, pose.time);
        if (basis != "bezier" && basis != "catmullRom") continue;
        int density = 5;
        prim.GetAttribute(TfToken("rigExec:samplesPerSpline"))
            .Get(&density, pose.time);
        if (density < 1) continue;
        const std::vector<GfVec3f> pool(points.begin(), points.end());
        RigExecCurvenetTopology topology;
        std::string error;
        if (!RigExecBuildCurvenetTopology(
                std::vector<int>(indices.begin(), indices.end()), pool.size(),
                basis == "bezier" ? RigExecCurvenetBasis::Bezier
                                   : RigExecCurvenetBasis::CatmullRom,
                pool, nullptr, &topology, &error)) continue;
        const RigExecCurvenetSampling sampled = RigExecSampleCurvenet(
            topology, pool, std::vector<int>(topology.GetSplineCount(), density));
        RigExecVolumeGuideElement element;
        element.primType = HdPrimTypeTokens->basisCurves;
        element.wireWidth = 0.02;
        for (size_t c = 0; c < sampled.GetCurveCount(); ++c) {
            const int begin = sampled.curveBegin[c];
            const int count = sampled.GetCurveSampleCount(c);
            if (count < 2) continue;
            element.counts.push_back(count + (topology.curves[c].closed ? 1 : 0));
            for (int i = 0; i < count; ++i)
                element.points.push_back(GfVec3f(sampled.positions[begin + i]));
            if (topology.curves[c].closed)
                element.points.push_back(GfVec3f(sampled.positions[begin]));
        }
        if (element.points.empty()) continue;
        RigExecPublishedPrim &published = snapshot->prims[prim.GetPath()];
        published.assetRoot = assetPath;
        published.hasVolumeGuides = true;
        published.volumeGuides = {std::move(element)};
        published.volumeGuideAnchor = prim.GetPath();
        // Bounds are exposed in asset space; drawing resolves the native
        // anchor through the same driven-transform composition as its Points.
        GfMatrix4d toAsset(1.0);
        for (UsdPrim ancestor = prim; ancestor && ancestor != asset;
             ancestor = ancestor.GetParent()) {
            bool reset = false;
            GfMatrix4d local = xforms.GetLocalTransformation(ancestor, &reset);
            const auto revised = pose.providerXforms.find(ancestor.GetPath());
            if (revised != pose.providerXforms.end()) local = revised->second;
            toAsset = toAsset * local;
            if (reset) {
                toAsset = toAsset * xforms.GetLocalToWorldTransform(asset).GetInverse();
                break;
            }
        }
        published.volumeGuideAnchorToAsset = toAsset;
        published.guideColor = GfVec3f(0.15f, 0.8f, 0.45f);
        published.guideOpacity = 1.0f;
        _ReadGuideStyle(prim, pose, &published);
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

// The test-only fence probe (see RigExecSetPublishFenceProbeForTesting):
// null in production, set only while no publish is in flight.
static std::function<void()> _sPublishFenceProbe;

bool
RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig)
{
    return RigExecPublishBackgroundCompletion(
        cache, key, time, pose, generation, scheduler, rig,
        RigExecWarmFenceToken(0), nullptr, 0, nullptr);
}

bool
RigExecPublishBackgroundCompletion(
    const std::shared_ptr<RigExecFrameCache> &cache,
    const RigExecFrameCacheKey &key, UsdTimeCode time,
    const RigExecRigPose &pose, RigExecFrameGeneration generation,
    const RigExecBackgroundScheduler *scheduler, const SdfPath &rig,
    RigExecWarmFenceToken fenceToken,
    std::shared_ptr<const void> retained, size_t retainedBytes,
    const RigExecEntryProvenance *provenance)
{
    if (!cache || !pose.valid) {
        return false;
    }
    if (!scheduler) {
        return cache->Publish(key, time, pose, retainedBytes, retained,
                              provenance);
    }
    // Fence-check and cache insert atomically under the fence mutex (plan
    // 3.3): the fenced clear holds the same mutex across bump + purge +
    // clear + reset, so no clear can land between the check and the store.
    // Lock order fence, then scheduler, then cache shards.
    std::lock_guard<std::mutex> fenceLock(scheduler->FenceMutex());
    if (!scheduler->IsWarmRequestCurrent(rig, generation, time,
                                         fenceToken)) {
        return false;
    }
    if (_sPublishFenceProbe) {
        _sPublishFenceProbe();
    }
    return cache->Publish(key, time, pose, retainedBytes, retained,
                          provenance);
}

void
RigExecSetPublishFenceProbeForTesting(std::function<void()> probe)
{
    _sPublishFenceProbe = std::move(probe);
}

void
RigExecImagingBridge::SetWarmFrameIndex(
    std::shared_ptr<RigExecWarmFrameIndex> index)
{
    _warmIndex = std::move(index);
}

void
RigExecImagingBridge::ClearFrameCache()
{
    if (_frameCache) {
        _frameCache->Clear();
    }
    _taskListMemo.Clear();
    _lastPublishedEpoch.store(0, std::memory_order_relaxed);
    _freshDigests.clear();
    _freshEpochValid = false;
    _cacheModeValid = false;
}

RigExecFreshProof::RigExecFreshProof(
    uint64_t digestIn, uint64_t unfoldedIn,
    const RigExecFrameInputs *inputs,
    const std::vector<RigExecValueOverride> *overrides)
    : digest(digestIn)
    , unfolded(unfoldedIn)
{
    // The dependency set the digest covers: first-win sample paths (the
    // digest's own granularity) plus standing override identities.
    // Stage seeds fold no path string, so they contribute none: a notice
    // that moves only seeds leaves the proof standing, and the digest
    // compare -- which does fold them -- misses honestly on the visit.
    std::set<std::string> ids;
    if (inputs) {
        for (const RigExecSampledInput &sample : inputs->values) {
            if (!sample.path.IsEmpty()) {
                ids.insert(sample.path.GetString());
            }
        }
    }
    if (overrides) {
        for (const RigExecValueOverride &o : *overrides) {
            ids.insert(RigExecControlIdForOverride(o));
        }
    }
    paths.assign(ids.begin(), ids.end());
}

void
RigExecImagingBridge::NoteWarmingEnqueued(
    UsdTimeCode time, uint64_t controlDigest, uint64_t unfoldedDigest,
    const RigExecFrameInputs &inputs)
{
    if (!_frameCache ||
        RigExecFrameCacheModeFromEnvironment() ==
            RigExecFrameCacheMode::Off) {
        return;
    }
    // Scope check, as on the lookup and memoization paths: the evaluation
    // mode or the epoch may have moved since the last proof was recorded.
    const uint64_t epoch = RigExecFrameCacheEpochDigest(*_evaluator);
    const RigExecEvaluationMode mode = _evaluator->GetEvaluationMode();
    if (!_freshEpochValid || _freshEpoch != epoch ||
        !_cacheModeValid || _cacheMode != mode) {
        _freshDigests.clear();
        _freshEpoch = epoch;
        _freshEpochValid = true;
        _cacheMode = mode;
        _cacheModeValid = true;
    }
    if (_freshDigests.size() >= _kFreshDigestCap) {
        _freshDigests.clear();
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        controlDigest, unfoldedDigest, &inputs, &_interactiveOverrides);
}

const RigExecOutputAffectedIndex *
RigExecImagingBridge::_SyncAffectedIndex()
{
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        _affectedIndex.reset();
        _affectedValid = false;
        return nullptr;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    const uint64_t epoch = RigExecFrameCacheEpochDigest(*_evaluator);
    // The program object test matters as much as the epoch: a rebuild can
    // answer a colliding epoch for wholly new clustering, and an index
    // built for the old shape would retire against the wrong cones.
    // (The epoch digest folds the build count, so a rebuild moves it in
    // practice; the pointer pins the shape regardless.)
    if (_affectedValid && _affectedIndex && _affectedEpoch == epoch &&
        _affectedProgram == program) {
        return _affectedIndex.get();
    }
    _affectedIndex.reset(new RigExecOutputAffectedIndex());
    _affectedIndex->Build(impl, epoch);
    _affectedEpoch = epoch;
    _affectedProgram = program;
    _affectedValid = true;
    // A rebuild drops every admission with the old index: the standing
    // overrides re-admit against the new shape, so a drag straddling a
    // rebuild keeps its exact seeds.
    AdmitOverrideControls(_interactiveOverrides);
    return _affectedIndex.get();
}

const RigExecOutputAffectedIndex *
RigExecImagingBridge::GetAffectedIndex()
{
    return _SyncAffectedIndex();
}

void
RigExecImagingBridge::AdmitOverrideControls(
    const std::vector<RigExecValueOverride> &overrides)
{
    if (overrides.empty()) {
        return;
    }
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        return;
    }
    const RigExecOutputAffectedIndex *synced = _SyncAffectedIndex();
    if (!synced || synced->Empty()) {
        // No oracle, no admission: the overrides stay foreign and every
        // plan over them stays conservative, as before.
        return;
    }
    const RigExecBakedProgramImpl &impl = program->GetStepGraph();
    for (const RigExecValueOverride &o : overrides) {
        const RigExecControlId id = RigExecControlIdForOverride(o);
        if (_affectedIndex->IsKnownControl(id)) {
            continue;
        }
        const std::vector<int> seeds = RigExecOverrideSeeds(impl, o);
        if (seeds.empty()) {
            // Places nowhere (folded, exec-typed, routed, or unknown):
            // left foreign, never admitted seedless. A seedless
            // admission would claim the override reaches nothing,
            // which only the placement below can prove.
            continue;
        }
        _affectedIndex->MapControl(id, seeds);
    }
}

size_t
RigExecImagingBridge::RetireProofsForControls(
    const std::vector<RigExecControlId> &controls)
{
    if (_freshDigests.empty() || controls.empty()) {
        return 0;
    }
    const std::set<std::string> doomed(controls.begin(), controls.end());
    size_t retired = 0;
    for (auto it = _freshDigests.begin(); it != _freshDigests.end();) {
        bool hit = false;
        for (const std::string &path : it->second.paths) {
            if (doomed.find(path) != doomed.end()) {
                hit = true;
                break;
            }
        }
        if (hit) {
            it = _freshDigests.erase(it);
            ++retired;
        } else {
            ++it;
        }
    }
    return retired;
}

bool
RigExecImagingBridge::RepointProof(UsdTimeCode time, uint64_t newDigest)
{
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const auto found =
        _freshDigests.find({time.IsDefault(), stamped});
    if (found == _freshDigests.end()) {
        return false;
    }
    // The digest names the new namespace; the unfolded half and the paths
    // are untouched -- the sampled values did not move, only the constants
    // they fold beside did. A second carry re-folds the same unfolded half
    // against the newer constants.
    found->second.digest = newDigest;
    return true;
}

bool
RigExecImagingBridge::GetFreshProof(UsdTimeCode time,
                                   RigExecFreshProof *proof) const
{
    if (!proof) {
        return false;
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const auto found =
        _freshDigests.find({time.IsDefault(), stamped});
    if (found == _freshDigests.end()) {
        return false;
    }
    *proof = found->second;
    return true;
}

bool
RigExecImagingBridge::NoteCaptureIndex(
    const UsdNotice::ObjectsChanged &notice)
{
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program || !_frameCache) {
        return false;
    }
    const uint64_t epoch = RigExecFrameCacheEpochDigest(*_evaluator);
    const bool hit = RigExecNoteCaptureIndex(*_frameCache, _taskListMemo,
                                             epoch, *program, notice);
    if (hit && _freshEpochValid && _freshEpoch == epoch) {
        // The proofs name evicted entries: drop them so the next visit
        // misses without paying a sample for the proof check.
        _freshDigests.clear();
    }
    // Epoch drift since the last memoization (an in-place avar patch, a
    // rebuild, a structural edit): the old half's frames are unreachable by
    // construction, so evict them eagerly rather than waiting for LRU.
    const uint64_t last =
        _lastPublishedEpoch.load(std::memory_order_relaxed);
    if (last != epoch) {
        _frameCache->EvictEpoch(last);
        _taskListMemo.InvalidateEpoch(last);
        if (_freshEpochValid && _freshEpoch == last) {
            _freshDigests.clear();
        }
    }
    return hit;
}

RigExecFrameCacheStats
RigExecImagingBridge::GetFrameCacheStats() const
{
    if (!_frameCache) {
        return RigExecFrameCacheStats();
    }
    return _frameCache->Stats();
}

bool
RigExecImagingBridge::_ComputeCacheKey(
    UsdTimeCode time, RigExecFrameCacheKey *key) const
{
    if (!key || !_frameCache) {
        return false;
    }
    RigExecFrameInputs inputs;
    if (!RigExecSampleFrameInputs(
            *_evaluator, time, _interactiveOverrides, &inputs)) {
        // D7: a rig with no baked program (a bake refusal, or an explicitly
        // dynamic rig) cannot sample a vector, so it memoizes on time plus
        // the evaluator's stage-edit serial plus the standing overrides.
        // The serial advances on every stage notice, so an edit moves the
        // digest at every frame by construction -- the D1 rule for a rig
        // that cannot re-sample -- on every path, registry-managed or bare
        // bridge. A rig that HAS a program but cannot sample (a prologue
        // read with no hook) stays live-only, as today.
        if (_evaluator->GetBakedProgram() != nullptr) {
            return false;
        }
        if (!RigExecRefusalControlDigestible(_interactiveOverrides)) {
            return false;
        }
        key->epochDigest = RigExecFrameCacheEpochDigest(*_evaluator);
        key->controlDigest = RigExecRefusalControlDigest(
            time, _evaluator->GetStageEditSerial(), _interactiveOverrides);
        return true;
    }
    if (!RigExecControlStateDigestible(inputs, _interactiveOverrides)) {
        // A held type the digest cannot fold exactly: two different frames
        // could share one key, which is a plausible wrong pose.
        return false;
    }
    // The sampler folds the overrides into the vector AND the two-argument
    // digest folds the explicit list: both halves are deterministic in the
    // same inputs, so lookup and memoization agree, and a drag always
    // digests apart from the authored frame it started from.
    key->epochDigest = RigExecFrameCacheEpochDigest(*_evaluator);
    // Epoch constants are not per-frame inputs, so the sampled digest
    // cannot see a value patch: the constant digest folds beside it (plan
    // D3), opening a new key namespace while the epoch half stands.
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    const uint64_t constants =
        program ? RigExecEpochConstantDigest(*program) : 0;
    key->controlDigest = RigExecControlStateDigestWithConstants(
        inputs, _interactiveOverrides, constants);
    return true;
}

void
RigExecImagingBridge::_ServeCachedPose(
    UsdTimeCode time, const RigExecRigPose &pose,
    const RigExecFrameCacheKey &key, PublishResult *result)
{
    _frameCache->NoteProvenanceAlias(key, time);
    RigExecRigPose cached = pose;
    const bool verify = RigExecFrameCacheVerifyRequested();
    if (verify) {
        // Shadow mode: prove the hit against a live evaluation, the same
        // judge as BakedWithParityCheck. The shadow IS an evaluator pull,
        // so whatever it says, this publication counts as a live one.
        RigExecRigPose live;
        {
            RigExecProfileScope scope(MutableProfiler(), "Imaging.Evaluate",
                                      "imaging");
            live = _evaluator->Evaluate(time);
        }
        const RigExecShadowVerdict verdict =
            RigExecVerifyHitWithLive(cached, [&live]() { return live; });
        if (!verdict.match && verdict.mismatches > 0) {
            TF_WARN("rigExec: frame-cache hit at %s mismatches live "
                    "evaluation; serving live and repairing the entry:\n%s",
                    time.IsDefault()
                        ? "default"
                        : std::to_string(time.GetValue()).c_str(),
                    verdict.report.c_str());
            _PublishPoseSnapshot(time, verdict.poseToServe, result);
            _MemoizeLiveResult(time, verdict.poseToServe);
            result->cacheHit = false;
            return;
        }
        // A match serves the cached pose (proven identical); an unverified
        // shadow (live failed) serves it too, advisory-only. Either way the
        // evaluator ran, so cacheHit stays false.
        cached.time = time;
        _PublishPoseSnapshot(time, cached, result);
        result->cacheHit = false;
        return;
    }
    // Guide styling reads pose.time; the stored pose was evaluated for an
    // earlier visit, so it is re-stamped for the requested frame before
    // anything reads it.
    cached.time = time;
    _PublishPoseSnapshot(time, cached, result);
    result->cacheHit = true;
}

bool
RigExecImagingBridge::CarryEntry(UsdTimeCode time,
                                const RigExecFrameCacheKey &oldKey,
                                uint64_t newConstants)
{
    if (!_frameCache) {
        return false;
    }
    RigExecEntryProvenance provenance;
    if (!_frameCache->LookupProvenance(oldKey, &provenance) ||
        provenance.unfoldedControlDigest == 0) {
        return false;
    }
    RigExecFrameCacheKey newKey;
    newKey.epochDigest = oldKey.epochDigest;
    newKey.controlDigest = RigExecFoldConstantDigest(
        provenance.unfoldedControlDigest, newConstants);
    if (newKey.controlDigest == oldKey.controlDigest) {
        // No namespace move for this entry -- the caller stands it.
        return false;
    }
    RigExecRigPose pose;
    std::shared_ptr<const void> retained;
    size_t bytes = 0;
    if (!_frameCache->Lookup(oldKey, &pose, &retained, &bytes) ||
        !pose.valid || !retained) {
        return false;
    }
    // The D5 rewrite sequence: publish under the new key, evict the old,
    // re-record the completion -- which REPLACES the row, so no Drop (a
    // Drop erases by (epoch, time) and would eat the new row). The old key
    // is unreachable by construction past the namespace move (no fresh
    // sample folds the old constants), so the evict frees without robbing
    // aliases: a frame whose values match serves the carried entry.
    RigExecEntryProvenance carried = provenance;
    if (time.IsNumeric()) {
        carried.aliasTimes.clear();
        carried.aliasTimes.push_back(time.GetValue());
    }
    if (!_frameCache->Publish(newKey, time, pose, bytes, retained,
                              &carried)) {
        return false;
    }
    _frameCache->Evict(oldKey);
    if (_warmIndex && time.IsNumeric()) {
        _warmIndex->NoteCompleted(
            _rigPath, time.GetValue(), newKey,
            _warmIndex->CurrentGeneration(_rigPath));
    }
    RepointProof(time, newKey.controlDigest);
    return true;
}

bool
RigExecImagingBridge::_TrySparseServe(
    UsdTimeCode time, const RigExecFrameInputs &inputs,
    const RigExecFrameCacheKey &key, uint64_t unfolded,
    PublishResult *result)
{
    if (!result || !_frameCache || !_warmIndex || !time.IsNumeric()) {
        return false;
    }
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (!program) {
        return false;
    }
    const double stamped = time.GetValue();
    RigExecFrameCacheKey baseKey{0, 0};
    if (!_warmIndex->FindKey(_rigPath, stamped, &baseKey)) {
        return false;
    }
    // Same key, retired proof: the entry stands under the fresh digest
    // (a no-op edit retired the proof without moving values). Serve it
    // and heal the proof -- no re-key, no pulls.
    if (baseKey == key) {
        RigExecRigPose cached;
        bool hit = false;
        {
            RigExecProfileScope scope(MutableProfiler(),
                                      "Imaging.FrameCacheLookup", "imaging");
            hit = _frameCache->Lookup(key, &cached);
        }
        MutableProfiler()->RecordCacheLookup(hit, stamped);
        if (!hit || !cached.valid) {
            return false;
        }
        _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
            key.controlDigest, unfolded, &inputs, &_interactiveOverrides);
        _warmIndex->NoteCompleted(
            _rigPath, stamped, key,
            _warmIndex->CurrentGeneration(_rigPath));
        _ServeCachedPose(time, cached, key, result);
        return true;
    }
    RigExecRigPose base;
    std::shared_ptr<const void> handle;
    bool baseHit = false;
    {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Imaging.FrameCacheLookup", "imaging");
        baseHit = _frameCache->Lookup(baseKey, &base, &handle);
    }
    if (!baseHit || !base.valid || !handle) {
        // No retained base (evicted, or pose-only): the frame evaluates
        // live. Records the miss: the store was consulted.
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecRetainedFrameState *retained =
        static_cast<const RigExecRetainedFrameState *>(handle.get());
    // The constant gate: sampled values exclude avar constants by design,
    // so a value comparison alone cannot see a constant patch. A moved
    // namespace plans nothing here -- the eager carry covers clean
    // entries, and anything else re-warms.
    if (retained->epochDigest != key.epochDigest ||
        retained->constantDigest != RigExecEpochConstantDigest(*program)) {
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecOutputAffectedIndex *index = _SyncAffectedIndex();
    if (!index || index->Empty()) {
        // An empty index answers empty sets, not all-clusters: planning
        // against it would Hit everything and serve stale poses. Live.
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    const RigExecSparsePlan plan = RigExecPlanSparseReuse(
        *index, &_taskListMemo, *retained, key.epochDigest, inputs,
        _interactiveOverrides, nullptr);
    if (plan.verdict != RigExecSparseVerdict::Hit) {
        // The UI thread serves Hit only: a Partial re-runs on a worker
        // through the lane-c partial job (plan 2.2), against the retained
        // slots both publish paths now capture, and a Miss has no base.
        // Both live-eval here.
        MutableProfiler()->RecordCacheLookup(false, stamped);
        return false;
    }
    // Proven identical with zero cluster work: re-key under the fresh
    // digest and serve with zero pulls. The old entry is deliberately
    // NOT evicted: another frame's values may still digest to it (an
    // alias serves it directly), and LRU drains what nobody serves.
    RigExecEntryProvenance provenance;
    const bool haveProvenance =
        _frameCache->LookupProvenance(baseKey, &provenance);
    if (haveProvenance) {
        provenance.unfoldedControlDigest = unfolded;
    }
    RigExecRigPose served = base;
    if (_frameCache->Publish(key, time, served,
                             retained->RetainedBytes(), handle,
                             haveProvenance ? &provenance : nullptr)) {
        _warmIndex->NoteCompleted(
            _rigPath, stamped, key,
            _warmIndex->CurrentGeneration(_rigPath));
    }
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        key.controlDigest, unfolded, &inputs, &_interactiveOverrides);
    MutableProfiler()->RecordCacheLookup(true, stamped);
    _ServeCachedPose(time, served, key, result);
    return true;
}

bool
RigExecImagingBridge::_TryPublishCachedResult(
    UsdTimeCode time, PublishResult *result)
{
    if (!result || !_frameCache) {
        return false;
    }
    // Scope check: a new epoch or a new evaluation mode retires every
    // freshness proof. Cached entries stay for LRU (their digests decide
    // reachability); proofs name the old scope's inputs and must not be
    // consulted. No proof can exist yet, so this is a miss either way.
    const uint64_t epoch = RigExecFrameCacheEpochDigest(*_evaluator);
    const RigExecEvaluationMode mode = _evaluator->GetEvaluationMode();
    if (!_freshEpochValid || _freshEpoch != epoch ||
        !_cacheModeValid || _cacheMode != mode) {
        _freshDigests.clear();
        _freshEpoch = epoch;
        _freshEpochValid = true;
        _cacheMode = mode;
        _cacheModeValid = true;
        return false;
    }
    // Servable completions skip sampling entirely: the index row names the
    // exact key, and the retirement paths (generation tags, epoch tags,
    // path-scoped dirt, eviction) keep a clean row exact, so a store hit
    // under it IS the proof compare. Sampling first here cost every hit
    // the full vector (~49 ms on the stack for a ~10 ms evaluation --
    // hits ran 4.5x slower than cache-off). A row miss, a store miss
    // under the row (concurrent eviction), or a D7 rig falls through to
    // the proof/sparse paths below, unchanged.
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    const RigExecBakedProgram *program = _evaluator->GetBakedProgram();
    if (program && _warmIndex && time.IsNumeric()) {
        RigExecFrameCacheKey cachedKey{0, 0};
        if (_warmIndex->FindCachedKey(_rigPath, stamped,
                                      _warmIndex->CurrentGeneration(_rigPath),
                                      epoch, &cachedKey)) {
            RigExecRigPose cached;
            const bool hit = [&] {
                RigExecProfileScope scope(
                    MutableProfiler(), "Imaging.FrameCacheLookup", "imaging");
                return _frameCache->Lookup(cachedKey, &cached);
            }();
            if (hit && cached.valid) {
                MutableProfiler()->RecordCacheLookup(true, stamped);
                _ServeCachedPose(time, cached, cachedKey, result);
                return true;
            }
            // Store miss under a live row: unrecorded, so the lane keeps
            // one record per SetTime -- the fallthrough records its own.
        }
    }
    // The proof is consulted by time BEFORE anything is sampled: a time with
    // no proof cannot hit whatever its digest says, and a cold frame is the
    // common case on a first scrub. Sampling and digesting it here would
    // make every miss pay the full vector twice (once here, once in the
    // memoization after the live run), which is exactly the cold-frame
    // latency the cache-off path never pays. A time with no proof but WITH
    // a retained base still samples: the sparse plan below may serve it
    // with zero pulls, which is worth one sample.
    const auto proof = _freshDigests.find({time.IsDefault(), stamped});
    const bool haveProof = proof != _freshDigests.end();
    RigExecFrameCacheKey baseKey{0, 0};
    const bool haveBase = _warmIndex && time.IsNumeric() &&
        _warmIndex->FindKey(_rigPath, stamped, &baseKey);
    if (!haveProof && !haveBase) {
        return false;
    }
    if (!program) {
        // D7: refusal keys name no sampled vector, so nothing plans. The
        // historical path, unchanged.
        if (!haveProof) {
            return false;
        }
        RigExecFrameCacheKey key;
        if (!_ComputeCacheKey(time, &key)) {
            return false;
        }
        if (proof->second.digest != key.controlDigest) {
            return false;
        }
        RigExecRigPose cached;
        const bool hit = [&] {
            RigExecProfileScope scope(MutableProfiler(),
                                      "Imaging.FrameCacheLookup", "imaging");
            return _frameCache->Lookup(key, &cached);
        }();
        MutableProfiler()->RecordCacheLookup(
            hit, time.IsDefault() ? 0.0 : time.GetValue());
        if (!hit) {
            return false;
        }
        _ServeCachedPose(time, cached, key, result);
        return true;
    }
    // The baked path samples once for the proof compare AND the sparse
    // plan: a mismatch falls through to planning on the same vector.
    // Reached only past the servable-completion fast path above (a dirty
    // or unrecorded time), so this scope also counts fast-path skips by
    // its absence.
    RigExecFrameInputs inputs;
    uint64_t unfolded = 0;
    {
        RigExecProfileScope sampleScope(
            MutableProfiler(), "Imaging.LookupSampleDigest", "imaging");
        if (!RigExecSampleFrameInputs(*_evaluator, time,
                                      _interactiveOverrides, &inputs) ||
            !RigExecControlStateDigestible(inputs, _interactiveOverrides)) {
            return false;
        }
        unfolded = RigExecControlStateDigest(inputs, _interactiveOverrides);
    }
    RigExecFrameCacheKey key;
    key.epochDigest = epoch;
    key.controlDigest = RigExecFoldConstantDigest(
        unfolded, RigExecEpochConstantDigest(*program));
    if (haveProof && proof->second.digest == key.controlDigest) {
        // The frameCache lane records actual store consultations only: a
        // lookup that never reaches the store (no proof, unsampleable,
        // stale scope) records nothing.
        RigExecRigPose cached;
        const bool hit = [&] {
            RigExecProfileScope scope(MutableProfiler(),
                                      "Imaging.FrameCacheLookup", "imaging");
            return _frameCache->Lookup(key, &cached);
        }();
        MutableProfiler()->RecordCacheLookup(
            hit, time.IsDefault() ? 0.0 : time.GetValue());
        if (hit) {
            _ServeCachedPose(time, cached, key, result);
            return true;
        }
        // Proven but evicted: the sparse base may still serve (a re-keyed
        // entry under a moved digest), so fall through to the plan.
    }
    // No proof the pre-evaluation sample is fresh -- never evaluated here,
    // retired by an edit, or the inputs moved since. Chain-resolved inputs
    // sampled for a time the evaluator has not run come from the standing
    // resolved state, so an unproven sample must never serve directly; the
    // sparse plan compares it by VALUE against this time's own retained
    // base instead, which a stale sample cannot forge into a Hit.
    if (!haveBase) {
        return false;
    }
    return _TrySparseServe(time, inputs, key, unfolded, result);
}

void
RigExecImagingBridge::_MemoizeLiveResult(
    UsdTimeCode time, const RigExecRigPose &pose)
{
    if (!_frameCache || !pose.valid) {
        return;
    }
    // Scope check, as on the lookup path: the evaluation above may have
    // settled a new epoch.
    const uint64_t epoch = RigExecFrameCacheEpochDigest(*_evaluator);
    const RigExecEvaluationMode mode = _evaluator->GetEvaluationMode();
    if (!_freshEpochValid || _freshEpoch != epoch ||
        !_cacheModeValid || _cacheMode != mode) {
        _freshDigests.clear();
        _freshEpoch = epoch;
        _freshEpochValid = true;
        _cacheMode = mode;
        _cacheModeValid = true;
    }
    // Sampled AFTER the evaluation, so the resolved state is fresh at exactly
    // this time: the digest below is the TRUE digest, and recording it as
    // this time's proof is what later lookups prove freshness against.
    // Sampled once, here, for the key AND the retained state:
    // _ComputeCacheKey samples internally, which would sample twice.
    RigExecFrameCacheKey key;
    RigExecFrameInputs memoInputs;
    uint64_t memoUnfolded = 0;
    bool haveInputs = false;
    {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Imaging.MemoizeSampleDigest", "imaging");
        if (_evaluator->GetBakedProgram()) {
            if (!RigExecSampleFrameInputs(*_evaluator, time,
                                          _interactiveOverrides,
                                          &memoInputs) ||
                !RigExecControlStateDigestible(memoInputs,
                                               _interactiveOverrides)) {
                return;
            }
            key.epochDigest = RigExecFrameCacheEpochDigest(*_evaluator);
            const uint64_t unfolded =
                RigExecControlStateDigest(memoInputs,
                                          _interactiveOverrides);
            key.controlDigest = RigExecFoldConstantDigest(
                unfolded,
                RigExecEpochConstantDigest(
                    *_evaluator->GetBakedProgram()));
            memoUnfolded = unfolded;
            haveInputs = true;
        } else if (!_ComputeCacheKey(time, &key)) {
            // D7: no program, no inputs -- the refusal key, pose-only.
            return;
        }
    }
    bool stored = false;
    {
        RigExecProfileScope scope(MutableProfiler(),
                                  "Imaging.FrameCachePublish", "imaging");
        if (haveInputs) {
            // Retained-state publish (plan 2.0): the source snapshot and
            // the dependency record ride with the pose, so a later edit
            // retires and re-runs only what touched it. The wider half --
            // the just-evaluated live program's pose-domain slots, which a
            // partial cone re-runs against -- is captured beside the source
            // snapshot.
            const RigExecBakedProgramImpl &impl =
                _evaluator->GetBakedProgram()->GetStepGraph();
            RigExecRetainedFrameState retainedState =
                RigExecCaptureRetainedState(memoInputs, _interactiveOverrides,
                                            key.epochDigest,
                                            impl.clustering.clusters.size(),
                                            RigExecEpochConstantDigest(
                                                *_evaluator->GetBakedProgram()));
            auto retained =
                std::make_shared<RigExecRetainedFrameState>(
                    std::move(retainedState));
            std::shared_ptr<const void> slots;
            size_t slotBytes = 0;
            if (RigExecCapturePartialSlots(impl, &slots, &slotBytes) &&
                slots && slotBytes > 0) {
                retained->slots = std::move(slots);
                retained->slotBytes = slotBytes;
            }
            const size_t retainedBytes = retained->RetainedBytes();
            RigExecEntryProvenance provenance =
                RigExecFullEvalProvenance(impl, time);
            provenance.unfoldedControlDigest = memoUnfolded;
            stored = _frameCache->Publish(key, time, pose, retainedBytes,
                                          retained, &provenance);
        } else {
            stored = _frameCache->Publish(key, time, pose);
        }
    }
    // A memoized entry is a cached frame: record the completion the strip
    // reads, under the generation this evaluation ran in. Default times
    // are not strip frames and record nothing.
    if (stored && _warmIndex && time.IsNumeric()) {
        _warmIndex->NoteCompleted(
            _rigPath, time.GetValue(), key,
            _warmIndex->CurrentGeneration(_rigPath));
    }
    // The epoch half frames were last stored under, for the notice path's
    // drift check -- kept even when the publish declines, so a later notice
    // evicts the half it would have named.
    _lastPublishedEpoch.store(key.epochDigest, std::memory_order_relaxed);
    if (_freshDigests.size() >= _kFreshDigestCap) {
        _freshDigests.clear();
    }
    const double stamped = time.IsDefault() ? 0.0 : time.GetValue();
    _freshDigests[{time.IsDefault(), stamped}] = RigExecFreshProof(
        key.controlDigest, memoUnfolded,
        haveInputs ? &memoInputs : nullptr, &_interactiveOverrides);
    // No override admission here: memoization runs only when no overrides
    // stand (the drag bypass), so the list is always empty. Admission
    // lives on the warming path, which samples under standing overrides.
}

void
RigExecImagingBridge::_PublishPoseSnapshot(
    UsdTimeCode time, const RigExecRigPose &pose, PublishResult *result)
{
    // Build the complete immutable generation from the exact native
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
            // Not Hydra data -- but a scalar here is a rig output a
            // tool may want (a blend weight, a pose-interpolator
            // result), and recovering it otherwise costs a whole
            // second evaluation of the rig. Recorded beside the
            // generation that produced it; see
            // RigExecImagingSnapshot::movedFloats.
            if (value.IsHolding<float>()) {
                snapshot->movedFloats[propertyPath] =
                    value.UncheckedGet<float>();
            } else if (value.IsHolding<double>()) {
                snapshot->movedFloats[propertyPath] =
                    static_cast<float>(value.UncheckedGet<double>());
            }
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
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.Guides",
                                  "imaging");
        _FillProviderXforms(pose, snapshot.get());
        _FillGuides(pose, snapshot.get());
        _FillControlGuides(pose, snapshot.get());
        _FillVolumeGuides(pose, snapshot.get());
        _FillCurvenetGuides(pose, snapshot.get());
        _FillWeightOverlay(pose, snapshot.get());
    }

    // A structural recompile publishes a replacement binding epoch
    // before value notices (spec §10.4).
    const size_t epochDigest = _evaluator->GetBindingEpochDigest();
    if (epochDigest != _publishedEpochDigest) {
        auto epoch = std::make_shared<
            RigExecBindingResolvingSceneIndex::BindingEpoch>();
        epoch->id = epochDigest;
        for (const auto &[primPath, published] : snapshot->prims) {
            epoch->publishedPrims.insert(primPath);
        }
        result->epoch = std::move(epoch);
        _publishedEpochDigest = epochDigest;
    }

    // Atomic snapshot swap; the caller sends the coalesced precise
    // dirtied notices from the notice owner (spec §8.2, §10.4).
    {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.StorePublish",
                                  "imaging");
        result->dirtied = _store->Publish(std::move(snapshot));
    }
    result->ok = true;
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
    RigExecProfileScope wholeScope(MutableProfiler(),
                                   "Imaging.EvaluateAndPublish", "imaging");
    // 0. Frame-cache fast path: a hit publishes the stored pose through the
    // same atomic snapshot without evaluating. Off, empty, unsampleable, and
    // unproven all fall through to the live path, which is the historical
    // behavior plus one memoization. Off disables both halves: the cache
    // is neither read nor written -- as does an active drag (below).
    const RigExecFrameCacheMode cacheMode =
        RigExecFrameCacheModeFromEnvironment();
    // Interactive bypass: every drag tick carries unique overrides, so a
    // lookup would always miss after paying a full sample+digest, and
    // memoization would store single-use entries no scrub can reach --
    // pure overhead on the hottest path (D5: the playhead is always live).
    // While overrides stand the cache is neither read nor written, exactly
    // as when off; proofs and entries are left untouched, so releasing the
    // drag resumes hitting the authored frames.
    const bool useCache = cacheMode != RigExecFrameCacheMode::Off &&
                          _interactiveOverrides.empty();
    if (useCache && _TryPublishCachedResult(time, &result)) {
        return result;
    }
    // 1. Evaluation always completes before publication (spec §8.2).
    const RigExecRigPose pose = [&] {
        RigExecProfileScope scope(MutableProfiler(), "Imaging.Evaluate",
                                  "imaging");
        return _evaluator->Evaluate(time);
    }();
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

    // 2-4. Snapshot publication, shared with the hit path so the two cannot
    // publish differently; then memoization into the frame cache (reads
    // on and no drag -- see step 0).
    _PublishPoseSnapshot(time, pose, &result);
    if (useCache) {
        _MemoizeLiveResult(time, pose);
    }
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
    for (size_t i = 0; i < shutterOffsets.size(); ++i) {
        if (!std::isfinite(shutterOffsets[i]) ||
            (i && shutterOffsets[i] <= shutterOffsets[i - 1])) {
            if (whyNot) *whyNot = "shutter offsets must be finite and strictly increasing";
            return false;
        }
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
        baseTime.IsDefault() || !std::isfinite(baseTime.GetValue())) {
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
    for (size_t i = 0; i < shutterOffsets.size(); ++i) {
        const UsdTimeCode sampleTime(
            baseTime.GetValue() + double(shutterOffsets[i]));
        const RigExecRigPose pose = _evaluator->Evaluate(sampleTime);
        if (!pose.valid) {
            return result;  // an incomplete sample set never publishes
        }
        for (const auto &[path, matrix] : pose.providerXforms) {
            const auto base = pose.providerBaseXforms.find(path);
            if (base == pose.providerBaseXforms.end()) return result;
            RigExecPublishedPrim &published = snapshot->prims[path];
            published.sampleOffsets = shutterOffsets;
            published.xformSamples.push_back(matrix);
            published.xformBaseSamples.push_back(base->second);
        }
        if (i == baseIndex) {
            // Guides are single-sampled at the base offset.
            _FillProviderXforms(pose, snapshot.get());
            _FillGuides(pose, snapshot.get());
            _FillControlGuides(pose, snapshot.get());
            _FillVolumeGuides(pose, snapshot.get());
            _FillCurvenetGuides(pose, snapshot.get());
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
                // Not Hydra data -- but a scalar here is a rig output a
                // tool may want (a blend weight, a pose-interpolator
                // result), and recovering it otherwise costs a whole
                // second evaluation of the rig. Recorded beside the
                // generation that produced it; see
                // RigExecImagingSnapshot::movedFloats.
                if (value.IsHolding<float>()) {
                    snapshot->movedFloats[propertyPath] =
                        value.UncheckedGet<float>();
                } else if (value.IsHolding<double>()) {
                    snapshot->movedFloats[propertyPath] =
                        static_cast<float>(value.UncheckedGet<double>());
                }
                continue;
            }
            RigExecPublishedPrim &published = snapshot->prims[primPath];
            published.sampleOffsets = shutterOffsets;
            const auto &array = value.UncheckedGet<VtVec3fArray>();
            if (property == "points") published.pointsSamples.push_back(array);
            else if (property == "normals") published.normalsSamples.push_back(array);
            else if (array.size() == 2) {
                published.extentMinSamples.push_back(GfVec3d(array[0]));
                published.extentMaxSamples.push_back(GfVec3d(array[1]));
            } else return result;
            if (i == baseIndex) {
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
    for (const auto &[primPath, published] : snapshot->prims) {
        for (size_t count : {published.pointsSamples.size(),
                             published.normalsSamples.size(),
                             published.xformSamples.size(),
                             published.xformBaseSamples.size(),
                             published.extentMinSamples.size(),
                             published.extentMaxSamples.size()}) {
            if (count && count != shutterOffsets.size()) return result;
        }
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

// Method reference: Moller and Trumbore (1997), https://en-cg-web.coecis.cornell.edu/pubs/1997/MT97.html
//
// The surface projector: a frame that rides a deforming surface, measured by
// casting a ray at the rest points and following the material point it hits
// onto the posed points. Header-only and USD-free so the USD evaluators
// (instantiated with Gf types) and the zero-USD runtime (Rr types) run one
// definition; every vector and matrix operation used here exists with the
// same arithmetic in both families.
//
// Row-vector convention throughout: a frame's rows are its X, Y and Z axes
// and its translation, and world = local * parent.
#ifndef RIGEXEC_MATH_SURFACE_PROJECTOR_KERNEL_H
#define RIGEXEC_MATH_SURFACE_PROJECTOR_KERNEL_H

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace rigExec {

/// Where a ray meets a mesh, kept as the MATERIAL point it landed on: the
/// corners of the fan triangle that won and the barycentric weights inside
/// it. A hit is topology, not position, so the same hit evaluates on any
/// point set that shares the mesh's topology.
struct RigExecSurfaceHit {
    int a = -1, b = -1, c = -1;  ///< corners of the winning triangle
    double u = 0.0, v = 0.0;     ///< weights of b and c; a carries 1 - u - v
    double distance = 0.0;       ///< along the normalized ray, from origin
};

namespace surfaceProjectorDetail {

template <class Vec3d, class Vec3f>
Vec3d ToDouble(const Vec3f &p)
{
    return Vec3d(double(p[0]), double(p[1]), double(p[2]));
}

template <class Mat4, class Vec3d>
Vec3d Row(const Mat4 &m, int row)
{
    return Vec3d(m[row][0], m[row][1], m[row][2]);
}

template <class Mat4, class Vec3d>
void SetRow(Mat4 *m, int row, const Vec3d &v)
{
    (*m)[row][0] = v[0];
    (*m)[row][1] = v[1];
    (*m)[row][2] = v[2];
}

template <class Mat4, class Vec3d>
double MeanRowLength(const Mat4 &m)
{
    double sum = 0.0;
    for (int i = 0; i < 3; ++i) {
        sum += Row<Mat4, Vec3d>(m, i).GetLength();
    }
    return sum;
}

inline std::string Format3(const char *label, double x, double y, double z)
{
    char text[96];
    std::snprintf(text, sizeof(text), "%s(%.4f %.4f %.4f)", label, x, y, z);
    return text;
}

}  // namespace surfaceProjectorDetail

/// Cast a ray at a mesh: Moller-Trumbore against each face fan-triangulated
/// about its first corner, two-sided, nearest hit strictly in front of the
/// origin. False, leaving \p hit untouched, when nothing is met, the
/// direction is degenerate, or the topology is invalid.
template <class Vec3d, class Vec3f>
bool RigExecRaycastSurfaceT(const std::vector<Vec3f> &points,
                            const std::vector<int> &faceVertexCounts,
                            const std::vector<int> &faceVertexIndices,
                            const Vec3d &origin, const Vec3d &direction,
                            RigExecSurfaceHit *hit)
{
    using surfaceProjectorDetail::ToDouble;
    if (!hit || points.empty() || faceVertexCounts.empty()) return false;
    Vec3d dir = direction;
    const double dirLength = dir.GetLength();
    if (!(dirLength > 0) || !std::isfinite(dirLength)) return false;
    dir /= dirLength;
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(origin[axis])) return false;
    }

    // Nearest forward hit, tracked with its barycentric coordinates so the
    // normal can be blended at exactly the point that won.
    double nearest = std::numeric_limits<double>::infinity();
    int hitA = -1, hitB = -1, hitC = -1;
    double hitU = 0, hitV = 0;

    size_t offset = 0;
    for (int count : faceVertexCounts) {
        if (count < 3 ||
            static_cast<size_t>(count) > faceVertexIndices.size() - offset) {
            return false;
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceVertexIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= points.size()) {
                return false;
            }
        }
        const int origin0 = faceVertexIndices[offset];
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int ia = origin0;
            const int ib = faceVertexIndices[offset + corner];
            const int ic = faceVertexIndices[offset + corner + 1];
            const Vec3d a = ToDouble<Vec3d>(points[ia]);
            const Vec3d b = ToDouble<Vec3d>(points[ib]);
            const Vec3d c = ToDouble<Vec3d>(points[ic]);
            const Vec3d e1 = b - a, e2 = c - a;
            const Vec3d p = dir ^ e2;
            const double det = e1 * p;
            // Two-sided: the ray starts INSIDE the eyeball, so the face it
            // leaves through is back-facing to it.
            if (std::abs(det) < 1e-16) continue;
            const double inv = 1.0 / det;
            const Vec3d t = origin - a;
            const double u = (t * p) * inv;
            if (u < 0.0 || u > 1.0) continue;
            const Vec3d q = t ^ e1;
            const double v = (dir * q) * inv;
            if (v < 0.0 || u + v > 1.0) continue;
            const double distance = (e2 * q) * inv;
            if (distance <= 1e-9 || distance >= nearest) continue;
            nearest = distance;
            hitA = ia; hitB = ib; hitC = ic;
            hitU = u; hitV = v;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceVertexIndices.size()) return false;
    if (hitA < 0 || !std::isfinite(nearest)) return false;

    hit->a = hitA; hit->b = hitB; hit->c = hitC;
    hit->u = hitU; hit->v = hitV;
    hit->distance = nearest;
    return true;
}

/// The surface frame at a hit, evaluated on the given points: the hit's
/// barycentric position as the translation and the blended vertex normal as
/// +Z, with \p upHint fixing the roll. \p normalsOf computes the vertex
/// normals of a point set; it is the evaluator's own normals kernel, so the
/// frame agrees with the normals that evaluator publishes.
template <class Mat4, class Vec3d, class Vec3f, class Normals>
bool RigExecSurfaceFrameAtHitT(const std::vector<Vec3f> &points,
                               const std::vector<int> &faceVertexCounts,
                               const std::vector<int> &faceVertexIndices,
                               const RigExecSurfaceHit &hit,
                               const Vec3d &upHint, Normals &&normalsOf,
                               Mat4 *frame)
{
    using surfaceProjectorDetail::SetRow;
    using surfaceProjectorDetail::ToDouble;
    if (!frame || points.empty()) return false;
    const int corners[3] = {hit.a, hit.b, hit.c};
    for (int index : corners) {
        if (index < 0 || static_cast<size_t>(index) >= points.size()) {
            return false;
        }
    }
    if (!std::isfinite(hit.u) || !std::isfinite(hit.v)) return false;

    // The vertex normals validate the topology on the way: an invalid mesh
    // yields none, and a frame on it would be a frame on nothing.
    const std::vector<Vec3f> normals =
        normalsOf(points, faceVertexCounts, faceVertexIndices);
    if (normals.size() != points.size()) return false;

    const double w = 1.0 - hit.u - hit.v;
    const Vec3d a = ToDouble<Vec3d>(points[hit.a]);
    const Vec3d b = ToDouble<Vec3d>(points[hit.b]);
    const Vec3d c = ToDouble<Vec3d>(points[hit.c]);
    Vec3d normal = w * ToDouble<Vec3d>(normals[hit.a]) +
                   hit.u * ToDouble<Vec3d>(normals[hit.b]) +
                   hit.v * ToDouble<Vec3d>(normals[hit.c]);
    double length = normal.GetLength();
    if (!(length > 0) || !std::isfinite(length)) {
        // A degenerate blend still has the triangle itself to fall back on.
        normal = (b - a) ^ (c - a);
        length = normal.GetLength();
        if (!(length > 0) || !std::isfinite(length)) return false;
    }
    normal /= length;

    // Roll. The hint is only a hint: parallel to the normal it says
    // nothing, and the least-aligned principal axis is the stable choice.
    Vec3d up = upHint - (upHint * normal) * normal;
    if (up.GetLengthSq() < 1e-12) {
        int axis = 0;
        for (int i = 1; i < 3; ++i) {
            if (std::abs(normal[i]) < std::abs(normal[axis])) axis = i;
        }
        Vec3d fallback(0.0, 0.0, 0.0);
        fallback[axis] = 1.0;
        up = fallback - (fallback * normal) * normal;
        if (up.GetLengthSq() < 1e-12) return false;
    }
    up.Normalize();
    const Vec3d side = up ^ normal;

    // The hit's own position on THESE points: the material point, wherever
    // this point set has carried it.
    const Vec3d position = w * a + hit.u * b + hit.v * c;
    frame->SetIdentity();
    SetRow(frame, 0, side);
    SetRow(frame, 1, up);
    SetRow(frame, 2, normal);
    SetRow(frame, 3, position);
    return true;
}

/// Everything a RigExecSurfaceProjector reads, resolved by the evaluator.
///
/// The provider frames share the evaluator's ASSET space: `Base` before pose constraints,
/// `Final` after them. A flag says whether the relationship named a provider
/// that resolved; an unnamed one leaves its frames unread.
template <class Mat4, class Vec3d>
struct RigExecSurfaceProjectorInputs {
    /// rigExec:rayOrigin / rayDirection / rayUp, in the mesh's own space.
    Vec3d rayOrigin;
    Vec3d rayDirection;
    Vec3d rayUp;
    /// rigExec:shaderOffset: the projector's own placement.
    Mat4 shaderOffset;
    /// rigExec:projectionMode == "reproject".
    bool reproject = false;
    /// rigExec:sources[0]: the frame the projector is anchored to.
    bool hasSource = false;
    Mat4 sourceBase, sourceFinal;
    /// rigExec:sourceSpace: the source's sibling space. Named but not
    /// resolved keeps the authored ray rather than the mesh-space one.
    bool sourceSpaceNamed = false;
    bool hasSourceSpace = false;
    Mat4 sourceSpaceBase, sourceSpaceFinal;
    /// rigExec:space: the provider carrying the whole rig. Named but not
    /// resolved is reported, as the rest ray then keeps the masters.
    bool spaceNamed = false;
    bool hasSpace = false;
    Mat4 spaceFinal;
    /// Asset-to-mesh transform: where a source with no sourceSpace is
    /// expressed in the mesh's space. The field retains its historical name.
    Mat4 worldToMesh;
};

/// The projector's target-local shader matrix: rigExec:shaderOffset carried by the look
/// of its source and by how the material point under its ray moved from the
/// rest points to the posed points, scaled with the rig. False when no frame
/// could be measured; \p diagnostics then says which step failed, prefixed
/// with \p who.
template <class Mat4, class Vec3d, class Vec3f, class Normals>
bool RigExecSolveSurfaceProjectorT(
    const RigExecSurfaceProjectorInputs<Mat4, Vec3d> &in,
    const std::vector<Vec3f> &basePoints,
    const std::vector<Vec3f> &posedPoints,
    const std::vector<int> &faceVertexCounts,
    const std::vector<int> &faceVertexIndices, Normals &&normalsOf,
    const std::string &who, Mat4 *shaderMatrix,
    std::vector<std::string> *diagnostics)
{
    using surfaceProjectorDetail::Format3;
    using surfaceProjectorDetail::MeanRowLength;
    using surfaceProjectorDetail::Row;
    if (basePoints.empty() || faceVertexCounts.empty() ||
        posedPoints.size() != basePoints.size()) {
        diagnostics->push_back(
            "SurfaceProjector " + who +
            ": surface has no usable points or topology");
        return false;
    }

    // The ray comes from the SOURCE frame when there is one, in the
    // surface's own space; the authored ray is the fallback.
    Vec3d rayOrigin = in.rayOrigin;
    Vec3d rayDirection = in.rayDirection;
    Vec3d rayUp = in.rayUp;
    if (in.hasSource && in.sourceSpaceNamed) {
        // The source's motion within its sibling space, applied to the
        // authored rest ray: head and face cancel, leaving the eye's own.
        if (in.hasSourceSpace) {
            const Mat4 restLocal =
                in.sourceBase * in.sourceSpaceBase.GetInverse();
            const Mat4 posedLocal =
                in.sourceFinal * in.sourceSpaceFinal.GetInverse();
            const Mat4 motion = restLocal.GetInverse() * posedLocal;
            rayOrigin = motion.Transform(in.rayOrigin);
            rayDirection = motion.TransformDir(in.rayDirection);
            rayUp = motion.TransformDir(in.rayUp);
        }
    } else if (in.hasSource) {
        const Mat4 local = in.sourceFinal * in.worldToMesh;
        rayOrigin = local.ExtractTranslation();
        rayDirection = Row<Mat4, Vec3d>(local, 2);
        rayUp = Row<Mat4, Vec3d>(local, 1);
    }

    // The rest cast gets a REST ray, built the same way from the source's
    // base frame, so each cast meets the surface it is aimed at.
    Vec3d restRayOrigin = rayOrigin;
    Vec3d restRayDirection = rayDirection;
    Vec3d restRayUp = rayUp;
    if (in.hasSource && !in.sourceSpaceNamed) {
        const Mat4 local = in.sourceBase * in.worldToMesh;
        restRayOrigin = local.ExtractTranslation();
        restRayDirection = Row<Mat4, Vec3d>(local, 2);
        restRayUp = Row<Mat4, Vec3d>(local, 1);
    }

    // The masters come back off the rest ray: the base frame composes
    // through their posed avars while the base points carry nothing. Their
    // uniform scale is put back on the shader matrix below.
    double spaceScale = 1.0;
    if (in.spaceNamed) {
        if (in.hasSpace) {
            const Mat4 back = in.spaceFinal.GetInverse();
            restRayOrigin = back.Transform(restRayOrigin);
            restRayDirection = back.TransformDir(restRayDirection);
            restRayUp = back.TransformDir(restRayUp);
            const double carried = MeanRowLength<Mat4, Vec3d>(in.spaceFinal);
            if (carried > 1e-9 && std::isfinite(carried)) {
                spaceScale = carried / 3.0;
            }
        } else {
            diagnostics->push_back(
                "SurfaceProjector " + who +
                ": rigExec:space names a prim that is not a frame provider; "
                "the rest ray keeps the masters and will miss once the rig "
                "leaves its rest");
        }
    }

    // material: one ray at the base surface, followed onto the posed one.
    // reproject: cast again at the posed surface, falling back to the
    // material point when the re-cast misses.
    RigExecSurfaceHit hit;
    Mat4 restFrame, posedFrame;
    restFrame.SetIdentity();
    posedFrame.SetIdentity();
    bool ok = RigExecRaycastSurfaceT(basePoints, faceVertexCounts,
                                     faceVertexIndices, restRayOrigin,
                                     restRayDirection, &hit) &&
              RigExecSurfaceFrameAtHitT(basePoints, faceVertexCounts,
                                        faceVertexIndices, hit, restRayUp,
                                        normalsOf, &restFrame);
    if (ok && in.reproject) {
        RigExecSurfaceHit posedHit;
        const bool reprojected =
            RigExecRaycastSurfaceT(posedPoints, faceVertexCounts,
                                   faceVertexIndices, rayOrigin, rayDirection,
                                   &posedHit) &&
            RigExecSurfaceFrameAtHitT(posedPoints, faceVertexCounts,
                                      faceVertexIndices, posedHit, rayUp,
                                      normalsOf, &posedFrame);
        if (!reprojected) {
            ok = RigExecSurfaceFrameAtHitT(posedPoints, faceVertexCounts,
                                           faceVertexIndices, hit, rayUp,
                                           normalsOf, &posedFrame);
            if (ok) {
                diagnostics->push_back(
                    "SurfaceProjector " + who +
                    ": the re-cast missed the deformed surface; falling back "
                    "to the material point");
            }
        }
    } else if (ok) {
        ok = RigExecSurfaceFrameAtHitT(posedPoints, faceVertexCounts,
                                       faceVertexIndices, hit, rayUp,
                                       normalsOf, &posedFrame);
    }
    if (!ok) {
        // Which step failed, and the ray it failed with.
        RigExecSurfaceHit probe;
        const bool castHit = RigExecRaycastSurfaceT(
            basePoints, faceVertexCounts, faceVertexIndices, restRayOrigin,
            restRayDirection, &probe);
        RigExecSurfaceHit posedProbe;
        const bool posedCast =
            !in.reproject ||
            RigExecRaycastSurfaceT(posedPoints, faceVertexCounts,
                                   faceVertexIndices, rayOrigin, rayDirection,
                                   &posedProbe);
        Mat4 scratch;
        scratch.SetIdentity();
        const bool restOk =
            castHit &&
            RigExecSurfaceFrameAtHitT(basePoints, faceVertexCounts,
                                      faceVertexIndices, probe, rayUp,
                                      normalsOf, &scratch);
        const bool posedOk =
            castHit &&
            RigExecSurfaceFrameAtHitT(posedPoints, faceVertexCounts,
                                      faceVertexIndices, probe, rayUp,
                                      normalsOf, &scratch);
        diagnostics->push_back(
            "SurfaceProjector " + who + ": ray " +
            Format3("o", rayOrigin[0], rayOrigin[1], rayOrigin[2]) + " " +
            Format3("d", rayDirection[0], rayDirection[1], rayDirection[2]) +
            " " + Format3("up", rayUp[0], rayUp[1], rayUp[2]) +
            " cast=" + (castHit ? "hit" : "MISS") +
            " recast=" + (posedCast ? "hit" : "MISS") +
            " restFrame=" + (restOk ? "ok" : "FAILED") +
            " posedFrame=" + (posedOk ? "ok" : "FAILED"));
        return false;
    }

    // How the material at the hit moved, rest * delta == posed. Orthonormal
    // frames, so delta is rigid; the rig's scale is put back below.
    const Mat4 delta = restFrame.GetInverse() * posedFrame;

    // The look: the source's own rotation, measured in the same space as
    // the ray. Material mode follows one material point, so the look
    // cancels out of delta and is put back beside it; reproject already
    // carries it.
    Mat4 look;
    look.SetIdentity();
    if (in.hasSource && !in.reproject) {
        Mat4 sourceRest = in.sourceBase;
        Mat4 sourcePosed = in.sourceFinal;
        if (in.hasSourceSpace) {
            sourceRest = sourceRest * in.sourceSpaceBase.GetInverse();
            sourcePosed = sourcePosed * in.sourceSpaceFinal.GetInverse();
        }
        look = sourceRest.GetInverse() * sourcePosed;
    }

    // The source's world scale, base to final, times the masters': the
    // scale of the head and of the world, and nothing that squashes the
    // ball itself.
    double sourceScale = 1.0;
    if (in.hasSource) {
        const double rest = MeanRowLength<Mat4, Vec3d>(in.sourceBase);
        const double posed = MeanRowLength<Mat4, Vec3d>(in.sourceFinal);
        if (rest > 1e-9 && std::isfinite(rest) && std::isfinite(posed)) {
            sourceScale = posed / rest;
        }
    }
    sourceScale *= spaceScale;
    Mat4 carried = delta;
    if (std::abs(sourceScale - 1.0) > 1e-9 && std::isfinite(sourceScale) &&
        sourceScale > 0.0) {
        // Scaled about the rest hit, so the material point still lands
        // where it ended up and only the size around it changes.
        const Vec3d restHit = restFrame.ExtractTranslation();
        Mat4 linear = delta;
        linear.SetTranslateOnly(Vec3d(0.0, 0.0, 0.0));
        const Vec3d mapped = linear.TransformDir(restHit);
        for (int i = 0; i < 3; ++i) {
            surfaceProjectorDetail::SetRow(
                &carried, i, Row<Mat4, Vec3d>(delta, i) * sourceScale);
        }
        carried.SetTranslateOnly(delta.ExtractTranslation() +
                                 (1.0 - sourceScale) * mapped);
    }
    *shaderMatrix = in.shaderOffset * look * carried;
    return true;
}

/// rigExec:shaderDialSources packed row-major into one matrix primvar, at
/// most sixteen; a non-finite dial is left at zero.
template <class Mat4>
Mat4 RigExecPackShaderDialsT(const std::vector<double> &values)
{
    Mat4 packed(0.0);
    const size_t slots = values.size() < 16 ? values.size() : 16;
    for (size_t i = 0; i < slots; ++i) {
        if (std::isfinite(values[i])) {
            packed[int(i / 4)][int(i % 4)] = values[i];
        }
    }
    return packed;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SURFACE_PROJECTOR_KERNEL_H

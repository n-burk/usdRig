// Affine frame kernels. Matrices use USD row-vector conventions.
// Pure value inputs: no stage, Exec context, plugin or evaluator dependency.
// The expressions themselves are affineFrameKernel.h's, shared with the
// zero-USD runtime; this header instantiates them over Gf.
// Numerical provenance and supported operations: docs/references.md.
#ifndef RIGEXEC_MATH_AFFINE_FRAME_KERNELS_H
#define RIGEXEC_MATH_AFFINE_FRAME_KERNELS_H
#include "affineFrameKernel.h"
#include "dualQuat.h"
#include "pxr/base/gf/matrix3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include <string>
#include <vector>
PXR_NAMESPACE_USING_DIRECTIVE
namespace rigExec {
/// The Gf math policy of RigExecAffineFrameKernel.
struct RigExecGfAffineMath {
    using Mat4 = GfMatrix4d;
    using Mat3 = GfMatrix3d;
    using Vec3d = GfVec3d;
    using Vec3f = GfVec3f;
    using Vec3i = GfVec3i;
    using Quat = GfQuatd;
    using DualQuat = RigExecDualQuat;
    static double Dot(const GfVec3d &a, const GfVec3d &b) { return GfDot(a, b); }
    static float Dot(const GfVec3f &a, const GfVec3f &b) { return GfDot(a, b); }
    static double Dot(const GfQuatd &a, const GfQuatd &b) { return GfDot(a, b); }
    static GfVec3d Cross(const GfVec3d &a, const GfVec3d &b) { return GfCross(a, b); }
    static GfVec3d CompMult(const GfVec3d &a, const GfVec3d &b) { return GfCompMult(a, b); }
    static GfVec3f ToFloat(const GfVec3d &v) { return GfVec3f(v); }
    static GfVec3d ToDouble(const GfVec3f &v) { return GfVec3d(v); }
    static GfMatrix4d Rotation(const GfVec3d &axis, double degrees)
    {
        return GfMatrix4d(GfRotation(axis, degrees), GfVec3d(0));
    }
    static GfQuatd RotationQuat(const GfMatrix3d &m) { return m.ExtractRotation().GetQuat(); }
    static GfMatrix3d FromQuat(const GfQuatd &q) { return GfMatrix3d(q); }
    static RigExecDualQuat DualQuatZero() { return RigExecDualQuat(GfQuatd(0), GfQuatd(0)); }
    static RigExecDualQuat DualQuatFromMatrix(const GfMatrix4d &m) { return RigExecDualQuatFromMatrix(m); }
    static bool DualQuatNormalize(RigExecDualQuat *dq) { return RigExecDualQuatNormalize(dq); }
    static GfMatrix4d DualQuatToMatrix(const RigExecDualQuat &dq) { return RigExecDualQuatToMatrix(dq); }
};
using RigExecAffineFrameInputs = RigExecAffineFrameInputsT<RigExecGfAffineMath>;
using RigExecGfAffineFrameKernel = RigExecAffineFrameKernel<RigExecGfAffineMath>;
/// The expressions one by one. ConstraintFrame throws std::runtime_error
/// when an Armature blend's target arrays disagree.
GfMatrix4d RigExecComputeArmatureParent(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeBoneFrame(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeSkinInfluence(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeMappedFrame(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeCopyTransforms(const RigExecAffineFrameInputs &inputs);
GfMatrix4d RigExecComputeConstraintFrame(const RigExecAffineFrameInputs &inputs);
}
#endif

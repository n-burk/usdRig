// Affine frame expressions (RigExecCopyFrame, MappedFrame, SkinInfluence,
// ArmatureParent, BoneFrame, ConstraintFrame), header-only and USD-free:
// the USD evaluators (GfMatrix4d, affineFrameKernels.h) and the zero-USD
// runtime (RrMat4d) instantiate one definition, so both run the same
// floating-point operations in the same order. Matrices use USD row-vector
// conventions. Numerical provenance and supported operations:
// docs/references.md.
//
// A math policy M names the types (Mat4, Mat3, Vec3d, Vec3f, Vec3i, Quat,
// DualQuat) and the operations whose spelling differs between Gf and the
// runtime mirror: Dot, Cross, CompMult, ToFloat/ToDouble (vector precision
// changes), Rotation (an axis/degrees rotation matrix), RotationQuat (a 3x3
// matrix's ExtractRotation().GetQuat()), FromQuat (the 3x3 rotation of a
// quaternion), and the dual quaternion conversions of dualQuat.h.
#ifndef RIGEXEC_MATH_AFFINE_FRAME_KERNEL_H
#define RIGEXEC_MATH_AFFINE_FRAME_KERNEL_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

/// An affine frame expression's inputs. Each field's default is the value
/// an absent input reads.
template <class M>
struct RigExecAffineFrameInputsT {
    using Mat4 = typename M::Mat4;
    using Vec3d = typename M::Vec3d;
    using Vec3i = typename M::Vec3i;
    Mat4 incoming{1};
    bool preserveLocation{false};
    Mat4 origin{1};
    Mat4 inverseBind{1};
    std::string operation{"COPY_LOCATION"};
    double influence{1};
    std::vector<int> targetIndices{};
    std::vector<int> objectIndices{};
    std::vector<Mat4> targetBinds{};
    std::vector<double> targetWeights{};
    Vec3d pivot{0, 0, 0};
    bool dualQuaternion{false};
    bool currentPivot{false};
    std::string ownerSpace{"WORLD"};
    std::string targetSpace{"WORLD"};
    int axisMask{7};
    int invertMask{0};
    bool offset{false};
    bool uniformScale{false};
    bool scaleAdd{false};
    double power{1};
    std::string rotationMix{"REPLACE"};
    bool removeTargetShear{false};
    std::string mapFrom{"LOCATION"};
    Vec3d mapFromMin{0, 0, 0};
    Vec3d mapFromMax{1, 1, 1};
    Vec3d mapToMin{0, 0, 0};
    Vec3d mapToMax{1, 1, 1};
    Vec3i mapAxes{0, 1, 2};
    bool mapExtrapolate{false};
    std::string mapMix{"ADD"};
    std::string trackAxis{"TRACK_Y"};
    std::string keepAxis{"PLANE_X"};
    std::string volume{"NO_VOLUME"};
    double restLength{1};
    double bulge{1};
    double bulgeMin{0};
    double bulgeMax{1};
    double bulgeSmooth{0};
    bool useBulgeMin{false};
    bool useBulgeMax{false};
    Vec3d targetOffset{0, 0, 0};
    Mat4 ownerLocal{1};
    Mat4 ownerRest{1};
    Mat4 ownerParentRest{1};
    bool ownerHasParent{false};
    bool ownerInheritRotation{true};
    bool ownerLocalLocation{true};
    std::string ownerInheritScale{"FULL"};
    Mat4 sourceLocal{1};
    Mat4 sourceRest{1};
    Mat4 sourceParentRest{1};
    bool sourceHasParent{false};
    bool sourceInheritRotation{true};
    bool sourceLocalLocation{true};
    std::string sourceInheritScale{"FULL"};
    Mat4 inverseMesh{1};
    bool fromBind{true};
    bool followOnly{false};
    Mat4 local{1};
    bool useIncoming{false};
    double tx{0};
    double ty{0};
    double tz{0};
    double rx{0};
    double ry{0};
    double rz{0};
    double sx{1};
    double sy{1};
    double sz{1};
    std::string spaceKind{"pose"};
    Mat4 parentRest{1};
    bool hasParent{false};
    bool inheritRotation{true};
    bool localLocation{true};
    bool connected{false};
    std::string inheritScale{"FULL"};
    Mat4 targetRest{1};
    Mat4 object{1};
    Mat4 source{1};
    std::vector<Mat4> targets{};
    std::vector<Mat4> targetObjects{};
    Mat4 sourceObject{1};
    Mat4 ownerObject{1};
    Mat4 customSpace{1};
    Mat4 ownerParent{1};
    Mat4 sourceParent{1};
    Mat4 owner{1};
    Mat4 parent{1};
};

/// The inputs a provider operation binds by name, one per single-valued
/// field of RigExecAffineFrameInputsT. A constraint frame's frame lists
/// (targets, targetObjects) are positional and have no field.
enum class RigExecAffineField : uint8_t {
    incoming, preserveLocation, origin, inverseBind, operation, influence,
    targetIndices, objectIndices, targetBinds, targetWeights, pivot,
    dualQuaternion, currentPivot, ownerSpace, targetSpace, axisMask,
    invertMask, offset, uniformScale, scaleAdd, power, rotationMix,
    removeTargetShear, mapFrom, mapFromMin, mapFromMax, mapToMin, mapToMax,
    mapAxes, mapExtrapolate, mapMix, trackAxis, keepAxis, volume, restLength,
    bulge, bulgeMin, bulgeMax, bulgeSmooth, useBulgeMin, useBulgeMax,
    targetOffset, ownerLocal, ownerRest, ownerParentRest, ownerHasParent,
    ownerInheritRotation, ownerLocalLocation, ownerInheritScale, sourceLocal,
    sourceRest, sourceParentRest, sourceHasParent, sourceInheritRotation,
    sourceLocalLocation, sourceInheritScale, inverseMesh, fromBind,
    followOnly, local, useIncoming, tx, ty, tz, rx, ry, rz, sx, sy, sz,
    spaceKind, parentRest, hasParent, inheritRotation, localLocation,
    connected, inheritScale, targetRest, object, source, sourceObject,
    ownerObject, customSpace, ownerParent, sourceParent, owner, parent
};

/// Calls \p f with \p in's member for \p field; false for no such field.
template <class M, class F>
bool
RigExecVisitAffineField(RigExecAffineFrameInputsT<M> &in,
                        RigExecAffineField field, F &&f)
{
    using A = RigExecAffineField;
    switch (field) {
#define RIGEXEC_AFFINE_FIELD(name) \
    case A::name: f(in.name); return true;
    RIGEXEC_AFFINE_FIELD(incoming) RIGEXEC_AFFINE_FIELD(preserveLocation)
    RIGEXEC_AFFINE_FIELD(origin) RIGEXEC_AFFINE_FIELD(inverseBind)
    RIGEXEC_AFFINE_FIELD(operation) RIGEXEC_AFFINE_FIELD(influence)
    RIGEXEC_AFFINE_FIELD(targetIndices) RIGEXEC_AFFINE_FIELD(objectIndices)
    RIGEXEC_AFFINE_FIELD(targetBinds) RIGEXEC_AFFINE_FIELD(targetWeights)
    RIGEXEC_AFFINE_FIELD(pivot) RIGEXEC_AFFINE_FIELD(dualQuaternion)
    RIGEXEC_AFFINE_FIELD(currentPivot) RIGEXEC_AFFINE_FIELD(ownerSpace)
    RIGEXEC_AFFINE_FIELD(targetSpace) RIGEXEC_AFFINE_FIELD(axisMask)
    RIGEXEC_AFFINE_FIELD(invertMask) RIGEXEC_AFFINE_FIELD(offset)
    RIGEXEC_AFFINE_FIELD(uniformScale) RIGEXEC_AFFINE_FIELD(scaleAdd)
    RIGEXEC_AFFINE_FIELD(power) RIGEXEC_AFFINE_FIELD(rotationMix)
    RIGEXEC_AFFINE_FIELD(removeTargetShear) RIGEXEC_AFFINE_FIELD(mapFrom)
    RIGEXEC_AFFINE_FIELD(mapFromMin) RIGEXEC_AFFINE_FIELD(mapFromMax)
    RIGEXEC_AFFINE_FIELD(mapToMin) RIGEXEC_AFFINE_FIELD(mapToMax)
    RIGEXEC_AFFINE_FIELD(mapAxes) RIGEXEC_AFFINE_FIELD(mapExtrapolate)
    RIGEXEC_AFFINE_FIELD(mapMix) RIGEXEC_AFFINE_FIELD(trackAxis)
    RIGEXEC_AFFINE_FIELD(keepAxis) RIGEXEC_AFFINE_FIELD(volume)
    RIGEXEC_AFFINE_FIELD(restLength) RIGEXEC_AFFINE_FIELD(bulge)
    RIGEXEC_AFFINE_FIELD(bulgeMin) RIGEXEC_AFFINE_FIELD(bulgeMax)
    RIGEXEC_AFFINE_FIELD(bulgeSmooth) RIGEXEC_AFFINE_FIELD(useBulgeMin)
    RIGEXEC_AFFINE_FIELD(useBulgeMax) RIGEXEC_AFFINE_FIELD(targetOffset)
    RIGEXEC_AFFINE_FIELD(ownerLocal) RIGEXEC_AFFINE_FIELD(ownerRest)
    RIGEXEC_AFFINE_FIELD(ownerParentRest) RIGEXEC_AFFINE_FIELD(ownerHasParent)
    RIGEXEC_AFFINE_FIELD(ownerInheritRotation)
    RIGEXEC_AFFINE_FIELD(ownerLocalLocation)
    RIGEXEC_AFFINE_FIELD(ownerInheritScale) RIGEXEC_AFFINE_FIELD(sourceLocal)
    RIGEXEC_AFFINE_FIELD(sourceRest) RIGEXEC_AFFINE_FIELD(sourceParentRest)
    RIGEXEC_AFFINE_FIELD(sourceHasParent)
    RIGEXEC_AFFINE_FIELD(sourceInheritRotation)
    RIGEXEC_AFFINE_FIELD(sourceLocalLocation)
    RIGEXEC_AFFINE_FIELD(sourceInheritScale) RIGEXEC_AFFINE_FIELD(inverseMesh)
    RIGEXEC_AFFINE_FIELD(fromBind) RIGEXEC_AFFINE_FIELD(followOnly)
    RIGEXEC_AFFINE_FIELD(local) RIGEXEC_AFFINE_FIELD(useIncoming)
    RIGEXEC_AFFINE_FIELD(tx) RIGEXEC_AFFINE_FIELD(ty) RIGEXEC_AFFINE_FIELD(tz)
    RIGEXEC_AFFINE_FIELD(rx) RIGEXEC_AFFINE_FIELD(ry) RIGEXEC_AFFINE_FIELD(rz)
    RIGEXEC_AFFINE_FIELD(sx) RIGEXEC_AFFINE_FIELD(sy) RIGEXEC_AFFINE_FIELD(sz)
    RIGEXEC_AFFINE_FIELD(spaceKind) RIGEXEC_AFFINE_FIELD(parentRest)
    RIGEXEC_AFFINE_FIELD(hasParent) RIGEXEC_AFFINE_FIELD(inheritRotation)
    RIGEXEC_AFFINE_FIELD(localLocation) RIGEXEC_AFFINE_FIELD(connected)
    RIGEXEC_AFFINE_FIELD(inheritScale) RIGEXEC_AFFINE_FIELD(targetRest)
    RIGEXEC_AFFINE_FIELD(object) RIGEXEC_AFFINE_FIELD(source)
    RIGEXEC_AFFINE_FIELD(sourceObject) RIGEXEC_AFFINE_FIELD(ownerObject)
    RIGEXEC_AFFINE_FIELD(customSpace) RIGEXEC_AFFINE_FIELD(ownerParent)
    RIGEXEC_AFFINE_FIELD(sourceParent) RIGEXEC_AFFINE_FIELD(owner)
    RIGEXEC_AFFINE_FIELD(parent)
#undef RIGEXEC_AFFINE_FIELD
    }
    return false;
}

/// One named input of an expression: an attribute, or a relationship whose
/// single target's point frame the field reads as a matrix.
struct RigExecAffineInputName {
    const char *name;
    RigExecAffineField field;
};

/// An expression's provider operation layout. The operation's inputs are,
/// in order: one value per attribute, one point frame per relation, then for
/// a constraint frame (targets non-null) one point frame per target and one
/// per target object.
struct RigExecAffineFrameType {
    const char *type;
    const RigExecAffineInputName *attributes;
    uint32_t attributeCount;
    const RigExecAffineInputName *relations;
    uint32_t relationCount;
    const char *targets, *targetObjects;
};

namespace affineFrameLayout {
using A = RigExecAffineField;
#define RIGEXEC_AFFINE_CHANNELS                                              \
    {"inputs:tx", A::tx}, {"inputs:ty", A::ty}, {"inputs:tz", A::tz},        \
    {"inputs:rx", A::rx}, {"inputs:ry", A::ry}, {"inputs:rz", A::rz},        \
    {"inputs:sx", A::sx}, {"inputs:sy", A::sy}, {"inputs:sz", A::sz}
inline constexpr RigExecAffineInputName copyAttributes[] = {
    {"inputs:incoming", A::incoming},
    {"inputs:preserveLocation", A::preserveLocation}};
inline constexpr RigExecAffineInputName copyRelations[] = {
    {"rigExec:source", A::source}};
inline constexpr RigExecAffineInputName mappedAttributes[] = {
    {"inputs:targetRest", A::targetRest}, {"inputs:sourceRest", A::sourceRest}};
inline constexpr RigExecAffineInputName mappedRelations[] = {
    {"rigExec:source", A::source}};
inline constexpr RigExecAffineInputName skinAttributes[] = {
    {"inputs:inverseBind", A::inverseBind},
    {"inputs:inverseMesh", A::inverseMesh},
    {"inputs:fromBind", A::fromBind}, {"inputs:followOnly", A::followOnly}};
inline constexpr RigExecAffineInputName skinRelations[] = {
    {"rigExec:owner", A::owner}, {"rigExec:source", A::source},
    {"rigExec:sourceObject", A::sourceObject}};
inline constexpr RigExecAffineInputName armatureAttributes[] = {
    {"inputs:local", A::local}, {"inputs:inverseBind", A::inverseBind},
    {"inputs:incoming", A::incoming}, {"inputs:useIncoming", A::useIncoming},
    {"inputs:preserveLocation", A::preserveLocation}, RIGEXEC_AFFINE_CHANNELS};
inline constexpr RigExecAffineInputName armatureRelations[] = {
    {"rigExec:parent", A::parent}, {"rigExec:sourceObject", A::sourceObject},
    {"rigExec:source", A::source}};
inline constexpr RigExecAffineInputName boneAttributes[] = {
    {"inputs:spaceKind", A::spaceKind}, {"inputs:local", A::local},
    {"inputs:parentRest", A::parentRest}, {"inputs:hasParent", A::hasParent},
    {"inputs:inheritRotation", A::inheritRotation},
    {"inputs:localLocation", A::localLocation},
    {"inputs:connected", A::connected},
    {"inputs:inheritScale", A::inheritScale}, RIGEXEC_AFFINE_CHANNELS};
inline constexpr RigExecAffineInputName boneRelations[] = {
    {"rigExec:parent", A::parent}, {"rigExec:sourceObject", A::object}};
inline constexpr RigExecAffineInputName constraintAttributes[] = {
    {"inputs:incoming", A::incoming}, {"inputs:origin", A::origin},
    {"inputs:inverseBind", A::inverseBind}, {"inputs:operation", A::operation},
    {"inputs:influence", A::influence},
    {"inputs:targetIndices", A::targetIndices},
    {"inputs:objectIndices", A::objectIndices},
    {"inputs:targetBinds", A::targetBinds},
    {"inputs:targetWeights", A::targetWeights}, {"inputs:pivot", A::pivot},
    {"inputs:dualQuaternion", A::dualQuaternion},
    {"inputs:currentPivot", A::currentPivot},
    {"inputs:ownerSpace", A::ownerSpace},
    {"inputs:targetSpace", A::targetSpace}, {"inputs:axisMask", A::axisMask},
    {"inputs:invertMask", A::invertMask}, {"inputs:offset", A::offset},
    {"inputs:uniformScale", A::uniformScale}, {"inputs:scaleAdd", A::scaleAdd},
    {"inputs:power", A::power}, {"inputs:rotationMix", A::rotationMix},
    {"inputs:removeTargetShear", A::removeTargetShear},
    {"inputs:mapFrom", A::mapFrom}, {"inputs:mapFromMin", A::mapFromMin},
    {"inputs:mapFromMax", A::mapFromMax}, {"inputs:mapToMin", A::mapToMin},
    {"inputs:mapToMax", A::mapToMax}, {"inputs:mapAxes", A::mapAxes},
    {"inputs:mapExtrapolate", A::mapExtrapolate}, {"inputs:mapMix", A::mapMix},
    {"inputs:trackAxis", A::trackAxis}, {"inputs:keepAxis", A::keepAxis},
    {"inputs:volume", A::volume}, {"inputs:restLength", A::restLength},
    {"inputs:bulge", A::bulge}, {"inputs:bulgeMin", A::bulgeMin},
    {"inputs:bulgeMax", A::bulgeMax}, {"inputs:bulgeSmooth", A::bulgeSmooth},
    {"inputs:useBulgeMin", A::useBulgeMin},
    {"inputs:useBulgeMax", A::useBulgeMax},
    {"inputs:targetOffset", A::targetOffset},
    {"inputs:ownerLocal", A::ownerLocal}, {"inputs:ownerRest", A::ownerRest},
    {"inputs:ownerParentRest", A::ownerParentRest},
    {"inputs:ownerHasParent", A::ownerHasParent},
    {"inputs:ownerInheritRotation", A::ownerInheritRotation},
    {"inputs:ownerLocalLocation", A::ownerLocalLocation},
    {"inputs:ownerInheritScale", A::ownerInheritScale},
    {"inputs:sourceLocal", A::sourceLocal}, {"inputs:sourceRest", A::sourceRest},
    {"inputs:sourceParentRest", A::sourceParentRest},
    {"inputs:sourceHasParent", A::sourceHasParent},
    {"inputs:sourceInheritRotation", A::sourceInheritRotation},
    {"inputs:sourceLocalLocation", A::sourceLocalLocation},
    {"inputs:sourceInheritScale", A::sourceInheritScale}};
inline constexpr RigExecAffineInputName constraintRelations[] = {
    {"rigExec:source", A::source}, {"rigExec:sourceObject", A::sourceObject},
    {"rigExec:ownerObject", A::ownerObject},
    {"rigExec:customSpace", A::customSpace},
    {"rigExec:ownerParent", A::ownerParent},
    {"rigExec:sourceParent", A::sourceParent}};
#undef RIGEXEC_AFFINE_CHANNELS
template <size_t N>
constexpr uint32_t Count(const RigExecAffineInputName (&)[N]) { return uint32_t(N); }
}  // namespace affineFrameLayout

/// Every expression, in affine kind order: a provider operation's
/// affineKind indexes this table.
inline constexpr RigExecAffineFrameType RigExecAffineFrameTypes[] = {
    {"RigExecCopyFrame", affineFrameLayout::copyAttributes,
     affineFrameLayout::Count(affineFrameLayout::copyAttributes),
     affineFrameLayout::copyRelations,
     affineFrameLayout::Count(affineFrameLayout::copyRelations), nullptr,
     nullptr},
    {"RigExecMappedFrame", affineFrameLayout::mappedAttributes,
     affineFrameLayout::Count(affineFrameLayout::mappedAttributes),
     affineFrameLayout::mappedRelations,
     affineFrameLayout::Count(affineFrameLayout::mappedRelations), nullptr,
     nullptr},
    {"RigExecSkinInfluence", affineFrameLayout::skinAttributes,
     affineFrameLayout::Count(affineFrameLayout::skinAttributes),
     affineFrameLayout::skinRelations,
     affineFrameLayout::Count(affineFrameLayout::skinRelations), nullptr,
     nullptr},
    {"RigExecArmatureParent", affineFrameLayout::armatureAttributes,
     affineFrameLayout::Count(affineFrameLayout::armatureAttributes),
     affineFrameLayout::armatureRelations,
     affineFrameLayout::Count(affineFrameLayout::armatureRelations), nullptr,
     nullptr},
    {"RigExecBoneFrame", affineFrameLayout::boneAttributes,
     affineFrameLayout::Count(affineFrameLayout::boneAttributes),
     affineFrameLayout::boneRelations,
     affineFrameLayout::Count(affineFrameLayout::boneRelations), nullptr,
     nullptr},
    {"RigExecConstraintFrame", affineFrameLayout::constraintAttributes,
     affineFrameLayout::Count(affineFrameLayout::constraintAttributes),
     affineFrameLayout::constraintRelations,
     affineFrameLayout::Count(affineFrameLayout::constraintRelations),
     "rigExec:targets", "rigExec:targetObjects"},
};
inline constexpr uint32_t RigExecAffineFrameTypeCount =
    uint32_t(sizeof(RigExecAffineFrameTypes) /
             sizeof(RigExecAffineFrameTypes[0]));

/// Whether a provider operation of affine kind \p kind may have \p inputs
/// inputs of which \p targets are a constraint frame's targets.
inline bool
RigExecAffineFrameInputsMatch(uint32_t kind, size_t inputs, size_t targets)
{
    if (kind >= RigExecAffineFrameTypeCount) {
        return false;
    }
    const RigExecAffineFrameType &type = RigExecAffineFrameTypes[kind];
    const size_t fixed = size_t(type.attributeCount) + type.relationCount;
    if (inputs < fixed) {
        return false;
    }
    return type.targets ? targets <= inputs - fixed
                        : inputs == fixed && targets == 0;
}

/// The expressions over a math policy. Compute reports a failure (an
/// Armature blend whose target arrays disagree) through \p failure and
/// returns false; the result is then unspecified.
template <class M>
struct RigExecAffineFrameKernel {
    using Mat4 = typename M::Mat4;
    using Mat3 = typename M::Mat3;
    using Vec3d = typename M::Vec3d;
    using Vec3f = typename M::Vec3f;
    using Quat = typename M::Quat;
    using DualQuat = typename M::DualQuat;
    using Inputs = RigExecAffineFrameInputsT<M>;

    /// A provider's point frame from its four landmarks, as a matrix: the
    /// axes relative to the origin as rows, then the origin.
    static Mat4 FrameMatrix(const Vec3d &origin, const Vec3d &x,
                            const Vec3d &y, const Vec3d &z)
    {
        Mat4 result(1);
        const Vec3d axes[3] = {x - origin, y - origin, z - origin};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) result[i][j] = axes[i][j];
        result.SetTranslateOnly(origin);
        return result;
    }

    /// Expression \p kind (RigExecAffineFrameTypes order) of \p in.
    static bool Compute(uint32_t kind, const Inputs &in, Mat4 *result,
                        const char **failure)
    {
        *failure = nullptr;
        switch (kind) {
        case 0: *result = CopyTransforms(in); return true;
        case 1: *result = MappedFrame(in); return true;
        case 2: *result = SkinInfluence(in); return true;
        case 3: *result = ArmatureParent(in); return true;
        case 4: *result = BoneFrame(in); return true;
        case 5: *result = ConstraintFrame(in, failure); return !*failure;
        default: *failure = "unknown affine frame expression"; return false;
        }
    }

    static Mat4 Channels(const Inputs &in)
    {
        Mat4 local(1);
        local.SetScale(Vec3d(in.sx,
                             in.sy, in.sz));
        const double rotations[] = {in.rx, in.ry, in.rz};
        for (int i = 0; i < 3; ++i) {
            Vec3d axis(0); axis[i] = 1;
            local *= M::Rotation(axis, rotations[i]);
        }
        local.SetTranslateOnly(Vec3d(in.tx,
                                     in.ty, in.tz));
        return local;
    }
    static Mat4 ArmatureParent(const Inputs &in)
    {
        Mat4 incoming = in.useIncoming
            ? in.incoming
            : Channels(in) * in.local *
              in.parent;
        // The armature constraint applies the target's rest-to-pose map
        // over the incoming owner frame, preserving the owner's local
        // channels.
        auto result = incoming *
            in.sourceObject.GetInverse() *
            in.inverseBind *
            in.source;
        if (in.preserveLocation)
            result.SetTranslateOnly(incoming.ExtractTranslation());
        return result;
    }
    static Vec3d Row(const Mat4 &m, int i) { return Vec3d(m[i][0], m[i][1], m[i][2]); }
    static void SetRow(Mat4 &m, int i, const Vec3d &v) { for (int j = 0; j < 3; ++j) m[i][j] = v[j]; }
    static Vec3d Sizes(const Mat4 &m)
    {
        return Vec3d(Row(m, 0).GetLength(), Row(m, 1).GetLength(), Row(m, 2).GetLength());
    }
    static Vec3d VolumeSizes(const Mat4 &m)
    {
        auto sizes = Sizes(m); double product = sizes[0] * sizes[1] * sizes[2];
        return product > 0 ? sizes * std::cbrt(std::abs(m.GetDeterminant()) / product) : sizes;
    }
    static void Rescale(Mat4 &m, const Vec3d &s) { for (int i = 0; i < 3; ++i) SetRow(m, i, Row(m, i) * s[i]); }
    static void NormalizeRows(Mat4 &m)
    {
        for (int i = 0; i < 3; ++i) { auto v = Row(m, i); v.Normalize(); SetRow(m, i, v); }
    }
    // Symmetric orthogonalization around the bone's Y axis: project X/Z,
    // then split their angular error equally. Preserving area retains
    // volume under shear. This is deliberately different from a generic
    // polar matrix split.
    static void Orthogonalize(Mat4 &m, bool normalize)
    {
        auto y = Row(m, 1), x = Row(m, 0), z = Row(m, 2);
        double y2 = y.GetLengthSq();
        if (y2 > 0) {
            x -= y * (M::Dot(x, y) / y2); z -= y * (M::Dot(z, y) / y2);
            if (normalize) y /= std::sqrt(y2);
        }
        double lx = x.Normalize(), lz = z.Normalize();
        double cosine = std::clamp(M::Dot(x, z), -1.0, 1.0);
        if (std::abs(cosine) > 1e-4 && std::abs(cosine) < 1 - 1.1920928955078125e-7) {
            auto sum = x + z, difference = x - z; sum.Normalize(); difference.Normalize();
            x = (sum + difference) / std::sqrt(2.0); z = (sum - difference) / std::sqrt(2.0);
            double area = std::sqrt(std::sqrt(std::max(0.0, 1 - cosine * cosine)));
            lx *= area; lz *= area;
        }
        SetRow(m, 0, x * (normalize ? 1 : lx)); SetRow(m, 1, y); SetRow(m, 2, z * (normalize ? 1 : lz));
    }
    struct ParentSpaces {
        Mat4 rotationScale, location;
        Vec3d post;
        Mat4 Apply(const Mat4 &input, bool inverse = false) const
        {
            auto rotation = inverse ? rotationScale.GetInverse() : rotationScale;
            auto position = inverse ? location.GetInverse() : location;
            auto output = input * rotation;
            output.SetTranslateOnly(position.Transform(input.ExtractTranslation()));
            Rescale(output, inverse ? Vec3d(1 / post[0], 1 / post[1], 1 / post[2]) : post);
            return output;
        }
    };
    static ParentSpaces BoneParentSpaces(const Mat4 &rest, const Mat4 &parentRest,
        const Mat4 &parent, bool hasParent, const std::string &mode, bool inheritRotation,
        bool localLocation)
    {
        Mat4 rotationScale = rest, location = rest; Vec3d post(1);
        if (hasParent) {
            Mat4 adjusted = parent;
            if (inheritRotation) {
                if (mode == "NONE" || mode == "AVERAGE") Orthogonalize(adjusted, true);
                else if (mode == "ALIGNED") { Orthogonalize(adjusted, false); post = Sizes(adjusted); NormalizeRows(adjusted); }
                else if (mode == "NONE_LEGACY") NormalizeRows(adjusted);
            } else {
                adjusted = parentRest;
                if (mode == "FULL") Rescale(adjusted, Sizes(parent));
                else if (mode == "FIX_SHEAR" || mode == "ALIGNED") {
                    if (mode == "ALIGNED") post = VolumeSizes(parent); else Rescale(adjusted, VolumeSizes(parent));
                }
            }
            if (mode == "AVERAGE") Rescale(adjusted, Vec3d(std::cbrt(std::abs(parent.GetDeterminant()))));
            rotationScale = rest * adjusted;
            if (mode == "FIX_SHEAR") Orthogonalize(rotationScale, false);
            location = rest * parent;
            if (!localLocation) { location = parent; location.SetTranslateOnly(parent.Transform(rest.ExtractTranslation())); }
        } else if (!localLocation) location.SetTranslate(rest.ExtractTranslation());
        return {rotationScale, location, post};
    }
    static Mat4 BoneFrame(const Inputs &in)
    {
        auto object = in.object;
        auto channels = Channels(in);
        if (in.connected) channels.SetTranslateOnly(Vec3d(0));
        auto spaces = BoneParentSpaces(in.local,
            in.parentRest,
            in.parent * object.GetInverse(),
            in.hasParent, in.inheritScale,
            in.inheritRotation, in.localLocation);
        const auto kind = in.spaceKind;
        if (kind == "rotation") return spaces.rotationScale * object;
        if (kind == "translation") return spaces.location * object;
        return spaces.Apply(channels) * object;
    }
    static ParentSpaces ConstraintParentSpaces(const Inputs &in, bool source, const Mat4 &object)
    {
        return BoneParentSpaces((source ? in.sourceLocal : in.ownerLocal),
            (source ? in.sourceParentRest : in.ownerParentRest),
            (source ? in.sourceParent : in.ownerParent) * object.GetInverse(),
            (source ? in.sourceHasParent : in.ownerHasParent),
            (source ? in.sourceInheritScale : in.ownerInheritScale),
            (source ? in.sourceInheritRotation : in.ownerInheritRotation),
            (source ? in.sourceLocalLocation : in.ownerLocalLocation));
    }
    static Mat4 SkinInfluence(const Inputs &in)
    {
        Mat4 prefix(1);
        if (in.fromBind)
            prefix = in.inverseMesh *
                in.owner;
        if (in.followOnly) return prefix;
        return prefix * in.sourceObject.GetInverse() *
            in.inverseBind *
            in.source;
    }
    static Mat4 MappedFrame(const Inputs &in)
    {
        return in.targetRest *
            in.sourceRest.GetInverse() *
            in.source;
    }
    static Mat4 CopyTransforms(const Inputs &in)
    {
        auto result = in.source;
        if (in.preserveLocation)
            result.SetTranslateOnly(in.incoming.ExtractTranslation());
        return result;
    }
    static void Track(Mat4 &m, Vec3d direction, const std::string &axisToken)
    {
        // Tracking direction and axis-angle use float precision. Retain
        // that boundary: near a half-turn, rounding after the cross product
        // instead of before it changes the chosen rotation axis appreciably.
        Vec3f targetDirection = M::ToFloat(direction);
        if (targetDirection.Normalize() == 0) return;
        int axis = axisToken.back() - 'X'; bool negative = axisToken.find("NEGATIVE") != std::string::npos;
        Vec3f original = M::ToFloat(Row(m, axis) * (negative ? -1 : 1));
        if (original.Normalize() == 0) { original = Vec3f(0.0f); original[axis] = negative ? -1 : 1; }
        Vec3f perpendicular = M::ToFloat(M::Cross(M::ToDouble(original), M::ToDouble(targetDirection)));
        float sine = perpendicular.Normalize();
        float cosine = std::clamp(M::Dot(original, targetDirection), -1.0f, 1.0f);
        float angle = std::acos(cosine);
        if (sine < 1.1920928955078125e-7) {
            if (angle < 3.14159265358979323846 - 0.01) return;
            // At a half-turn the next local track direction resolves the ambiguity.
            auto next = Row(m, (axis + 1) % 3) * (axis == 2 ? (negative ? 1 : -1) : (negative ? -1 : 1));
            perpendicular = M::ToFloat(M::Cross(M::ToDouble(original), next)); if (perpendicular.Normalize() == 0) return;
            angle = 3.14159265358979323846;
        } else if (sine < 0.1f) angle = cosine < 0 ? float(3.14159265358979323846) - std::asin(sine) : std::asin(sine);
        auto rotation = M::Rotation(M::ToDouble(perpendicular), angle * 180 / 3.14159265358979323846);
        auto origin = m.ExtractTranslation(); m *= rotation; m.SetTranslateOnly(origin);
    }
    static Mat4 ArmatureBlend(const Inputs &in, const Mat4 &incoming, const char **failure)
    {
        const auto &binds = in.targetBinds;
        const auto &weights = in.targetWeights;
        const auto &targetIndices = in.targetIndices;
        const auto &objectIndices = in.objectIndices;
        const auto &targets = in.targets, &objects = in.targetObjects;
        if (binds.size() != weights.size() || targetIndices.size() != weights.size() || objectIndices.size() != weights.size()) {
            *failure = "Armature constraint target array mismatch";
            return incoming;
        }
        auto pivot = in.currentPivot ? incoming.ExtractTranslation() :
            in.ownerObject.Transform(in.pivot);
        bool dq = in.dualQuaternion;
        Mat4 matrix(0.0), stretchSum(0.0);
        DualQuat sum = M::DualQuatZero();
        double total = 0;
        for (size_t i = 0; i < weights.size(); ++i) {
            double weight = weights[i]; if (weight == 0) continue; total += weight;
            int ti = targetIndices[i], oi = objectIndices[i];
            if (ti < 0 || oi < 0 || size_t(ti) >= targets.size() || size_t(oi) >= objects.size()) {
                *failure = "invalid Armature target index";
                return incoming;
            }
            const auto delta = objects[oi].GetInverse() * binds[i] * targets[ti];
            if (!dq) { matrix += delta * weight; continue; }
            // The scale gauge follows the rest bone's Y axis. A generic
            // polar split differs for stretched bones. Retain the residual
            // as an affine matrix, and move its pivot displacement into the
            // rigid part before blending. See docs/references.md.
            auto base = binds[i].GetInverse() * objects[oi]; Orthogonalize(base, true);
            auto posed = base * delta;
            // The posed gauge uses sequential Y/X orthogonalization,
            // whereas the rest gauge above uses its symmetric stable rule.
            auto y = Row(posed, 1); y.Normalize();
            auto z = M::Cross(Row(posed, 0), y); z.Normalize();
            auto x = M::Cross(y, z); x.Normalize();
            SetRow(posed, 0, x); SetRow(posed, 1, y); SetRow(posed, 2, z);
            auto rigid = base.GetInverse() * posed;
            auto stretch = delta * rigid.GetInverse();
            auto shift = stretch.Transform(pivot) - pivot;
            rigid.SetTranslateOnly(rigid.ExtractTranslation() + rigid.TransformDir(shift));
            stretch.SetTranslateOnly(pivot - stretch.TransformDir(pivot));
            auto value = M::DualQuatFromMatrix(rigid);
            // Source order matters: correct against the accumulated
            // rotation, rather than choosing a fixed reference target.
            double signedWeight = M::Dot(sum.real, value.real) < 0 ? -weight : weight;
            sum.real += value.real * signedWeight; sum.dual += value.dual * signedWeight;
            stretchSum += stretch * weight;
        }
        if (total <= 0) return incoming;
        if (dq) {
            auto normalized = sum;
            if (!M::DualQuatNormalize(&normalized)) return incoming;
            matrix = (stretchSum * (1.0 / total)) * M::DualQuatToMatrix(normalized);
        } else matrix *= 1.0 / total;
        return incoming * matrix;
    }
    static Vec3d EulerXYZ(Mat4 matrix)
    {
        NormalizeRows(matrix);
        const double cy = std::hypot(matrix[0][0], matrix[0][1]);
        Vec3d first(std::atan2(matrix[1][2], matrix[2][2]), std::atan2(-matrix[0][2], cy), std::atan2(matrix[0][1], matrix[0][0]));
        if (cy <= 0.0000375) return Vec3d(std::atan2(-matrix[2][1], matrix[1][1]), first[1], 0);
        const Vec3d second(std::atan2(-matrix[1][2], -matrix[2][2]), std::atan2(-matrix[0][2], -cy), std::atan2(-matrix[0][1], -matrix[0][0]));
        auto magnitude = [](const Vec3d &v) { return std::abs(v[0]) + std::abs(v[1]) + std::abs(v[2]); };
        return magnitude(first) <= magnitude(second) ? first : second;
    }
    static Mat4 ConstraintValue(const Inputs &in, const char **failure)
    {
        auto result = in.incoming;
        auto operation = in.operation;
        if (operation == "PRESERVE_ORIGIN") {
            result.SetTranslateOnly(in.origin.ExtractTranslation()); return result;
        }
        if (operation == "LIMIT_ROTATION") { Orthogonalize(result, false); return result; }
        if (operation == "ARMATURE_BLEND") return ArmatureBlend(in, result, failure);
        auto source = in.source;
        auto ownerSpace = in.ownerSpace;
        auto targetSpace = in.targetSpace;
        auto ownerObject = in.ownerObject;
        auto sourceObject = in.sourceObject;
        auto customSpace = in.customSpace;
        bool localTarget = targetSpace == "LOCAL" || targetSpace == "LOCAL_OWNER_ORIENT";
        if (localTarget) source.SetTranslateOnly(source.Transform(in.targetOffset));
        ParentSpaces ownerLocal;
        if (ownerSpace == "CUSTOM") result *= customSpace.GetInverse();
        else if (ownerSpace != "WORLD") {
            result *= ownerObject.GetInverse();
            if (ownerSpace == "LOCAL") {
                ownerLocal = ConstraintParentSpaces(in, false, ownerObject);
                result = ownerLocal.Apply(result, true);
            }
        }
        if (targetSpace == "CUSTOM") source *= customSpace.GetInverse();
        else if (targetSpace != "WORLD") {
            source *= sourceObject.GetInverse();
            if (targetSpace == "LOCAL" || targetSpace == "LOCAL_OWNER_ORIENT") {
                source = ConstraintParentSpaces(in, true, sourceObject).Apply(source, true);
                if (targetSpace == "LOCAL_OWNER_ORIENT") {
                    auto difference = in.sourceRest *
                        in.ownerRest.GetInverse();
                    difference.SetTranslateOnly(Vec3d(0));
                    source = difference.GetInverse() * source * difference;
                }
            }
        }
        auto world = [&](const Mat4 &matrix) {
            if (ownerSpace == "WORLD") return matrix;
            if (ownerSpace == "CUSTOM") return Mat4(matrix * customSpace);
            return Mat4((ownerSpace == "LOCAL" ? ownerLocal.Apply(matrix) : matrix) * ownerObject);
        };
        auto target = localTarget ? source.ExtractTranslation() : source.Transform(in.targetOffset);
        if (operation == "TRANSFORM_LOCATION") {
            const auto from = in.mapFrom;
            Vec3d channels = source.ExtractTranslation();
            if (from == "ROTATION") channels = EulerXYZ(source);
            else if (from == "SCALE") channels = Sizes(source) * (source.GetDeterminant() < 0 ? -1.0 : 1.0);
            const auto minimum = in.mapFromMin, maximum = in.mapFromMax;
            const auto outputMin = in.mapToMin, outputMax = in.mapToMax;
            const auto axes = in.mapAxes;
            Vec3d normalized(0), location(0);
            for (int i = 0; i < 3; ++i) {
                if (minimum[i] > maximum[i] || axes[i] < 0 || axes[i] > 2) return in.incoming;
                const double channel = in.mapExtrapolate ? channels[i] : std::clamp(channels[i], minimum[i], maximum[i]);
                if (maximum[i] != minimum[i]) normalized[i] = (channel - minimum[i]) / (maximum[i] - minimum[i]);
            }
            for (int i = 0; i < 3; ++i) location[i] = outputMin[i] + normalized[axes[i]] * (outputMax[i] - outputMin[i]);
            if (in.mapMix == "ADD") location += result.ExtractTranslation();
            result.SetTranslateOnly(location); return world(result);
        }
        if (operation == "COPY_TRANSFORMS") {
            if (in.removeTargetShear) Orthogonalize(source, false);
            auto mix = in.rotationMix;
            if (mix == "REPLACE") return world(source);
            bool before = mix.find("BEFORE") == 0;
            if (mix == "BEFORE_FULL" || mix == "AFTER_FULL") return world(before ? result * source : source * result);
            auto location = mix.find("SPLIT") != std::string::npos ? result.ExtractTranslation() + source.ExtractTranslation() :
                before ? source.Transform(result.ExtractTranslation()) : result.Transform(source.ExtractTranslation());
            auto size = M::CompMult(Sizes(result), Sizes(source));
            auto a = result, b = source; NormalizeRows(a); NormalizeRows(b);
            if (a.GetDeterminant() < 0) { Rescale(a, Vec3d(-1)); size = -size; }
            if (b.GetDeterminant() < 0) { Rescale(b, Vec3d(-1)); size = -size; }
            a.SetTranslateOnly(Vec3d(0)); b.SetTranslateOnly(Vec3d(0));
            auto combined = before ? a * b : b * a; Rescale(combined, size); combined.SetTranslateOnly(location);
            return world(combined);
        }
        if (operation == "ARMATURE")
            return result * in.sourceObject.GetInverse() *
                in.inverseBind * source;
        if (operation == "COPY_LOCATION") {
            auto origin = result.ExtractTranslation(); int axes = in.axisMask, invert = in.invertMask;
            for (int i = 0; i < 3; ++i) if (axes & (1 << i)) origin[i] = (invert & (1 << i) ? -target[i] : target[i]) + (in.offset ? origin[i] : 0);
            result.SetTranslateOnly(origin); return world(result);
        }
        if (operation == "COPY_SCALE") {
            auto size = Sizes(source), original = Sizes(result);
            int axes = in.axisMask;
            bool uniform = in.uniformScale;
            if (uniform) {
                double product = axes == 7 ? std::abs(source.GetDeterminant()) : 1;
                if (axes != 7) for (int i = 0; i < 3; ++i) if (axes & (1 << i)) product *= size[i];
                size = Vec3d(std::cbrt(product));
            }
            for (int i = 0; i < 3; ++i) {
                size[i] = std::pow(size[i], in.power);
                if (in.offset)
                    size[i] = in.scaleAdd ? size[i] + original[i] - 1 : size[i] * original[i];
                if ((uniform || (axes & (1 << i))) && original[i] != 0) SetRow(result, i, Row(result, i) * (size[i] / original[i]));
            }
            return world(result);
        }
        if (operation == "COPY_ROTATION") {
            auto size = Sizes(result), origin = result.ExtractTranslation();
            if (in.axisMask == 0) return world(result);
            auto mix = in.rotationMix;
            Orthogonalize(source, true);
            source.SetTranslateOnly(Vec3d(0));
            if (mix != "REPLACE") {
                auto old = result; NormalizeRows(old); old.SetTranslateOnly(Vec3d(0));
                if (old.GetDeterminant() < 0) { Rescale(old, Vec3d(-1)); size = -size; }
                source = mix == "BEFORE" ? old * source : source * old;
            }
            Rescale(source, size); source.SetTranslateOnly(origin);
            return world(source);
        }
        auto direction = target - result.ExtractTranslation();
        if (operation == "DAMPED_TRACK") {
            direction = M::ToDouble(M::ToFloat(target) - M::ToFloat(result.ExtractTranslation()));
            Track(result, direction, in.trackAxis); return result;
        }
        // RigExec point-frame providers require invertible transforms. A
        // zero length stretch collapses the frame; retain the incoming
        // frame in this unsupported case rather than poisoning all
        // downstream skinning.
        double length = in.restLength;
        if (direction.GetLength() < 1e-6 * std::max(1.0, length) || Row(result, 1).GetLength() < 1e-12) return result;
        auto keep = in.keepAxis;
        if (keep == "SWING_Y") Orthogonalize(result, false);
        auto size = Sizes(result); NormalizeRows(result);
        double distance = direction.Normalize(); distance = size[1] != 0 ? distance / size[1] : 0;
        double stretch = distance / length;
        auto volume = in.volume;
        double bulge = volume == "NO_VOLUME" ? 1 : std::pow(length / std::max(distance, 1e-30), in.bulge);
        double smooth = in.bulgeSmooth;
        if (bulge > 1 && in.useBulgeMax) {
            double bound = std::max(1.0, in.bulgeMax), range = bound - 1;
            double soft = range > 0 ? 1 + range * std::atan((bulge - 1) / range) * 2 / 3.14159265358979323846 : 1;
            bulge = (1 - smooth) * std::min(bulge, bound) + smooth * soft;
        }
        if (bulge < 1 && in.useBulgeMin) {
            double bound = std::clamp(in.bulgeMin, 0.0, 1.0), range = 1 - bound;
            double soft = range > 0 ? 1 - range * std::atan((1 - bulge) / range) * 2 / 3.14159265358979323846 : 1;
            bulge = (1 - smooth) * std::max(bulge, bound) + smooth * soft;
        }
        Vec3d factor(1, stretch, 1);
        if (volume == "VOLUME_XZX") factor[0] = factor[2] = std::sqrt(bulge);
        else if (volume == "VOLUME_X") factor[0] = bulge;
        else if (volume == "VOLUME_Z") factor[2] = bulge;
        if (keep == "SWING_Y") Track(result, direction, "TRACK_Y");
        else {
            auto reference = Row(result, keep == "PLANE_X" ? 0 : 2);
            auto perpendicular = M::Cross(reference, direction); perpendicular.Normalize();
            SetRow(result, 1, direction);
            if (keep == "PLANE_X") { SetRow(result, 2, perpendicular); auto x = M::Cross(direction, perpendicular); x.Normalize(); SetRow(result, 0, x); }
            else { SetRow(result, 0, -perpendicular); auto z = M::Cross(direction, perpendicular); z.Normalize(); SetRow(result, 2, z); }
        }
        Rescale(result, Vec3d(size[0] * factor[0], size[1] * factor[1], size[2] * factor[2]));
        return result;
    }
    static void PolarFrame(const Mat4 &matrix, Mat3 *rotation, Mat3 *stretch)
    {
        Mat3 linear(matrix[0][0], matrix[0][1], matrix[0][2],
                    matrix[1][0], matrix[1][1], matrix[1][2],
                    matrix[2][0], matrix[2][1], matrix[2][2]);
        auto orthogonal = linear;
        // Newton's polar iteration for invertible point-frame inputs. In
        // USD's row convention the symmetric stretch is on the left:
        // linear = stretch * rotation.
        for (int iteration = 0; iteration < 64; ++iteration) {
            auto next = (orthogonal + orthogonal.GetInverse().GetTranspose()) * 0.5;
            double difference = 0;
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                difference = std::max(difference, std::abs(next[i][j] - orthogonal[i][j]));
            orthogonal = next; if (difference < 1e-12) break;
        }
        *rotation = orthogonal;
        *stretch = linear * orthogonal.GetTranspose();
        // Quaternions require a proper rotation; retain a reflection in stretch.
        if (rotation->GetDeterminant() < 0) { *rotation *= -1; *stretch *= -1; }
    }
    static Mat4 BlendFrame(Mat4 before, Mat4 after, double weight)
    {
        auto location = (1 - weight) * before.ExtractTranslation() + weight * after.ExtractTranslation();
        Mat3 rotation0, rotation1, stretch0, stretch1;
        PolarFrame(before, &rotation0, &stretch0); PolarFrame(after, &rotation1, &stretch1);
        auto q0 = M::RotationQuat(rotation0), q1 = M::RotationQuat(rotation1);
        double cosine = M::Dot(q0, q1); if (cosine < 0) { q0 = -q0; cosine = -cosine; }
        double w0 = 1 - weight, w1 = weight;
        if (cosine < 1 - 1e-4) {
            double angle = std::acos(std::clamp(cosine, -1.0, 1.0)), denominator = std::sin(angle);
            w0 = std::sin((1 - weight) * angle) / denominator; w1 = std::sin(weight * angle) / denominator;
        }
        auto linear = ((1 - weight) * stretch0 + weight * stretch1) * M::FromQuat(w0 * q0 + w1 * q1);
        Mat4 result(1);
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) result[i][j] = linear[i][j];
        result.SetTranslateOnly(location);
        return result;
    }
    static Mat4 ConstraintFrame(const Inputs &in, const char **failure)
    {
        auto value = ConstraintValue(in, failure);
        if (*failure) return value;
        double influence = in.influence;
        if (influence == 1) return value;
        auto before = in.incoming;
        // Return the solution to world space before blending influence.
        return BlendFrame(before, value, influence);
    }
};

}  // namespace rigExec

#endif

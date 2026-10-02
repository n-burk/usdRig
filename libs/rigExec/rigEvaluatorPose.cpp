// Constraint operators, pose interpolators, and rest-frame composition.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorConstraints.h"
#include "movers/moverRegistry.h"
#include "frameExtraction.h"
#include "rigExecMath/rbf.h"
#include "solverKernels.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/tf/notice.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// Bodies sit next to the dispatch they replaced.
RigExecPointFrame _SolvePositionConstraint(const _ConstraintSolveContext &);

RigExecPointFrame _SolveRotationConstraint(const _ConstraintSolveContext &);

RigExecPointFrame _SolveScaleConstraint(const _ConstraintSolveContext &);

RigExecPointFrame _SolveParentConstraint(const _ConstraintSolveContext &);

const std::vector<_ConstraintHandler> &
_ConstraintHandlers()
{
    static const std::vector<_ConstraintHandler> handlers = {
        {"RigExecAimConstraint", true, true, true, true,
         _ChannelGroup::Rotation, _ChannelGroup::Rotation, nullptr},
        {"RigExecPositionConstraint", true, true, false, false,
         _ChannelGroup::Translation, _ChannelGroup::Translation,
         _SolvePositionConstraint},
        {"RigExecRotationConstraint", true, true, true, false,
         _ChannelGroup::Rotation, _ChannelGroup::Rotation,
         _SolveRotationConstraint},
        {"RigExecScaleConstraint", true, true, false, false,
         _ChannelGroup::Scale, _ChannelGroup::Scale,
         _SolveScaleConstraint},
        {"RigExecParentConstraint", true, true, true, false,
         _ChannelGroup::All, _ChannelGroup::None,
         _SolveParentConstraint},
        {"RigExecSingleChainIkConstraint", false, true, false, true,
         _ChannelGroup::None, _ChannelGroup::None, nullptr},
    };
    return handlers;
}

RigExecConstraintAxisMask
_ReadConstraintAxisMask(
    const RigExecResolvedInputs &resolved, const UsdPrim &prim,
    const char *x, const char *y, const char *z, UsdTimeCode time,
    bool fallback = true)
{
    RigExecConstraintAxisMask mask;
    mask.x = _ResolvedRead(resolved, prim, x, fallback, time);
    mask.y = _ResolvedRead(resolved, prim, y, fallback, time);
    mask.z = _ResolvedRead(resolved, prim, z, fallback, time);
    return mask;
}

RigExecPointFrame
_SolvePositionConstraint(const _ConstraintSolveContext &c)
{
    RigExecPositionConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, "inputs:translationOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    return RigExecApplyPositionConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveRotationConstraint(const _ConstraintSolveContext &c)
{
    RigExecRotationConstraintParams params;
    params.offsetDegrees = _ResolvedRead(
        *c.resolved, c.prim, "inputs:rotationOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.rotationOrder = c.order;
    params.weight = c.weight;
    params.carry = c.carry;
    return RigExecApplyRotationConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveScaleConstraint(const _ConstraintSolveContext &c)
{
    RigExecScaleConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, "inputs:scaleOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    return RigExecApplyScaleConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveParentConstraint(const _ConstraintSolveContext &c)
{
    RigExecParentConstraintParams params;
    if (c.masksStatic) {
        params.translationAxes = c.precompTranslation;
        params.rotationAxes = c.precompRotation;
        params.scaleAxes = c.precompScale;
    } else {
        params.translationAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, "inputs:affectTranslationX",
            "inputs:affectTranslationY", "inputs:affectTranslationZ", c.time);
        params.rotationAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, "inputs:affectRotationX",
            "inputs:affectRotationY", "inputs:affectRotationZ", c.time);
        // FBX disables scale by default; the explicit false fallback is the
        // authored contract, not an oversight (schema.usda:769-771).
        params.scaleAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, "inputs:affectScaleX",
            "inputs:affectScaleY", "inputs:affectScaleZ", c.time, false);
    }
    params.rotationOrder = c.order;
    params.weight = c.weight;
    params.carry = c.carry;
    return RigExecApplyParentConstraint(c.inputFrame, *c.sources, params);
}

} // namespace

namespace evaluatorDetail {

const _ConstraintHandler *
_FindConstraintHandler(const TfToken &typeName)
{
    for (const _ConstraintHandler &handler : _ConstraintHandlers()) {
        if (typeName == handler.schemaType) {
            return &handler;
        }
    }
    return nullptr;
}

/// Source-blending constraints that revise one transform provider.
bool
_IsSourceFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler && handler->sourceFrame;
}

/// Every built-in constraint with fixed evaluator semantics.
bool
_IsFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler && handler->frameConstraint;
}

RigExecEulerOrder
_ParseConstraintEulerOrder(const TfToken &token)
{
    if (token == "XZY") return RigExecEulerOrder::XZY;
    if (token == "YXZ") return RigExecEulerOrder::YXZ;
    if (token == "YZX") return RigExecEulerOrder::YZX;
    if (token == "ZXY") return RigExecEulerOrder::ZXY;
    if (token == "ZYX") return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}

bool
_IsUsableConstraintFrame(const RigExecPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const GfVec3d &point : frame.points) {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2])) {
            return false;
        }
    }
    return true;
}

bool
_TokenIsOneOf(const TfToken &value,
              std::initializer_list<const char *> allowed)
{
    return std::any_of(
        allowed.begin(), allowed.end(),
        [&value](const char *candidate) { return value == candidate; });
}

/// Reads the per-axis mask for an operator's own channel group. The masks
/// are spelled by (group, axis) on the base class, so which triple to read is
/// a property of the operator, not of the call site.
RigExecConstraintAxisMask
_ReadGroupMask(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
               _ChannelGroup group, UsdTimeCode time)
{
    switch (group) {
    case _ChannelGroup::Translation:
        return _ReadConstraintAxisMask(
            resolved, prim, "inputs:affectTranslationX",
            "inputs:affectTranslationY", "inputs:affectTranslationZ", time);
    case _ChannelGroup::Rotation:
        return _ReadConstraintAxisMask(
            resolved, prim, "inputs:affectRotationX", "inputs:affectRotationY",
            "inputs:affectRotationZ", time);
    case _ChannelGroup::Scale:
        return _ReadConstraintAxisMask(
            resolved, prim, "inputs:affectScaleX", "inputs:affectScaleY",
            "inputs:affectScaleZ", time);
    case _ChannelGroup::All:
    case _ChannelGroup::None:
        break;
    }
    // Parent reads all three groups itself; the operators that honor no mask
    // are unmasked. Both want the all-true identity.
    return RigExecConstraintAxisMask();
}

} // namespace evaluatorDetail

// Pose interpolators (the conventional poseInterpolator)

bool
RigExecRigEvaluator::_CompilePoseInterpolators(
    const std::vector<SdfPath> &joints,
    const std::vector<SdfPath> &controls,
    std::vector<_PoseInterpolator> *out,
    std::vector<std::string> *notes,
    std::string *error, SdfPath *operation) const
{
    static const TfToken kPoseType("RigExecPose");
    static const TfToken kDriver("rigExec:driver");
    static const TfToken kWeight("outputs:weight");

    out->clear();
    std::vector<SdfPath> interpolators =
        _DiscoverPoseInterpolators(_stage, _rigPath);
    interpolators.erase(
        std::remove_if(interpolators.begin(), interpolators.end(),
                       [this](const SdfPath &path) {
                           return _IsSkippedOperation(path);
                       }),
        interpolators.end());
    if (interpolators.empty()) {
        return true;
    }

    // A driver has to be something that PUBLISHES A FRAME, because a frame is
    // the only thing this phase can read. A RigExecControl qualifies exactly
    // as a RigExecJoint does -- the conventional drivers are hidden joints, but a
    // control has a local rotation just as a joint does -- so the set is both.
    std::set<SdfPath> providers(joints.begin(), joints.end());
    providers.insert(controls.begin(), controls.end());

    for (const SdfPath &path : interpolators) {
        *operation = path;
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        _PoseInterpolator record;
        record.prim = path;

        // A NUMERIC driver reads properties instead of a frame: one to
        // three of them, in order, as the driver's position.
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:driverAttributes"))) {
            SdfPathVector values;
            rel.GetTargets(&values);
            for (const SdfPath &value : values) {
                if (!value.IsPropertyPath() ||
                    !_stage->GetAttributeAtPath(value)) {
                    *error = "pose interpolator " + path.GetString() +
                             " names a rigExec:driverAttributes target, " +
                             value.GetString() +
                             ", that is not an attribute of this stage";
                    return false;
                }
                record.driverAttributes.push_back(value);
            }
            if (record.driverAttributes.size() > 3) {
                *error = "pose interpolator " + path.GetString() + " names " +
                         std::to_string(record.driverAttributes.size()) +
                         " rigExec:driverAttributes; at most three are read, "
                         "one per axis of the position they stand for";
                return false;
            }
        }

        SdfPathVector driverTargets;
        if (const UsdRelationship rel = prim.GetRelationship(kDriver)) {
            rel.GetTargets(&driverTargets);
        }
        if (!record.driverAttributes.empty() && driverTargets.empty()) {
            // Numeric: there is no frame to measure, so there is no driver
            // prim to name and nothing asks for one.
        } else if (driverTargets.size() != 1) {
            *error = "pose interpolator " + path.GetString() + " names " +
                     std::to_string(driverTargets.size()) +
                     " rigExec:driver targets; exactly one is required";
            return false;
        }
        if (!driverTargets.empty()) {
            record.driver = driverTargets[0].GetPrimPath();
            if (!providers.count(record.driver)) {
                *error = "pose interpolator " + path.GetString() +
                         " names a rigExec:driver, " +
                         record.driver.GetString() +
                         ", that is not a RigExecJoint or RigExecControl of "
                         "this rig and therefore publishes no frame to "
                         "measure";
                return false;
            }
        }
        // The driver's local rotation is measured against its nearest
        // frame-publishing ancestor. That is the immediate namespace parent on
        // every rig this has been run on; anything else is REPORTED rather
        // than silently accepted, because it means the delta is being measured
        // across a prim that may carry a transform of its own.
        for (SdfPath walk = record.driver.IsEmpty()
                 ? SdfPath() : record.driver.GetParentPath();
             !walk.IsEmpty() && !walk.IsAbsoluteRootPath() &&
                 walk != _rigPath.GetParentPath();
             walk = walk.GetParentPath()) {
            if (providers.count(walk)) {
                record.driverParent = walk;
                break;
            }
        }
        if (notes && !record.driverParent.IsEmpty() &&
            record.driverParent != record.driver.GetParentPath()) {
            notes->push_back(
                "warning: pose interpolator " + path.GetString() +
                " measures its driver against " +
                record.driverParent.GetString() +
                ", which is not the driver's immediate namespace parent");
        }

        record.allowNegativeWeights =
            _ReadAttribute(prim, "rigExec:allowNegativeWeights", true);

        RigExecRbfSolverDesc desc;
        desc.kernel =
            _ReadAttribute(prim, "rigExec:kernel", TfToken("gaussian")) ==
                    TfToken("linear")
                ? RigExecRbfKernel::Linear
                : RigExecRbfKernel::Gaussian;
        desc.regularization =
            _ReadAttribute(prim, "rigExec:regularization", 0.0f);
        desc.normalize = _ReadAttribute(prim, "rigExec:normalize", true);
        desc.enableRotation =
            _ReadAttribute(prim, "rigExec:enableRotation", true);
        desc.enableTranslation =
            _ReadAttribute(prim, "rigExec:enableTranslation", false);
        const TfToken axis =
            _ReadAttribute(prim, "rigExec:twistAxis", TfToken("X"));
        desc.twistAxis = axis == TfToken("Y")   ? GfVec3d(0.0, 1.0, 0.0)
                         : axis == TfToken("Z") ? GfVec3d(0.0, 0.0, 1.0)
                                                : GfVec3d(1.0, 0.0, 0.0);
        if (!record.driverAttributes.empty()) {
            // The numeric driver IS the translation channel, and there is no
            // rotation to measure whatever the prim says.
            desc.enableTranslation = true;
            desc.enableRotation = false;
        }
        record.enableTranslation = desc.enableTranslation;

        std::vector<double> radii;
        std::vector<double> translationRadii;
        size_t poseChildren = 0;
        for (const UsdPrim &child : prim.GetChildren()) {
            if (child.GetTypeName() != kPoseType) {
                *error = "pose interpolator " + path.GetString() +
                         " has a child, " + child.GetPath().GetString() +
                         ", that is not a RigExecPose but a " +
                         child.GetTypeName().GetString();
                return false;
            }
            ++poseChildren;
            const SdfPath weightPath = child.GetPath().AppendProperty(kWeight);
            if (const UsdAttribute weight = child.GetAttribute(kWeight)) {
                // outputs:weight is a SOURCE. An authored outbound connection
                // on it would say it takes its value from somewhere else,
                // which is the opposite of what this phase does to it.
                if (weight.HasAuthoredConnections()) {
                    *error = "pose " + child.GetPath().GetString() +
                             " carries an authored connection on "
                             "outputs:weight, which its interpolator writes";
                    return false;
                }
            }
            if (!_ReadAttribute(child, "inputs:enabled", true)) {
                record.disabledPoseWeights.push_back(weightPath);
                continue;
            }

            const GfQuatf rotation =
                _ReadAttribute(child, "rigExec:rotation", GfQuatf(1.0f));
            desc.poses.push_back(RigExecRbfEulerFromQuaternion(
                GfQuatd(rotation.GetReal(),
                        GfVec3d(rotation.GetImaginary()))));
            // The schema stores centimetres, the solver takes metres.
            const GfVec3f translation =
                _ReadAttribute(child, "rigExec:translation", GfVec3f(0.0f));
            desc.translations.push_back(GfVec3d(translation[0] / 100.0,
                                                translation[1] / 100.0,
                                                translation[2] / 100.0));
            const TfToken kind =
                _ReadAttribute(child, "rigExec:poseType", TfToken("swing"));
            desc.poseTypes.push_back(
                kind == TfToken("twist")   ? RigExecRbfPoseType::Twist
                : kind == TfToken("whole") ? RigExecRbfPoseType::Whole
                                           : RigExecRbfPoseType::Swing);

            const double radius =
                _ReadAttribute(child, "rigExec:rotationRadius", 0.0f);
            const double translationRadius =
                _ReadAttribute(child, "rigExec:translationRadius", 0.0f);
            if (!std::isfinite(radius) || radius < 0.0 ||
                !std::isfinite(translationRadius) ||
                translationRadius < 0.0) {
                *error = "pose " + child.GetPath().GetString() +
                         " has a radius that is negative or not finite";
                return false;
            }
            radii.push_back(radius);
            // Centimetres here too, and NOT the rotation radius: a brow's
            // poses sit millimetres apart and a rotation width would swallow
            // every one of them.
            translationRadii.push_back(translationRadius / 100.0);
            record.poseWeights.push_back(weightPath);
        }
        if (poseChildren == 0) {
            *error = "pose interpolator " + path.GetString() +
                     " has no RigExecPose children";
            return false;
        }
        if (record.poseWeights.empty()) {
            // Every pose disabled is not an error -- it is a shape-preserving
            // enable applied to all of them -- and there is nothing to solve.
            out->push_back(std::move(record));
            continue;
        }
        if (!desc.enableTranslation) {
            desc.translations.clear();
            translationRadii.clear();
        }

        // The shared widths stay at zero and the PER-POSE ones are adopted: a
        // shipped table's widths carry a painted poseFalloff that no falloff
        // vector reproduces, so they are data, not something to re-derive
        // (rbf.h SetSolvedTable). The inverted matrix is the one thing that IS
        // re-derived, because it is a pure function of everything above and
        // storing it would be a second copy to keep in step (schema
        // RigExecPoseInterpolator).
        desc.radius = 0.0;
        desc.translationRadius = 0.0;
        RigExecRbfSolver solver(desc);
        solver.SetSolvedTable(radii, translationRadii, {});
        if (!solver.Solve()) {
            *error = "pose interpolator " + path.GetString() +
                     " could not be solved";
            return false;
        }
        if (notes && solver.Degenerate()) {
            notes->push_back(
                "warning: pose interpolator " + path.GetString() +
                " has poses that are coincident under the channels it has "
                "enabled; its weights will sit at 1/n");
        }
        record.solver = std::move(solver);
        out->push_back(std::move(record));
    }
    return true;
}

void
RigExecRigEvaluator::_EvaluatePoseInterpolators(
    UsdTimeCode time,
    const std::vector<RigExecPointFrame> &restFrames,
    const std::vector<char> &restLive,
    const std::vector<RigExecPointFrame> &finalFrames,
    const std::vector<char> &finalLive,
    RigExecRigPose *pose)
{
    if (_poseInterpolators.empty()) {
        return;
    }
    // Its own scope and its own category. A third of the evaluate was
    // invisible to the profiler until 2026-09-13, because nobody had scoped
    // the publish loops; this phase is not going to be the next one.
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PoseInterpolators", "psd");

    // ORDERING, ASSERTED RATHER THAN TRUSTED (first half; the second is at
    // the head of the geometry chains).
    // WHAT THIS PHASE MUST RUN AFTER: the complete pose walk. Not the pose
    // SEED -- the full pose, every constraint included, and the driver
    // constraints in particular. the conventional pose drivers are hidden joints
    // orient-constrained to the bone that carries everything, so that the
    // parent subtracts the twist back out and the driver's local rotation is
    // the swing alone; run this before that constraint and every driver reads
    // its seed, which is its rest, and every neutral weighs 1.000000 forever
    // however the rig is posed -- a failure that looks exactly like success.
    // The frame lookup below is what says so out loud: a driver with no
    // published FINAL frame has not been posed, and its interpolator
    // diagnoses and publishes zeros rather than quietly measuring a rest.
    const auto rotationOf =
        [&](const SdfPath &path, GfQuatd *out) {
            const auto fi = _providerIndex.find(path);
            if (fi == _providerIndex.end() || !restLive[fi->second]) {
                return false;
            }
            return RigExecFrameRotation(restFrames[fi->second], out);
        };
    const auto rotationOfFinal =
        [&](const SdfPath &path, GfQuatd *out) {
            const auto fi = _providerIndex.find(path);
            if (fi == _providerIndex.end() || !finalLive[fi->second]) {
                return false;
            }
            return RigExecFrameRotation(finalFrames[fi->second], out);
        };

    std::vector<double> weights;
    for (const _PoseInterpolator &interpolator : _poseInterpolators) {
        const UsdPrim prim = _stage->GetPrimAtPath(interpolator.prim);
        const bool enabled =
            _ResolvedRead(_resolvedInputs, prim, "inputs:enabled", true, time);

        // Publishes into BOTH: _resolvedInputs is what a consumer's read
        // resolves through (a RigExecBlendInput's inputs:weight follows its
        // single authored connection to <pose>.outputs:weight and finds the
        // value at that path), and movedProperties is what makes the number
        // observable to a host, a test and the picker.
        const auto publish = [&](const SdfPath &path, float value) {
            _resolvedInputs.SetProperty(path, VtValue(value));
            pose->movedProperties[path] = VtValue(value);
        };
        const auto publishAllZero = [&]() {
            for (const SdfPath &path : interpolator.poseWeights) {
                publish(path, 0.0f);
            }
        };
        // A disabled POSE publishes a hard zero whatever else happens: it was
        // left out of the solve, so it has no weight to be told.
        for (const SdfPath &path : interpolator.disabledPoseWeights) {
            publish(path, 0.0f);
        }
        if (!enabled) {
            // Shape-preserving enable: a corrective that is off has to be
            // off, not frozen at its last value.
            publishAllZero();
            continue;
        }

        GfQuatd driverFinal(1.0), driverRest(1.0);
        GfQuatd parentFinal(1.0), parentRest(1.0);
        const bool numeric = !interpolator.driverAttributes.empty();
        if (!numeric &&
            (!rotationOfFinal(interpolator.driver, &driverFinal) ||
            !rotationOf(interpolator.driver, &driverRest) ||
            (!interpolator.driverParent.IsEmpty() &&
             (!rotationOfFinal(interpolator.driverParent,
                          &parentFinal) ||
              !rotationOf(interpolator.driverParent,
                          &parentRest))))) {
            pose->diagnostics.push_back(
                "pose interpolator " + interpolator.prim.GetString() +
                " has no usable frame for its driver " +
                interpolator.driver.GetString() +
                " after the pose walk; its weights are zero this generation");
            publishAllZero();
            continue;
        }

        // The driver's LOCAL rotation relative to its own REST, which is what
        // every authored pose is measured from and why a rig standing still
        // reads its neutral at 1.000000.
        //   local = parent^-1 * world       (row-vector; see RigExecFrameRotation)
        //   delta = restLocal^-1 * local
        // Exactly the `neutral^-1 * pose` the authored quaternions were
        // rebased by (schema RigExecPose.rigExec:rotation), and therefore
        // directly comparable to them. The rest local is taken from the
        // EVALUATED rest frames rather than reconstructed from the driver's
        // authored rest:space: the two agree on a rig whose rest avars are
        // default, and the evaluated pair also carries the rest avars and any
        // intervening plain Xform, which the authored matrix alone does not.
        const GfQuatd local = parentFinal.GetInverse() * driverFinal;
        const GfQuatd restLocal = parentRest.GetInverse() * driverRest;
        const GfQuatd delta = (restLocal.GetInverse() * local).GetNormalized();

        // Through the euler, not around it: rbf_evaluate -- which is what
        // tools/biped/verify_psd.py computes its expected weights with --
        // takes an euler and converts it back inside Evaluate. Taking the same
        // route makes the gate's numbers and the engine's the same
        // floating-point values and not merely the same rotation.
        // The translation channel, when the interpolator measures one: the
        // driver's origin relative to its rest, in its rest frame
        // (RigExecFrameTranslation), in METRES because the solver's poses
        // were converted from the schema's centimetres at compile.
        GfVec3d translation(0.0);
        const GfVec3d *translationPtr = nullptr;
        if (numeric) {
            // The dial's own value, per axis, through the resolved inputs so
            // a property chain writing it (and an interactive override on
            // that chain's head) is what this reads.
            for (size_t i = 0; i < interpolator.driverAttributes.size(); ++i) {
                const SdfPath &at = interpolator.driverAttributes[i];
                const UsdAttribute attribute =
                    _stage->GetAttributeAtPath(at);
                double value = 0.0;
                float asFloat = 0.0f;
                if (_resolvedInputs.GetAttribute(attribute, time, &value) ||
                    attribute.Get(&value, time)) {
                    translation[int(i)] = value;
                } else if (_resolvedInputs.GetAttribute(attribute, time,
                                                        &asFloat) ||
                           attribute.Get(&asFloat, time)) {
                    translation[int(i)] = double(asFloat);
                }
            }
            translation /= 100.0;
            translationPtr = &translation;
        } else if (interpolator.enableTranslation) {
            const auto frameOfRest =
                [&](const SdfPath &at) -> const RigExecPointFrame * {
                    const auto fi = _providerIndex.find(at);
                    if (fi == _providerIndex.end() || !restLive[fi->second]) {
                        return nullptr;
                    }
                    return &restFrames[fi->second];
                };
            const auto frameOfFinal =
                [&](const SdfPath &at) -> const RigExecPointFrame * {
                    const auto fi = _providerIndex.find(at);
                    if (fi == _providerIndex.end() || !finalLive[fi->second]) {
                        return nullptr;
                    }
                    return &finalFrames[fi->second];
                };
            const bool parented = !interpolator.driverParent.IsEmpty();
            const RigExecPointFrame *pf =
                parented ? frameOfFinal(interpolator.driverParent) : nullptr;
            const RigExecPointFrame *pr =
                parented ? frameOfRest(interpolator.driverParent) : nullptr;
            const RigExecPointFrame *df = frameOfFinal(interpolator.driver);
            const RigExecPointFrame *dr = frameOfRest(interpolator.driver);
            if (!df || !dr ||
                !RigExecFrameTranslation(*df, *dr, pf, pr, &translation)) {
                pose->diagnostics.push_back(
                    "pose interpolator " + interpolator.prim.GetString() +
                    " could not measure its driver's translation; its "
                    "weights are zero this generation");
                publishAllZero();
                continue;
            }
            translation /= 100.0;
            translationPtr = &translation;
        }
        interpolator.solver.Evaluate(
            RigExecRbfEulerFromQuaternion(delta), translationPtr, &weights,
            interpolator.allowNegativeWeights);
        if (weights.size() != interpolator.poseWeights.size()) {
            pose->diagnostics.push_back(
                "pose interpolator " + interpolator.prim.GetString() +
                " solved " + std::to_string(weights.size()) +
                " weights for " +
                std::to_string(interpolator.poseWeights.size()) + " poses");
            publishAllZero();
            continue;
        }
        for (size_t i = 0; i < weights.size(); ++i) {
            // float, not double, and that is load-bearing: a consumer reads
            // inputs:weight as a float, and RigExecResolvedInputs::Get answers
            // only the type the VtValue actually holds -- a double here would
            // miss, fall on through the connection walk, and be answered with
            // the authored zero with nothing reported anywhere.
            publish(interpolator.poseWeights[i],
                    static_cast<float>(weights[i]));
        }
    }
}

std::vector<std::string>
RigExecRigEvaluator::_ApplyDerivedStartFrames()
{
    // Process-wide: two evaluators compiling concurrently (two characters
    // sharing one stage) must not interleave session-layer writes, and USD
    // layers are not thread-safe. Microseconds per compile.
    static std::mutex derivedOpinionsMutex;
    std::lock_guard<std::mutex> lock(derivedOpinionsMutex);

    static const TfToken startFrameRel("rigExec:startFrame");
    static const TfToken startFramePolicy("rigExec:startFramePolicy");
    static const TfToken jointsRel("rigExec:joints");
    static const TfToken jointType("RigExecJoint");
    static const TfToken controlType("RigExecControl");
    static const TfToken policyNone("none");
    static const TfToken policyParent("parent");

    SdfLayerHandle session = _stage->GetSessionLayer();
    // Collected, not emitted: nothing that can Send -- TF_WARN included --
    // may run inside the notice block below, so the caller emits these
    // after this returns (both blocks then closed).
    std::vector<std::string> warnings;
    auto warn = [&warnings](const std::string &message) {
        warnings.push_back(message);
    };

    // NO NOTICE may leave this function. It runs inside Compile, and the
    // imaging registry registers its ObjectsChanged listener BEFORE
    // Compile and holds a non-recursive mutex across the whole call, so a
    // notice fired here re-enters the registry on its own held mutex
    // (measured: an access violation in _OnObjectsChanged on the first
    // usdview activation of a policy-carrying rig). TfNotice::Block
    // swallows the send while the ChangeBlock still batches the Sdf-side
    // work. Sound because the in-flight compile is the only reader of
    // these opinions: the structure digest runs after this returns, and
    // every Compile rebuilds the epoch (fresh taps) rather than
    // invalidating the old one, so no cached exec value can strand on the
    // swallowed send. Block is thread-scoped, and this runs
    // single-threaded, before the digest dispatch.
    TfNotice::Block noticeBlock;
    SdfChangeBlock block;
    // Retract this evaluator's previous opinions first, surgically: only
    // OUR tracked provider leaves each session list, so a hand-authored
    // session opinion on the same relationship survives. Anything still
    // composed afterwards is not ours, which is exactly the "authored
    // wins" test the derivation below applies.
    for (const auto &[solverPath, ourProvider] : _derivedStartFrames) {
        SdfRelationshipSpecHandle spec = session->GetRelationshipAtPath(
            solverPath.AppendProperty(startFrameRel));
        if (!spec) {
            continue;
        }
        bool present = false;
        // Explicit items only: our writes are SetTargets (explicit), and
        // a user's prepended/appended opinions are not ours to inspect.
        const auto explicitItems =
            spec->GetTargetPathList().GetExplicitItems();
        for (size_t i = 0, n = explicitItems.size(); i < n; ++i) {
            if (explicitItems[i] == ourProvider) {
                present = true;
                break;
            }
        }
        if (!present) {
            continue;
        }
        spec->RemoveTargetPath(ourProvider);
        if (!spec->HasTargetPathList()) {
            if (SdfPrimSpecHandle primSpec =
                    session->GetPrimAtPath(solverPath)) {
                primSpec->RemoveProperty(spec);
            }
        }
    }
    _derivedStartFrames.clear();

    UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (rig) {
        UsdEditContext sessionCtx(_stage, session);
        for (const UsdPrim &prim : UsdPrimRange(rig)) {
            if (prim.GetTypeName() != "RigExecFkChain" ||
                _IsSkippedOperation(prim.GetPath())) {
                continue;
            }
            TfToken policy;
            prim.GetAttribute(startFramePolicy).Get(&policy);
            if (policy.IsEmpty()) {
                policy = policyNone;
            }
            if (policy == policyNone) {
                continue;
            }
            const SdfPath solverPath = prim.GetPath();
            if (policy != policyParent) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy '" +
                     policy.GetString() +
                     "', expected 'none' or 'parent'; solving absolute.");
                continue;
            }
            SdfPathVector composed;
            prim.GetRelationship(startFrameRel).GetTargets(&composed);
            if (!composed.empty()) {
                continue;  // Authored (asset or session) always wins.
            }
            SdfPathVector joints;
            prim.GetRelationship(jointsRel).GetTargets(&joints);
            if (joints.empty()) {
                continue;  // A jointless chain hangs from nothing.
            }
            // The inference: nearest namespace ancestor of the chain's
            // joints that is a joint or control. Structural ancestry
            // only -- GetParentPath, never a name -- so a reparented
            // chain follows its new parent with no authoring change.
            UsdPrim first =
                _stage->GetPrimAtPath(joints[0].GetPrimPath());
            SdfPath provider;
            for (SdfPath a = first ? first.GetPath().GetParentPath()
                                   : SdfPath::EmptyPath();
                 !a.IsEmpty() && a != SdfPath::AbsoluteRootPath();
                 a = a.GetParentPath()) {
                if (a == _rigPath) {
                    break;
                }
                const UsdPrim ancestor = _stage->GetPrimAtPath(a);
                if (!ancestor) {
                    continue;
                }
                const TfToken type = ancestor.GetTypeName();
                if (type == jointType || type == controlType) {
                    provider = a;
                    break;
                }
            }
            if (provider.IsEmpty()) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy 'parent' but no "
                     "RigExecJoint/RigExecControl ancestor; solving "
                     "absolute.");
                continue;
            }
            bool shared = true;
            for (const SdfPath &j : joints) {
                const SdfPath jp = j.GetPrimPath();
                if (jp == provider || !jp.HasPrefix(provider)) {
                    shared = false;
                    break;
                }
            }
            if (!shared) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy 'parent' but its "
                     "joints span providers; solving absolute.");
                continue;
            }
            prim.GetRelationship(startFrameRel)
                .SetTargets(SdfPathVector{provider});
            _derivedStartFrames[solverPath] = provider;
        }
    }
    return warnings;
}

// Whether any of \p providers has a rest channel that can no longer be held
// as an epoch constant.
// Asked only for the providers the rest gate saw a notice reach, never per
// frame: the answer is a function of the provider's own seven rest
// attributes and of the epoch's property chains, the chains change only with
// a recompile, and the attributes change only through a notice on one of the
// paths the gate watches.
bool
RigExecRigEvaluator::_EpochRestsMightVary(
    const std::set<SdfPath> &providers) const
{
    std::set<SdfPath> chainTargets;
    for (const auto &[target, revisions] : _propertyChains) {
        chainTargets.insert(target);
    }
    for (const SdfPath &provider : providers) {
        if (_ProviderRestMightVary(_stage, provider, chainTargets)) {
            return true;
        }
    }
    return false;
}

// The rest gate.
// The epoch's rest paths are p.rest:space and the six rest avars for every
// provider p the rest request pulls -- every provider and every RigExec
// ancestor of one, since seedProvider climbs to the root -- and
// computeRestFrame reads exactly those seven names on a provider and on its
// namespace ancestor, so no other attribute can move an epoch rest. A notice
// reaches them in three ways, and each adds the providers it reached:
//   * a resync or changed-info on one of the paths itself, which is how a
//     value, a first spec in a layer, a connection or a time sample arrives;
//   * a resync of a prim at or above a provider, which is how a reference,
//     payload, variant, retype, reparent or deactivation arrives;
//   * a resync of the pseudo-root, which is how a sublayer or a layer mute
//     arrives, and reaches every provider as the case above.
// A resolved-asset resync is read as a resync as well: it names prims the
// same way, and reading it costs nothing.
// Membership is tested as "a rest name on a key of _restTapIds", which is the
// same set without spelling out seven paths per provider at every commit.
// The keys are the providers the compile's type list recognizes, but
// computeRestFrame is inherited -- a RigExecCurvenetAdjustment is a
// RigExecControl to exec -- so a namespace ancestor that publishes a rest
// frame need not be a key. A rest name edited on a prim that is not a key
// but has keys under it therefore marks the rests stale too: one of those
// keys can read it through its namespace-ancestor input. It reaches no
// provider's OWN channels, so it adds nothing to re-classify.
void
RigExecRigEvaluator::_NoteRestEdits(const UsdNotice::ObjectsChanged &notice)
{
    if (_restTapIds.empty()) {
        return;
    }
    bool reached = false;
    const auto noteProperty = [&](const SdfPath &path) {
        if (!path.IsPrimPropertyPath() ||
            !_IsRestInputName(path.GetNameToken())) {
            return;
        }
        const SdfPath prim = path.GetPrimPath();
        // A subtree is one contiguous run of the ordered map, starting at
        // the prim itself when the prim is a key.
        const auto first = _restTapIds.lower_bound(prim);
        if (first == _restTapIds.end() || !first->first.HasPrefix(prim)) {
            return;
        }
        if (first->first == prim) {
            _restEditedProviders.insert(prim);
        }
        reached = true;
    };
    const auto noteResync = [&](const SdfPath &path) {
        if (!path.IsAbsoluteRootOrPrimPath()) {
            noteProperty(path);
            return;
        }
        // Every provider at or under the resynced prim. A subtree is one
        // contiguous run of the ordered map, starting at the prim itself.
        for (auto provider = _restTapIds.lower_bound(path);
             provider != _restTapIds.end() &&
             provider->first.HasPrefix(path); ++provider) {
            _restEditedProviders.insert(provider->first);
            reached = true;
        }
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        noteResync(path);
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        noteResync(path);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        noteProperty(path);
    }
    if (reached) {
        _epochRestFramesStale = true;
    }
}

bool
RigExecRigEvaluator::_RefreshEpochRestFrames()
{
    if (!_restTaps || _restTapIds.empty()) {
        _epochRestFramesStale = false;
        return true;
    }
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "RestFramesRefresh", "evaluate");
    const RigExecSnapshot rests = _restTaps->Evaluate(_restTime);
    if (!rests.IsValid() || !rests.IsComplete()) {
        return false;
    }
    for (const auto &[provider, tap] : _restTapIds) {
        _epochRestFrames[provider] = rests.Get<RigExecPointFrame>(tap);
    }
    _epochRestFramesStale = false;
    return true;
}

bool
RigExecRigEvaluator::_ComposeInterveningXforms(
    const UsdPrim &assetRoot,
    UsdGeomXformCache *xformCache,
    std::vector<RigExecPointFrame> *restFrames,
    std::vector<char> *restLive,
    std::map<SdfPath, RigExecPointFrame> *baseFrames,
    std::vector<RigExecPointFrame> *finalFrames,
    std::vector<char> *finalLive,
    RigExecRigPose *pose) const
{
    if (!assetRoot || !xformCache) {
        return true;
    }
    // No provider has anything standing between it and its anchor, so there
    // is no transform to compose and nothing to ask the xform cache.
    if (_interveningXformProviders.empty()) {
        return true;
    }

    // X(P) per provider, and which provider (if any) anchors it. The anchor
    // is the nearest RigExec ancestor -- exec has already folded that one's
    // rest:space and avars in -- and the asset root otherwise. Only a
    // provider with an intervening prim can have a non-identity X(P); the
    // rest are anchored to their own parent.
    std::map<SdfPath, GfMatrix4d> intervening;
    const std::map<SdfPath, SdfPath> &anchorOf = _poseProviderAnchors;
    bool anyIntervening = false;
    for (const auto &[provider, tap] : _firstFramePoseFrames) {
        intervening[provider] = GfMatrix4d(1.0);
    }
    for (const SdfPath &provider : _interveningXformProviders) {
        const SdfPath &anchorPath = anchorOf.at(provider);
        const UsdPrim anchor = anchorPath.IsEmpty()
            ? assetRoot : _stage->GetPrimAtPath(anchorPath);
        const UsdPrim parent = _stage->GetPrimAtPath(provider.GetParentPath());
        GfMatrix4d x(1.0);
        if (parent && anchor && parent != anchor) {
            bool resetsBelowAnchor = false;
            x = xformCache->ComputeRelativeTransform(
                parent, anchor, &resetsBelowAnchor);
            if (resetsBelowAnchor) {
                // !resetXformStack! detaches the provider from the anchor
                // entirely, so "relative to the anchor" is not a quantity
                // that exists. Reported rather than composed: guessing here
                // would place the provider somewhere nobody asked for.
                pose->diagnostics.push_back(
                    "resetXformStack between " + anchor.GetPath().GetString() +
                    " and " + provider.GetString() +
                    "; the intervening transform is not composed");
                x = GfMatrix4d(1.0);
            }
        }
        intervening[provider] = x;
        anyIntervening = anyIntervening || x != GfMatrix4d(1.0);
    }
    // The overwhelmingly common rig has no such Xform anywhere, and must not
    // pay a frame rebuild for the ones that do.
    if (!anyIntervening) {
        return true;
    }

    // Parents before children, so an anchor is already corrected when the
    // providers under it are reached. Ordered by path element COUNT rather
    // than by SdfPath's own ordering, which makes no parent-first promise.
    std::vector<SdfPath> ordered;
    ordered.reserve(intervening.size());
    for (const auto &[provider, x] : intervening) {
        ordered.push_back(provider);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
                     [](const SdfPath &a, const SdfPath &b) {
                         return a.GetPathElementCount() <
                                b.GetPathElementCount();
                     });

    // The uncorrected matrices, captured before anything is overwritten. The
    // correction divides a provider by its anchor's OLD value to recover its
    // own local factor, so reading the anchor after correcting it would
    // divide by the answer instead of by the question.
    auto toMatrix = [](const RigExecPointFrame &frame, GfMatrix4d *out) {
        if (!frame.IsValid() || frame.IsDegenerate()) {
            return false;
        }
        return RigExecPointsToMatrix(
            RigExecIdentityLandmarks(), frame.points, out);
    };
    std::map<SdfPath, GfMatrix4d> execRest, execBase;
    for (const SdfPath &provider : ordered) {
        GfMatrix4d m(1.0);
        {
            const auto ri = _providerIndex.find(provider);
            if (ri != _providerIndex.end() && (*restLive)[ri->second] &&
                toMatrix((*restFrames)[ri->second], &m))
                execRest[provider] = m;
        }
        m = GfMatrix4d(1.0);
        if (toMatrix(baseFrames->at(provider), &m)) execBase[provider] = m;
    }

    auto correctDense = [&](std::vector<RigExecPointFrame> *frames,
                            std::vector<char> *live,
                            const std::map<SdfPath, GfMatrix4d> &exec,
                            const SdfPath &provider) {
        const auto own = exec.find(provider);
        if (own == exec.end()) {
            return;
        }
        const int fi = _providerIndex.at(provider);
        const SdfPath &anchorPath = anchorOf.at(provider);
        GfMatrix4d anchorExec(1.0), anchorTrue(1.0);
        if (!anchorPath.IsEmpty()) {
            const auto execIt = exec.find(anchorPath);
            if (execIt != exec.end()) {
                anchorExec = execIt->second;
            }
            const auto ai = _providerIndex.find(anchorPath);
            GfMatrix4d corrected(1.0);
            if (ai != _providerIndex.end() && (*live)[ai->second] &&
                toMatrix((*frames)[ai->second], &corrected)) {
                anchorTrue = corrected;
            }
        }
        (*frames)[fi] = RigExecFrameFromMatrix(
            own->second * anchorExec.GetInverse() *
            intervening.at(provider) * anchorTrue);
        (*live)[fi] = 1;
    };
    auto correct = [&](std::map<SdfPath, RigExecPointFrame> *frames,
                       const std::map<SdfPath, GfMatrix4d> &exec,
                       const SdfPath &provider) {
        const auto own = exec.find(provider);
        if (own == exec.end()) {
            return;
        }
        const SdfPath &anchorPath = anchorOf.at(provider);
        GfMatrix4d anchorExec(1.0), anchorTrue(1.0);
        if (!anchorPath.IsEmpty()) {
            const auto execIt = exec.find(anchorPath);
            if (execIt != exec.end()) {
                anchorExec = execIt->second;
            }
            GfMatrix4d corrected(1.0);
            if (toMatrix(frames->at(anchorPath), &corrected)) {
                anchorTrue = corrected;
            }
        }
        (*frames)[provider] = RigExecFrameFromMatrix(
            own->second * anchorExec.GetInverse() *
            intervening.at(provider) * anchorTrue);
    };

    for (const SdfPath &provider : ordered) {
        correctDense(restFrames, restLive, execRest, provider);
        correct(baseFrames, execBase, provider);
        // base and final are the same frame at seeding time; final is
        // reassigned rather than corrected again so the two cannot drift.
        const int fi = _providerIndex.at(provider);
        (*finalFrames)[fi] = baseFrames->at(provider);
        (*finalLive)[fi] = 1;
    }
    return true;
}

// One per-frame array of constraint source parameters, read RAW.
// Straight off the attribute at the frame's time: no connection walk, no
// resolved-input lookup, no interactive override. A source weight is an
// input of the constraint operator, not of the rig, and the evaluator and
// the program have to read it the same way -- so both read it here, and the
// cardinality diagnostic has one wording rather than one per caller.
// An absent or empty array is not a failure: it means the neutral value on
// every source, which is what an unauthored blend has always meant.
bool
RigExecRigEvaluator::_ReadConstraintSourceWeights(
    const UsdPrim &prim,
    const char *name,
    size_t count,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    std::vector<double> *weights)
{
    VtFloatArray authored;
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&authored, time);
    }
    if (!authored.empty() && authored.size() != count) {
        diagnostics->push_back(
            prim.GetPath().GetString() + " " + name + " has " +
            std::to_string(authored.size()) + " entries for " +
            std::to_string(count) + " sources");
        return false;
    }
    weights->assign(count, 1.0);
    for (size_t i = 0; i < authored.size(); ++i) {
        (*weights)[i] = authored[i];
    }
    return true;
}

// The same read for a per-source offset array, whose neutral value is zero.
bool
RigExecRigEvaluator::_ReadConstraintSourceOffsets(
    const UsdPrim &prim,
    const char *name,
    size_t count,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    std::vector<GfVec3d> *offsets)
{
    VtVec3dArray authored;
    if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
        a.Get(&authored, time);
    }
    if (!authored.empty() && authored.size() != count) {
        diagnostics->push_back(
            prim.GetPath().GetString() + " " + name + " has " +
            std::to_string(authored.size()) + " entries for " +
            std::to_string(count) + " sources");
        return false;
    }
    offsets->assign(count, GfVec3d(0));
    for (size_t i = 0; i < authored.size(); ++i) {
        (*offsets)[i] = authored[i];
    }
    return true;
}

// `neverTS` retains current rotations/root placement while rebuilding
// child placement and handle lengths from rest frames. A joint without
// any authored rest transform has the schema's identity fallback, which
// is not an actual chain rest layout; use its current static layout in
// that case. The public math solver can therefore keep measuring its
// input chain; evaluator-side preparation decides whether those
// measurements are rest- or animation-derived.
bool
RigExecPrepareRestDerivedIkChain(
    const std::vector<RigExecPointFrame> &current,
    const std::vector<RigExecPointFrame> &rest,
    std::vector<RigExecPointFrame> *prepared)
{
    if (current.size() != rest.size() || current.empty()) {
        return false;
    }
    bool usableRestLayout = true;
    for (size_t i = 1; i < rest.size(); ++i) {
        const double segmentLength =
            (rest[i].Origin() - rest[i - 1].Origin()).GetLength();
        if (!std::isfinite(segmentLength) || segmentLength <= 0.0) {
            usableRestLayout = false;
            break;
        }
    }
    const std::vector<RigExecPointFrame> &lengthReference =
        usableRestLayout ? rest : current;
    prepared->clear();
    prepared->reserve(current.size());
    for (size_t i = 0; i < current.size(); ++i) {
        if (!_IsUsableConstraintFrame(current[i]) ||
            !_IsUsableConstraintFrame(rest[i])) {
            return false;
        }
        GfVec3d origin = current[i].Origin();
        if (i > 0) {
            GfMatrix4d parentRest(1.0), parentPrepared(1.0);
            if (!RigExecPointsToMatrix(
                    RigExecIdentityLandmarks(),
                    lengthReference[i - 1].points,
                    &parentRest) ||
                !RigExecPointsToMatrix(
                    RigExecIdentityLandmarks(), prepared->back().points,
                    &parentPrepared)) {
                return false;
            }
            origin = parentPrepared.TransformAffine(
                parentRest.GetInverse().TransformAffine(
                    lengthReference[i].Origin()));
        }

        RigExecPointFrame frame = current[i];
        frame.points[0] = origin;
        for (size_t axis = 1; axis < frame.points.size(); ++axis) {
            GfVec3d direction =
                current[i].points[axis] - current[i].Origin();
            const double directionLength = direction.GetLength();
            const double length =
                (lengthReference[i].points[axis] -
                 lengthReference[i].Origin()).GetLength();
            if (!std::isfinite(length) || length <= 0.0 ||
                !std::isfinite(directionLength) ||
                directionLength <= 0.0) {
                return false;
            }
            direction /= directionLength;
            frame.points[axis] = origin + direction * length;
        }
        if (!_IsUsableConstraintFrame(frame)) {
            return false;
        }
        prepared->push_back(frame);
    }
    return true;
}

// The pieces of the pose walk that are not the walk: frames read off the
// stage, the deltas a native source rides, the placements a commit
// republishes. Each one is called from the dynamic walk below and is written
// to be callable from the baked program over its dense slots, because a
// second implementation of any of them is a second answer.

bool
RigExecRigEvaluator::_FrameFromXformRelativeToAsset(
    const UsdPrim &assetRoot,
    UsdGeomXformCache *xformCache,
    const SdfPath &path,
    RigExecPointFrame *outFrame,
    GfMatrix4d *outMatrix) const
{
    const UsdPrim prim = _stage->GetPrimAtPath(path);
    if (!prim || !assetRoot || !UsdGeomXformable(prim)) {
        return false;
    }
    bool resetsBelowAsset = false;
    const GfMatrix4d relative =
        xformCache->ComputeRelativeTransform(prim, assetRoot,
                                             &resetsBelowAsset);
    if (outFrame) {
        *outFrame = RigExecFrameFromMatrix(relative);
    }
    if (outMatrix) {
        *outMatrix = relative;
    }
    return true;
}

bool
RigExecApplyRevisedAncestorDelta(
    const SdfPath &xformPath,
    const RigExecPoseFrameEnumerator &providers,
    RigExecPointFrame *frame)
{
    // A native source that is not itself a written provider may still
    // sit beneath a constrained transform provider. The closest
    // revised ancestor contains all higher ancestor deltas, so apply
    // it once to the stage-derived source frame.
    // The comparison is over POINTS and not whole frames: a provider whose
    // flags differ from its base while its points do not has not moved, and
    // comparing the frames would make it the closest revised ancestor and
    // ride the source on an identity that is not one.
    SdfPath closest;
    RigExecPointFrame closestBase, closestCurrent;
    auto select = [&](const SdfPath &provider,
                      const RigExecPointFrame &base,
                      const RigExecPointFrame &current) {
        if (provider == xformPath || !xformPath.HasPrefix(provider) ||
            current.points == base.points) {
            return;
        }
        if (closest.IsEmpty() ||
            provider.GetPathElementCount() >
                closest.GetPathElementCount()) {
            closest = provider;
            closestBase = base;
            closestCurrent = current;
        }
    };
    providers(select);
    if (!closest.IsEmpty()) {
        GfMatrix4d delta(1.0);
        if (!RigExecPointsToMatrix(
                closestBase.points, closestCurrent.points, &delta)) {
            return false;
        }
        *frame = RigExecMatrixToPoints(frame->points, delta);
    }
    return frame->IsValid();
}

bool
RigExecRigEvaluator::_ResolveNativeXformSource(
    const UsdPrim &assetRoot,
    UsdGeomXformCache *xformCache,
    const SdfPath &xformPath,
    const RigExecPoseFrameEnumerator &providers,
    RigExecPointFrame *out) const
{
    if (!_FrameFromXformRelativeToAsset(assetRoot, xformCache, xformPath, out,
                                        nullptr) ||
        !out->IsValid()) {
        return false;
    }
    return RigExecApplyRevisedAncestorDelta(xformPath, providers, out);
}

void
RigExecRigEvaluator::_UpdateVolumePlacements(
    const RigExecPoseFrameLookup &finalFrameOf,
    RigExecRigPose *pose)
{
    _volumeWeightMatrices.clear();
    for (const auto &[path, tap] : _volumeWeightMatrixTaps) {
        RigExecPointFrame frame;
        GfMatrix4d placement(1.0);
        if (finalFrameOf(path, &frame) && _IsUsableConstraintFrame(frame)) {
            RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                  frame.points, &placement);
        }
        _volumeWeightMatrices[path] = placement;
    }
    pose->weightFrames = _volumeWeightMatrices;
}

bool
RigExecRigEvaluator::_IkUsesAnimatedTs(const std::vector<SdfPath> &chain) const
{
    for (const SdfPath &path : chain) {
        if (_jointSolverBinding.count(path)) {
            return true;
        }
        const UsdPrim joint = _stage->GetPrimAtPath(path);
        for (const char *name : {
                 "posed:space", "avars:tx", "avars:ty", "avars:tz",
                 "avars:sx", "avars:sy", "avars:sz"}) {
            const UsdAttribute attr = joint.GetAttribute(TfToken(name));
            SdfPathVector connections;
            // HasAuthoredConnections first: see _AuthoredConnections.
            if (attr &&
                (attr.GetNumTimeSamples() > 0 ||
                 (attr.HasAuthoredConnections() &&
                  attr.GetConnections(&connections) &&
                  !connections.empty()))) {
                return true;
            }
        }
    }
    return false;
}

const std::vector<TfToken> &
RigExecRigEvaluator::GetConstraintOperatorTypeNames()
{
    // Built from the table itself rather than written out again, so an
    // operator added there is named here without anyone remembering to.
    static const std::vector<TfToken> names = [] {
        std::vector<TfToken> types;
        types.reserve(_ConstraintHandlers().size());
        for (const _ConstraintHandler &handler : _ConstraintHandlers()) {
            types.push_back(TfToken(handler.schemaType));
        }
        return types;
    }();
    return names;
}

bool
RigExecRigEvaluator::IsConstraintOperatorType(const TfToken &schemaType)
{
    return _FindConstraintHandler(schemaType) != nullptr;
}

size_t
RigExecConstraintHandlerCount(const TfToken &schemaType)
{
    return _FindConstraintHandler(schemaType) ? 1 : 0;
}

size_t
RigExecConstraintHandlerTotal()
{
    return _ConstraintHandlers().size();
}

bool
RigExecConstraintUsesRotationOrder(const TfToken &schemaType)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(schemaType);
    return handler && handler->usesRotationOrder;
}

} // namespace rigExec

// RigExec mover registry storage and shared stage-reading helpers.
#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "pxr/base/plug/plugin.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include "pxr/usd/usdSkel/blendShape.h"

#include <cmath>
#include <deque>
#include <mutex>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

TF_DEFINE_PUBLIC_TOKENS(RigExecMoverExecTokens, RIGEXEC_MOVER_EXEC_TOKENS);

namespace {

struct _Entry {
    explicit _Entry(RigExecMoverHandler value)
        : schemaType(value.schemaType), handler(std::move(value))
    {
        handler.schemaType = schemaType.c_str();
    }
    std::string schemaType;
    RigExecMoverHandler handler;
};

struct _Registry {
    // Deque insertion keeps previously returned handler pointers valid.
    std::deque<_Entry> rows;
    std::mutex mutex;
};

_Registry &
_GetRegistry()
{
    static _Registry registry;
    return registry;
}

struct _PluginLoader {
    std::recursive_mutex mutex;
    bool discovered = false;
};

_PluginLoader &
_GetPluginLoader()
{
    static _PluginLoader loader;
    return loader;
}

const RigExecMoverHandler *
_Find(const TfToken &schemaType)
{
    _Registry &registry = _GetRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    for (const _Entry &entry : registry.rows) {
        if (schemaType == entry.handler.schemaType) {
            return &entry.handler;
        }
    }
    return nullptr;
}

}  // namespace

bool
RigExecRegisterMoverHandler(RigExecMoverHandler handler, std::string *error)
{
    const auto fail = [error](const std::string &message) {
        if (error) {
            *error = message;
        } else {
            TF_WARN("%s", message.c_str());
        }
        return false;
    };
    if (!handler.schemaType || !*handler.schemaType || !handler.resolveOp) {
        return fail("Mover registration requires a schema type and resolveOp");
    }
    const bool external =
        handler.resolveOp(TfToken()) == RigExecRevisionOp::External;
    if (external || handler.assembleExternal || handler.applyExternal) {
        if (!external || handler.domain != RigExecMoverDomain::Points ||
            !handler.assembleExternal || !handler.applyExternal) {
            return fail(std::string(handler.schemaType) +
                ": external movers require the External points operation "
                "and both assembleExternal and applyExternal callbacks");
        }
        if (!handler.oracle) {
            handler.hasScalarOracle = false;
        }
    }
    if ((handler.encodeExternal || handler.runtimeKernel.prepare ||
         handler.runtimeKernel.apply) &&
        (!external || (handler.runtimeKernel.prepare != nullptr) !=
                          (handler.runtimeKernel.apply != nullptr))) {
        return fail(std::string(handler.schemaType) +
            ": .rigexec export and playback callbacks belong to external "
            "movers, and a playback kernel needs both prepare and apply");
    }
    _Registry &registry = _GetRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    for (const _Entry &entry : registry.rows) {
        if (entry.schemaType == handler.schemaType) {
            return fail("Mover already registered: " + entry.schemaType);
        }
    }
    registry.rows.emplace_back(std::move(handler));
    return true;
}

bool
RigExecLoadMoverPlugins(std::vector<std::string> *diagnostics)
{
    // Plug::Load can run registration code which itself performs a lookup.
    // Do not hold the row mutex across library initialization or recursively
    // ask Plug to load the library whose initializer is running.
    _PluginLoader &loader = _GetPluginLoader();
    static thread_local bool loading = false;
    std::lock_guard<std::recursive_mutex> lock(loader.mutex);
    if (loading) {
        return true;
    }
    struct _LoadingGuard {
        bool &flag;
        explicit _LoadingGuard(bool &value) : flag(value) { flag = true; }
        ~_LoadingGuard() { flag = false; }
    } guard(loading);
    loader.discovered = true;
    bool ok = true;
    for (const PlugPluginPtr &plugin :
             PlugRegistry::GetInstance().GetAllPlugins()) {
        const JsObject metadata = plugin->GetMetadata();
        const auto version = metadata.find("RigExecMoverPlugin");
        if (version == metadata.end()) {
            continue;
        }
        std::string message;
        if (!version->second.IsInt() ||
            version->second.GetInt() != RigExecMoverPluginApiVersion) {
            message = "Mover plugin " + plugin->GetName() +
                " has an incompatible RigExecMoverPlugin API version; "
                "expected " + std::to_string(RigExecMoverPluginApiVersion);
        } else if (!plugin->Load()) {
            message = "Could not load mover plugin " + plugin->GetName();
        }
        if (!message.empty()) {
            ok = false;
            if (diagnostics) {
                diagnostics->push_back(std::move(message));
            } else {
                TF_WARN("%s", message.c_str());
            }
        }
    }
    return ok;
}

const RigExecMoverHandler *
RigExecFindMoverHandler(const TfToken &schemaType)
{
    if (const RigExecMoverHandler *handler = _Find(schemaType)) {
        return handler;
    }
    _PluginLoader &loader = _GetPluginLoader();
    std::lock_guard<std::recursive_mutex> lock(loader.mutex);
    if (!loader.discovered) {
        RigExecLoadMoverPlugins();
    }
    return _Find(schemaType);
}

std::vector<RigExecMoverHandler>
RigExecMoverHandlers()
{
    RigExecLoadMoverPlugins();
    _Registry &registry = _GetRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    std::vector<RigExecMoverHandler> result;
    result.reserve(registry.rows.size());
    for (const _Entry &entry : registry.rows) {
        result.push_back(entry.handler);
    }
    return result;
}

SdfPathVector
RigExecRelationshipTargets(const UsdPrim &prim, const char *rel)
{
    SdfPathVector targets;
    if (const UsdRelationship r = prim.GetRelationship(TfToken(rel))) {
        r.GetTargets(&targets);
    }
    return targets;
}

RigExecReadPhase
RigExecPhaseForInput(const UsdPrim &moverPrim, const char *rel)
{
    RigExecReadPhase phase;
    if (const UsdRelationship r = moverPrim.GetRelationship(TfToken(rel))) {
        RigExecResolveReadPhase(r, &phase, nullptr);
    }
    return phase;
}

VtValue
RigExecPhasedConsumerValue(const VtValue &chainValue,
                           const SdfValueTypeName &consumerType)
{
    if (consumerType == SdfValueTypeNames->Double &&
        chainValue.IsHolding<float>()) {
        return VtValue(double(chainValue.UncheckedGet<float>()));
    }
    if (consumerType == SdfValueTypeNames->Float &&
        chainValue.IsHolding<double>()) {
        return VtValue(float(chainValue.UncheckedGet<double>()));
    }
    return chainValue;
}

void
RigExecReadPhasedPoints(
    const RigExecMoverOracleContext &ctx,
    const char *relName,
    const SdfPath &pointsPath,
    VtVec3fArray *out)
{
    const RigExecReadPhase phase = RigExecPhaseForInput(ctx.prim, relName);
    // The reader's own record is made after it reads, so no snapshot holds
    // `preceding` on its own chain; the points entering it are that value.
    if (phase.kind == RigExecReadPhaseKind::Preceding &&
        pointsPath == ctx.target && ctx.entering) {
        *out = *ctx.entering;
        return;
    }
    if (!phase.IsBase()) {
        if (const VtValue *v = ctx.snapshots.Lookup(
                pointsPath, phase, ctx.moverPath)) {
            if (v->IsHolding<VtVec3fArray>()) {
                *out = v->UncheckedGet<VtVec3fArray>();
                return;
            }
        }
    }
    if (const UsdAttribute a = ctx.stage->GetAttributeAtPath(pointsPath)) {
        a.Get(out, ctx.time);
    }
}

bool
RigExecIsTransformDomainAmbiguous(
    const UsdStageRefPtr &stage, const SdfPath &target)
{
    const UsdPrim prim = stage->GetPrimAtPath(target);
    return prim && prim.IsA<UsdGeomPointBased>();
}

std::string
RigExecPointsTargetHint(
    const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (!target.IsPrimPath()) {
        return std::string();
    }
    const UsdPrim prim = stage->GetPrimAtPath(target);
    if (!prim || !prim.IsA<UsdGeomPointBased>()) {
        return std::string();
    }
    return ". Did you mean " +
           target.AppendProperty(TfToken("points")).GetString() + "?";
}

bool
RigExecValidateMoverTargets(
    const UsdStageRefPtr &stage,
    const UsdPrim &prim,
    const RigExecMoverHandler *handler,
    const std::vector<SdfPath> &targets,
    std::string *error)
{
    if (!handler || handler->customTargetValidation) {
        return true;
    }
    const std::string who =
        std::string(handler->schemaType) + " " + prim.GetPath().GetString();
    if (handler->domain == RigExecMoverDomain::Points) {
        // Typed geometry movers move points: every canonical target
        // must be a native points property -- anything else would
        // pass validation yet silently produce no application.
        for (const SdfPath &t : targets) {
            const UsdPrim owner = t.IsPropertyPath()
                ? stage->GetPrimAtPath(t.GetPrimPath())
                : UsdPrim();
            const UsdAttribute attr = t.IsPropertyPath()
                ? stage->GetAttributeAtPath(t)
                : UsdAttribute();
            if (!t.IsPropertyPath() ||
                t.GetNameToken() != "points" ||
                !owner || !owner.IsA<UsdGeomPointBased>() ||
                !attr ||
                attr.GetTypeName() != SdfValueTypeNames->Point3fArray) {
                *error = who + " target " + t.GetString() +
                         " is not a native UsdGeomPointBased "
                         "point3f[] points attribute" +
                         RigExecPointsTargetHint(stage, t);
                return false;
            }
        }
        if (handler->singleTarget && targets.size() != 1) {
            *error = who + " must have exactly one canonical points "
                           "target in v0.1 (multi-target fan-out would "
                           "alias mover-level parameters)";
            return false;
        }
        return true;
    }

    // Property-domain movers: exactly one exact scalar target, of the
    // value type the mover is statically typed for.
    if (targets.size() != 1) {
        *error = who + " must have exactly one target (its parameters "
                       "are mover-level, so a fan-out would alias them "
                       "across targets)";
        return false;
    }
    const SdfPath &t = targets[0];
    // A prim path canonicalizes to .points, which is never what a
    // property mover means; require the exact property the author wrote.
    const UsdAttribute attr = t.IsPropertyPath()
        ? stage->GetAttributeAtPath(t)
        : UsdAttribute();
    if (!attr) {
        *error = who + " target " + t.GetString() +
                 " is not an exact property path";
        return false;
    }
    const SdfValueTypeName valueType = attr.GetTypeName();
    bool typeOk = false;
    const char *expected = "";
    if (handler->domain == RigExecMoverDomain::PropertyFloat) {
        // A double target is a control's avar: the chain computes in
        // float and publishes the double, so a hidden control's
        // channels can be driven by keys.
        expected = "float/double";
        typeOk = valueType == SdfValueTypeNames->Float ||
                 valueType == SdfValueTypeNames->Double;
    } else if (handler->domain == RigExecMoverDomain::PropertyVec3f) {
        // Every GfVec3f-backed scalar role, not just float3: a mover
        // offsetting a vector3f or a color3f is doing the identical
        // arithmetic, and refusing it would be a distinction the
        // kernel does not make.
        expected = "float3/vector3f/point3f/normal3f/color3f";
        typeOk = valueType == SdfValueTypeNames->Float3 ||
                 valueType == SdfValueTypeNames->Vector3f ||
                 valueType == SdfValueTypeNames->Point3f ||
                 valueType == SdfValueTypeNames->Normal3f ||
                 valueType == SdfValueTypeNames->Color3f;
    } else {
        expected = "matrix4d";
        typeOk = valueType == SdfValueTypeNames->Matrix4d;
    }
    if (!typeOk) {
        *error = who + " target " + t.GetString() + " has type " +
                 valueType.GetAsToken().GetString() + "; expected " +
                 expected;
        return false;
    }
    return true;
}

bool
RigExecResolveBlendSampleLayout(
    const UsdStageRefPtr &stage,
    const SdfPath &blendShapePath,
    size_t pointCount,
    RigExecBlendSampleLayout *layout)
{
    layout->pointCount = pointCount;
    layout->valid = false;

    const UsdSkelBlendShape shape(
        stage->GetPrimAtPath(blendShapePath.GetPrimPath()));
    if (!shape) {
        return true;   // cache the miss; it cannot become a hit this epoch
    }
    const UsdAttribute offsetsAttr = shape.GetOffsetsAttr();
    const UsdAttribute indicesAttr = shape.GetPointIndicesAttr();

    // The admission rule. `uniform` already forbids time samples, so a
    // connection is the only route by which these can change while the epoch
    // stands -- and a connected value may itself be animated, which is
    // exactly what the skin layout refuses for.
    // Noted and carried, NOT returned early: a refusal means "read this every
    // frame", so the read still has to happen. Returning here would hand the
    // caller an empty layout, the sample would fail validation, and a
    // perfectly good connected blend shape would come out as MoverFailed
    // instead of merely uncached.
    const bool cacheable = !offsetsAttr.HasAuthoredConnections() &&
                           !indicesAttr.HasAuthoredConnections();

    VtVec3fArray offsets;
    VtIntArray indices;
    offsetsAttr.Get(&offsets);
    indicesAttr.Get(&indices);

    // An empty pointIndices means the offsets are dense and parallel to the
    // base points -- UsdSkelBlendShape's own convention, kept rather than
    // invented so an authored blend shape from anywhere else reads correctly.
    if (!indices.empty()) {
        if (indices.size() != offsets.size()) {
            return cacheable;   // stable and wrong; cached as invalid
        }
        for (int index : indices) {
            if (index < 0 || size_t(index) >= pointCount) {
                return cacheable;
            }
        }
    } else if (!offsets.empty() && offsets.size() != pointCount) {
        return cacheable;
    }
    for (const GfVec3f &offset : offsets) {
        if (!std::isfinite(offset[0]) || !std::isfinite(offset[1]) ||
            !std::isfinite(offset[2])) {
            return cacheable;
        }
    }

    layout->offsets.assign(offsets.begin(), offsets.end());
    layout->indices.assign(indices.begin(), indices.end());
    layout->valid = true;
    return cacheable;
}

}  // namespace rigExec

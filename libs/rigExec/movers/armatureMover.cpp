// Masked/chained armatures are shared RigExec point revisions.
// These callbacks have no file-format or application dependency.
// Numerical/source references: docs/references.md.
#include "rigExec/movers/moverRegistry.h"
#include "rigExecGraph/sceneDescriptors.h"
#include "rigExecMath/solvers.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include <cmath>
#include <cstring>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
// The declared inputs, in this order: assembly reads them by position.
enum Input { kJointIndices, kJointWeights, kElementSize, kSkinningMethod, kMask,
             kUseBaseInput, kTransformInput, kInputCount };

// Every read is through the resolved inputs at the evaluated time, over
// the schema fallback.
std::vector<RigExecRevisionLeafKey>
DeclaredInputs(const SdfPath &mover)
{
    using Type = RigExecRevisionLeafType;
    const auto key = [&](const char *name, Type type, VtValue fallback) {
        return RigExecRevisionLeafKey{mover.AppendProperty(TfToken(name)), type,
            RigExecRevisionLeafTime::AtTime, RigExecRevisionLeafFlavour::Resolved,
            std::move(fallback)};
    };
    return {key("rigExec:jointIndices", Type::IntArray, VtValue(VtIntArray())),
            key("rigExec:jointWeights", Type::FloatArray, VtValue(VtFloatArray())),
            key("rigExec:elementSize", Type::Int, VtValue(1)),
            key("rigExec:skinningMethod", Type::Token, VtValue(TfToken("classicLinear"))),
            key("inputs:mask", Type::FloatArray, VtValue(VtFloatArray())),
            key("inputs:useBaseInput", Type::Bool, VtValue(false)),
            key("inputs:transformInput", Type::Bool, VtValue(false))};
}

// \p value as T, or false when it holds another type.
template<class T> bool As(const VtValue &value, T *out) {
    if (!value.IsHolding<T>()) return false;
    *out = value.UncheckedGet<T>();
    return true;
}

// Text compare without building a token: TfToken(text) takes the registry.
bool Is(const TfToken &token, const char *text) {
    return std::strcmp(token.GetText(), text) == 0;
}

void Bind(const RigExecMoverBindContext &ctx) {
    auto &binding = *ctx.binding;
    binding.influences = RigExecRelationshipTargets(ctx.moverPrim, "rigExec:influences");
    binding.transformPhase = RigExecPhaseForInput(ctx.moverPrim, "rigExec:influences");
    if (binding.transformPhase.kind == RigExecReadPhaseKind::Final)
        for (auto &path : binding.influences) {
            auto it = ctx.frameChainHeads.find(path);
            if (it != ctx.frameChainHeads.end()) path = it->second;
        }
    auto transform=RigExecRelationshipTargets(ctx.moverPrim,"rigExec:transform");
    if(transform.size()==1) {
        binding.transform=transform[0];
        auto it=ctx.frameChainHeads.find(binding.transform);
        if(it!=ctx.frameChainHeads.end()) binding.transform=it->second;
    }
}

void DeclareInputs(const RigExecMoverBindContext &ctx,
                   std::vector<RigExecRevisionLeafKey> *inputs) {
    *inputs = DeclaredInputs(ctx.moverPrim.GetPath());
}

bool CompileScene(const RigExecSceneDescriptors &scene, const SdfPath &mover,
                  const SdfPath &, RigExecRevisionBinding *binding, std::string *error) {
    if (!binding) return false;
    const auto targets = [&](const char *name) {
        const auto found = scene.relationships.find(mover.AppendProperty(TfToken(name)));
        return found == scene.relationships.end() ? SdfPathVector() : found->second.fact.targets;
    };
    binding->influences = targets("rigExec:influences");
    const auto influences = scene.relationships.find(mover.AppendProperty(TfToken("rigExec:influences")));
    if (influences != scene.relationships.end() &&
        !RigExecParseReadPhase(influences->second.readPhase.GetString(), &binding->transformPhase, error))
        return false;
    const auto transform = targets("rigExec:transform");
    if (transform.size() == 1) binding->transform = transform[0];
    binding->externalInputs = DeclaredInputs(mover);
    return true;
}

// The payload from the declared inputs and the provider values. Pure: no
// stage, registry or shared state.
bool Payload(const std::vector<VtValue> &inputs, const RigExecExternalProviderValues &values,
             VtValue *data) {
    if (inputs.size() != kInputCount || !values.influenceTransforms ||
        values.influenceTransforms->empty()) return false;
    VtIntArray indices; VtFloatArray weights, mask; int slots = 0;
    bool fromBase = false, transformInput = false; TfToken method;
    if (!As(inputs[kJointIndices], &indices) || !As(inputs[kJointWeights], &weights) ||
        !As(inputs[kElementSize], &slots) || !As(inputs[kSkinningMethod], &method) ||
        !As(inputs[kMask], &mask) || !As(inputs[kUseBaseInput], &fromBase) ||
        !As(inputs[kTransformInput], &transformInput)) return false;
    if (transformInput && !values.transform) return false;
    const bool dualQuaternion = Is(method, "dualQuaternion");
    if (weights.size() != indices.size() || slots < 1 ||
        (!mask.empty() && mask.size() != values.basePoints.size()) ||
        (!Is(method, "classicLinear") && !dualQuaternion)) return false;
    for (float w : mask) if (!std::isfinite(w) || w < 0 || w > 1) return false;
    const auto &transforms = *values.influenceTransforms;
    RigExecSkinLayout layout{transforms.data(),transforms.size(),indices.data(),weights.data(),
                            indices.size(),size_t(slots),values.basePoints.size()};
    if (!layout.Validate()) return false;
    VtDictionary payload;
    payload["matrices"] = VtValue(VtMatrix4dArray(transforms.begin(),transforms.end()));
    payload["indices"] = VtValue(indices); payload["weights"] = VtValue(weights);
    payload["mask"] = VtValue(mask); payload["slots"] = VtValue(slots);
    payload["dq"] = VtValue(dualQuaternion);
    payload["base"] = VtValue(fromBase ? VtVec3fArray(values.basePoints.begin(),values.basePoints.end()) : VtVec3fArray());
    payload["inputMatrix"] = VtValue(transformInput ? *values.transform : GfMatrix4d(1));
    *data = VtValue(payload);
    return true;
}

bool Assemble(const RigExecExternalInputContext &ctx, VtValue *data) {
    return Payload(ctx.inputs, ctx.values, data);
}

// The payload entry \p key as T, or null.
template<class T> const T *Entry(const VtDictionary &payload, const char *key) {
    const auto found = payload.find(key);
    return found != payload.end() && found->second.IsHolding<T>()
        ? &found->second.UncheckedGet<T>() : nullptr;
}

bool Apply(const VtValue &data, std::vector<GfVec3f> *points) {
    if (!data.IsHolding<VtDictionary>()) return false;
    const auto &p = data.UncheckedGet<VtDictionary>();
    const auto *matrices = Entry<VtMatrix4dArray>(p, "matrices");
    const auto *indices = Entry<VtIntArray>(p, "indices");
    const auto *weights = Entry<VtFloatArray>(p, "weights");
    const auto *mask = Entry<VtFloatArray>(p, "mask");
    const auto *base = Entry<VtVec3fArray>(p, "base");
    const auto *slots = Entry<int>(p, "slots");
    const auto *dq = Entry<bool>(p, "dq");
    const auto *inputMatrix = Entry<GfMatrix4d>(p, "inputMatrix");
    if (!matrices || !indices || !weights || !mask || !base || !slots || !dq ||
        !inputMatrix) return false;
    if ((!base->empty() && base->size() != points->size()) ||
        (!mask->empty() && mask->size() != points->size())) return false;
    RigExecSkinLayout layout{matrices->data(),matrices->size(),indices->data(),weights->data(),
                            indices->size(),size_t(*slots),points->size()};
    if (!layout.Validate()) return false;
    std::vector<GfVec3f> candidate(points->size());
    const auto *input = base->empty() ? points->data() : base->data();
    if (*dq) {
        if (!RigExecApplyDualQuatSkin(input,candidate.data(),layout)) return false;
    } else RigExecApplyLinearBlendSkin(input,candidate.data(),layout);
    for (size_t i=0;i<points->size();++i) {
        (*points)[i]=GfVec3f(inputMatrix->Transform(GfVec3d((*points)[i])));
        float w = mask->empty() ? 1.0f : (*mask)[i];
        if (w == 1) (*points)[i] = candidate[i];
        else if (w != 0) (*points)[i] += w*(candidate[i]-(*points)[i]);
    }
    return true;
}

bool Validate(const RigExecMoverValidateContext &ctx, std::string *error) {
    auto reject = [&](const std::string &message) { *error = ctx.prim.GetPath().GetString()+": "+message; return false; };
    if (ctx.targets.size()!=1 || ctx.targets[0].GetNameToken()!=TfToken("points") ||
        !UsdGeomPointBased(ctx.stage->GetPrimAtPath(ctx.targets[0].GetPrimPath())))
        return reject("requires one native point3f[] points property");
    VtVec3fArray points; ctx.stage->GetAttributeAtPath(ctx.targets[0]).Get(&points);
    auto influences = RigExecRelationshipTargets(ctx.prim,"rigExec:influences");
    if (influences.empty()) return reject("no influences");
    for (const auto &path : influences) {
        auto type = ctx.stage->GetPrimAtPath(path).GetTypeName();
        if (type!=TfToken("RigExecJoint") && type!=TfToken("RigExecControl")) return reject("invalid frame provider");
    }
    // The authored layout against identity frames and the authored points.
    std::vector<VtValue> inputs;
    for (const auto &key : DeclaredInputs(ctx.prim.GetPath())) {
        VtValue value = key.fallback;
        if (const UsdAttribute a = ctx.prim.GetAttribute(key.path.GetNameToken())) a.Get(&value);
        inputs.push_back(value);
    }
    RigExecProviderValues values; values.basePoints.assign(points.begin(),points.end());
    std::vector<GfMatrix4d> matrices(influences.size(),GfMatrix4d(1)); values.influenceTransforms=&matrices;
    GfMatrix4d transform(1);values.transform=&transform;
    VtValue data;
    return Payload(inputs,RigExecExternalProviderValues(values),&data) ||
        reject("invalid skin layout, method or mask");
}

RigExecMoverHandler Handler(const char *type) {
    RigExecMoverHandler handler(type,
        &RigExecFixedMoverOp<RigExecRevisionOp::External>, RigExecMoverDomain::Points);
    handler.singleTarget = true; handler.frameRelationships = {"rigExec:influences","rigExec:transform"};
    handler.transformRelationship = "rigExec:influences";
    handler.bind = &Bind; handler.validate = &Validate;
    handler.declareExternalInputs = &DeclareInputs; handler.compileScene = &CompileScene;
    handler.assembleExternal = &Assemble; handler.applyExternal = &Apply;
    handler.hasScalarOracle = false;
    handler.layoutAttributes = {TfToken("rigExec:jointIndices"),TfToken("rigExec:jointWeights"),
        TfToken("rigExec:elementSize"),TfToken("inputs:mask"),TfToken("inputs:useBaseInput")};
    return handler;
}
}
RIGEXEC_REGISTER_MOVER(Handler("RigExecLayeredSkinMover"));

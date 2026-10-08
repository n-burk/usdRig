// Masked/chained armatures are shared RigExec point revisions.
// These callbacks have no file-format or application dependency.
// Numerical/source references: docs/references.md.
#include "rigExec/movers/moverRegistry.h"
#include "rigExecMath/solvers.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include <cmath>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
template<class T> bool Read(const UsdPrim &prim, const RigExecProviderValues &values,
                           const char *name, UsdTimeCode time, T *out) {
    auto attr = prim.GetAttribute(TfToken(name));
    return values.resolved ? values.resolved->GetAttribute(attr, time, out) : attr.Get(out, time);
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
bool Assemble(const UsdPrim &prim, const RigExecRevisionBinding &,
              const RigExecProviderValues &values, UsdTimeCode time, VtValue *data) {
    if (!values.influenceTransforms || values.influenceTransforms->empty()) return false;
    VtIntArray indices; VtFloatArray weights, mask; int slots = 0;
    bool fromBase = false, transformInput=false; TfToken method;
    if (!Read(prim,values,"rigExec:jointIndices",time,&indices) ||
        !Read(prim,values,"rigExec:jointWeights",time,&weights) ||
        !Read(prim,values,"rigExec:elementSize",time,&slots) ||
        !Read(prim,values,"rigExec:skinningMethod",time,&method) ||
        !Read(prim,values,"inputs:mask",time,&mask) ||
        !Read(prim,values,"inputs:useBaseInput",time,&fromBase) ||
        !Read(prim,values,"inputs:transformInput",time,&transformInput)) return false;
    if(transformInput && !values.transform) return false;
    if (weights.size() != indices.size() ||
        (!mask.empty() && mask.size() != values.basePoints.size()) ||
        (method != TfToken("classicLinear") && method != TfToken("dualQuaternion"))) return false;
    for (float w : mask) if (!std::isfinite(w) || w < 0 || w > 1) return false;
    const auto &transforms = *values.influenceTransforms;
    RigExecSkinLayout layout{transforms.data(),transforms.size(),indices.data(),weights.data(),
                            indices.size(),size_t(slots),values.basePoints.size()};
    if (!layout.Validate()) return false;
    VtDictionary payload;
    payload["matrices"] = VtValue(VtMatrix4dArray(transforms.begin(),transforms.end()));
    payload["indices"] = VtValue(indices); payload["weights"] = VtValue(weights);
    payload["mask"] = VtValue(mask); payload["slots"] = VtValue(slots);
    payload["dq"] = VtValue(method == TfToken("dualQuaternion"));
    payload["base"] = VtValue(fromBase ? VtVec3fArray(values.basePoints.begin(),values.basePoints.end()) : VtVec3fArray());
    payload["inputMatrix"] = VtValue(transformInput ? *values.transform : GfMatrix4d(1));
    *data = VtValue(payload);
    return true;
}
bool Apply(const VtValue &data, std::vector<GfVec3f> *points) {
    if (!data.IsHolding<VtDictionary>()) return false;
    const auto &p = data.UncheckedGet<VtDictionary>();
    const auto &matrices = p.find("matrices")->second.Get<VtMatrix4dArray>();
    const auto &indices = p.find("indices")->second.Get<VtIntArray>();
    const auto &weights = p.find("weights")->second.Get<VtFloatArray>();
    const auto &mask = p.find("mask")->second.Get<VtFloatArray>();
    const auto &base = p.find("base")->second.Get<VtVec3fArray>();
    if ((!base.empty() && base.size() != points->size()) || (!mask.empty() && mask.size() != points->size())) return false;
    RigExecSkinLayout layout{matrices.data(),matrices.size(),indices.data(),weights.data(),
                            indices.size(),size_t(p.find("slots")->second.Get<int>()),points->size()};
    if (!layout.Validate()) return false;
    std::vector<GfVec3f> candidate(points->size());
    const auto *input = base.empty() ? points->data() : base.data();
    if (p.find("dq")->second.Get<bool>()) {
        if (!RigExecApplyDualQuatSkin(input,candidate.data(),layout)) return false;
    } else RigExecApplyLinearBlendSkin(input,candidate.data(),layout);
    for (size_t i=0;i<points->size();++i) {
        (*points)[i]=GfVec3f(p.find("inputMatrix")->second.Get<GfMatrix4d>().Transform(GfVec3d((*points)[i])));
        float w = mask.empty() ? 1.0f : mask[i];
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
    RigExecProviderValues values; values.basePoints.assign(points.begin(),points.end());
    std::vector<GfMatrix4d> matrices(influences.size(),GfMatrix4d(1)); values.influenceTransforms=&matrices;
    GfMatrix4d transform(1);values.transform=&transform;
    VtValue data;
    return Assemble(ctx.prim,{},values,UsdTimeCode::Default(),&data) || reject("invalid skin layout, method or mask");
}
RigExecMoverHandler Handler(const char *type) {
    RigExecMoverHandler handler(type,
        &RigExecFixedMoverOp<RigExecRevisionOp::External>, RigExecMoverDomain::Points);
    handler.singleTarget = true; handler.frameRelationships = {"rigExec:influences","rigExec:transform"};
    handler.transformRelationship = "rigExec:influences";
    handler.bind = &Bind; handler.validate = &Validate;
    handler.assembleExternal = &Assemble; handler.applyExternal = &Apply;
    handler.hasScalarOracle = false;
    handler.layoutAttributes = {TfToken("rigExec:jointIndices"),TfToken("rigExec:jointWeights"),
        TfToken("rigExec:elementSize"),TfToken("inputs:mask"),TfToken("inputs:useBaseInput")};
    return handler;
}
}
RIGEXEC_REGISTER_MOVER(Handler("RigExecLayeredSkinMover"));

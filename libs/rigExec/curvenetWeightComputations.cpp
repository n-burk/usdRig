#include "curvenetWeightComputations.h"
#include "rigExecMath/curvenetWeights.h"
#include "pxr/base/tf/hash.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include <algorithm>
#include <cmath>
#include <list>
#include <mutex>

namespace rigExec {
bool RigExecCurvenetWeightTokensAreValid(const TfToken &basis,
                                         const TfToken &rangePolicy)
{
    return (basis == "bezier" || basis == "catmullRom") &&
           (rangePolicy == "strict" || rangePolicy == "clamp");
}

std::shared_ptr<RigExecCurvenetWeightBinding> RigExecBindCurvenetWeightPacket(
    const std::vector<GfVec3f> &mesh, const std::vector<int> &counts,
    const std::vector<int> &indices, const std::vector<GfVec3f> &net,
    const std::vector<int> &splines, const TfToken &basis, int samples,
    const std::vector<int> &autoSmooth, std::string *error)
{
    RigExecCurvenetTopology topology;
    if (!RigExecBuildCurvenetTopology(splines, net.size(),
            basis == "bezier" ? RigExecCurvenetBasis::Bezier : RigExecCurvenetBasis::CatmullRom,
            net, nullptr, &topology, error)) return nullptr;
    auto binding = std::make_shared<RigExecCurvenetWeightBinding>();
    if (!RigExecBindCurvenetWeights(topology, net, mesh, counts, indices,
            autoSmooth, samples, binding.get(), error)) return nullptr;
    return binding;
}

RigExecWeightPacket RigExecCurvenetWeightPacketFromBinding(
    const RigExecCurvenetWeightBinding &binding,
    const std::vector<float> &weights, const TfToken &rangePolicy,
    float fallback, std::string *error)
{
    RigExecWeightPacket packet;
    packet.representation = TfToken("dense");
    packet.rangePolicy = rangePolicy;
    if (!RigExecEvaluateCurvenetWeights(binding, weights, 1, fallback, &packet.values, error))
        return packet;
    for (float &value : packet.values) {
        if (!std::isfinite(value) || (rangePolicy == "strict" && (value < 0 || value > 1))) {
            if (error) *error = "curvenet weights violate the finite [0,1] envelope range";
            return packet;
        }
        value = std::clamp(value, 0.0f, 1.0f);
    }
    packet.valid = true;
    return packet;
}

RigExecWeightPacket RigExecComputeCurvenetWeightPacket(
    const std::vector<GfVec3f> &mesh, const std::vector<int> &counts,
    const std::vector<int> &indices, const std::vector<GfVec3f> &net,
    const std::vector<int> &splines, const TfToken &basis, int samples,
    const std::vector<int> &autoSmooth, const std::vector<float> &weights,
    const TfToken &rangePolicy, float fallback, std::string *error)
{
    if (!RigExecCurvenetWeightTokensAreValid(basis, rangePolicy)) {
        RigExecWeightPacket packet;
        packet.representation = TfToken("dense");
        packet.rangePolicy = rangePolicy;
        if (error) *error = "invalid curvenet weight basis or range policy";
        return packet;
    }
    // Small bounded LRU keyed by exact geometry, independent of the animated
    // weight values. Shared pointers keep concurrent solves alive on eviction.
    struct Entry {
        std::vector<GfVec3f> mesh, net;
        std::vector<int> counts, indices, splines, smooth;
        TfToken basis;
        int samples;
        std::shared_ptr<RigExecCurvenetWeightBinding> binding;
    };
    static std::mutex mutex;
    static std::list<Entry> cache;
    std::shared_ptr<RigExecCurvenetWeightBinding> binding;
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto it = cache.begin(); it != cache.end(); ++it) {
            if (it->mesh == mesh && it->net == net && it->counts == counts &&
                it->indices == indices && it->splines == splines && it->smooth == autoSmooth &&
                it->basis == basis && it->samples == samples) {
                binding = it->binding;
                cache.splice(cache.begin(), cache, it);
                break;
            }
        }
    }
    if (!binding) {
        binding = RigExecBindCurvenetWeightPacket(mesh, counts, indices, net,
            splines, basis, samples, autoSmooth, error);
        if (!binding) {
            RigExecWeightPacket packet;
            packet.representation = TfToken("dense");
            packet.rangePolicy = rangePolicy;
            return packet;
        }
        std::lock_guard<std::mutex> lock(mutex);
        cache.push_front({mesh,net,counts,indices,splines,autoSmooth,basis,samples,binding});
        if (cache.size() > 32) cache.pop_back();
    }
    return RigExecCurvenetWeightPacketFromBinding(*binding, weights,
                                                  rangePolicy, fallback, error);
}
}

PXR_NAMESPACE_USING_DIRECTIVE
TF_DEFINE_PRIVATE_TOKENS(_cw,
    (computeWeightPacket)
    ((mesh, "rigExec:weightTarget"))
    ((net, "rigExec:curvenetPoints"))
    ((counts, "rigExec:meshFaceCounts"))
    ((indices, "rigExec:meshFaceIndices"))
    ((splines, "rigExec:curvenetSplineIndices"))
    ((weights, "inputs:weights"))
    ((smooth, "rigExec:autoSmooth"))
    ((basis, "rigExec:basis"))
    ((samples, "rigExec:samplesPerSpline"))
    ((policy, "rigExec:rangePolicy"))
    ((fallback, "rigExec:unreachedValue"))
);

namespace {
template<class T> std::vector<T> Array(const VdfContext &ctx, const TfToken &name) {
    std::vector<T> values;
    for (VdfReadIterator<T> it(ctx,name); !it.IsAtEnd(); ++it) values.push_back(*it);
    return values;
}
rigExec::RigExecWeightPacket Compute(const VdfContext &ctx) {
    const auto *basis = ctx.GetInputValuePtr<TfToken>(_cw->basis);
    const auto *policy = ctx.GetInputValuePtr<TfToken>(_cw->policy);
    const auto *samples = ctx.GetInputValuePtr<int>(_cw->samples);
    const auto *fallback = ctx.GetInputValuePtr<float>(_cw->fallback);
    return rigExec::RigExecComputeCurvenetWeightPacket(Array<GfVec3f>(ctx,_cw->mesh),
        Array<int>(ctx,_cw->counts), Array<int>(ctx,_cw->indices), Array<GfVec3f>(ctx,_cw->net),
        Array<int>(ctx,_cw->splines), basis ? *basis : TfToken("catmullRom"), samples ? *samples : 5,
        Array<int>(ctx,_cw->smooth), Array<float>(ctx,_cw->weights),
        policy ? *policy : TfToken("clamp"), fallback ? *fallback : 0.0f, nullptr);
}
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecCurvenetWeight)
{
    self.PrimComputation(_cw->computeWeightPacket)
        .Callback<rigExec::RigExecWeightPacket>(&Compute)
        .Inputs(
            Relationship(_cw->mesh).TargetedObjects<GfVec3f>(ExecBuiltinComputations->computeValue).InputName(_cw->mesh).Required(),
            Relationship(_cw->net).TargetedObjects<GfVec3f>(ExecBuiltinComputations->computeValue).InputName(_cw->net).Required(),
            Relationship(_cw->counts).TargetedObjects<int>(ExecBuiltinComputations->computeValue).InputName(_cw->counts).Required(),
            Relationship(_cw->indices).TargetedObjects<int>(ExecBuiltinComputations->computeValue).InputName(_cw->indices).Required(),
            Relationship(_cw->splines).TargetedObjects<int>(ExecBuiltinComputations->computeValue).InputName(_cw->splines).Required(),
            AttributeValue<float>(_cw->weights), AttributeValue<int>(_cw->smooth),
            AttributeValue<TfToken>(_cw->basis), AttributeValue<TfToken>(_cw->policy),
            AttributeValue<int>(_cw->samples), AttributeValue<float>(_cw->fallback));
}

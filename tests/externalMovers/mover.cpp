#include "rigExec/movers/moverRegistry.h"
#include "rigExecScene/sceneDescriptors.h"

#include <cmath>
#include <cstring>
#include <limits>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

// The payload: (gain, reference y, whether a reference was read). A
// reference is any points property named by rigExec:reference, read at the
// phase that relationship declares; its first point's y is added to every
// point, so a phased read that playback re-evaluates shows in the result.
using Payload = GfVec3f;

void Bind(const RigExecMoverBindContext &ctx)
{
    const SdfPathVector references =
        RigExecRelationshipTargets(ctx.moverPrim, "rigExec:reference");
    if (references.size() != 1) return;
    const RigExecReadPhase phase =
        RigExecPhaseForInput(ctx.moverPrim, "rigExec:reference");
    if (!phase.IsBase()) {
        ctx.binding->phases[references[0]] = phase;
    }
}

void DeclareInputs(const RigExecMoverBindContext &ctx,
                   std::vector<RigExecRevisionLeafKey> *inputs)
{
    inputs->push_back({ctx.moverPrim.GetPath().AppendProperty(TfToken("inputs:gain")),
        RigExecRevisionLeafType::Float, RigExecRevisionLeafTime::AtTime,
        RigExecRevisionLeafFlavour::Resolved, VtValue(1.0f)});
    const auto references = RigExecRelationshipTargets(ctx.moverPrim, "rigExec:reference");
    if (references.size() == 1) {
        inputs->push_back({references[0], RigExecRevisionLeafType::Vec3fArray,
            RigExecRevisionLeafTime::AtTime, RigExecRevisionLeafFlavour::Resolved,
            VtValue(VtVec3fArray())});
    }
}

bool CompileScene(const RigExecSceneDescriptors &scene,const SdfPath &mover,
                  const SdfPath &,RigExecRevisionBinding *binding,std::string *error)
{
    if(!binding)return false;
    binding->externalInputs={{mover.AppendProperty(TfToken("inputs:gain")),
        RigExecRevisionLeafType::Float,RigExecRevisionLeafTime::AtTime,
        RigExecRevisionLeafFlavour::Resolved,VtValue(1.0f)}};
    const auto reference=scene.relationships.find(mover.AppendProperty(TfToken("rigExec:reference")));
    if(reference!=scene.relationships.end() && reference->second.fact.targets.size()==1) {
        const auto &path=reference->second.fact.targets.front();RigExecReadPhase phase;
        if(!RigExecParseReadPhase(reference->second.readPhase.GetString(),&phase,error))return false;
        if(!phase.IsBase())binding->phases[path]=phase;
        binding->externalInputs.push_back({path,RigExecRevisionLeafType::Vec3fArray,
            RigExecRevisionLeafTime::AtTime,RigExecRevisionLeafFlavour::Resolved,VtValue(VtVec3fArray())});
    }
    return true;
}

bool Assemble(const RigExecExternalInputContext &ctx, VtValue *data)
{
    if (ctx.values.basePoints.empty() || ctx.inputs.empty() ||
        !ctx.inputs[0].IsHolding<float>()) return false;
    const float gain = ctx.inputs[0].UncheckedGet<float>();
    if (!std::isfinite(gain)) return false;
    Payload payload(gain, 0.0f, 0.0f);
    if (ctx.inputs.size() == 2) {
        if (!ctx.inputs[1].IsHolding<VtVec3fArray>()) return false;
        const auto &points = ctx.inputs[1].UncheckedGet<VtVec3fArray>();
        if (points.empty()) return false;
        payload[1] = points[0][1];
        payload[2] = 1.0f;
    }
    *data = VtValue(payload);
    return true;
}

// The deformation, shared by the stage-side callback and the playback
// kernel. False is a failed mover; the -2 and -3 gains exercise the hosts'
// atomic failure paths (a changed count, a non-finite point).
bool Deform(float gain, float referenceY, float *xyz, size_t count)
{
    if (gain == -3.0f && count > 0) {
        xyz[2] = std::numeric_limits<float>::quiet_NaN();
        return true;
    }
    for (size_t i = 0; i < count; ++i) {
        xyz[i * 3 + 1] += referenceY;
        xyz[i * 3 + 2] += gain * xyz[i * 3] * xyz[i * 3];
    }
    return gain >= 0.0f;
}

bool Apply(const VtValue &data, std::vector<GfVec3f> *points)
{
    if (!data.IsHolding<Payload>()) return false;
    const Payload &payload = data.UncheckedGet<Payload>();
    if (payload[0] == -2.0f) {
        points->push_back(GfVec3f(0));
        return true;
    }
    return Deform(payload[0], payload[1],
                  points->empty() ? nullptr : points->front().data(),
                  points->size());
}

RigExecOracleResult Oracle(const RigExecMoverOracleContext &ctx)
{
    float gain = 1.0f;
    ctx.resolved.GetAttribute(ctx.prim.GetAttribute(TfToken("inputs:gain")),
                              ctx.time, &gain);
    if (!std::isfinite(gain) || gain < 0.0f) {
        return RigExecOracleResult::PassThrough;
    }
    for (GfVec3f &point : *ctx.points) {
        point[2] += gain * point[0] * point[0];
    }
    return RigExecOracleResult::Blend;
}

// .rigexec export: epoch bytes are a format tag the kernel checks; frame
// bytes are the payload's three floats.
constexpr char kEpochTag[] = {'Q', 'U', 'A', '1'};

bool EncodeEpoch(const RigExecRevisionBinding &,std::vector<uint8_t> *epoch) {
    epoch->assign(kEpochTag,kEpochTag+sizeof(kEpochTag));return true;
}

bool Encode(const VtValue &data, const RigExecRevisionBinding &,
            std::vector<uint8_t> *epoch, std::vector<uint8_t> *frame)
{
    if (!data.IsHolding<Payload>()) return false;
    const Payload &payload = data.UncheckedGet<Payload>();
    epoch->assign(kEpochTag, kEpochTag + sizeof(kEpochTag));
    frame->resize(sizeof(float) * 3);
    std::memcpy(frame->data(), payload.data(), frame->size());
    return true;
}

std::shared_ptr<const void> Prepare(const uint8_t *epoch, size_t size,
                                    std::string *error)
{
    if (size != sizeof(kEpochTag) ||
        std::memcmp(epoch, kEpochTag, size) != 0) {
        if (error) *error = "unknown epoch format";
        return nullptr;
    }
    return std::make_shared<int>(1);
}

bool Play(const void *, const uint8_t *frame, size_t frameSize,
          const RigExecExternalPhasedPoints *phased, size_t phasedCount,
          const RigExecExternalInputValue *inputs, size_t inputCount,
          float *xyz, size_t count)
{
    float payload[3];
    if (frameSize != sizeof(payload)) return false;
    std::memcpy(payload, frame, sizeof(payload));
    if (inputCount > 0) {
        if (inputs[0].type != uint8_t(RigExecExternalInputType::Float) ||
            !inputs[0].hasValue || !inputs[0].data || inputs[0].count != 1) return false;
        payload[0] = *static_cast<const float *>(inputs[0].data);
    }
    if (inputCount > 1) {
        if (inputs[1].type != uint8_t(RigExecExternalInputType::Vec3fArray) ||
            !inputs[1].hasValue || !inputs[1].data || inputs[1].count == 0) return false;
        payload[1] = static_cast<const float *>(inputs[1].data)[1];
        payload[2] = 1.0f;
    }
    if (payload[0] == -2.0f) return false;
    // Playback's own evaluation of the reference wins over the exported
    // value, so a posed playback moves it.
    if (payload[2] != 0.0f && phasedCount == 1 && phased[0].xyz &&
        phased[0].count > 0) {
        payload[1] = phased[0].xyz[1];
    }
    return Deform(payload[0], payload[1], xyz, count);
}

RigExecMoverHandler Handler()
{
    RigExecMoverHandler handler("ExternalQuadraticMover",
        &RigExecFixedMoverOp<RigExecRevisionOp::External>,
        RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.bind = &Bind;
    handler.declareExternalInputs = &DeclareInputs;
    handler.compileScene = &CompileScene;
    handler.assembleExternal = &Assemble;
    handler.applyExternal = &Apply;
    handler.oracle = &Oracle;
    handler.encodeExternalEpoch = &EncodeEpoch;
    handler.encodeExternal = &Encode;
    handler.runtimeKernel.prepare = &Prepare;
    handler.runtimeKernel.apply = &Play;
    return handler;
}

// The same mover from a plugin that never adopted .rigexec export.
RigExecMoverHandler UnexportableHandler()
{
    RigExecMoverHandler handler = Handler();
    handler.schemaType = "ExternalUnexportableMover";
    handler.encodeExternalEpoch = nullptr;
    handler.encodeExternal = nullptr;
    handler.runtimeKernel = RigExecExternalKernel();
    return handler;
}

}  // namespace

RIGEXEC_REGISTER_MOVER(Handler());
RIGEXEC_REGISTER_MOVER(UnexportableHandler());

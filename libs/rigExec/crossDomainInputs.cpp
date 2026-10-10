#include "crossDomainInputs.h"
#include "bakedProgramImpl.h"
#include "rigEvaluatorInternal.h"
#include "rigExecMath/pointFrame.h"
#include <algorithm>
#include <limits>
#include <set>

namespace rigExec {
namespace {
int Ordinal(const RigExecBakedProgramImpl &B, const SdfPath &path)
{
    const auto found = B.crossDomainOrdinals.find(path);
    return found == B.crossDomainOrdinals.end() ? -1 : found->second;
}
bool Matches(const SdfPath &writer, const SdfPath &scope)
{
    return writer == scope || writer.HasPrefix(scope);
}
void WalkCrossReads(const RigExecBakedWalk &walk,
                    const std::function<void(int)> &visit)
{
    for (const auto *hops : {&walk.hops,&walk.doubleHops})
        for (const auto &hop : *hops) if (hop.crossDomain >= 0) visit(hop.crossDomain);
}
void RevisionCrossReads(const RigExecBakedPropertyChain::Revision &revision,
                        const std::function<void(int)> &visit)
{
    for (const auto *walk : {&revision.enabled,&revision.defaultWeight,&revision.value,
         &revision.minimum,&revision.maximum,&revision.keys,&revision.tangents})
        WalkCrossReads(*walk,visit);
}
}

void RigExecBakedCaptureCrossDomainOrder(RigExecBakedProgramImpl *program)
{
    auto &B = *program;
    if (!B.crossDomainOrdinals.empty()) return;
    int ordinal = 0;
    for (const auto &prim : evaluatorDetail::_GetPoseStackOrder(
             B.stage->GetPrimAtPath(B.assetRootPath)))
        B.crossDomainOrdinals.emplace(prim.GetPath(),ordinal++);
}

RigExecBakedPointsBinding RigExecBakedBindPointInput(
    const RigExecBakedProgramImpl &B, const SdfPath &input,
    const RigExecReadPhase &phase, const SdfPath &reader)
{
    RigExecBakedPointsBinding out;
    out.input = input;
    out.phase = phase;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const auto &chain = B.chains[c];
        if (chain.target != input) continue;
        const auto count = chain.revisions.size();
        switch (phase.kind) {
        case RigExecReadPhaseKind::Base:
            out.candidates.push_back({int(c),0});
            break;
        case RigExecReadPhaseKind::Final:
            out.finalRead = true;
            out.candidates.push_back({int(c),int(count)});
            break;
        case RigExecReadPhaseKind::Preceding: {
            int version = 0;
            const int here = Ordinal(B,reader);
            for (size_t r = 0; r < count; ++r) {
                const auto &writer = chain.revisions[r].moverPath;
                if (writer == reader) { version = int(r); break; }
                const int before = Ordinal(B,writer);
                if (before >= 0 && here >= 0 && before < here) version = int(r)+1;
            }
            out.candidates.push_back({int(c),version});
            break;
        }
        case RigExecReadPhaseKind::AtPrim:
            for (size_t r = count; r-- > 0;)
                if (Matches(chain.revisions[r].moverPath,phase.prim))
                    out.candidates.push_back({int(c),int(r)+1});
            break;
        }
        break;
    }
    return out;
}

bool RigExecBakedBindConnectionValue(const RigExecBakedProgramImpl &B,
    const SdfPath &consumer, const SdfPath &source, const RigExecReadPhase &phase,
    RigExecCrossDomainRead *read, int element)
{
    RigExecCrossDomainRead out;
    out.consumer = consumer; out.source = source;
    out.reader = consumer.GetPrimPath(); out.phase = phase;
    for (size_t c = 0; c < B.propertyChains.size(); ++c) {
        const auto &chain = B.propertyChains[c];
        if (chain.target != source) continue;
        out.kind = RigExecCrossDomainRead::Kind::PropertyResult;
        out.propertyChain = int(c);
        size_t applied = 0;
        if (phase.kind == RigExecReadPhaseKind::Final) applied = chain.revisions.size();
        else if (!phase.IsBase()) {
            for (size_t r = 0; r < chain.revisions.size(); ++r) {
                const auto &writer = chain.revisions[r].mover;
                if (phase.kind == RigExecReadPhaseKind::AtPrim ? Matches(writer,phase.prim) :
                    Ordinal(B,writer) >= 0 && Ordinal(B,writer) < Ordinal(B,out.reader)) applied = r+1;
            }
        }
        out.propertyVersion = chain.versionBase+uint32_t(applied);
        *read = std::move(out);
        return true;
    }
    const auto provider = B.index.find(source.GetPrimPath());
    if (provider != B.index.end() && source.GetName() == "posed:space") {
        out.kind = RigExecCrossDomainRead::Kind::PoseFrame;
        out.provider = provider->second;
        out.baseFrame = phase.IsBase();
        *read = std::move(out);
        return true;
    }
    const auto space = B.providerProgram.attributeValues.find(source);
    if (space != B.providerProgram.attributeValues.end()) {
        out.kind = RigExecCrossDomainRead::Kind::SpaceValue;
        out.spaceValue = int(space->second);
        *read = std::move(out);
        return true;
    }
    if (element >= 0 || source.IsPropertyPath()) {
        const auto binding = RigExecBakedBindPointInput(B,source,phase,out.reader);
        if (!binding.candidates.empty()) {
            out.kind = element >= 0 ? RigExecCrossDomainRead::Kind::PointElement : RigExecCrossDomainRead::Kind::Points;
            out.element = element; out.finalPoints = binding.finalRead;
            for (const auto &candidate : binding.candidates)
                out.points.push_back({candidate.chain,candidate.version});
            *read = std::move(out);
            return true;
        }
    }
    return false;
}

void RigExecBakedDeclarePointVersion(const RigExecBakedProgramImpl &B,
    int chain, int version, std::vector<RigExecBakedSlotRange> *reads)
{
    if (chain < 0 || size_t(chain) >= B.chains.size()) return;
    if (version == 0) {
        reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainBase,chain));
        return;
    }
    const int revision = B.chainRevisionBegin[size_t(chain)]+version-1;
    reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone,revision));
    reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty,revision));
}

void RigExecBakedDeclarePointInput(const RigExecBakedProgramImpl &B,
    const RigExecBakedPointsBinding &binding, std::vector<RigExecBakedSlotRange> *reads)
{
    for (const auto &candidate : binding.candidates) {
        if (binding.finalRead)
            reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints,candidate.chain));
        else RigExecBakedDeclarePointVersion(B,candidate.chain,candidate.version,reads);
    }
}

void RigExecBakedDeclareCrossDomainRead(const RigExecBakedProgramImpl &B, int index,
    std::vector<RigExecBakedSlotRange> *reads)
{
    if (index < 0 || size_t(index) >= B.crossDomainReads.size()) return;
    const auto &read = B.crossDomainReads[size_t(index)];
    if ((read.kind == RigExecCrossDomainRead::Kind::PointElement || read.kind == RigExecCrossDomainRead::Kind::Points)) {
        for (const auto &point : read.points) {
            if (read.finalPoints)
                reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints,point.chain));
            else RigExecBakedDeclarePointVersion(B,point.chain,point.version,reads);
        }
    } else if (read.kind == RigExecCrossDomainRead::Kind::PoseFrame) {
        for (const auto version : read.frames)
            reads->push_back(RigExecBakedOne(read.baseFrame ? RigExecBakedSlotDomain::PoseBase :
                RigExecBakedSlotDomain::PoseFin,int(version)));
    } else if (read.kind == RigExecCrossDomainRead::Kind::PropertyResult)
        reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::PropertyResult,int(read.propertyVersion)));
    else if (read.spaceValue >= 0)
        reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,read.spaceValue));
}

bool RigExecBakedFinalizeCrossDomainReads(RigExecBakedProgramImpl *program, std::string *error)
{
    auto &B = *program;
    if (!B.crossDomainErrors.empty()) {
        if (error) *error = B.crossDomainErrors.front();
        return false;
    }
    RigExecBakedCaptureCrossDomainOrder(&B);
    if (!RigExecBakedBindWeightPointInputs(&B,error)) return false;
    for (auto &read : B.crossDomainReads) {
        if ((read.kind == RigExecCrossDomainRead::Kind::PointElement || read.kind == RigExecCrossDomainRead::Kind::Points)) {
            const auto binding = RigExecBakedBindPointInput(B,read.source,read.phase,read.reader);
            read.finalPoints = binding.finalRead;
            read.points.clear();
            for (const auto &candidate : binding.candidates)
                read.points.push_back({candidate.chain,candidate.version});
        } else if (read.kind == RigExecCrossDomainRead::Kind::PoseFrame) {
            read.frames.clear();
            const size_t slot = size_t(read.provider);
            if (read.provider < 0 || slot >= B.paths.size()) continue;
            read.baseFrame = read.phase.IsBase();
            if (read.baseFrame) read.frames.push_back(B.baseLast[slot]);
            else if (read.phase.kind == RigExecReadPhaseKind::Final)
                read.frames.push_back(B.finLast[slot]);
            else {
                // Every commit owns versions, including propagated descendants.
                // Sort by authored stack position, then try newest valid first.
                std::vector<std::pair<int,uint32_t>> candidates;
                for (size_t w = 0; w < B.commits.size(); ++w) {
                    const auto &commit = B.commits[w];
                    int ordinal = -1;
                    const auto admit = [&](const SdfPath &writer) {
                        const int position = Ordinal(B,writer);
                        const bool selected = read.phase.kind == RigExecReadPhaseKind::AtPrim
                            ? Matches(writer,read.phase.prim)
                            : position >= 0 && position < Ordinal(B,read.reader);
                        if (selected) ordinal = std::max(ordinal,position);
                    };
                    const auto &walk = B.walkSteps[w];
                    if (walk.solverBatch)
                        for (const int solver : walk.batchSolvers) admit(B.solvers[size_t(solver)].path);
                    else admit(commit.moverPath);
                    if (ordinal < 0) continue;
                    for (size_t k = 0; k < commit.slots.size(); ++k)
                        if (commit.slots[k] == read.provider) candidates.emplace_back(ordinal,commit.slotWrites[k]);
                    for (size_t k = 0; k < commit.propagate.size(); ++k)
                        if (commit.propagate[k].first == read.provider)
                            candidates.emplace_back(ordinal,commit.descendantWrites[k]);
                }
                std::stable_sort(candidates.begin(),candidates.end(),
                    [](const auto &a,const auto &b){ return a.first > b.first; });
                for (const auto &candidate : candidates) read.frames.push_back(candidate.second);
                if (read.phase.kind == RigExecReadPhaseKind::Preceding && read.frames.empty())
                    read.frames.push_back(uint32_t(slot));
            }
        }
    }
    for (auto &solver : B.solvers)
        if (!solver.ribbonPointsPath.IsEmpty()) solver.ribbonPointsBinding =
            RigExecBakedBindPointInput(B,solver.ribbonPointsPath,solver.ribbonPointsPhase,solver.path);
    for (auto &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::WeightField)
            RigExecBakedDeclareWeightPointInputs(B,step.object,&step.reads);
        if (step.kind == RigExecBakedStepKind::PropertyRevision && step.part > 0) {
            const auto &revision = B.propertyChains[size_t(step.object)].revisions[size_t(step.part)-1];
            RevisionCrossReads(revision,[&](int read){
                RigExecBakedDeclareCrossDomainRead(B,read,&step.reads);
                const int leaf = B.crossDomainReads[size_t(read)].rawLeaf;
                if (leaf >= 0) step.leaves.push_back(uint32_t(leaf));
            });
        }
        for (const int index : step.readerWalks)
            if (index >= 0 && size_t(index) < B.readerWalks.size())
                WalkCrossReads(B.readerWalks[size_t(index)].walk,[&](int read){
                    RigExecBakedDeclareCrossDomainRead(B,read,&step.reads);
                });
        if (step.kind == RigExecBakedStepKind::Solve && step.object >= 0 &&
            size_t(step.object) < B.solvers.size())
            RigExecBakedDeclarePointInput(B,B.solvers[size_t(step.object)].ribbonPointsBinding,&step.reads);
    }
    return true;
}

bool RigExecBakedReadCrossDomain(const RigExecBakedProgramImpl &B, int index,
    VtValue *value, std::string *diagnostic)
{
    if (index < 0 || size_t(index) >= B.crossDomainReads.size()) return false;
    const auto &read = B.crossDomainReads[size_t(index)];
    if ((read.kind == RigExecCrossDomainRead::Kind::PointElement || read.kind == RigExecCrossDomainRead::Kind::Points)) {
        const GfVec3f *points = nullptr;
        size_t count = 0;
        // Local, never the read's own: many steps read one cross-domain read,
        // and a version cut into groups gathers into the binding's buffer.
        // Every use of `points` below copies out before it goes.
        RigExecBakedPointsBinding binding;
        binding.finalRead = read.finalPoints;
        for (const auto &candidate : read.points)
            binding.candidates.push_back({candidate.chain,candidate.version});
        bool available = RigExecBakedResolvePoints(B,binding,&points,&count);
        if (!available && read.points.empty() && read.rawLeaf >= 0 &&
            size_t(read.rawLeaf) < B.headLeaves.size()) {
            const auto &raw = B.headLeaves[size_t(read.rawLeaf)].value;
            if (raw.IsHolding<VtVec3fArray>()) {
                const auto &array = raw.UncheckedGet<VtVec3fArray>();
                points = array.cdata(); count = array.size(); available = true;
            }
        }
        if (available && read.kind == RigExecCrossDomainRead::Kind::Points) {
            VtVec3fArray array(count);
            if (count) std::copy(points,points+count,array.begin());
            *value = VtValue(std::move(array));
            return true;
        }
        if (available && read.element >= 0 && size_t(read.element) < count) {
            *value = VtValue(points[size_t(read.element)]);
            return true;
        }
    } else if (read.kind == RigExecCrossDomainRead::Kind::PoseFrame) {
        const auto &frames = read.baseFrame ? B.base : B.fin;
        for (const uint32_t version : read.frames) {
            if (version >= frames.size() || !frames[version].IsValid() || frames[version].IsDegenerate()) continue;
            GfMatrix4d matrix(1);
            if (RigExecPointsToMatrix(RigExecIdentityLandmarks(),frames[version],&matrix)) {
                *value = VtValue(matrix);
                return true;
            }
        }
    } else if (read.kind == RigExecCrossDomainRead::Kind::PropertyResult) {
        if (read.propertyChain >= 0 && size_t(read.propertyChain) < B.propertyChains.size() &&
            read.propertyVersion < B.propertyValues.size() && B.propertyVersionValid[read.propertyVersion]) {
            const auto &result = B.propertyValues[read.propertyVersion];
            using Arm = RigExecBakedPropertyChain::Arm;
            switch (B.propertyChains[size_t(read.propertyChain)].arm) {
            case Arm::Float: *value = VtValue(result.f); return true;
            case Arm::Double: *value = VtValue(result.d); return true;
            case Arm::Vec3f: *value = VtValue(result.v); return true;
            case Arm::Matrix4d: *value = VtValue(result.m); return true;
            }
        }
    } else if (read.spaceValue >= 0 && size_t(read.spaceValue) < B.providerValues.values.size()) {
        const auto &state = B.providerValues.values[size_t(read.spaceValue)];
        if (state.initialized && !state.blocked) {
            const bool available = std::visit([&](const auto &typed) -> bool {
                using T = std::decay_t<decltype(typed)>;
                if constexpr (std::is_same_v<T,std::monostate>) return false;
                else if constexpr (std::is_same_v<T,VtValue>) {
                    *value = typed; return !typed.IsEmpty();
                } else if constexpr (std::is_same_v<T,RigExecPointFrame>) {
                    GfMatrix4d matrix(1);
                    if (!typed.IsValid() || typed.IsDegenerate() ||
                        !RigExecPointsToMatrix(RigExecIdentityLandmarks(),typed,&matrix)) return false;
                    *value = VtValue(matrix); return true;
                } else { *value = VtValue(typed); return true; }
            },state.value);
            if (available) return true;
        }
    }
    if (diagnostic) *diagnostic = read.unavailable;
    return false;
}

bool RigExecBakedBindWeightPointInputs(RigExecBakedProgramImpl *program,std::string *error)
{
    auto &B = *program;
    constexpr const char *names[] = {"rigExec:sampleSource","rigExec:weightTarget","rigExec:curve"};
    const TfToken phaseName("rigExecReadPhase");
    for (auto &field : B.weightFields) {
        field.pointReads.clear();
        SdfPath reader;
        using Form = RigExecBakedProgramImpl::WeightField::Form;
        if (field.form == Form::Revision && field.consumer >= 0 &&
            size_t(field.consumer) < B.revisionIndex.size()) {
            const auto [chain,revision] = B.revisionIndex[size_t(field.consumer)];
            reader = B.chains[size_t(chain)].revisions[size_t(revision)].moverPath;
        } else if (field.form == Form::EnvelopeProperty && field.consumer >= 0 &&
                   size_t(field.consumer) < B.propertyChains.size() && field.part > 0) {
            reader = B.propertyChains[size_t(field.consumer)].revisions[size_t(field.part)-1].mover;
        } else if (field.form == Form::EnvelopeConstraint && field.consumer >= 0 &&
                   size_t(field.consumer) < B.walkSteps.size()) {
            const auto &walk = B.walkSteps[size_t(field.consumer)];
            if (!walk.solverBatch && walk.index >= 0 && size_t(walk.index) < B.constraints.size())
                reader = B.constraints[size_t(walk.index)].path;
        }
        std::set<int> visited;
        const auto visit = [&](const auto &self,int object) -> bool {
            if (object < 0 || !visited.insert(object).second) return true;
            const auto &weight = B.weightObjects[size_t(object)];
            if (!self(self,weight.base)) return false;
            for (const int input : weight.inputs) if (!self(self,input)) return false;
            const auto prim = B.stage->GetPrimAtPath(weight.path);
            for (int leaf = 0; leaf < 3; ++leaf) {
                const auto relation = prim.GetRelationship(TfToken(names[leaf]));
                if (!relation || !relation.HasAuthoredMetadata(phaseName)) continue;
                RigExecReadPhase phase;
                if (!RigExecResolveReadPhase(relation,&phase,error)) return false;
                if (phase.IsBase()) continue;
                if (size_t(leaf) >= weight.oracleLeaves.decl.keys.size()) continue;
                RigExecBakedProgramImpl::WeightField::PointInput input;
                input.object = object; input.leaf = leaf;
                input.binding = RigExecBakedBindPointInput(B,
                    weight.oracleLeaves.decl.keys[size_t(leaf)].path,phase,reader);
                field.pointReads.push_back(std::move(input));
            }
            return true;
        };
        if (!visit(visit,field.object)) return false;
    }
    return true;
}

void RigExecBakedDeclareWeightPointInputs(const RigExecBakedProgramImpl &B,int field,
    std::vector<RigExecBakedSlotRange> *reads)
{
    if (field < 0 || size_t(field) >= B.weightFields.size()) return;
    for (const auto &input : B.weightFields[size_t(field)].pointReads)
        RigExecBakedDeclarePointInput(B,input.binding,reads);
}

RigExecBakedPointInputValue RigExecBakedReadWeightPointInput(
    const RigExecBakedProgramImpl &B,int field,int object,int leaf)
{
    RigExecBakedPointInputValue value;
    if (field < 0 || size_t(field) >= B.weightFields.size()) return value;
    for (const auto &input : B.weightFields[size_t(field)].pointReads) {
        if (input.object != object || input.leaf != leaf) continue;
        value.declared = true;
        if (!input.binding.candidates.empty()) {
            value.available = RigExecBakedResolvePoints(B,input.binding,&value.data,&value.count);
            return value;
        }
        // A plain authored source has no producer version. An unmatched
        // checkpoint remains unavailable rather than substituting raw data.
        if (input.binding.phase.kind == RigExecReadPhaseKind::AtPrim) return value;
        const auto &values = B.weightObjects[size_t(object)].oracleLeaves.values;
        if (leaf >= 0 && size_t(leaf) < values.size() && values[size_t(leaf)].IsHolding<VtVec3fArray>()) {
            const auto &array = values[size_t(leaf)].UncheckedGet<VtVec3fArray>();
            value.available = true; value.data = array.cdata(); value.count = array.size();
        }
        return value;
    }
    return value;
}
}

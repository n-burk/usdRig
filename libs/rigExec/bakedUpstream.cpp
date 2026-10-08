// The baked program's upstream layer: admitted upstream values placed by
// path, compared by value from run to run, and the two sets that admit them
// (the attributes a bake lists as inputs, and those the weight oracle reads).

#include "bakedProgram.h"
#include "inputReplay.h"
#include "bakedProgramImpl.h"
#include "frozenContextInternal.h"
#include "moverGraph.h"
#include "rigEvaluator.h"
#include "rigEvaluatorInternal.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <atomic>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace rigExec {

bool
RigExecUpstreamSlotType(const SdfValueTypeName &typeName)
{
    if (!typeName || typeName.IsArray()) {
        return false;
    }
    const TfType type = typeName.GetType();
    return type == TfType::Find<double>() || type == TfType::Find<float>() ||
           type == TfType::Find<bool>() || type == TfType::Find<int>() ||
           type == TfType::Find<TfToken>() ||
           type == TfType::Find<GfMatrix4d>() ||
           type == TfType::Find<GfVec3d>() || type == TfType::Find<GfVec3f>();
}

namespace {

// The array part of admission. Off until the .rigexec format carries array
// slots (AI); a test turns it on. Read on the owning thread and the
// sampler's, set only by a test between evaluations.
std::atomic<bool> _upstreamArrays{false};

}  // namespace

void
RigExecSetUpstreamArrayAdmissionForTesting(bool on)
{
    RigExecInputReplayObserver::ArrayAdmission(on);
    _upstreamArrays.store(on, std::memory_order_relaxed);
}

bool
RigExecUpstreamArrayAdmission()
{
    return _upstreamArrays.load(std::memory_order_relaxed);
}

bool
RigExecUpstreamArraySlotType(const SdfValueTypeName &typeName)
{
    if (!typeName || !typeName.IsArray()) {
        return false;
    }
    const TfType type = typeName.GetType();
    return type == TfType::Find<VtIntArray>() ||
           type == TfType::Find<VtFloatArray>() ||
           type == TfType::Find<VtDoubleArray>() ||
           type == TfType::Find<VtVec2fArray>() ||
           type == TfType::Find<VtVec3fArray>();
}

std::string
RigExecUpstreamDropReason(const UsdStageRefPtr &stage,
                          const std::map<SdfPath, TfType> *listed,
                          const SdfPath &path, const VtValue &value,
                          UsdTimeCode time, RigExecUpstreamCountMemo *memo,
                          const std::map<SdfPath, TfType> *listedArrays)
{
    const UsdAttribute attribute =
        stage ? stage->GetAttributeAtPath(path) : UsdAttribute();
    if (!attribute) {
        return "no attribute stands there";
    }
    SdfPathVector connections;
    if (attribute.HasAuthoredConnections()) {
        attribute.GetConnections(&connections);
    }
    if (!connections.empty()) {
        return "the attribute is connected";
    }
    if (!attribute.HasValue()) {
        return "the attribute has no stage value";
    }
    const SdfValueTypeName typeName = attribute.GetTypeName();
    const bool array = typeName.IsArray();
    if (array && !RigExecUpstreamArrayAdmission()) {
        return "array values are not admitted";
    }
    if (array ? !RigExecUpstreamArraySlotType(typeName)
              : !RigExecUpstreamSlotType(typeName)) {
        return "no input slot holds a " + typeName.GetAsToken().GetString();
    }
    if (value.GetType() != typeName.GetType()) {
        return "a " + value.GetTypeName() + " value on a " +
               typeName.GetAsToken().GetString() + " attribute";
    }
    if (array ? !listedArrays || !listedArrays->count(path)
              : listed && !listed->count(path)) {
        return "no listed read reaches it";
    }
    if (!array) {
        return std::string();
    }
    // Condition 4: the element count of the stage value at this time, from
    // the memo while the stage cannot vary it.
    const auto known =
        memo ? memo->find(path) : RigExecUpstreamCountMemo::iterator();
    size_t count = 0;
    if (memo && known != memo->end() && !known->second.varying) {
        count = known->second.count;
    } else {
        VtValue stageValue;
        if (!attribute.Get(&stageValue, time) ||
            !stageValue.IsArrayValued()) {
            return "the attribute has no stage value at this time";
        }
        count = stageValue.GetArraySize();
        if (memo && known == memo->end()) {
            RigExecUpstreamCountEntry &entry = (*memo)[path];
            // One time sample is not time-varying, yet Default and a
            // numeric time read different opinions of it.
            entry.varying = attribute.ValueMightBeTimeVarying() ||
                            attribute.GetNumTimeSamples() > 0;
            entry.count = entry.varying ? 0 : count;
        }
    }
    if (value.GetArraySize() != count) {
        return "its element count " + std::to_string(value.GetArraySize()) +
               " differs from the stage value's " + std::to_string(count);
    }
    return std::string();
}

std::vector<RigExecUpstreamArrayRow>
RigExecBakedUpstreamAdmissibleArrays(const RigExecRigEvaluator &evaluator)
{
    return RigExecBakedProgram::DescribeUpstreamArrays(evaluator);
}

std::vector<RigExecUpstreamArrayRow>
RigExecBakedProgram::DescribeUpstreamArrays(const RigExecRigEvaluator &E)
{
    using Time = RigExecUpstreamArrayRow::Time;
    std::map<SdfPath, RigExecUpstreamArrayRow> rows;
    const UsdStageRefPtr &stage = E._stage;
    const auto add = [&](const SdfPath &path, Time time,
                         const std::string &consumer) {
        if (path.IsEmpty() || !path.IsPrimPropertyPath()) {
            return;
        }
        const UsdAttribute a = stage->GetAttributeAtPath(path);
        if (!a || !RigExecUpstreamArraySlotType(a.GetTypeName())) {
            return;
        }
        const auto [it, fresh] = rows.try_emplace(path);
        RigExecUpstreamArrayRow &row = it->second;
        if (fresh) {
            row.path = path;
            row.type = a.GetTypeName().GetType();
            row.time = time;
            row.consumer = consumer;
            return;
        }
        if (row.time != time) {
            row.time = Time::Both;
        }
        if (("," + row.consumer + ",").find("," + consumer + ",") ==
            std::string::npos) {
            row.consumer += "," + consumer;
        }
    };
    const auto isArrayLeaf = [](RigExecRevisionLeafType type) {
        return type == RigExecRevisionLeafType::IntArray ||
               type == RigExecRevisionLeafType::FloatArray ||
               type == RigExecRevisionLeafType::DoubleArray ||
               type == RigExecRevisionLeafType::Vec2fArray ||
               type == RigExecRevisionLeafType::Vec3fArray;
    };
    std::set<SdfPath> weightObjects;
    // The objects whose packet exec computes in the dynamic walk (a point
    // revision's weight tap): seeds of the exec-read points below.
    std::set<SdfPath> execWeightObjects;
    const auto revisionRows = [&](const auto &revision, const char *owner) {
        RigExecRevisionLeafDecl decl;
        RigExecDeclareRevisionLeaves(revision.op, revision.moverPath,
                                     revision.binding, &decl);
        const int indices = decl.Role(RigExecRevisionLeafRole::JointIndices);
        const int weights = decl.Role(RigExecRevisionLeafRole::JointWeights);
        for (size_t k = 0; k < decl.keys.size(); ++k) {
            const RigExecRevisionLeafKey &key = decl.keys[k];
            if (!isArrayLeaf(key.type)) {
                continue;
            }
            const bool layout = int(k) == indices || int(k) == weights;
            add(key.path,
                key.time == RigExecRevisionLeafTime::AtDefault
                    ? Time::AtDefault
                    : Time::AtTime,
                layout ? "skin layout" : owner);
        }
        for (const auto &[input, samples] : revision.binding.blendSamples) {
            for (const RigExecBlendSampleBinding &sample : samples) {
                if (sample.blendShape.IsEmpty()) {
                    add(sample.points, Time::AtTime, "blend sample");
                }
            }
        }
        if (!revision.binding.weightObject.IsEmpty()) {
            weightObjects.insert(revision.binding.weightObject);
            execWeightObjects.insert(revision.binding.weightObject);
        }
    };
    for (const auto &[target, revisions] : E._graphChains) {
        add(target, Time::AtTime, "chain base");
        for (const auto &revision : revisions) {
            revisionRows(revision, "revision");
        }
    }
    // A derived target's base is not a leaf: only its revisions' reads.
    for (const auto &[target, revisions] : E._graphDerivedChains) {
        for (const auto &revision : revisions) {
            revisionRows(revision, "derived");
        }
    }

    // Every weight object a mover or a constraint binds, and what each
    // composes.
    const TfToken weightObjectRel("rigExec:weightObject");
    for (const RigExecMoverRecord &mover : E._movers) {
        if (const UsdPrim prim = stage->GetPrimAtPath(mover.moverPath)) {
            SdfPathVector targets;
            if (const UsdRelationship rel =
                    prim.GetRelationship(weightObjectRel)) {
                rel.GetTargets(&targets);
            }
            weightObjects.insert(targets.begin(), targets.end());
        }
    }
    for (const auto &constraint : E._frameConstraints) {
        if (!constraint.weightObject.IsEmpty()) {
            weightObjects.insert(constraint.weightObject);
        }
    }
    const TfToken baseWeightRel("rigExec:baseWeight");
    const TfToken inputWeightsRel("rigExec:inputWeights");
    const TfToken weightTargetRel("rigExec:weightTarget");
    const TfToken sampleSourceRel("rigExec:sampleSource");
    const TfToken curveRel("rigExec:curve");
    const TfToken combineType("RigExecCombineWeight");
    // Visits every object \p seeds reach through composition and hands
    // \p point each array its packet gathers (authored property targets)
    // and, as `true`, the one target the oracle reads, canonicalized.
    const auto visitPoints = [&](const std::set<SdfPath> &seeds,
                                 const auto &point) {
        std::set<SdfPath> visited;
        std::vector<SdfPath> pending(seeds.begin(), seeds.end());
        while (!pending.empty()) {
            const SdfPath path = pending.back();
            pending.pop_back();
            if (!visited.insert(path).second) {
                continue;
            }
            const UsdPrim prim = stage->GetPrimAtPath(path);
            if (!prim) {
                continue;
            }
            const auto targetsOf = [&prim](const TfToken &name) {
                SdfPathVector targets;
                if (const UsdRelationship rel = prim.GetRelationship(name)) {
                    rel.GetTargets(&targets);
                }
                return targets;
            };
            for (const TfToken &name : {baseWeightRel, inputWeightsRel}) {
                for (const SdfPath &t : targetsOf(name)) {
                    pending.push_back(t);
                }
            }
            const TfToken type = prim.GetTypeName();
            const bool volumetric =
                evaluatorDetail::_IsVolumeWeightType(type);
            if (!volumetric && type != combineType) {
                continue;
            }
            const std::vector<TfToken> relationships =
                volumetric ? std::vector<TfToken>{weightTargetRel,
                                                  sampleSourceRel, curveRel}
                           : std::vector<TfToken>{weightTargetRel};
            for (const TfToken &name : relationships) {
                const SdfPathVector targets = targetsOf(name);
                for (const SdfPath &t : targets) {
                    if (t.IsPropertyPath()) {
                        point(t, false);
                    }
                }
                if (volumetric && targets.size() == 1) {
                    point(evaluatorDetail::_ResolveGeometryInput(stage,
                                                                 targets[0]),
                          true);
                }
            }
        }
    };
    visitPoints(weightObjects, [&](const SdfPath &path, bool oracle) {
        add(path, Time::AtTime, oracle ? "weight oracle" : "weight packet");
    });
    // Exec reads a computed packet's points element by element, and an exec
    // value override cannot replace such a read with an array, so the
    // dynamic walk could not follow a value there: those attributes are
    // structural for every reader and in every backend.
    visitPoints(execWeightObjects, [&rows](const SdfPath &path, bool) {
        rows.erase(path);
    });

    std::vector<RigExecUpstreamArrayRow> out;
    out.reserve(rows.size());
    for (auto &[path, row] : rows) {
        out.push_back(std::move(row));
    }
    return out;
}

void
RigExecBakedProgram::SetUpstreamInputs(
    const std::vector<RigExecValueOverride> &inputs)
{
    RigExecBakedProgramImpl &B = *_impl;
    if (inputs.empty() && B.upstream.empty()) {
        return;
    }
    B.upstream.clear();
    for (const RigExecValueOverride &o : inputs) {
        if (!o.attribute.IsEmpty() && o.computation.IsEmpty()) {
            B.upstream[o.prim.AppendProperty(o.attribute)] = o.value;
        }
    }
}

namespace {

// The attributes RigExecResolvedInputs::GetAttribute walks from \p head, as
// the exporter's Walk lists them (RigExecBakedClassifyInput's walk).
void
_AddWalk(const RigExecBakedProgramImpl &B, const UsdAttribute &head,
         std::set<SdfPath> *paths)
{
    if (!head) {
        return;
    }
    bool viaChain = false, varying = false;
    UsdAttribute selected;
    SdfPathVector walk;
    RigExecBakedClassifyInput<float>(head, UsdTimeCode::Default(),
                                     B.chainTargets, &viaChain, &varying,
                                     &selected, &walk);
    paths->insert(walk.begin(), walk.end());
}

// Builds `upstreamAdmissible` and `upstreamOracle`, restating the reads the
// .rigexec exporter lists as input slots (rigExecBake computedCapture.cpp)
// over the program's own tables, and the oracle-resolved objects its
// descending pass marks. Owning thread.
void
_BuildUpstreamSets(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecRigEvaluator &E = *B.evaluator;
    std::set<SdfPath> paths;

    // Every registered binding the visitor reaches: the paths Build filed
    // its override number under, which are its walk from the head.
    std::map<int, std::vector<SdfPath>> byNumber;
    for (const auto &[path, numbers] : B.overridableInputs) {
        for (const int number : numbers) {
            byNumber[number].push_back(path);
        }
    }
    frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
        if (!input.head || input.overrideIndex < 0) {
            return;
        }
        const auto found = byNumber.find(input.overrideIndex);
        if (found != byNumber.end()) {
            paths.insert(found->second.begin(), found->second.end());
        }
    });

    // The property chains: each target, each revision's five reads (no
    // min or max on a matrix chain), each phased consumer and its hops.
    std::vector<RigExecBakedPropertyChainDesc> chains;
    std::string why;
    const bool described =
        RigExecBakedProgram::DescribePropertyChains(E, &chains, &why);
    if (described) {
        for (const RigExecBakedPropertyChainDesc &chain : chains) {
            paths.insert(chain.target);
            const bool matrix = chain.valueType ==
                RigExecBakedPropertyChainDesc::ValueType::Matrix4d;
            for (const auto &r : chain.revisions) {
                _AddWalk(B, r.enabled, &paths);
                _AddWalk(B, r.defaultWeight, &paths);
                _AddWalk(B, r.value, &paths);
                if (!matrix) {
                    _AddWalk(B, r.minimum, &paths);
                    _AddWalk(B, r.maximum, &paths);
                }
            }
            for (const auto &phased : chain.phased) {
                paths.insert(phased.consumer);
                paths.insert(phased.hops.begin(), phased.hops.end());
            }
        }
    }

    // The envelope-only weight objects' six reads.
    std::vector<RigExecBakedEnvelopeObject> envelopes;
    std::map<SdfPath, int> envelopeIndex;
    const bool composed = RigExecBakedProgram::ComposeEnvelopeObjects(
        E, B, &envelopes, &envelopeIndex, &why);
    const auto envelopeReads = [](const RigExecBakedEnvelopeObject &object) {
        return std::vector<const RigExecBakedEnvelopeObject::Read *>{
            &object.defaultWeight, &object.driver,   &object.scale,
            &object.bias,          &object.strength, &object.invert};
    };
    if (composed) {
        for (const RigExecBakedEnvelopeObject &object : envelopes) {
            for (const auto *read : envelopeReads(object)) {
                _AddWalk(B, read->head, &paths);
            }
        }
    }

    // The geometry assembly's scalar reads: blend weights and activations,
    // a revision's inputs:defaultWeight, and the connection-following mover
    // inputs (enable, skin layout scalars, iterative scalars, shader dials,
    // projector ray settings).
    const TfToken defaultWeightName("inputs:defaultWeight");
    const auto moverReads =
        [&](const RigExecBakedProgramImpl::GeomRevision &revision) {
            for (const auto &channel : revision.blendChannels) {
                _AddWalk(B, channel.weight, &paths);
                for (const auto &sample : channel.samples) {
                    _AddWalk(B, sample.activation, &paths);
                }
            }
            const UsdPrim &prim = revision.moverPrim;
            if (!prim) {
                return;
            }
            const RigExecRevisionOp op = revision.op;
            const auto attr = [&prim](const char *name) {
                return prim.GetAttribute(TfToken(name));
            };
            if (op != RigExecRevisionOp::RecomputeNormals &&
                op != RigExecRevisionOp::RecomputeExtent &&
                !RigExecIsDerivedMatrixOp(op)) {
                _AddWalk(B, attr("inputs:enabled"), &paths);
            }
            if (op == RigExecRevisionOp::External) {
                const auto numeric = [](RigExecRevisionLeafType type) {
                    using Type = RigExecRevisionLeafType;
                    return type == Type::Double || type == Type::Dial ||
                           type == Type::Float || type == Type::Bool ||
                           type == Type::Int || type == Type::Matrix4d ||
                           type == Type::Vec3d || type == Type::Vec3f;
                };
                const auto addSource = [&](const SdfPath &path, RigExecRevisionLeafType requested) {
                    const auto attribute = B.stage->GetAttributeAtPath(path);
                    if (!attribute || !attribute.HasValue() ||
                        !RigExecUpstreamSlotType(attribute.GetTypeName()) ||
                        attribute.GetTypeName().GetType() == TfType::Find<TfToken>()) return;
                    const auto actual = attribute.GetTypeName().GetType();
                    using Type = RigExecRevisionLeafType;
                    const bool compatible =
                        ((requested == Type::Float || requested == Type::Dial) &&
                         (actual == TfType::Find<float>() || actual == TfType::Find<double>())) ||
                        (requested == Type::Double && actual == TfType::Find<double>()) ||
                        (requested == Type::Bool && actual == TfType::Find<bool>()) ||
                        (requested == Type::Int && actual == TfType::Find<int>()) ||
                        (requested == Type::Matrix4d && actual == TfType::Find<GfMatrix4d>()) ||
                        (requested == Type::Vec3d && actual == TfType::Find<GfVec3d>()) ||
                        (requested == Type::Vec3f && actual == TfType::Find<GfVec3f>());
                    if (!compatible) return;
                    SdfPathVector connections;
                    if (attribute.HasAuthoredConnections()) attribute.GetConnections(&connections);
                    if (connections.empty()) paths.insert(path);
                };
                for (size_t k = 0; k < revision.binding.externalInputs.size(); ++k) {
                    const auto &key = revision.binding.externalInputs[k];
                    using Flavour = RigExecRevisionLeafFlavour;
                    if (key.time != RigExecRevisionLeafTime::AtTime || !numeric(key.type) ||
                        (key.flavour != Flavour::Resolved && key.flavour != Flavour::ResolvedOnly &&
                         key.flavour != Flavour::OverlayThenRaw)) continue;
                    addSource(key.path, key.type);
                    if (revision.leaves.decl.externalBegin < 0) continue;
                    const size_t leaf = size_t(revision.leaves.decl.externalBegin) + k;
                    if (leaf < revision.leaves.hops.size())
                        for (const auto &path : revision.leaves.hops[leaf]) addSource(path, key.type);
                }
            }
            if (op == RigExecRevisionOp::Skin) {
                _AddWalk(B, attr("rigExec:elementSize"), &paths);
                _AddWalk(B, attr("rigExec:skinningMethod"), &paths);
            }
            if (op == RigExecRevisionOp::DeltaMush ||
                op == RigExecRevisionOp::Wrinkle) {
                RigExecMoverParameters parameters;
                frozenDetail::_VisitIterativeMoverScalars(
                    op, parameters, [&](const char *name, auto, auto &) {
                        _AddWalk(B, attr(name), &paths);
                    });
            }
            if (op == RigExecRevisionOp::ShaderDials) {
                for (const SdfPath &dial : revision.binding.shaderDials) {
                    _AddWalk(B, B.stage->GetAttributeAtPath(dial), &paths);
                }
            } else if (RigExecIsDerivedMatrixOp(op)) {
                for (const char *name : {"rigExec:rayOrigin",
                                         "rigExec:rayDirection",
                                         "rigExec:rayUp",
                                         "rigExec:shaderOffset"}) {
                    _AddWalk(B, attr(name), &paths);
                }
            }
        };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.moverPrim) {
                _AddWalk(B, revision.moverPrim.GetAttribute(defaultWeightName),
                         &paths);
            }
            moverReads(revision);
        }
        for (const auto &derived : chain.derived) {
            moverReads(derived.revision);
        }
    }

    B.upstreamAdmissible.clear();
    for (const SdfPath &path : paths) {
        const UsdAttribute a = B.stage->GetAttributeAtPath(path);
        if (a && RigExecUpstreamSlotType(a.GetTypeName())) {
            B.upstreamAdmissible.emplace_hint(B.upstreamAdmissible.end(),
                                              path,
                                              a.GetTypeName().GetType());
        }
    }

    // The objects the volatile oracle resolves, over the exporter's index
    // space (step-backed objects, then the envelope list): constraint and
    // property-mover envelopes and current-phase fields, then whatever they
    // compose, in one descending pass.
    const size_t baked = B.weightObjects.size();
    const size_t total = baked + (composed ? envelopes.size() : 0);
    std::vector<char> resolved(total, 0);
    const auto indexOf = [&](const SdfPath &object) {
        const auto found = B.weightIndex.find(object);
        if (found != B.weightIndex.end() && found->second >= 0) {
            return found->second;
        }
        const auto env = envelopeIndex.find(object);
        return env != envelopeIndex.end() && composed ? env->second : -1;
    };
    const auto mark = [&](int index) {
        if (index >= 0 && size_t(index) < total) {
            resolved[size_t(index)] = 1;
        }
    };
    for (const RigExecBakedProgramImpl::Constraint &c : B.constraints) {
        if (!c.weightObject.IsEmpty() && c.pointsTarget.IsEmpty()) {
            mark(indexOf(c.weightObject));
        }
    }
    if (described) {
        for (const RigExecBakedPropertyChainDesc &chain : chains) {
            for (const auto &r : chain.revisions) {
                if (!r.weightObject.IsEmpty()) {
                    mark(indexOf(r.weightObject));
                }
            }
        }
    }
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.weightCurrentPhase) {
                mark(revision.weightObject);
            }
        }
    }
    B.upstreamOracle.clear();
    for (size_t i = total; i-- > 0;) {
        if (!resolved[i]) {
            continue;
        }
        std::set<SdfPath> hops;
        if (i < baked) {
            const RigExecBakedProgramImpl::WeightObject &w = B.weightObjects[i];
            for (const RigExecBakedInput<float> *input :
                 {&w.defaultWeight, &w.driver,    &w.scale,     &w.bias,
                  &w.strength,      &w.invert,    &w.falloffMin,
                  &w.falloffMax,    &w.scaleXPos, &w.scaleYPos,
                  &w.scaleZPos,     &w.scaleXNeg, &w.scaleYNeg,
                  &w.scaleZNeg,     &w.scaleX,    &w.scaleY,
                  &w.scaleZ,        &w.extentU,   &w.extentV}) {
                _AddWalk(B, input->head, &hops);
            }
            mark(w.base);
            for (const int input : w.inputs) {
                mark(input);
            }
        } else {
            const RigExecBakedEnvelopeObject &object = envelopes[i - baked];
            for (const auto *read : envelopeReads(object)) {
                _AddWalk(B, read->head, &hops);
            }
            mark(object.base);
            for (const int input : object.inputs) {
                mark(input);
            }
        }
        B.upstreamOracle.insert(hops.begin(), hops.end());
    }
    B.upstreamSetsBuilt = true;
}

}  // namespace

const std::map<SdfPath, TfType> &
RigExecBakedProgram::GetUpstreamAdmissible() const
{
    if (!_impl->upstreamSetsBuilt) {
        _BuildUpstreamSets(_impl.get());
    }
    return _impl->upstreamAdmissible;
}

const std::set<SdfPath> &
RigExecBakedProgram::GetUpstreamOracle() const
{
    if (!_impl->upstreamSetsBuilt) {
        _BuildUpstreamSets(_impl.get());
    }
    return _impl->upstreamOracle;
}

void
RigExecBakedPlaceUpstream(RigExecBakedProgramImpl *program, bool placeOracle)
{
    RigExecBakedProgramImpl &B = *program;
    if (B.upstreamMovedThisRun) {
        std::fill(B.upstreamChanged.begin(), B.upstreamChanged.end(), 0);
        B.upstreamMovedThisRun = false;
    }
    const size_t numbers = B.overridden.size();
    const bool sized = B.upstreamOn.size() == numbers &&
                       B.upstreamChanged.size() == numbers;
    if (sized && B.upstream == B.lastUpstream) {
        if (B.upstream.empty()) {
            return;
        }
    } else {
        B.upstreamOn.assign(numbers, 0);
        B.upstreamChanged.resize(numbers, 0);
        // Rule 8: every leaf filed under a value placed, moved or lifted
        // re-reads, through the layer; and the readers of each override
        // number on such a path are seeded this run, once.
        const auto moved = [&B, numbers](const SdfPath &path) {
            const auto leaves = B.leafByPath.find(path);
            if (leaves != B.leafByPath.end()) {
                for (const uint32_t id : leaves->second) {
                    RigExecBakedMarkLeaf(&B, id);
                }
            }
            const auto inputs = B.overridableInputs.find(path);
            if (inputs != B.overridableInputs.end()) {
                for (const int index : inputs->second) {
                    if (index >= 0 && size_t(index) < numbers) {
                        B.upstreamChanged[size_t(index)] = 1;
                    }
                }
            }
            B.upstreamMovedThisRun = true;
        };
        for (const auto &[path, value] : B.upstream) {
            const auto last = B.lastUpstream.find(path);
            if (last == B.lastUpstream.end() || !(last->second == value)) {
                moved(path);
            }
            const auto inputs = B.overridableInputs.find(path);
            if (inputs != B.overridableInputs.end()) {
                for (const int index : inputs->second) {
                    if (index >= 0 && size_t(index) < numbers) {
                        B.upstreamOn[size_t(index)] = 1;
                    }
                }
            }
        }
        for (const auto &[path, value] : B.lastUpstream) {
            if (!B.upstream.count(path)) {
                moved(path);
            }
        }
        B.lastUpstream = B.upstream;
    }
    // The oracle reads the generation's resolved inputs and the stage, so
    // a value on one of its paths is placed there too, beneath any
    // interactive override placed after it and any chain result published
    // over it, as the dynamic walk places it.
    if (placeOracle && !B.upstream.empty()) {
        if (!B.upstreamSetsBuilt) {
            _BuildUpstreamSets(&B);
        }
        const std::set<SdfPath> &oracle = B.upstreamOracle;
        for (const auto &[path, value] : B.upstream) {
            if (oracle.count(path)) {
                B.resolvedInputs->SetProperty(path, value);
            }
        }
    }
}

}  // namespace rigExec

// The baked program's upstream layer: admitted upstream values placed by
// path, compared by value from run to run, and the two sets that admit them
// (the attributes a bake lists as inputs, and those the weight oracle reads).

#include "bakedProgram.h"
#include "bakedProgramImpl.h"
#include "frozenContextInternal.h"
#include "moverGraph.h"
#include "rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
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
RigExecBakedPlaceUpstream(RigExecBakedProgramImpl *program)
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
    if (!B.upstream.empty()) {
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

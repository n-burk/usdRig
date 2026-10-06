#include "rigExecBake/propertyChainsBake.h"

#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecMath/propertyMath.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <map>
#include <set>
#include <unordered_map>

namespace rigExec {

namespace {

// What a revision input reads as: the evaluator's _PinnedRead type.
enum class Reading { Bool, Float };

// The attributes a read of `attribute` visits through authored connections,
// in RigExecResolvedInputs::GetAttribute's order: itself, then each single
// connection's source, stopping at a fork, a dead end or a cycle.
std::vector<UsdAttribute>
Walk(const UsdAttribute &attribute)
{
    std::vector<UsdAttribute> hops;
    std::set<SdfPath> visiting;
    UsdAttribute a = attribute;
    while (a && visiting.insert(a.GetPath()).second) {
        hops.push_back(a);
        if (!a.HasAuthoredConnections()) {
            break;
        }
        SdfPathVector connections;
        a.GetConnections(&connections);
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
    }
    return hops;
}

// The value the walk falls back to at `time` when no step holds a live
// value: GetAttribute's nearest readable authored value from the end. A
// float read that meets a double step continues as a double read from that
// step, so only the steps from there on are candidates, read as double.
bool
Fallback(const std::vector<UsdAttribute> &hops, Reading reading,
         UsdTimeCode time, double *out)
{
    if (reading == Reading::Bool) {
        for (auto it = hops.rbegin(); it != hops.rend(); ++it) {
            bool value = false;
            if (it->Get(&value, time)) {
                *out = value ? 1.0 : 0.0;
                return true;
            }
        }
        return false;
    }
    size_t first = 0;
    bool asDouble = false;
    for (size_t i = 0; i < hops.size(); ++i) {
        if (hops[i].GetTypeName() == SdfValueTypeNames->Double) {
            first = i;
            asDouble = true;
            break;
        }
    }
    for (size_t i = hops.size(); i-- > first;) {
        if (asDouble) {
            double value = 0.0;
            if (hops[i].Get(&value, time)) {
                *out = value;
                return true;
            }
        } else {
            float value = 0.0f;
            if (hops[i].Get(&value, time)) {
                *out = double(value);
                return true;
            }
        }
    }
    return false;
}

bool
ConstantScalar(const VtValue &value, double *out)
{
    if (value.IsHolding<bool>()) {
        *out = value.UncheckedGet<bool>() ? 1.0 : 0.0;
    } else if (value.IsHolding<float>()) {
        *out = double(value.UncheckedGet<float>());
    } else if (value.IsHolding<double>()) {
        *out = value.UncheckedGet<double>();
    } else {
        return false;
    }
    return true;
}

}  // namespace

bool
RigExecBakePropertyChains(const RigExecRigEvaluator &evaluator,
                          const std::vector<double> &frames,
                          const RigExecWireInputTable &table,
                          RigExecBinaryWriter *writer,
                          RigExecWirePropertyChains *out,
                          std::vector<std::string> *skipped,
                          std::string *error)
{
    RigExecChainSampleBindings bindings;
    if (!RigExecBindChainSampleInputs(evaluator, &bindings, error)) {
        return false;
    }
    const UsdStageRefPtr &stage = evaluator.GetEvaluationStage();
    const auto skip = [skipped](const SdfPath &target, const char *why) {
        if (skipped) {
            skipped->push_back(target.GetString() + ": " + why);
        }
    };

    // Unsupported until it is a constant, a walk, or absent.
    bool inputOk = true;
    const auto input = [&](const RigExecChainSampleInput &in,
                           Reading reading) {
        RigExecWirePropertyChainInput wire;
        if (!in.attribute) {
            return wire;
        }
        if (in.constant) {
            wire.kind = RigExecWirePropertyChainInput::Kind::Constant;
            if (!ConstantScalar(in.constantValue, &wire.constant)) {
                inputOk = false;
            }
            return wire;
        }
        wire.kind = RigExecWirePropertyChainInput::Kind::Walk;
        const std::vector<UsdAttribute> hops = Walk(in.attribute);
        for (const UsdAttribute &hop : hops) {
            wire.hops.push_back(writer->AddString(hop.GetPath().GetString()));
        }
        for (double frame : frames) {
            double value = 0.0;
            const bool have =
                Fallback(hops, reading, UsdTimeCode(frame), &value);
            wire.frameHave.push_back(have ? 1 : 0);
            wire.frameValues.push_back(have ? value : 0.0);
        }
        return wire;
    };

    std::unordered_map<SdfPath, uint32_t, SdfPath::Hash> chainOf;
    for (const RigExecChainSampleChain &chain : bindings.chains) {
        const bool isFloat = chain.valueType == SdfValueTypeNames->Float;
        const bool isDouble = chain.valueType == SdfValueTypeNames->Double;
        if (!chain.target || (!isFloat && !isDouble)) {
            skip(chain.targetPath, "value type other than float or double");
            continue;
        }
        RigExecWirePropertyChain wire;
        wire.target = writer->AddString(chain.targetPath.GetString());
        wire.valueType = isFloat ? RigExecWirePropertyChain::ValueType::Float
                                 : RigExecWirePropertyChain::ValueType::Double;
        for (double frame : frames) {
            double base = 0.0;
            bool have = false;
            if (isFloat) {
                float value = 0.0f;
                have = chain.targetQuery.Get(&value, UsdTimeCode(frame));
                base = double(value);
            } else {
                have = chain.targetQuery.Get(&base, UsdTimeCode(frame));
            }
            wire.frameBaseHave.push_back(have ? 1 : 0);
            wire.frameBase.push_back(have ? base : 0.0);
        }
        bool ok = true;
        for (const RigExecChainSampleRevision &revision : chain.revisions) {
            if (!revision.weightObjects.empty()) {
                skip(chain.targetPath, "a revision binds a weight object");
                ok = false;
                break;
            }
            if (!revision.moverPrim) {
                continue;
            }
            RigExecWirePropertyChainRevision r;
            r.mover = writer->AddString(revision.moverPath.GetString());
            // The operation is folded: a varying operation would change
            // which kernel runs, which the section does not carry.
            TfToken operation;
            if (revision.operation) {
                if (!revision.operation.constant ||
                    !revision.operation.constantValue.IsHolding<TfToken>()) {
                    skip(chain.targetPath, "operation is not constant");
                    ok = false;
                    break;
                }
                operation =
                    revision.operation.constantValue.UncheckedGet<TfToken>();
            }
            RigExecPropertyOp op;
            if (!RigExecParsePropertyOp(operation, &op)) {
                // The evaluator passes such a revision through; so does the
                // runtime, by never running it. Kept out of the program.
                continue;
            }
            r.op = uint8_t(op);
            inputOk = true;
            r.enabled = input(revision.enabled, Reading::Bool);
            r.defaultWeight = input(revision.defaultWeight, Reading::Float);
            r.value = input(revision.value, Reading::Float);
            r.minimum = input(revision.minimum, Reading::Float);
            r.maximum = input(revision.maximum, Reading::Float);
            if (!inputOk) {
                skip(chain.targetPath, "an input holds an unsupported type");
                ok = false;
                break;
            }
            if (op == RigExecPropertyOp::Curve) {
                for (const auto *in : {&revision.keys, &revision.tangents}) {
                    if (in->attribute && !in->constant) {
                        skip(chain.targetPath, "curve keys are not constant");
                        ok = false;
                        break;
                    }
                }
                if (!ok) {
                    break;
                }
                VtArray<GfVec2f> keys, tangents;
                if (revision.keys && revision.keys.constantValue
                                         .IsHolding<VtArray<GfVec2f>>()) {
                    keys = revision.keys.constantValue
                               .UncheckedGet<VtArray<GfVec2f>>();
                }
                if (revision.tangents &&
                    revision.tangents.constantValue
                        .IsHolding<VtArray<GfVec2f>>()) {
                    tangents = revision.tangents.constantValue
                                   .UncheckedGet<VtArray<GfVec2f>>();
                }
                for (const GfVec2f &k : keys) {
                    r.keys.push_back({k[0], k[1]});
                }
                for (const GfVec2f &t : tangents) {
                    r.tangents.push_back({t[0], t[1]});
                }
            }
            wire.revisions.push_back(std::move(r));
        }
        if (!ok) {
            continue;
        }
        chainOf[chain.targetPath] = uint32_t(out->chains.size());
        out->chains.push_back(std::move(wire));
    }

    // Consumers: every directory entry whose head is a chain target, or
    // whose head's walk reaches one first.
    std::unordered_map<uint32_t, uint32_t> chainOfHead;
    for (const auto &[target, index] : chainOf) {
        chainOfHead[writer->AddString(target.GetString())] = index;
    }
    std::set<uint32_t> heads;
    for (const RigExecWireInputDirectoryEntry &entry : table.directory) {
        if (entry.head != 0) {
            heads.insert(entry.head);
        }
    }
    for (const UsdPrim &prim : stage->Traverse()) {
        for (const UsdAttribute &attribute : prim.GetAttributes()) {
            if (!attribute.HasAuthoredConnections()) {
                continue;
            }
            const std::vector<UsdAttribute> hops = Walk(attribute);
            for (size_t i = 1; i < hops.size(); ++i) {
                const auto it = chainOf.find(hops[i].GetPath());
                if (it == chainOf.end()) {
                    continue;
                }
                const uint32_t id =
                    writer->AddString(attribute.GetPath().GetString());
                if (heads.count(id)) {
                    chainOfHead[id] = it->second;
                }
                break;
            }
        }
    }
    for (size_t uid = 0; uid < table.directory.size(); ++uid) {
        const auto it = chainOfHead.find(table.directory[uid].head);
        if (table.directory[uid].head != 0 && it != chainOfHead.end()) {
            out->consumers.push_back({uint32_t(uid), it->second});
        }
    }
    return true;
}

}  // namespace rigExec

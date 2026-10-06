// .rigexec array inputs.
#include "rigExecBake/arrayReads.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluatorInternal.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/valueTypeName.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <set>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

using Consumer = RigExecBakeArrayConsumer;
using Role = RigExecRevisionLeafRole;

// The input tag of an array leaf type; false for a scalar one.
bool
_LeafTag(RigExecRevisionLeafType type, fb::InputTag *tag)
{
    switch (type) {
    case RigExecRevisionLeafType::IntArray:
        *tag = fb::InputTag::IntArray;
        return true;
    case RigExecRevisionLeafType::FloatArray:
        *tag = fb::InputTag::FloatArray;
        return true;
    case RigExecRevisionLeafType::DoubleArray:
        *tag = fb::InputTag::DoubleArray;
        return true;
    case RigExecRevisionLeafType::Vec2fArray:
        *tag = fb::InputTag::Vec2fArray;
        return true;
    case RigExecRevisionLeafType::Vec3fArray:
        *tag = fb::InputTag::Vec3fArray;
        return true;
    default:
        return false;
    }
}

// Whether \p a holds an array of \p tag's element type, any role: the
// type a typed Get of the read's array succeeds on.
bool
_HoldsTag(const UsdAttribute &a, fb::InputTag tag)
{
    if (!a) {
        return false;
    }
    const SdfValueTypeName typeName = a.GetTypeName();
    if (!typeName || !typeName.IsArray()) {
        return false;
    }
    const TfType type = typeName.GetType();
    switch (tag) {
    case fb::InputTag::IntArray:
        return type == TfType::Find<VtIntArray>();
    case fb::InputTag::FloatArray:
        return type == TfType::Find<VtFloatArray>();
    case fb::InputTag::DoubleArray:
        return type == TfType::Find<VtDoubleArray>();
    case fb::InputTag::Vec2fArray:
        return type == TfType::Find<VtVec2fArray>();
    case fb::InputTag::Vec3fArray:
        return type == TfType::Find<VtVec3fArray>();
    default:
        return false;
    }
}

struct _Lister {
    const RigExecBakedProgramImpl &program;
    UsdStageRefPtr stage;
    UsdTimeCode time;
    std::vector<RigExecBakeArrayRead> *reads;

    // The attributes \p key's read reaches (RigExecRevisionLeafHops), into
    // \p read; false when one is not an array of \p tag, which no slot of
    // that tag can stand for.
    bool Hops(const RigExecRevisionLeafKey &key, fb::InputTag tag,
              RigExecBakeArrayRead *read) const
    {
        const UsdAttribute head = stage->GetAttributeAtPath(key.path);
        if (!_HoldsTag(head, tag)) {
            return false;
        }
        bool varying = false;
        RigExecRevisionLeafHops(key, head, &read->hops, &varying);
        for (const SdfPath &hop : read->hops) {
            if (!_HoldsTag(stage->GetAttributeAtPath(hop), tag)) {
                return false;
            }
        }
        read->tag = tag;
        read->rest = key.time == RigExecRevisionLeafTime::AtDefault;
        return true;
    }

    // A raw read of the attribute at \p path, of \p tag.
    bool Raw(const SdfPath &path, fb::InputTag tag,
             RigExecBakeArrayRead *read) const
    {
        if (!_HoldsTag(stage->GetAttributeAtPath(path), tag)) {
            return false;
        }
        read->hops = {path};
        read->tag = tag;
        return true;
    }

    // One chain revision or derived target: its assembly's arrays as rows,
    // its dense blend samples' points, and a fixed main skin's layout.
    bool Revision(const RigExecBakedProgramImpl::GeomRevision &revision,
                  uint32_t chain, uint32_t index, bool derived,
                  std::string *error) const
    {
        const RigExecRevisionLeafDecl &decl = revision.leaves.decl;
        std::vector<int> roleOf(decl.keys.size(), -1);
        for (size_t role = 0; role < size_t(Role::Count); ++role) {
            const int k = decl.roles[role];
            if (k >= 0 && size_t(k) < roleOf.size()) {
                roleOf[size_t(k)] = int(role);
            }
        }
        std::set<int> blendPoints;
        for (const auto &channel : revision.blendChannels) {
            for (const auto &sample : channel.samples) {
                blendPoints.insert(sample.pointsLeaf);
            }
        }
        // The points inputs a read phase answers: live takes the phase
        // overlay there before the leaf, and the file's binding answers
        // alike on every run. One the phase leaves unanswered reads its
        // leaf.
        std::set<SdfPath> phased;
        for (const RigExecBakedPointsBinding &binding :
             revision.pointBindings) {
            const GfVec3f *points = nullptr;
            size_t count = 0;
            if (RigExecBakedResolvePoints(program, binding, &points,
                                          &count)) {
                phased.insert(binding.input);
            }
        }
        const bool fixed = !derived && revision.skinTopologyFixed;
        const int transformDrivers = revision.binding.driverTransformCount;
        for (size_t k = 0; k < decl.keys.size(); ++k) {
            const RigExecRevisionLeafKey &key = decl.keys[k];
            fb::InputTag tag = fb::InputTag::FloatArray;
            if (!_LeafTag(key.type, &tag)) {
                continue;
            }
            const int role = roleOf[k];
            if (role < 0) {
                if (blendPoints.count(int(k))) {
                    continue;
                }
                *error = "the assembly of " +
                         revision.moverPath.GetString() + " reads array " +
                         key.path.GetString() +
                         " at a site no input of the file stands for";
                return false;
            }
            switch (Role(role)) {
            case Role::JointIndices:
            case Role::JointWeights:
                // A layout the epoch does not fix is read per frame from
                // the bake's arrays.
                if (!fixed) {
                    continue;
                }
                break;
            case Role::CurveLive:
                if (transformDrivers > 0) {
                    continue;
                }
                break;
            case Role::DriverWeights:
            case Role::DriverBaseWeights:
                if (transformDrivers == 0) {
                    continue;
                }
                break;
            default:
                break;
            }
            if (tag == fb::InputTag::Vec3fArray &&
                key.flavour == RigExecRevisionLeafFlavour::OverlayThenRaw &&
                phased.count(key.path)) {
                continue;
            }
            RigExecBakeArrayRead read;
            if (!Hops(key, tag, &read)) {
                continue;
            }
            read.consumer = Consumer::Row;
            read.chain = chain;
            read.revision = index;
            read.derived = derived;
            reads->push_back(std::move(read));
        }
        for (size_t ch = 0; ch < revision.blendChannels.size(); ++ch) {
            const auto &samples = revision.blendChannels[ch].samples;
            for (size_t s = 0; s < samples.size(); ++s) {
                const int k = samples[s].pointsLeaf;
                if (k < 0 || size_t(k) >= decl.keys.size()) {
                    continue;
                }
                RigExecBakeArrayRead read;
                if (!Hops(decl.keys[size_t(k)], fb::InputTag::Vec3fArray,
                          &read)) {
                    continue;
                }
                read.consumer = Consumer::BlendPoints;
                read.chain = chain;
                read.revision = index;
                read.derived = derived;
                read.channel = uint32_t(ch);
                read.sample = uint32_t(s);
                reads->push_back(std::move(read));
            }
        }
        if (fixed && revision.op == RigExecRevisionOp::Skin) {
            const RigExecRevisionLeafDecl &layout =
                revision.layoutLeaves.decl;
            const int indices = layout.Role(Role::JointIndices);
            const int weights = layout.Role(Role::JointWeights);
            RigExecBakeArrayRead first, second;
            // Both arrays or neither: the layout is rebuilt from the pair.
            if (indices >= 0 && weights >= 0 &&
                Hops(layout.keys[size_t(indices)], fb::InputTag::IntArray,
                     &first) &&
                Hops(layout.keys[size_t(weights)], fb::InputTag::FloatArray,
                     &second) &&
                first.hops.size() == 1 && second.hops.size() == 1) {
                for (RigExecBakeArrayRead *read : {&first, &second}) {
                    read->consumer = Consumer::Layout;
                    read->chain = chain;
                    read->revision = index;
                    read->indices = read == &first;
                    reads->push_back(std::move(*read));
                }
            }
        }
        return true;
    }

    // A weight object's point gathers, one row each.
    void Gathers(const RigExecBakedProgramImpl::WeightObject &object,
                 uint32_t index) const
    {
        for (const RigExecRevisionLeafKey &key : object.pointLeaves.decl.keys) {
            RigExecBakeArrayRead read;
            if (!Hops(key, fb::InputTag::Vec3fArray, &read)) {
                continue;
            }
            read.consumer = Consumer::Row;
            read.gather = true;
            read.object = index;
            reads->push_back(std::move(read));
        }
    }

    // _ReadTargetPoints' attribute: the relationship's one target, as
    // _ResolveGeometryInput resolves it. A typed missing value still needs
    // storage so later successful stage samples can recover.
    SdfPath OraclePoints(const UsdPrim &prim, const char *relationship) const
    {
        SdfPathVector targets;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken(relationship))) {
            rel.GetTargets(&targets);
        }
        if (targets.size() != 1) {
            return SdfPath();
        }
        const SdfPath canonical =
            evaluatorDetail::_ResolveGeometryInput(stage, targets[0]);
        const UsdAttribute a = stage->GetAttributeAtPath(canonical);
        return _HoldsTag(a, fb::InputTag::Vec3fArray) ? canonical : SdfPath();
    }

    // One weight object entry: its painted arrays, read at Default, and,
    // where the oracle resolves it, the points its volume samples and a
    // curve weight's curve, read raw at the time.
    void Object(const RigExecBakeArrayObject &object, uint32_t index) const
    {
        const UsdPrim prim = stage->GetPrimAtPath(object.path);
        if (!prim) {
            return;
        }
        const std::pair<const char *, fb::InputTag> painted[] = {
            {"rigExec:values", fb::InputTag::FloatArray},
            {"rigExec:indices", fb::InputTag::IntArray}};
        for (const auto &[name, tag] : painted) {
            RigExecBakeArrayRead read;
            if (!Raw(object.path.AppendProperty(TfToken(name)), tag,
                     &read)) {
                continue;
            }
            read.consumer = Consumer::Painted;
            read.rest = true;
            read.object = index;
            read.indices = tag == fb::InputTag::IntArray;
            reads->push_back(std::move(read));
        }
        const bool curve = object.type == "RigExecCurveWeight";
        const bool volume = curve || object.type == "RigExecSphereWeight" ||
                            object.type == "RigExecPlaneWeight";
        if (!object.oracle || !volume) {
            return;
        }
        const auto add = [&](const SdfPath &path, Consumer consumer) {
            RigExecBakeArrayRead read;
            if (!path.IsEmpty() &&
                Raw(path, fb::InputTag::Vec3fArray, &read)) {
                read.consumer = consumer;
                read.object = index;
                reads->push_back(std::move(read));
            }
        };
        if (!object.samplesInFlight) {
            SdfPath samples = OraclePoints(prim, "rigExec:sampleSource");
            if (samples.IsEmpty()) {
                samples = OraclePoints(prim, "rigExec:weightTarget");
            }
            add(samples, Consumer::OracleSamples);
            add(OraclePoints(prim, "rigExec:weightTarget"),
                Consumer::OracleFallback);
        }
        if (curve) {
            add(OraclePoints(prim, "rigExec:curve"), Consumer::OracleCurve);
        }
    }
};

}  // namespace

bool
RigExecBakeListArrayReads(const RigExecBakedProgramImpl &program,
                          const std::vector<RigExecBakeArrayObject> &objects,
                          double time,
                          std::vector<RigExecBakeArrayRead> *reads,
                          std::string *error)
{
    reads->clear();
    const _Lister lister{program, program.stage, UsdTimeCode(time), reads};
    for (size_t c = 0; c < program.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = program.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            if (!lister.Revision(chain.revisions[r], uint32_t(c),
                                 uint32_t(r), false, error)) {
                return false;
            }
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            if (!lister.Revision(chain.derived[d].revision, uint32_t(c),
                                 uint32_t(d), true, error)) {
                return false;
            }
        }
    }
    for (size_t o = 0; o < program.weightObjects.size(); ++o) {
        lister.Gathers(program.weightObjects[o], uint32_t(o));
    }
    // A chain whose base reads at the time: the prologue's raw read of the
    // target.
    for (size_t c = 0; c < program.chains.size(); ++c) {
        const SdfPath &target = program.chains[c].target;
        const UsdAttribute a = program.stage->GetAttributeAtPath(target);
        VtVec3fArray base;
        RigExecBakeArrayRead read;
        if (!a || !a.Get(&base, UsdTimeCode(time)) ||
            !lister.Raw(target, fb::InputTag::Vec3fArray, &read)) {
            continue;
        }
        read.consumer = Consumer::ChainBase;
        read.chain = uint32_t(c);
        reads->push_back(std::move(read));
    }
    for (size_t o = 0; o < objects.size(); ++o) {
        lister.Object(objects[o], uint32_t(o));
    }
    return true;
}

}  // namespace rigExec

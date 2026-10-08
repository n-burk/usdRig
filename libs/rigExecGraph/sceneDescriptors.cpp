#include "sceneDescriptors.h"
#include "usdSceneAccess.h"
#include "rigExec/moverGraph.h"
#include "rigExec/movers/moverRegistry.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include "pxr/usd/usdGeom/xformable.h"
#include <algorithm>
#include <set>

namespace rigExec {
namespace {
bool Fail(std::string *error, const std::string &message) {
    if (error) *error = message;
    return false;
}
RigExecSceneDomain Domain(const TfToken &type) {
    const std::string name = type.GetString();
    if(const auto *handler=RigExecFindMoverHandler(type))
        if(handler->resolveOp(TfToken())==RigExecRevisionOp::External && handler->domain==RigExecMoverDomain::Points)
            return RigExecSceneDomain::GeometryMover;
    if (name == "RigExecControl" || name == "RigExecJoint") return RigExecSceneDomain::Provider;
    if (name == "RigExecFkChain" || name == "RigExecTwoBoneIk" || name == "RigExecBlendPointFrames" ||
        name == "RigExecTwistDistribution" || name == "RigExecRibbon" || name == "RigExecSplineIk") return RigExecSceneDomain::Solver;
    if (name == "RigExecFloatMathMover" || name == "RigExecVec3fMathMover" || name == "RigExecMatrixMathMover") return RigExecSceneDomain::PropertyMover;
    if (name == "RigExecSpaceSwitch") return RigExecSceneDomain::SpaceSwitch;
    if (name == "RigExecPoseInterpolator") return RigExecSceneDomain::PoseInterpolator;
    if (name == "RigExecSurfaceProjector") return RigExecSceneDomain::GeometryMover;
    if(name.rfind("RigExec",0)==0 && UsdSchemaRegistry::GetTypeFromSchemaTypeName(type).IsUnknown()) {
        for(const auto &handler:RigExecMoverHandlers())for(const auto &schema:handler.sceneDataSchemas)
            if(name==schema)return RigExecSceneDomain::Data;
        return RigExecSceneDomain::UnknownRigExec;
    }
    if (name.size() >= 10 && name.compare(name.size()-10, 10, "Constraint") == 0) return RigExecSceneDomain::Constraint;
    if (name.size() >= 6 && name.compare(name.size()-6, 6, "Weight") == 0) return RigExecSceneDomain::Weight;
    if (name.size() >= 5 && name.compare(name.size()-5, 5, "Mover") == 0) return RigExecSceneDomain::GeometryMover;
    if (name.rfind("RigExec", 0) != 0 || name == "RigExecRoot" || name == "RigExecBlendInput" ||
        name == "RigExecBlendSample" || name == "RigExecPose" || name.rfind("RigExecPicker",0) == 0 ||
        name.rfind("RigExecTouch",0) == 0) return RigExecSceneDomain::Data;
    for(const auto &handler:RigExecMoverHandlers())for(const auto &schema:handler.sceneDataSchemas)
        if(name==schema)return RigExecSceneDomain::Data;
    return RigExecSceneDomain::UnknownRigExec;
}
bool Phase(const VtDictionary &metadata, TfToken *phase, std::string *error, const SdfPath &path) {
    *phase = TfToken("base");
    const auto found = metadata.find("rigExecReadPhase");
    if (found == metadata.end() || found->second.IsEmpty()) return true;
    if (found->second.IsHolding<TfToken>()) *phase = found->second.UncheckedGet<TfToken>();
    else if (found->second.IsHolding<std::string>()) *phase = TfToken(found->second.UncheckedGet<std::string>());
    else return Fail(error,"invalid rigExecReadPhase type: " + path.GetString());
    RigExecReadPhase parsed;
    std::string why;
    if(!RigExecParseReadPhase(phase->GetString(),&parsed,&why))
        return Fail(error,"invalid rigExecReadPhase: "+path.GetString()+": "+why);
    if(phase->IsEmpty())*phase=TfToken("base");
    return true;
}
bool Expression(const RigExecSceneDescriptors &scene, const SdfPath &path) {
    const auto node = scene.nodes.find(path.GetPrimPath());
    if (node == scene.nodes.end()) return false;
    const auto type=node->second.fact.type;
    if (node->second.domain != RigExecSceneDomain::Provider && type!="RigExecSphereWeight" &&
        type!="RigExecPlaneWeight" && type!="RigExecCurveWeight") return false;
    const auto name = path.GetNameToken();
    return name == "default:space" || name == "avars:defaultSpace" || name == "posed:defaultSpace" ||
        name == "parent:space" || name == "parent:defaultSpace";
}
bool Forward(const RigExecSceneDescriptors &scene, const SdfPath &path,
    std::set<SdfPath> *visiting, SdfPathVector *targets, std::string *error) {
    const auto relation = scene.relationships.find(path);
    if (relation == scene.relationships.end()) { targets->push_back(path); return true; }
    if (!visiting->insert(path).second) return Fail(error,"relationship forwarding cycle: " + path.GetString());
    for (const auto &target : relation->second.fact.targets)
        if (!Forward(scene,target,visiting,targets,error)) return false;
    visiting->erase(path);
    return true;
}
RigExecSceneInput Input(const RigExecSceneDescriptors &scene, const SdfPath &path, size_t identity) {
    RigExecSceneInput result;
    const auto first = scene.attributes.find(path);
    if (first == scene.attributes.end()) return result;
    result.raw = first->second.inputs[identity].raw;
    result.rawBlocked = first->second.inputs[identity].rawBlocked;
    SdfPath current = path;
    std::set<SdfPath> visited;
    while (visited.insert(current).second) {
        result.hops.push_back(current);
        result.source = current;
        const auto attr = scene.attributes.find(current);
        const auto prim = scene.nodes.find(current.GetPrimPath());
        if (attr == scene.attributes.end() || prim == scene.nodes.end() || !prim->second.fact.active) return result;
        if (Expression(scene,current)) { result.state = RigExecSceneInputState::Computed; return result; }
        const auto &connections = attr->second.fact.connections;
        if (connections.size() == 1) {
            const auto input = scene.attributes.find(connections.front());
            const auto provider = scene.nodes.find(connections.front().GetPrimPath());
            if (input != scene.attributes.end() && provider != scene.nodes.end() && provider->second.fact.active &&
                input->second.fact.type.GetType() == attr->second.fact.type.GetType()) {
                current = connections.front();
                continue;
            }
        }
        if (connections.size() > 1 && attr->second.fact.type.IsArray() &&
            prim->second.fact.type.GetString().rfind("RigExec",0) == 0) {
            result.state = RigExecSceneInputState::Computed;
            return result;
        }
        result.resolved = attr->second.inputs[identity].raw;
        result.resolvedBlocked = attr->second.inputs[identity].rawBlocked;
        result.state = result.resolved.IsEmpty() ? RigExecSceneInputState::Empty :
            current == path ? RigExecSceneInputState::Raw : RigExecSceneInputState::Connected;
        return result;
    }
    result.state = RigExecSceneInputState::Cycle;
    return result;
}
}
bool RigExecCaptureSceneDescriptors(const RigExecSceneAccess &source,
    const SdfPath &root, const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *output, std::string *error) {
    if (!output || identities.empty()) return Fail(error,"scene capture needs an output and exact identities");
    RigExecSceneDescriptors scene;
    const auto *usd=dynamic_cast<const RigExecUsdSceneAccess *>(&source);
    scene.rigRoot = root;
    scene.identities = identities;
    scene.timeCodesPerSecond = source.TimeCodesPerSecond();
    scene.framesPerSecond = source.FramesPerSecond();
    scene.interpolation = source.Interpolation();
    scene.upAxis = source.UpAxis();
    for (const auto time : identities)
        if (!source.HasIdentity(time)) return Fail(error,"scene capture identity unavailable");
    for (const auto &path : source.Prims(SdfPath::AbsoluteRootPath())) {
        RigExecSceneNodeDescriptor node;
        if (!source.Prim(path,&node.fact)) return Fail(error,"scene prim unavailable: " + path.GetString());
        node.domain = Domain(node.fact.type);
        node.attributes = source.Attributes(path);
        node.relationships = source.Relationships(path);
        const auto schemaType=UsdSchemaRegistry::GetTypeFromSchemaTypeName(node.fact.type);
        node.transformProvider=node.domain==RigExecSceneDomain::Provider || schemaType.IsA<UsdGeomXformable>();
        node.pointBased=schemaType.IsA<UsdGeomPointBased>();
        for (const auto &property : node.attributes) {
            RigExecSceneAttributeDescriptor attr;
            const auto attribute=usd ? usd->BindAttribute(property) : UsdAttribute();
            if (!(usd ? usd->Attribute(attribute,&attr.fact) : source.Attribute(property,&attr.fact)) ||
                !Phase(attr.fact.metadata,&attr.readPhase,error,property)) return false;
            for (const auto time : identities) {
                RigExecSceneInput input;
                if (node.fact.active && !(usd ? usd->Resolve(attribute,time,&input.raw) : source.Resolve(property,time,&input.raw)))
                    return Fail(error,"scene attribute identity unavailable: " + property.GetString());
                input.rawBlocked = usd ? usd->ValueBlocked(attribute,time) : source.ValueBlocked(property,time);
                attr.inputs.push_back(std::move(input));
            }
            scene.attributes.emplace(property,std::move(attr));
        }
        for (const auto &property : node.relationships) {
            RigExecSceneRelationshipDescriptor rel;
            if (!source.Relationship(property,&rel.fact) || !Phase(rel.fact.metadata,&rel.readPhase,error,property)) return false;
            scene.relationships.emplace(property,std::move(rel));
        }
        scene.nodes.emplace(path,std::move(node));
    }
    const auto rig = scene.nodes.find(root);
    if (rig == scene.nodes.end() || !rig->second.fact.active ||
        (root!=SdfPath::AbsoluteRootPath() && rig->second.fact.type != "RigExecRoot"))
        return Fail(error,"scene capture needs an active RigExecRoot");
    // Preserve map-order child diagnostics without rescanning the whole
    // detached node inventory for every visited parent.
    std::map<SdfPath,std::vector<SdfPath>> actualChildren;
    for (const auto &[child, unused] : scene.nodes)
        if (child != SdfPath::AbsoluteRootPath())
            actualChildren[child.GetParentPath()].push_back(child);
    int ordinal = 0;
    std::function<bool(const SdfPath &)> walk = [&](const SdfPath &path) {
        auto &node = scene.nodes.at(path);
        std::set<SdfPath> children;
        for (const auto &child : node.fact.children)
            if (!scene.nodes.count(child) || child.GetParentPath() != path || !children.insert(child).second)
                return Fail(error,"invalid composed child order: " + path.GetString());
        const auto actual = actualChildren.find(path);
        if (actual != actualChildren.end())
            for (const auto &child : actual->second)
                if (!children.count(child))
                    return Fail(error,"missing composed child order: " + path.GetString());
        for (auto it = node.fact.children.rbegin(); it != node.fact.children.rend(); ++it)
            if (!walk(*it)) return false;
        node.stackOrdinal = ordinal++;
        return true;
    };
    if (!walk(root)) return false;
    for (auto &[path, relation] : scene.relationships) {
        std::set<SdfPath> visiting;
        if (!Forward(scene,path,&visiting,&relation.forwardedTargets,error)) return false;
        for (const auto &target : relation.forwardedTargets) {
            const auto node = scene.nodes.find(target.GetPrimPath());
            relation.targetExists.push_back(node != scene.nodes.end() && node->second.fact.active &&
                (target.IsPrimPath() || scene.attributes.count(target) || scene.relationships.count(target)));
        }
    }
    for (auto &[path, attr] : scene.attributes)
        for (size_t i = 0; i < identities.size(); ++i) attr.inputs[i] = Input(scene,path,i);
    for (const auto &[path, node] : scene.nodes) {
        if (node.stackOrdinal < 0 || !node.fact.active) continue;
        const auto moves = scene.relationships.find(path.AppendProperty(TfToken("rigExec:moves")));
        if (moves != scene.relationships.end()) for (size_t targetIndex=0;targetIndex<moves->second.forwardedTargets.size();++targetIndex) {
            const auto &target=moves->second.forwardedTargets[targetIndex];
            SdfPath canonical = target;
            if (target.IsPrimPath() && scene.attributes.count(target.AppendProperty(TfToken("points"))))
                canonical = target.AppendProperty(TfToken("points"));
            scene.applications.push_back({path,target,node.domain,node.stackOrdinal,canonical,targetIndex});
        }
        if (node.domain == RigExecSceneDomain::SpaceSwitch) {
            const auto target = scene.relationships.find(path.AppendProperty(TfToken("rigExec:target")));
            if (target != scene.relationships.end()) for (size_t targetIndex=0;targetIndex<target->second.forwardedTargets.size();++targetIndex) {
                const auto &provider=target->second.forwardedTargets[targetIndex];
                scene.applications.push_back({path,provider,node.domain,node.stackOrdinal,provider,targetIndex});
            }
        }
        const auto joints = scene.relationships.find(path.AppendProperty(TfToken("rigExec:joints")));
        if (node.domain == RigExecSceneDomain::Solver && joints != scene.relationships.end()) {
            VtIntArray elements;
            const auto elementPath=path.AppendProperty(TfToken("rigExec:jointElements"));
            const auto elementFact=scene.attributes.find(elementPath);
            if(elementFact!=scene.attributes.end()) {
                if(!elementFact->second.fact.sampleTimes.empty())
                    return Fail(error,"solver jointElements must not carry time samples: "+path.GetString());
                VtValue value;
                if(!source.Resolve(elementPath,UsdTimeCode::Default(),&value))
                    return Fail(error,"solver output binding requires captured Default identity: "+path.GetString());
                if(value.IsHolding<VtIntArray>())elements=value.UncheckedGet<VtIntArray>();
            }
            if(!elements.empty() && elements.size()!=joints->second.forwardedTargets.size())
                return Fail(error,"solver jointElements/targets cardinality mismatch: "+path.GetString());
            for (size_t i = 0; i < joints->second.forwardedTargets.size(); ++i) {
                const int element=elements.empty()?int(i):elements[i];
                if(element<0)return Fail(error,"negative solver output element: "+path.GetString());
                scene.jointBindings.push_back({path,joints->second.forwardedTargets[i],size_t(element)});
            }
        }
    }
    std::sort(scene.applications.begin(),scene.applications.end(),[](const auto &a,const auto &b) {
        return a.stackOrdinal != b.stackOrdinal ? a.stackOrdinal < b.stackOrdinal :
            a.targetIndex!=b.targetIndex?a.targetIndex<b.targetIndex:a.target<b.target;
    });
    *output = std::move(scene);
    return true;
}
bool RigExecSelectSceneDescriptorIdentities(const RigExecSceneDescriptors &source,
    const std::vector<UsdTimeCode> &identities,
    RigExecSceneDescriptors *output, std::string *error) {
    if (!output || identities.empty())
        return Fail(error,"scene identity selection needs an output and exact identities");
    std::vector<size_t> selected;
    for (const UsdTimeCode time : identities) {
        const auto found=std::find(source.identities.begin(),source.identities.end(),time);
        if (found==source.identities.end())
            return Fail(error,"scene capture identity unavailable");
        selected.push_back(size_t(found-source.identities.begin()));
    }
    for (const auto &[path,attr] : source.attributes)
        if (attr.inputs.size()!=source.identities.size())
            return Fail(error,"incomplete captured scene inputs: "+path.GetString());
    RigExecSceneDescriptors result=source;
    result.identities=identities;
    for (auto &[path,attr] : result.attributes) {
        const auto &inputs=source.attributes.at(path).inputs;
        attr.inputs.clear();attr.inputs.reserve(selected.size());
        for (size_t identity : selected) attr.inputs.push_back(inputs[identity]);
    }
    *output=std::move(result);
    return true;
}
bool RigExecCompileSceneDescriptors(const RigExecSceneDescriptors &scene,
    const RigExecSceneKernelLowering &lower, RigExecCyclePolicy policy,
    RigExecCompiledGraph *graph, std::string *error) {
    if (!lower) return Fail(error,"scene compiler needs production typed kernel lowering");
    std::vector<RigExecOpDescriptor> ops;
    std::vector<RigExecValueId> leaves;
    if (!lower(scene,&ops,&leaves,error)) return false;
    return RigExecCompileOpGraph(ops,leaves,policy,graph,error);
}
}

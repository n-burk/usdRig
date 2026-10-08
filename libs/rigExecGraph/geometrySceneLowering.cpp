#include "geometrySceneLowering.h"
#include "providerProgram.h"
#include "rigExec/movers/moverRegistry.h"
#include "pxr/usd/usdGeom/xformOp.h"
#include <algorithm>
#include <cstring>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message) {if(error)*error=message;return false;}
}
bool RigExecLowerSceneGeometry(const RigExecSceneDescriptors &scene,const SdfPath &mover,
    const SdfPath &target,RigExecSceneGeometryDescriptor *out,std::string *error,
    const RigExecGeometryRecord *normalized)
{
    if(!out)return Fail(error,"null geometry descriptor");
    RigExecSceneCompileInputs source(scene);
    const auto *node=source.Node(mover);
    const auto *targetAttribute=source.Attribute(target);
    if(!node)return Fail(error,"missing geometry mover: "+mover.GetString());
    RigExecSceneGeometryDescriptor result;
    result.mover=mover;result.target=target;
    result.pointsTarget=target;
    result.targetExists=targetAttribute!=nullptr;
    if(targetAttribute)result.targetType=targetAttribute->fact.type;
    auto targets=[&](const SdfPath &owner,const char *name) {
        return source.Targets(owner.AppendProperty(TfToken(name)));
    };
    auto first=[&](const char *name) {auto t=targets(mover,name);return t.empty()?SdfPath():t.front();};
    auto points=[&](SdfPath path) {return path.IsPrimPath()?path.AppendProperty(TfToken("points")):path;};
    auto phase=[&](const SdfPath &owner,const char *name,RigExecReadPhase *value) {
        const auto *rel=source.Relationship(owner.AppendProperty(TfToken(name)));
        return !rel || RigExecParseReadPhase(rel->readPhase.GetString(),value,error);
    };
    if(normalized)result.record=*normalized;
    else {
        TfToken mode;
        const auto *attribute=source.Attribute(mover.AppendProperty(TfToken("rigExec:mode")));
        if(attribute && !attribute->inputs.empty() && attribute->inputs.front().raw.IsHolding<TfToken>())
            mode=attribute->inputs.front().raw.UncheckedGet<TfToken>();
        auto op=RigExecRevisionOpForSchema(node->fact.type,mode);
        if(!op)return Fail(error,"unsupported geometry schema: "+node->fact.type.GetString());
        result.record.op=*op;
    }
    auto &binding=result.record.binding;
    binding.moverPath=mover;binding.target=target;
    if(!normalized) {
        binding.weightObject=first("rigExec:weightObject");
        const auto owner=target.GetPrimPath();
        auto meshTopology=[&](const SdfPath &mesh) {
            binding.topologyCounts=mesh.AppendProperty(TfToken("faceVertexCounts"));
            binding.topologyIndices=mesh.AppendProperty(TfToken("faceVertexIndices"));
        };
        switch(result.record.op) {
        case RigExecRevisionOp::Matrix:
            binding.transform=first("rigExec:transform");
            binding.transformSpace=first("rigExec:transformSpace");binding.carrySpace=first("rigExec:space");
            if(!phase(mover,"rigExec:transform",&binding.transformPhase))return false;
            for(const char *name:{"rigExec:referenceTransform","rigExec:referenceTransformSpace"}) {
                const auto t=targets(mover,name);if(!t.empty())binding.influences.push_back(t.front());
            } break;
        case RigExecRevisionOp::Skin:
            binding.influences=targets(mover,"rigExec:influences");
            if(!phase(mover,"rigExec:influences",&binding.transformPhase))return false;
            break;
        case RigExecRevisionOp::BlendShape:
            binding.base=target;
            if(const auto *mesh=source.Node(owner))if(mesh->fact.type=="Mesh")meshTopology(owner);
            binding.blendInputs=targets(mover,"rigExec:blendInputs");
            std::sort(binding.blendInputs.begin(),binding.blendInputs.end());
            for(const auto &input:binding.blendInputs)for(const auto &sample:targets(input,"rigExec:samples")) {
                const auto p=targets(sample,"rigExec:targetPoints"),shape=targets(sample,"rigExec:blendShape");
                if(p.size()+shape.size()!=1)return Fail(error,"blend sample must name one shape: "+sample.GetString());
                RigExecReadPhase readPhase;
                if(!p.empty() && !phase(sample,"rigExec:targetPoints",&readPhase))return false;
                binding.blendSamples[input].push_back({sample,p.empty()?SdfPath():points(p.front()),readPhase,
                                                       shape.empty()?SdfPath():shape.front()});
            } break;
        case RigExecRevisionOp::VolumeCorrect:binding.base=target;break;
        case RigExecRevisionOp::Smooth:case RigExecRevisionOp::DeltaMush:case RigExecRevisionOp::Wrinkle:
        case RigExecRevisionOp::RecomputeNormals:
            binding.base=target;meshTopology(owner);break;
        case RigExecRevisionOp::Lattice: {
            binding.base=target;binding.cagePoints=points(first("rigExec:cage"));
            RigExecReadPhase p;if(!phase(mover,"rigExec:cage",&p))return false;
            if(!p.IsBase())binding.phases[binding.cagePoints]=p;break;
        }
        case RigExecRevisionOp::SurfaceProject: {
            const auto surface=first("rigExec:surface").GetPrimPath();
            if(!surface.IsEmpty()) {binding.surfacePoints=points(surface);meshTopology(surface);}
            RigExecReadPhase p;if(!phase(mover,"rigExec:surface",&p))return false;
            if(!p.IsBase())binding.phases[binding.surfacePoints]=p;break;
        }
        case RigExecRevisionOp::Ribbon:case RigExecRevisionOp::Wire:case RigExecRevisionOp::EmitGuidePoints: {
            binding.bindCoords=first("rigExec:bindCoordinates");binding.driverFrames=first("rigExec:driverFrames");
            const auto curve=first("rigExec:driverCurve").GetPrimPath();
            if(!curve.IsEmpty()) {
                binding.driverCurvePoints=points(curve);binding.driverCurveOrder=curve.AppendProperty(TfToken("order"));
                binding.driverCurveKnots=curve.AppendProperty(TfToken("knots"));
                RigExecReadPhase p;if(!phase(mover,"rigExec:driverCurve",&p))return false;
                if(!p.IsBase())binding.phases[binding.driverCurvePoints]=p;
            }
            const auto transforms=targets(mover,"rigExec:driverTransforms");
            if(!transforms.empty()) {
                if(!phase(mover,"rigExec:driverTransforms",&binding.transformPhase))return false;
                binding.influences=transforms;binding.driverTransformCount=int(transforms.size());
                for(const char *name:{"rigExec:driverTransformSpaces","rigExec:driverBaseTransforms","rigExec:driverBaseTransformSpaces"}) {
                    auto list=targets(mover,name);
                    if(std::string(name)=="rigExec:driverTransformSpaces")binding.driverSpaceCount=int(list.size());
                    if(std::string(name)=="rigExec:driverBaseTransforms")binding.driverBaseTransformCount=int(list.size());
                    binding.influences.insert(binding.influences.end(),list.begin(),list.end());
                }
                binding.carrySpace=first("rigExec:space");
            }
            RigExecReadPhase p;if(!phase(mover,"rigExec:bindCoordinates",&p))return false;
            if(!p.IsBase())binding.phases[binding.bindCoords]=p;
            break;
        }
        case RigExecRevisionOp::RecomputeExtent:
            binding.widths=owner.AppendProperty(TfToken("widths"));break;
        case RigExecRevisionOp::SurfaceProjector:case RigExecRevisionOp::ShaderDials:
            return Fail(error,"projector requires normalized static mesh/frame binding");
        case RigExecRevisionOp::External: {
            const auto *handler=RigExecFindMoverHandler(node->fact.type);
            if(!handler || !handler->compileScene)return Fail(error,"external geometry has no detached compiler");
            binding.handler=handler;binding.externalSchema=node->fact.type;
            if(!handler->compileScene(scene,mover,target,&binding,error))return false;
            break;
        }
        }
    }
    RigExecDeclareRevisionLeaves(result.record.op,mover,binding,&result.record.leaves);
    if(result.record.op==RigExecRevisionOp::Skin) {
        RigExecRevisionLeafDecl layout;RigExecDeclareSkinLayoutLeaves(mover,&layout);
        const int first=int(result.record.leaves.keys.size());
        for(const auto &key:layout.keys)result.record.leaves.Add(key);
        for(size_t role=0;role<layout.roles.size();++role)
            if(layout.roles[role]>=0)result.record.leaves.roles[role]=first+layout.roles[role];
    }
    using Type=RigExecRevisionLeafType;using Time=RigExecRevisionLeafTime;
    using Flavour=RigExecRevisionLeafFlavour;
    for(const auto &input:binding.blendInputs) {
        RigExecSceneGeometryBlendChannel channel;
        channel.weight=result.record.leaves.Add({input.AppendProperty(TfToken("inputs:weight")),
            Type::Float,Time::AtTime,Flavour::ResolvedOnly,VtValue(0.0f)});
        const auto found=binding.blendSamples.find(input);
        if(found!=binding.blendSamples.end())for(const auto &sample:found->second) {
            RigExecSceneGeometryBlendSample item;item.path=sample.sample;
            item.activation=result.record.leaves.Add({sample.sample.AppendProperty(TfToken("rigExec:activation")),
                Type::Float,Time::AtTime,Flavour::ResolvedOnly,VtValue(1.0f)});
            item.sparse=!sample.blendShape.IsEmpty();
            if(item.sparse) {
                const auto shape=sample.blendShape.GetPrimPath();const auto *node=source.Node(shape);
                item.shapeExists=node && node->fact.type=="BlendShape";
                item.offsets=result.record.leaves.Add({shape.AppendProperty(TfToken("offsets")),
                    Type::Vec3fArray,Time::AtDefault,Flavour::Raw,VtValue(VtVec3fArray())});
                item.indices=result.record.leaves.Add({shape.AppendProperty(TfToken("pointIndices")),
                    Type::IntArray,Time::AtDefault,Flavour::Raw,VtValue(VtIntArray())});
            } else {
                item.points=result.record.leaves.Add({sample.points,Type::Vec3fArray,
                    Time::AtTime,Flavour::ResolvedOnly,VtValue(VtVec3fArray())});
                if(!sample.phase.IsBase())binding.phases[sample.points]=sample.phase;
            }
            channel.samples.push_back(std::move(item));
        }
        result.blendChannels.push_back(std::move(channel));
    }
    result.typedLeaves.resize(result.record.leaves.keys.size());
    result.inputs.resize(result.record.leaves.keys.size());result.bound.assign(result.inputs.size(),0);
    for(size_t k=0;k<result.inputs.size();++k) {
        const auto &key=result.record.leaves.keys[k];
        if(!source.Attribute(key.path))continue;
        const auto route=key.flavour==RigExecRevisionLeafFlavour::Resolved ||
            key.flavour==RigExecRevisionLeafFlavour::ResolvedOnly ? RigExecSceneReadRoute::ConnectionResolved:RigExecSceneReadRoute::Raw;
        if(!source.Bind(key.path,route,&result.inputs[k],error))return false;
        if(key.flavour!=Flavour::Raw) {
            SdfValueTypeName requested;
            switch(key.type) {
            case Type::Bool:requested=SdfValueTypeNames->Bool;break;
            case Type::Int:requested=SdfValueTypeNames->Int;break;
            case Type::Float:requested=SdfValueTypeNames->Float;break;
            case Type::Double:case Type::Dial:requested=SdfValueTypeNames->Double;break;
            case Type::Token:requested=SdfValueTypeNames->Token;break;
            case Type::IntArray:requested=SdfValueTypeNames->IntArray;break;
            case Type::FloatArray:requested=SdfValueTypeNames->FloatArray;break;
            case Type::DoubleArray:requested=SdfValueTypeNames->DoubleArray;break;
            case Type::Vec2fArray:requested=SdfValueTypeNames->Float2Array;break;
            case Type::Vec3fArray:requested=SdfValueTypeNames->Float3Array;break;
            case Type::Vec3f:requested=SdfValueTypeNames->Float3;break;
            case Type::Vec3d:requested=SdfValueTypeNames->Double3;break;
            case Type::Vec3i:requested=SdfValueTypeNames->Int3;break;
            case Type::Matrix4d:requested=SdfValueTypeNames->Matrix4d;break;
            }
            if(key.type==Type::Dial && source.Attribute(key.path)->fact.type.GetType()==SdfValueTypeNames->Float.GetType())
                requested=SdfValueTypeNames->Float;
            if(key.flavour==Flavour::Present)requested=source.Attribute(key.path)->fact.type;
            if(!RigExecBindSceneTypedRead(scene,key.path,requested,&result.typedLeaves[k],error))return false;
            const auto phase=binding.phases.find(key.path);
            if(phase!=binding.phases.end())result.typedLeaves[k].readPhase=TfToken(phase->second.GetAsString());
            if(key.flavour==Flavour::Present || key.flavour==Flavour::OverlayThenRaw) {
                result.typedLeaves[k].hops.resize(std::min(size_t(1),result.typedLeaves[k].hops.size()));
                result.typedLeaves[k].doubleHops.clear();
            }
        }
        result.bound[k]=1;
    }
    *out=std::move(result);return true;
}
bool RigExecLowerSceneBlendChannel(const RigExecSceneDescriptors &scene,const SdfPath &channel,
    RigExecSceneGeometryDescriptor *out,std::string *error)
{
    RigExecSceneCompileInputs source(scene);const auto *node=source.Node(channel);
    if(!node || node->fact.type!="RigExecBlendInput")return Fail(error,"missing blend channel: "+channel.GetString());
    RigExecGeometryRecord record;record.op=RigExecRevisionOp::BlendShape;
    record.binding.moverPath=channel;record.binding.blendInputs={channel};
    std::vector<size_t> groups;
    for(const auto &sample:source.Targets(channel.AppendProperty(TfToken("rigExec:samples")))) {
        const auto points=source.Targets(sample.AppendProperty(TfToken("rigExec:targetPoints")));
        RigExecReadPhase phase;
        const auto *relationship=source.Relationship(sample.AppendProperty(TfToken("rigExec:targetPoints")));
        if(relationship && !RigExecParseReadPhase(relationship->readPhase.GetString(),&phase,error))return false;
        groups.push_back(std::max(size_t(1),points.size()));
        if(points.empty())record.binding.blendSamples[channel].push_back({sample,{},phase,{}});
        else for(auto path:points){if(path.IsPrimPath())path=path.AppendProperty(TfToken("points"));record.binding.blendSamples[channel].push_back({sample,path,phase,{}});}
    }
    if(!RigExecLowerSceneGeometry(scene,channel,channel.AppendProperty(TfToken("computeBlendChannel")),out,error,&record))return false;
    if(out->blendChannels.size()!=1)return Fail(error,"invalid public channel descriptor");
    auto &samples=out->blendChannels.front().samples;std::vector<RigExecSceneGeometryBlendSample> grouped;
    size_t first=0;for(size_t count:groups){auto sample=std::move(samples[first]);for(size_t n=1;n<count;++n)sample.additionalPoints.push_back(samples[first+n].points);grouped.push_back(std::move(sample));first+=count;}
    samples=std::move(grouped);return true;
}
bool RigExecGatherSceneGeometryBlendChannels(const RigExecSceneGeometryDescriptor &descriptor,
    const std::vector<VtValue> &values,size_t baseCount,std::vector<RigExecBlendChannel> *out,
    std::string *error,bool publicChannelOrder)
{
    if(!out)return Fail(error,"null geometry blend channels");
    auto value=[&](int index)->const VtValue* {
        return index>=0 && size_t(index)<values.size()?&values[size_t(index)]:nullptr;
    };
    out->resize(descriptor.blendChannels.size());
    for(size_t c=0;c<out->size();++c) {
        const auto &record=descriptor.blendChannels[c];auto &channel=(*out)[c];
        const auto *weight=value(record.weight);
        channel.weight=weight && weight->IsHolding<float>()?weight->UncheckedGet<float>():0.0f;
        channel.samples.resize(record.samples.size());
        for(size_t s=0;s<channel.samples.size();++s) {
            const auto &recordSample=record.samples[s];auto &sample=channel.samples[s];
            const auto *activation=value(recordSample.activation);
            sample.activation=activation && activation->IsHolding<float>()?activation->UncheckedGet<float>():1.0f;
            if(recordSample.sparse) {
                sample.points.clear();
                const auto *offsets=value(recordSample.offsets),*indices=value(recordSample.indices);
                const VtVec3fArray emptyOffsets;const VtIntArray emptyIndices;
                const auto &o=offsets && offsets->IsHolding<VtVec3fArray>()?offsets->UncheckedGet<VtVec3fArray>():emptyOffsets;
                const auto &i=indices && indices->IsHolding<VtIntArray>()?indices->UncheckedGet<VtIntArray>():emptyIndices;
                bool same=recordSample.shapeExists && sample.layout && sample.layout->valid &&
                    sample.layout->pointCount==baseCount && sample.layout->indices.size()==i.size() &&
                    std::equal(i.begin(),i.end(),sample.layout->indices.begin()) && sample.layout->offsets.size()==o.size();
                if(same)for(size_t n=0;n<o.size() && same;++n)for(int axis=0;axis<3;++axis) {
                    const float a=sample.layout->offsets[n][axis],b=o[n][axis];
                    if(std::memcmp(&a,&b,sizeof(float))){same=false;break;}
                }
                if(same)continue;
                auto layout=std::make_shared<RigExecBlendSampleLayout>();
                if(recordSample.shapeExists)RigExecBuildGeometryBlendLayout(o,i,baseCount,layout.get());
                else layout->pointCount=baseCount;
                sample.layout=std::move(layout);
            } else {
                sample.layout.reset();const auto *points=value(recordSample.points);
                if(points && points->IsHolding<VtVec3fArray>()) {
                    const auto &p=points->UncheckedGet<VtVec3fArray>();sample.points.assign(p.begin(),p.end());
                } else sample.points.clear();
                for(int index:recordSample.additionalPoints) {
                    const auto *additional=value(index);
                    if(additional && additional->IsHolding<VtVec3fArray>()) {
                        const auto &points=additional->UncheckedGet<VtVec3fArray>();
                        sample.points.insert(sample.points.end(),points.begin(),points.end());
                    }
                }
            }
        }
        const auto less=[](const auto &a,const auto &b){return a.activation<b.activation;};
        if(publicChannelOrder)std::sort(channel.samples.begin(),channel.samples.end(),less);
        else std::stable_sort(channel.samples.begin(),channel.samples.end(),less);
    }
    return true;
}
bool RigExecResolveSceneGeometryLeaves(const RigExecSceneDescriptors &scene,
    const RigExecSceneGeometryDescriptor &descriptor,UsdTimeCode identity,
    const std::map<SdfPath,VtValue> &delivered,std::vector<VtValue> *out,std::string *error)
{
    if(!out)return Fail(error,"null geometry sampled inputs");
    const RigExecSceneCompileInputs source(scene);
    out->resize(descriptor.record.leaves.keys.size());
    for(size_t k=0;k<out->size();++k) {
        const auto &key=descriptor.record.leaves.keys[k];
        const auto found=delivered.find(key.path);
        if(key.flavour==RigExecRevisionLeafFlavour::Present) {
            (*out)[k]=VtValue(found!=delivered.end());continue;
        }
        const bool overlay=key.flavour!=RigExecRevisionLeafFlavour::Raw;
        if(overlay && found!=delivered.end()) {(*out)[k]=found->second;continue;}
        if(k>=descriptor.bound.size() || !descriptor.bound[k]) {(*out)[k]=key.fallback;continue;}
        VtValue sampled;
        const auto time=key.time==RigExecRevisionLeafTime::AtDefault?UsdTimeCode::Default():identity;
        if(!source.Read(descriptor.inputs[k],time,&sampled,nullptr,error))return false;
        (*out)[k]=sampled.IsEmpty()?key.fallback:sampled;
    }
    return true;
}
namespace {
bool StaticMeshWorld(const RigExecSceneDescriptors &scene,const SdfPath &mesh,GfMatrix4d *world,std::string *error) {
    RigExecSceneCompileInputs source(scene);
    std::vector<GfMatrix4d> locals;
    for(SdfPath owner=mesh;owner!=SdfPath::AbsoluteRootPath() && !owner.IsEmpty();owner=owner.GetParentPath()) {
        const auto *node=source.Node(owner);
        if(!node)return Fail(error,"uncaptured projector surface ancestry: "+owner.GetString());
        RigExecProviderProgram program;RigExecTypedValueStore values(1);
        program.valueKeys.push_back(owner.GetString() + "|localXform");
        RigExecProviderOp op;op.kind=RigExecProviderOpKind::LocalXform;op.owner=owner;op.output=0;
        auto raw=[&](const SdfPath &path,RigExecValueId *id) {
            *id=RigExecNoProviderValue;const auto *attribute=source.Attribute(path);
            if(!attribute)return true;
            if(attribute->fact.mightBeTimeVarying)return Fail(error,"projector requires static surface transform: "+path.GetString());
            RigExecSceneBoundInput input;VtValue value;
            if(!source.Bind(path,RigExecSceneReadRoute::Raw,&input,error) ||
               !source.Read(input,UsdTimeCode::Default(),&value,nullptr,error))return false;
            *id=values.values.size();
            program.valueKeys.push_back(RigExecProviderRawKey(path));
            values.values.emplace_back();values.PublishSource(*id,value);return true;
        };
        RigExecValueId order;
        if(!raw(owner.AppendProperty(TfToken("xformOpOrder")),&order))return false;
        op.inputs.push_back(order);bool reset=false;
        VtValue orderValue;
        if(order!=RigExecNoProviderValue)orderValue=values.values[size_t(order)].raw;
        if(orderValue.IsHolding<VtTokenArray>())for(const auto &token:orderValue.UncheckedGet<VtTokenArray>()) {
            std::string name=token.GetString();
            if(name=="!resetXformStack!"){reset=true;continue;}
            if(name.rfind("!invert!",0)==0)name.erase(0,8);
            if(std::find_if(op.xforms.begin(),op.xforms.end(),[&](const auto &x){return x.name.GetString()==name;})!=op.xforms.end())continue;
            if(name.rfind("xformOp:",0)!=0)return Fail(error,"invalid projector xform operation: "+name);
            const auto end=name.find(':',8);
            const auto type=UsdGeomXformOp::GetOpTypeEnum(TfToken(name.substr(8,end==std::string::npos?end:end-8)));
            if(type==UsdGeomXformOp::TypeInvalid)return Fail(error,"unknown projector xform operation: "+name);
            RigExecValueId id;if(!raw(owner.AppendProperty(TfToken(name)),&id))return false;
            op.xforms.push_back({TfToken(name),id,int(type)});
        }
        program.ops.push_back(std::move(op));
        if(!RigExecRunProviderOp(program,0,&values,error))return false;
        const auto *local=values.Read<GfMatrix4d>(0);
        if(!local)return Fail(error,"projector local transform unavailable");
        locals.push_back(*local);
        // Admission checks all ancestors, even when reset stops composition.
        if(reset) {
            for(SdfPath parent=owner.GetParentPath();!parent.IsEmpty() && parent!=SdfPath::AbsoluteRootPath();parent=parent.GetParentPath()) {
                const auto *parentNode=source.Node(parent);
                if(!parentNode)return Fail(error,"uncaptured projector surface ancestry: "+parent.GetString());
                for(const auto &path:parentNode->attributes)if(path.GetName().rfind("xformOp:",0)==0 || path.GetName()=="xformOpOrder")
                    if(const auto *attribute=source.Attribute(path))if(attribute->fact.mightBeTimeVarying)
                        return Fail(error,"projector requires static surface transform: "+path.GetString());
            }
            break;
        }
    }
    *world=GfMatrix4d(1);
    for(auto i=locals.rbegin();i!=locals.rend();++i)*world=*i * *world;
    return true;
}
}
bool RigExecLowerSceneGeometryVariants(const RigExecSceneDescriptors &scene,const SdfPath &mover,
    const SdfPath &pointsTarget,std::vector<RigExecSceneGeometryDescriptor> *out,std::string *error)
{
    if(!out)return Fail(error,"null geometry variants");out->clear();
    RigExecSceneCompileInputs source(scene);const auto *node=source.Node(mover);
    if(!node)return Fail(error,"missing geometry mover: "+mover.GetString());
    if(node->fact.type!="RigExecSurfaceProjector") {
        RigExecSceneGeometryDescriptor descriptor;
        if(!RigExecLowerSceneGeometry(scene,mover,pointsTarget,&descriptor,error))return false;
        out->push_back(std::move(descriptor));return true;
    }
    const auto mesh=pointsTarget.GetPrimPath();const auto *surface=source.Node(mesh);
    if(!surface || surface->fact.type!="Mesh")return Fail(error,"surface projector must move one mesh's points");
    RigExecGeometryRecord record;record.binding.moverPath=mover;record.binding.base=pointsTarget;
    record.binding.topologyCounts=mesh.AppendProperty(TfToken("faceVertexCounts"));
    record.binding.topologyIndices=mesh.AppendProperty(TfToken("faceVertexIndices"));
    GfMatrix4d world;if(!StaticMeshWorld(scene,mesh,&world,error))return false;
    record.binding.meshWorldInverse=world.GetInverse();
    auto provider=[&](const char *name,SdfPath *value) {
        const auto paths=source.Targets(mover.AppendProperty(TfToken(name)));
        if(paths.size()>1)return Fail(error,"projector has more than one "+std::string(name));
        if(paths.empty())return true;
        const auto *node=source.Node(paths[0].GetPrimPath());
        if(!node || node->domain!=RigExecSceneDomain::Provider)return Fail(error,"projector target is not a frame provider: "+paths[0].GetString());
        *value=paths[0].GetPrimPath();return true;
    };
    if(!provider("rigExec:sources",&record.binding.transform) ||
       !provider("rigExec:sourceSpace",&record.binding.transformSpace) ||
       !provider("rigExec:space",&record.binding.carrySpace))return false;
    auto name=[&](const char *attribute,const TfToken &fallback,TfToken *value) {
        *value=fallback;const auto path=mover.AppendProperty(TfToken(attribute));
        if(!source.Attribute(path))return true;
        RigExecSceneBoundInput input;VtValue boxed;
        if(!source.Bind(path,RigExecSceneReadRoute::Raw,&input,error) ||
           !source.Read(input,UsdTimeCode::Default(),&boxed,nullptr,error))return false;
        if(boxed.IsHolding<TfToken>())*value=boxed.UncheckedGet<TfToken>();return true;
    };
    for(bool dials:{false,true}) {
        TfToken primvar;
        if(!name(dials?"rigExec:shaderDialPrimvar":"rigExec:shaderPrimvar",dials?TfToken():TfToken("eyeProjector"),&primvar))return false;
        if(primvar.IsEmpty())continue;
        auto variant=record;variant.op=dials?RigExecRevisionOp::ShaderDials:RigExecRevisionOp::SurfaceProjector;
        if(dials) {
            variant.binding.transform=SdfPath();variant.binding.transformSpace=SdfPath();variant.binding.carrySpace=SdfPath();
            variant.binding.topologyCounts=SdfPath();variant.binding.topologyIndices=SdfPath();
            for(const auto &dial:source.Targets(mover.AppendProperty(TfToken("rigExec:shaderDialSources")))) {
                if(!dial.IsPropertyPath())return Fail(error,"shader dial source must be a property: "+dial.GetString());
                if(variant.binding.shaderDials.size()==16)break;
                variant.binding.shaderDials.push_back(dial);
            }
        }
        const auto target=mesh.AppendProperty(TfToken("primvars:"+primvar.GetString()));
        RigExecSceneGeometryDescriptor descriptor;
        if(!RigExecLowerSceneGeometry(scene,mover,target,&descriptor,error,&variant))return false;
        descriptor.pointsTarget=pointsTarget;descriptor.derived=true;descriptor.matrixOutput=true;
        descriptor.targetType=SdfValueTypeNames->Matrix4d;out->push_back(std::move(descriptor));
    }
    return true;
}
bool RigExecLowerSceneDerivedGeometry(const RigExecSceneDescriptors &scene,const SdfPath &pointsTarget,
    std::vector<RigExecSceneGeometryDescriptor> *out,std::string *error)
{
    if(!out)return Fail(error,"null derived geometry variants");out->clear();
    RigExecSceneCompileInputs source(scene);const auto owner=pointsTarget.GetPrimPath();const auto *node=source.Node(owner);
    if(!node)return Fail(error,"missing derived geometry owner");
    for(bool normals:{true,false}) {
        const auto target=owner.AppendProperty(TfToken(normals?"normals":"extent"));const auto *attribute=source.Attribute(target);
        if(!attribute || !attribute->fact.hasAuthoredValue)continue;
        if(normals && node->fact.type!="Mesh")return Fail(error,"authored normals on non-mesh points target");
        RigExecGeometryRecord record;record.op=normals?RigExecRevisionOp::RecomputeNormals:RigExecRevisionOp::RecomputeExtent;
        record.binding.moverPath=owner;record.binding.target=target;
        record.binding.topologyCounts=owner.AppendProperty(TfToken("faceVertexCounts"));
        record.binding.topologyIndices=owner.AppendProperty(TfToken("faceVertexIndices"));
        const auto widths=owner.AppendProperty(TfToken("widths"));
        if(!normals)if(const auto *value=source.Attribute(widths))if(value->fact.hasAuthoredValue)record.binding.widths=widths;
        RigExecSceneGeometryDescriptor descriptor;
        if(!RigExecLowerSceneGeometry(scene,owner,target,&descriptor,error,&record))return false;
        descriptor.pointsTarget=pointsTarget;descriptor.derived=true;out->push_back(std::move(descriptor));
    }
    return true;
}

}

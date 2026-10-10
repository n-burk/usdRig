#include "rigExec/scalarReferenceAdapter.h"
#include "rigExec/crossDomainInputs.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/scalarReference.h"
#include "rigExec/weightPackets.h"
#include "pxr/base/gf/math.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/tf/token.h"
#include <algorithm>
#include <unordered_map>
namespace rigExec {
namespace {
const TfToken relationNames[]={TfToken("rigExec:sampleSource"),TfToken("rigExec:weightTarget"),TfToken("rigExec:curve")};
}
RigExecWeightReferenceContext RigExecRebindWeightReference(
    RigExecWeightReferenceContext inputs,const RigExecOracleScene &reader,UsdTimeCode time) {
    for(auto &prim:inputs.prims) {
        for(auto &attribute:prim.second.attributes) {
            auto &value=attribute.second;
            if(!value.resolvedFloatSite)continue;
            float scalar=0.0f;
            const auto a=reader.GetAttributeAtPath(prim.first.AppendProperty(attribute.first));
            value.resolvedFloat=reader.GetAttribute(a,time,&scalar)?VtValue(scalar):VtValue();
        }
    }
    return inputs;
}

void RigExecOracleBeginBody(RigExecBakedProgramImpl *program,uint64_t generation,UsdTimeCode time) {
    auto &B=*program;
    if(!B.oraclePublications)return;
    std::vector<SdfPath> roots;
    std::set<SdfPath> produced,protectedPaths;
    for(const auto &chain:B.propertyChains) {
        roots.push_back(chain.target.GetPrimPath());produced.insert(chain.target);
        for(const auto &revision:chain.revisions)roots.push_back(revision.mover);
        for(uint32_t record:chain.records) {
            const auto &path=B.propertyRecords[record].consumer;
            roots.push_back(path.GetPrimPath());produced.insert(path);
            if(B.recordStoodAside[record])protectedPaths.insert(path);
        }
    }
    for(const auto &chain:B.chains) {
        roots.push_back(chain.target.GetPrimPath());
        for(const auto &revision:chain.revisions)roots.push_back(revision.moverPath);
    }
    for(const auto &weight:B.weightObjects)roots.push_back(weight.path);
    auto source=[&] {
        RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
            B.profiler && B.profiler->IsEnabled() ? "Oracle.Capture" : "",
            B.profiler && B.profiler->IsEnabled() ? "reference" : "");
        return RigExecCaptureOracleInputs(B.stage,*B.resolvedInputs,time,0,roots,B.upstream,
            RigExecOracleCaptureMode::PublicationFacts);
    }();
    B.oraclePublications->Begin(generation,std::move(source),B.propertyChains.size(),produced,protectedPaths);
    B.oracleWeightInputs.clear();
    for(const auto &weight:B.weightObjects) {
        const auto noPlacement=[](const SdfPath &) -> const GfMatrix4d * { return nullptr; };
        RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
            B.profiler && B.profiler->IsEnabled() ? "Oracle.CaptureWeight" : "",
            B.profiler && B.profiler->IsEnabled() ? "reference" : "");
        B.oracleWeightInputs.emplace(weight.path,RigExecCaptureWeightReference(
            B.stage,weight.path,*B.resolvedInputs,B.upstream,time,noPlacement));
    }
}

bool RigExecOracleRunBody(RigExecBakedProgramImpl *program,UsdTimeCode time,RigExecRigPose *pose) {
    auto &B=*program;
    RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
        B.profiler && B.profiler->IsEnabled() ? "Reference.Run" : "",
        B.profiler && B.profiler->IsEnabled() ? "reference" : "");
    if(!B.oraclePublications)return true;
    std::vector<int> allChains;
    for(size_t i=0;i<B.propertyChains.size();++i)allChains.push_back(int(i));
    auto scene=[&] {
        RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
            B.profiler && B.profiler->IsEnabled() ? "Reference.ReaderScene" : "",
            B.profiler && B.profiler->IsEnabled() ? "reference" : "");
        return B.oraclePublications->ReaderScene(allChains);
    }();
    for(size_t i=0;i<B.poseWeightPaths.size() && i<B.poseWeights.size();++i)
        scene.overlay[B.poseWeightPaths[i]]=VtValue(B.poseWeights[i]);
    std::unordered_map<SdfPath,GfMatrix4d,SdfPath::Hash> baseMatrices,finalMatrices,deltas;
    for(size_t i=0;i<B.paths.size();++i) {
        if(i<B.baseMatrix.size())baseMatrices.emplace(B.paths[i],B.baseMatrix[i]);
        if(i<B.finalMatrix.size())finalMatrices.emplace(B.paths[i],B.finalMatrix[i]);
    }
    std::map<std::pair<SdfPath,SdfPath>,VtValue> pointValues,sampleValues;
    std::map<std::pair<SdfPath,SdfPath>,RigExecOracleFrameInput> frameValues;
    for(const auto &chain:B.chains)for(const auto &revision:chain.revisions) {
        if(!revision.binding.driverFrames.IsEmpty()) {
            RigExecOracleFrameInput input;
            input.owner=revision.binding.driverFrames;input.time=time;
            input.generation=B.oraclePublications->Generation();
            const int slot=revision.driverFramesSolver;
            if(slot>=0 && size_t(slot)<B.solvers.size() && size_t(slot)<B.aggregates.size() &&
               B.solvers[size_t(slot)].path==input.owner) {
                // Aggregate has one typed producer per solver. The common
                // compiler rejects multiple writers, so this retained value
                // is exactly RevisionStatic's declared input even on a held
                // generation; a later operation cannot overwrite it.
                for(const auto &value:B.opAdapter.values)
                    if(value.domain==uint32_t(RigExecBakedSlotDomain::Aggregate) &&
                       value.slot==uint32_t(slot) && value.initialized) {
                        input.available=true;input.value=B.aggregates[size_t(slot)];break;
                    }
            }
            frameValues.emplace(std::make_pair(revision.moverPath,input.owner),std::move(input));
        }
        if(revision.constraintDelta>=0 && B.deltaPresent[size_t(revision.constraintDelta)])
            deltas[revision.moverPath]=B.deltaValues[size_t(revision.constraintDelta)];
        const auto capture=[&](const RigExecBakedPointsBinding &binding,VtValue *out) {
            const GfVec3f *points=nullptr;size_t count=0;
            if(!RigExecBakedResolvePoints(B,binding,&points,&count))return false;
            *out=VtValue(count?VtVec3fArray(points,points+count):VtVec3fArray());return true;
        };
        for(const auto &binding:revision.pointBindings) {
            VtValue value;if(capture(binding,&value))pointValues[{revision.moverPath,binding.input}]=std::move(value);
        }
        for(const auto &channel:revision.blendChannels)for(const auto &sample:channel.samples) {
            VtValue value;if(capture(sample.pointBinding,&value))sampleValues[{revision.moverPath,sample.samplePath}]=std::move(value);
        }
    }
    bool valid=true;
    for(const auto &chain:B.chains) {
        if(!chain.haveBase || !chain.haveResult)continue;
        std::vector<RigExecMoverRecord> records;records.reserve(chain.revisions.size());
        for(const auto &revision:chain.revisions) {
            RigExecMoverRecord record;record.moverPath=revision.moverPath;
            record.schemaType=revision.binding.externalSchema;
            record.handler=revision.binding.handler;
            record.targets.push_back(chain.target);records.push_back(std::move(record));
        }
        std::vector<const RigExecMoverRecord *> rows;for(const auto &record:records)rows.push_back(&record);
        std::string firstRevisionMismatch;
        std::vector<std::string> referenceDiagnostics;
        const RigExecScalarReferenceContext context{scene,scene,B.upstream,
            [&](const SdfPath &path,const RigExecReadPhase &,const SdfPath &reader)->const VtValue* {
                const auto it=pointValues.find({reader,path});return it==pointValues.end()?nullptr:&it->second;
            },
            [&](const SdfPath &path,const RigExecReadPhase &phase,const SdfPath &)->const GfMatrix4d* {
                const auto slot=B.index.find(path);
                if(slot==B.index.end())return nullptr;
                if(phase.kind==RigExecReadPhaseKind::AtPrim) {
                    for(size_t i=B.frameRecords.size();i>0;--i) {
                        const auto &record=B.frameRecords[i-1];
                        if(record.slot==slot->second && B.frameMatrixValid[i-1] &&
                            (record.mover==phase.prim || record.mover.HasPrefix(phase.prim)))return &B.frameMatrix[i-1];
                    }
                }
                const auto &table=phase.kind==RigExecReadPhaseKind::Final?finalMatrices:baseMatrices;
                const auto it=table.find(path);return it==table.end()?nullptr:&it->second;
            },
            [&](const SdfPath &reader,const SdfPath &,const SdfPath &sample)->const VtValue* {
                const auto it=sampleValues.find({reader,sample});return it==sampleValues.end()?nullptr:&it->second;
            },
            [&](const SdfPath &reader,const SdfPath &weight,size_t count,std::vector<float>*out,std::string*error,const std::vector<GfVec3f>*preceding) {
                const auto original=B.oracleWeightInputs.find(weight);
                if(original==B.oracleWeightInputs.end()){*error="missing sampled weight reference inputs";return false;}
                const RigExecBakedProgramImpl::GeomRevision *revision=nullptr;
                for(const auto &candidate:chain.revisions)if(candidate.moverPath==reader){revision=&candidate;break;}
                if(!revision){*error="missing bound reference weight consumer";return false;}
                const auto *field=revision->weightField>=0
                    ? &B.weightFields[size_t(revision->weightField)] : nullptr;
                auto readerScene=[&] {
                    RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
                        B.profiler && B.profiler->IsEnabled() ? "Reference.ReaderScene" : "",
                        B.profiler && B.profiler->IsEnabled() ? "reference" : "");
                    return B.oraclePublications->ReaderScene(field?field->availableChains:allChains);
                }();
                for(size_t i=0;i<B.poseWeightPaths.size() && i<B.poseWeights.size();++i)
                    readerScene.overlay[B.poseWeightPaths[i]]=VtValue(B.poseWeights[i]);
                auto inputs=[&] {
                    RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
                        B.profiler && B.profiler->IsEnabled() ? "Reference.RebindWeight" : "",
                        B.profiler && B.profiler->IsEnabled() ? "reference" : "");
                    return RigExecRebindWeightReference(original->second,readerScene,time);
                }();
                inputs.phasedPoints.clear();
                if(field) for(const auto &read:field->pointReads) {
                    const auto answer=RigExecBakedReadWeightPointInput(B,revision->weightField,read.object,read.leaf);
                    if(!answer.declared || read.leaf<0 || read.leaf>=3)continue;
                    const auto objectPath=B.weightObjects[size_t(read.object)].path;
                    inputs.phasedPoints[{objectPath,relationNames[read.leaf]}]=answer.available
                        ? VtValue(answer.count?VtVec3fArray(answer.data,answer.data+answer.count):VtVec3fArray()):VtValue();
                }
                inputs.placements.clear();
                const bool basePlacement=!field ||
                    field->placementPhase==RigExecBakedProgramImpl::WeightField::PlacementPhase::Base;
                const auto &frames=basePlacement?B.base:B.fin;
                const auto &versions=basePlacement?B.baseLast:B.finLast;
                for(size_t slot=0;slot<B.paths.size() && slot<versions.size();++slot)
                    if(slot<B.placedVolumes.size() && B.placedVolumes[slot] && size_t(versions[slot])<frames.size())
                        inputs.placements.emplace(B.paths[slot],RigExecVolumePlacement(frames[size_t(versions[slot])]));
                RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
                    B.profiler && B.profiler->IsEnabled() ? "Reference.ResolveWeight" : "",
                    B.profiler && B.profiler->IsEnabled() ? "reference" : "");
                return RigExecResolveWeightReference(inputs,weight,count,out,error,preceding);
            },
            [&](size_t index,const SdfPath &reader,const VtVec3fArray &reference) {
                if(!firstRevisionMismatch.empty() || index>=chain.revisions.size())return;
                // Version index + 1, what this revision left, as one array.
                std::vector<GfVec3f> scratch;
                const GfVec3f *native=nullptr;size_t nativeCount=0;
                RigExecBakedVersionPoints(chain,index+1,&scratch,&native,&nativeCount);
                if(reference.size()!=nativeCount) {
                    firstRevisionMismatch="cpu reference first differing revision: "+reader.GetString()+
                        " count reference="+std::to_string(reference.size())+
                        " native="+std::to_string(nativeCount);
                    return;
                }
                for(size_t i=0;i<reference.size();++i) {
                    const auto &expected=reference[i];const auto &actual=native[i];
                    if(GfIsClose(expected,actual,1e-4))continue;
                    firstRevisionMismatch="cpu reference first differing revision: "+reader.GetString()+
                        TfStringPrintf(" point %zu reference=(%.9g, %.9g, %.9g) native=(%.9g, %.9g, %.9g)",
                            i,double(expected[0]),double(expected[1]),double(expected[2]),
                            double(actual[0]),double(actual[1]),double(actual[2]));
                    break;
                }
            },
            [&](const SdfPath &reader,const SdfPath &owner,UsdTimeCode request)->const RigExecOracleFrameInput* {
                const auto it=frameValues.find({reader,owner});
                return it==frameValues.end() || it->second.time!=request ||
                    it->second.generation!=B.oraclePublications->Generation()?nullptr:&it->second;
            },
            [&](const SdfPath &reader) {
                for (const auto &revision : chain.revisions)
                    if (revision.moverPath == reader)
                        return revision.weightObject >= 0 && revision.weightCurrentPhase;
                return false;
            }};
        const auto reference=[&] {
            RigExecProfileScope referenceProfile(B.profiler && B.profiler->IsEnabled() ? B.profiler : nullptr,
                B.profiler && B.profiler->IsEnabled() ? "Reference.ScalarChain" : "",
                B.profiler && B.profiler->IsEnabled() ? "reference" : "");
            return RigExecScalarChainReference(context,chain.target,rows,baseMatrices,finalMatrices,time,&referenceDiagnostics,deltas);
        }();
        pose->movedPropertiesCpu[chain.target]=VtValue(reference);
        bool agrees=reference.size()==chain.result.size();
        for(size_t i=0;agrees && i<reference.size();++i)agrees=GfIsClose(reference[i],chain.result[i],1e-4);
        if(agrees)++pose->referenceAgreements;
        else {++pose->referenceMismatches;valid=false;
            // Oracle-only details explain a mismatch, not a successful native pose.
            pose->diagnostics.insert(pose->diagnostics.end(),referenceDiagnostics.begin(),referenceDiagnostics.end());
            pose->diagnostics.push_back("cpu reference parity: value mismatch on "+chain.target.GetString());
            if(!firstRevisionMismatch.empty())pose->diagnostics.push_back(std::move(firstRevisionMismatch));}
    }
    return valid;
}
} // namespace rigExec

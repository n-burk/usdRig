#ifndef RIGEXEC_GRAPH_POSE_ARITHMETIC_H
#define RIGEXEC_GRAPH_POSE_ARITHMETIC_H
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
namespace rigExec {
struct RigExecSpaceParentContext {
    int anchor=-1;
    std::vector<int> recompose;
};
/// Source depth selects a namespace-parent checkpoint, without scheduling
/// operations or removing cycles. Explicit sources retain their composed
/// producers; carry spaces select the same contextual checkpoints as parents.
inline bool RigExecDeriveSpaceSwitchParentContexts(const std::vector<int> &parents,
    const std::vector<uint8_t> &composed,const std::vector<int> &switchSlots,
    const std::vector<std::vector<int>> &sources,
    std::vector<RigExecSpaceParentContext> *output,
    const std::vector<int> *carrySlots=nullptr,
    std::vector<RigExecSpaceParentContext> *carryOutput=nullptr)
{
    if(!output || parents.size()!=composed.size() || switchSlots.size()!=sources.size())return false;
    const int count=int(parents.size()),switchCount=int(switchSlots.size());
    if((carrySlots==nullptr)!=(carryOutput==nullptr) ||
       (carrySlots && carrySlots->size()!=switchSlots.size()))return false;
    if(carrySlots)for(int slot:*carrySlots)if(slot < -1 || slot>=count)return false;
    std::vector<int> bySlot(size_t(count),-1);
    for(int i=0;i<count;++i)if(parents[size_t(i)] < -1 || parents[size_t(i)]>=count)return false;
    for(int i=0;i<switchCount;++i) {
        const int slot=switchSlots[size_t(i)];
        if(slot<0 || slot>=count || bySlot[size_t(slot)]>=0)return false;
        bySlot[size_t(slot)]=i;
        for(int source:sources[size_t(i)])if(source < -1 || source>=count)return false;
    }
    const auto nearest=[&](int slot) {
        for(int remaining=count;slot>=0 && remaining>0;--remaining) {
            const int found=bySlot[size_t(slot)];
            if(found>=0)return found;
            slot=parents[size_t(slot)];
        }
        return -1;
    };
    std::vector<int> depth(size_t(switchCount),0);
    for(int pass=0;pass<switchCount;++pass) {
        auto next=depth;
        for(int i=0;i<switchCount;++i)for(int source:sources[size_t(i)]) {
            const int producer=nearest(source);
            if(producer>=0)next[size_t(i)]=std::max(next[size_t(i)],
                std::min(switchCount,depth[size_t(producer)]+1));
        }
        if(next==depth)break;
        depth.swap(next);
    }
    const auto bind=[&](int reader,int at) {
        RigExecSpaceParentContext context;
        for(int remaining=count;at>=0 && composed[size_t(at)] && remaining>0;--remaining) {
            const int producer=nearest(at);
            if(producer<0 || depth[size_t(producer)]<depth[size_t(reader)])break;
            context.recompose.push_back(at);at=parents[size_t(at)];
        }
        context.anchor=at;
        std::reverse(context.recompose.begin(),context.recompose.end());
        return context;
    };
    output->assign(size_t(switchCount),RigExecSpaceParentContext());
    if(carryOutput)carryOutput->assign(size_t(switchCount),RigExecSpaceParentContext());
    for(int i=0;i<switchCount;++i) {
        (*output)[size_t(i)]=bind(i,parents[size_t(switchSlots[size_t(i)])]);
        if(carryOutput) {
            const int slot=(*carrySlots)[size_t(i)];
            bool ownTarget=false;
            for(int at=slot,remaining=count;at>=0 && remaining>0;--remaining) {
                if(at==switchSlots[size_t(i)]){ownTarget=true;break;}
                at=parents[size_t(at)];
            }
            // An explicit reference under its own target keeps its real
            // producer edge. The final SCC owns this authored cycle.
            if(ownTarget)(*carryOutput)[size_t(i)].anchor=slot;
            else (*carryOutput)[size_t(i)]=bind(i,slot);
        }
    }
    return true;
}
template<class Matrix> struct RigExecSpaceCheckpointInputT {
    Matrix avars{1.0},posedDefault{1.0},parentDefaultInverse{1.0};
    Matrix parentExpression{1.0},posedAuthoredMatrix{1.0};
    bool parentExpressionAuthored=false,posedAuthored=false;
};
/// Exact ordinary provider multiplication shared with contextual closures.
template<class Matrix>
Matrix RigExecComposeUnswitchedPoseMatrix(const Matrix &avars,const Matrix &posedDefault,
    const Matrix &parentDefaultInverse,const Matrix &parentPosed)
{
    return avars*posedDefault*parentDefaultInverse*parentPosed;
}
template<class Math>
typename Math::Matrix RigExecRunSpaceCheckpointArithmetic(
    const typename Math::Matrix &ancestor,
    const std::vector<RigExecSpaceCheckpointInputT<typename Math::Matrix>> &inputs)
{
    auto posed=ancestor;
    for(const auto &input:inputs) {
        const auto matrix=input.posedAuthored?input.posedAuthoredMatrix:
            RigExecComposeUnswitchedPoseMatrix(input.avars,input.posedDefault,
                input.parentDefaultInverse,input.parentExpressionAuthored?input.parentExpression:posed);
        posed=Math::RoundTripCheckpoint(matrix);
    }
    return posed;
}
template<class Vector> struct RigExecSpaceSwitchRecordT {
    std::vector<int> filters;
    Vector twistAxis{1,0,0};
    std::array<bool,3> affectTranslation{{true,true,true}};
    std::array<bool,3> affectRotation{{true,true,true}};
    std::array<bool,3> affectScale{{true,true,true}};
};
template<class Matrix> struct RigExecSpaceSwitchSourceT {
    bool world=false;
    Matrix defaultSpace{1.0},posedSpace{1.0};
};
template<class Matrix> struct RigExecSpaceSwitchInputsT {
    Matrix avars{1.0},posedDefault{1.0},parentDefaultInverse{1.0};
    Matrix parentPosed{1.0},parentDefault{1.0};
    Matrix spaceDefault{1.0},spacePosed{1.0};
    bool hasCarry=false;
    double active=0;
    std::vector<RigExecSpaceSwitchSourceT<Matrix>> sources;
};
/// The native and portable backends supply only matrix/filter primitives.
/// Multiply order and identity bypasses are part of the shared kernel.
template<class Math,class Record,class Input>
bool RigExecRunSpaceSwitchArithmetic(const Record &record,const Input &input,
    typename Math::Matrix *output)
{
    using M=typename Math::Matrix;
    if(!output || input.sources.empty() ||
       (!record.filters.empty() && record.filters.size()!=input.sources.size()))return false;
    const M local=Math::RoundTrip(input.avars*input.posedDefault*
        input.parentDefaultInverse*input.parentPosed)*
        input.parentPosed.GetInverse()*input.parentDefault;
    const M localInverse=local.GetInverse();
    double active=std::isfinite(input.active)?input.active:0.0;
    active=std::max(0.0,std::min(active,double(input.sources.size()-1)));
    const size_t lower=size_t(std::floor(active));
    const size_t upper=std::min(lower+1,input.sources.size()-1);
    const double blend=active-double(lower);
    M carry(1.0),carryInverse(1.0);
    if(input.hasCarry) {
        carry=input.spaceDefault.GetInverse()*input.spacePosed;
        carryInverse=carry.GetInverse();
    }
    const auto deltaOf=[&](size_t index) {
        const auto &source=input.sources[index];
        if(source.world)return input.hasCarry?local*carry*localInverse:M(1.0);
        const int filter=record.filters.empty()?0:record.filters[index];
        const auto axis=source.defaultSpace.TransformDir(record.twistAxis);
        const M moved=source.defaultSpace.GetInverse()*source.posedSpace;
        const M motion=input.hasCarry?
            Math::Filter(moved*carryInverse,axis,filter)*carry:
            Math::Filter(moved,axis,filter);
        return local*motion*localInverse;
    };
    M delta=deltaOf(lower);
    if(upper!=lower && blend>0.0)delta=Math::Blend(delta,deltaOf(upper),blend);
    delta=Math::Mask(delta,record.affectTranslation.data(),record.affectRotation.data(),
        record.affectScale.data());
    *output=delta*local;
    return true;
}
enum class RigExecPosePropagateOutcome : unsigned char {
    Staged,Skipped,NoCandidate,UnusableDescendant,SingularDelta,InvalidResult
};
template<class Math,class Frame,class Matrix>
RigExecPosePropagateOutcome RigExecPropagatePoseArithmetic(const Frame &current,
    const Frame &before,const Frame &after,const Matrix &delta,bool deltaOk,
    bool solverOutput,bool parentBlocked,bool candidatePresent,Frame *output) {
    using O=RigExecPosePropagateOutcome;
    if(parentBlocked || !candidatePresent)return O::Skipped;
    if(solverOutput && (!Math::Usable(current) || !Math::Usable(before) || !Math::Usable(after)))return O::Skipped;
    if(!Math::Usable(current))return O::UnusableDescendant;
    if(!deltaOk)return O::SingularDelta;
    const Frame result=Math::Carry(current,delta);
    if(!Math::Usable(result))return O::InvalidResult;
    *output=result;return O::Staged;
}
template<class Solver> struct RigExecPoseInterpolatorRecordT {
    Solver solver;
    bool enableTranslation=false,allowNegativeWeights=true;
    size_t poseCount=0;
};
template<class Frame> struct RigExecPoseInterpolatorInputsT {
    bool enabled=true;
    std::vector<double> numeric;
    const Frame *driverFinal=nullptr,*driverRest=nullptr;
    const Frame *parentFinal=nullptr,*parentRest=nullptr;
};
enum class RigExecPoseInterpolatorStatus {
    Success,Disabled,UnusableRotation,UnusableTranslation,CountMismatch
};
/// Current frames and numeric channels are delivered after graph readiness.
template<class Math,class Record,class Input>
RigExecPoseInterpolatorStatus RigExecRunPoseInterpolatorArithmetic(
    Record &record,const Input &input,std::vector<double> *weights)
{
    using Q=typename Math::Quaternion;using V=typename Math::Vector;
    if(!input.enabled || !record.poseCount) {
        weights->clear();return RigExecPoseInterpolatorStatus::Disabled;
    }
    const bool numeric=!input.numeric.empty();
    Q driverFinal(1.0),driverRest(1.0),parentFinal(1.0),parentRest(1.0);
    bool usable=numeric || (input.driverFinal && input.driverRest &&
        Math::FrameRotation(*input.driverFinal,&driverFinal) &&
        Math::FrameRotation(*input.driverRest,&driverRest));
    if(!numeric && usable && input.parentFinal)
        usable=input.parentRest && Math::FrameRotation(*input.parentFinal,&parentFinal) &&
            Math::FrameRotation(*input.parentRest,&parentRest);
    if(!usable) {weights->clear();return RigExecPoseInterpolatorStatus::UnusableRotation;}
    const Q local=parentFinal.GetInverse()*driverFinal;
    const Q restLocal=parentRest.GetInverse()*driverRest;
    const Q delta=(restLocal.GetInverse()*local).GetNormalized();
    V translation(0.0);const V *translationPtr=nullptr;
    if(numeric) {
        for(size_t k=0;k<input.numeric.size() && k<3;++k)translation[int(k)]=input.numeric[k];
        translation/=100.0;translationPtr=&translation;
    } else if(record.enableTranslation) {
        if(!Math::FrameTranslation(*input.driverFinal,*input.driverRest,
            input.parentFinal,input.parentRest,&translation)) {
            weights->clear();return RigExecPoseInterpolatorStatus::UnusableTranslation;
        }
        translation/=100.0;translationPtr=&translation;
    }
    record.solver.Evaluate(Math::EulerFromQuaternion(delta),translationPtr,weights,
        record.allowNegativeWeights);
    return weights->size()==record.poseCount?RigExecPoseInterpolatorStatus::Success:
        RigExecPoseInterpolatorStatus::CountMismatch;
}
}
#endif

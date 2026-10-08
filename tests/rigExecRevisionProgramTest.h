#ifndef RIGEXEC_REVISION_PROGRAM_TEST_H
#define RIGEXEC_REVISION_PROGRAM_TEST_H
#include "rigExec/moverGraph.h"
#include "rigExec/types.h"
#include "rigExecGraph/opGraph.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace rigExecTest {
struct RevisionValue {
    size_t index=std::numeric_limits<size_t>::max();
};

// Packet/kernel tests use the production producer compiler and sole executor.
// The fixture owns immutable source packets and retained result values only.
class RevisionProgram {
    static bool SamePoints(const pxr::VtVec3fArray &a,const pxr::VtVec3fArray &b) {
        if(a.size()!=b.size())return false;
        for(size_t i=0;i<a.size();++i)for(size_t c=0;c<3;++c)
            if(std::memcmp(&a[i][c],&b[i][c],sizeof(float)))return false;
        return true;
    }
    struct Value {
        bool source=true,changed=true,packetChanged=true;
        RevisionValue incoming;
        rigExec::RigExecRevisionOp op{};
        rigExec::RigExecMoverParameters parameters;
        rigExec::RigExecMoverStatus requested,status;
        pxr::VtVec3fArray points;
        rigExec::RigExecWireBasisCache wire;
    };
    std::vector<Value> _values;
    std::vector<size_t> _operations;
    rigExec::RigExecCompiledGraph _graph;
    rigExec::RigExecOpWorkspace _workspace;
    size_t _compiledValues=0,_builds=0,_executed=0;
    bool _everRan=false;
public:
    RevisionValue AddPointSource(const pxr::SdfPath&,const pxr::VtVec3fArray &points) {
        Value value;value.points=points;
        _values.push_back(std::move(value));return {_values.size()-1};
    }
    RevisionValue AddRevision(rigExec::RigExecRevisionOp op,RevisionValue incoming,
        const rigExec::RigExecMoverParameters &parameters,const rigExec::RigExecMoverStatus &status) {
        if(incoming.index>=_values.size())throw std::runtime_error("invalid incoming revision");
        Value value;value.source=false;value.incoming=incoming;value.op=op;
        value.parameters=parameters;value.requested=status;
        _values.push_back(std::move(value));return {_values.size()-1};
    }
    bool UpdatePointSource(RevisionValue id,const pxr::VtVec3fArray &points) {
        if(id.index>=_values.size() || !_values[id.index].source ||
           points.size()!=_values[id.index].points.size())return false;
        auto &value=_values[id.index];value.changed=value.changed || !SamePoints(value.points,points);
        value.points=points;return true;
    }
    bool UpdateRevision(RevisionValue id,const rigExec::RigExecMoverParameters &parameters,
        const rigExec::RigExecMoverStatus &status) {
        if(id.index>=_values.size() || _values[id.index].source)return false;
        auto &value=_values[id.index];
        value.packetChanged=value.packetChanged || value.parameters!=parameters || !(value.requested==status);
        value.parameters=parameters;value.requested=status;return true;
    }
    pxr::VtVec3fArray Evaluate(RevisionValue requested) {
        if(requested.index>=_values.size())return {};
        std::vector<uint32_t> newOperations;
        if(_compiledValues!=_values.size()) {
            std::vector<rigExec::RigExecOpDescriptor> descriptors;
            std::vector<rigExec::RigExecValueId> leaves;
            _operations.clear();
            for(size_t i=0;i<_values.size();++i) {
                const auto &value=_values[i];
                if(value.source)leaves.push_back(i*2);
                else {
                    rigExec::RigExecOpDescriptor op;op.key="revision/"+std::to_string(i);
                    op.kind=uint32_t(value.op);op.reads={value.incoming.index*2,i*2+1};op.writes={i*2};
                    descriptors.push_back(std::move(op));_operations.push_back(i);leaves.push_back(i*2+1);
                }
            }
            std::string error;
            if(!rigExec::RigExecCompileOpGraph(descriptors,leaves,rigExec::RigExecCyclePolicy::Reject,&_graph,&error))
                throw std::runtime_error(error);
            for(uint32_t c=0;c<_graph.ops.size();++c)
                if(_operations[_graph.ops[c].originalIndex]>=_compiledValues)newOperations.push_back(c);
            _compiledValues=_values.size();++_builds;
        }
        std::vector<rigExec::RigExecValueId> changed;
        for(size_t i=0;i<_values.size();++i) {
            auto &value=_values[i];
            if(value.source && value.changed)changed.push_back(i*2);
            if(!value.source && value.packetChanged)changed.push_back(i*2+1);
            if(!value.source)value.changed=false;
        }
        rigExec::RigExecOpCallbacks callbacks;
        callbacks.changed=[&](rigExec::RigExecValueId id) {
            return id%2 ? _values[size_t(id/2)].packetChanged : _values[size_t(id/2)].changed;
        };
        callbacks.run=[&](uint32_t c) {
            auto &value=_values[_operations[_graph.ops[c].originalIndex]];
            const auto &incoming=_values[value.incoming.index].points;
            std::vector<pxr::GfVec3f> result(incoming.begin(),incoming.end());
            auto status=value.requested;
            if(status.state=="ok" && !rigExec::RigExecRunRevisionKernel(value.op,value.parameters,&result,
                rigExec::RigExecSimdEnabled(),&value.wire)) {
                result.assign(incoming.begin(),incoming.end());status.state=pxr::TfToken("moverFailed");
            } else if(status.state!="ok")result.assign(incoming.begin(),incoming.end());
            pxr::VtVec3fArray points(result.begin(),result.end());
            value.changed=!SamePoints(points,value.points) || !(status==value.status);
            value.points=std::move(points);value.status=std::move(status);++_executed;return true;
        };
        callbacks.skip=[](uint32_t){};
        rigExec::RigExecOpExecution execution;std::string error;
        if(!rigExec::RigExecExecuteOpGraph(_graph,changed,newOperations,!_everRan,callbacks,&execution,&error,&_workspace))
            throw std::runtime_error(error);
        _everRan=true;
        for(auto &value:_values){value.changed=false;value.packetChanged=false;}
        return _values[requested.index].points;
    }
    rigExec::RigExecMoverStatus GetRevisionStatus(RevisionValue id)const {return _values.at(id.index).status;}
    size_t GetRevisionCount()const {return size_t(std::count_if(_values.begin(),_values.end(),[](const Value&v){return !v.source;}));}
    size_t GetRevisionExecutionCount()const {return _executed;}
    size_t GetScheduleBuildCount()const {return _builds;}
};
}
#endif

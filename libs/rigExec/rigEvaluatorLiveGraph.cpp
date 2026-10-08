// Diagnostic projection of the authoritative compiled operation graph.
#include "rigEvaluator.h"
#include "bakedProgram.h"
#include <algorithm>
namespace rigExec {
RigExecLiveOperationGraph RigExecRigEvaluator::DescribeLiveOperations() const {
    RigExecLiveOperationGraph graph;
    if(!_bakedProgram)return graph;
    const auto operations=_bakedProgram->GetOpGraph();
    std::vector<size_t> depths(operations.size(),0);
    for(const auto &operation:operations) {
        size_t depth=0;
        for(auto predecessor:operation.preds)
            if(predecessor<depths.size())depth=std::max(depth,depths[predecessor]+1);
        if(operation.step<depths.size())depths[operation.step]=depth;
        RigExecLiveOpNode node;
        node.id="native:"+std::to_string(operation.step);
        node.domain=operation.domain;node.kind=operation.kind;node.label=operation.label;
        node.profile=operation.label;node.level=int(depth);node.order=int(operation.step);
        node.details["cluster"]=std::to_string(operation.cluster);
        graph.nodes.push_back(std::move(node));
        for(const auto predecessor:operation.preds)
            graph.edges.push_back({"native:"+std::to_string(predecessor),
                "native:"+std::to_string(operation.step),"dependency"});
    }
    return graph;
}
} // namespace rigExec

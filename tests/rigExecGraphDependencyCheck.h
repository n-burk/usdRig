#ifndef RIGEXEC_TEST_GRAPH_DEPENDENCY_CHECK_H
#define RIGEXEC_TEST_GRAPH_DEPENDENCY_CHECK_H
#include "rigExec/bakedTrace.h"
#include "pxr/usd/sdf/path.h"
#include <limits>
#include <vector>
namespace rigExecTest {
inline size_t FindOperation(const std::vector<rigExec::RigExecOpGraphNode> &graph,
                            const pxr::SdfPath &owner,const char *kind) {
    for (const auto &node:graph)
        if (node.kind==kind && node.label.find(owner.GetString())!=std::string::npos) return node.step;
    return std::numeric_limits<size_t>::max();
}
inline bool HasDependencyPath(const std::vector<rigExec::RigExecOpGraphNode> &graph,size_t from,size_t to) {
    if(from>=graph.size() || to>=graph.size() || from==to)return false;
    std::vector<unsigned char> seen(graph.size(),0);std::vector<size_t> pending{from};
    while(!pending.empty()) {
        const size_t current=pending.back();pending.pop_back();
        if(current>=graph.size() || seen[current])continue;seen[current]=1;
        for(size_t next:graph[current].succs) {if(next==to)return true;pending.push_back(next);}
    }
    return false;
}
}
#endif

#ifndef RIGEXEC_GRAPH_AUTO_CLAVICLE_GRAPH_H
#define RIGEXEC_GRAPH_AUTO_CLAVICLE_GRAPH_H
#include "sceneGraphTypedRead.h"
#include "rigExecMath/autoClavicleKernel.h"
#include <array>

namespace rigExec {
struct RigExecBoundAutoClavicle {
    RigExecAutoClavicleConstants constants;
    // Target, pivot, anchor, anchor default, FK posed, three FK defaults,
    // optional IK target and pole, and optional three solver rest frames.
    std::array<RigExecValueId,13> frames;
    std::array<RigExecGraphTypedRead,11> scalars;
    bool hasLimb=false;
    RigExecValueId output=UINT64_MAX;
    std::vector<RigExecValueId> reads;
    std::string unavailable;
    RigExecBoundAutoClavicle() { frames.fill(UINT64_MAX); }
};
bool RigExecBindAutoClavicle(const RigExecSceneDescriptors &,const SdfPath &,
    const RigExecSceneGraphBindingContext &,RigExecValueId incoming,
    RigExecValueId output,RigExecBoundAutoClavicle *,std::string *error=nullptr);
bool RigExecRunAutoClavicle(const RigExecBoundAutoClavicle &,
    RigExecTypedValueStore *,std::string *error=nullptr);
}
#endif

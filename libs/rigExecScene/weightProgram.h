#ifndef RIGEXEC_GRAPH_WEIGHT_PROGRAM_H
#define RIGEXEC_GRAPH_WEIGHT_PROGRAM_H
#include "rigExec/weightPackets.h"
#include "pxr/base/ts/spline.h"
#include <array>
#include <deque>
#include <cstdint>
#include <string>
#include <vector>
namespace rigExec {
/// Immutable numerical record. Indices name records in the same program;
/// composition preserves the authored relationship order.
struct RigExecWeightRecord {
    std::string name, staticError, phaseError;
    int kind=-1, representation=-1, combineMode=-1, base=-1;
    bool clamp=false, samplesInFlight=false;
    TfToken type, representationToken, rangePolicy, combineToken, planeAxis, planeBounds;
    std::vector<int> inputs, indices;
    std::vector<float> values, falloffCurve;
};
struct RigExecWeightPointInput {
    bool declared=false, available=false;
    const GfVec3f *data=nullptr;
    size_t count=0;
};
/// Current-generation inputs only. Available empty arrays retain their
/// success state; declared unavailable phases cannot reuse raw/older data.
struct RigExecWeightFieldInputs {
    std::array<float,19> scalars{{0,1,1,0,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1}};
    std::array<RigExecWeightPointInput,3> rawPoints, phasedPoints;
    bool hasPlacement=false, blocked=false;
    bool paintedArraysDeclared=false;
    const float *paintedValues=nullptr;
    size_t paintedValueCount=0;
    const int *paintedIndices=nullptr;
    size_t paintedIndexCount=0;
    GfMatrix4d placement=GfMatrix4d(1.0);
    std::string axis="x", bounds="unbounded";
};
/// Exclusive scratch for one field producer. Dense child storage grows with
/// recursion depth, not the number of authored combine inputs.
struct RigExecWeightFieldWorkspace {
    std::deque<std::vector<float>> children;
    std::vector<uint32_t> seen;
    uint32_t seenGeneration=0;
    std::vector<GfVec3f> localCurve;
};
bool RigExecRunWeightField(const std::vector<RigExecWeightRecord> &,int root,
    const std::vector<RigExecWeightFieldInputs> &,size_t count,
    const RigExecWeightPointView *currentPoints,RigExecWeightFieldWorkspace *,
    std::vector<float> *result,std::string *error);
bool RigExecRunWeightField(const std::vector<RigExecWeightRecord> &,int root,
    const std::vector<RigExecWeightFieldInputs> &,size_t count,
    const std::vector<GfVec3f> *currentPoints,std::vector<float> *result,
    std::string *error);
/// Shared field/default envelope policy. A bound object uses its own range
/// policy; defaultWeight is consulted only when root is absent (-1).
bool RigExecRunEffectiveWeightField(const std::vector<RigExecWeightRecord> &,int root,
    const std::vector<RigExecWeightFieldInputs> &,size_t count,
    const std::vector<GfVec3f> *currentPoints,float defaultWeight,
    std::vector<float> *result,std::string *error);
/// Ordinary mover packets retain their existing packet validity policy;
/// field/envelope evaluation above retains its diagnostic policy.
struct RigExecWeightPacketInputs {
    RigExecStaticWeightInputsView painted;
    RigExecDynamicWeightInputs dynamic;
    RigExecVolumeWeightInputs volume;
    const RigExecWeightPacket *base=nullptr;
    RigExecWeightPacketWorkspace *workspace=nullptr;
    std::vector<const RigExecWeightPacket *> inputs;
    const std::vector<const RigExecWeightPacket *> *borrowedInputs=nullptr;
    size_t targetCount=0;
    float strength=1, invert=0;
};
RigExecWeightPacket RigExecRunWeightPacket(const RigExecWeightRecord &,
    const RigExecWeightPacketInputs &);
/// Compile-time spline resampling shared by both source adapters.
std::vector<float> RigExecBakeWeightFalloff(const TfToken &,const TsSpline *spline=nullptr);
}
#endif

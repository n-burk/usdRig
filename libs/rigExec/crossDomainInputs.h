// Compile-selected cross-domain inputs, consumed through the shared op graph.
#ifndef RIGEXEC_CROSS_DOMAIN_INPUTS_H
#define RIGEXEC_CROSS_DOMAIN_INPUTS_H
#include "moverGraphTypes.h"
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rigExec {
struct RigExecBakedProgramImpl;
struct RigExecBakedPointsBinding;
struct RigExecBakedSlotRange;

inline constexpr const char *RigExecInputElementMetadataName = "rigExecInputElement";

struct RigExecCrossDomainRead {
    enum class Kind : uint8_t { PointElement, PoseFrame, SpaceValue, PropertyResult, Points };
    Kind kind = Kind::PointElement;
    SdfPath consumer, source, reader;
    RigExecReadPhase phase;
    int element = -1;
    int provider = -1;
    int rawLeaf = -1;
    int spaceValue = -1;
    int propertyChain = -1;
    uint32_t propertyVersion = 0;
    bool baseFrame = false;
    bool finalPoints = false;
    struct PointVersion { int chain = -1, version = 0; };
    std::vector<PointVersion> points;
    std::vector<uint32_t> frames;
    std::string unavailable;
};

/// Captures the composed stack order once. It numbers versions and positional
/// reads; the shared producer graph determines execution order.
void RigExecBakedCaptureCrossDomainOrder(RigExecBakedProgramImpl *);
RigExecBakedPointsBinding RigExecBakedBindPointInput(
    const RigExecBakedProgramImpl &, const SdfPath &input,
    const RigExecReadPhase &, const SdfPath &reader);
void RigExecBakedDeclarePointInput(const RigExecBakedProgramImpl &,
    const RigExecBakedPointsBinding &, std::vector<RigExecBakedSlotRange> *);
void RigExecBakedDeclarePointVersion(const RigExecBakedProgramImpl &,
    int chain, int version, std::vector<RigExecBakedSlotRange> *);
/// Source-free normalized connection binding for the provider expression IR.
/// Property/points default to version zero; pose base is the last solver
/// version. Final pose IDs are completed by FinalizeCrossDomainReads.
bool RigExecBakedBindConnectionValue(const RigExecBakedProgramImpl &,
    const SdfPath &consumer, const SdfPath &source, const RigExecReadPhase &,
    RigExecCrossDomainRead *, int element = -1);

/// Called after pose SSA and every consumer step exist, before the shared
/// compiler. Adds producer value reads, including reverse domain dependencies.
bool RigExecBakedFinalizeCrossDomainReads(RigExecBakedProgramImpl *, std::string *);
bool RigExecBakedReadCrossDomain(const RigExecBakedProgramImpl &, int read,
    VtValue *value, std::string *diagnostic = nullptr);
void RigExecBakedDeclareCrossDomainRead(const RigExecBakedProgramImpl &, int read,
    std::vector<RigExecBakedSlotRange> *);
struct RigExecBakedPointInputValue {
    bool declared = false, available = false;
    const GfVec3f *data = nullptr;
    size_t count = 0;
};
bool RigExecBakedBindWeightPointInputs(RigExecBakedProgramImpl *,std::string *);
void RigExecBakedDeclareWeightPointInputs(const RigExecBakedProgramImpl &,int field,
    std::vector<RigExecBakedSlotRange> *);
RigExecBakedPointInputValue RigExecBakedReadWeightPointInput(
    const RigExecBakedProgramImpl &,int field,int object,int leaf);
}
#endif

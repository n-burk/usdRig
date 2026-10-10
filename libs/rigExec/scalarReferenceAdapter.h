// Current-generation oracle publication adapter, independent of retained caches.
#ifndef RIGEXEC_SCALAR_REFERENCE_ADAPTER_H
#define RIGEXEC_SCALAR_REFERENCE_ADAPTER_H
#include "oracleInputs.h"
#include "weightReference.h"
#include <set>
namespace rigExec {
class RigExecOraclePublicationContext {
public:
    using Writes=std::vector<std::pair<SdfPath,VtValue>>;
    /// producedPaths names every target and phased reader alias a chain can publish.
    /// protectedPaths are interactive reader aliases set aside by the original walk.
    void Begin(uint64_t generation,RigExecOracleScene source,size_t chainCount,
        const std::set<SdfPath> &producedPaths,const std::set<SdfPath> &protectedPaths);
    /// Begin another execution over the already sampled immutable source facts.
    /// Frozen playback calls this without consulting USD or retained outputs.
    void Restart(uint64_t generation);
    /// Called at logical completion (including a skipped, unchanged source), with
    /// target first and records in original recorded order. No retained validity inference.
    bool Finish(size_t chain,const Writes &writes);
    RigExecOracleScene ReaderScene(const std::vector<int> &availableChains) const;
    uint64_t Generation() const { return _generation; }
    const RigExecOracleScene &SourceScene() const { return _source; }
private:
    struct Chain { uint64_t finished=0;bool completed=false;Writes writes; };
    uint64_t _generation=0;
    RigExecOracleScene _source;
    std::set<SdfPath> _protected;
    std::vector<Chain> _chains;
};
/// Re-resolves original scalar input sites over captured source facts and the
/// reader's publication context; production WeightField outputs are never inputs.
RigExecWeightReferenceContext RigExecRebindWeightReference(
    RigExecWeightReferenceContext inputs,const RigExecOracleScene &reader,UsdTimeCode time);
struct RigExecBakedProgramImpl;
struct RigExecRigPose;
/// Owning-thread source sampling before graph bodies; disabled unless the
/// program's oraclePublications optional has been enabled by the check owner.
void RigExecBakedBeginOracleReference(RigExecBakedProgramImpl *,uint64_t generation,UsdTimeCode time);
/// Called on the source owner before dispatch; detached workers only reset
/// already captured facts and never query a stage.
void RigExecBakedPrepareOracleReference(RigExecBakedProgramImpl *,UsdTimeCode);
/// Publishes each canonical SCC loop once, in compiled canonical order.
void RigExecBakedAppendCycleDiagnostics(const RigExecBakedProgramImpl &,RigExecRigPose *);
/// After the graph joins, independently evaluates point chains over sampled
/// source facts and actual bound phase inputs; returns false on disagreement.
bool RigExecBakedRunScalarReference(RigExecBakedProgramImpl *,UsdTimeCode,RigExecRigPose *);
} // namespace rigExec
#endif

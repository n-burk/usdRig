// rigExecRuntime: zero-USD .rigexec playback (M2).
// Opens a baked .rigexec file, replays its cluster DAG per frame, and
// publishes joint matrices and deformed points -- no USD headers, no USD
// library. The wire structs and decoders come from rigExecBinary (already
// USD-free); the math is runtimeMath.h (a bit-identical Gf mirror); the
// step bodies are ports of the baked program's, one family per .cpp.
// Fidelity rule (plan section 5): for the same frame, floating-point
// results must be bit-identical to the baked path. rigExecPose
// --verify-binary gates every family on dynamic==baked==binary over the
// shipped examples.
// Threading (D2): Execute is serial over clusters; the consumer may run
// clusters in parallel when the cluster DAG allows (the OpenUSD side
// keeps its dispatcher, Godot uses WorkerThreadPool). The reader holds
// no locks: one reader per thread, or external synchronization.
#ifndef RIGEXEC_RUNTIME_H
#define RIGEXEC_RUNTIME_H

#include "rigExecBinary/computed.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/external.h"
#include "rigExecBinary/geometry.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/pose.h"
#include "rigExecBinary/program.h"
#include "rigExecRuntime/runtimeMath.h"
#include "rigExecRuntime/store.h"

#include <cstdint>
#include <memory>
#include <map>
#include <string>
#include <vector>

namespace rigExec {

// A joint-matrix output: the joint's path (as baked) plus its final
// asset-space matrix, row-major.
struct RigExecRuntimeJointMatrix {
    std::string path;
    RrMat4d matrix;
};

// A deformed-points output: the moved property's path plus its points.
struct RigExecRuntimePoints {
    std::string path;
    std::vector<RrVec3f> points;
};

// A matrix-primvar output: the `<prim>.primvars:<name>` property a surface
// projector publishes plus its value, row-major.
struct RigExecRuntimeMatrixPrimvar {
    std::string path;
    RrMat4d matrix;
};

// A weight-frame output: the volume weight's path plus its placement.
struct RigExecRuntimeWeightFrame {
    std::string path;
    RrMat4d matrix;
};

// A resolved weight-field output: the weight object's path, its target
// path, and the resolved per-element weights.
struct RigExecRuntimeWeightField {
    std::string path;
    std::string target;
    std::vector<float> weights;
};

// A constraint-driven transform output: the provider prim's path plus the
// revised asset-space matrix and the asset-space base it revised. Both or
// neither, like the baked pose's providerXforms/providerBaseXforms pair:
// the imaging chain turns the pair into a world-space delta.
struct RigExecRuntimeProviderXform {
    std::string path;
    RrMat4d matrix;
    RrMat4d base;
};

// A property chain's published value: the target's or a phased consumer's
// attribute path plus the value.
struct RigExecRuntimePropertyValue {
    std::string path;
    RrPropertyValue value;
};

// The generation's work counters, summed over steps like the epilogue.
struct RigExecRuntimeCounters {
    uint32_t revisionsExecuted = 0;
    uint32_t revisionsCreated = 0;
    uint32_t schedulesBuilt = 0;
    uint32_t chainsBuilt = 0;
    uint32_t revisionsBuilt = 0;
};

// Plays a .rigexec file. Open decodes every section; SetFrame selects a
// baked frame's inputs; Execute replays the cluster DAG; the getters
// publish the frame's outputs. Any failure returns false with a reason;
// the reader keeps its last good state.
class RigExecRuntimeReader
{
public:
    ~RigExecRuntimeReader() = default;

    RigExecRuntimeReader(const RigExecRuntimeReader &) = delete;
    RigExecRuntimeReader &operator=(const RigExecRuntimeReader &) = delete;

    // Opens a .rigexec image. False with the reason on a malformed file,
    // an undecodable section, or tables whose cross-references do not
    // close (versions, input reads, cone sizes).
    static std::unique_ptr<RigExecRuntimeReader> Open(
        const uint8_t *bytes, size_t size, std::string *error);

    // Baked frame times, in file order.
    std::vector<double> GetFrameTimes() const;

    // Selects a baked frame's inputs by exact time. False naming the
    // miss when the file carries no such frame.
    bool SetFrame(double frame, std::string *error);

    // Persistent TRS avar overrides, placed on every Execute as the program
    // places interactive overrides. Angles are degrees. Only compiled
    // control slots with TRS poses and finite values are supported. The
    // override is what every read whose connection walk passes the avar
    // meets there, the avar's own binding and any reader downstream of it
    // alike; a reader that reads a chain at a phase stands aside when the
    // override is on it or on a hop of its connection, as in the USD
    // evaluators. On an avar math movers revise, the override is the
    // chain's base: the movers revise it as they would the value authored,
    // so every reader plays what a file baked with that value plays.
    // One gap against the USD evaluators remains for now: a plugin mover
    // applies the payload its bake assembled for the frame, so a drag that
    // reaches an input the plugin reads does not reach that mover's output.
    bool SetAvar(const std::string &propertyPath, double value,
                 std::string *error);
    void ClearAvars();

    // Replays the cluster DAG for the selected frame. False naming the
    // first failing step; outputs keep their previous frame.
    bool Execute(std::string *error);

    // Plugin movers: a file may hold movers an external library registered.
    // Playback runs each through the kernel the host installs for its type
    // (rigExecBinary/external.h). A type with no kernel passes its points
    // through, and every Execute names it in the diagnostics.

    // Every plugin mover type the file holds, sorted, each once.
    std::vector<std::string> GetExternalMoverTypes() const;
    // Installs \p kernel for \p type and prepares each of its revisions.
    // False with the reason when the file holds no such type or a
    // revision's epoch bytes do not prepare; those revisions keep passing
    // their points through.
    bool SetExternalKernel(const std::string &type,
                           const RigExecExternalKernel &kernel,
                           std::string *error);
    // The plugin mover types with no prepared kernel, sorted.
    std::vector<std::string> GetMissingExternalKernels() const;

    // Test-only family mask (bit 0 pose, 1 weights, 2 geometry, all set
    // by default). A masked family's steps are skipped, which is how
    // one family's outputs are compared while another is still landing.
    void SetRunMaskForTesting(unsigned mask);

    // Test-only cross-check of what the runtime computes against the frame
    // record the bake also wrote, bit for bit: every property-chain result,
    // constraint envelope and current-phase weight packet; every registered
    // read evaluated over the slots against the record's value where it
    // holds one, else the table's constant; every blend channel weight,
    // revision default weight and connection-following mover scalar the
    // assembly reads, where the record holds it. A run with an avar
    // override standing is not compared: the record never saw the drag. A
    // mismatch fails Execute naming the record field and its index. Off by
    // default; RIGEXEC_RUNTIME_CROSSCHECK set to anything but empty or "0"
    // when Open runs turns it on.
    void SetCrossCheckForTesting(bool enabled);
    bool GetCrossCheckForTesting() const { return _program.crossCheck; }
    // Values the cross-check compared since Open, over the Executes that
    // succeeded, of every kind.
    uint64_t GetCrossCheckCountForTesting() const
    {
        uint64_t total = 0;
        for (uint64_t count : _crossCheckCounts) {
            total += count;
        }
        return total;
    }
    // The same count for one kind.
    uint64_t GetCrossCheckCountForTesting(RrCrossCheckKind kind) const
    {
        return _crossCheckCounts[size_t(kind)];
    }
    // The computed results alone: envelopes, current-phase packets,
    // property values and chain-crossing reads.
    uint64_t GetCrossCheckResultCountForTesting() const
    {
        return _crossCheckCounts[RrCrossCheckEnvelope] +
               _crossCheckCounts[RrCrossCheckPhasePacket] +
               _crossCheckCounts[RrCrossCheckPropertyValue] +
               _crossCheckCounts[RrCrossCheckChainRead];
    }
    uint64_t GetCrossCheckEnvelopeCountForTesting() const
    {
        return _crossCheckCounts[RrCrossCheckEnvelope];
    }
    uint64_t GetCrossCheckPhasePacketCountForTesting() const
    {
        return _crossCheckCounts[RrCrossCheckPhasePacket];
    }
    uint64_t GetCrossCheckPropertyValueCountForTesting() const
    {
        return _crossCheckCounts[RrCrossCheckPropertyValue];
    }
    uint64_t GetCrossCheckChainReadCountForTesting() const
    {
        return _crossCheckCounts[RrCrossCheckChainRead];
    }

    // Test-only: the property chains' published values as the last
    // Execute computed them, in path order. They are inputs to the
    // program, not outputs of the runtime contract.
    std::vector<RigExecRuntimePropertyValue> GetPropertyResultsForTesting()
        const;

    // Final asset-space skinning deltas (rest -> posed), in path order.
    const std::vector<RigExecRuntimeJointMatrix> &GetJointMatrices() const
    {
        return _jointMatrices;
    }

    // GetJointMatrices preserves the skinning delta (rest -> posed) API.
    // Skeleton consumers need the asset-space rest and posed frames.
    std::vector<RigExecRuntimeJointMatrix> GetJointRestMatrices() const;
    std::vector<RigExecRuntimeJointMatrix> GetJointPoseMatrices() const;

    // Deformed points per moved property, in path order.
    const std::vector<RigExecRuntimePoints> &GetPoints() const
    {
        return _points;
    }

    // Matrix primvars a surface projector published, in path order.
    const std::vector<RigExecRuntimeMatrixPrimvar> &GetMatrixPrimvars() const
    {
        return _matrixPrimvars;
    }

    // Volume weight placements, in path order.
    const std::vector<RigExecRuntimeWeightFrame> &GetWeightFrames() const
    {
        return _weightFrames;
    }

    // Resolved weight fields, in path order.
    const std::vector<RigExecRuntimeWeightField> &GetWeightFields() const
    {
        return _weightFields;
    }

    // Constraint-driven provider transforms, in path order.
    const std::vector<RigExecRuntimeProviderXform> &GetProviderXforms()
        const
    {
        return _providerXforms;
    }

    // The generation's diagnostics, in publication order.
    const std::vector<std::string> &GetDiagnostics() const
    {
        return _diagnostics;
    }

    RigExecRuntimeCounters GetCounters() const { return _counters; }

    // Family outputs for the parity tests: the SSA version pools, the
    // rest -> pose matrices, and the weight packets.
    const std::vector<RrPointFrame> &GetFinFrames() const;
    const std::vector<RrPointFrame> &GetBaseFrames() const;
    const std::vector<RrMat4d> &GetFinalMatrices() const;
    const std::vector<RrMat4d> &GetBaseMatrices() const;
    const std::vector<RrWeightPacket> &GetWeightPackets() const;

    // Section presence, for loader tests and --verify-binary diagnostics.
    bool HasSteps() const { return _hasSteps; }
    bool HasPoses() const { return _hasPoses; }
    bool HasGeometry() const { return _hasGeometry; }
    bool HasInputTable() const { return _hasInputTable; }
    bool HasCones() const { return _hasCones; }
    bool HasClusters() const { return _hasClusters; }
    bool HasSlotMeta() const { return _hasSlotMeta; }
    bool HasConstants() const { return _hasConstants; }

private:
    RigExecRuntimeReader() = default;

    std::unique_ptr<RigExecBinaryReader> _reader;
    std::vector<RigExecWireStep> _steps;
    RigExecWireClustering _clustering;
    RigExecWireCones _cones;
    RigExecWireSlotMeta _slotMeta;
    RigExecWireConstants _constants;
    RigExecWireDomainPose _poses;
    RigExecWireDomainGeometry _geometry;
    RigExecWireInputTable _inputs;
    RigExecWireExternalMovers _external;
    // The Computed section (temporary): the input slots every read
    // evaluates over.
    RigExecWireComputed _computed;
    bool _hasSteps = false;
    bool _hasPoses = false;
    bool _hasGeometry = false;
    bool _hasInputTable = false;
    bool _hasCones = false;
    bool _hasClusters = false;
    bool _hasSlotMeta = false;
    bool _hasConstants = false;

    RrProgram _program;

    size_t _frameIndex = 0;
    bool _frameSelected = false;
    // The standing avar overrides, by input slot.
    std::map<uint32_t, double> _avarOverrides;

    std::vector<RigExecRuntimeJointMatrix> _jointMatrices;
    std::vector<RigExecRuntimePoints> _points;
    std::vector<RigExecRuntimeMatrixPrimvar> _matrixPrimvars;
    std::vector<RigExecRuntimeWeightFrame> _weightFrames;
    std::vector<RigExecRuntimeWeightField> _weightFields;
    std::vector<RigExecRuntimeProviderXform> _providerXforms;
    std::vector<std::string> _diagnostics;
    RigExecRuntimeCounters _counters;
    RrCrossCheckCounts _crossCheckCounts{};
};

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_H

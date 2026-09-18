//
// rigExecRuntime: zero-USD .rigexec playback (M2).
//
// Opens a baked .rigexec file, replays its cluster DAG per frame, and
// publishes joint matrices and deformed points -- no USD headers, no USD
// library. The wire structs and decoders come from rigExecBinary (already
// USD-free); the math is runtimeMath.h (a bit-identical Gf mirror); the
// step bodies are ports of the baked program's, one family per .cpp.
//
// Fidelity rule (plan section 5): for the same frame, floating-point
// results must be bit-identical to the baked path. rigExecPose
// --verify-binary gates every family on dynamic==baked==binary over the
// shipped examples.
//
// Threading (D2): Execute is serial over clusters; the consumer may run
// clusters in parallel when the cluster DAG allows (the OpenUSD side
// keeps its dispatcher, Godot uses WorkerThreadPool). The reader holds
// no locks: one reader per thread, or external synchronization.
//

#ifndef RIGEXEC_RUNTIME_H
#define RIGEXEC_RUNTIME_H

#include "rigExecBinary/container.h"
#include "rigExecBinary/geometry.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/pose.h"
#include "rigExecBinary/program.h"
#include "rigExecRuntime/runtimeMath.h"
#include "rigExecRuntime/store.h"

#include <cstdint>
#include <memory>
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
    // close (versions, uid routing, cone sizes).
    static std::unique_ptr<RigExecRuntimeReader> Open(
        const uint8_t *bytes, size_t size, std::string *error);

    // Baked frame times, in file order.
    std::vector<double> GetFrameTimes() const;

    // Selects a baked frame's inputs by exact time. False naming the
    // miss when the file carries no such frame.
    bool SetFrame(double frame, std::string *error);

    // Replays the cluster DAG for the selected frame. False naming the
    // first failing step; outputs keep their previous frame.
    bool Execute(std::string *error);

    // Test-only family mask (bit 0 pose, 1 weights, 2 geometry, all set
    // by default). A masked family's steps are skipped, which is how
    // one family's outputs are compared while another is still landing.
    void SetRunMaskForTesting(unsigned mask);

    // Final asset-space joint matrices, in path order.
    const std::vector<RigExecRuntimeJointMatrix> &GetJointMatrices() const
    {
        return _jointMatrices;
    }

    // Deformed points per moved property, in path order.
    const std::vector<RigExecRuntimePoints> &GetPoints() const
    {
        return _points;
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

    std::vector<RigExecRuntimeJointMatrix> _jointMatrices;
    std::vector<RigExecRuntimePoints> _points;
    std::vector<RigExecRuntimeWeightFrame> _weightFrames;
    std::vector<RigExecRuntimeWeightField> _weightFields;
    std::vector<RigExecRuntimeProviderXform> _providerXforms;
    std::vector<std::string> _diagnostics;
    RigExecRuntimeCounters _counters;
};

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_H

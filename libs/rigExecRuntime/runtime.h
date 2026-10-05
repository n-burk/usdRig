// rigExecRuntime: zero-USD .rigexec playback (M2).
// Opens a baked .rigexec file (one FlatBuffer, rigExecBinary/format.h),
// replays its cluster DAG over the inputs the caller sets, and publishes
// joint matrices and deformed points -- no USD headers, no USD library. The
// opened program is private to the library (store.h); the math is
// runtimeMath.h (a bit-identical Gf mirror); the step bodies are ports of
// the baked program's, one family per .cpp.
// Fidelity rule: for the same input values, floating-point results must be
// bit-identical to the baked path with those values authored. rigExecPose
// --verify-binary gates every family on dynamic==baked==binary over the
// shipped examples.
// Threading (D2): Execute is serial over clusters; the consumer may run
// clusters in parallel when the cluster DAG allows (the OpenUSD side
// keeps its dispatcher, Godot uses WorkerThreadPool). The reader holds
// no locks: one reader per thread, or external synchronization.
#ifndef RIGEXEC_RUNTIME_H
#define RIGEXEC_RUNTIME_H

#include "rigExecBinary/external.h"
#include "rigExecRuntime/runtimeMath.h"
#include "rigExecRuntime/values.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

// The opened program and its working state (store.h).
struct RrProgram;

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

// Plays a .rigexec file: a static graph whose inputs are the attributes the
// rig reads. Open decodes the file, and the inputs start at their
// bake-time defaults; each input set takes effect as an authored value at
// the next Execute; Execute replays the cluster DAG over the inputs; the
// getters publish its outputs. A fresh reader executes its defaults, which
// reproduce the rig at the bake time. Any failure returns false with a
// reason; the reader keeps its last good state.
class RigExecRuntimeReader
{
public:
    ~RigExecRuntimeReader();

    RigExecRuntimeReader(const RigExecRuntimeReader &) = delete;
    RigExecRuntimeReader &operator=(const RigExecRuntimeReader &) = delete;

    // Opens a .rigexec image. False with the reason on a file
    // RigExecFormatOpen refuses (another format, a malformed buffer, or a
    // rule of the format broken, the step graph's among them), or on
    // tables whose cross-references do not close (versions, input reads,
    // cone sizes).
    static std::unique_ptr<RigExecRuntimeReader> Open(
        const uint8_t *bytes, size_t size, std::string *error);

    // The inputs: every attribute a read of the rig walks, sorted by path.
    size_t GetInputCount() const;
    // Input \p index's name, type, animation flag and bake-time default;
    // an empty info past the count.
    const RigExecRuntimeInputInfo &GetInputInfo(size_t index) const;
    // The index of the input at attribute path \p name.
    bool FindInput(const std::string &name, size_t *index) const;
    // Input \p index's value: its default, or the value last set; a
    // Double zero past the count.
    const RrInputValue &GetInputValue(size_t index) const;
    // The authored value of attribute \p name becomes \p value: Execute
    // then evaluates as the rig would with that value authored. On a
    // property chain's target it is the chain's base, which the chain's
    // movers revise; on an attribute connected upstream it takes effect
    // only where the connection walk falls back to it. False with the
    // reason, and nothing changes, for an unknown name, a value of another
    // type (a Double sets a Float input through static_cast<float>), a
    // non-finite component, or a Token id that names no text.
    // One gap against the USD evaluators remains: a plugin mover applies
    // the payload its bake assembled, so an input it reads does not reach
    // that mover's output.
    bool SetInput(const std::string &name, const RrInputValue &value,
                  std::string *error);
    // A Double input, or a Float input through static_cast<float>.
    bool SetInput(const std::string &name, double value, std::string *error);
    // A Token input: text the file holds sets its id; other text is
    // interned on the reader (GetTokenText resolves it).
    bool SetInputToken(const std::string &name, const std::string &text,
                       std::string *error);
    // SetInput by index.
    bool SetInputAt(size_t index, const RrInputValue &value,
                    std::string *error);
    // The stage's own value of input \p index at a sampled time, for a
    // stage sampler: SetInputAt, except that a non-finite component is
    // taken as the stage holds it (a bake-time default can hold one too);
    // the steps that read it report it as the evaluators do.
    bool SetSampledInputAt(size_t index, const RrInputValue &value,
                           std::string *error);
    // Input \p name holds no value, as an attribute whose typed read fails
    // (a blocked sample, or Default on an attribute keyed alone): every
    // read falls back as it does over such an attribute. False with the
    // reason for an unknown name.
    bool ClearInput(const std::string &name, std::string *error);
    // ClearInput by index.
    bool ClearInputAt(size_t index, std::string *error);
    // Input \p name returns to its bake-time default.
    bool ResetInput(const std::string &name, std::string *error);
    void ResetInputs();
    // Says time moved: the next Execute recomputes what a time change
    // recomputes even where no Animated input took a new value. A stage
    // sampler calls it on every change of time.
    void TouchAnimatedInputs();
    // The time the inputs' defaults were read at.
    double GetBakeTime() const;
    // The text of Token value \p token: a token the file holds, or text
    // SetInputToken interned; empty for an unknown id.
    std::string GetTokenText(uint32_t token) const;

    // Replays the cluster DAG over the inputs. False naming the first
    // failing step; outputs keep their previous values.
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

    // Test-only: the label error text names step \p step by, as the baked
    // program spells it; the step's number past the steps.
    std::string GetStepLabelForTesting(size_t step) const;

    // The property chains' published values (chain targets and phased
    // consumers) as the last Execute computed them, in path order.
    std::vector<RigExecRuntimePropertyValue> GetPropertyValues() const;

    // Final asset-space skinning deltas (rest -> posed), in path order.
    const std::vector<RigExecRuntimeJointMatrix> &GetJointMatrices() const
    {
        return _jointMatrices;
    }

    // GetJointMatrices preserves the skinning delta (rest -> posed) API.
    // Skeleton consumers need the asset-space rest and posed frames. The
    // rest frames are the live ones: an input that recomposes the ladder
    // moves them.
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

private:
    RigExecRuntimeReader();

    std::unique_ptr<RrProgram> _program;

    // What the input getters answer past the count: a Double +0.0, every
    // other member zero.
    RigExecRuntimeInputInfo _noInput;
    RrInputValue _noValue;

    std::vector<RigExecRuntimeJointMatrix> _jointMatrices;
    std::vector<RigExecRuntimePoints> _points;
    std::vector<RigExecRuntimeMatrixPrimvar> _matrixPrimvars;
    std::vector<RigExecRuntimeWeightFrame> _weightFrames;
    std::vector<RigExecRuntimeWeightField> _weightFields;
    std::vector<RigExecRuntimeProviderXform> _providerXforms;
    std::vector<std::string> _diagnostics;
    RigExecRuntimeCounters _counters;
};

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_H

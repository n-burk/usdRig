#ifndef RIGEXEC_INPUT_REPLAY_H
#define RIGEXEC_INPUT_REPLAY_H

#include "observerHost.h"
#include "types.h"
#include "pxr/usd/usd/stage.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {
struct RigExecValueOverride;
struct RigExecRigPose;
class RigExecBakedProgram;
class RigExecRigEvaluator;
class RigExecHeldProgramGoldenObserver;
class RigExecInputReplayHeldProgram;

/// One explicit caller Compile action; passive result metadata stays separate.
struct RigExecInputReplayCompileCall {
    bool recorded = false, goldenEnabled = false;
    unsigned owner = 0;
    uint64_t ordinal = 0;
    const std::vector<std::string> *diagnostics = nullptr;
};

/// Opt-in source-input recording. This observer never receives pose outputs.
/// RIGEXEC_INPUT_REPLAY=capture:<new file> enables a serial owning-thread log.
class RigExecInputReplayObserver {
public:
    static std::unique_ptr<RigExecInputReplayObserver> Create(
        const UsdStageRefPtr &, const SdfPath &);
    /// Refuse unrecorded public entry points outside a captured evaluator body.
    static void UnsupportedAction(const char *action);
    /// Record actual process-global caller array-admission state before mutation.
    static void ArrayAdmission(bool on);
    ~RigExecInputReplayObserver();
    RigExecInputReplayCompileCall Compile(const std::vector<std::string> *);
    void CompileResult(const RigExecInputReplayCompileCall &, bool,
                       const std::vector<std::string> *);
    void Interactive(const std::vector<RigExecValueOverride> &);
    void ClearInteractive();
    void Upstream(const std::vector<RigExecValueOverride> &);
    void Options(bool solverGuides, bool publishWeightFields);
    void BeginEvaluate(UsdTimeCode);
    void EndEvaluate();
private:
    RigExecInputReplayObserver(unsigned stage, unsigned evaluator, const SdfPath &rig);
    unsigned _stage = 0, _evaluator = 0;
    SdfPath _rig;
    bool _evaluating = false, _compilePending = false;
    uint64_t _compileCalls = 0;
    RigExecInputReplayCompileCall _pendingCompile;
    friend class RigExecInputReplayHeldProgram;
};

/// Test-owned retained public program. Source actions contain no pose outputs.
/// Its router must close first, then the program, then its evaluator.
/// Only explicit Build/Run/router actions represented in the protocol are supported;
/// raw program access is for graph inspection and the existing notice router,
/// never unrecorded overrides, options, requests or other mutating entry points.
class RigExecInputReplayHeldProgram {
public:
    static std::unique_ptr<RigExecInputReplayHeldProgram> Build(
        RigExecRigEvaluator *, std::vector<std::string> *reasons);
    ~RigExecInputReplayHeldProgram();
    RigExecBakedProgram *get() const { return _program.get(); }
    RigExecBakedProgram *operator->() const { return get(); }
    RigExecBakedProgram &operator*() const { return *get(); }
    explicit operator bool() const { return bool(_program); }
    /// Always initializes fresh output; requestedTimeSeed describes caller policy.
    bool Run(UsdTimeCode, RigExecRigPose *, bool requestedTimeSeed);
    /// Observe activation of the actual caller's ApplyValueEdits/false->stamp router.
    static void Router(RigExecBakedProgram *, bool enabled);
private:
    RigExecInputReplayHeldProgram(RigExecRigEvaluator *, unsigned owner,
                                  unsigned program, bool recording);
    std::unique_ptr<RigExecBakedProgram> _program;
    std::unique_ptr<RigExecHeldProgramGoldenObserver> _golden;
    unsigned _owner, _id;
    bool _recording, _router = false;
};

/// Native-history harnesses execute comparison branches unchanged, while
/// recording their nonauthoring boundaries. Direct mathematical runs require
/// a separately declared native-equivalent adapter.
class RigExecInputReplayComparisonScope {
public:
    explicit RigExecInputReplayComparisonScope(const char *action,
                                               bool nativeEquivalent = false);
    ~RigExecInputReplayComparisonScope();
    RigExecInputReplayComparisonScope(const RigExecInputReplayComparisonScope &) = delete;
    RigExecInputReplayComparisonScope &operator=(const RigExecInputReplayComparisonScope &) = delete;
private:
    bool _enabled = false;
    uint64_t _ordinal = 0;
};

/// Exact caller-supplied ImportFromString action; no generated numeric text.
bool RigExecInputReplayImportFromString(const SdfLayerHandle &,
                                      const std::string &);

/// Exact known source replacement actions; SDK methods execute unchanged.
void RigExecInputReplayClearLayer(const SdfLayerHandle &);
void RigExecInputReplayTransferLayerContent(const SdfLayerHandle &target,
                                           const SdfLayerHandle &source);

/// Public Compile can invoke internal frozen helpers without adding actions.
/// The caller enables this scope only for a captured evaluator implementation.
class RigExecInputReplayImplementationScope {
public:
    explicit RigExecInputReplayImplementationScope(bool enabled);
    ~RigExecInputReplayImplementationScope();
    RigExecInputReplayImplementationScope(const RigExecInputReplayImplementationScope &) = delete;
    RigExecInputReplayImplementationScope &operator=(const RigExecInputReplayImplementationScope &) = delete;
private:
    bool _enabled;
};

/// Internal validation evaluators do not create additional caller histories.
class RigExecInputReplaySuppression {
public:
    RigExecInputReplaySuppression();
    ~RigExecInputReplaySuppression();
    RigExecInputReplaySuppression(const RigExecInputReplaySuppression &) = delete;
    RigExecInputReplaySuppression &operator=(const RigExecInputReplaySuppression &) = delete;
};

/// Replay source actions against the linked evaluator. Expected numerical
/// outputs are produced only by the separately instrumented original host.
bool RigExecReplayInputActions(const std::string &file,
                              std::string *error = nullptr);
}
#endif

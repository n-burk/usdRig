#ifndef RIGEXEC_BAKED_EXEC_CROSS_CHECK_H
#define RIGEXEC_BAKED_EXEC_CROSS_CHECK_H

#include "tapSet.h"
#include <functional>
#include <map>
#include <string>

namespace rigExec {

enum class RigExecExecCheckSkip { None, NoEquivalent, ArrayInput, Unusable };

struct RigExecExecCheckDescriptor {
    std::string key;
    uint32_t kind = 0;
    /// A row with no public equivalent has no request and must explicitly skip.
    bool skipOnly = false;
    RigExecValueAddress address;
    /// A required override names a bound value, never the op's output.
    std::vector<RigExecValueAddress> requiredOverrides;
    /// Optional pure conversion from the public exec result to stored op type.
    std::function<bool(const VtValue &, VtValue *)> project;
    using Reference = std::function<bool(VtValue *, std::string *)>;
    /// Owning-thread acquisition of independent ORIGINAL input context.
    /// The returned numerical witness owns its inputs and performs no stage reads.
    std::function<bool(const UsdStageRefPtr &, UsdTimeCode,
        const std::vector<RigExecValueOverride> &, Reference *, std::string *)> acquireReference;
};
struct RigExecExecCheckInput {
    VtValue expected;
    std::vector<RigExecValueOverride> overrides;
    RigExecExecCheckSkip skip = RigExecExecCheckSkip::None;
};
struct RigExecExecCrossCheckReport {
    size_t checked = 0, failed = 0, dropped = 0;
    std::map<uint32_t, size_t> checkedByKind;
    std::map<RigExecExecCheckSkip, size_t> skipped;
    std::vector<std::string> diagnostics;
    bool Passed() const { return failed == 0 && dropped == 0; }
};

/// Owning-thread verification only. Each request contains exactly one public
/// builtin key or an independent ORIGINAL constraint witness. Inputs are
/// immutable copies of the values the operation read.
/// No network, sampling, or stage access occurs in the production executor.
class RigExecBakedExecCrossCheck {
public:
    explicit RigExecBakedExecCrossCheck(const UsdStageRefPtr &stage) : _stage(stage) {}
    bool Add(RigExecExecCheckDescriptor row, std::string *error = nullptr);
    RigExecExecCrossCheckReport Evaluate(UsdTimeCode time,
        const std::vector<RigExecExecCheckInput> &inputs);
    size_t Size() const { return _rows.size(); }
private:
    struct Row {
        RigExecExecCheckDescriptor descriptor;
        std::unique_ptr<RigExecTapSet> tap;
    };
    UsdStageRefPtr _stage;
    std::vector<Row> _rows;
};
}
#endif

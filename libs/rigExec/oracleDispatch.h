// Function pointers the test oracle library installs. Evaluator translation
// units call the public functions below; those forward here. A missing hook
// aborts, so a cpu-reference or golden run cannot succeed without the library.
#ifndef RIGEXEC_ORACLE_DISPATCH_H
#define RIGEXEC_ORACLE_DISPATCH_H

#include "goldenPose.h"
#include "goldenSuite.h"
#include "oracleInputs.h"
#include "weightReference.h"

#include <memory>

namespace rigExec {
struct RigExecBakedProgramImpl;
struct RigExecRigPose;

struct RigExecOracleHooks {
    void (*begin)(RigExecBakedProgramImpl *, uint64_t, UsdTimeCode) = nullptr;
    bool (*run)(RigExecBakedProgramImpl *, UsdTimeCode, RigExecRigPose *) = nullptr;
    RigExecOracleScene (*captureInputs)(
        const UsdStageRefPtr &, const RigExecResolvedInputs &, UsdTimeCode,
        size_t, const std::vector<SdfPath> &,
        const std::map<SdfPath, VtValue> &, RigExecOracleCaptureMode) = nullptr;
    RigExecWeightReferenceContext (*captureWeight)(
        const UsdStageRefPtr &, const SdfPath &, const RigExecResolvedInputs &,
        const std::map<SdfPath, VtValue> &, UsdTimeCode,
        const std::function<const GfMatrix4d *(const SdfPath &)> &) = nullptr;
    std::unique_ptr<RigExecGoldenSuiteObserver> (*createGolden)(const SdfPath &) = nullptr;
    void (*finalizeGolden)() = nullptr;
    bool (*encodeValue)(const VtValue &, std::string *, std::string *) = nullptr;
    bool (*encodePose)(const RigExecRigPose &, std::vector<RigExecGoldenValue> *,
                       std::string *, bool) = nullptr;
    std::string (*goldenDouble)(double) = nullptr;
    std::string (*goldenEscape)(const std::string &) = nullptr;
    uint64_t (*goldenDigest)(const std::string &) = nullptr;
    std::string (*goldenHex)(uint64_t) = nullptr;
    std::string (*goldenVisit)(const std::string &, size_t, const RigExecRigPose &,
                               const std::vector<RigExecGoldenValue> &, bool) = nullptr;
    bool (*compareGolden)(const std::string &, const std::string &, std::string *) = nullptr;
};

void RigExecInstallOracleHooks(const RigExecOracleHooks &hooks);
const RigExecOracleHooks &RigExecOracleInstalledHooks();
[[noreturn]] void RigExecOracleMissing(const char *feature);

RigExecOracleScene RigExecDispatchCaptureOracleInputs(
    const UsdStageRefPtr &stage, const RigExecResolvedInputs &resolved,
    UsdTimeCode time, size_t pointCount, const std::vector<SdfPath> &roots,
    const std::map<SdfPath, VtValue> &upstream, RigExecOracleCaptureMode mode);

RigExecWeightReferenceContext RigExecDispatchCaptureWeightReference(
    const UsdStageRefPtr &stage, const SdfPath &root,
    const RigExecResolvedInputs &resolved, const std::map<SdfPath, VtValue> &upstream,
    UsdTimeCode time, const std::function<const GfMatrix4d *(const SdfPath &)> &placement);
}
#endif

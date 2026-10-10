#include "rigExec/oracleDispatch.h"
#include "rigExec/goldenSuite.h"
#include "rigExec/goldenPose.h"
#include "rigExec/oracleInputs.h"
#include "rigExec/scalarReferenceAdapter.h"
#include "rigExec/weightReference.h"

#include <memory>

namespace rigExec {
void RigExecOracleBeginBody(RigExecBakedProgramImpl *, uint64_t, UsdTimeCode);
bool RigExecOracleRunBody(RigExecBakedProgramImpl *, UsdTimeCode, RigExecRigPose *);
void RigExecOracleFinalizeGolden();
std::unique_ptr<RigExecGoldenSuiteObserver> RigExecOracleCreateGolden(const SdfPath &);
bool RigExecOracleEncodeGoldenValue(const VtValue &, std::string *, std::string *);
bool RigExecOracleEncodeGoldenPose(const RigExecRigPose &, std::vector<RigExecGoldenValue> *,
    std::string *, bool);
std::string RigExecOracleGoldenDouble(double);
std::string RigExecOracleGoldenEscape(const std::string &);
uint64_t RigExecOracleGoldenDigest(const std::string &);
std::string RigExecOracleGoldenHex(uint64_t);
std::string RigExecOracleGoldenVisit(const std::string &, size_t, const RigExecRigPose &,
    const std::vector<RigExecGoldenValue> &, bool);
bool RigExecOracleCompareGolden(const std::string &, const std::string &, std::string *);

namespace {
const bool registered = [] {
    RigExecOracleHooks hooks;
    hooks.begin = &RigExecOracleBeginBody;
    hooks.run = &RigExecOracleRunBody;
    hooks.captureInputs = &RigExecCaptureOracleInputs;
    hooks.captureWeight = &RigExecCaptureWeightReference;
    hooks.createGolden = &RigExecOracleCreateGolden;
    hooks.finalizeGolden = &RigExecOracleFinalizeGolden;
    hooks.encodeValue = &RigExecOracleEncodeGoldenValue;
    hooks.encodePose = &RigExecOracleEncodeGoldenPose;
    hooks.goldenDouble = &RigExecOracleGoldenDouble;
    hooks.goldenEscape = &RigExecOracleGoldenEscape;
    hooks.goldenDigest = &RigExecOracleGoldenDigest;
    hooks.goldenHex = &RigExecOracleGoldenHex;
    hooks.goldenVisit = &RigExecOracleGoldenVisit;
    hooks.compareGolden = &RigExecOracleCompareGolden;
    RigExecInstallOracleHooks(hooks);
    return true;
}();
}

void RigExecOracleLinkAnchor() {}
}

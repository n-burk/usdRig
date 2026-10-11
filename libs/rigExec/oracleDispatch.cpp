#include "oracleDispatch.h"
#include "goldenSuite.h"

#include <cstdio>
#include <cstdlib>

namespace rigExec {
namespace {
RigExecOracleHooks g_hooks;
}

void RigExecInstallOracleHooks(const RigExecOracleHooks &hooks) { g_hooks = hooks; }

const RigExecOracleHooks &RigExecOracleInstalledHooks() { return g_hooks; }

void RigExecOracleMissing(const char *feature)
{
    std::fprintf(stderr,
        "RigExec oracle library is not linked; %s cannot run\n", feature);
    std::fflush(stderr);
    std::_Exit(EXIT_FAILURE);
}

namespace {
template <class Fn>
Fn Hook(Fn fn, const char *feature)
{
    if (!fn) RigExecOracleMissing(feature);
    return fn;
}
}

RigExecOracleScene RigExecDispatchCaptureOracleInputs(
    const UsdStageRefPtr &stage, const RigExecResolvedInputs &resolved,
    UsdTimeCode time, size_t pointCount, const std::vector<SdfPath> &roots,
    const std::map<SdfPath, VtValue> &upstream, RigExecOracleCaptureMode mode)
{
    return Hook(g_hooks.captureInputs, "cpu reference")(
        stage, resolved, time, pointCount, roots, upstream, mode);
}

RigExecWeightReferenceContext RigExecDispatchCaptureWeightReference(
    const UsdStageRefPtr &stage, const SdfPath &root,
    const RigExecResolvedInputs &resolved, const std::map<SdfPath, VtValue> &upstream,
    UsdTimeCode time, const std::function<const GfMatrix4d *(const SdfPath &)> &placement)
{
    return Hook(g_hooks.captureWeight, "cpu reference")(
        stage, root, resolved, upstream, time, placement);
}

bool RigExecEncodeGoldenValue(const VtValue &value, std::string *encoded, std::string *error)
{
    return Hook(g_hooks.encodeValue, "golden pose")(value, encoded, error);
}

bool RigExecEncodeGoldenPose(const RigExecRigPose &pose,
    std::vector<RigExecGoldenValue> *values, std::string *error, bool includeOracle)
{
    return Hook(g_hooks.encodePose, "golden pose")(pose, values, error, includeOracle);
}

std::string RigExecGoldenDouble(double value)
{
    return Hook(g_hooks.goldenDouble, "golden pose")(value);
}

std::string RigExecGoldenEscape(const std::string &value)
{
    return Hook(g_hooks.goldenEscape, "golden pose")(value);
}

uint64_t RigExecGoldenDigest(const std::string &bytes)
{
    return Hook(g_hooks.goldenDigest, "golden pose")(bytes);
}

std::string RigExecGoldenHex(uint64_t value)
{
    return Hook(g_hooks.goldenHex, "golden pose")(value);
}

std::string RigExecGoldenVisit(const std::string &leg, size_t ordinal,
    const RigExecRigPose &pose, const std::vector<RigExecGoldenValue> &values,
    bool digestDomains)
{
    return Hook(g_hooks.goldenVisit, "golden pose")(
        leg, ordinal, pose, values, digestDomains);
}

bool RigExecCompareGolden(const std::string &expected, const std::string &actual,
    std::string *error)
{
    return Hook(g_hooks.compareGolden, "golden pose")(expected, actual, error);
}

std::unique_ptr<RigExecGoldenSuiteObserver>
RigExecGoldenSuiteObserver::Create(const SdfPath &rigPath)
{
    if (!g_hooks.createGolden) {
        const char *value = std::getenv("RIGEXEC_GOLDEN_SUITE");
        if (value && *value) RigExecOracleMissing("golden suite");
        return nullptr;
    }
    return g_hooks.createGolden(rigPath);
}

void RigExecFinalizeGoldenSuite()
{
    if (g_hooks.finalizeGolden) {
        g_hooks.finalizeGolden();
        return;
    }
    const char *value = std::getenv("RIGEXEC_GOLDEN_SUITE");
    if (value && *value) RigExecOracleMissing("golden suite");
}
}

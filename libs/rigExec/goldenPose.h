#ifndef RIGEXEC_GOLDEN_POSE_H
#define RIGEXEC_GOLDEN_POSE_H

#include "rigEvaluator.h"
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

/// A typed published value. The encoding retains every floating-point bit.
struct RigExecGoldenValue {
    std::string domain, key, value;
};

/// Encode one typed value for exact independent operation checks.
bool RigExecEncodeGoldenValue(const VtValue &value, std::string *encoded,
    std::string *error = nullptr);

/// Encode published values and ordered diagnostics, excluding work counters.
/// Unknown VtValue types fail rather than becoming a printable approximation.
bool RigExecEncodeGoldenPose(const RigExecRigPose &pose,
    std::vector<RigExecGoldenValue> *values, std::string *error = nullptr,
    bool includeOracle = false);

/// Stable hexadecimal encodings used in golden keys and protocol metadata.
std::string RigExecGoldenDouble(double value);
std::string RigExecGoldenEscape(const std::string &value);
uint64_t RigExecGoldenDigest(const std::string &bytes);
std::string RigExecGoldenHex(uint64_t value);

/// Write one explicit protocol visit, with no hidden warmup or benchmark.
std::string RigExecGoldenVisit(const std::string &leg, size_t ordinal,
    const RigExecRigPose &pose, const std::vector<RigExecGoldenValue> &values,
    bool digestDomains = false);

/// Exact store comparison; reports the first differing protocol/value row.
bool RigExecCompareGolden(const std::string &expected,
    const std::string &actual, std::string *error = nullptr);

} // namespace rigExec
#endif

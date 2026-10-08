#include "bakedExecCrossCheck.h"
#include "goldenPose.h"
#include <algorithm>
#include <set>
#include <tuple>

namespace rigExec {
namespace {
using Key = std::tuple<SdfPath, TfToken, TfToken>;
Key OverrideKey(const RigExecValueOverride &value)
{
    return {value.prim, value.attribute.IsEmpty() ? value.computation : TfToken(), value.attribute};
}
Key AddressKey(const RigExecValueAddress &address)
{
    return address.target.IsPropertyPath()
        ? Key{address.target.GetPrimPath(), TfToken(), address.target.GetNameToken()}
        : Key{address.target, address.publicComputation, TfToken()};
}
}

bool RigExecBakedExecCrossCheck::Add(RigExecExecCheckDescriptor descriptor, std::string *error)
{
    const auto fail = [&](const std::string &message) { if (error) *error = message; return false; };
    if (!_stage || descriptor.key.empty() ||
        (!descriptor.skipOnly && descriptor.address.target.IsEmpty()))
        return fail("invalid exec check descriptor");
    if (std::any_of(_rows.begin(), _rows.end(), [&](const Row &row) {
            return row.descriptor.key == descriptor.key;
        })) return fail("duplicate exec check key: " + descriptor.key);
    if (descriptor.skipOnly) {
        if (!descriptor.requiredOverrides.empty() || descriptor.project || descriptor.acquireReference)
            return fail("skip-only exec check cannot declare a computation: " + descriptor.key);
        _rows.push_back({std::move(descriptor), {}});
        return true;
    }
    const Key output = AddressKey(descriptor.address);
    std::set<Key> required;
    for (const auto &address : descriptor.requiredOverrides) {
        if (address.target.IsEmpty() || !required.insert(AddressKey(address)).second)
            return fail("invalid or duplicate required override: " + descriptor.key);
        if (AddressKey(address) == output)
            return fail("exec check cannot override its own result: " + descriptor.key);
    }
    if (descriptor.acquireReference) {
        if (descriptor.project) return fail("independent reference cannot project a public result: " + descriptor.key);
        _rows.push_back({std::move(descriptor), {}});
        return true;
    }
    auto tap = std::make_unique<RigExecTapSet>(_stage);
    tap->Add(descriptor.address);
    if (!tap->Prepare()) return fail("cannot prepare exec check: " + descriptor.key);
    _rows.push_back({std::move(descriptor), std::move(tap)});
    return true;
}

RigExecExecCrossCheckReport RigExecBakedExecCrossCheck::Evaluate(
    UsdTimeCode time, const std::vector<RigExecExecCheckInput> &inputs)
{
    RigExecExecCrossCheckReport report;
    if (inputs.size() != _rows.size()) {
        ++report.failed;
        report.diagnostics.push_back("exec check input count differs from compiled row count");
        return report;
    }
    for (size_t i = 0; i < _rows.size(); ++i) {
        auto &row = _rows[i]; const auto &input = inputs[i];
        if (input.skip != RigExecExecCheckSkip::None) { ++report.skipped[input.skip]; continue; }
        const auto fail = [&](const std::string &reason) {
            ++report.failed; report.diagnostics.push_back(row.descriptor.key + ": " + reason);
        };
        if (row.descriptor.skipOnly) { fail("skip-only row requires an explicit reason"); continue; }
        const Key output = AddressKey(row.descriptor.address);
        std::set<Key> present;
        bool valid = true;
        for (const auto &override : input.overrides) {
            const Key key = OverrideKey(override);
            if (key == output || !present.insert(key).second) {
                fail("duplicate override or attempted result override"); valid = false; break;
            }
        }
        if (!valid) continue;
        for (const auto &address : row.descriptor.requiredOverrides)
            if (!present.count(AddressKey(address))) {
                fail("missing bound input override: " + address.target.GetString()); valid = false; break;
            }
        if (!valid) continue;
        VtValue actual;
        if (row.descriptor.acquireReference) {
            if (present.size()!=row.descriptor.requiredOverrides.size()) {
                fail("independent reference received an undeclared input override"); continue;
            }
            RigExecExecCheckDescriptor::Reference reference;
            std::string referenceError;
            if (!row.descriptor.acquireReference(_stage,time,input.overrides,&reference,&referenceError) || !reference) {
                fail("independent context acquisition failed: " + referenceError); continue;
            }
            if (!reference(&actual,&referenceError)) {
                fail("independent numerical witness failed: " + referenceError); continue;
            }
        } else {
            size_t dropped = 0;
            const auto snapshot = row.tap->Evaluate(time, input.overrides, &dropped);
            report.dropped += dropped;
            if (dropped) { fail("dropped " + std::to_string(dropped) + " override addresses"); continue; }
            if (!snapshot.IsComplete()) { fail("exec did not produce a value"); continue; }
            actual = snapshot.Get(0);
        }
        if (row.descriptor.project) {
            VtValue projected;
            if (!row.descriptor.project(actual, &projected)) { fail("result conversion failed"); continue; }
            actual = std::move(projected);
        }
        std::string expectedText, actualText, error;
        if (!RigExecEncodeGoldenValue(input.expected, &expectedText, &error) ||
            !RigExecEncodeGoldenValue(actual, &actualText, &error)) { fail(error); continue; }
        ++report.checked; ++report.checkedByKind[row.descriptor.kind];
        if (actualText != expectedText)
            fail("value differs\nexpected: " + expectedText + "\nactual: " + actualText);
    }
    return report;
}
}

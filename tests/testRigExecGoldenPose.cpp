#include "rigExec/goldenPose.h"
#include "pxr/base/vt/array.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

#define CHECK(expression) do { if (!(expression)) { \
    std::cerr << "failed: " << #expression << '\n'; return 1; } } while (false)

namespace {
double DoubleBits(uint64_t bits)
{
    double value;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

bool Encode(const RigExecRigPose &pose, std::string *text, bool compact = false)
{
    std::vector<RigExecGoldenValue> values;
    std::string error;
    if (!RigExecEncodeGoldenPose(pose, &values, &error)) {
        std::cerr << error << '\n';
        return false;
    }
    *text = RigExecGoldenVisit("first", 0, pose, values, compact);
    return true;
}
}

int main()
{
    RigExecRigPose pose;
    pose.valid = true;
    pose.time = UsdTimeCode(1.0);
    pose.movedProperties[SdfPath("/Rig/B.value")] = VtValue(1.0);
    pose.movedProperties[SdfPath("/Rig/A.value")] = VtValue(0.0);
    pose.diagnostics = {"first\nline", "second \"quoted\"\\line"};
    std::string original, changed, compact;
    CHECK(Encode(pose, &original));
    CHECK(original.find("time=3ff0000000000000") != std::string::npos);
    CHECK(original.find("/Rig/A.value") < original.find("/Rig/B.value"));
    CHECK(original.find("first\\nline") != std::string::npos);
    CHECK(original.find("second \\\"quoted\\\"\\\\line") != std::string::npos);

    // Only the exact retired scheduler summary is excluded.
    RigExecRigPose summary = pose;
    summary.diagnostics.insert(summary.diagnostics.begin() + 1,
        "mover graph: 2 chain(s), 3 revision(s); 1 created, 3 executed, 2 schedule(s) built");
    CHECK(Encode(summary, &changed));
    CHECK(RigExecCompareGolden(original, changed));
    summary.diagnostics[1] += "; failed";
    CHECK(Encode(summary, &changed));
    CHECK(!RigExecCompareGolden(original, changed));

    // One representable step must fail exact comparison and identify the value row.
    RigExecRigPose mutated = pose;
    mutated.movedProperties[SdfPath("/Rig/B.value")] =
        VtValue(std::nextafter(1.0, std::numeric_limits<double>::infinity()));
    CHECK(Encode(mutated, &changed));
    std::string error;
    CHECK(!RigExecCompareGolden(original, changed, &error));
    CHECK(error.find("/Rig/B.value") != std::string::npos);

    // Signed zero and type identity are part of the reference contract.
    mutated = pose;
    mutated.movedProperties[SdfPath("/Rig/A.value")] = VtValue(-0.0);
    CHECK(Encode(mutated, &changed));
    CHECK(changed.find("double 8000000000000000") != std::string::npos);
    CHECK(!RigExecCompareGolden(original, changed));
    mutated.movedProperties[SdfPath("/Rig/A.value")] = VtValue(0.0f);
    CHECK(Encode(mutated, &changed));
    CHECK(!RigExecCompareGolden(original, changed));

    // Neither NaN payloads nor an array's cardinality are normalized away.
    pose.movedProperties[SdfPath("/Rig/NaN.value")] =
        VtValue(DoubleBits(UINT64_C(0x7ff8000000000001)));
    CHECK(Encode(pose, &original));
    CHECK(original.find("7ff8000000000001") != std::string::npos);
    mutated = pose;
    mutated.movedProperties[SdfPath("/Rig/NaN.value")] =
        VtValue(DoubleBits(UINT64_C(0x7ff8000000000002)));
    CHECK(Encode(mutated, &changed));
    CHECK(!RigExecCompareGolden(original, changed));
    pose.movedProperties[SdfPath("/Rig/Mesh.points")] = VtValue(VtFloatArray{0.0f, -0.0f});
    CHECK(Encode(pose, &original));
    CHECK(original.find("float[2] 00000000 80000000") != std::string::npos);
    CHECK(Encode(pose, &compact, true));
    mutated = pose;
    mutated.movedProperties[SdfPath("/Rig/Mesh.points")] = VtValue(VtFloatArray{0.0f});
    CHECK(Encode(mutated, &changed, true));
    CHECK(!RigExecCompareGolden(compact, changed));

    mutated = pose;
    std::swap(mutated.diagnostics[0], mutated.diagnostics[1]);
    CHECK(Encode(mutated, &changed));
    CHECK(!RigExecCompareGolden(original, changed));
    mutated = pose;
    mutated.valid = false;
    CHECK(Encode(mutated, &changed));
    CHECK(!RigExecCompareGolden(original, changed));
    pose.time = UsdTimeCode::Default();
    CHECK(Encode(pose, &changed));
    CHECK(changed.find("time=default") != std::string::npos);
    CHECK(!RigExecCompareGolden(original, changed));

    // Unsupported values must stop capture rather than stringify approximately.
    pose.movedProperties[SdfPath("/Rig/Unsupported.value")] = VtValue(VtStringArray{"value"});
    std::vector<RigExecGoldenValue> values;
    CHECK(!RigExecEncodeGoldenPose(pose, &values, &error));
    CHECK(error.find("unsupported golden value type") != std::string::npos);
    CHECK(RigExecCompareGolden("same\n", "same\n"));
    CHECK(!RigExecCompareGolden("prefix", "prefix\n", &error));
    CHECK(error.find("row 1") != std::string::npos);
    CHECK(RigExecGoldenHex(RigExecGoldenDigest("")) == "cbf29ce484222325");
    return 0;
}

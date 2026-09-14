//
// The per-point geometry kernels run over point ranges on several threads.
// "Bit-identical" is the claim that makes that safe, and it has two halves,
// both asserted here:
//
//  - splitting the range changes nothing. A point range is an independent
//    sub-layout -- point i reads slots i * elementSize and writes index i --
//    so the parallel result must equal the scalar reference computed one
//    point at a time, exactly, not to a tolerance.
//  - running it on one thread changes nothing either. The same binary is
//    driven at the maximum concurrency limit and at a limit of one, and the
//    two results compared bit for bit; ctest also runs this whole suite a
//    second time with RIGEXEC_ENABLE_PARALLEL_EVAL=0, which takes the other
//    branch entirely.
//
// The meshes here are deliberately larger than RigExecGeometryParallelThreshold
// (and the small one deliberately smaller), because a threshold that is never
// crossed in a test is a threshold that is never tested.
//
#include "rigExec/moverGraph.h"
#include "rigExec/parallel.h"
#include "rigExec/types.h"

#include "rigExecMath/envelope.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/work/threadLimits.h"
#include "pxr/exec/exec/typeRegistry.h"
#include "pxr/usd/sdf/path.h"

#include <cstdio>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

using namespace rigExec;

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

namespace {

const SdfPath kTarget("/Mesh.points");

std::vector<GfMatrix4d>
MakeInfluences()
{
    // Rotation and scale as well as translation: a pure translation blends
    // to the same answer under almost any grouping, which would hide a
    // difference that a real rig would show.
    GfMatrix4d a(1.0), b(1.0);
    a.SetTranslate(GfVec3d(3.5, -1.25, 0.75));
    b = GfMatrix4d(GfRotation(GfVec3d(0.3, 0.5, 0.8).GetNormalized(), 37.0),
                   GfVec3d(-2.0, 4.0, 1.5));
    GfMatrix4d c(1.0);
    c.SetScale(GfVec3d(1.3, 0.7, 2.1));
    c.SetTranslateOnly(GfVec3d(0.125, 0.25, -0.5));
    return {a, b, c};
}

VtVec3fArray
MakeBase(size_t count)
{
    VtVec3fArray points(count);
    for (size_t i = 0; i < count; ++i) {
        const float f = float(i);
        points[i] = GfVec3f(f * 0.013f, std::sin(f * 0.011f) * 7.0f,
                            std::cos(f * 0.017f) * 3.0f);
    }
    return points;
}

RigExecMoverParameters
MakeSkinParams(size_t count, size_t elementSize,
               const std::vector<GfMatrix4d> &influences)
{
    RigExecMoverParameters params;
    params.kind = TfToken("skin");
    params.enabled = true;
    params.valid = true;
    params.skinningMethod = TfToken("classicLinear");
    params.skinTransforms = influences;
    params.skinElementSize = int(elementSize);
    params.skinIndices.resize(count * elementSize);
    params.skinWeights.resize(count * elementSize);
    for (size_t i = 0; i < count; ++i) {
        for (size_t k = 0; k < elementSize; ++k) {
            params.skinIndices[i * elementSize + k] =
                int((i + k) % influences.size());
            // Deliberately not normalised: the weight complement is part of
            // the kernel's rule, so a split that got the layout wrong would
            // show up in the rest-retaining term too.
            params.skinWeights[i * elementSize + k] =
                float(((i * 7 + k * 3) % 11)) / 20.0f;
        }
    }
    params.weights = RigExecWeightPacket::Constant(1.0f);
    return params;
}

RigExecMoverStatus
OkStatus()
{
    RigExecMoverStatus status;
    status.state = TfToken("ok");
    return status;
}

RigExecSkinLayout
LayoutOf(const VtVec3fArray &base, const RigExecMoverParameters &params)
{
    RigExecSkinLayout layout;
    layout.transforms = params.skinTransforms.data();
    layout.transformCount = params.skinTransforms.size();
    layout.indices = params.skinIndices.data();
    layout.weights = params.skinWeights.data();
    layout.indexCount = params.skinIndices.size();
    layout.elementSize = size_t(params.skinElementSize);
    layout.pointCount = base.size();
    return layout;
}

// The unsplit reference: ONE call over the whole point range, through the
// same method dispatch the kernel makes. That is what "splitting the range
// changes nothing" means, stated as something comparable bit for bit --
// comparing against the double-precision per-point kernel instead would be
// comparing two different arithmetics wherever the SIMD path is compiled in.
VtVec3fArray
ReferenceSkin(const VtVec3fArray &base, const RigExecMoverParameters &params)
{
    const RigExecSkinLayout layout = LayoutOf(base, params);
    VtVec3fArray out(base.size());
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    if (useSimd) {
        RigExecApplyLinearBlendSkinSimd(base.cdata(), out.data(), layout);
    } else {
        RigExecApplyLinearBlendSkin(base.cdata(), out.data(), layout);
    }
    return out;
}

// The independent arithmetic: one point at a time, accumulated in double.
// Held to a tolerance rather than to equality, because that is the
// relationship the SIMD kernel is parity-gated on in the first place.
void
CheckAgreesWithScalarReference(
    const VtVec3fArray &result, const VtVec3fArray &base,
    const RigExecMoverParameters &params, const char *what)
{
    const RigExecSkinLayout layout = LayoutOf(base, params);
    for (size_t i = 0; i < base.size(); ++i) {
        const GfVec3d expected =
            RigExecApplyLinearBlendSkin(GfVec3d(base[i]), layout, i);
        if ((GfVec3d(result[i]) - expected).GetLength() > 1e-3) {
            ++failures;
            std::printf("FAIL %s: point %zu is (%g %g %g), scalar reference "
                        "says (%g %g %g)\n", what, i, result[i][0],
                        result[i][1], result[i][2], expected[0], expected[1],
                        expected[2]);
            return;
        }
    }
}

bool
Identical(const VtVec3fArray &a, const VtVec3fArray &b, const char *what)
{
    if (a.size() != b.size()) {
        ++failures;
        std::printf("FAIL %s: sizes %zu != %zu\n", what, a.size(), b.size());
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        // Exact: every component, every bit. Nothing here is allowed to be
        // "close enough".
        if (a[i][0] != b[i][0] || a[i][1] != b[i][1] || a[i][2] != b[i][2]) {
            ++failures;
            std::printf("FAIL %s: point %zu is (%.9g %.9g %.9g), expected "
                        "(%.9g %.9g %.9g)\n", what, i, a[i][0], a[i][1],
                        a[i][2], b[i][0], b[i][1], b[i][2]);
            return false;
        }
    }
    return true;
}

VtVec3fArray
RunSkin(const VtVec3fArray &base, const RigExecMoverParameters &params)
{
    RigExecMoverGraph graph;
    const VdfMaskedOutput source = graph.AddPointSource(kTarget, base);
    const VdfMaskedOutput head = graph.AddRevision(
        RigExecRevisionOp::Skin, source, params, OkStatus());
    const VtVec3fArray out = graph.Evaluate(head);
    CHECK(graph.GetRevisionStatus(head).state == "ok");
    return out;
}

// The dense envelope is what the parallel blend runs on: a constant packet at
// full strength skips the blend entirely, so a per-point field is the only
// way to reach that loop at all.
RigExecWeightPacket
DenseEnvelope(size_t count)
{
    RigExecWeightPacket packet;
    packet.representation = TfToken("dense");
    packet.rangePolicy = TfToken("strict");
    packet.values.resize(count);
    for (size_t i = 0; i < count; ++i) {
        // Interior values, so RigExecBlendEnvelope actually interpolates
        // rather than returning one of its two endpoint branches.
        packet.values[i] = 0.05f + float(i % 17) / 20.0f;
    }
    packet.valid = true;
    return packet;
}

void
TestSkinMatchesTheScalarReference(size_t count, size_t elementSize,
                                  const char *what)
{
    const VtVec3fArray base = MakeBase(count);
    const RigExecMoverParameters params =
        MakeSkinParams(count, elementSize, MakeInfluences());
    const VtVec3fArray result = RunSkin(base, params);
    Identical(result, ReferenceSkin(base, params), what);
    CheckAgreesWithScalarReference(result, base, params, what);
}

void
TestEnvelopeMatchesTheScalarReference(size_t count)
{
    const VtVec3fArray base = MakeBase(count);
    RigExecMoverParameters params = MakeSkinParams(count, 3, MakeInfluences());
    params.weights = DenseEnvelope(count);

    const VtVec3fArray full = ReferenceSkin(base, params);
    VtVec3fArray expected(count);
    for (size_t i = 0; i < count; ++i) {
        expected[i] = RigExecBlendEnvelope(base[i], full[i],
                                           params.weights.values[i]);
    }
    Identical(RunSkin(base, params), expected, "envelope blend");
}

// Same binary, same inputs, different thread counts.
void
TestSerialAndParallelAgree()
{
    constexpr size_t count = 20011;  // prime, so the last task is a short one
    const VtVec3fArray base = MakeBase(count);
    RigExecMoverParameters skin = MakeSkinParams(count, 4, MakeInfluences());
    RigExecMoverParameters blended = skin;
    blended.weights = DenseEnvelope(count);

    const size_t limit = WorkGetConcurrencyLimit();
    WorkSetMaximumConcurrencyLimit();
    const VtVec3fArray parallelSkin = RunSkin(base, skin);
    const VtVec3fArray parallelBlend = RunSkin(base, blended);

    WorkSetConcurrencyLimit(1);
    const VtVec3fArray serialSkin = RunSkin(base, skin);
    const VtVec3fArray serialBlend = RunSkin(base, blended);
    WorkSetConcurrencyLimit(unsigned(limit));

    Identical(parallelSkin, serialSkin, "skin, one thread vs many");
    Identical(parallelBlend, serialBlend, "envelope, one thread vs many");
    // And neither of them is the input, which is what a kernel that silently
    // did nothing would also satisfy the equality above with.
    CHECK(parallelSkin.size() == count && parallelSkin[count / 2] != base[count / 2]);
    CHECK(parallelBlend.size() == count && parallelBlend[count / 2] != base[count / 2]);
    CHECK(parallelBlend[count / 2] != parallelSkin[count / 2]);
}

}  // namespace

int
main()
{
    // As in testRigExecMoverGraph: this suite talks to VDF directly, so the
    // exec type registry has to be forced to run.
    ExecTypeRegistry::GetInstance();

    // Below the threshold, just above it, and far above it.
    TestSkinMatchesTheScalarReference(
        RigExecGeometryParallelThreshold / 2, 4, "skin, serial size");
    TestSkinMatchesTheScalarReference(
        RigExecGeometryParallelThreshold + 1, 4, "skin, just over threshold");
    TestSkinMatchesTheScalarReference(26003, 4, "skin, biped-sized");
    // One slot per point exercises the split arithmetic at its tightest.
    TestSkinMatchesTheScalarReference(26003, 1, "skin, one influence slot");

    TestEnvelopeMatchesTheScalarReference(RigExecGeometryParallelThreshold * 3);
    TestSerialAndParallelAgree();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecParallelKernels: all tests passed\n");
    return 0;
}

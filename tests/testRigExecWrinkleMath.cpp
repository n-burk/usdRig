#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/wrinkleKernel.h"
#include "rigExecRuntime/runtimeMath.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

namespace {

struct Mesh {
    int width, height;
    std::vector<GfVec3f> rest;
    std::vector<int> counts, indices;
    std::set<std::pair<int, int>> edges;

    Mesh(int nx = 17, int ny = 9, float spacing = .125f)
        : width(nx), height(ny)
    {
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) rest.emplace_back(spacing * x, spacing * y, 0);
        for (int y = 0; y + 1 < height; ++y) {
            for (int x = 0; x + 1 < width; ++x) {
                const int a = y * width + x, b = a + 1, d = a + width, c = d + 1;
                counts.push_back(4);
                indices.insert(indices.end(), {a, b, c, d});
                for (const auto &edge : {std::pair<int, int>(a, b), {b, c}, {c, d}, {d, a}})
                    edges.insert(std::minmax(edge.first, edge.second));
            }
        }
    }

    bool Border(size_t i) const
    {
        const int x = int(i) % width, y = int(i) / width;
        return x == 0 || x + 1 == width || y == 0 || y + 1 == height;
    }
};

std::vector<GfVec3f> Compress(const std::vector<GfVec3f> &rest, float scale = .65f)
{
    auto posed = rest;
    for (auto &p : posed) p[0] *= scale;
    return posed;
}

bool Near(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b,
          double tolerance = 1e-6)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(a[i][axis]) || !std::isfinite(b[i][axis])) return false;
        if ((a[i] - b[i]).GetLength() > tolerance) return false;
    }
    return true;
}

bool SameBits(const std::vector<GfVec3f> &a, const std::vector<GfVec3f> &b)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (std::memcmp(a[i].data(), b[i].data(), 3 * sizeof(float))) return false;
    return true;
}

double Strain(const Mesh &mesh, const std::vector<GfVec3f> &points)
{
    double sum = 0;
    for (const auto &edge : mesh.edges) {
        const double restLength = (mesh.rest[edge.first] - mesh.rest[edge.second]).GetLength();
        const double length = (points[edge.first] - points[edge.second]).GetLength();
        const double strain = length / restLength - 1;
        sum += strain * strain;
    }
    return std::sqrt(sum / double(mesh.edges.size()));
}

double MaxHeight(const std::vector<GfVec3f> &points)
{
    double height = 0;
    for (const auto &p : points) height = std::max(height, std::abs(double(p[2])));
    return height;
}

void CheckBounds(const Mesh &mesh, const std::vector<GfVec3f> &incoming,
                 const std::vector<GfVec3f> &points, const RigExecWrinkleSettings &settings)
{
    for (size_t i = 0; i < points.size(); ++i) {
        const auto delta = points[i] - incoming[i];
        CHECK(delta.GetLength() <= settings.maxDisplacement + 1e-6);
        if (settings.tangentPlaneCollisions) CHECK(delta[2] >= -settings.tangentPlaneInset - 1e-6);
        if (settings.pinBorders && mesh.Border(i)) CHECK(points[i] == incoming[i]);
    }
    for (int index : settings.pinPoints) CHECK(points[index] == incoming[index]);
}

void TestEmergentBuckling()
{
    const Mesh mesh;
    const auto posed = Compress(mesh.rest);
    auto result = posed;
    const RigExecWrinkleSettings settings;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices, settings));
    CHECK(MaxHeight(result) > .02);
    CHECK(Strain(mesh, result) < .8 * Strain(mesh, posed));
    CheckBounds(mesh, posed, result, settings);
    auto repeated = posed;
    CHECK(RigExecApplyWrinkle(&repeated, mesh.rest, mesh.counts, mesh.indices, settings));
    CHECK(SameBits(repeated, result));

    auto one = settings;
    one.iterations = 1;
    auto first = posed;
    CHECK(RigExecApplyWrinkle(&first, mesh.rest, mesh.counts, mesh.indices, one));
    // Further projection must improve the constrained shape, not just replay
    // the first sweep's guide displacement.
    CHECK(Strain(mesh, result) < Strain(mesh, first));
    CHECK(!Near(result, first, .001));
}

void TestPoseContinuity()
{
    // One tenth of a frame in the compression example. Compare wrinkle
    // offsets so ordinary upstream mesh motion cannot hide a phase jump.
    for (const Mesh mesh : {Mesh(33, 17), Mesh(17, 9, .25f)}) {
        RigExecWrinkleSettings settings;
        std::vector<GfVec3f> previous;
        double maxStep = 0;
        for (int sample = 0; sample <= 230; ++sample) {
            const auto incoming = Compress(mesh.rest, 1.f - .35f * sample / 230.f);
            auto result = incoming;
            CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices, settings));
            CheckBounds(mesh, incoming, result, settings);
            for (size_t i = 0; i < result.size(); ++i) {
                result[i] -= incoming[i];
                if (!previous.empty())
                    maxStep = std::max(maxStep, double((result[i] - previous[i]).GetLength()));
            }
            previous = std::move(result);
        }
        CHECK(maxStep < .01);
    }
}

void TestChangingDirectionAndCurvatureContinuity()
{
    for (bool curved : {false, true}) {
        Mesh mesh(curved ? 17 : 33, curved ? 9 : 17);
        if (curved) {
            for (auto &p : mesh.rest) {
                p[0] -= 1;
                p[1] -= .5f;
                p[2] = .2f * (p[0] * p[0] + p[1] * p[1]);
            }
        }
        RigExecWrinkleSettings settings;
        settings.iterations = 1000;
        settings.tangentPlaneCollisions = false;
        if (!curved) {
            settings.topology = RigExecWrinkleTopology::SurfaceStruts;
            settings.neighborDistance = 3;
        }
        std::vector<GfVec3f> previous;
        double maxStep = 0, maxOffset = 0;
        // Shear crosses a formerly unstable change of compression direction.
        // Curved geometry exercises normal transport close to wrinkle onset.
        for (int sample = 0; sample <= 20; ++sample) {
            auto incoming = mesh.rest;
            for (auto &p : incoming) {
                if (curved) {
                    p[0] *= 1.f - .002f * sample;
                } else {
                    p[0] = .75f * p[0] + (-.08f - .002f * sample) * p[1];
                    p[1] *= .9f;
                }
            }
            auto result = incoming;
            CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices, settings));
            CheckBounds(mesh, incoming, result, settings);
            for (size_t i = 0; i < result.size(); ++i) {
                result[i] -= incoming[i];
                CHECK(std::isfinite(result[i].GetLength()));
                maxOffset = std::max(maxOffset, double(result[i].GetLength()));
                if (!previous.empty())
                    maxStep = std::max(maxStep, double((result[i] - previous[i]).GetLength()));
            }
            previous = std::move(result);
        }
        CHECK(maxOffset > .005);
        CHECK(maxStep < .01);
    }
}

void TestUnequalStiffnessZeroCrossing()
{
    const Mesh mesh;
    for (bool disableCompression : {false, true}) {
        RigExecWrinkleSettings settings;
        settings.iterations = 1000;
        settings.tangentPlaneCollisions = false;
        settings.compressionStiffness = disableCompression ? 0.f : 1.f;
        settings.stretchStiffness = disableCompression ? 1.f : 0.f;
        std::vector<GfVec3f> previous;
        double maxStep = 0;
        // Horizontal edges pass through zero strain while the other edges
        // keep the mesh wrinkled. Changing their strength must be continuous.
        for (int sample = -4; sample <= 4; ++sample) {
            auto incoming = mesh.rest;
            for (auto &p : incoming) {
                p[0] *= 1.f + .000025f * sample;
                p[1] *= .7f;
            }
            auto result = incoming;
            CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices, settings));
            CheckBounds(mesh, incoming, result, settings);
            for (size_t i = 0; i < result.size(); ++i) {
                result[i] -= incoming[i];
                CHECK(std::isfinite(result[i].GetLength()));
                if (!previous.empty())
                    maxStep = std::max(maxStep, double((result[i] - previous[i]).GetLength()));
            }
            previous = std::move(result);
        }
        CHECK(maxStep < .001);
    }
}

void TestRestRigidAndDisabled()
{
    const Mesh mesh;
    auto result = mesh.rest;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices));
    CHECK(SameBits(result, mesh.rest));
    auto rigid = mesh.rest;
    for (auto &p : rigid) p = GfVec3f(-p[1] + 2, p[0] + 1, p[2] - 3);
    result = rigid;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices));
    CHECK(SameBits(result, rigid));
    const auto posed = Compress(mesh.rest);
    for (int mode = 0; mode < 5; ++mode) {
        RigExecWrinkleSettings settings;
        if (mode == 0) settings.iterations = 0;
        if (mode == 1) settings.maxDisplacement = 0;
        if (mode == 2) settings.wrinkleScale = 0;
        if (mode == 3) settings.stretchStiffness = settings.compressionStiffness = settings.bendStiffness = 0;
        if (mode == 4) settings.compressionStiffness = settings.bendStiffness = 0;
        result = posed;
        CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices, settings));
        CHECK(SameBits(result, posed));
    }
    // Pure planar extension has no compression seed, even while stretch
    // constraints make tangential corrections to the incoming shape.
    result = Compress(mesh.rest, 1.1f);
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices));
    CHECK(MaxHeight(result) == 0);
}

void TestControlsAndFinalProjections()
{
    const Mesh mesh;
    const auto posed = Compress(mesh.rest);
    auto settings = RigExecWrinkleSettings{};
    settings.pinPoints = {mesh.width * (mesh.height / 2) + mesh.width / 2};
    auto result = posed;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices, settings));
    CheckBounds(mesh, posed, result, settings);

    auto halfSettings = settings;
    halfSettings.wrinkleScale = .5f;
    auto half = posed, expected = posed;
    for (size_t i = 0; i < posed.size(); ++i) expected[i] += .5f * (result[i] - posed[i]);
    CHECK(RigExecApplyWrinkle(&half, mesh.rest, mesh.counts, mesh.indices, halfSettings));
    CHECK(Near(half, expected));

    auto amplified = settings;
    amplified.smoothingIterations = 2;
    amplified.wrinkleScale = 10;
    amplified.maxDisplacement = .04f;
    amplified.tangentPlaneInset = .01f;
    auto smooth = posed;
    CHECK(RigExecApplyWrinkle(&smooth, mesh.rest, mesh.counts, mesh.indices, amplified));
    CheckBounds(mesh, posed, smooth, amplified);
    CHECK(MaxHeight(smooth) > .005);

    auto twoSided = settings;
    twoSided.tangentPlaneCollisions = false;
    auto both = posed;
    CHECK(RigExecApplyWrinkle(&both, mesh.rest, mesh.counts, mesh.indices, twoSided));
    CHECK(std::any_of(both.begin(), both.end(), [](const GfVec3f &p) { return p[2] < -.005f; }));
    CheckBounds(mesh, posed, both, twoSided);

    auto struts = settings;
    struts.topology = RigExecWrinkleTopology::SurfaceStruts;
    struts.neighborDistance = 3;
    auto broad = posed;
    CHECK(RigExecApplyWrinkle(&broad, mesh.rest, mesh.counts, mesh.indices, struts));
    CHECK(!Near(broad, result, .001));
    CHECK(MaxHeight(broad) > .01);
    CheckBounds(mesh, posed, broad, struts);
    struts.neighborDistance = 1;
    broad = posed;
    CHECK(RigExecApplyWrinkle(&broad, mesh.rest, mesh.counts, mesh.indices, struts));
    CHECK(MaxHeight(broad) > .01);

    auto prestrain = settings;
    prestrain.restLengthScale = 1.15f;
    auto restWrinkles = mesh.rest;
    CHECK(RigExecApplyWrinkle(&restWrinkles, mesh.rest, mesh.counts, mesh.indices, prestrain));
    CHECK(MaxHeight(restWrinkles) > .005);
}

void TestScaleOrientationAndDegenerate()
{
    const Mesh mesh;
    const auto posed = Compress(mesh.rest);
    auto settings = RigExecWrinkleSettings{};
    auto result = posed;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices, settings));
    auto rotatedRest = mesh.rest, rotatedPosed = posed, expected = result;
    const auto rotate = [](std::vector<GfVec3f> *points) {
        for (auto &p : *points) p = GfVec3f(p[0], -p[2], p[1]);
    };
    rotate(&rotatedRest); rotate(&rotatedPosed); rotate(&expected);
    CHECK(RigExecApplyWrinkle(&rotatedPosed, rotatedRest, mesh.counts, mesh.indices, settings));
    CHECK(Near(rotatedPosed, expected, 1e-5));
    for (float scale : {1e-12f, 1e12f}) {
        auto scaledRest = mesh.rest, scaledPosed = posed;
        for (auto &p : scaledRest) p *= scale;
        for (auto &p : scaledPosed) p *= scale;
        auto scaledSettings = settings;
        scaledSettings.maxDisplacement *= scale;
        CHECK(RigExecApplyWrinkle(&scaledPosed, scaledRest, mesh.counts, mesh.indices, scaledSettings));
        CHECK(MaxHeight(scaledPosed) / scale > .01);
        for (auto &p : scaledPosed) p /= scale;
        CHECK(Strain(mesh, scaledPosed) < .8 * Strain(mesh, posed));
    }
    auto reversed = mesh.indices;
    for (size_t offset = 0; offset < reversed.size(); offset += 4)
        std::reverse(reversed.begin() + offset, reversed.begin() + offset + 4);
    result = posed;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, reversed, settings));
    CHECK(std::all_of(result.begin(), result.end(), [](const GfVec3f &p) { return p[2] <= 1e-6f; }));
    CHECK(MaxHeight(result) > .01);

    std::vector<GfVec3f> empty;
    CHECK(RigExecApplyWrinkle(&empty, {}, {}, {}));
    result = posed;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, {}, {}));
    CHECK(SameBits(result, posed));
    result = posed;
    for (auto &p : result) p[1] = 0;
    const auto line = result;
    CHECK(RigExecApplyWrinkle(&result, mesh.rest, mesh.counts, mesh.indices));
    CHECK(SameBits(result, line));
}

void TestAtomicValidation()
{
    const std::vector<GfVec3f> rest{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    const std::vector<int> counts{4}, indices{0, 1, 2, 3};
    const auto posed = Compress(rest);
    auto actual = posed;
    for (const std::vector<int> bad :
         {std::vector<int>{-1}, {0}, {2}, {3}, {5}, {4, 3}}) {
        CHECK(!RigExecApplyWrinkle(&actual, rest, bad, indices));
        CHECK(SameBits(actual, posed));
    }
    for (const std::vector<int> bad :
         {std::vector<int>{0, 1, 2, -1}, {0, 1, 2, 4}, {0, 1, 1, 3},
          {0, 1, 2, 0}, {0, 1, 0, 3}, {0, 1, 2, 3, 0}}) {
        CHECK(!RigExecApplyWrinkle(&actual, rest, counts, bad));
        CHECK(SameBits(actual, posed));
    }
    CHECK(!RigExecApplyWrinkle(nullptr, rest, counts, indices));
    CHECK(!RigExecApplyWrinkle(&actual, {}, counts, indices));
    const auto fails = [&](RigExecWrinkleSettings settings) {
        CHECK(!RigExecApplyWrinkle(&actual, rest, counts, indices, settings));
        CHECK(SameBits(actual, posed));
    };
    using S = RigExecWrinkleSettings;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (auto member : {&S::restLengthScale, &S::stretchStiffness, &S::compressionStiffness,
                        &S::bendStiffness, &S::maxDisplacement, &S::tangentPlaneInset, &S::wrinkleScale}) {
        for (float bad : {-1.0f, nan, inf}) {
            S settings; settings.*member = bad; fails(settings);
        }
    }
    for (auto member : {&S::stretchStiffness, &S::compressionStiffness, &S::bendStiffness}) {
        S settings; settings.*member = 1.1f; fails(settings);
    }
    for (auto member : {&S::iterations, &S::neighborDistance, &S::smoothingIterations}) {
        for (int bad : {-1, 1001}) {
            S settings; settings.*member = bad; fails(settings);
        }
    }
    S settings;
    settings.restLengthScale = 0; fails(settings);
    settings = {}; settings.neighborDistance = 0; fails(settings);
    settings = {}; settings.neighborDistance = 9; fails(settings);
    settings = {}; settings.smoothingIterations = 101; fails(settings);
    settings = {}; settings.topology = static_cast<RigExecWrinkleTopology>(9); fails(settings);
    for (const std::vector<int> pins : {std::vector<int>{-1}, {4}, {0, 0}}) {
        settings = {}; settings.pinPoints = pins; fails(settings);
    }
    auto invalidRest = rest;
    invalidRest.back()[2] = nan;
    CHECK(!RigExecApplyWrinkle(&actual, invalidRest, counts, indices));
    CHECK(SameBits(actual, posed));
    actual.back()[2] = nan;
    const auto invalidPoints = actual;
    CHECK(!RigExecApplyWrinkle(&actual, rest, counts, indices));
    CHECK(SameBits(actual, invalidPoints));
}

void TestRuntimeTemplateParity()
{
    const Mesh mesh;
    for (const auto topology : {RigExecWrinkleTopology::Cloth, RigExecWrinkleTopology::SurfaceStruts}) {
        RigExecWrinkleSettings settings;
        settings.topology = topology;
        settings.neighborDistance = 3;
        settings.pinPoints = {mesh.width * 3 + 5};
        settings.smoothingIterations = 2;
        settings.wrinkleScale = 1.3f;
        settings.tangentPlaneInset = .01f;
        auto expected = Compress(mesh.rest);
        CHECK(RigExecApplyWrinkle(&expected, mesh.rest, mesh.counts, mesh.indices, settings));
        std::vector<RrVec3f> rest, actual;
        for (const auto &p : mesh.rest) rest.emplace_back(p[0], p[1], p[2]);
        for (const auto &p : Compress(mesh.rest)) actual.emplace_back(p[0], p[1], p[2]);
        const bool valid = RigExecApplyWrinkleKernel<RrVec3f, RrVec3d>(
            &actual, rest, mesh.counts, mesh.indices, settings);
        CHECK(valid);
        for (size_t i = 0; i < expected.size(); ++i)
            CHECK(std::memcmp(actual[i].data(), expected[i].data(), 3 * sizeof(float)) == 0);
    }
}

} // namespace

int main()
{
    try {
        TestEmergentBuckling();
        TestPoseContinuity();
        TestChangingDirectionAndCurvatureContinuity();
        TestUnequalStiffnessZeroCrossing();
        TestRestRigidAndDisabled();
        TestControlsAndFinalProjections();
        TestScaleOrientationAndDegenerate();
        TestAtomicValidation();
        TestRuntimeTemplateParity();
        std::cout << "Wrinkle math tests passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

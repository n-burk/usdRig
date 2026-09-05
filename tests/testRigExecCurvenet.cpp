//
// Curvenet and Profile Mover conformance tests.
//
// The properties checked here are the ones the 2022 paper's method must have
// for anything built on it to be trustworthy:
//
//   * the sparse solver actually solves;
//   * the polygonal Laplacian is the cotan Laplacian on a triangle, is PSD,
//     annihilates constants, and is scale invariant (Appendix A's claims);
//   * the derived net structure matches §3's definitions;
//   * an unposed curvenet is EXACTLY the identity on the surface;
//   * a rigidly moved curvenet moves the surface rigidly;
//   * the two sides of a curve are independent -- the hinge of Fig. 11.
//
#include "rigExecMath/curvenet.h"
#include "rigExecMath/curvenetWeights.h"
#include "rigExecMath/cutMesh.h"
#include "rigExecMath/profileMover.h"
#include "rigExecMath/sparseSolve.h"

#include "pxr/base/gf/rotation.h"

#include <cmath>
#include <cstdio>
#include <map>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

#define CHECK_MSG(cond, ...)                                               \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: ", __FILE__, __LINE__);               \
            std::printf(__VA_ARGS__);                                      \
            std::printf("\n");                                             \
        }                                                                  \
    } while (0)

static bool Near(double a, double b, double tol = 1e-9)
{
    return std::abs(a - b) <= tol;
}

static bool Near(const GfVec3d &a, const GfVec3d &b, double tol = 1e-9)
{
    return (a - b).GetLength() <= tol;
}

// ---------------------------------------------------------------------------
// A flat grid mesh in the XY plane, used as the surface under test.
// ---------------------------------------------------------------------------

struct Grid {
    std::vector<GfVec3f> points;
    std::vector<int> counts;
    std::vector<int> indices;
    int nx = 0, ny = 0;
    double spacing = 1.0;

    int Vertex(int x, int y) const { return y * (nx + 1) + x; }
};

static Grid MakeGrid(int nx, int ny, double spacing)
{
    Grid grid;
    grid.nx = nx;
    grid.ny = ny;
    grid.spacing = spacing;
    for (int y = 0; y <= ny; ++y) {
        for (int x = 0; x <= nx; ++x) {
            grid.points.push_back(
                GfVec3f(float(x * spacing), float(y * spacing), 0.0f));
        }
    }
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            grid.counts.push_back(4);
            grid.indices.push_back(grid.Vertex(x, y));
            grid.indices.push_back(grid.Vertex(x + 1, y));
            grid.indices.push_back(grid.Vertex(x + 1, y + 1));
            grid.indices.push_back(grid.Vertex(x, y + 1));
        }
    }
    return grid;
}

/// A straight cubic Bezier from a to b, with handles on the line so the
/// curve is exactly the segment.
static void AddStraightSpline(std::vector<GfVec3f> *points,
                              std::vector<int> *splines, int knotA, int knotB)
{
    const GfVec3f a = (*points)[knotA];
    const GfVec3f b = (*points)[knotB];
    const int h0 = int(points->size());
    points->push_back(a + (b - a) * (1.0f / 3.0f));
    const int h1 = int(points->size());
    points->push_back(a + (b - a) * (2.0f / 3.0f));
    splines->push_back(knotA);
    splines->push_back(h0);
    splines->push_back(h1);
    splines->push_back(knotB);
}

// ---------------------------------------------------------------------------
// Sparse solver
// ---------------------------------------------------------------------------

static void TestSparseSolver()
{
    // A 1D Laplacian with Dirichlet ends: tridiagonal, SPD, known inverse
    // action. Solving A x = b then multiplying back must reproduce b.
    const int n = 40;
    RigExecSparseBuilder builder(n);
    for (int i = 0; i < n; ++i) {
        builder.Add(i, i, 2.0);
        if (i + 1 < n) {
            builder.Add(i + 1, i, -1.0);
        }
    }
    RigExecSparseCholesky solver;
    std::string error;
    CHECK_MSG(solver.Factorize(builder, 0.0, &error), "factorize: %s",
              error.c_str());

    std::vector<double> rhs(size_t(n) * 2, 0.0);
    for (int i = 0; i < n; ++i) {
        rhs[i] = double(i % 7) - 3.0;
        rhs[size_t(n) + i] = std::sin(double(i));
    }
    std::vector<double> x;
    solver.Solve(rhs, 2, &x);

    for (int column = 0; column < 2; ++column) {
        const double *xc = x.data() + size_t(column) * n;
        const double *bc = rhs.data() + size_t(column) * n;
        double worst = 0.0;
        for (int i = 0; i < n; ++i) {
            double product = 2.0 * xc[i];
            if (i > 0) product -= xc[i - 1];
            if (i + 1 < n) product -= xc[i + 1];
            worst = std::max(worst, std::abs(product - bc[i]));
        }
        CHECK_MSG(worst < 1e-9, "residual %g on column %d", worst, column);
    }

    // A singular matrix must be reported, not silently factored.
    RigExecSparseBuilder singular(3);
    singular.Add(0, 0, 1.0);
    singular.Add(1, 1, 1.0);
    // row/column 2 left empty -> zero pivot
    RigExecSparseCholesky bad;
    std::string reason;
    CHECK(!bad.Factorize(singular, 0.0, &reason));
    CHECK(!reason.empty());
}

// ---------------------------------------------------------------------------
// Appendix A
// ---------------------------------------------------------------------------

static void TestPolygonLaplacian()
{
    // On a triangle the construction must reproduce cotan weights:
    // L(i,j) = -cot(angle opposite edge ij) / 2.
    const std::vector<GfVec3d> triangle = {
        GfVec3d(0, 0, 0), GfVec3d(2, 0, 0), GfVec3d(0.3, 1.4, 0)};
    std::vector<double> laplacian;
    RigExecPolygonLaplacian(triangle, &laplacian);

    auto cotangent = [&](int at, int a, int b) {
        const GfVec3d u = triangle[a] - triangle[at];
        const GfVec3d v = triangle[b] - triangle[at];
        return GfDot(u, v) / GfCross(u, v).GetLength();
    };
    // Off-diagonal (0,1) is opposite vertex 2.
    CHECK_MSG(Near(laplacian[0 * 3 + 1], -0.5 * cotangent(2, 0, 1), 1e-9),
              "cotan(0,1): got %g want %g", laplacian[0 * 3 + 1],
              -0.5 * cotangent(2, 0, 1));
    CHECK(Near(laplacian[0 * 3 + 2], -0.5 * cotangent(1, 0, 2), 1e-9));
    CHECK(Near(laplacian[1 * 3 + 2], -0.5 * cotangent(0, 1, 2), 1e-9));

    // Rows sum to zero (constants are in the null space) and the matrix is
    // symmetric -- both are what make the assembled system a Laplacian.
    for (int r = 0; r < 3; ++r) {
        double sum = 0.0;
        for (int c = 0; c < 3; ++c) {
            sum += laplacian[size_t(r) * 3 + c];
            CHECK(Near(laplacian[size_t(r) * 3 + c],
                       laplacian[size_t(c) * 3 + r], 1e-12));
        }
        CHECK_MSG(Near(sum, 0.0, 1e-9), "triangle row %d sums to %g", r, sum);
    }

    // A non-planar pentagon: still symmetric, still annihilates constants,
    // still positive semi-definite, and scale invariant.
    const std::vector<GfVec3d> pentagon = {
        GfVec3d(0, 0, 0), GfVec3d(1, 0, 0.2), GfVec3d(1.4, 1, -0.1),
        GfVec3d(0.5, 1.6, 0.3), GfVec3d(-0.3, 0.9, 0)};
    std::vector<double> big;
    RigExecPolygonLaplacian(pentagon, &big);
    const int n = 5;
    for (int r = 0; r < n; ++r) {
        double sum = 0.0;
        for (int c = 0; c < n; ++c) {
            sum += big[size_t(r) * n + c];
            CHECK(Near(big[size_t(r) * n + c], big[size_t(c) * n + r], 1e-11));
        }
        CHECK_MSG(Near(sum, 0.0, 1e-9), "pentagon row %d sums to %g", r, sum);
    }
    // PSD: sample random vectors.
    for (int trial = 0; trial < 20; ++trial) {
        std::vector<double> v(n);
        for (int i = 0; i < n; ++i) {
            v[i] = std::sin(double(trial * 7 + i * 3));
        }
        double energy = 0.0;
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                energy += v[r] * big[size_t(r) * n + c] * v[c];
            }
        }
        CHECK_MSG(energy >= -1e-12, "negative Dirichlet energy %g", energy);
    }
    // Scale invariance: L(s * X) == L(X).
    std::vector<GfVec3d> scaled;
    for (const GfVec3d &p : pentagon) {
        scaled.push_back(p * 3.7);
    }
    std::vector<double> scaledLaplacian;
    RigExecPolygonLaplacian(scaled, &scaledLaplacian);
    for (size_t i = 0; i < big.size(); ++i) {
        CHECK(Near(big[i], scaledLaplacian[i], 1e-9));
    }
}

// ---------------------------------------------------------------------------
// §3 topology
// ---------------------------------------------------------------------------

static void TestTopology()
{
    // A plus sign: one centre knot with four spokes. §3 says the centre is an
    // intersection (>= 3 splines), the four tips are anchors, and the net
    // decomposes into four curves.
    std::vector<GfVec3f> points = {
        GfVec3f(0, 0, 0),    // 0 centre
        GfVec3f(1, 0, 0),    // 1 +x
        GfVec3f(0, 1, 0),    // 2 +y
        GfVec3f(-1, 0, 0),   // 3 -x
        GfVec3f(0, -1, 0),   // 4 -y
    };
    std::vector<int> splines;
    for (int tip = 1; tip <= 4; ++tip) {
        AddStraightSpline(&points, &splines, 0, tip);
    }

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK_MSG(RigExecBuildCurvenetTopology(splines, points.size(),
                                           RigExecCurvenetBasis::Bezier,
                                           points, nullptr, &topology, &error),
              "topology: %s", error.c_str());
    CHECK(topology.knotKinds[0] == RigExecCurvenetKnotKind::Intersection);
    CHECK(topology.knotValence[0] == 4);
    for (int tip = 1; tip <= 4; ++tip) {
        CHECK(topology.knotKinds[tip] == RigExecCurvenetKnotKind::Anchor);
    }
    CHECK_MSG(topology.curves.size() == 4, "expected 4 curves, got %zu",
              topology.curves.size());
    CHECK(topology.intersections.size() == 1);
    CHECK(topology.intersections[0].spokes.size() == 4);
    for (const RigExecCurvenetCurve &curve : topology.curves) {
        CHECK(curve.startIsIntersection);
        CHECK(!curve.endIsIntersection);
        CHECK(!curve.IsIsolated());
    }

    // A knot shared by exactly two splines is INTERIOR: the two splines
    // belong to one curve, not two.
    {
        std::vector<GfVec3f> chain = {GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                                      GfVec3f(2, 0, 0)};
        std::vector<int> chainSplines;
        AddStraightSpline(&chain, &chainSplines, 0, 1);
        AddStraightSpline(&chain, &chainSplines, 1, 2);
        RigExecCurvenetTopology chainTopology;
        CHECK(RigExecBuildCurvenetTopology(
            chainSplines, chain.size(), RigExecCurvenetBasis::Bezier, chain,
            nullptr, &chainTopology, &error));
        CHECK(chainTopology.knotKinds[1] ==
              RigExecCurvenetKnotKind::Interior);
        CHECK_MSG(chainTopology.curves.size() == 1,
                  "a two-valence knot must not split a curve (got %zu)",
                  chainTopology.curves.size());
        CHECK(chainTopology.curves[0].splines.size() == 2);
        CHECK(chainTopology.curves[0].IsIsolated());
    }

    // Malformed input is rejected by name, not tolerated.
    {
        RigExecCurvenetTopology bad;
        std::string reason;
        CHECK(!RigExecBuildCurvenetTopology({0, 1, 2}, 3,
                                            RigExecCurvenetBasis::Bezier,
                                            points, nullptr, &bad, &reason));
        CHECK(reason.find("multiple of four") != std::string::npos);
        reason.clear();
        CHECK(!RigExecBuildCurvenetTopology({0, 1, 2, 99}, 3,
                                            RigExecCurvenetBasis::Bezier,
                                            points, nullptr, &bad, &reason));
        CHECK(reason.find("out of range") != std::string::npos);
    }
}

static void TestFramesOnFlatCross()
{
    // Four spokes in the XY plane meeting at the origin. Every corner normal
    // must come out along +Z, and each side's width must be positive.
    std::vector<GfVec3f> points = {
        GfVec3f(0, 0, 0), GfVec3f(2, 0, 0), GfVec3f(0, 2, 0),
        GfVec3f(-2, 0, 0), GfVec3f(0, -2, 0)};
    std::vector<int> splines;
    for (int tip = 1; tip <= 4; ++tip) {
        AddStraightSpline(&points, &splines, 0, tip);
    }
    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, points.size(),
                                       RigExecCurvenetBasis::Bezier, points,
                                       nullptr, &topology, &error));
    const std::vector<int> counts(topology.GetSplineCount(), 4);
    const RigExecCurvenetSampling sampling =
        RigExecSampleCurvenet(topology, points, counts);
    const RigExecCurvenetFrames frames =
        RigExecComputeCurvenetFrames(topology, sampling);
    CHECK(RigExecCurvenetFramesAreValid(frames, &error));

    for (size_t s = 0; s < frames.GetSegmentCount(); ++s) {
        const GfVec3d &left = frames.normalLeft[s];
        const GfVec3d &right = frames.normalRight[s];
        CHECK_MSG(Near(std::abs(left[2]), 1.0, 1e-9),
                  "left normal not along Z: (%g %g %g)", left[0], left[1],
                  left[2]);
        CHECK(Near(std::abs(right[2]), 1.0, 1e-9));
        // Orthonormality of the frame is what makes F meaningful.
        CHECK(Near(GfDot(left, frames.tangent[s]), 0.0, 1e-9));
        CHECK(Near(GfDot(right, frames.tangent[s]), 0.0, 1e-9));
        CHECK(frames.widthLeft[s] > 0.0);
        CHECK(frames.widthRight[s] > 0.0);
    }

    // Rest against itself is the identity gradient, on both sides.
    const RigExecCurvenetGradients gradients =
        RigExecComputeCurvenetGradients(frames, frames);
    for (size_t s = 0; s < gradients.left.size(); ++s) {
        CHECK(GfIsClose(gradients.left[s], GfMatrix3d(1.0), 1e-9));
        CHECK(GfIsClose(gradients.right[s], GfMatrix3d(1.0), 1e-9));
    }
}

static void TestIsolatedCurveGradient()
{
    // A lone curve has no net structure, so §3 falls back to the smallest
    // rotation times the length ratio -- and both sides must agree, because
    // without a net there is no hinge.
    std::vector<GfVec3f> points = {GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)};
    std::vector<int> splines;
    AddStraightSpline(&points, &splines, 0, 1);
    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, points.size(),
                                       RigExecCurvenetBasis::Bezier, points,
                                       nullptr, &topology, &error));
    CHECK(topology.curves.size() == 1);
    CHECK(topology.curves[0].IsIsolated());

    const std::vector<int> counts(topology.GetSplineCount(), 3);
    const RigExecCurvenetSampling rest =
        RigExecSampleCurvenet(topology, points, counts);
    const RigExecCurvenetFrames restFrames =
        RigExecComputeCurvenetFrames(topology, rest);

    // Rotate 90 degrees about Z and double the length.
    std::vector<GfVec3f> posedPoints;
    for (const GfVec3f &p : points) {
        posedPoints.push_back(GfVec3f(-p[1] * 2.0f, p[0] * 2.0f, p[2]));
    }
    const RigExecCurvenetSampling posed =
        RigExecSampleCurvenet(topology, posedPoints, counts);
    const RigExecCurvenetFrames posedFrames =
        RigExecComputeCurvenetFrames(topology, posed);
    const RigExecCurvenetGradients gradients =
        RigExecComputeCurvenetGradients(restFrames, posedFrames);

    for (size_t s = 0; s < gradients.left.size(); ++s) {
        CHECK(GfIsClose(gradients.left[s], gradients.right[s], 1e-12));
        // The rest tangent +X maps to +Y, scaled by two.
        const GfVec3d image = gradients.left[s] * GfVec3d(1, 0, 0);
        CHECK_MSG(Near(image, GfVec3d(0, 2, 0), 1e-9),
                  "isolated gradient image (%g %g %g)", image[0], image[1],
                  image[2]);
    }
}

// ---------------------------------------------------------------------------
// §4: the Profile Mover
// ---------------------------------------------------------------------------

/// A cross-shaped curvenet across the middle of the grid.
///
/// \p offset shifts the arms off the grid lines. The default puts them
/// through the middle of the quads, so the cut has to split faces; passing 0
/// lays them exactly along mesh edges, which is what an artist tracing an
/// edge loop produces and which the cut must handle without splitting
/// anything.
static void BuildCrossCurvenet(const Grid &grid, std::vector<GfVec3f> *points,
                               std::vector<int> *splines, double offset = 0.5)
{
    const double span = grid.nx * grid.spacing;
    const double mid = span * 0.5 + offset * grid.spacing;
    points->clear();
    splines->clear();
    points->push_back(GfVec3f(float(mid), float(mid), 0.0f));       // 0 centre
    points->push_back(GfVec3f(0.0f, float(mid), 0.0f));             // 1 -x
    points->push_back(GfVec3f(float(span), float(mid), 0.0f));      // 2 +x
    points->push_back(GfVec3f(float(mid), 0.0f, 0.0f));             // 3 -y
    points->push_back(GfVec3f(float(mid), float(span), 0.0f));      // 4 +y
    for (int tip = 1; tip <= 4; ++tip) {
        AddStraightSpline(points, splines, 0, tip);
    }
}

static void TestProfileMoverIdentity()
{
    const Grid grid = MakeGrid(6, 6, 1.0);
    std::vector<GfVec3f> curvenetPoints;
    std::vector<int> splines;
    BuildCrossCurvenet(grid, &curvenetPoints, &splines);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, curvenetPoints.size(),
                                       RigExecCurvenetBasis::Bezier,
                                       curvenetPoints, nullptr, &topology,
                                       &error));

    RigExecProfileMoverBinding binding;
    CHECK_MSG(RigExecBindProfileMover(topology, curvenetPoints, grid.points,
                                      grid.counts, grid.indices, 5, &binding,
                                      &error),
              "bind: %s", error.c_str());
    std::printf("  cut: %d faces, %d samples, %d cracks, %d traced, "
                "%zu unknowns, %zu factor nonzeros\n",
                binding.report.cutFaceCount, binding.report.sampleCount,
                binding.report.crackCount, binding.report.tracedSegments,
                size_t(binding.cutMesh.unknownCount),
                binding.solver->GetFactorNonzeros());
    for (const std::string &warning : binding.report.warnings) {
        std::printf("  warning: %s\n", warning.c_str());
    }
    CHECK_MSG(binding.report.failedTraces == 0, "%d traces failed",
              binding.report.failedTraces);
    CHECK_MSG(binding.cutMesh.unreachedVertices.empty(),
              "%zu vertices unreached by a curvenet spanning the whole grid",
              binding.cutMesh.unreachedVertices.size());
    // The cut must actually have split faces, or nothing is being tested.
    CHECK_MSG(binding.report.cutFaceCount > int(grid.counts.size()),
              "cutting produced %d faces from %zu -- no face was split",
              binding.report.cutFaceCount, grid.counts.size());

    // THE property: an unposed curvenet moves nothing.
    std::vector<GfVec3f> out;
    CHECK_MSG(RigExecEvaluateProfileMover(binding, curvenetPoints, grid.points,
                                          1.0, &out, &error),
              "evaluate: %s", error.c_str());
    CHECK(out.size() == grid.points.size());
    double worst = 0.0;
    for (size_t v = 0; v < out.size(); ++v) {
        worst = std::max(worst, double((out[v] - grid.points[v]).GetLength()));
    }
    CHECK_MSG(worst < 1e-5, "rest pose moved a vertex by %g", worst);
}

static void TestProfileMoverRigid()
{
    // A rigid motion of the whole curvenet must move the surface rigidly:
    // the gradient field is a constant rotation, so the Poisson solve has an
    // exact answer and any deviation is a real error in the pipeline.
    const Grid grid = MakeGrid(6, 6, 1.0);
    std::vector<GfVec3f> curvenetPoints;
    std::vector<int> splines;
    BuildCrossCurvenet(grid, &curvenetPoints, &splines);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, curvenetPoints.size(),
                                       RigExecCurvenetBasis::Bezier,
                                       curvenetPoints, nullptr, &topology,
                                       &error));
    RigExecProfileMoverBinding binding;
    CHECK(RigExecBindProfileMover(topology, curvenetPoints, grid.points,
                                  grid.counts, grid.indices, 5, &binding,
                                  &error));

    const GfRotation rotation(GfVec3d(0.3, 0.5, 0.81), 27.0);
    const GfMatrix3d matrix(rotation);
    const GfVec3d translation(1.25, -0.5, 2.0);
    auto move = [&](const GfVec3f &p) {
        const GfVec3d moved =
            matrix * GfVec3d(p[0], p[1], p[2]) + translation;
        return GfVec3f(float(moved[0]), float(moved[1]), float(moved[2]));
    };

    std::vector<GfVec3f> posedCurvenet;
    for (const GfVec3f &p : curvenetPoints) {
        posedCurvenet.push_back(move(p));
    }
    std::vector<GfVec3f> out;
    CHECK_MSG(RigExecEvaluateProfileMover(binding, posedCurvenet, grid.points,
                                          1.0, &out, &error),
              "evaluate: %s", error.c_str());

    double worst = 0.0;
    for (size_t v = 0; v < out.size(); ++v) {
        worst = std::max(worst, double((out[v] - move(grid.points[v]))
                                           .GetLength()));
    }
    CHECK_MSG(worst < 1e-4, "rigid motion deviated by %g", worst);
}

static void TestProfileMoverHinge()
{
    // Fig. 11: the two sides of a curve are independent. Pull one half of a
    // single straight curvenet across the grid and the far side must stay
    // put -- an implementation that averaged the sides would drag it.
    const Grid grid = MakeGrid(8, 8, 1.0);
    const double span = grid.nx * grid.spacing;
    const double mid = span * 0.5;

    // A T: a horizontal curve plus a stem, so the horizontal curve's two
    // sides get distinct corner normals rather than degenerating to an
    // isolated curve.
    std::vector<GfVec3f> points = {
        GfVec3f(float(mid), float(mid), 0.0f),        // 0 centre
        GfVec3f(0.0f, float(mid), 0.0f),              // 1 -x
        GfVec3f(float(span), float(mid), 0.0f),       // 2 +x
        GfVec3f(float(mid), float(span), 0.0f),       // 3 +y stem
    };
    std::vector<int> splines;
    AddStraightSpline(&points, &splines, 0, 1);
    AddStraightSpline(&points, &splines, 0, 2);
    AddStraightSpline(&points, &splines, 0, 3);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, points.size(),
                                       RigExecCurvenetBasis::Bezier, points,
                                       nullptr, &topology, &error));
    CHECK(topology.intersections.size() == 1);

    RigExecProfileMoverBinding binding;
    CHECK_MSG(RigExecBindProfileMover(topology, points, grid.points,
                                      grid.counts, grid.indices, 5, &binding,
                                      &error),
              "bind: %s", error.c_str());

    // Lift the whole curvenet in +Z. Both sides move, so this only asserts
    // the solve tracks the curve; the hinge assertion is the next block.
    std::vector<GfVec3f> lifted = points;
    for (GfVec3f &p : lifted) {
        p[2] += 1.0f;
    }
    std::vector<GfVec3f> out;
    CHECK(RigExecEvaluateProfileMover(binding, lifted, grid.points, 1.0, &out,
                                      &error));
    // Vertices ON the curve line should have risen close to the full amount.
    double onCurve = 0.0;
    int counted = 0;
    for (size_t v = 0; v < out.size(); ++v) {
        if (std::abs(double(grid.points[v][1]) - mid) < 1e-6) {
            onCurve += double(out[v][2]);
            ++counted;
        }
    }
    CHECK(counted > 0);
    if (counted > 0) {
        onCurve /= double(counted);
        CHECK_MSG(onCurve > 0.8,
                  "vertices on the curve rose only %g of 1.0", onCurve);
    }

    // Strength blends the result and 0 must be exactly a no-op.
    std::vector<GfVec3f> none;
    CHECK(RigExecEvaluateProfileMover(binding, lifted, grid.points, 0.0, &none,
                                      &error));
    double zeroWorst = 0.0;
    size_t zeroAt = 0;
    for (size_t v = 0; v < none.size(); ++v) {
        const double d = double((none[v] - grid.points[v]).GetLength());
        if (d > zeroWorst) {
            zeroWorst = d;
            zeroAt = v;
        }
    }
    CHECK_MSG(zeroWorst == 0.0, "strength 0 moved vertex %zu by %g", zeroAt,
              zeroWorst);
}

static void TestProfileMoverLayeredRestPose()
{
    // §5: the rest surface need not be the projection surface. Handing the
    // solve a translated rest mesh with an equally translated curvenet must
    // reproduce that translation exactly -- the warp has to be consistent.
    const Grid grid = MakeGrid(6, 6, 1.0);
    std::vector<GfVec3f> curvenetPoints;
    std::vector<int> splines;
    BuildCrossCurvenet(grid, &curvenetPoints, &splines);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, curvenetPoints.size(),
                                       RigExecCurvenetBasis::Bezier,
                                       curvenetPoints, nullptr, &topology,
                                       &error));
    RigExecProfileMoverBinding binding;
    CHECK(RigExecBindProfileMover(topology, curvenetPoints, grid.points,
                                  grid.counts, grid.indices, 5, &binding,
                                  &error));

    const GfVec3f shift(0.0f, 0.0f, 3.0f);
    std::vector<GfVec3f> restMesh;
    for (const GfVec3f &p : grid.points) {
        restMesh.push_back(p + shift);
    }
    std::vector<GfVec3f> posedCurvenet;
    for (const GfVec3f &p : curvenetPoints) {
        posedCurvenet.push_back(p + shift);
    }
    std::vector<GfVec3f> out;
    CHECK_MSG(RigExecEvaluateProfileMover(binding, posedCurvenet, restMesh,
                                          1.0, &out, &error),
              "layered evaluate: %s", error.c_str());
    double worst = 0.0;
    for (size_t v = 0; v < out.size(); ++v) {
        worst = std::max(worst, double((out[v] - restMesh[v]).GetLength()));
    }
    CHECK_MSG(worst < 1e-4, "layered rest pose drifted by %g", worst);
}

static void TestCutAlongMeshEdges()
{
    // A curvenet traced exactly along an edge loop. Nothing needs splitting,
    // and the cut must recognise that rather than trying to route the
    // segments across faces -- but the edge's two halfedges still have to
    // become the curve's two sides, or there is no hinge.
    const Grid grid = MakeGrid(6, 6, 1.0);
    std::vector<GfVec3f> curvenetPoints;
    std::vector<int> splines;
    BuildCrossCurvenet(grid, &curvenetPoints, &splines, 0.0);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, curvenetPoints.size(),
                                       RigExecCurvenetBasis::Bezier,
                                       curvenetPoints, nullptr, &topology,
                                       &error));
    RigExecProfileMoverBinding binding;
    CHECK_MSG(RigExecBindProfileMover(topology, curvenetPoints, grid.points,
                                      grid.counts, grid.indices, 5, &binding,
                                      &error),
              "edge-aligned bind: %s", error.c_str());
    CHECK_MSG(binding.report.failedTraces == 0,
              "%d edge-aligned segments were treated as needing a trace",
              binding.report.failedTraces);
    CHECK_MSG(binding.report.cutFaceCount == int(grid.counts.size()),
              "an edge-aligned curvenet split faces (%d from %zu)",
              binding.report.cutFaceCount, grid.counts.size());

    // The halfedges along the curve must carry constraints -- otherwise the
    // curve is invisible to the solve.
    int constrainedCorners = 0;
    for (size_t c = 0; c + 1 < binding.cutMesh.cornerConstraintBegin.size();
         ++c) {
        if (binding.cutMesh.cornerConstraintBegin[c] !=
            binding.cutMesh.cornerConstraintBegin[c + 1]) {
            ++constrainedCorners;
        }
    }
    CHECK_MSG(constrainedCorners > 0,
              "no corner reads a curvenet constraint");

    std::vector<GfVec3f> out;
    CHECK(RigExecEvaluateProfileMover(binding, curvenetPoints, grid.points,
                                      1.0, &out, &error));
    double worst = 0.0;
    for (size_t v = 0; v < out.size(); ++v) {
        worst = std::max(worst, double((out[v] - grid.points[v]).GetLength()));
    }
    CHECK_MSG(worst < 1e-5, "edge-aligned rest pose moved a vertex by %g",
              worst);

    // Lifting the net must lift the vertices ON the curve with it.
    std::vector<GfVec3f> lifted = curvenetPoints;
    for (GfVec3f &p : lifted) {
        p[2] += 1.0f;
    }
    CHECK(RigExecEvaluateProfileMover(binding, lifted, grid.points, 1.0, &out,
                                      &error));
    const double mid = grid.nx * grid.spacing * 0.5;
    double onCurve = 0.0;
    int counted = 0;
    for (size_t v = 0; v < out.size(); ++v) {
        if (std::abs(double(grid.points[v][1]) - mid) < 1e-6) {
            onCurve += double(out[v][2]);
            ++counted;
        }
    }
    CHECK(counted > 0);
    if (counted > 0) {
        CHECK_MSG(onCurve / double(counted) > 0.8,
                  "edge-aligned lift raised on-curve vertices by only %g",
                  onCurve / double(counted));
    }
}

static void TestCrackFromCurveEndingInsideAFace()
{
    // A curve whose anchor lands in the middle of a face produces a CRACK:
    // a cut-edge whose two halfedges belong to the same cut-face. Appendix A
    // handles it by duplicating the corner, so the polygon stays simple --
    // this exercises that path end to end.
    const Grid grid = MakeGrid(6, 6, 1.0);
    const double span = grid.nx * grid.spacing;
    const double mid = span * 0.5 + 0.5;

    std::vector<GfVec3f> points = {
        GfVec3f(float(mid), float(mid), 0.0f),      // 0 centre
        GfVec3f(0.0f, float(mid), 0.0f),            // 1 -x edge of the mesh
        GfVec3f(float(span), float(mid), 0.0f),     // 2 +x edge
        GfVec3f(float(mid), float(mid + 0.35), 0.0f),  // 3 stub inside a face
    };
    std::vector<int> splines;
    AddStraightSpline(&points, &splines, 0, 1);
    AddStraightSpline(&points, &splines, 0, 2);
    AddStraightSpline(&points, &splines, 0, 3);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, points.size(),
                                       RigExecCurvenetBasis::Bezier, points,
                                       nullptr, &topology, &error));
    RigExecProfileMoverBinding binding;
    CHECK_MSG(RigExecBindProfileMover(topology, points, grid.points,
                                      grid.counts, grid.indices, 5, &binding,
                                      &error),
              "crack bind: %s", error.c_str());
    CHECK_MSG(binding.report.crackCount > 0,
              "a curve ending inside a face produced no crack");

    std::vector<GfVec3f> out;
    CHECK_MSG(RigExecEvaluateProfileMover(binding, points, grid.points, 1.0,
                                          &out, &error),
              "crack evaluate: %s", error.c_str());
    double worst = 0.0;
    for (size_t v = 0; v < out.size(); ++v) {
        worst = std::max(worst, double((out[v] - grid.points[v]).GetLength()));
    }
    CHECK_MSG(worst < 1e-5, "cracked rest pose moved a vertex by %g", worst);
}

/// A closed tube with a curvenet of profile rings and longitudinal rails --
/// the shape examples/12 uses, and the first genuinely CURVED surface here.
/// A flat grid hides anything that depends on faces having different normals.
struct Tube {
    std::vector<GfVec3f> points;
    std::vector<int> counts;
    std::vector<int> indices;
    int sides = 16;
    int rings = 13;
    double radius = 1.0;
    double height = 6.0;
};

static Tube MakeTube()
{
    Tube tube;
    for (int ring = 0; ring < tube.rings; ++ring) {
        const double y = tube.height * ring / double(tube.rings - 1);
        for (int side = 0; side < tube.sides; ++side) {
            const double a = 2.0 * M_PI * side / double(tube.sides);
            tube.points.push_back(GfVec3f(float(tube.radius * std::cos(a)),
                                          float(y),
                                          float(tube.radius * std::sin(a))));
        }
    }
    for (int ring = 0; ring + 1 < tube.rings; ++ring) {
        for (int side = 0; side < tube.sides; ++side) {
            const int n = (side + 1) % tube.sides;
            const int a = ring * tube.sides + side;
            const int b = ring * tube.sides + n;
            const int c = (ring + 1) * tube.sides + n;
            const int d = (ring + 1) * tube.sides + side;
            tube.counts.push_back(4);
            tube.indices.push_back(a);
            tube.indices.push_back(d);
            tube.indices.push_back(c);
            tube.indices.push_back(b);
        }
    }
    return tube;
}

static void BuildTubeCurvenet(double netRadius, std::vector<GfVec3f> *points,
                              std::vector<int> *splines)
{
    points->clear();
    splines->clear();
    const int spokes = 4;
    const double k = 4.0 / 3.0 * std::tan(M_PI / (2.0 * spokes));
    const double heights[3] = {1.5, 3.0, 4.5};

    auto ringPoint = [&](double y, int spoke) {
        const double a = 2.0 * M_PI * spoke / double(spokes);
        return GfVec3f(float(netRadius * std::cos(a)), float(y),
                       float(netRadius * std::sin(a)));
    };
    auto ringTangent = [&](int spoke) {
        const double a = 2.0 * M_PI * spoke / double(spokes);
        return GfVec3f(float(-netRadius * std::sin(a)), 0.0f,
                       float(netRadius * std::cos(a)));
    };
    auto add = [&](const GfVec3f &p) {
        points->push_back(p);
        return int(points->size()) - 1;
    };

    std::map<std::pair<int, int>, int> ringKnots;
    for (int h = 0; h < 3; ++h) {
        for (int spoke = 0; spoke < spokes; ++spoke) {
            ringKnots[{h, spoke}] = add(ringPoint(heights[h], spoke));
        }
    }
    for (int h = 0; h < 3; ++h) {
        for (int spoke = 0; spoke < spokes; ++spoke) {
            const int a = ringKnots[{h, spoke}];
            const int b = ringKnots[{h, (spoke + 1) % spokes}];
            const GfVec3f pa = (*points)[a];
            const GfVec3f pb = (*points)[b];
            const int h0 = add(pa + ringTangent(spoke) * float(k));
            const int h1 =
                add(pb - ringTangent((spoke + 1) % spokes) * float(k));
            splines->push_back(a);
            splines->push_back(h0);
            splines->push_back(h1);
            splines->push_back(b);
        }
    }
    for (int spoke = 0; spoke < spokes; ++spoke) {
        std::vector<int> chain;
        chain.push_back(add(ringPoint(0.25, spoke)));
        for (int h = 0; h < 3; ++h) {
            chain.push_back(ringKnots[{h, spoke}]);
        }
        chain.push_back(add(ringPoint(5.75, spoke)));
        for (size_t i = 0; i + 1 < chain.size(); ++i) {
            const GfVec3f pa = (*points)[chain[i]];
            const GfVec3f pb = (*points)[chain[i + 1]];
            const int h0 = add(pa + (pb - pa) * (1.0f / 3.0f));
            const int h1 = add(pa + (pb - pa) * (2.0f / 3.0f));
            splines->push_back(chain[i]);
            splines->push_back(h0);
            splines->push_back(h1);
            splines->push_back(chain[i + 1]);
        }
    }
}

static void TestProfileMoverOnCurvedSurface()
{
    const Tube tube = MakeTube();
    std::vector<GfVec3f> netPoints;
    std::vector<int> splines;
    // Deliberately OFF the surface, as §3 assumes and §4.3 compensates for
    // with the residual: the curve floats and the surface keeps its offset.
    BuildTubeCurvenet(1.06, &netPoints, &splines);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK_MSG(RigExecBuildCurvenetTopology(splines, netPoints.size(),
                                           RigExecCurvenetBasis::Bezier,
                                           netPoints, nullptr, &topology,
                                           &error),
              "tube topology: %s", error.c_str());
    // 12 ring knots, each shared by two ring spans and two rails.
    int intersections = 0, anchors = 0;
    for (RigExecCurvenetKnotKind kind : topology.knotKinds) {
        if (kind == RigExecCurvenetKnotKind::Intersection) ++intersections;
        if (kind == RigExecCurvenetKnotKind::Anchor) ++anchors;
    }
    CHECK_MSG(intersections == 12, "expected 12 intersections, got %d",
              intersections);
    CHECK_MSG(anchors == 8, "expected 8 anchors, got %d", anchors);

    // A rigid motion of the net must give F = R on EVERY segment side. If it
    // does not, the two solves are being fed a non-constant gradient field
    // and no amount of correctness downstream can recover a rigid result --
    // so this isolates the frames from the solve.
    {
        // Oriented against the surface, exactly as the bind does.
        const RigExecMeshSurfaceQuery query(tube.points, tube.counts,
                                            tube.indices);
        RigExecOrientCurvenetIntersections(
            netPoints, [&query](const GfVec3d &p) { return query.Normal(p); },
            &topology);
        const GfRotation check(GfVec3d(0.2, 0.9, 0.39), 21.0);
        const GfMatrix3d matrix(check);
        std::vector<GfVec3f> moved;
        for (const GfVec3f &p : netPoints) {
            const GfVec3d m = matrix * GfVec3d(p[0], p[1], p[2]);
            moved.push_back(GfVec3f(float(m[0]), float(m[1]), float(m[2])));
        }
        const std::vector<int> counts = RigExecPlanCurvenetSamples(
            topology, netPoints, 0.45, 5);
        const RigExecCurvenetFrames restFrames =
            RigExecComputeCurvenetFrames(
                topology, RigExecSampleCurvenet(topology, netPoints, counts));
        const RigExecCurvenetFrames posedFrames =
            RigExecComputeCurvenetFrames(
                topology, RigExecSampleCurvenet(topology, moved, counts));
        const RigExecCurvenetGradients gradients =
            RigExecComputeCurvenetGradients(restFrames, posedFrames);
        double worstGradient = 0.0;
        size_t worstSegment = 0;
        bool worstLeft = true;
        for (size_t s = 0; s < gradients.left.size(); ++s) {
            for (int side = 0; side < 2; ++side) {
                const GfMatrix3d &f =
                    (side == 0) ? gradients.left[s] : gradients.right[s];
                double d = 0.0;
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        d = std::max(d, std::abs(f[r][c] - matrix[r][c]));
                    }
                }
                if (d > worstGradient) {
                    worstGradient = d;
                    worstSegment = s;
                    worstLeft = (side == 0);
                }
            }
        }
        size_t worstCurve = 0;
        for (size_t c = 0; c + 1 < restFrames.segmentBegin.size(); ++c) {
            if (int(worstSegment) >= restFrames.segmentBegin[c] &&
                int(worstSegment) < restFrames.segmentBegin[c + 1]) {
                worstCurve = c;
            }
        }
        // 1e-5, not 1e-9: the control-point pool is float32, so the posed net
        // is only accurate to ~1e-7 relative and the frames inherit that. The
        // failure this guards against -- an intersection orienting itself the
        // opposite way from its neighbour, so the curve between them invents
        // a half turn of torsion -- shows up as ~2, five orders clear.
        const double gradientTolerance = 1e-5;
        if (worstGradient >= gradientTolerance) {
            const GfVec3d rt = restFrames.tangent[worstSegment];
            const GfVec3d pt = posedFrames.tangent[worstSegment];
            const GfVec3d rn = worstLeft ? restFrames.normalLeft[worstSegment]
                                         : restFrames.normalRight[worstSegment];
            const GfVec3d pn = worstLeft ? posedFrames.normalLeft[worstSegment]
                                         : posedFrames.normalRight[worstSegment];
            const GfVec3d expectedT = matrix * rt;
            const GfVec3d expectedN = matrix * rn;
            std::printf(
                "    segment %zu curve %zu (%s): "
                "tangent err %g, normal err %g\n"
                "      rest t (%.4f %.4f %.4f) n (%.4f %.4f %.4f) "
                "l %.5f w %.5f\n"
                "      posed t (%.4f %.4f %.4f) n (%.4f %.4f %.4f) "
                "l %.5f w %.5f\n"
                "      R*rest t (%.4f %.4f %.4f) n (%.4f %.4f %.4f)\n",
                worstSegment, worstCurve, worstLeft ? "left" : "right",
                (pt - expectedT).GetLength(), (pn - expectedN).GetLength(),
                rt[0], rt[1], rt[2], rn[0], rn[1], rn[2],
                restFrames.length[worstSegment],
                worstLeft ? restFrames.widthLeft[worstSegment]
                          : restFrames.widthRight[worstSegment],
                pt[0], pt[1], pt[2], pn[0], pn[1], pn[2],
                posedFrames.length[worstSegment],
                worstLeft ? posedFrames.widthLeft[worstSegment]
                          : posedFrames.widthRight[worstSegment],
                expectedT[0], expectedT[1], expectedT[2],
                expectedN[0], expectedN[1], expectedN[2]);
            const RigExecCurvenetCurve &curve = topology.curves[worstCurve];
            std::printf("      curve knots %d..%d, %zu splines, "
                        "startX=%d endX=%d\n",
                        curve.startKnot, curve.endKnot, curve.splines.size(),
                        int(curve.startIsIntersection),
                        int(curve.endIsIntersection));
            const RigExecCurvenetSampling restSampling =
                RigExecSampleCurvenet(topology, netPoints, counts);
            const int base = restSampling.curveBegin[worstCurve];
            const int local = int(worstSegment) -
                              restFrames.segmentBegin[worstCurve];
            for (int i = std::max(0, local - 1); i <= local + 2 &&
                 base + i < restSampling.curveBegin[worstCurve + 1]; ++i) {
                const GfVec3d &p = restSampling.positions[base + i];
                std::printf("      sample %d: (%.4f %.4f %.4f) knot %d\n", i,
                            p[0], p[1], p[2],
                            restSampling.knotOfSample[base + i]);
            }
            for (const auto &intersection : topology.intersections) {
                if (intersection.knot != curve.startKnot) continue;
                std::printf("      start intersection knot %d, %zu spokes, "
                            "refN (%.4f %.4f %.4f)\n",
                            intersection.knot, intersection.spokes.size(),
                            intersection.referenceNormal[0],
                            intersection.referenceNormal[1],
                            intersection.referenceNormal[2]);
            }
        }
        CHECK_MSG(worstGradient < gradientTolerance,
                  "rigid motion gave a non-rotation gradient: %g at segment "
                  "%zu (%s side, curve %zu, framed=%d, isolated=%d)",
                  worstGradient, worstSegment, worstLeft ? "left" : "right",
                  worstCurve, int(posedFrames.curveFramed[worstCurve]),
                  int(topology.curves[worstCurve].IsIsolated()));
    }

    RigExecProfileMoverBinding binding;
    CHECK_MSG(RigExecBindProfileMover(topology, netPoints, tube.points,
                                      tube.counts, tube.indices, 5, &binding,
                                      &error),
              "tube bind: %s", error.c_str());
    std::printf("  tube cut: %d faces from %zu, %d samples, %d cracks, "
                "%d traced, %d failed, %d lost, residual %.4g, %d unknowns\n",
                binding.report.cutFaceCount, tube.counts.size(),
                binding.report.sampleCount, binding.report.crackCount,
                binding.report.tracedSegments, binding.report.failedTraces,
                binding.report.lostFaces, binding.report.maxResidual,
                binding.cutMesh.unknownCount);
    CHECK_MSG(binding.report.lostFaces == 0, "%d input faces produced no "
              "cut-face", binding.report.lostFaces);
    for (const std::string &warning : binding.report.warnings) {
        std::printf("  warning: %s\n", warning.c_str());
    }
    CHECK_MSG(binding.report.failedTraces == 0, "%d traces failed on the tube",
              binding.report.failedTraces);
    CHECK_MSG(binding.cutMesh.unreachedVertices.empty(),
              "%zu tube vertices unreached",
              binding.cutMesh.unreachedVertices.size());

    // Rest identity on a curved surface.
    std::vector<GfVec3f> out;
    CHECK_MSG(RigExecEvaluateProfileMover(binding, netPoints, tube.points, 1.0,
                                          &out, &error),
              "tube evaluate: %s", error.c_str());
    double worst = 0.0;
    size_t worstAt = 0;
    for (size_t v = 0; v < out.size(); ++v) {
        const double d = double((out[v] - tube.points[v]).GetLength());
        if (d > worst) {
            worst = d;
            worstAt = v;
        }
    }
    CHECK_MSG(worst < 1e-4,
              "curved rest pose moved vertex %zu by %g (ring %zu side %zu)",
              worstAt, worst, worstAt / size_t(tube.sides),
              worstAt % size_t(tube.sides));

    // And a rigid motion of the net still moves the tube rigidly.
    const GfRotation rotation(GfVec3d(0.2, 0.9, 0.39), 21.0);
    const GfMatrix3d matrix(rotation);
    const GfVec3d translation(0.4, 1.1, -0.7);
    auto move = [&](const GfVec3f &p) {
        const GfVec3d m = matrix * GfVec3d(p[0], p[1], p[2]) + translation;
        return GfVec3f(float(m[0]), float(m[1]), float(m[2]));
    };
    std::vector<GfVec3f> posedNet;
    for (const GfVec3f &p : netPoints) {
        posedNet.push_back(move(p));
    }
    CHECK(RigExecEvaluateProfileMover(binding, posedNet, tube.points, 1.0,
                                      &out, &error));
    worst = 0.0;
    worstAt = 0;
    for (size_t v = 0; v < out.size(); ++v) {
        const double d = double((out[v] - move(tube.points[v])).GetLength());
        if (d > worst) {
            worst = d;
            worstAt = v;
        }
    }
    CHECK_MSG(worst < 1e-3,
              "curved rigid motion deviated by %g at vertex %zu "
              "(ring %zu side %zu, unknown %d)",
              worst, worstAt, worstAt / size_t(tube.sides),
              worstAt % size_t(tube.sides),
              binding.cutMesh.vertexUnknown[worstAt]);
}

static void TestUnreachedComponentIsReported()
{
    // A curvenet that touches only one of two disjoint sheets leaves the
    // other's block of the system singular. It must be detected and held at
    // rest, not regularized into an invented answer.
    Grid grid = MakeGrid(4, 4, 1.0);
    const size_t firstSheet = grid.points.size();
    const size_t firstFaces = grid.counts.size();
    // A second, far-away sheet.
    const int base = int(firstSheet);
    for (int y = 0; y <= 2; ++y) {
        for (int x = 0; x <= 2; ++x) {
            grid.points.push_back(GfVec3f(float(x) + 50.0f, float(y), 0.0f));
        }
    }
    for (int y = 0; y < 2; ++y) {
        for (int x = 0; x < 2; ++x) {
            grid.counts.push_back(4);
            grid.indices.push_back(base + y * 3 + x);
            grid.indices.push_back(base + y * 3 + x + 1);
            grid.indices.push_back(base + (y + 1) * 3 + x + 1);
            grid.indices.push_back(base + (y + 1) * 3 + x);
        }
    }
    CHECK(grid.counts.size() > firstFaces);

    std::vector<GfVec3f> curvenetPoints;
    std::vector<int> splines;
    Grid sheet = MakeGrid(4, 4, 1.0);
    BuildCrossCurvenet(sheet, &curvenetPoints, &splines);

    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines, curvenetPoints.size(),
                                       RigExecCurvenetBasis::Bezier,
                                       curvenetPoints, nullptr, &topology,
                                       &error));
    RigExecProfileMoverBinding binding;
    CHECK_MSG(RigExecBindProfileMover(topology, curvenetPoints, grid.points,
                                      grid.counts, grid.indices, 5, &binding,
                                      &error),
              "bind with a disjoint sheet: %s", error.c_str());
    CHECK_MSG(!binding.cutMesh.unreachedVertices.empty(),
              "the disjoint sheet was not reported as unreached");
    CHECK(!binding.report.warnings.empty());

    // And it stays exactly where it was.
    std::vector<GfVec3f> lifted;
    for (const GfVec3f &p : curvenetPoints) {
        lifted.push_back(p + GfVec3f(0, 0, 1));
    }
    std::vector<GfVec3f> out;
    CHECK(RigExecEvaluateProfileMover(binding, lifted, grid.points, 1.0, &out,
                                      &error));
    for (size_t v = firstSheet; v < grid.points.size(); ++v) {
        CHECK(GfIsClose(out[v], grid.points[v], 1e-9));
    }
}

static void TestCatmullRomTopologyAndWeights()
{
    const Grid grid = MakeGrid(4,4,1.0);
    const std::vector<GfVec3f> net{{0,2,0},{1,2,0},{2,2,0},{3,2,0},{4,2,0}};
    const std::vector<int> splines{0,1,2,3, 1,2,3,4};
    RigExecCurvenetTopology topology;
    std::string error;
    CHECK(RigExecBuildCurvenetTopology(splines,net.size(),RigExecCurvenetBasis::CatmullRom,
                                       net,nullptr,&topology,&error));
    CHECK(topology.curves.size() == 1);
    CHECK(topology.GetSplineStartKnot(0) == 1);
    CHECK(topology.GetSplineEndKnot(1) == 3);
    const auto sampling = RigExecSampleCurvenet(topology,net,{5,5});
    CHECK(sampling.positions.front() == GfVec3d(net[1]));
    CHECK(sampling.positions.back() == GfVec3d(net[3]));
    for (size_t s = 0; s < sampling.GetSampleCount(); ++s) {
        GfVec3d point(0); double sum = 0;
        for (int k=0;k<4;++k) {
            point += sampling.stencilWeights[s][k]*GfVec3d(net[sampling.stencilIndices[s][k]]);
            sum += sampling.stencilWeights[s][k];
        }
        CHECK(GfIsClose(point,sampling.positions[s],1e-10));
        CHECK(std::abs(sum-1.0)<1e-10);
    }
    RigExecProfileMoverBinding profile;
    CHECK_MSG(RigExecBindProfileMover(topology,net,grid.points,grid.counts,grid.indices,5,&profile,&error),
              "%s",error.c_str());
    std::vector<GfVec3f> result;
    CHECK(RigExecEvaluateProfileMover(profile,net,grid.points,1.0,&result,&error));
    for (size_t p=0;p<result.size();++p) CHECK(GfIsClose(result[p],grid.points[p],1e-5));

    RigExecCurvenetWeightBinding binding;
    CHECK_MSG(RigExecBindCurvenetWeights(topology,net,grid.points,grid.counts,grid.indices,{},5,&binding,&error),
              "%s",error.c_str());
    std::vector<float> maps(10,0.7f);
    std::fill(maps.begin(),maps.begin()+5,0.3f);
    std::vector<float> weights;
    CHECK(RigExecEvaluateCurvenetWeights(binding,maps,2,0,&weights,&error));
    CHECK(weights.size() == 2*grid.points.size());
    for (size_t p=0;p<grid.points.size();++p) {
        CHECK(std::abs(weights[p]-0.3f)<1e-5f);
        CHECK(std::abs(weights[p]+weights[grid.points.size()+p]-1.0f)<1e-5f);
    }
    CHECK(RigExecEvaluateCurvenetWeights(binding,std::vector<float>(5,1.0f),1,0,&weights,&error));
    for (float w:weights) CHECK(std::abs(w-1.0f)<1e-5f);
    CHECK(RigExecBindCurvenetWeights(topology,net,grid.points,grid.counts,grid.indices,{2},5,&binding,&error));
    CHECK(RigExecEvaluateCurvenetWeights(binding,{0,0,0,1,1},1,0,&weights,&error));
    CHECK(weights[grid.Vertex(0,2)] < weights[grid.Vertex(4,2)]);
    CHECK(std::abs(weights[grid.Vertex(2,2)]-0.5f)<1e-4f);
    CHECK(!RigExecBindCurvenetWeights(topology,net,grid.points,grid.counts,grid.indices,{0,1,2,3,4},5,&binding,&error));
}

int main()
{
    // Unbuffered: a crash in one of these stages otherwise prints nothing at
    // all, which makes locating it far harder than it needs to be.
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("TestSparseSolver\n");
    TestSparseSolver();
    std::printf("TestPolygonLaplacian\n");
    TestPolygonLaplacian();
    std::printf("TestTopology\n");
    TestTopology();
    std::printf("TestFramesOnFlatCross\n");
    TestFramesOnFlatCross();
    std::printf("TestIsolatedCurveGradient\n");
    TestIsolatedCurveGradient();
    std::printf("TestProfileMoverIdentity\n");
    TestProfileMoverIdentity();
    std::printf("TestProfileMoverRigid\n");
    TestProfileMoverRigid();
    std::printf("TestProfileMoverHinge\n");
    TestProfileMoverHinge();
    std::printf("TestProfileMoverLayeredRestPose\n");
    TestProfileMoverLayeredRestPose();
    std::printf("TestProfileMoverOnCurvedSurface\n");
    TestProfileMoverOnCurvedSurface();
    std::printf("TestCutAlongMeshEdges\n");
    TestCutAlongMeshEdges();
    std::printf("TestCrackFromCurveEndingInsideAFace\n");
    TestCrackFromCurveEndingInsideAFace();
    std::printf("TestUnreachedComponentIsReported\n");
    TestUnreachedComponentIsReported();
    TestCatmullRomTopologyAndWeights();

    if (failures == 0) {
        std::printf("testRigExecCurvenet: OK\n");
        return 0;
    }
    std::printf("testRigExecCurvenet: %d failure(s)\n", failures);
    return 1;
}

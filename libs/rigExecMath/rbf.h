//
// RigExec radial-basis pose interpolation -- the conventional poseInterpolator, and a
// A native port of the radial-basis pose solver.
//
// A pose interpolator answers one question: given where a driver joint is
// NOW, how much does each authored pose count? This answers it with a radial
// basis function, and this is the same maths, so the weights match the authored data rather than merely resembling it.
//
// Solved once, evaluated many times. RigExecRbfSolver::Solve builds the
// matrix of every pose's kernel value at every other pose and inverts it;
// what comes out is a constant, and the per-frame work is then a multiply
// and a divide against that constant. The split is the same and it is
// why an interpolator costs so little to evaluate.
//
// WHY THIS IS C++ AND NOT THE PYTHON IT CAME FROM
//   * The per-frame evaluation runs inside the evaluator's pose phase, which
//     is C++. There is no seam to call Python across, and a GIL acquire on
//     that path is what deadlocked Compile().
//   * The converter would otherwise import the reference solver, putting
//     a hard dependency on a sibling repository into the rig build. The
//     project's rule (stated for the .shp reader) is that a fresh clone must
//     work.
//   * The shape editor re-fits a pose's falloff live, which needs the fitter
//     native too.
//
// The Python stays as the ORACLE, frozen into tests/fixtures/psd_parity.json
// by tools/biped/gen_psd_parity.py and asserted at 1e-6 by
// tests/testRigExecRbf.cpp over all 67 solvable interpolators of the shipped
// biped. Where a comment below cites `rbf.py:NNN-NNN`, that is the original
// reasoning and it is worth reading before changing anything.
//
// CONVENTIONS (state them, then trust them)
//
//   * Angles in radians, translations in METRES (the authored data is in centimetres;
//     the converter scales on the way in). Pose rotations arrive as XYZ
//     eulers, which is how the authored data is read and how the fixture stores
//     them.
//
//   * Quaternions are GfQuatd (real, imaginary), Hamilton product -- the
//     same product the Python writes out by hand at rbf.py:1073-1076, so
//     `q * twist.GetConjugate()` is the swing there and here. Verified
//     component for component; see the note in RigExecRbfSwingTwist.
//
//   * WHERE Gf DIFFERS FROM THE PYTHON, and it matters in exactly two
//     places:
//
//     1. Euler -> quaternion. GfRotation composes with
//        `GfRotation(Z,z) * GfRotation(Y,y) * GfRotation(X,x)` under the
//        row-vector convention this codebase uses everywhere (p' = p*M),
//        which is the REVERSE spelling of the column-vector composition the
//        Python's closed form encodes. Rather than rely on getting that
//        reversal right, RigExecRbfQuaternionFromEuler reproduces
//        rbf.py:1021-1027's closed form term for term. It is the same
//        rotation either way, but only the closed form is the same
//        FLOATING-POINT VALUE, and the fixture is compared at 1e-6 through
//        a matrix inverse that amplifies.
//
//     2. Angle between two rotations. GfRotation(q1.GetInverse()*q2)
//        .GetAngle() takes the quaternion product and an atan2; the Python
//        takes the 4-vector dot and 2*acos(min(1,|dot|)) (rbf.py:1080-1091).
//        The two agree analytically and not bit for bit, and the |dot|
//        (rather than a signed clamp) is load-bearing: it takes the short
//        way round the double cover, without which poses more than a half
//        turn apart measure as though they were close. The dot form is what
//        is implemented here.
//
//     Nothing else in this file needs a convention decision: the swing/twist
//     split, the metric, the matrix build and the Gauss-Jordan inverse are
//     pure arithmetic on components and are reproduced in the Python's own
//     operation order, because a reassociated sum is not the same double and
//     a near-singular inverse magnifies the difference.
//
//   * No matrices appear here at all, so the row-vector convention never
//     comes up. If one is ever added, note that GfMatrix4d::SetRotate(q)
//     builds M with p*M == q.Transform(p), so (q1*q2) corresponds to
//     M(q2)*M(q1) and NOT to M(q1)*M(q2).
//
#ifndef RIGEXEC_MATH_RBF_H
#define RIGEXEC_MATH_RBF_H

#include "pxr/pxr.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/vec3d.h"

#include <array>
#include <cstddef>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The kernels.
///
/// Gaussian falls away smoothly and never quite reaches zero, which makes a
/// soft blend that keeps a little of every pose everywhere. Linear falls
/// away in a straight line and reaches zero AT the radius -- compact
/// support, so past the radius its value is not small, it is nothing, which
/// is what makes RigExecRbfSolver::Coverage worth measuring.
///
/// The `interpolation` value: 0 is Linear, 1 is Gaussian
///.
enum class RigExecRbfKernel {
    Gaussian = 0,
    Linear = 1,
};

/// `poseType`.
///
/// A swing pose is measured by the swing part of the driver's rotation and
/// a twist pose by the twist part, because a driver usually carries both at
/// once and they mean different things: a neck that has twisted has not
/// bent, and its bend shapes should stay at zero. Measuring the whole
/// rotation instead leaks about 0.05 of every swing pose into a pure twist
/// on the shipped biped (rbf.py:1030-1038).
///
/// The metric therefore belongs to the POSE, not to the interpolator.
enum class RigExecRbfPoseType {
    Whole = 0,
    Swing = 1,
    Twist = 2,
};

/// Below this a matrix counts as singular and the solve is regularised.
/// rbf.py:69.
constexpr double RigExecRbfSingular = 1.0e-12;

/// The least total kernel an interpolator may have anywhere between its
/// poses. Below this the weights are being decided by a sum near zero, and
/// at zero every shape switches off -- a dead zone. rbf.py:217-220.
constexpr double RigExecRbfCoverageFloor = 0.5;

/// Below this the normalisation is refused: there is nothing meaningful to
/// divide by, and scaling by an almost-zero divisor turns rounding into a
/// huge weight. rbf.py:964-987.
constexpr double RigExecRbfNormalizeFloor = 1.0e-6;

// ---------------------------------------------------------------------------
// Kernels and the metric
// ---------------------------------------------------------------------------

/// A bell falling away from a pose: exp(-(distance/radius)^2).
/// A radius of zero or less answers 1 at the pose and 0 everywhere else.
/// rbf.py:78-91.
double RigExecRbfGaussian(double distance, double radius);

/// A straight line falling to zero at the radius: max(0, 1 - distance/radius).
/// rbf.py:94-106.
double RigExecRbfLinear(double distance, double radius);

/// Either kernel, by enum.
double RigExecRbfEvalKernel(RigExecRbfKernel kernel, double distance,
                            double radius);

/// One unit-less distance from a rotation gap and a translation gap.
///
///     d^2 = enR * (angle / width)^2  +  enT * (gap / translationWidth)^2
///
/// Radians and metres cannot be added, so each channel is divided by its OWN
/// falloff first and the two are combined as a right-angled triangle. What
/// comes out is in units of "one falloff", which is why the kernel is then
/// evaluated with a radius of exactly 1.
///
/// THE ROTATION-ONLY CASE IS THE BARE RATIO, not sqrt(x*x). With
/// enableTranslation off this returns `angle / width` and nothing else,
/// because that is the float the kernels were handed before the translation
/// channel existed and every table already shipped was solved with it.
/// Routing it through a square and a root would be right to within an ulp
/// and wrong as a promise (rbf.py:161-169).
///
/// Returns infinity when a channel has no width at all and the driver is not
/// sitting exactly on the pose; both kernels take that to zero -- exp(-inf)
/// and max(0, 1-inf) -- which is the same answer a zero-width branch would
/// give by hand, without the branch (rbf.py:71-75).
double RigExecRbfCombine(double angle, double width, double gap,
                         double translationWidth, bool enableRotation,
                         bool enableTranslation);

// ---------------------------------------------------------------------------
// Rotations
// ---------------------------------------------------------------------------

/// An XYZ euler (radians) as a quaternion, reproducing rbf.py:1009-1027's
/// closed form term for term. See the conventions note at the top of this
/// file for why this is not spelled with GfRotation.
GfQuatd RigExecRbfQuaternionFromEuler(const GfVec3d &euler);

/// A quaternion as an XYZ euler, the inverse of the above and the exact
/// arithmetic of rbf.py:1130-1146 (including the copysign branch at the
/// gimbal pole).
GfVec3d RigExecRbfEulerFromQuaternion(const GfQuatd &quaternion);

/// The angle between two quaternions, the short way round the double cover:
/// 2 * acos(min(1, |dot|)). rbf.py:1080-1091.
double RigExecRbfAngleBetween(const GfQuatd &first, const GfQuatd &second);

/// Split a rotation into its twist about an axis and the rest.
///
/// The twist is the part of the rotation that spins about `axis`; the swing
/// is what is left, and takes the axis itself to where it ends up. A half
/// turn perpendicular to the axis has no twist at all and answers identity
/// there. rbf.py:1041-1077.
///
/// `axis` is normalised internally; a zero-length axis is treated as unit
/// length, matching the Python's `or 1.0` guard.
void RigExecRbfSwingTwist(const GfQuatd &quaternion, const GfVec3d &axis,
                          GfQuatd *swing, GfQuatd *twist);

/// A rotation part way between two, as an XYZ euler -- the path a driver
/// actually takes. rbf.py:1094-1127.
GfVec3d RigExecRbfSlerpEuler(const GfVec3d &first, const GfVec3d &second,
                             double amount);

/// Invert a square matrix by Gauss-Jordan with partial pivoting.
///
/// By hand rather than through a library for the same reason the Python is:
/// these matrices are one row per POSE -- two to fifteen on the shipped
/// biped -- so there is nothing to gain, and the pivoting order is part of
/// what the fixture pins. Returns false when the matrix is singular, leaving
/// `inverse` untouched. rbf.py:1149-1187.
bool RigExecRbfInvert(const std::vector<std::vector<double>> &matrix,
                      std::vector<std::vector<double>> *inverse);

// ---------------------------------------------------------------------------
// The solver
// ---------------------------------------------------------------------------

/// Everything an interpolator is built from. Defaults match the Python's.
struct RigExecRbfSolverDesc {
    /// The authored poses, XYZ eulers in radians. One per pose.
    std::vector<GfVec3d> poses;

    /// The poses' translations, in METRES, in the driver's own frame. One
    /// per pose, same order. `poseTranslation`, converted. Only read
    /// when `enableTranslation` is on -- and `enableTranslation` is forced
    /// off when this is empty, because an interpolator with no translations
    /// has nothing to say about translation and saying "every pose is
    /// equally close" would peg its weights at 1/n (rbf.py:431-436).
    std::vector<GfVec3d> translations;

    RigExecRbfKernel kernel = RigExecRbfKernel::Gaussian;

    /// The rotation falloff width in radians, shared by every pose. Zero
    /// measures one from the poses themselves -- the MEAN distance to a
    /// nearest neighbour, which is the width at which neighbouring poses
    /// just meet. rbf.py:552-576.
    double radius = 0.0;

    /// The same for the translation channel, in metres. Zero measures one
    /// from the translations. It is NOT the rotation radius and must not be
    /// borrowed from it: a brow's poses are millimetres apart and a rotation
    /// width would swallow all of them. rbf.py:578-606.
    double translationRadius = 0.0;

    /// the conventional per-pose `poseFalloff`, one per pose, or empty for a shared
    /// radius. The falloff is stated RELATIVE to each pose's own closest
    /// neighbour, so with these each pose gets its own width and an unevenly
    /// spread set stops being judged by one average. 0.3 is the default
    /// and means "just reach the neighbour". rbf.py:367-377, 608-631.
    std::vector<double> falloffs;

    /// `poseType` per pose, or empty to measure whole rotations.
    std::vector<RigExecRbfPoseType> poseTypes;

    /// The axis a twist pose spins about, in the driver's own frame. the conventional tool's
    /// `driverTwistAxis`: 0 X, 1 Y, 2 Z.
    GfVec3d twistAxis{0.0, 1.0, 0.0};

    /// Added to the matrix diagonal before inverting, the conventional tool's
    /// `regularization`. Trades exactness at the poses for a calmer result
    /// between them, and rescues a solve whose poses are so close that the
    /// matrix is singular.
    double regularization = 0.0;

    /// Divide the weights by their sum, so they stay a partition of unity.
    /// The weights are normalised.
    bool normalize = true;

    /// `enableRotation` / `enableTranslation`. On the shipped biped
    /// they are mutually exclusive -- 39 interpolators are rotation, 28 are
    /// translation -- and honoured rather than assumed: reading a
    /// translation interpolator as a rotation makes all of its poses
    /// identity, so it solves degenerate and drives nothing.
    bool enableRotation = true;
    bool enableTranslation = false;
};

/// A pose interpolator: poses in, weights out.
class RigExecRbfSolver {
public:
    RigExecRbfSolver() = default;
    explicit RigExecRbfSolver(const RigExecRbfSolverDesc &desc);

    /// Invert the pose matrix, which is the whole of the solve. Idempotent;
    /// Evaluate calls it if it has not been called. Returns false only when
    /// there are no poses at all.
    bool Solve();

    /// The kernel value of a driver pose against every authored pose. This
    /// is the only part that has to happen per frame.
    void Kernels(const GfVec3d &euler, const GfVec3d *translation,
                 std::vector<double> *out) const;

    /// How much each authored pose counts, for a driver pose.
    ///
    /// `allowNegativeWeights` is the conventional flag, and the clamp it selects is
    /// max(0, w) applied AFTER normalisation, never before and never inside
    /// the solve. Negative weights are not a
    /// special case: they fall out of the inverse, and clamping them is what
    /// produces the soft, muddy blend that makes a pose fail to reach its
    /// own shape -- so the default is to keep them.
    void Evaluate(const GfVec3d &euler, const GfVec3d *translation,
                  std::vector<double> *out,
                  bool allowNegativeWeights = true) const;

    /// Divide weights by their SUM, keeping their signs -- not by the sum of
    /// magnitudes, because a negative weight is a real instruction to lean
    /// away from a pose. Where |sum| < RigExecRbfNormalizeFloor there is
    /// nothing to normalise against and the weights are left alone, which is
    /// what falling away to nothing outside the poses should look like.
    /// rbf.py:964-987.
    void Normalize(std::vector<double> *weights) const;

    /// How far a rotation is from one authored pose, in radians, measured
    /// the way THAT pose is measured (whole / swing / twist).
    /// rbf.py:462-483.
    double Distance(const GfQuatd &quaternion, size_t index) const;

    /// Euclidean distance from a position to one authored pose, in metres.
    /// There is no per-axis weighting here.
    double TranslationDistance(const GfVec3d *translation,
                               size_t index) const;

    /// How far the driver is from one pose, in units of THAT pose's falloff.
    double Ratio(const GfQuatd &quaternion, size_t index,
                 const GfVec3d *translation) const;

    /// Every pose's kernel value at every other pose. Row r is exactly what
    /// Kernels() returns with the driver standing on pose r -- and with
    /// per-pose widths the matrix is NOT symmetric, which is what makes the
    /// transpose in Solve() load-bearing.
    std::vector<std::vector<double>> Matrix() const;

    /// Whether the poses are too alike to tell apart AT ALL, under the
    /// channels that are turned on. Not "nearly singular" -- actually
    /// coincident. Regularising one of these anyway "succeeds", returns
    /// weights around 1e11, and evaluates to a flat 1/n forever.
    /// rbf.py:804-837.
    bool Degenerate() const;

    /// Each pose's distance to its own nearest neighbour, in radians.
    std::vector<double> Spacing() const;

    /// The same over the translations, in metres.
    std::vector<double> TranslationSpacing() const;

    /// The lowest total kernel value anywhere between the poses, sampled
    /// along every ordered pair. Zero is a DEAD ZONE: every shape switches
    /// off there and snaps back on as the driver leaves, which is what
    /// popping usually is and is invisible to any check made AT the poses.
    /// rbf.py:736-765.
    double Coverage(int steps = 8) const;

    /// How far past 0..1 the weights reach between the poses.
    /// rbf.py:767-787.
    double Overshoot(int steps = 8) const;

    /// Where the driver is part way from one pose to another: the rotation
    /// slerps and the translation lerps, which is the path a driver actually
    /// takes. rbf.py:693-718.
    void Walk(size_t first, size_t second, double amount, GfVec3d *euler,
              GfVec3d *translation) const;

    // -- the solved constants -------------------------------------------
    const std::vector<GfVec3d> &GetPoses() const { return _poses; }
    const std::vector<GfVec3d> &GetTranslations() const {
        return _translations;
    }
    double GetRadius() const { return _radius; }
    double GetTranslationRadius() const { return _translationRadius; }
    const std::vector<double> &GetRadii() const { return _radii; }
    const std::vector<double> &GetTranslationRadii() const {
        return _translationRadii;
    }
    /// One row per pose; constant once solved. Empty until Solve().
    const std::vector<std::vector<double>> &GetWeights() const {
        return _weights;
    }
    bool GetEnableRotation() const { return _enableRotation; }
    bool GetEnableTranslation() const { return _enableTranslation; }
    RigExecRbfKernel GetKernel() const { return _kernel; }
    bool GetNormalize() const { return _normalize; }
    /// True when Solve() had to fall back on the RigExecRbfSingular nudge
    /// because the matrix was singular outright. rbf.py:908-919.
    bool GetRegularizedSingular() const { return _regularizedSingular; }

    /// Adopt an already-solved table: the per-pose widths and the inverted
    /// matrix, exactly as a converted rig stores them.
    ///
    /// This is the RUNTIME path. A rig loaded from disk carries the solve as
    /// data -- Solve() ran once at conversion time and its answer is a
    /// constant -- so the per-frame side must be able to reconstitute an
    /// interpolator without re-deriving anything. Re-deriving would also be
    /// wrong: the widths in a shipped table may have been scaled by a painted
    /// poseFalloff after the fit, and there is
    /// no falloff vector that reproduces them.
    ///
    /// The desc's `radius` and `translationRadius` still carry the shared
    /// widths; these are the per-pose ones, either of which may be empty.
    void SetSolvedTable(const std::vector<double> &radii,
                        const std::vector<double> &translationRadii,
                        const std::vector<std::vector<double>> &weights);

    /// Rescale both channels' widths by a factor -- the "painted share"
    /// poseFalloff carries on top of a fitted width
    ///. Call before Solve().
    void ScaleWidths(double factor);

private:
    double _Kernel(double ratio) const;
    double _Width(size_t index) const;
    double _TranslationWidth(size_t index) const;
    const GfVec3d *_Translation(size_t index) const;
    double _MeasureRadius() const;
    double _MeasureTranslationRadius() const;
    std::vector<double> _MeasureRadii(const std::vector<double> &falloffs)
        const;
    std::vector<double> _MeasureTranslationRadii(
        const std::vector<double> &falloffs) const;

    std::vector<GfVec3d> _poses;
    std::vector<GfVec3d> _translations;
    /// Each pose as (whole, swing, twist), split once rather than per call.
    std::vector<std::array<GfQuatd, 3>> _parts;
    std::vector<RigExecRbfPoseType> _poseTypes;
    GfVec3d _twistAxis{0.0, 1.0, 0.0};
    RigExecRbfKernel _kernel = RigExecRbfKernel::Gaussian;
    double _radius = 0.0;
    double _translationRadius = 0.0;
    std::vector<double> _radii;
    std::vector<double> _translationRadii;
    double _regularization = 0.0;
    bool _normalize = true;
    bool _enableRotation = true;
    bool _enableTranslation = false;
    bool _regularizedSingular = false;
    std::vector<std::vector<double>> _weights;
};

/// What RigExecRbfFitWidth settled on.
struct RigExecRbfFitReport {
    double width = 0.0;             ///< the rotation falloff it chose
    double translationWidth = 0.0;  ///< and the translation one
    bool perPose = false;           ///< per-pose widths or one shared
    double scale = 0.0;             ///< the sweep step that won (0 = none)
    double coverage = 0.0;          ///< the worst kernel sum between poses
    double overshoot = 0.0;         ///< how far past 0..1 the weights went
};

/// Choose an interpolator's falloff width by what it has to do.
///
/// The width the fit solved with is not carried in the data. It exports `poseFalloff`,
/// which its own documentation calls a share "relative to the closest other
/// pose", and a `poseRotationFalloff` it ignores for every non-independent
/// pose -- which is all of them on this rig. So the SHAPE of the rule is
/// the conventional and the size is not recoverable, and one scale for the whole rig
/// does not work because the pose layouts are not alike: the thigh has poses
/// 25 to 150 degrees apart, the shoulder has eight poses with pairs 45
/// apart, the index has three in a line. Narrow enough for the shoulder
/// leaves the thigh with a hole in the middle of every swing; wide enough
/// for the thigh sends the shoulder's weights to 9.
///
/// So the width is fitted per interpolator, to two requirements, in order:
///
///   1. NO DEAD ZONE. The kernels must still sum to at least `floor`
///      everywhere between the poses. Not negotiable -- at zero every shape
///      switches off and snaps back, which is the pop.
///   2. THE LEAST OVERSHOOT among the widths that manage it, so the weights
///      stay inside 0..1 and shapes do not push past themselves.
///
/// Ties go to the wider width, which blends more smoothly. The translation
/// channel is fitted the same way and separately, but by the SAME swept
/// scale: the metric already divides each channel by its own falloff, so the
/// shape of the pose space is fixed by the ratio between the two widths and
/// only its overall size is free. rbf.py:223-352.
///
/// Calibration, for the reader who wants the numbers: measured on the biped,
/// summing how far past 0..1 the weights reach between every pair of poses,
/// a fixed scale of 0.5 gives 0.81, 0.7 gives 2.88 and 1.0 gives 5.10
///. It falls all the way down, so there is no
/// interior best -- which is exactly why the converter stopped using a fixed
/// scale and calls this instead. A painted poseFalloff other than the default 0.3
/// default is applied on top of the fitted width, as poseFalloff/0.3, by
/// RigExecRbfSolver::ScaleWidths.
///
/// \param desc the interpolator. `radius`, `translationRadius` and
///        `falloffs` are ignored: fitting them is the point.
/// \param report filled in with what was chosen, when non-null.
/// \return the fitted, SOLVED solver.
RigExecRbfSolver RigExecRbfFitWidth(const RigExecRbfSolverDesc &desc,
                                    RigExecRbfFitReport *report = nullptr,
                                    double floor = RigExecRbfCoverageFloor);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_RBF_H

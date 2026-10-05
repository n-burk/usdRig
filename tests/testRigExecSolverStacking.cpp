// ONE hierarchical pose stack over aggregate solvers AND frame constraints.
// rigExec:joints is an ordered WRITE, not an exclusive claim: two or more
// solvers may name one joint, and a pose constraint that moves a joint is a
// step of the SAME KIND in the SAME stack. All of them run, their commits
// serialize into one chain per joint, and the LAST step in that chain supplies
// the joint's final frame.
// The order is the reverse composed pre-order of the WHOLE RIG -- bottom
// sibling first, a parent after its descendants, the rule the mover stack
// already used -- and NOTHING ELSE breaks a tie. A constraint BELOW a solver
// therefore runs first and FEEDS it (the frame it leaves becomes that solver's
// rest reference); a constraint ABOVE it revises the solver's output. A frame
// read below its writer is POSITIONAL and reads the earlier version; an
// AGGREGATE read cannot be resolved that way, so a solver whose aggregate
// another solver reads is a PRODUCER with no stack position at all and is
// scheduled by data flow.
// The order decides the RELATIVE order of two steps that touch the same
// joint, and nothing else: two limbs that share no joint share a Kahn level
// and evaluate concurrently (TestUnrelatedLimbsShareALevel).
// Every fixture here is authored as .usda TEXT rather than built prim by
// prim, because the thing under test is COMPOSED ORDER: `reorder nameChildren`
// is a layer opinion, and authoring it the way a rigger does is the only way
// the test exercises what a rigger would hit.
// argv[1] = path to the examples directory (for the schema plugin).
#include "rigExecFrameRecordCheck.h"
#include "rigExecOpTrace.h"
#include "rigExecPoseCompare.h"

#include "rigExec/bakedProgram.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <cstdio>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

const SdfPath kRigPath("/Asset/Rig");
const SdfPath kHip("/Asset/Rig/Joints/Hip");
const SdfPath kKnee("/Asset/Rig/Joints/Hip/Knee");
const SdfPath kAnkle("/Asset/Rig/Joints/Hip/Knee/Ankle");
const SdfPath kLegFk("/Asset/Rig/Solvers/LegFK");
const SdfPath kLegIk("/Asset/Rig/Solvers/LegIK");
const SdfPath kProbePoints("/Asset/Geom/Probe.points");

// The fixture.

/// Which solvers the stage carries and how they are wired.
struct RigSpec {
    /// `reorder nameChildren` on /Asset/Rig/Solvers. Empty authors none, so
    /// the composed order is definition order: LegIK, then LegFK -- whose
    /// REVERSE is the stack, so FK writes first and IK writes last.
    std::vector<std::string> solverOrder;
    bool ik = true;
    bool fk = true;
    /// Author the IK's root/effector/pole controls. False leaves it bound to
    /// the joints and wired to nothing, so it publishes an empty aggregate
    /// and every joint it names falls back -- the shape the per-joint verdict
    /// below is about.
    bool ikWired = true;
    /// A SECOND FK chain over the same three joints, so the stack is three
    /// deep rather than two.
    bool fk2 = false;
    /// An aim constraint that moves the knee. A pose constraint carries no
    /// read phase, so it observes whatever stands at ITS OWN place in the
    /// stack.
    bool kneeAim = false;
    /// A position constraint that moves the knee onto a displaced control.
    /// Below the solvers it FEEDS them; above them it revises their output.
    bool kneeMove = false;
    /// `reorder nameChildren` on /Asset/Rig. Empty authors none, so the
    /// composed order is definition order -- Controls, Joints, Solvers,
    /// Movers -- which puts MOVERS AT THE BOTTOM and therefore runs every
    /// constraint BEFORE every solver. SolversLast() below is the other
    /// shape: the classic "solve, then revise" every shipped rig authors.
    std::vector<std::string> rigOrder;
    /// The FK chain's controls and the joints it writes. The defaults name
    /// the same three joints the IK does, which is the stack under test.
    std::string fkControls =
        "</Asset/Rig/Controls/FkHip>, "
        "</Asset/Rig/Controls/FkHip/FkKnee>, "
        "</Asset/Rig/Controls/FkHip/FkKnee/FkAnkle>";
    std::string fkJoints =
        "</Asset/Rig/Joints/Hip>, "
        "</Asset/Rig/Joints/Hip/Knee>, "
        "</Asset/Rig/Joints/Hip/Knee/Ankle>";
    /// A three-point mesh carried by a RigExecMatrixMover that reads the knee
    /// at this read phase. Empty authors neither the mesh nor the mover.
    std::string probePhase;
};

std::string
Rest(double x, double y, double z)
{
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer),
                  "( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), "
                  "(%g, %g, %g, 1) )",
                  x, y, z);
    return buffer;
}

std::string
RigText(const RigSpec &spec)
{
    std::string text =
        "#usda 1.0\n"
        "(\n"
        "    defaultPrim = \"Asset\"\n"
        "    startTimeCode = 1\n"
        "    endTimeCode = 5\n"
        "    metersPerUnit = 0.01\n"
        "    timeCodesPerSecond = 24\n"
        "    upAxis = \"Y\"\n"
        ")\n"
        "\n"
        "def Xform \"Asset\"\n"
        "{\n"
        "    def RigExecRoot \"Rig\"\n"
        "    {\n"
        "        uniform token rigExec:partition = \"Asset\"\n";
    if (!spec.rigOrder.empty()) {
        text += "        reorder nameChildren = [";
        for (size_t i = 0; i < spec.rigOrder.size(); ++i) {
            text += (i ? ", \"" : "\"") + spec.rigOrder[i] + "\"";
        }
        text += "]\n";
    }
    text +=
        "\n"
        "        def Scope \"Controls\"\n"
        "        {\n"
        "            def RigExecControl \"HipRoot\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
        "            }\n"
        "            def RigExecControl \"FootIK\"\n"
        "            {\n"
        "                double avars:ty.timeSamples = {\n"
        "                    1: 0,\n"
        "                    3: 1.5,\n"
        "                    5: 0.5,\n"
        "                }\n"
        "                matrix4d rest:space = " + Rest(0, 0, 0) + "\n"
        "            }\n"
        "            def RigExecControl \"KneePole\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 4, 3) + "\n"
        "            }\n"
        "            def RigExecControl \"FkHip\"\n"
        "            {\n"
        "                double avars:rz.timeSamples = {\n"
        "                    1: 0,\n"
        "                    5: 30,\n"
        "                }\n"
        "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
        "\n"
        "                def RigExecControl \"FkKnee\"\n"
        "                {\n"
        "                    matrix4d rest:space = " + Rest(4, 0, 0) + "\n"
        "\n"
        "                    def RigExecControl \"FkAnkle\"\n"
        "                    {\n"
        "                        matrix4d rest:space = " + Rest(4, 0, 0) +
        "\n"
        "                    }\n"
        "                }\n"
        "            }\n";
    if (spec.fk2) {
        text +=
            "            def RigExecControl \"Fk2Hip\"\n"
            "            {\n"
            "                double avars:rz.timeSamples = {\n"
            "                    1: 0,\n"
            "                    5: -20,\n"
            "                }\n"
            "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
            "\n"
            "                def RigExecControl \"Fk2Knee\"\n"
            "                {\n"
            "                    matrix4d rest:space = " + Rest(4, 0, 0) +
            "\n"
            "\n"
            "                    def RigExecControl \"Fk2Ankle\"\n"
            "                    {\n"
            "                        matrix4d rest:space = " + Rest(4, 0, 0) +
            "\n"
            "                    }\n"
            "                }\n"
            "            }\n";
    }
    if (spec.kneeAim) {
        text +=
            "            def RigExecControl \"AimTarget\"\n"
            "            {\n"
            "                matrix4d rest:space = " + Rest(0, 20, 6) + "\n"
            "            }\n";
    }
    if (spec.kneeMove) {
        text +=
            "            def RigExecControl \"KneeTarget\"\n"
            "            {\n"
            "                matrix4d rest:space = " + Rest(4, 10, 2) + "\n"
            "            }\n";
    }
    text +=
        "        }\n"
        "\n"
        "        def Scope \"Joints\"\n"
        "        {\n"
        "            def RigExecJoint \"Hip\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
        "\n"
        "                def RigExecJoint \"Knee\"\n"
        "                {\n"
        "                    matrix4d rest:space = " + Rest(4, 0, 0) + "\n"
        "\n"
        "                    def RigExecJoint \"Ankle\"\n"
        "                    {\n"
        "                        matrix4d rest:space = " + Rest(4, 0, 0) +
        "\n"
        "                    }\n"
        "                }\n"
        "            }\n"
        "        }\n"
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n";
    if (!spec.solverOrder.empty()) {
        text += "            reorder nameChildren = [";
        for (size_t i = 0; i < spec.solverOrder.size(); ++i) {
            text += (i ? ", \"" : "\"") + spec.solverOrder[i] + "\"";
        }
        text += "]\n\n";
    }
    if (spec.ik) {
        text +=
            "            def RigExecTwoBoneIk \"LegIK\"\n"
            "            {\n";
        if (spec.ikWired) {
            text +=
                "                rel rigExec:rootControl = "
                "</Asset/Rig/Controls/HipRoot>\n"
                "                rel rigExec:effectorControl = "
                "</Asset/Rig/Controls/FootIK>\n"
                "                rel rigExec:poleControl = "
                "</Asset/Rig/Controls/KneePole>\n";
        }
        text +=
            "                rel rigExec:joints = [\n"
            "                    </Asset/Rig/Joints/Hip>,\n"
            "                    </Asset/Rig/Joints/Hip/Knee>,\n"
            "                    </Asset/Rig/Joints/Hip/Knee/Ankle>,\n"
            "                ]\n"
            "                double rigExec:preferredBendRadians = 0.3\n"
            "            }\n";
    }
    if (spec.fk) {
        text +=
            "            def RigExecFkChain \"LegFK\"\n"
            "            {\n"
            "                rel rigExec:controls = [ " + spec.fkControls +
            " ]\n"
            "                rel rigExec:joints = [ " + spec.fkJoints +
            " ]\n"
            "            }\n";
    }
    if (spec.fk2) {
        text +=
            "            def RigExecFkChain \"LegFK2\"\n"
            "            {\n"
            "                rel rigExec:controls = [ "
            "</Asset/Rig/Controls/Fk2Hip>, "
            "</Asset/Rig/Controls/Fk2Hip/Fk2Knee>, "
            "</Asset/Rig/Controls/Fk2Hip/Fk2Knee/Fk2Ankle> ]\n"
            "                rel rigExec:joints = [ "
            "</Asset/Rig/Joints/Hip>, "
            "</Asset/Rig/Joints/Hip/Knee>, "
            "</Asset/Rig/Joints/Hip/Knee/Ankle> ]\n"
            "            }\n";
    }
    text +=
        "        }\n";
    if (!spec.probePhase.empty() || spec.kneeAim || spec.kneeMove) {
        text +=
            "\n"
            "        def Scope \"Movers\"\n"
            "        {\n";
    }
    if (spec.kneeAim) {
        text +=
            "            def RigExecAimConstraint \"KneeAim\" (\n"
            "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
            "            )\n"
            "            {\n"
            "                float inputs:defaultWeight = 1\n"
            "                uniform token rigExec:aimAxis = \"x\"\n"
            "                rel rigExec:aimTarget = "
            "</Asset/Rig/Controls/AimTarget>\n"
            "                rel rigExec:moves = "
            "</Asset/Rig/Joints/Hip/Knee>\n"
            "                uniform token[] rigExec:preserve = "
            "[\"origin\", \"scale\"]\n"
            "            }\n";
    }
    if (!spec.probePhase.empty()) {
        text +=
            "            def RigExecMatrixMover \"KneeProbe\" (\n"
            "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
            "            )\n"
            "            {\n"
            "                float inputs:defaultWeight = 1\n"
            "                rel rigExec:moves = </Asset/Geom/Probe.points>\n"
            "                rel rigExec:transform = "
            "</Asset/Rig/Joints/Hip/Knee> (\n"
            "                    rigExecReadPhase = \"" + spec.probePhase +
            "\"\n"
            "                )\n";
        text +=
            "            }\n";
    }
    // LAST in the composed order, so it is the BOTTOM sibling and runs
    // FIRST of the movers -- which leaves the probe above it, where a
    // `final` read of the knee is legal and AtPrim(Movers) means "after
    // this constraint".
    if (spec.kneeMove) {
        text +=
            "            def RigExecPositionConstraint \"KneeMove\" (\n"
            "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
            "            )\n"
            "            {\n"
            "                float inputs:defaultWeight = 1\n"
            "                rel rigExec:sources = "
            "</Asset/Rig/Controls/KneeTarget>\n"
            "                rel rigExec:moves = "
            "</Asset/Rig/Joints/Hip/Knee>\n"
            "            }\n";
    }
    if (!spec.probePhase.empty() || spec.kneeAim || spec.kneeMove) {
        text +=
            "        }\n";
    }
    text +=
        "    }\n";
    if (!spec.probePhase.empty()) {
        text +=
            "\n"
            "    def Scope \"Geom\"\n"
            "    {\n"
            "        def Mesh \"Probe\"\n"
            "        {\n"
            "            int[] faceVertexCounts = [3]\n"
            "            int[] faceVertexIndices = [0, 1, 2]\n"
            "            point3f[] points = [(0, 0, 0), (1, 0, 0), "
            "(0, 0, 1)]\n"
            "            uniform token subdivisionScheme = \"none\"\n"
            "        }\n"
            "    }\n";
    }
    text += "}\n";
    return text;
}

UsdStageRefPtr
MakeStage(const RigSpec &spec)
{
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    if (!layer || !layer->ImportFromString(RigText(spec))) {
        std::printf("FAIL: the stacking fixture does not parse\n");
        ++failures;
        return UsdStageRefPtr();
    }
    return UsdStage::Open(layer);
}

/// The stacked rig: an FK chain and a two-bone IK naming the same three
/// joints, neither consumed by the other, so both write.
RigSpec
Stacked()
{
    return RigSpec();
}

/// The rig-root order that puts Solvers at the BOTTOM, so every solver runs
/// before every constraint -- the classic "solve, then revise" shape, and the
/// one every shipped rig authors.
std::vector<std::string>
SolversLast()
{
    return {"Controls", "Joints", "Movers", "Solvers"};
}

// Small helpers.

bool
Near(const GfVec3d &a, const GfVec3d &b, double tolerance)
{
    return (a - b).GetLength() <= tolerance;
}

/// Whether two posed frames agree to the suite's usual tolerance.
bool
SameFrame(const RigExecPointFrame &a, const RigExecPointFrame &b)
{
    if (a.points.size() != b.points.size()) {
        return false;
    }
    for (size_t i = 0; i < a.points.size(); ++i) {
        if (!Near(GfVec3d(a.points[i]), GfVec3d(b.points[i]), 1e-5)) {
            return false;
        }
    }
    return true;
}

bool
SameJoint(const RigExecRigPose &a, const RigExecRigPose &b,
          const SdfPath &joint)
{
    const auto ia = a.jointFramesFinal.find(joint);
    const auto ib = b.jointFramesFinal.find(joint);
    if (ia == a.jointFramesFinal.end() || ib == b.jointFramesFinal.end()) {
        return false;
    }
    return SameFrame(ia->second, ib->second);
}

/// Compiles \p stage and returns its pose at \p time, reporting any compile
/// failure with the messages that caused it.
///
/// \p parity drives the scalar CPU oracle. It is OFF for the checkpoint
/// fixtures and only for them: the oracle's RigExecMatrixMover branch reads
/// rigExec:transform's rigExecReadPhase as "base" or "final" only -- it has
/// no AtPrim branch, so ANY AtPrim transform phase, solver-named or
/// constraint-named, reads the base matrix there and disagrees with the
/// graph. No shipped rig authors an AtPrim transform phase; the baked parity
/// harness below still judges every stacking fixture that carries no such
/// phase.
RigExecRigPose
Evaluate(const char *what, const UsdStageRefPtr &stage, double time,
         std::vector<std::string> *errors, bool parity = true,
         std::map<SdfPath, std::vector<SdfPath>> *chains = nullptr)
{
    std::vector<std::string> local;
    std::vector<std::string> &out = errors ? *errors : local;
    RigExecRigPose pose;
    if (!stage) {
        return pose;
    }
    RigExecRigEvaluator evaluator(stage, kRigPath);
    evaluator.cpuParityMode = parity;
    if (!evaluator.Compile(&out)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : out) {
            std::printf("    %s\n", error.c_str());
        }
        return pose;
    }
    if (chains) *chains = evaluator.GetFrameChains();
    pose = evaluator.Evaluate(UsdTimeCode(time));
    if (!pose.valid) {
        ++failures;
        std::printf("FAIL %s: the pose at %g is not valid\n", what, time);
        for (const std::string &diagnostic : pose.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
    }
    return pose;
}

/// \p joint's interleaved writer chain, rendered as "a, b, c".
///
/// The compile says NOTHING about stacking -- it is ordinary authoring under
/// the unified pose stack, not a shape worth a diagnostic -- so the order is
/// read from the compiled chains, which is the very list the stack edges are
/// built from and the one the pose walk runs.
std::string
ChainText(const std::map<SdfPath, std::vector<SdfPath>> &chains,
          const SdfPath &joint)
{
    const auto it = chains.find(joint);
    if (it == chains.end()) {
        return std::string();
    }
    std::string text;
    for (size_t i = 0; i < it->second.size(); ++i) {
        text += (i ? ", " : "") + it->second[i].GetString();
    }
    return text;
}

/// How many steps write \p joint.
size_t
ChainSize(const std::map<SdfPath, std::vector<SdfPath>> &chains,
          const SdfPath &joint)
{
    const auto it = chains.find(joint);
    return it == chains.end() ? 0 : it->second.size();
}

/// True when \p first appears before \p second in \p text, and both appear.
bool
InOrder(const std::string &text, const std::string &first,
        const std::string &second)
{
    const size_t a = text.find(first);
    if (a == std::string::npos) {
        return false;
    }
    return text.find(second, a + first.size()) != std::string::npos;
}

bool
Mentions(const std::vector<std::string> &lines, const char *needle)
{
    for (const std::string &line : lines) {
        if (line.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// The probe mesh's first moved point, which is the knee matrix the mover
/// read applied to the mesh's origin.
bool
ProbePoint(const RigExecRigPose &pose, GfVec3f *out)
{
    const auto it = pose.movedProperties.find(kProbePoints);
    if (it == pose.movedProperties.end() ||
        !it->second.IsHolding<VtVec3fArray>()) {
        return false;
    }
    const VtVec3fArray &points = it->second.UncheckedGet<VtVec3fArray>();
    if (points.empty()) {
        return false;
    }
    *out = points[0];
    return true;
}

// Two fixtures written as their own text, because what they need is writer
// SUBSETS that differ per joint and an ordering that runs through a
// CONSTRAINT -- neither of which the RigSpec above spells.

/// A control chain <name>Hip/Knee/Ankle curling by \p degrees at frame 5.
std::string
FkControls(const char *name, int degrees)
{
    return std::string("            def RigExecControl \"") + name +
           "Hip\"\n"
           "            {\n"
           "                double avars:rz.timeSamples = { 1: 0, 5: " +
           std::to_string(degrees) + " }\n"
           "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
           "\n"
           "                def RigExecControl \"Knee\"\n"
           "                {\n"
           "                    matrix4d rest:space = " + Rest(4, 0, 0) +
           "\n"
           "\n"
           "                    def RigExecControl \"Ankle\"\n"
           "                    {\n"
           "                        matrix4d rest:space = " + Rest(4, 0, 0) +
           "\n"
           "                    }\n"
           "                }\n"
           "            }\n";
}

/// Stage head: the rig root, \p controls, and the Hip/Knee/Ankle chain.
std::string
Head(const std::string &controls, const std::string &extraJoints = "")
{
    return
        "#usda 1.0\n"
        "(\n"
        "    defaultPrim = \"Asset\"\n"
        "    startTimeCode = 1\n"
        "    endTimeCode = 5\n"
        "    metersPerUnit = 0.01\n"
        "    timeCodesPerSecond = 24\n"
        "    upAxis = \"Y\"\n"
        ")\n"
        "\n"
        "def Xform \"Asset\"\n"
        "{\n"
        "    def RigExecRoot \"Rig\"\n"
        "    {\n"
        "        uniform token rigExec:partition = \"Asset\"\n"
        "\n"
        "        def Scope \"Controls\"\n"
        "        {\n" + controls +
        "        }\n"
        "\n"
        "        def Scope \"Joints\"\n"
        "        {\n"
        "            def RigExecJoint \"Hip\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
        "\n"
        "                def RigExecJoint \"Knee\"\n"
        "                {\n"
        "                    matrix4d rest:space = " + Rest(4, 0, 0) + "\n"
        "\n"
        "                    def RigExecJoint \"Ankle\"\n"
        "                    {\n"
        "                        matrix4d rest:space = " + Rest(4, 0, 0) +
        "\n"
        "                    }\n"
        "                }\n"
        "            }\n" + extraJoints +
        "        }\n";
}

std::string
Chain(const char *name, const std::string &controls,
      const std::string &joints)
{
    return std::string("            def RigExecFkChain \"") + name +
           "\"\n"
           "            {\n"
           "                rel rigExec:controls = [ " + controls + " ]\n"
           "                rel rigExec:joints = [ " + joints + " ]\n"
           "            }\n";
}

/// Three chains whose writer SETS differ, plus one real data-flow edge.
///
/// Definition order FkA, FkC, FkB, so the stack ordinals are FkB=0, FkC=1,
/// FkA=2. FkB reads the ankle, which only FkA writes, so data flow puts FkA
/// before FkB. The hip is written by all three; the knee by FkC and FkB only.
///
/// Settled per joint, the knee never sees FkA, so FkB looks free there and
/// takes the lowest ordinal -- knee = [FkB, FkC] while the hip = [FkC, FkA,
/// FkB]. The adjacent-pair edges then say both "FkC after FkB" and "FkB after
/// FkA after FkC", which is a cycle, and the author is handed a generic pose
/// loop for a rig that is entirely legal. Settled ONCE over every writer, the
/// two lists are restrictions of one order and there is nothing to disagree
/// about.
std::string
SubsetOrderText()
{
    std::string text =
        Head(FkControls("FkA", 10) + FkControls("FkB", 25) +
             FkControls("FkC", 40)) +
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n";
    text += Chain("FkA",
                  "</Asset/Rig/Controls/FkAHip>, "
                  "</Asset/Rig/Controls/FkAHip/Knee>, "
                  "</Asset/Rig/Controls/FkAHip/Knee/Ankle>",
                  "</Asset/Rig/Joints/Hip>, "
                  "</Asset/Rig/Joints/Hip/Knee/Ankle>");
    text += Chain("FkC",
                  "</Asset/Rig/Controls/FkCHip>, "
                  "</Asset/Rig/Controls/FkCHip/Knee>, "
                  "</Asset/Rig/Controls/FkCHip/Knee/Ankle>",
                  "</Asset/Rig/Joints/Hip>, "
                  "</Asset/Rig/Joints/Hip/Knee>");
    text += Chain("FkB",
                  "</Asset/Rig/Joints/Hip/Knee/Ankle>, "
                  "</Asset/Rig/Controls/FkBHip/Knee>, "
                  "</Asset/Rig/Controls/FkBHip/Knee/Ankle>",
                  "</Asset/Rig/Joints/Hip>, "
                  "</Asset/Rig/Joints/Hip/Knee>");
    text +=
        "        }\n"
        "    }\n"
        "}\n";
    return text;
}

/// Two writers of the hip whose only ordering runs THROUGH a constraint.
///
/// FkB writes the hip and the ankle. TipAim aims /Joints/Tip at that ankle,
/// so it waits on FkB. FkA reads Tip as a control, so it waits on the aim --
/// and FkA also writes the hip. Definition order FkB, FkA makes FkA the lower
/// ordinal, so the namespace alone would run FkA first and the stack edge
/// would say "FkB after FkA", closing FkA -> TipAim -> FkB -> FkA.
///
/// Nothing in the SOLVER graph orders the pair: the ordering exists only in
/// the pose graph, and only once the constraint pass and the frame-inheritance
/// walk have run. Without the aim the same rig is namespace-ordered and runs
/// FkA first, which is what makes the flip below a fact about data flow
/// rather than about the text.
std::string
ConstraintOrderText(bool withAim)
{
    // A second root joint for the aim to move, outside the stacked chain.
    std::string text =
        Head(FkControls("FkA", 10) + FkControls("FkB", 40),
             "            def RigExecJoint \"Tip\"\n"
             "            {\n"
             "                matrix4d rest:space = " + Rest(0, 20, 0) +
             "\n"
             "            }\n");
    text +=
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n";
    text += Chain("FkB",
                  "</Asset/Rig/Controls/FkBHip>, "
                  "</Asset/Rig/Controls/FkBHip/Knee>, "
                  "</Asset/Rig/Controls/FkBHip/Knee/Ankle>",
                  "</Asset/Rig/Joints/Hip>, "
                  "</Asset/Rig/Joints/Hip/Knee>, "
                  "</Asset/Rig/Joints/Hip/Knee/Ankle>");
    text += Chain("FkA", "</Asset/Rig/Joints/Tip>",
                  "</Asset/Rig/Joints/Hip>");
    text += "        }\n";
    if (withAim) {
        text +=
            "\n"
            "        def Scope \"Movers\"\n"
            "        {\n"
            "            def RigExecAimConstraint \"TipAim\" (\n"
            "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
            "            )\n"
            "            {\n"
            "                float inputs:defaultWeight = 1\n"
            "                uniform token rigExec:aimAxis = \"x\"\n"
            "                rel rigExec:aimTarget = "
            "</Asset/Rig/Joints/Hip/Knee/Ankle>\n"
            "                rel rigExec:moves = "
            "</Asset/Rig/Joints/Tip>\n"
            "                uniform token[] rigExec:preserve = "
            "[\"origin\", \"scale\"]\n"
            "            }\n"
            "        }\n";
    }
    text +=
        "    }\n"
        "}\n";
    return text;
}

UsdStageRefPtr
OpenText(const std::string &text)
{
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    if (!layer || !layer->ImportFromString(text)) {
        std::printf("FAIL: a hand-written stacking fixture does not parse\n");
        ++failures;
        return UsdStageRefPtr();
    }
    return UsdStage::Open(layer);
}

// The baked half: the parity harness testRigExecSolverBake uses, over the
// stacking fixtures.

void
CheckParity(const char *what, const RigSpec &spec,
            const std::vector<double> &frames, bool expectBaked)
{
    const UsdStageRefPtr referenceStage = MakeStage(spec);
    const UsdStageRefPtr bakedStage = MakeStage(spec);
    CHECK(referenceStage && bakedStage);
    if (!referenceStage || !bakedStage) return;

    RigExecRigEvaluator reference(referenceStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!reference.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);

    std::vector<std::string> reasons;
    const bool bakeable = baked.IsBakeable(&reasons);
    if (bakeable != expectBaked) {
        ++failures;
        std::printf("FAIL %s: the rig is %sbakeable\n", what,
                    bakeable ? "" : "not ");
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
    }

    // Each frame twice, then the list backwards: a repeated frame is what
    // makes a run skip steps, and a backwards sweep is what makes it skip
    // different ones. A commit that published last frame's answer -- which is
    // exactly what a mis-versioned second writer on one slot would do --
    // shows up here and nowhere else.
    std::vector<double> sweep = frames;
    sweep.insert(sweep.end(), frames.rbegin(), frames.rend());
    sweep.insert(sweep.end(), frames.begin(), frames.end());
    size_t generations = 0;
    for (const double frame : sweep) {
        const std::string where =
            std::string(what) + " frame " + std::to_string(frame);
        const RigExecRigPose a = reference.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose b = baked.Evaluate(UsdTimeCode(frame));
        CHECK(a.valid && b.valid);
        if (b.bakedParityMismatches) {
            ++failures;
            std::printf("FAIL %s: %zu baked parity mismatch(es)\n",
                        where.c_str(), b.bakedParityMismatches);
            for (const std::string &line : b.diagnostics) {
                std::printf("    %s\n", line.c_str());
            }
        }
        ++generations;
        rigExecTest::ComparePose(&failures, where, a, b);
    }
    if (expectBaked && baked.GetBakedGenerationCount() != generations) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                    "program\n", what, baked.GetBakedGenerationCount(),
                    generations);
    }
}

// The cases.

/// Compile succeeds SILENTLY, and the chains carry the stack order.
///
/// Stacking is ordinary authoring: it is not reported, so a stacking rig
/// compiles with an EMPTY message vector, and the order is read from the
/// compiled per-joint chains.
void
TestCompilesAndChains()
{
    const UsdStageRefPtr stage = MakeStage(Stacked());
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRigPath);
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    if (!errors.empty()) {
        ++failures;
        std::printf("FAIL: a stacking rig is not silent; compile said:\n");
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
    }

    const std::map<SdfPath, std::vector<SdfPath>> chains =
        evaluator.GetFrameChains();
    for (const SdfPath &joint : {kHip, kKnee, kAnkle}) {
        CHECK(ChainSize(chains, joint) == 2);
        // Definition order is LegIK then LegFK, and the stack is its
        // REVERSE, so FK writes first and IK writes last.
        CHECK(InOrder(ChainText(chains, joint), kLegFk.GetString(),
                      kLegIk.GetString()));
    }
}

/// The stacked rig's joints carry the LAST writer's solution, and the earlier
/// writer really ran.
void
TestLastWriterWins()
{
    RigSpec ikOnly = Stacked();
    ikOnly.fk = false;
    RigSpec fkOnly = Stacked();
    fkOnly.ik = false;

    const UsdStageRefPtr stackedStage = MakeStage(Stacked());
    const UsdStageRefPtr ikStage = MakeStage(ikOnly);
    const UsdStageRefPtr fkStage = MakeStage(fkOnly);
    CHECK(stackedStage && ikStage && fkStage);
    if (!stackedStage || !ikStage || !fkStage) return;

    bool differs = false;
    bool reMeasured = false;
    for (const double time : {1.0, 3.0, 5.0}) {
        const RigExecRigPose stacked =
            Evaluate("stacked", stackedStage, time, nullptr);
        const RigExecRigPose ik = Evaluate("ik only", ikStage, time, nullptr);
        const RigExecRigPose fk = Evaluate("fk only", fkStage, time, nullptr);
        if (!stacked.valid || !ik.valid || !fk.valid) return;
        for (const SdfPath &joint : {kHip, kKnee, kAnkle}) {
            // The LAST writer supplies the frame: never the FK's answer.
            CHECK(!SameJoint(stacked, fk, joint) || SameJoint(ik, fk, joint));
            differs = differs || !SameJoint(ik, fk, joint);
            reMeasured = reMeasured || !SameJoint(stacked, ik, joint);
        }
        if (time == 1.0) {
            // At frame 1 the FK control chain is at its rest, so the frames
            // it leaves ARE the joints' authored rests and the IK's rest
            // reference is unchanged: the stack answers exactly as the IK
            // alone does. This is the parity edge of the live-rest rule.
            for (const SdfPath &joint : {kHip, kKnee, kAnkle}) {
                CHECK(SameJoint(stacked, ik, joint));
            }
        }
        // Both solvers evaluated: one of them being inert would make the
        // comparisons above pass for the wrong reason.
        CHECK(stacked.solverEvaluations == 2);
        CHECK(stacked.solverFrames.count(kLegFk) == 1);
        CHECK(stacked.solverFrames.count(kLegIk) == 1);
    }
    // ... and the two solutions genuinely differ, or neither this case nor
    // the reorder below proves anything.
    CHECK(differs);
    // The earlier writer is not merely overwritten: the IK MEASURES its
    // chain from the frames the FK left, so once the FK moves the joints the
    // stack stops agreeing with the IK alone (spec 4.2, live rests).
    CHECK(reMeasured);
}

/// `reorder nameChildren` on /Asset/Rig/Solvers flips the stack --
/// through the layer text, and again through the session layer with no
/// intervening Compile.
void
TestReorderFlipsTheStack()
{
    RigSpec reordered = Stacked();
    reordered.solverOrder = {"LegFK", "LegIK"};

    const UsdStageRefPtr reorderedStage = MakeStage(reordered);
    const UsdStageRefPtr defaultStage = MakeStage(Stacked());
    CHECK(reorderedStage && defaultStage);
    if (!reorderedStage || !defaultStage) return;

    std::vector<std::string> errors;
    std::map<SdfPath, std::vector<SdfPath>> chains, defaultChains;
    const RigExecRigPose flipped =
        Evaluate("reordered", reorderedStage, 5.0, &errors, true, &chains);
    const RigExecRigPose plain =
        Evaluate("default order", defaultStage, 5.0, nullptr, true,
                 &defaultChains);
    if (!flipped.valid || !plain.valid) return;
    CHECK(errors.empty());
    // The chain really is the other way round...
    CHECK(InOrder(ChainText(defaultChains, kHip), kLegFk.GetString(),
                  kLegIk.GetString()));
    CHECK(InOrder(ChainText(chains, kHip), kLegIk.GetString(),
                  kLegFk.GetString()));
    // ... and the frames say so too: every step of the stack measures from
    // what the steps below it left, so running them the other way round is a
    // different rig, not the same answer relabelled.
    bool moved = false;
    for (const SdfPath &joint : {kHip, kKnee, kAnkle}) {
        moved = moved || !SameJoint(flipped, plain, joint);
    }
    CHECK(moved);

    // The same edit on a LIVE evaluator, in the session layer, with no
    // Compile() call: the composed solver order is part of the binding-epoch
    // digest, so Evaluate has to notice and rebuild. A reordered stack that
    // kept a stale schedule would be the same rig quietly answering with the
    // other solver's solution.
    const UsdStageRefPtr liveStage = MakeStage(Stacked());
    CHECK(liveStage);
    if (!liveStage) return;
    RigExecRigEvaluator live(liveStage, kRigPath);
    live.cpuParityMode = true;
    std::vector<std::string> liveErrors;
    CHECK(live.Compile(&liveErrors));
    const RigExecRigPose before = live.Evaluate(UsdTimeCode(5.0));
    CHECK(before.valid);
    const size_t beforeDigest = live.GetBindingEpochDigest();

    liveStage->SetEditTarget(liveStage->GetSessionLayer());
    const UsdPrim solvers =
        liveStage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers"));
    CHECK(solvers);
    if (!solvers) return;
    solvers.SetChildrenReorder({TfToken("LegFK"), TfToken("LegIK")});

    const RigExecRigPose after = live.Evaluate(UsdTimeCode(5.0));
    CHECK(after.valid);
    CHECK(live.GetBindingEpochDigest() != beforeDigest);
    // The live edit lands on exactly the answer the reordered TEXT gives,
    // and not on the one it had a moment ago.
    for (const SdfPath &joint : {kHip, kKnee, kAnkle}) {
        CHECK(SameJoint(after, flipped, joint));
    }
    CHECK(InOrder(ChainText(live.GetFrameChains(), kHip), kLegIk.GetString(),
                  kLegFk.GetString()));
}

/// A read phase naming a solver sees the joint as THAT writer left it.
///
/// Solver checkpoints are addressable from the geometry domain only -- the
/// rigExec:transform phase of a matrix mover -- because a pose constraint's
/// frame sources carry no phase and always observe the top of the stack.
void
TestCheckpointSeesTheEarlierWriter()
{
    RigSpec atFk = Stacked();
    atFk.probePhase = kLegFk.GetString();
    RigSpec fkOnly = Stacked();
    fkOnly.ik = false;
    fkOnly.probePhase = "final";
    RigSpec ikOnly = Stacked();
    ikOnly.fk = false;
    ikOnly.probePhase = "final";

    const UsdStageRefPtr atFkStage = MakeStage(atFk);
    const UsdStageRefPtr fkStage = MakeStage(fkOnly);
    const UsdStageRefPtr ikStage = MakeStage(ikOnly);
    CHECK(atFkStage && fkStage && ikStage);
    if (!atFkStage || !fkStage || !ikStage) return;

    const RigExecRigPose checkpoint =
        Evaluate("checkpoint probe", atFkStage, 5.0, nullptr, false);
    const RigExecRigPose fk =
        Evaluate("fk probe", fkStage, 5.0, nullptr, false);
    const RigExecRigPose ik =
        Evaluate("ik probe", ikStage, 5.0, nullptr, false);
    if (!checkpoint.valid || !fk.valid || !ik.valid) return;

    GfVec3f atCheckpoint, atFkOnly, atIkOnly;
    CHECK(ProbePoint(checkpoint, &atCheckpoint));
    CHECK(ProbePoint(fk, &atFkOnly));
    CHECK(ProbePoint(ik, &atIkOnly));
    // The earlier writer's knee, not the top of the stack.
    CHECK(Near(GfVec3d(atCheckpoint), GfVec3d(atFkOnly), 1e-4));
    CHECK(!Near(GfVec3d(atFkOnly), GfVec3d(atIkOnly), 1e-4));

    // ... and naming a solver is refused by the bake, because the baked
    // phased-read store is written per CONSTRAINT and has no solver
    // equivalent. A silent fall back to a different value is the one thing
    // this program refuses.
    RigExecRigEvaluator evaluator(atFkStage, kRigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    std::vector<std::string> reasons;
    CHECK(!RigExecBakedProgram::IsBakeable(evaluator, &reasons));
    CHECK(Mentions(reasons, "read phase names a solver checkpoint"));
}

/// The tear: a later writer replaces only the joints it NAMES.
///
/// A joint an earlier writer wrote and this one does not name keeps its
/// absolute out-space frame and does not follow its moved ancestor, because
/// namespace propagation stops at any path a solver writes. That is the
/// documented rule, and the note says so -- this case exists so that nobody
/// later "fixes" it into a silent composition.
void
TestPartialSubsetTearIsPinned()
{
    RigSpec tear = Stacked();
    // IK first, FK last, and the FK names only the root joint.
    tear.solverOrder = {"LegFK", "LegIK"};
    tear.fkControls = "</Asset/Rig/Controls/FkHip>";
    tear.fkJoints = "</Asset/Rig/Joints/Hip>";
    RigSpec ikOnly = Stacked();
    ikOnly.fk = false;

    const UsdStageRefPtr tearStage = MakeStage(tear);
    const UsdStageRefPtr ikStage = MakeStage(ikOnly);
    CHECK(tearStage && ikStage);
    if (!tearStage || !ikStage) return;

    std::vector<std::string> errors;
    const RigExecRigPose torn =
        Evaluate("partial subset", tearStage, 5.0, &errors);
    const RigExecRigPose ik = Evaluate("ik only", ikStage, 5.0, nullptr);
    if (!torn.valid || !ik.valid) return;

    // The hip moved to the FK's solution; the knee and ankle did NOT follow.
    CHECK(!SameJoint(torn, ik, kHip));
    CHECK(SameJoint(torn, ik, kKnee));
    CHECK(SameJoint(torn, ik, kAnkle));

    // Two writers on the hip and one on each of the others, with nothing
    // reported: the tear is defined behaviour, and the frames above are
    // what pins it.
    CHECK(errors.empty());
}

/// A FRAME read below its writer is POSITIONAL, and does not reorder anything.
///
/// The FK reads the ankle, which only the IK writes. Under the old rule that
/// read forced the FK to run second, reversing the composed namespace. Under
/// the unified pose stack the namespace IS the order and nothing bends it: the
/// FK is the bottom sibling, so it runs first and reads the ankle as it stands
/// at its own place in the stack -- the rest-propagated frame, before the IK
/// wrote it. Nothing contradicts anything and nothing is reported.
void
TestDataFlowOutranksTheNamespace()
{
    RigSpec spec = Stacked();
    spec.fkControls =
        "</Asset/Rig/Controls/FkHip>, </Asset/Rig/Joints/Hip/Knee/Ankle>";
    spec.fkJoints =
        "</Asset/Rig/Joints/Hip>, </Asset/Rig/Joints/Hip/Knee>";

    const UsdStageRefPtr stage = MakeStage(spec);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRigPath);
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const std::string hipLine =
        ChainText(evaluator.GetFrameChains(), kHip);
    CHECK(!hipLine.empty());
    // Definition order is LegIK then LegFK, so the REVERSE -- the stack --
    // is LegFK then LegIK, and the ankle read no longer flips it.
    CHECK(InOrder(hipLine, kLegFk.GetString(), kLegIk.GetString()));
    CHECK(!Mentions(errors, "dependency cycle"));
    const RigExecRigPose stacked = evaluator.Evaluate(UsdTimeCode(5.0));
    CHECK(stacked.valid);

    // ... and the FRAMES say the same thing. The note is rendered from the
    // very list the stack edges are built from, so a note-only assertion
    // cannot catch an order that is reported correctly and RUN backwards.
    // The hip is the discriminator: the FK drives it from FkHip alone, so its
    // frame is the FK's solution when the FK lands last and the IK's when it
    // does not.
    RigSpec ikOnly = spec;
    ikOnly.fk = false;
    RigSpec fkOnly = spec;
    fkOnly.ik = false;
    const UsdStageRefPtr ikStage = MakeStage(ikOnly);
    const UsdStageRefPtr fkStage = MakeStage(fkOnly);
    CHECK(ikStage && fkStage);
    if (!ikStage || !fkStage || !stacked.valid) return;
    const RigExecRigPose ik = Evaluate("ik only", ikStage, 5.0, nullptr);
    const RigExecRigPose fk = Evaluate("fk only", fkStage, 5.0, nullptr);
    if (!ik.valid || !fk.valid) return;
    // The IK lands LAST on the hip now, so the hip is not the FK's answer.
    CHECK(!SameJoint(stacked, fk, kHip));
    // The ankle is the IK's alone -- nothing stacks there -- and the IK's
    // rest for it is the authored one, because no step below the IK writes
    // the ankle.
    CHECK(SameJoint(stacked, ik, kAnkle));
}

/// The per-joint verdict when a writer publishes nothing, on BOTH paths.
///
/// An unwired TwoBoneIk publishes an empty aggregate, so every joint it names
/// "published no element". The joint is not left at rest -- the other writer
/// already wrote it -- so the message has to say which frame it kept, and the
/// dynamic and baked paths have to say it in the same words or the parity
/// comparator, which compares diagnostics verbatim, is comparing nothing.
void
TestFallbackVerdictUnderStack(bool ikLast)
{
    const char *what = ikLast ? "unwired ik last" : "unwired ik first";
    RigSpec spec = Stacked();
    spec.ikWired = false;
    if (!ikLast) {
        // Composed order FK, IK -> reversed -> the IK writes FIRST.
        spec.solverOrder = {"LegFK", "LegIK"};
    }

    const UsdStageRefPtr dynamicStage = MakeStage(spec);
    const UsdStageRefPtr bakedStage = MakeStage(spec);
    CHECK(dynamicStage && bakedStage);
    if (!dynamicStage || !bakedStage) return;

    RigExecRigEvaluator dynamicRig(dynamicStage, kRigPath);
    RigExecRigEvaluator bakedRig(bakedStage, kRigPath);
    // The verdict is judged against the exec walk itself, not against
    // whatever the default mode runs.
    dynamicRig.SetEvaluationMode(RigExecEvaluationMode::ExecReference);
    std::vector<std::string> errors;
    CHECK(dynamicRig.Compile(&errors));
    CHECK(bakedRig.Compile(&errors));
    bakedRig.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> reasons;
    if (!bakedRig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not bake\n", what);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }

    for (const double time : {1.0, 5.0}) {
        const RigExecRigPose a = dynamicRig.Evaluate(UsdTimeCode(time));
        const RigExecRigPose b = bakedRig.Evaluate(UsdTimeCode(time));
        CHECK(a.valid && b.valid);
        CHECK(b.bakedParityMismatches == 0);
        // The verdict names the writer that published nothing AND the frame
        // the joint kept -- never "fell back to its rest chain", which is
        // what it would say if the other writer had not run.
        CHECK(Mentions(a.diagnostics,
                       "solver /Asset/Rig/Solvers/LegIK published no element"));
        CHECK(Mentions(a.diagnostics,
                       "the joint keeps the frame /Asset/Rig/Solvers/LegFK "
                       "left"));
        CHECK(!Mentions(a.diagnostics, "fell back to its rest chain"));
        // Identical on both paths, line for line.
        if (a.diagnostics != b.diagnostics) {
            ++failures;
            std::printf("FAIL %s: the paths disagree about the verdict\n",
                        what);
            for (const std::string &line : a.diagnostics) {
                std::printf("    [dynamic] %s\n", line.c_str());
            }
            for (const std::string &line : b.diagnostics) {
                std::printf("    [baked]   %s\n", line.c_str());
            }
        }
        rigExecTest::ComparePose(&failures,
                                 std::string(what) + " t=" +
                                     std::to_string(time), a, b);
    }
}

/// A pose constraint on a stacked joint observes the TOP of the stack.
///
/// Constraint frame sources carry no read phase -- the solver checkpoints are
/// readable from the geometry domain only -- so an aim on a joint two solvers
/// write sits at the TOP of that joint's chain and revises what the whole
/// stack left, rather than feeding any part of it.
void
TestConstraintObservesTopOfStack()
{
    RigSpec stacked = Stacked();
    stacked.kneeAim = true;
    // Solvers at the BOTTOM of the rig, so both of them run before the aim --
    // the classic "solve, then revise" shape. Without it the aim would be the
    // bottom sibling and would FEED the solvers instead (see
    // TestConstraintBelowASolverFeedsIt).
    stacked.rigOrder = SolversLast();
    // Composed FK, IK -> reversed -> the IK writes first and the FK last.
    stacked.solverOrder = {"LegFK", "LegIK"};

    const UsdStageRefPtr stackedStage = MakeStage(stacked);
    CHECK(stackedStage);
    if (!stackedStage) return;

    RigSpec noAim = stacked;
    noAim.kneeAim = false;
    const UsdStageRefPtr noAimStage = MakeStage(noAim);
    CHECK(noAimStage);
    if (!noAimStage) return;

    std::map<SdfPath, std::vector<SdfPath>> chains;
    const RigExecRigPose a =
        Evaluate("aim over a stack", stackedStage, 5.0, nullptr, true,
                 &chains);
    const RigExecRigPose none =
        Evaluate("the same stack, no aim", noAimStage, 5.0, nullptr);
    if (!a.valid || !none.valid) return;
    // The aim is the LAST step of the knee's chain -- above both solvers --
    // so it revises what they left rather than feeding them.
    const std::vector<SdfPath> &knee = chains[kKnee];
    CHECK(knee.size() == 3);
    if (knee.size() == 3) {
        CHECK(knee[0] == kLegIk);
        CHECK(knee[1] == kLegFk);
        CHECK(knee[2] == SdfPath("/Asset/Rig/Movers/KneeAim"));
    }
    // ... and it really moved the knee, and only the knee.
    CHECK(!SameJoint(a, none, kKnee));
    CHECK(SameJoint(a, none, kHip));
    CHECK(SameJoint(a, none, kAnkle));
    // Both writers ran: an inert one would make the lines above pass for the
    // wrong reason.
    CHECK(a.solverEvaluations == 2);
}

/// Every joint's writer list is a restriction of ONE order.
///
/// Two joints with different writer sets used to settle their order
/// independently, so a writer waiting on a solver absent from one of the sets
/// looked free there and blocked in the other -- and the adjacent-pair stack
/// edges then pointed both ways and Kahn reported a pose cycle on a rig that
/// is entirely legal. Partial-subset stacking is one of the shapes this
/// feature exists for, so this compiles or the feature does not work.
void
TestPartialSubsetsAgreeOnOneOrder()
{
    const UsdStageRefPtr stage = OpenText(SubsetOrderText());
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRigPath);
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        ++failures;
        std::printf("FAIL: differing writer subsets do not compile\n");
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    CHECK(!Mentions(errors, "dependency cycle"));

    const std::map<SdfPath, std::vector<SdfPath>> chains =
        evaluator.GetFrameChains();
    const std::string hipLine = ChainText(chains, kHip);
    const std::string kneeLine = ChainText(chains, kKnee);
    CHECK(!hipLine.empty());
    CHECK(!kneeLine.empty());
    CHECK(ChainSize(chains, kHip) == 3);
    CHECK(ChainSize(chains, kKnee) == 2);

    const std::string fkA("/Asset/Rig/Solvers/FkA");
    const std::string fkB("/Asset/Rig/Solvers/FkB");
    const std::string fkC("/Asset/Rig/Solvers/FkC");
    // Definition order FkA, FkC, FkB; the stack is its REVERSE, and FkB's
    // read of the ankle is positional and does not move it.
    CHECK(InOrder(hipLine, fkB, fkC));
    CHECK(InOrder(hipLine, fkC, fkA));
    // ... and both joints agree, because every list is a restriction of ONE
    // order rather than a per-joint settlement.
    CHECK(InOrder(hipLine, fkB, fkC) == InOrder(kneeLine, fkB, fkC));
    CHECK(InOrder(kneeLine, fkB, fkC));
    CHECK(evaluator.Evaluate(UsdTimeCode(5.0)).valid);
}

/// An ordering that runs THROUGH a constraint orders the stack too.
///
/// The stack order is settled against the finished pose graph, so a pair only
/// a solver -> constraint -> solver path reaches is ordered by that path and
/// no stack edge is inserted against it. Adding a constraint that names
/// neither solver's rigExec:joints used to turn this compiling rig into a
/// pose cycle.
void
TestConstraintMediatedOrderIsRespected()
{
    const std::string fkA("/Asset/Rig/Solvers/FkA");
    const std::string fkB("/Asset/Rig/Solvers/FkB");
    for (const bool aim : {false, true}) {
        const char *what = aim ? "with the aim" : "without the aim";
        const UsdStageRefPtr stage = OpenText(ConstraintOrderText(aim));
        CHECK(stage);
        if (!stage) continue;
        RigExecRigEvaluator evaluator(stage, kRigPath);
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            ++failures;
            std::printf("FAIL: the constraint-ordered stack (%s) does not "
                        "compile\n", what);
            for (const std::string &error : errors) {
                std::printf("    %s\n", error.c_str());
            }
            continue;
        }
        CHECK(!Mentions(errors, "dependency cycle"));
        const std::string hipLine =
            ChainText(evaluator.GetFrameChains(), kHip);
        CHECK(!hipLine.empty());
        // The NAMESPACE decides, with or without the aim: definition order
        // FkB, FkA reversed makes FkA the first writer, and a constraint in
        // the path no longer bends that. (void)aim is deliberate -- the point
        // of running both is that the answer is the SAME.
        CHECK(InOrder(hipLine, fkA, fkB));
        CHECK(evaluator.Evaluate(UsdTimeCode(5.0)).valid);
    }
}

/// One solver naming one joint twice is an error -- and USD will not let a
/// rigger author it.
///
/// Two SOLVERS on one joint stack, in an order something decides. Two entries
/// from ONE solver have no order at all: they would collapse at runtime (the
/// candidate table is keyed by joint, and a baked commit's slots are uniqued)
/// and a stack edge between them would be a self-edge, which Kahn can only
/// report as an unexplained pose cycle. So the compiler rejects the shape by
/// name -- but Sdf dedupes a repeated relationship target before the compiler
/// ever sees it: the text parser refuses the layer outright ("Duplicate items
/// exist for field 'targetPaths'") and SetTargets silently drops the repeat.
///
/// What this case pins is therefore the DEDUPE, so that if Sdf ever stops
/// doing it the compiler's guard gets a real fixture instead of quietly
/// becoming the thing that fires.
void
TestDuplicateJointOnOneSolverIsUnauthorable()
{
    RigSpec fkOnly = Stacked();
    fkOnly.ik = false;
    const UsdStageRefPtr stage = MakeStage(fkOnly);
    CHECK(stage);
    if (!stage) return;
    const UsdPrim fk = stage->GetPrimAtPath(kLegFk);
    CHECK(fk);
    if (!fk) return;
    UsdRelationship joints = fk.GetRelationship(TfToken("rigExec:joints"));
    CHECK(joints);
    if (!joints) return;
    CHECK(joints.SetTargets({kHip, kKnee, kHip}));
    SdfPathVector targets;
    joints.GetTargets(&targets);
    if (targets.size() != 2) {
        ++failures;
        std::printf("FAIL: Sdf kept a repeated rigExec:joints target "
                    "(%zu targets); the compiler's duplicate guard is now "
                    "reachable and wants its own fixture\n",
                    targets.size());
        return;
    }
    // Deduped to two distinct joints, so the FK is a single writer of each
    // and the rig compiles silently.
    RigExecRigEvaluator evaluator(stage, kRigPath);
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(errors.empty());
    CHECK(ChainSize(evaluator.GetFrameChains(), kHip) == 1);
    CHECK(evaluator.Evaluate(UsdTimeCode(5.0)).valid);
}

/// A constraint BELOW a solver FEEDS it: the incoming frame is the rest.
///
/// The fixture authors no rig-root reorder, so Movers is the bottom sibling
/// and KneeMove runs FIRST. The two-bone IK then measures its bones from
/// Hip/Knee/Ankle as KneeMove left them -- a displaced knee RE-PROPORTIONS the
/// limb -- and replaces all three outright, so the constraint's displacement
/// is not visible in the answer while its EFFECT is. Both halves are asserted,
/// because either one alone passes for the wrong reason.
void
TestConstraintBelowASolverFeedsIt()
{
    RigSpec fed = Stacked();
    fed.fk = false;      // the IK alone: FkChain takes no joint rest
    fed.kneeMove = true;
    RigSpec plain = Stacked();
    plain.fk = false;

    const UsdStageRefPtr fedStage = MakeStage(fed);
    const UsdStageRefPtr plainStage = MakeStage(plain);
    CHECK(fedStage && plainStage);
    if (!fedStage || !plainStage) return;

    std::vector<std::string> errors;
    const RigExecRigPose a = Evaluate("constraint below", fedStage, 5.0,
                                      &errors);
    const RigExecRigPose b = Evaluate("no constraint", plainStage, 5.0,
                                      nullptr);
    if (!a.valid || !b.valid) return;

    // The knee is the IK's answer, not the constraint's target (0, 10, 2)...
    const auto knee = a.jointFramesFinal.find(kKnee);
    CHECK(knee != a.jointFramesFinal.end());
    if (knee != a.jointFramesFinal.end()) {
        CHECK(!Near(knee->second.Origin(), GfVec3d(4, 10, 2), 1e-4));
    }
    // ... and it is NOT the answer the same IK gives with no constraint
    // below it, because the bones were measured from the displaced knee.
    CHECK(!SameJoint(a, b, kKnee));
    // (The ankle is not asserted: it rides the IK's effector control, which
    // the re-measured bones still reach, so it lands in the same place.)

    // ... and the compile is SILENT about it: a constraint below a solver
    // is ordinary authoring, and the chain is where the order is readable.
    CHECK(errors.empty());
}

/// The same fixture with Solvers at the BOTTOM: the constraint revises.
///
/// One `reorder nameChildren` is the whole difference, and it flips the
/// constraint from an input of the solver to a revision of its output -- so
/// the knee lands exactly on the constraint's target, and the compile has
/// nothing to note.
void
TestConstraintAboveASolverRevisesIt()
{
    RigSpec revised = Stacked();
    revised.fk = false;
    revised.kneeMove = true;
    revised.rigOrder = SolversLast();

    const UsdStageRefPtr stage = MakeStage(revised);
    CHECK(stage);
    if (!stage) return;
    std::vector<std::string> errors;
    const RigExecRigPose pose = Evaluate("constraint above", stage, 5.0,
                                         &errors);
    if (!pose.valid) return;
    const auto knee = pose.jointFramesFinal.find(kKnee);
    CHECK(knee != pose.jointFramesFinal.end());
    if (knee != pose.jointFramesFinal.end()) {
        CHECK(Near(knee->second.Origin(), GfVec3d(4, 10, 2), 1e-4));
    }
    // One writer, one constraint above it: nothing is reported.
    CHECK(errors.empty());

    // ... and the reorder is the ONLY difference: without it the same rig
    // answers differently, which is what makes the order authorable.
    RigSpec fed = revised;
    fed.rigOrder.clear();
    const UsdStageRefPtr fedStage = MakeStage(fed);
    CHECK(fedStage);
    if (!fedStage) return;
    const RigExecRigPose flipped =
        Evaluate("reorder flips it", fedStage, 5.0, nullptr);
    if (!flipped.valid) return;
    CHECK(!SameJoint(pose, flipped, kKnee));
}

/// An FK CHAIN composes over the incoming frame too: nothing is discarded.
///
/// RigExecFkChain measures its deltas from its CONTROLS' rests, but the basis
/// it composes them ONTO is the joint's rest reference -- which the unified
/// pose stack replaces with the frame the steps below it left (spec 4.2). So
/// a constraint below the chain moves the joint and the chain carries that
/// displacement through its own solve instead of replacing it.
///
/// Every aggregate solver answers to this rule; the FK chain is the one that
/// used to be the exception, so it is the one pinned here.
void
TestFkChainComposesOverTheIncomingFrame()
{
    RigSpec fed = Stacked();
    fed.ik = false;
    fed.kneeMove = true;
    RigSpec plain = Stacked();
    plain.ik = false;

    const UsdStageRefPtr fedStage = MakeStage(fed);
    const UsdStageRefPtr plainStage = MakeStage(plain);
    CHECK(fedStage && plainStage);
    if (!fedStage || !plainStage) return;
    std::vector<std::string> errors;
    const RigExecRigPose a = Evaluate("fk over a feed", fedStage, 5.0,
                                      &errors);
    const RigExecRigPose b = Evaluate("fk alone", plainStage, 5.0, nullptr);
    if (!a.valid || !b.valid) return;
    // The constrained joint carries the constraint's displacement...
    CHECK(!SameJoint(a, b, kKnee));
    // ... and the joints no step below the chain wrote are untouched, which
    // is the parity edge of the rule: with no earlier writer the rest
    // reference is the authored one and the answer is the old answer.
    CHECK(SameJoint(a, b, kHip));
    CHECK(SameJoint(a, b, kAnkle));
    CHECK(errors.empty());

    // The chain reads as constraint-then-solver, which is what makes the
    // displacement an INPUT to the chain rather than a revision of it.
    std::map<SdfPath, std::vector<SdfPath>> chains;
    Evaluate("fk over a feed", fedStage, 5.0, nullptr, true, &chains);
    CHECK(InOrder(ChainText(chains, kKnee), "/Asset/Rig/Movers/KneeMove",
                  kLegFk.GetString()));
}

/// A geometry reader's phases resolve against the INTERLEAVED chain.
///
/// With Solvers at the bottom the chain on the knee is LegFK, LegIK, KneeMove,
/// so `base` -- after the last SOLVER write -- is the IK's knee, AtPrim of the
/// Solvers scope is the same frame, `final` is the constraint's, and AtPrim of
/// the Movers scope is `final` too. With Movers at the bottom the constraint
/// comes FIRST, and AtPrim(Movers) must then resolve to its PRE-solver value.
void
TestReadPhasesOverTheUnifiedStack()
{
    const auto probe = [](const char *what, const std::string &phase,
                          bool solversLast, GfVec3f *out) {
        RigSpec spec = Stacked();
        spec.fk = false;
        spec.kneeMove = true;
        spec.probePhase = phase;
        if (solversLast) spec.rigOrder = SolversLast();
        const UsdStageRefPtr stage = MakeStage(spec);
        CHECK(stage);
        if (!stage) return false;
        const RigExecRigPose pose = Evaluate(what, stage, 5.0, nullptr, false);
        if (!pose.valid) return false;
        return ProbePoint(pose, out);
    };
    GfVec3f base, atSolvers, atMovers, atFinal;
    CHECK(probe("base, solvers last", "base", true, &base));
    CHECK(probe("AtPrim(Solvers), solvers last", "/Asset/Rig/Solvers", true,
                &atSolvers));
    CHECK(probe("AtPrim(Movers), solvers last", "/Asset/Rig/Movers", true,
                &atMovers));
    CHECK(probe("final, solvers last", "final", true, &atFinal));
    // base == after the last SOLVER == AtPrim(Solvers).
    CHECK(Near(GfVec3d(base), GfVec3d(atSolvers), 1e-4));
    // final == the top of the chain == AtPrim(Movers).
    CHECK(Near(GfVec3d(atFinal), GfVec3d(atMovers), 1e-4));
    // ... and the constraint really moved it, or the pairs above agree for
    // the wrong reason.
    CHECK(!Near(GfVec3d(base), GfVec3d(atFinal), 1e-4));

    // Movers at the bottom: the constraint runs FIRST, so AtPrim(Movers) is
    // the PRE-solver frame and `base`/`final` are both the solver's.
    GfVec3f fedBase, fedMovers, fedFinal;
    CHECK(probe("base, movers first", "base", false, &fedBase));
    CHECK(probe("AtPrim(Movers), movers first", "/Asset/Rig/Movers", false,
                &fedMovers));
    CHECK(probe("final, movers first", "final", false, &fedFinal));
    CHECK(Near(GfVec3d(fedBase), GfVec3d(fedFinal), 1e-4));
    CHECK(!Near(GfVec3d(fedMovers), GfVec3d(fedFinal), 1e-4));
}

/// The AtPrim(Movers) probe held to the run's phased-read store, baked, in
/// both rig orders. Movers holds the constraint, while the IK writing the
/// same knee sits in Solvers, so the probe's list is KneeMove's record alone
/// whether the record lands after the IK's write or before it.
void
TestAtPrimMoversRecordsMatchTheStore()
{
    for (const bool solversLast : {true, false}) {
        const std::string what = solversLast
                                     ? "AtPrim(Movers) records, solvers last"
                                     : "AtPrim(Movers) records, movers first";
        RigSpec spec = Stacked();
        spec.fk = false;
        spec.kneeMove = true;
        spec.probePhase = "/Asset/Rig/Movers";
        if (solversLast) spec.rigOrder = SolversLast();
        const UsdStageRefPtr stage = MakeStage(spec);
        CHECK(stage);
        if (!stage) return;
        RigExecRigEvaluator evaluator(stage, kRigPath);
        evaluator.SetEvaluationMode(
            RigExecEvaluationMode::BakedWithParityCheck);
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        std::vector<std::string> reasons;
        CHECK(evaluator.IsBakeable(&reasons));
        for (const std::string &reason : reasons) {
            std::printf("    %s: %s\n", what.c_str(), reason.c_str());
        }
        for (const double frame : {1.0, 5.0}) {
            const std::string where =
                what + " frame " + std::to_string(int(frame));
            const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(frame));
            CHECK(pose.valid);
            CHECK(pose.bakedParityMismatches == 0);
            CHECK(rigExecTest::CheckFrameRecords(&failures, where,
                                                 evaluator) == 1);
        }
        CHECK(evaluator.GetBakedGenerationCount() == 2);
        const RigExecBakedProgram *program = evaluator.GetBakedProgram();
        CHECK(program != nullptr);
        if (!program) continue;
        const RigExecBakedProgramImpl &B = program->GetStepGraph();
        const RigExecBakedProgramImpl::GeomRevision *probe =
            rigExecTest::FindRevision(B,
                                      SdfPath("/Asset/Rig/Movers/KneeProbe"));
        CHECK(probe != nullptr);
        if (probe) {
            CHECK(rigExecTest::RecordMovers(B, probe->transformRecords) ==
                  std::vector<SdfPath>{SdfPath("/Asset/Rig/Movers/KneeMove")});
        }
    }
}

/// A PRODUCER carries no stack position, and the classic blend still works.
///
/// FK and IK are defined ABOVE the blend that reads their aggregates, so the
/// reversed sibling order puts the BLEND FIRST. That only compiles because a
/// solver whose aggregate another solver reads writes no joint and is
/// therefore not a stack step at all -- it is scheduled by data flow. Treat
/// every aggregate solver as a stack step and every IK/FK blend in the repo
/// becomes an unschedulable cycle, so this is the tripwire for that mistake.
void
TestProducersCarryNoStackPosition()
{
    std::string text =
        Head(FkControls("Fk", 30)) +
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n";
    text += Chain("FK",
                  "</Asset/Rig/Controls/FkHip>, "
                  "</Asset/Rig/Controls/FkHip/Knee>, "
                  "</Asset/Rig/Controls/FkHip/Knee/Ankle>",
                  "</Asset/Rig/Joints/Hip>, "
                  "</Asset/Rig/Joints/Hip/Knee>, "
                  "</Asset/Rig/Joints/Hip/Knee/Ankle>");
    text += Chain("FK2",
                  "</Asset/Rig/Controls/FkHip>, "
                  "</Asset/Rig/Controls/FkHip/Knee>, "
                  "</Asset/Rig/Controls/FkHip/Knee/Ankle>",
                  "</Asset/Rig/Joints/Hip>, "
                  "</Asset/Rig/Joints/Hip/Knee>, "
                  "</Asset/Rig/Joints/Hip/Knee/Ankle>");
    // The blend claims the WHOLE chain both inputs name, which is what makes
    // them producers: the relaxation is PER JOINT, so an input that named a
    // joint the blend does not would still write it and would still be a
    // stack step.
    text +=
        "            def RigExecBlendPointFrames \"Blend\"\n"
        "            {\n"
        "                rel rigExec:inputA = </Asset/Rig/Solvers/FK>\n"
        "                rel rigExec:inputB = </Asset/Rig/Solvers/FK2>\n"
        "                float inputs:weight = 0.5\n"
        "                rel rigExec:joints = [\n"
        "                    </Asset/Rig/Joints/Hip>,\n"
        "                    </Asset/Rig/Joints/Hip/Knee>,\n"
        "                    </Asset/Rig/Joints/Hip/Knee/Ankle>,\n"
        "                ]\n"
        "            }\n"
        "        }\n"
        "    }\n"
        "}\n";
    const UsdStageRefPtr stage = OpenText(text);
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRigPath);
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        ++failures;
        std::printf("FAIL: the producer fixture does not compile\n");
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    CHECK(!Mentions(errors, "dependency cycle"));
    CHECK(!Mentions(errors, "executes before it in the composed hierarchy"));
    CHECK(evaluator.Evaluate(UsdTimeCode(5.0)).valid);
}

/// An AGGREGATE read takes the pair OUT of the stack, either way round.
///
/// A frame read below its writer is positional and legal; an aggregate is a
/// dataflow value with no earlier version, so the consumer MUST run after the
/// producer whatever the hierarchy says. The rule that resolves this is the
/// PRODUCER split: a solver whose aggregate another solver reads carries no
/// stack position at all, so there is nothing for the hierarchy to contradict
/// -- which is why a blend authored BELOW its inputs (every blend in the repo,
/// because a blend is written last) schedules.
///
/// Note the producer here also writes a joint of its own that the blend does
/// not name. The consumed-solver relaxation is per JOINT, so it still writes;
/// what takes it out of the stack is being READ, and this is the case that
/// pins that distinction. `docs/examples/blend_point_frames.usda` is the same
/// shape.
///
/// The compile-time contradiction check (spec 4.2) survives as a forward
/// guard for the day a consumed solver is allowed to hold a stack position:
/// it compares two STACK STEPS, and a producer is never one.
std::string
AggregateProducerText()
{
    std::string text =
        Head(FkControls("Fk", 30),
             "            def RigExecJoint \"Tip\"\n"
             "            {\n"
             "                matrix4d rest:space = " + Rest(0, 20, 0) +
             "\n"
             "            }\n") +
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n"
        "            def RigExecBlendPointFrames \"Blend\"\n"
        "            {\n"
        "                rel rigExec:inputA = </Asset/Rig/Solvers/Producer>\n"
        "                rel rigExec:inputB = </Asset/Rig/Solvers/Producer>\n"
        "                float inputs:weight = 0.5\n"
        "                rel rigExec:joints = </Asset/Rig/Joints/Hip>\n"
        "            }\n";
    text += Chain("Producer", "</Asset/Rig/Controls/FkHip>",
                  "</Asset/Rig/Joints/Tip>");
    text +=
        "        }\n"
        "    }\n"
        "}\n";
    return text;
}

void
TestAggregateContradictionIsRejected()
{
    const UsdStageRefPtr stage = OpenText(AggregateProducerText());
    CHECK(stage);
    if (!stage) return;
    RigExecRigEvaluator evaluator(stage, kRigPath);
    evaluator.cpuParityMode = true;
    std::vector<std::string> errors;
    // Definition order Blend, Producer -> reversed -> Producer runs FIRST,
    // which agrees with the aggregate edge.
    CHECK(evaluator.Compile(&errors));
    CHECK(!Mentions(errors, "executes before it in the composed hierarchy"));
    const RigExecRigPose agreeing = evaluator.Evaluate(UsdTimeCode(5.0));
    CHECK(agreeing.valid);

    // Flip the scope so the hierarchy demands the opposite. It still
    // compiles, and to the SAME answer, because the producer has no stack
    // position for the flip to move.
    stage->SetEditTarget(stage->GetSessionLayer());
    const UsdPrim solvers = stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers"));
    CHECK(solvers);
    if (!solvers) return;
    solvers.SetChildrenReorder({TfToken("Producer"), TfToken("Blend")});
    RigExecRigEvaluator flipped(stage, kRigPath);
    flipped.cpuParityMode = true;
    std::vector<std::string> flippedErrors;
    if (!flipped.Compile(&flippedErrors)) {
        ++failures;
        std::printf("FAIL: the flipped aggregate fixture does not compile\n");
        for (const std::string &error : flippedErrors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    CHECK(!Mentions(flippedErrors,
                    "executes before it in the composed hierarchy"));
    const RigExecRigPose after = flipped.Evaluate(UsdTimeCode(5.0));
    CHECK(after.valid);
    CHECK(SameJoint(agreeing, after, kHip));
    CHECK(SameJoint(agreeing, after, SdfPath("/Asset/Rig/Joints/Tip")));
}

/// The blend above reads its producer's aggregate from the generation it
/// answers, in either hierarchy order: frame 5 then frame 2 publishes what
/// frame 2 alone does, on the walk and on the program.
void
TestAggregateReadIsHistoryIndependent()
{
    const auto make = [](bool flipped) {
        return [flipped]() {
            const UsdStageRefPtr stage = OpenText(AggregateProducerText());
            if (stage && flipped) {
                stage->GetPrimAtPath(SdfPath("/Asset/Rig/Solvers"))
                    .SetChildrenReorder(
                        {TfToken("Producer"), TfToken("Blend")});
            }
            return stage;
        };
    };
    const rigExecTest::EvaluationState frame5{UsdTimeCode(5.0), {}};
    const rigExecTest::EvaluationState frame2{UsdTimeCode(2.0), {}};
    for (const RigExecEvaluationMode mode :
             {RigExecEvaluationMode::ExecReference,
              RigExecEvaluationMode::Baked}) {
        rigExecTest::CheckHistoryIndependent(
            &failures, "blend of a producer", make(false), kRigPath, mode,
            frame5, frame2);
        rigExecTest::CheckHistoryIndependent(
            &failures, "blend of a producer, flipped", make(true), kRigPath,
            mode, frame5, frame2);
    }
}

/// Two unrelated stacked limbs stay in the SAME Kahn level.
///
/// The hierarchical order decides the RELATIVE order of two steps that write
/// or read the same joint. It must never put an edge between steps that do
/// not interact, or the stack would become one global serial chain and the
/// whole pose phase would lose its width -- on the biped, and in the baked
/// cone schedule that is built from the same graph.
///
/// The fixture is two independent limbs, each a two-solver stack over its own
/// three joints with its own controls and its own constraint. Nothing is
/// shared. Every step of the left limb must therefore sit in the same
/// dependency level as its opposite number on the right, and the whole rig
/// must use no more levels than one limb alone does -- with the constraints
/// above the solvers and with them below, feeding the solvers.
void
TestUnrelatedLimbsShareALevel()
{
    const auto limb = [](const char *side) {
        const std::string s(side);
        std::string text =
            "            def RigExecControl \"" + s + "Root\"\n"
            "            {\n"
            "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
            "            }\n"
            "            def RigExecControl \"" + s + "Foot\"\n"
            "            {\n"
            "                double avars:ty.timeSamples = { 1: 0, 5: 1.5 }\n"
            "                matrix4d rest:space = " + Rest(0, 0, 0) + "\n"
            "            }\n"
            "            def RigExecControl \"" + s + "Pole\"\n"
            "            {\n"
            "                matrix4d rest:space = " + Rest(0, 4, 3) + "\n"
            "            }\n" +
            FkControls((s + "Fk").c_str(), 30);
        return text;
    };
    const auto joints = [](const char *side) {
        const std::string s(side);
        return
            "            def RigExecJoint \"" + s + "Hip\"\n"
            "            {\n"
            "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
            "\n"
            "                def RigExecJoint \"Knee\"\n"
            "                {\n"
            "                    matrix4d rest:space = " + Rest(4, 0, 0) +
            "\n"
            "\n"
            "                    def RigExecJoint \"Ankle\"\n"
            "                    {\n"
            "                        matrix4d rest:space = " + Rest(4, 0, 0) +
            "\n"
            "                    }\n"
            "                }\n"
            "            }\n";
    };
    const auto solvers = [](const char *side) {
        const std::string s(side);
        const std::string chain =
            "</Asset/Rig/Joints/" + s + "Hip>, "
            "</Asset/Rig/Joints/" + s + "Hip/Knee>, "
            "</Asset/Rig/Joints/" + s + "Hip/Knee/Ankle>";
        std::string text =
            "            def RigExecTwoBoneIk \"" + s + "IK\"\n"
            "            {\n"
            "                rel rigExec:rootControl = "
            "</Asset/Rig/Controls/" + s + "Root>\n"
            "                rel rigExec:effectorControl = "
            "</Asset/Rig/Controls/" + s + "Foot>\n"
            "                rel rigExec:poleControl = "
            "</Asset/Rig/Controls/" + s + "Pole>\n"
            "                rel rigExec:joints = [ " + chain + " ]\n"
            "                double rigExec:preferredBendRadians = 0.3\n"
            "            }\n";
        text += Chain((s + "FK").c_str(),
                      "</Asset/Rig/Controls/" + s + "FkHip>, "
                      "</Asset/Rig/Controls/" + s + "FkHip/Knee>, "
                      "</Asset/Rig/Controls/" + s + "FkHip/Knee/Ankle>",
                      chain);
        return text;
    };
    const auto aim = [](const char *side) {
        const std::string s(side);
        return
            "            def RigExecAimConstraint \"" + s + "Aim\" (\n"
            "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
            "            )\n"
            "            {\n"
            "                float inputs:defaultWeight = 1\n"
            "                uniform token rigExec:aimAxis = \"x\"\n"
            "                rel rigExec:aimTarget = "
            "</Asset/Rig/Controls/" + s + "Pole>\n"
            "                rel rigExec:moves = "
            "</Asset/Rig/Joints/" + s + "Hip/Knee>\n"
            "                uniform token[] rigExec:preserve = "
            "[\"origin\", \"scale\"]\n"
            "            }\n";
    };
    const auto build = [&](bool bothLimbs, bool solversLast) {
        std::string controls = limb("L");
        std::string jointText = joints("L");
        std::string solverText = solvers("L");
        std::string moverText = aim("L");
        if (bothLimbs) {
            controls += limb("R");
            jointText += joints("R");
            solverText += solvers("R");
            moverText += aim("R");
        }
        // solversLast: Solvers at the BOTTOM of the rig, the classic
        // "solve, then revise" shape, where every solver runs before every
        // constraint. Otherwise Movers sit at the bottom and each aim FEEDS
        // its own limb's solvers, so the solver levels also show that the
        // two aims are not ordered against each other.
        std::string text = Head(controls, jointText);
        const std::string partition =
            "        uniform token rigExec:partition = \"Asset\"\n";
        text.replace(text.find(partition), partition.size(),
                     partition +
                     (solversLast
                          ? "        reorder nameChildren = [\"Controls\", "
                            "\"Joints\", \"Movers\", \"Solvers\"]\n"
                          : "        reorder nameChildren = [\"Controls\", "
                            "\"Joints\", \"Solvers\", \"Movers\"]\n"));
        return
            text +
            "\n"
            "        def Scope \"Solvers\"\n"
            "        {\n" + solverText +
            "        }\n"
            "\n"
            "        def Scope \"Movers\"\n"
            "        {\n" + moverText +
            "        }\n"
            "    }\n"
            "}\n";
    };
    // Pose step levels (solvers and constraints) and solver levels.
    struct Levels {
        std::map<SdfPath, size_t> steps, solvers;
    };
    const auto levelsOf = [](const UsdStageRefPtr &stage,
                             const std::string &what) {
        Levels levels;
        if (!stage) {
            ++failures;
            return levels;
        }
        RigExecRigEvaluator evaluator(stage, kRigPath);
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            ++failures;
            std::printf("FAIL %s: the parallel fixture does not compile\n",
                        what.c_str());
            for (const std::string &error : errors) {
                std::printf("    %s\n", error.c_str());
            }
            return levels;
        }
        CHECK(evaluator.Evaluate(UsdTimeCode(5.0)).valid);
        levels.steps = evaluator.GetPoseStepLevels();
        levels.solvers = evaluator.GetSolverBatchLevels();
        return levels;
    };
    const auto distinct = [](const std::map<SdfPath, size_t> &levels) {
        std::set<size_t> seen;
        for (const auto &[step, level] : levels) seen.insert(level);
        return seen.size();
    };
    for (const bool solversLast : {true, false}) {
        const std::string order =
            solversLast ? "solvers last" : "movers last";
        // Head() always emits the Hip/Knee/Ankle chain of its own; the limbs
        // above are appended beside it and nothing names it, so it is inert.
        const Levels single =
            levelsOf(OpenText(build(false, solversLast)),
                     "one limb, " + order);
        const Levels pair =
            levelsOf(OpenText(build(true, solversLast)),
                     "two limbs, " + order);
        if (single.steps.empty() || pair.steps.empty()) continue;

        CHECK(single.solvers.size() == 2);
        CHECK(pair.solvers.size() == 4);
        CHECK(single.steps.size() == 3);
        CHECK(pair.steps.size() == 6);
        // The second limb adds STEPS, never LEVELS: it is independent, so it
        // runs beside the first and not after it.
        CHECK(distinct(single.solvers) == distinct(pair.solvers));
        CHECK(distinct(single.steps) == distinct(pair.steps));
        // ... and each step sits in exactly its opposite number's level.
        for (const char *name :
             {"Solvers/IK", "Solvers/FK", "Movers/Aim"}) {
            const std::string text(name);
            const std::string scope = text.substr(0, text.find('/') + 1);
            const std::string leaf = text.substr(text.find('/') + 1);
            const SdfPath leftPath("/Asset/Rig/" + scope + "L" + leaf);
            const SdfPath rightPath("/Asset/Rig/" + scope + "R" + leaf);
            const auto left = pair.steps.find(leftPath);
            const auto right = pair.steps.find(rightPath);
            CHECK(left != pair.steps.end() && right != pair.steps.end());
            if (left == pair.steps.end() || right == pair.steps.end()) {
                continue;
            }
            CHECK(left->second == right->second);
            const auto alone = single.steps.find(leftPath);
            CHECK(alone != single.steps.end() &&
                  alone->second == left->second);
            if (left->second != right->second) {
                std::printf("    %s: %s at level %zu, %s at %zu\n",
                            order.c_str(), leftPath.GetText(), left->second,
                            rightPath.GetText(), right->second);
            }
        }
    }
}

}  // namespace

static std::string
SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

/// A solver input's read phase is the one way past the hierarchy.
///
/// The FK chain sits BELOW a position constraint that moves the chain's
/// root CONTROL (not a joint), so by the namespace the constraint revises
/// what the chain already read: the hip stays where the unmoved control
/// put it. `rigExecReadPhase = "final"` on rigExec:controls asks for the
/// control as the constraint left it, the chain waits on the constraint,
/// and the hip follows.
std::string
ControlFollowText(const char *controlsPhase)
{
    const std::string phase =
        controlsPhase ? std::string(" (\n                    "
                                    "rigExecReadPhase = \"") +
                            controlsPhase + "\"\n                )"
                      : std::string();
    std::string text = Head(
        FkControls("Fk", 0) +
        "            def RigExecControl \"Lift\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 20, 6) + "\n"
        "            }\n");
    // Movers defined before Solvers, so Solvers is the bottom sibling and
    // every solver executes before every constraint.
    text +=
        "\n"
        "        def Scope \"Movers\"\n"
        "        {\n"
        "            def RigExecPositionConstraint \"HipFollow\" (\n"
        "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
        "            )\n"
        "            {\n"
        "                float inputs:defaultWeight = 1\n"
        "                rel rigExec:sources = </Asset/Rig/Controls/Lift>\n"
        "                rel rigExec:moves = </Asset/Rig/Controls/FkHip>\n"
        "            }\n"
        "        }\n"
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n"
        "            def RigExecFkChain \"LegFK\"\n"
        "            {\n"
        "                rel rigExec:controls = [ "
        "</Asset/Rig/Controls/FkHip>, "
        "</Asset/Rig/Controls/FkHip/Knee>, "
        "</Asset/Rig/Controls/FkHip/Knee/Ankle> ]" + phase + "\n"
        "                rel rigExec:joints = [ "
        "</Asset/Rig/Joints/Hip>, </Asset/Rig/Joints/Hip/Knee>, "
        "</Asset/Rig/Joints/Hip/Knee/Ankle> ]\n"
        "            }\n"
        "        }\n"
        "    }\n"
        "}\n";
    return text;
}

/// The baked half for a hand-written fixture: CheckParity's comparison,
/// over stages opened from \p text, with the program required.
void
CheckTextParity(const char *what, const std::string &text,
                const std::vector<double> &frames)
{
    const UsdStageRefPtr referenceStage = OpenText(text);
    const UsdStageRefPtr bakedStage = OpenText(text);
    CHECK(referenceStage && bakedStage);
    if (!referenceStage || !bakedStage) return;
    RigExecRigEvaluator reference(referenceStage, kRigPath);
    RigExecRigEvaluator baked(bakedStage, kRigPath);
    std::vector<std::string> errors;
    if (!reference.Compile(&errors) || !baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        return;
    }
    baked.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> reasons;
    CHECK(baked.IsBakeable(&reasons));
    for (const double frame : frames) {
        const std::string where =
            std::string(what) + " frame " + std::to_string(frame);
        const RigExecRigPose a = reference.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose b = baked.Evaluate(UsdTimeCode(frame));
        CHECK(a.valid && b.valid);
        CHECK(b.bakedParityMismatches == 0);
        rigExecTest::ComparePose(&failures, where, a, b);
    }
    CHECK(baked.GetBakedGenerationCount() == frames.size());
}

void
TestSolverInputReadPhaseFollowsAConstraintAbove()
{
    const UsdStageRefPtr plain = OpenText(ControlFollowText(nullptr));
    const UsdStageRefPtr declared = OpenText(ControlFollowText("final"));
    CHECK(plain && declared);
    if (!plain || !declared) return;
    const RigExecRigPose before = Evaluate("no read phase", plain, 1.0,
                                           nullptr);
    const RigExecRigPose after = Evaluate("final read phase", declared,
                                          1.0, nullptr);
    if (!before.valid || !after.valid) return;
    const auto hipBefore = before.jointFramesFinal.find(kHip);
    const auto hipAfter = after.jointFramesFinal.find(kHip);
    CHECK(hipBefore != before.jointFramesFinal.end());
    CHECK(hipAfter != after.jointFramesFinal.end());
    if (hipBefore == before.jointFramesFinal.end() ||
        hipAfter == after.jointFramesFinal.end()) {
        return;
    }
    // The hierarchy: the chain read the control before it was moved.
    CHECK(Near(hipBefore->second.Origin(), GfVec3d(0, 8, 0), 1e-6));
    // The declared phase: the chain read the moved control.
    CHECK(Near(hipAfter->second.Origin(), GfVec3d(0, 20, 6), 1e-6));
    // And the baked program schedules both the same way.
    CheckTextParity("no read phase", ControlFollowText(nullptr), {1, 3, 5});
    CheckTextParity("final read phase", ControlFollowText("final"),
                    {1, 3, 5});
}

/// Constraint B reads a control constraint A moves: B's source is the
/// control itself, or (\p child) a namespace child the move propagates to.
/// A third constraint D, at the bottom of the stack, writes the same target
/// as one of the two and so delays it into a later Kahn level. Mover order
/// adds no edge between constraints, so the constraint -> constraint frame
/// edges alone put B after A or before it; the values pin which.
///   \p aBelowB: D moves Ctl, A moves Ctl again, B reads after A.
///   otherwise:  D moves OutB, B moves it again reading Ctl, A moves Ctl.
std::string
ConstraintReadsConstraintText(bool aBelowB, bool child)
{
    const auto position = [](const char *name, const char *moves,
                             const char *source) {
        return std::string(
                   "            def RigExecPositionConstraint \"") +
               name + "\" (\n"
               "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
               "            )\n"
               "            {\n"
               "                float inputs:defaultWeight = 1\n"
               "                rel rigExec:sources = <" + source + ">\n"
               "                rel rigExec:moves = <" + moves + ">\n"
               "            }\n";
    };
    const char *ctl = "/Asset/Rig/Controls/Ctl";
    const char *read = child ? "/Asset/Rig/Controls/Ctl/Child"
                             : "/Asset/Rig/Controls/Ctl";
    const char *srcA = "/Asset/Rig/Controls/SrcA";
    const char *srcD = "/Asset/Rig/Controls/SrcD";
    const char *outB = "/Asset/Rig/Controls/OutB";
    std::string text = Head(
        "            def RigExecControl \"Ctl\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 0, 0) + "\n"
        "\n"
        "                def RigExecControl \"Child\"\n"
        "                {\n"
        "                    matrix4d rest:space = " + Rest(2, 0, 0) + "\n"
        "                }\n"
        "            }\n"
        "            def RigExecControl \"SrcA\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(10, 0, 0) + "\n"
        "            }\n"
        "            def RigExecControl \"SrcD\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 0, 7) + "\n"
        "            }\n"
        "            def RigExecControl \"OutB\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 5, 0) + "\n"
        "            }\n");
    // Definition order is top to bottom; the stack runs bottom first.
    const std::string a = position("A", ctl, srcA);
    const std::string b = position("B", outB, read);
    const std::string d = aBelowB ? position("D", ctl, srcD)
                                  : position("D", outB, srcD);
    text +=
        "\n"
        "        def Scope \"Movers\"\n"
        "        {\n" +
        (aBelowB ? b + a + d : a + b + d) +
        "        }\n"
        "    }\n"
        "}\n";
    return text;
}

void
TestConstraintReadsAConstraintInStackOrder()
{
    for (const bool aBelowB : {true, false}) {
        for (const bool child : {false, true}) {
            const std::string what =
                std::string(aBelowB ? "A below B" : "A above B") +
                (child ? ", B reads Ctl's child" : ", B reads Ctl");
            const std::string text =
                ConstraintReadsConstraintText(aBelowB, child);
            const UsdStageRefPtr stage = OpenText(text);
            CHECK(stage);
            if (!stage) continue;
            const RigExecRigPose pose =
                Evaluate(what.c_str(), stage, 1.0, nullptr);
            if (!pose.valid) continue;
            const auto ctl =
                pose.controlFrames.find(SdfPath("/Asset/Rig/Controls/Ctl"));
            const auto out =
                pose.controlFrames.find(SdfPath("/Asset/Rig/Controls/OutB"));
            CHECK(ctl != pose.controlFrames.end());
            CHECK(out != pose.controlFrames.end());
            if (ctl == pose.controlFrames.end() ||
                out == pose.controlFrames.end()) {
                continue;
            }
            // A writes Ctl last either way.
            const GfVec3d ctlFinal(10, 0, 0);
            // B reads Ctl at B's place in the stack: after A when A is below
            // it, the rest frame otherwise; the child rides 2 along x.
            const GfVec3d ctlRead = aBelowB ? ctlFinal : GfVec3d(0, 0, 0);
            const GfVec3d expected =
                ctlRead + (child ? GfVec3d(2, 0, 0) : GfVec3d(0));
            CHECK(Near(ctl->second.Origin(), ctlFinal, 1e-6));
            CHECK(Near(out->second.Origin(), expected, 1e-6));
            if (!Near(out->second.Origin(), expected, 1e-6)) {
                const GfVec3d o = out->second.Origin();
                std::printf("    %s: OutB at (%g, %g, %g)\n", what.c_str(),
                            o[0], o[1], o[2]);
            }
            CheckTextParity(what.c_str(), text, {1});
        }
    }
}

/// Constraint B reads beneath a solver-bound joint, the Knee, which an FK
/// chain at the bottom of the stack writes. Constraint A moves \p aMoves:
/// the Knee's parent, the Hip, or the Knee itself. B's source is the Ankle,
/// a provider under the Knee, or (\p xformSource) Tip, a native Xform under
/// it. E and D, at the bottom of the movers, move A's target first, which
/// delays A past B in the Kahn order unless an A -> B edge holds B back;
/// one delaying constraint does not separate them. As above, only the frame
/// edges order A against B; the values pin which writers reach B's source.
/// Mover stack: KneeFK, E, D, A, B.
std::string
ConstraintReadsUnderASolvedJointText(const char *aMoves, bool xformSource)
{
    const auto position = [](const char *name, const std::string &moves,
                             const char *source) {
        return std::string(
                   "            def RigExecPositionConstraint \"") +
               name + "\" (\n"
               "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
               "            )\n"
               "            {\n"
               "                float inputs:defaultWeight = 1\n"
               "                rel rigExec:sources = <" + source + ">\n"
               "                rel rigExec:moves = <" + moves + ">\n"
               "            }\n";
    };
    const std::string moved = std::string("/Asset/Rig/Joints/") + aMoves;
    const char *read = xformSource ? "/Asset/Rig/Joints/Hip/Knee/Tip"
                                   : "/Asset/Rig/Joints/Hip/Knee/Ankle";
    std::string text = Head(
        "            def RigExecControl \"SrcA\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(10, 0, 0) + "\n"
        "            }\n"
        "            def RigExecControl \"SrcD\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 0, 7) + "\n"
        "            }\n"
        "            def RigExecControl \"SrcE\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 6, 6) + "\n"
        "            }\n"
        "            def RigExecControl \"OutB\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(0, 5, 0) + "\n"
        "            }\n"
        "            def RigExecControl \"FkKnee\"\n"
        "            {\n"
        "                matrix4d rest:space = " + Rest(4, 1, 0) + "\n"
        "            }\n");
    const std::string ankle = "                    def RigExecJoint \"Ankle\"";
    text.insert(text.find(ankle),
                "                    def Xform \"Tip\"\n"
                "                    {\n"
                "                        double3 xformOp:translate = "
                "(0, 3, 0)\n"
                "                        uniform token[] xformOpOrder = "
                "[\"xformOp:translate\"]\n"
                "                    }\n");
    const std::string partition =
        "        uniform token rigExec:partition = \"Asset\"\n";
    text.replace(text.find(partition), partition.size(),
                 partition +
                 "        reorder nameChildren = [\"Controls\", "
                 "\"Joints\", \"Movers\", \"Solvers\"]\n");
    text +=
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n" +
        Chain("KneeFK", "</Asset/Rig/Controls/FkKnee>",
              "</Asset/Rig/Joints/Hip/Knee>") +
        "        }\n"
        "\n"
        "        def Scope \"Movers\"\n"
        "        {\n" +
        position("B", "/Asset/Rig/Controls/OutB", read) +
        position("A", moved, "/Asset/Rig/Controls/SrcA") +
        position("D", moved, "/Asset/Rig/Controls/SrcD") +
        position("E", moved, "/Asset/Rig/Controls/SrcE") +
        "        }\n"
        "    }\n"
        "}\n";
    return text;
}

void
TestConstraintReadsUnderASolvedJoint()
{
    for (const char *aMoves : {"Hip", "Hip/Knee"}) {
        for (const bool xformSource : {false, true}) {
            const std::string what =
                std::string("A moves ") + aMoves +
                (xformSource ? ", B reads Tip" : ", B reads Ankle");
            const std::string text =
                ConstraintReadsUnderASolvedJointText(aMoves, xformSource);
            const UsdStageRefPtr stage = OpenText(text);
            CHECK(stage);
            if (!stage) continue;
            const RigExecRigPose pose =
                Evaluate(what.c_str(), stage, 1.0, nullptr);
            if (!pose.valid) continue;
            const auto out =
                pose.controlFrames.find(SdfPath("/Asset/Rig/Controls/OutB"));
            CHECK(out != pose.controlFrames.end());
            if (out == pose.controlFrames.end()) continue;
            // The FK chain puts the Knee at its control, (4, 1, 0); A then
            // moves the Hip from (0, 8, 0) to (10, 0, 0), or the Knee to
            // (10, 0, 0). Tip's stage frame is (0, 3, 0).
            const bool hip = std::string(aMoves) == "Hip";
            GfVec3d expected;
            if (hip) {
                // The Knee stops A's move: the Ankle stays on the solved
                // Knee, while the native Tip rides the Hip's delta.
                expected = xformSource ? GfVec3d(10, -5, 0)
                                       : GfVec3d(8, 1, 0);
            } else {
                // A's own write to the solved Knee reaches both.
                expected = xformSource ? GfVec3d(6, 2, 0)
                                       : GfVec3d(14, 0, 0);
            }
            CHECK(Near(out->second.Origin(), expected, 1e-6));
            if (!Near(out->second.Origin(), expected, 1e-6)) {
                const GfVec3d o = out->second.Origin();
                std::printf("    %s: OutB at (%g, %g, %g)\n", what.c_str(),
                            o[0], o[1], o[2]);
            }
            CheckTextParity(what.c_str(), text, {1});
        }
    }
}

/// Two limbs, each an FK chain with an aim constraint on its knee and a
/// position constraint that pins a Tip control to that knee. The aims share
/// no frame with each other, nor do the tips; each tip reads the frame its
/// own limb's aim writes. Movers are defined LTip, RTip, LAim, RAim, so the
/// stack runs RAim, LAim, RTip, LTip and every adjacent pair in it crosses
/// from one limb to the other.
std::string
IndependentConstraintsText()
{
    const auto controls = [](const char *side, int degrees) {
        const std::string s(side);
        return FkControls((s + "Fk").c_str(), degrees) +
               "            def RigExecControl \"" + s + "Pole\"\n"
               "            {\n"
               "                matrix4d rest:space = " + Rest(4, 12, 3) +
               "\n"
               "            }\n"
               "            def RigExecControl \"" + s + "Tip\"\n"
               "            {\n"
               "                matrix4d rest:space = " + Rest(0, 0, 0) +
               "\n"
               "            }\n";
    };
    const auto joints = [](const char *side) {
        const std::string s(side);
        return
            "            def RigExecJoint \"" + s + "Hip\"\n"
            "            {\n"
            "                matrix4d rest:space = " + Rest(0, 8, 0) + "\n"
            "\n"
            "                def RigExecJoint \"Knee\"\n"
            "                {\n"
            "                    matrix4d rest:space = " + Rest(4, 0, 0) +
            "\n"
            "\n"
            "                    def RigExecJoint \"Ankle\"\n"
            "                    {\n"
            "                        matrix4d rest:space = " + Rest(4, 0, 0) +
            "\n"
            "                    }\n"
            "                }\n"
            "            }\n";
    };
    const auto chain = [](const char *side) {
        const std::string s(side);
        return Chain((s + "FK").c_str(),
                     "</Asset/Rig/Controls/" + s + "FkHip>, "
                     "</Asset/Rig/Controls/" + s + "FkHip/Knee>, "
                     "</Asset/Rig/Controls/" + s + "FkHip/Knee/Ankle>",
                     "</Asset/Rig/Joints/" + s + "Hip>, "
                     "</Asset/Rig/Joints/" + s + "Hip/Knee>, "
                     "</Asset/Rig/Joints/" + s + "Hip/Knee/Ankle>");
    };
    const auto aim = [](const char *side) {
        const std::string s(side);
        return
            "            def RigExecAimConstraint \"" + s + "Aim\" (\n"
            "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
            "            )\n"
            "            {\n"
            "                float inputs:defaultWeight = 1\n"
            "                uniform token rigExec:aimAxis = \"x\"\n"
            "                rel rigExec:aimTarget = "
            "</Asset/Rig/Controls/" + s + "Pole>\n"
            "                rel rigExec:moves = "
            "</Asset/Rig/Joints/" + s + "Hip/Knee>\n"
            "                uniform token[] rigExec:preserve = "
            "[\"origin\", \"scale\"]\n"
            "            }\n";
    };
    const auto tip = [](const char *side) {
        const std::string s(side);
        return
            "            def RigExecPositionConstraint \"" + s + "Tip\" (\n"
            "                prepend apiSchemas = [\"RigExecMoverAPI\"]\n"
            "            )\n"
            "            {\n"
            "                float inputs:defaultWeight = 1\n"
            "                rel rigExec:sources = "
            "</Asset/Rig/Joints/" + s + "Hip/Knee>\n"
            "                rel rigExec:moves = "
            "</Asset/Rig/Controls/" + s + "Tip>\n"
            "            }\n";
    };
    std::string text = Head(controls("L", 30) + controls("R", -20),
                            joints("L") + joints("R"));
    const std::string partition =
        "        uniform token rigExec:partition = \"Asset\"\n";
    text.replace(text.find(partition), partition.size(),
                 partition +
                 "        reorder nameChildren = [\"Controls\", "
                 "\"Joints\", \"Movers\", \"Solvers\"]\n");
    text +=
        "\n"
        "        def Scope \"Solvers\"\n"
        "        {\n" + chain("L") + chain("R") +
        "        }\n"
        "\n"
        "        def Scope \"Movers\"\n"
        "        {\n" + tip("L") + tip("R") + aim("L") + aim("R") +
        "        }\n"
        "    }\n"
        "}\n";
    return text;
}

/// Constraints are parallelized opportunistically: only the frames they read
/// and write order them, never their place in the mover stack alone.
///
/// In the pose schedule the two aims share a Kahn level, as do the two tips,
/// and each tip sits in a later level than its own limb's aim. In the baked
/// op graph no step of one limb's constraints reaches a step of the other's,
/// while each tip stays downstream of its own aim; both paths publish the
/// same pose, with each tip on its own knee.
void
TestIndependentConstraintsShareALevel()
{
    const std::string text = IndependentConstraintsText();
    const SdfPath lAim("/Asset/Rig/Movers/LAim");
    const SdfPath rAim("/Asset/Rig/Movers/RAim");
    const SdfPath lTip("/Asset/Rig/Movers/LTip");
    const SdfPath rTip("/Asset/Rig/Movers/RTip");

    // The dynamic pose schedule.
    {
        const UsdStageRefPtr stage = OpenText(text);
        CHECK(stage);
        if (!stage) return;
        RigExecRigEvaluator evaluator(stage, kRigPath);
        evaluator.cpuParityMode = true;
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            ++failures;
            std::printf("FAIL independent constraints: the fixture does not "
                        "compile\n");
            for (const std::string &error : errors) {
                std::printf("    %s\n", error.c_str());
            }
            return;
        }
        const std::map<SdfPath, size_t> levels =
            evaluator.GetPoseStepLevels();
        for (const SdfPath &step : {lAim, rAim, lTip, rTip}) {
            CHECK(levels.count(step) == 1);
        }
        if (levels.count(lAim) && levels.count(rAim) && levels.count(lTip) &&
            levels.count(rTip)) {
            CHECK(levels.at(lAim) == levels.at(rAim));
            CHECK(levels.at(lTip) == levels.at(rTip));
            CHECK(levels.at(lTip) > levels.at(lAim));
            CHECK(levels.at(rTip) > levels.at(rAim));
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(5.0));
        CHECK(pose.valid);
        for (const char *side : {"L", "R"}) {
            const std::string s(side);
            const auto knee = pose.jointFramesFinal.find(
                SdfPath("/Asset/Rig/Joints/" + s + "Hip/Knee"));
            const auto tipFrame = pose.controlFrames.find(
                SdfPath("/Asset/Rig/Controls/" + s + "Tip"));
            CHECK(knee != pose.jointFramesFinal.end());
            CHECK(tipFrame != pose.controlFrames.end());
            if (knee == pose.jointFramesFinal.end() ||
                tipFrame == pose.controlFrames.end()) {
                continue;
            }
            CHECK(Near(tipFrame->second.Origin(), knee->second.Origin(),
                       1e-6));
        }
        // The limbs curl differently, so each tip found its own knee.
        const auto lTipFrame =
            pose.controlFrames.find(SdfPath("/Asset/Rig/Controls/LTip"));
        const auto rTipFrame =
            pose.controlFrames.find(SdfPath("/Asset/Rig/Controls/RTip"));
        if (lTipFrame != pose.controlFrames.end() &&
            rTipFrame != pose.controlFrames.end()) {
            CHECK(!Near(lTipFrame->second.Origin(),
                        rTipFrame->second.Origin(), 1e-3));
        }
    }

    // The baked op graph.
    {
        const UsdStageRefPtr stage = OpenText(text);
        CHECK(stage);
        if (!stage) return;
        RigExecRigEvaluator evaluator(stage, kRigPath);
        std::vector<std::string> errors;
        if (!evaluator.Compile(&errors)) {
            ++failures;
            return;
        }
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::vector<std::string> reasons;
        CHECK(evaluator.IsBakeable(&reasons));
        CHECK(evaluator.Evaluate(UsdTimeCode(5.0)).valid);
        CHECK(evaluator.GetBakedGenerationCount() == 1);
        const std::vector<RigExecOpGraphNode> graph = evaluator.GetOpGraph();
        CHECK(!graph.empty());
        if (graph.empty()) return;
        // Every step a mover owns (its Constraint step and its commit
        // steps), and its Constraint step alone.
        const auto stepsOf = [&graph](const SdfPath &mover) {
            return rigExecTest::FindOpGraphSteps(graph, "",
                                                 mover.GetString());
        };
        const auto constraintOf = [&graph](const SdfPath &mover) {
            const std::vector<size_t> steps = rigExecTest::FindOpGraphSteps(
                graph, "Constraint", mover.GetString());
            return steps.size() == 1 ? steps[0] : SIZE_MAX;
        };
        for (const SdfPath &mover : {lAim, rAim, lTip, rTip}) {
            CHECK(constraintOf(mover) != SIZE_MAX);
        }
        if (constraintOf(lAim) == SIZE_MAX || constraintOf(rAim) == SIZE_MAX ||
            constraintOf(lTip) == SIZE_MAX || constraintOf(rTip) == SIZE_MAX) {
            return;
        }
        // No path, either way round, between one limb's constraints and the
        // other's.
        const auto reaches = [&](const std::vector<SdfPath> &from,
                                 const std::vector<SdfPath> &to) {
            std::vector<size_t> seeds;
            for (const SdfPath &mover : from) {
                const std::vector<size_t> steps = stepsOf(mover);
                seeds.insert(seeds.end(), steps.begin(), steps.end());
            }
            const std::vector<char> cone =
                rigExecTest::OpGraphForwardCone(graph, seeds);
            for (const SdfPath &mover : to) {
                for (const size_t step : stepsOf(mover)) {
                    if (cone[step]) return true;
                }
            }
            return false;
        };
        CHECK(!reaches({lAim, lTip}, {rAim, rTip}));
        CHECK(!reaches({rAim, rTip}, {lAim, lTip}));
        // The pair that shares a frame stays ordered.
        CHECK(reaches({lAim}, {lTip}));
        CHECK(reaches({rAim}, {rTip}));
        CHECK(!reaches({lTip}, {lAim}));
        CHECK(!reaches({rTip}, {rAim}));
        CHECK(graph[constraintOf(lAim)].level ==
              graph[constraintOf(rAim)].level);
        CHECK(graph[constraintOf(lTip)].level ==
              graph[constraintOf(rTip)].level);
    }

    CheckTextParity("independent constraints", text, {1, 3, 5});
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecSolverStacking <examplesDir>\n");
        return 2;
    }
    const std::string resources = SchemaResourceDir(argv[1]);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    TestCompilesAndChains();
    TestLastWriterWins();
    TestReorderFlipsTheStack();
    TestCheckpointSeesTheEarlierWriter();
    TestPartialSubsetTearIsPinned();
    TestDataFlowOutranksTheNamespace();
    TestDuplicateJointOnOneSolverIsUnauthorable();
    TestFallbackVerdictUnderStack(false);
    TestFallbackVerdictUnderStack(true);
    TestConstraintObservesTopOfStack();
    TestPartialSubsetsAgreeOnOneOrder();
    TestConstraintMediatedOrderIsRespected();
    TestConstraintBelowASolverFeedsIt();
    TestConstraintAboveASolverRevisesIt();
    TestFkChainComposesOverTheIncomingFrame();
    TestReadPhasesOverTheUnifiedStack();
    TestAtPrimMoversRecordsMatchTheStore();
    TestProducersCarryNoStackPosition();
    TestAggregateContradictionIsRejected();
    TestAggregateReadIsHistoryIndependent();
    TestUnrelatedLimbsShareALevel();
    TestSolverInputReadPhaseFollowsAConstraintAbove();
    TestConstraintReadsAConstraintInStackOrder();
    TestConstraintReadsUnderASolvedJoint();
    TestIndependentConstraintsShareALevel();

    // The baked half. Every fixture below stacks and carries no solver-named
    // read phase, so every one of them bakes: the SSA ladder gives each
    // commit on a slot its own version, and two solver commits on one slot is
    // the shape that machinery was built for.
    const std::vector<double> frames{1, 2, 3, 4, 5};
    {
        RigSpec reordered = Stacked();
        reordered.solverOrder = {"LegFK", "LegIK"};
        RigSpec tear = reordered;
        tear.fkControls = "</Asset/Rig/Controls/FkHip>";
        tear.fkJoints = "</Asset/Rig/Joints/Hip>";
        RigSpec dataFlow = Stacked();
        dataFlow.fkControls =
            "</Asset/Rig/Controls/FkHip>, "
            "</Asset/Rig/Joints/Hip/Knee/Ankle>";
        dataFlow.fkJoints =
            "</Asset/Rig/Joints/Hip>, </Asset/Rig/Joints/Hip/Knee>";

        CheckParity("stacked fk + ik", Stacked(), frames, true);
        CheckParity("stacked fk + ik, reordered", reordered, frames, true);
        CheckParity("stacked with a partial subset", tear, frames, true);
        CheckParity("stacked, ordered by data flow", dataFlow, frames, true);

        // Three writers on one joint, and a pose constraint sitting on top of
        // a stacked one: two writers is the shape that is easy to get right
        // by accident, and a constraint over a stack is where the walk order
        // and the commit order have to be the same order on both paths.
        RigSpec three = Stacked();
        three.fk2 = true;
        RigSpec aim = Stacked();
        aim.kneeAim = true;
        CheckParity("three writers on one joint", three, frames, true);
        CheckParity("a constraint over a stacked joint", aim, frames, true);

        // The unified stack, both ways round. A LIVE rest -- a solver
        // measuring from the frame a constraint below it left -- is the one
        // place the baked description stops being a bake-time constant, so
        // parity here is the whole §3.4 mechanism under test.
        RigSpec fed = Stacked();
        fed.fk = false;
        fed.kneeMove = true;
        RigSpec above = fed;
        above.rigOrder = SolversLast();
        RigSpec fedFk = Stacked();
        fedFk.ik = false;
        fedFk.kneeMove = true;
        CheckParity("a constraint BELOW a solver feeds it", fed, frames,
                    true);
        CheckParity("a constraint ABOVE a solver revises it", above, frames,
                    true);
        CheckParity("an fk chain over a constrained joint", fedFk, frames,
                    true);
        // Both solver kinds over the same constrained joint, and the three
        // writers case on top of it: the live rest is the one place the
        // baked description stops being a bake-time constant, and it has to
        // be rebuilt on the same frames the dynamic path overrides with.
        RigSpec fedBoth = Stacked();
        fedBoth.kneeMove = true;
        RigSpec fedThree = fedBoth;
        fedThree.fk2 = true;
        CheckParity("both solvers over a constrained joint", fedBoth, frames,
                    true);
        CheckParity("three writers over a constrained joint", fedThree,
                    frames, true);
    }

    if (failures) {
        std::printf("testRigExecSolverStacking: %d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecSolverStacking: all tests passed\n");
    return 0;
}

// The binary runtime poses the biped's face live.
//
// A poseable bake carries the property chains as programs (rigExecBinary/
// propertyChains.h) and the runtime reads pose-driven blend weights from its
// own interpolators, so setting a control on the runtime moves the mesh
// exactly as authoring the same value moves the evaluator -- face sliders
// that drive only chains included. Checked per channel on every deformed
// mesh and every joint, against the evaluator with the value authored in
// the session layer at the baked frame.
//
// Mode switches read straight off an avar -- the IK/FK blend weight and a
// space switch's active index -- are live too: set with the controls they
// select, each must change the pose and still match the evaluator, and
// clearing them must return the baked pose. Rotation-only (`orient`) spaces
// -- the shoulder swings' default world, the neck's hips -- match too.
// So does the auto clavicle: an arm raised in FK or in IK lifts the
// shoulder, and its avars:autoClav dial is live.
//
// Also: every chain bakes (none skipped), a chain's own output is refused,
// and a default bake carries no chains, so a replay file is unchanged.
//
// Usage: testRigExecRuntimeLiveFace <schema resources dir> <Biped_stack.usda>
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/bake.h"
#include "rigExecBinary/container.h"
#include "rigExecRuntime/runtime.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

int gFailures = 0;

void
Check(bool condition, const std::string &message)
{
    if (!condition) {
        ++gFailures;
        std::printf("FAIL: %s\n", message.c_str());
    }
}

// Exact up to the last bits a float point can differ by.
constexpr double kPointTolerance = 1e-5;

struct Channel {
    const char *control;
    const char *attr;
    double value;
};

// Channels set together. When `modeMatters`, the first channel is a mode
// switch and leaving it out must change the pose.
struct Case {
    std::vector<Channel> channels;
    bool modeMatters = false;
};

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 3) {
        std::printf("usage: %s <schema resources> <Biped_stack.usda>\n",
                    argv[0]);
        return 2;
    }
    PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
    UsdStageRefPtr stage = UsdStage::Open(argv[2]);
    if (!stage) {
        std::printf("cannot open %s\n", argv[2]);
        return 1;
    }
    std::map<std::string, SdfPath> byName;
    for (const UsdPrim &prim : stage->Traverse()) {
        byName[prim.GetName().GetString()] = prim.GetPath();
    }
    RigExecRigEvaluator live(stage, SdfPath("/Biped/Rig"));
    live.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    Check(live.Compile(&errors), "the biped compiles");
    const UsdTimeCode time(1.0);

    // A default bake replays: no chains.
    {
        RigExecBakeOpts replay;
        replay.frames = {1.0};
        RigExecBakeResult result;
        std::string error;
        Check(RigExecBakeToBinary(live, replay, &result, &error),
              "default bake: " + error);
        auto reader = RigExecRuntimeReader::Open(result.bytes.data(),
                                                 result.bytes.size(), &error);
        Check(reader && !reader->HasPropertyChains(),
              "a default bake carries no property chains");
    }

    RigExecBakeOpts options;
    options.frames = {1.0};
    options.overridableInputs = true;
    RigExecBakeResult result;
    std::string error;
    Check(RigExecBakeToBinary(live, options, &result, &error),
          "poseable bake: " + error);
    Check(result.manifestJson.find("\"propertyChains\": 313") !=
              std::string::npos,
          "all 313 property chains bake");
    Check(result.manifestJson.find("\"propertyChainsSkipped\": []") !=
              std::string::npos,
          "no property chain is skipped");
    auto reader = RigExecRuntimeReader::Open(result.bytes.data(),
                                             result.bytes.size(), &error);
    if (!reader || !reader->SetFrame(1.0, &error)) {
        std::printf("FAIL: runtime open: %s\n", error.c_str());
        return 1;
    }
    Check(reader->HasPropertyChains(), "the poseable bake carries chains");

    // A chain's own output is the chain's to write.
    {
        std::string why;
        const SdfPath output =
            byName["eye_l_converge"].AppendProperty(TfToken("avars:ry"));
        Check(!reader->SetAvar(output.GetString(), 1.0, &why),
              "a property chain's output is refused");
    }

    const std::vector<Case> cases = {
        {{{"L_LoArm", "avars:rz", -40.0}}},          // body, elbow correctives
        {{{"M_Jaw", "avars:rx", -15.0}}},            // jaw and its shapes
        {{{"L_UpLid", "avars:ty", -1.5}}},           // blink
        {{{"L_UpLid", "avars:autoSquash", 0.5}}},    // face slider (chains only)
        {{{"M_Mouth", "avars:tx", 1.0}}},
        {{{"L_Brow", "avars:ty", 1.0}}},
        {{{"M_LookRot", "avars:ry", 15.0}}},         // eye look
        {{{"M_LookRot", "avars:convergence", 0.5}}}, // face slider, float avar
        {{{"M_UpLip", "avars:ty", 0.6}}},
        // The arm to IK, then the IK control moved: the blend weight.
        {{{"L_Arm", "avars:ikfk", 1.0}, {"L_ArmIK", "avars:tx", 5.0}}, true},
        // In IK, the hand's space to the chest, then the chest turned.
        {{{"L_ArmIK", "avars:space", 1.0}, {"L_Arm", "avars:ikfk", 1.0},
          {"M_Chest", "avars:rx", 20.0}}, true},
        // The shoulder swings default to world, a rotation-only space: the
        // chest turning carries them without turning them.
        {{{"M_Chest", "avars:rx", 25.0}}},
        // The neck in hips, the same kind of space, under a turned chest.
        {{{"M_Neck", "avars:space", 2.0}, {"M_Chest", "avars:rz", 20.0}},
         true},
        // The arm raised in FK: the auto clavicle lifts the shoulder.
        {{{"L_UpArm", "avars:ry", -80.0}}},
        // Its dial off under the same arm: the shoulder stays.
        {{{"L_Shldr", "avars:autoClav", 0.0}, {"L_UpArm", "avars:ry", -80.0}},
         true},
        // In IK, the hand raised: the clavicle follows the IK estimate.
        {{{"L_Arm", "avars:ikfk", 1.0}, {"L_ArmIK", "avars:ty", 40.0}}, true},
        // Cleared after the switches: the baked pose again.
        {{}},
    };
    // The runtime's points with `channels` set, or empty on failure.
    auto runtimePoints = [&](const std::vector<Channel> &channels,
                             const std::string &label) {
        std::map<std::string, std::vector<RrVec3f>> out;
        reader->ClearAvars();
        for (const Channel &ch : channels) {
            std::string why;
            const SdfPath path =
                byName[ch.control].AppendProperty(TfToken(ch.attr));
            Check(reader->SetAvar(path.GetString(), ch.value, &why),
                  label + " " + ch.attr + " accepted: " + why);
        }
        if (!reader->Execute(&error)) {
            Check(false, label + " runtime Execute: " + error);
            return out;
        }
        for (const auto &points : reader->GetPoints()) {
            out[points.path] = points.points;
        }
        return out;
    };
    for (const Case &c : cases) {
        std::string label = c.channels.empty() ? "cleared" : "";
        bool known = true;
        for (const Channel &ch : c.channels) {
            label += (label.empty() ? "" : "+") + std::string(ch.control) +
                     "." + ch.attr;
            if (byName.find(ch.control) == byName.end()) {
                Check(false, label + ": no such control");
                known = false;
            }
        }
        if (!known) {
            continue;
        }
        if (c.modeMatters) {
            const auto without = runtimePoints(
                std::vector<Channel>(c.channels.begin() + 1, c.channels.end()),
                label + " (without the switch)");
            const auto with = runtimePoints(c.channels, label);
            double moved = 0.0;
            for (const auto &entry : with) {
                const auto it = without.find(entry.first);
                for (size_t i = 0; it != without.end() &&
                                   i < entry.second.size() &&
                                   i < it->second.size(); ++i) {
                    for (int k = 0; k < 3; ++k) {
                        moved = std::max(moved,
                                         std::abs(double(entry.second[i][k]) -
                                                  double(it->second[i][k])));
                    }
                }
            }
            std::printf("%-26s the switch moves %.3g\n", label.c_str(), moved);
            Check(moved > 1e-3, label + ": the mode switch changes the pose");
        }
        runtimePoints(c.channels, label);
        // The reference: the values authored, each in its attribute's type.
        {
            UsdEditContext session(stage, stage->GetSessionLayer());
            for (const Channel &ch : c.channels) {
                const UsdAttribute target = stage->GetAttributeAtPath(
                    byName[ch.control].AppendProperty(TfToken(ch.attr)));
                if (target.GetTypeName() == SdfValueTypeNames->Float) {
                    target.Set(float(ch.value), time);
                } else {
                    target.Set(ch.value, time);
                }
            }
        }
        const RigExecRigPose pose = live.Evaluate(time);
        stage->GetSessionLayer()->Clear();
        double worstPoint = 0.0;
        size_t meshes = 0;
        for (const auto &points : reader->GetPoints()) {
            const auto it = pose.movedProperties.find(SdfPath(points.path));
            if (it == pose.movedProperties.end() ||
                !it->second.IsHolding<VtVec3fArray>()) {
                continue;
            }
            const VtVec3fArray &expected =
                it->second.UncheckedGet<VtVec3fArray>();
            Check(expected.size() == points.points.size(),
                  label + " point count " + points.path);
            ++meshes;
            for (size_t i = 0;
                 i < expected.size() && i < points.points.size(); ++i) {
                for (int k = 0; k < 3; ++k) {
                    worstPoint = std::max(
                        worstPoint, std::abs(double(expected[i][k]) -
                                             double(points.points[i][k])));
                }
            }
        }
        double worstJoint = 0.0;
        for (const auto &joint : reader->GetJointPoseMatrices()) {
            const auto it = pose.jointFramesFinal.find(SdfPath(joint.path));
            if (it == pose.jointFramesFinal.end()) {
                continue;
            }
            const GfVec3d origin = it->second.Origin();
            for (int k = 0; k < 3; ++k) {
                worstJoint = std::max(worstJoint,
                                      std::abs(origin[k] -
                                               joint.matrix[3][k]));
            }
        }
        // Control frames, as a viewer places its manipulators.
        double worstControl = 0.0;
        size_t controls = 0;
        for (const auto &entry : pose.controlFrames) {
            double frame[16];
            if (!reader->GetControlFrame(entry.first.GetString(), frame)) {
                continue;
            }
            ++controls;
            const GfVec3d o = entry.second.Origin();
            const GfVec3d axes[3] = {entry.second.X() - o, entry.second.Y() - o,
                                     entry.second.Z() - o};
            for (int k = 0; k < 3; ++k) {
                worstControl = std::max(worstControl, std::abs(o[k] - frame[12 + k]));
                for (int r = 0; r < 3; ++r) {
                    worstControl = std::max(
                        worstControl, std::abs(axes[r][k] - frame[r * 4 + k]));
                }
            }
        }
        std::printf("%-26s meshes %zu  worst point %.3g  worst joint %.3g"
                    "  controls %zu worst %.3g\n",
                    label.c_str(), meshes, worstPoint, worstJoint, controls,
                    worstControl);
        Check(controls > 0 && worstControl <= kPointTolerance,
              label + ": runtime control frames match the evaluator");
        Check(meshes > 0, label + ": meshes compared");
        Check(worstPoint <= kPointTolerance,
              label + ": runtime points match the evaluator");
        Check(worstJoint <= kPointTolerance,
              label + ": runtime joints match the evaluator");
    }
    if (gFailures) {
        std::printf("%d failure(s)\n", gFailures);
        return 1;
    }
    std::printf("testRigExecRuntimeLiveFace: all checks passed\n");
    return 0;
}

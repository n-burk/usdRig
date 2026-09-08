//
// Probe: disconnect solver bindings, move rest positions, reconnect.
//
// Drives the REAL UsdImaging chain (as probeImagingPipeline does) through the
// interactive sequence a rigger performs in usdview: unbind the solver so the
// joints are free, move rest positions, then rebind. Runs the variants of
// "disconnect" the graph editor can produce, because each authors a different
// edit and they invalidate differently.
//
#include "rigExecImaging/registry.h"

#include "pxr/imaging/hd/sceneIndex.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include "pxr/usdImaging/usdImaging/sceneIndices.h"

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int failures = 0;

void Check(bool condition, const std::string &what)
{
    std::printf("  %-62s %s\n", what.c_str(), condition ? "ok" : "FAIL");
    if (!condition) ++failures;
}

bool ReadXform(const HdSceneIndexBaseRefPtr &si, const SdfPath &path,
               GfMatrix4d *out)
{
    const HdSceneIndexPrim prim = si->GetPrim(path);
    if (!prim.dataSource) return false;
    HdXformSchema schema = HdXformSchema::GetFromParent(prim.dataSource);
    if (!schema || !schema.GetMatrix()) return false;
    *out = schema.GetMatrix()->GetTypedValue(0.0);
    return true;
}

const char *kSolver = "/RigRoot/Solvers/RigExecTwoBoneIk1";
const char *kJoints[3] = {
    "/RigRoot/Joints/Shoulder",
    "/RigRoot/Joints/Shoulder/ankle",
    "/RigRoot/Joints/Shoulder/ankle/foot"};
const double kNewAnkleTx = 2.0;

std::string gExamples;

SdfPath GuidePath(const char *joint)
{
    return SdfPath(joint).AppendChild(TfToken("rigGuideSphere_0"));
}

void Read(const HdSceneIndexBaseRefPtr &si, GfVec3d out[3])
{
    for (int i = 0; i < 3; ++i) {
        GfMatrix4d m(1.0);
        out[i] = ReadXform(si, GuidePath(kJoints[i]), &m)
            ? m.ExtractTranslation() : GfVec3d(-999);
    }
}

void Print(const char *tag, const GfVec3d v[3])
{
    std::printf("    %-14s", tag);
    for (int i = 0; i < 3; ++i)
        std::printf("  (%7.4f %7.4f %7.4f)", v[i][0], v[i][1], v[i][2]);
    std::printf("\n");
}

void MoveRest(const UsdStageRefPtr &stage)
{
    stage->GetAttributeAtPath(
        SdfPath(kJoints[1]).AppendProperty(TfToken("rest:tx")))
        .Set(kNewAnkleTx);
}

// Opens the example on a session-layer edit target and attaches the real chain.
struct Session {
    UsdStageRefPtr stage;
    HdSceneIndexBaseRefPtr terminal;

    explicit Session(bool preMoveRest)
    {
        stage = UsdStage::Open(gExamples + "/components/spider_leg_ik.usd");
        if (!stage) return;
        stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
        if (preMoveRest) MoveRest(stage);
        UsdImagingCreateSceneIndicesInfo info;
        info.stage = stage;
        terminal = UsdImagingCreateSceneIndices(info).finalSceneIndex;
        const long long id =
            UsdUtilsStageCache::Get().Insert(stage).ToLongInt();
        RigExecImaging_Activate(id, "", 0.0);
    }
    ~Session() { RigExecImaging_Deactivate(); }
};

GfVec3d gReference[3];

// Runs one "disconnect -> move rest -> reconnect" variant and compares the
// end state against the never-disconnected reference.
void RunVariant(const char *name,
                const std::function<void(const UsdStageRefPtr &)> &disconnect,
                const std::function<void(const UsdStageRefPtr &)> &reconnect,
                bool evaluateBetweenSteps = true)
{
    std::printf("  variant: %s\n", name);
    Session s(false);
    if (!s.terminal) { Check(false, "stage opens"); return; }

    GfVec3d state[3];
    Read(s.terminal, state);
    Print("initial", state);

    if (evaluateBetweenSteps) {
        disconnect(s.stage);
        Read(s.terminal, state);
        Print("disconnected", state);
        MoveRest(s.stage);
        Read(s.terminal, state);
        Print("rest moved", state);
        reconnect(s.stage);
    } else {
        SdfChangeBlock block;
        disconnect(s.stage);
        MoveRest(s.stage);
        reconnect(s.stage);
    }
    Read(s.terminal, state);
    Print("reconnected", state);

    bool ok = true;
    for (int i = 0; i < 3; ++i)
        ok = ok && GfIsClose(state[i], gReference[i], 1e-6);
    Check(ok, std::string(name) + ": IK re-solved with the moved rest");
}

UsdRelationship Rel(const UsdStageRefPtr &stage, const char *name)
{
    return stage->GetPrimAtPath(SdfPath(kSolver)).GetRelationship(TfToken(name));
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: probeIkReconnect <examplesDir>\n");
        return 2;
    }
    gExamples = argv[1];

    {   // Reference: rest already moved, never disconnected.
        Session s(true);
        if (!s.terminal) { std::printf("cannot open example\n"); return 2; }
        Read(s.terminal, gReference);
        Print("reference", gReference);
    }

    SdfPathVector original;
    {
        Session s(false);
        Rel(s.stage, "rigExec:joints").GetTargets(&original);
    }
    Check(original.size() == 3, "solver binds three joints");

    RunVariant("A clear+restore joints",
        [](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets({}); },
        [&](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets(original); });

    RunVariant("B remove+reauthor rel",
        [](const UsdStageRefPtr &s) {
            s->GetPrimAtPath(SdfPath(kSolver)).RemoveProperty(
                TfToken("rigExec:joints"));
        },
        [&](const UsdStageRefPtr &s) {
            s->GetPrimAtPath(SdfPath(kSolver)).CreateRelationship(
                TfToken("rigExec:joints")).SetTargets(original);
        });

    RunVariant("C drop one target",
        [&](const UsdStageRefPtr &s) {
            Rel(s, "rigExec:joints").SetTargets({original[0], original[1]});
        },
        [&](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets(original); });

    RunVariant("D reconnect reordered",
        [](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets({}); },
        [&](const UsdStageRefPtr &s) {
            Rel(s, "rigExec:joints").SetTargets(
                {original[2], original[1], original[0]});
            Rel(s, "rigExec:joints").SetTargets(original);
        });

    RunVariant("E all four rels cleared",
        [](const UsdStageRefPtr &s) {
            Rel(s, "rigExec:joints").SetTargets({});
            Rel(s, "rigExec:effectorControl").SetTargets({});
            Rel(s, "rigExec:poleControl").SetTargets({});
            Rel(s, "rigExec:rootControl").SetTargets({});
        },
        [&](const UsdStageRefPtr &s) {
            Rel(s, "rigExec:joints").SetTargets(original);
            Rel(s, "rigExec:effectorControl").SetTargets(
                {SdfPath("/RigRoot/Controller/foot")});
            Rel(s, "rigExec:poleControl").SetTargets(
                {SdfPath("/RigRoot/Controller/kneePole")});
            Rel(s, "rigExec:rootControl").SetTargets(
                {SdfPath("/RigRoot/Controller/hip")});
        });

    // usdNoodles authors a reconnect as list(targets) + append, so a joint
    // unbound and rebound on its own comes back at the END of rigExec:joints.
    RunVariant("G unbind mid, rebind appended",
        [&](const UsdStageRefPtr &s) {
            Rel(s, "rigExec:joints").SetTargets({original[0], original[2]});
        },
        [&](const UsdStageRefPtr &s) {
            SdfPathVector t{original[0], original[2], original[1]};
            Rel(s, "rigExec:joints").SetTargets(t);
        });

    RunVariant("H unbind all, rebind one at a time",
        [](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets({}); },
        [&](const UsdStageRefPtr &s) {
            for (const SdfPath &p : original) {
                SdfPathVector t;
                Rel(s, "rigExec:joints").GetTargets(&t);
                t.push_back(p);
                Rel(s, "rigExec:joints").SetTargets(t);
            }
        });

    // usdNoodles resolves a link endpoint to a PROPERTY path whenever the pin
    // it was dragged from names a real property (_resolve_endpoint_path), while
    // the disconnect path -- driven by the rebuilt link, whose joint-side pin
    // name is empty for a bare prim-path target -- resolves to the PRIM path.
    // So an unbind/rebind round trip can come back property-qualified.
    RunVariant("I rebind as property paths",
        [](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets({}); },
        [&](const UsdStageRefPtr &s) {
            SdfPathVector t;
            for (const SdfPath &p : original)
                t.push_back(p.AppendProperty(TfToken("rest:tx")));
            Rel(s, "rigExec:joints").SetTargets(t);
        });

    RunVariant("F clear+restore, one change block",
        [](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets({}); },
        [&](const UsdStageRefPtr &s) { Rel(s, "rigExec:joints").SetTargets(original); },
        /*evaluateBetweenSteps=*/false);

    std::printf("%s (%d failure(s))\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}

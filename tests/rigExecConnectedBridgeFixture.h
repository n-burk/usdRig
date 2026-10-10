#pragma once

// Shared authored fixture; numerical expectations remain in each test.
struct RigExecConnectedBridgeFixture {
    UsdStageRefPtr stage;
    UsdPrim driver,altDriver,target,source,relay,altRelay,bridge,rider,owner,held,goal,moveDriver;
    SdfPathVector joints;
};
template<class XformFn,class ConstraintFn>
static RigExecConnectedBridgeFixture RigExecMakeConnectedBridgeFixture(XformFn MakeXform,ConstraintFn MakeConstraint)
{
    const auto Matrix=[](const GfVec3d &translation=GfVec3d(0)) {
        GfMatrix4d scaling(1.0);scaling.SetScale(GfVec3d(1));
        return scaling*GfMatrix4d(GfRotation(GfVec3d(0,0,1),0.0),translation);
    };
    const auto stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/DriverTarget"), Matrix(GfVec3d(4, 0, 0)));
    const UsdPrim target = MakeXform(stage, SdfPath("/Asset/JointTarget"), Matrix(GfVec3d(5, 0, 0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, double x, double y = 0) {
        const UsdPrim prim = stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(x);
        prim.GetAttribute(TfToken("rest:ty")).Set(y);
        prim.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
        return prim;
    };
    const UsdPrim driver = control("/Asset/Rig/Controls/Driver", 1);
    const UsdPrim altDriver = control("/Asset/Rig/Controls/AltDriver", 2);
    const UsdPrim source = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/Source"), TfToken("RigExecJoint"));
    const UsdPrim altSource = stage->DefinePrim(SdfPath("/Asset/Rig/Joints/AltSource"), TfToken("RigExecJoint"));
    for (const auto &binding : {std::make_pair(driver, source), std::make_pair(altDriver, altSource)}) {
        // Every solver in this fixture lives in the SAME SCOPE as the
        // constraints, because the IK has to read a control whose space
        // resolves through a joint a CONSTRAINT revises -- and under the
        // unified pose stack (spec 4.2) the only thing that can put the IK
        // after that constraint is the composed namespace. Solvers are
        // discovered by type anywhere beneath the rig, so one scope and one
        // `reorder nameChildren` spells the whole order.
        const UsdPrim fk = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers").AppendChild(binding.first.GetName()), TfToken("RigExecFkChain"));
        fk.GetRelationship(TfToken("rigExec:controls")).SetTargets({binding.first.GetPath()});
        fk.GetRelationship(TfToken("rigExec:joints")).SetTargets({binding.second.GetPath()});
    }
    const UsdPrim relay = control("/Asset/Rig/Joints/Source/Relay", 0);
    const UsdPrim altRelay = control("/Asset/Rig/Joints/AltSource/Relay", 0);
    const UsdPrim bridge = control("/Asset/Rig/Controls/Bridge", 0);
    bridge.GetAttribute(TfToken("default:space"))
        .SetConnections({relay.GetPath().AppendProperty(TfToken("parent:space"))});
    // Bridge's own namespace descendants: the connected refresh has to carry
    // the BASE phase of a descendant with its connected ancestor, and has to
    // honour the same ownership boundary the constraint walk honours. Rider
    // inherits its pose and must follow; Owner is written by a solver of its
    // own and must not. Owner has to be solver-bound for the boundary to be
    // observable at all: a descendant the refresh walk visits in its own
    // right recomputes its base frame from its own tap straight afterwards,
    // while a solver-bound joint is skipped, so what the carry leaves on it
    // is what the pose publishes.
    const UsdPrim rider = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Bridge/Rider"), TfToken("RigExecJoint"));
    rider.GetAttribute(TfToken("rest:tx")).Set(1.0);
    rider.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    const UsdPrim owner = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Bridge/Owner"), TfToken("RigExecJoint"));
    owner.GetAttribute(TfToken("rest:tx")).Set(1.0);
    owner.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    // A descendant a constraint owns is skipped by the refresh walk too, so
    // its base phase is exactly what the loop leaves behind.
    const UsdPrim held = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Bridge/Held"), TfToken("RigExecJoint"));
    held.GetAttribute(TfToken("rest:tx")).Set(2.0);
    held.GetAttribute(TfToken("purpose")).Set(TfToken("guide"));
    MakeXform(stage, SdfPath("/Asset/HeldTarget"), Matrix(GfVec3d(9, 0, 0)));
    MakeConstraint(stage, "MoveHeld", "RigExecPositionConstraint", {held.GetPath()})
        .GetRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/HeldTarget")});
    const UsdPrim ownerDriver = control("/Asset/Rig/Controls/OwnerDriver", 7);
    {
        const UsdPrim fk = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/Owner"), TfToken("RigExecFkChain"));
        fk.GetRelationship(TfToken("rigExec:controls"))
            .SetTargets({ownerDriver.GetPath()});
        fk.GetRelationship(TfToken("rigExec:joints")).SetTargets({owner.GetPath()});
    }
    const UsdPrim goal = control("/Asset/Rig/Controls/Goal", 2);
    goal.GetAttribute(TfToken("parent:space"))
        .SetConnections({bridge.GetPath().AppendProperty(TfToken("posed:defaultSpace"))});
    const UsdPrim root = control("/Asset/Rig/Controls/Root", 0);
    const UsdPrim pole = control("/Asset/Rig/Controls/Pole", 0, 5);
    // rest:tx 0/5/10 so the IK's bones MEASURE 5 and 5. These tests
    // used to author absolute lengths, which Compile now rejects.
    SdfPathVector joints;
    {
        double tx = 0.0;
        for (const char *name : {"J0", "J1", "J2"}) {
            const SdfPath path =
                SdfPath("/Asset/Rig/Joints").AppendChild(TfToken(name));
            stage->DefinePrim(path, TfToken("RigExecJoint"))
                .GetAttribute(TfToken("rest:tx")).Set(tx);
            tx += 5.0;
            joints.push_back(path);
        }
    }
    const UsdPrim ik = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/IK"), TfToken("RigExecTwoBoneIk"));
    ik.GetRelationship(TfToken("rigExec:rootControl")).SetTargets({root.GetPath()});
    ik.GetRelationship(TfToken("rigExec:effectorControl")).SetTargets({goal.GetPath()});
    ik.GetRelationship(TfToken("rigExec:poleControl")).SetTargets({pole.GetPath()});
    ik.GetRelationship(TfToken("rigExec:joints")).SetTargets(joints);
    const UsdPrim moveJoint = MakeConstraint(stage, "MoveJoint", "RigExecPositionConstraint", {source.GetPath()});
    moveJoint.GetRelationship(TfToken("rigExec:sources")).SetTargets({target.GetPath()});
    const UsdPrim moveDriver = MakeConstraint(stage, "MoveDriver", "RigExecPositionConstraint", {driver.GetPath()});
    moveDriver.GetRelationship(TfToken("rigExec:sources")).SetTargets({SdfPath("/Asset/DriverTarget")});
    // Bottom sibling first, so this list is the execution order REVERSED:
    // MoveDriver, Driver FK, AltDriver FK, MoveJoint, Owner FK, IK, MoveHeld.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers"))
        .SetChildrenReorder({TfToken("MoveHeld"), TfToken("IK"),
                             TfToken("Owner"), TfToken("MoveJoint"),
                             TfToken("AltDriver"), TfToken("Driver"),
                             TfToken("MoveDriver")});
    return {stage,driver,altDriver,target,source,relay,altRelay,bridge,rider,owner,held,goal,moveDriver,joints};
}

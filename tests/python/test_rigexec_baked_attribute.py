#!/usr/bin/env python
"""The public evaluator has one graph policy; legacy authored policy is inert."""
import sys
from test_rigexec_python import _setup_environment
_setup_environment()
from pxr import Sdf, Usd
import rigexec
_RIG="/Asset/Rig"
_BAKED="rigExec:baked"
def _Check(condition,message):
    if not condition: raise AssertionError(message)

def _MakeRig(baked):
    """A control, a joint and a skinned slab -- one pose and one geometry
    domain, which is the smallest rig that actually bakes.

    `baked` is None to author nothing, or a bool to author it.
    """
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset", "Scope")
    rig = stage.DefinePrim(_RIG, "RigExecRoot")
    if baked is not None:
        attribute = rig.CreateAttribute(_BAKED,Sdf.ValueTypeNames.Bool,custom=True)
        attribute.Set(baked)
    control = stage.DefinePrim("/Asset/Rig/Root", "RigExecControl")
    control.GetAttribute("avars:tx").Set(3.0)
    joint = stage.DefinePrim("/Asset/Rig/Bone", "RigExecJoint")
    joint.GetAttribute("avars:ty").Set(2.0)

    stage.DefinePrim("/Asset/Geom", "Scope")
    mesh = stage.DefinePrim("/Asset/Geom/Slab", "Mesh")
    mesh.GetAttribute("points").Set([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    stage.DefinePrim("/Asset/Rig/Movers", "Scope")
    skin = stage.DefinePrim("/Asset/Rig/Movers/Skin", "RigExecSkinMover")
    skin.ApplyAPI("RigExecMoverAPI")
    skin.GetRelationship("rigExec:moves").SetTargets(
        ["/Asset/Geom/Slab.points"])
    skin.CreateRelationship("rigExec:influences").SetTargets(
        ["/Asset/Rig/Bone"])
    skin.CreateAttribute("rigExec:elementSize", Sdf.ValueTypeNames.Int).Set(1)
    skin.CreateAttribute(
        "rigExec:jointIndices", Sdf.ValueTypeNames.IntArray).Set([0, 0, 0])
    skin.CreateAttribute(
        "rigExec:jointWeights", Sdf.ValueTypeNames.FloatArray).Set(
            [1.0, 1.0, 1.0])
    return stage


def TestRemovedPolicy():
    definition=Usd.SchemaRegistry().FindConcretePrimDefinition("RigExecRoot")
    _Check(_BAKED not in definition.GetPropertyNames(),"retired schema policy still declared")
    for authored in (None,False,True):
        stage=_MakeRig(authored); rig=rigexec.Rig(stage,_RIG)
        for name in ("evaluation_mode","evaluation_mode_source","cpu_parity_mode",
                     "baked_cluster_count","baked_clusters_run_last_generation",
                     "baked_generation_count","baked_steps","baked_clusters",
                     "solver_batch_levels","chain_levels","operation_graph"):
            _Check(not hasattr(rig,name),"retired exposure remains: "+name)
        rig.cpu_reference=True; rig.compile(); pose=rig.evaluate(1.0)
        _Check(pose.valid,"single graph evaluation failed")
        _Check(bool(rig.op_graph()),"no compiled operations")
        _Check(pose.reference_agreements>0 and pose.reference_mismatches==0,
               "independent scalar reference did not agree")
        _Check(not hasattr(pose,"baked_parity_mismatches"),"retired pose counter remains")
        # Explicit authored arithmetic: control translation and bone translation.
        _Check(abs(pose.control_frame("/Asset/Rig/Root").to_matrix4()[12]-3.0)<1e-12,
               "legacy authored policy changed control output")
        _Check(abs(pose.joint_matrix("/Asset/Rig/Bone")[13]-2.0)<1e-12,
               "legacy authored policy changed bone output")

def main():
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv)>1 else None)
    TestRemovedPolicy(); print("RIGEXEC_SINGLE_GRAPH_POLICY_OK"); return 0
if __name__=="__main__": sys.exit(main())

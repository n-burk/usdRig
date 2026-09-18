# Run under bin/_env.sh (it puts the USD python modules on the path).
import os, sys
from pxr import Usd, Sdf
comp = Sdf.Layer.CreateAnonymous("comp.usda")
comp.ImportFromString('''#usda 1.0
def Scope "Module" {
    rel connector:Parent:targets = </Skeleton/head>
    def Scope "Op" {
        rel rigExec:sources = </Module.connector:Parent:targets>
        rel rigExec:direct = </Skeleton/head>
    }
}
def Scope "Skeleton" {
    def Scope "head" {
    }
}
''')
s = Usd.Stage.CreateInMemory()
s.DefinePrim("/Rig/Joints/neck")
a = s.DefinePrim("/Rig/Movers/FaceA")
a.GetReferences().AddReference(comp.identifier, "/Module")
op = s.GetPrimAtPath("/Rig/Movers/FaceA/Op")
print("before instance-site: src", op.GetRelationship("rigExec:sources").GetTargets(),
      "fwd", op.GetRelationship("rigExec:sources").GetForwardedTargets(),
      "direct", op.GetRelationship("rigExec:direct").GetTargets())
a.GetRelationship("connector:Parent:targets").SetTargets([Sdf.Path("/Rig/Joints/neck")])
print("after instance-site: fwd", op.GetRelationship("rigExec:sources").GetForwardedTargets())
print("errors:", [str(e)[:120] for e in s.GetCompositionErrors()][:3])

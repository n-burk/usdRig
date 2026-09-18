# Run under bin/_env.sh (it puts the USD python modules on the path).
import os, sys
from pxr import Usd, Sdf
s = Usd.Stage.CreateInMemory()
p = s.DefinePrim("/A")
for n in ("h","g"): s.DefinePrim("/"+n)
r = p.CreateRelationship("pins")
ok = r.SetTargets([Sdf.Path("/h"), Sdf.Path("/g"), Sdf.Path("/h")])
print("SetTargets dup ok:", ok, r.GetTargets())
# forwarding
comp = s.DefinePrim("/Comp")
c = comp.CreateRelationship("connector:Parent:targets")
c.SetTargets([Sdf.Path("/g")])
op = s.DefinePrim("/Comp/Op")
o = op.CreateRelationship("rigExec:sources")
o.SetTargets([Sdf.Path("/Comp.connector:Parent:targets")])
print("targets:", o.GetTargets(), "forwarded:", o.GetForwardedTargets())

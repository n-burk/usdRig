# Cross-domain inputs

The one-loop evaluator binds every computed input to a typed producer value.
Property revisions can read pose matrices and selected geometry points. A
Ribbon can read a revised driver curve. Volume and curve weights can read
revised sample targets and curves. These are new S9 capabilities; the existing
scalar-reference fixtures remain the contract for previously supported rigs.

Author `rigExecReadPhase` on the consuming attribute or relationship:

| Value | Selected producer |
| --- | --- |
| `base` or absent | Authored property/points version zero; pose frame after solvers and before movers |
| `preceding` | The value entering the consumer on its own chain, otherwise the last stack writer below it |
| `final` | Last version of the named producer |
| Absolute prim path | Newest usable version written at or below that prim |

Connections retain typed traversal and input overrides. A matrix input
connected to a provider's `posed:space` reads its asset-space frame matrix.
There is no implicit points-array-to-vector conversion. To read one point,
author a nonnegative integer `rigExecInputElement` on the consuming `float3`
attribute:

```usda
float3 inputs:value = (0, 0, 0) (
    rigExecInputElement = 1
    rigExecReadPhase = "final"
)
float3 inputs:value.connect = </Asset/Geom/Curve.points>
```

The array must contain the selected element in the current generation. A
negative index or metadata on a different input type is rejected at compile.
An unavailable producer or a shorter array produces a diagnostic naming the
consumer, source and index and uses the existing typed input fallback. It
never reuses a value from an earlier generation. Restoring the source count
restores the read without changing the authoring contract. Selecting scalar
components is not part of this contract.

On a Ribbon, put the phase on `rigExec:driverCurve`. Its driver uses the
selected points version; the rest projection still uses authored default
points. On weight objects, put it on `rigExec:weightTarget`,
`rigExec:sampleSource`, or `rigExec:curve`. Explicit metadata selects that
producer; unannotated weight inputs retain their original raw sampling and
source fallback behavior. A declared missing producer differs from an
available empty array.

All reads become ordinary edges in the same producer graph. Reverse reads
such as geometry to property or geometry to Ribbon are allowed when that
graph is acyclic. The common cycle diagnostic identifies a loop through
property, pose, geometry or weights and sets aside its cyclic operators;
independent operations still run. Exact output equality cuts off
downstream work even when an upstream array changes outside the selected
element. Compiled and binary execution consume value IDs and typed storage;
operation bodies do not query the source stage.

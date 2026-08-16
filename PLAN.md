# RigExec — Implementation Plan (distilled from spec)

Spec source: Google Doc "Character Rig Execution Engine — Architecture Spec & Implementation Plan (v0.1)",
extracted verbatim to `C:\Users\nburk\AppData\Local\Temp\claude\D--work-usdRig\aa82ee88-e819-499c-ade5-7aba06f845cd\scratchpad\spec.md`
(also copied to `D:\work\usdRig\usdRig\docs\spec.md`).

## What RigExec is
A rigging object model + character-computation layer **on top of OpenExec** (OpenUSD 26.08, unchanged).
- Canonical pose value: **RigExecPointFrame** — 4 points [O, X, Y, Z] (origin + transformed basis endpoints);
  round-trips exactly to nonsingular affine matrix (incl. scale/shear).
- All durable authoring is USD. Schemas: RigExecRoot, RigExecControl, RigExecJoint, RigExecFkChain,
  RigExecTwoBoneIk, RigExecBlendPointFrames, RigExecAimConstraint, math movers, RigExecTwistDistribution,
  RigExecRibbon, RigExecPointFrameView, blend shape mover/input/sample, RigExecMatrixMover,
  weight objects (static/dynamic), lattice/curve/surface/post movers, applied APIs
  (RigExecMoverAPI, RigExecPointTransformAPI, RigExecControlAPI, RigExecPartitionAPI, RigExecTapAPI).
- Movers author `rel rigExec:moves` → exact prim/property they modify. Compiler walks composed `Movers`
  namespace post-order (descendants first, composed child order) → SSA chain of immutable revisions
  base → afterMover(i) → final. Last writer in logical order wins.
- Geometry stays native UsdGeom (points point3f[], normals, extent, widths, primvars). No parallel geometry schema.
- Extraction: RigExecValueAddress / RigExecTapSet / RigExecSnapshot (§9). Every semantic transform publishes
  paired computePointFrame/computeMatrix.
- Hydra: 3 filtering scene indices (pruning / binding-resolving / results overlay) — Phase 3.
- Standalone Esf backend + .rigpack — Phase 0/4 (stop-gate if Esf unusable).
- Animation: stock USD value resolution only (.spline via Ts for scalars, timeSamples otherwise).

## Key math (§5)
- Frame reconstruction policies: affine | orthogonal (default) | axial | rigid.
  orthogonal: ex = normalize(px-po); ey = normalize((py-po) - ex*dot); ez = ex×ey; ey = ez×ex; twist rotates ey/ez.
  scales: sx=|a|/lx, sy=|u|/ly, dz=(pz-po)·ez, sz=|dz|/lz, σz=sign(dz).
- PointsToMatrix: B_Q=[qx-qo, qy-qo, qz-qo], B_P likewise; L = B_P B_Q^-1; t = po - L*qo. M=[L t;0 1] column-vector convention.
  GfMatrix4d is row-major/row-vector: conformance = transform each q_i, compare p_i (avoid convention assumptions).
- Degeneracy: ε = 1e-10 * max(1, lx, ly, lz); fallback: parent up → rest up → world axis least parallel to ex.
- SRT: SVD L=UΣVᵀ, δ=det(UVᵀ), reflection pinned to reflectionAxis; R=UDVᵀ, H=RᵀL.
- Two-bone IK: analytic, pole vector, stretch/softness, preferredBendRadians (§4.5 params:
  upperLength, lowerLength, stretchPolicy uniformSegments, unreachablePolicy clampWithSoftness).
- Blend: rotationBlend shortestArc, scaleBlend log; weight 0 = A, 1 = B.
- Matrix mover: p' = q + w*(T q - q), weight in [0,1].
- Blend shapes: p'_i = p_i + Σ_k α_k(w_k) d_{k,i}, deltas derived from native target shapes vs base points.

## Scope for this effort (evidence-gated phases 0–1, plus as much 2–3 as feasible)
1. Build OpenUSD v26.08 (PXR_BUILD_EXEC=ON default) → D:\work\usdRig\usd-install. [in progress]
2. `usdRig/` project (CMake, C++17) against the install:
   - **rigExecMath**: RigExecPointFrame, reconstruction policies, PointsToMatrix/MatrixToPoints, SRT/SVD,
     two-bone IK, frame blend, twist distribution kernels. Pure, no USD deps beyond gf/vt.
   - **rigExecSchema**: schema.usda + usdGenSchema-generated C++ (subset: Rig, Control, Joint, FkChain,
     TwoBoneIk, BlendPointFrames, PointFrameView, applied APIs; movers as feasible).
   - **rigExecCompute / rigExecUsd**: ExecTypeRegistry registration of RigExecPointFrame (+array),
     EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA for computePointFrame/computeMatrix on Control/Joint,
     FK chain aggregate + per-joint scalar providers, IK, blend; RigExecTapSet/Snapshot façade over
     ExecUsdSystem/ExecUsdRequest/ExecUsdCacheView.
   - Assets: examples/ArmRig.usda + ArmShotAnim.usda (verbatim from spec §4.5/4.6, minus not-yet-supported prims if needed).
   - Tests: point-frame round-trip incl. reflection/shear; policy reconstruction; IK reach/stretch;
     exec evaluation of ArmRig joints over time vs direct math parity; timed Get() vs exec parity.
3. Defer (documented, not implemented): full mover/geometry chains & session-layer lowering, Hydra filters,
   standalone Esf backend, .rigpack, mobile. Structure code so these bolt on.

## Real exec API notes (fill in as verified from source)
- pxr/exec: vdf, ef, esf, esfUsd, exec, execUsd, execGeom, execIr all present in v26.08.
- Registration macro: EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA (pxr/exec/exec/registerSchema.h) — verify exact spelling.
- ExecUsdSystem / ExecUsdRequest / ExecUsdValueKey / ExecUsdCacheView in pxr/exec/execUsd.
- Types: ExecTypeRegistry::RegisterType — verify signature.
- Build outputs: usd-install/{include,lib,bin,plugin}.

## Build facts
- VS2022 Community, CMake 4.0.2 (user PATH), Ninja via pip, Python 3.10.8.
- Build cmd: D:\work\usdRig\build_usd.bat (vcvars64 + build_usd.py --generator Ninja
  --no-materialx --no-usdview --no-examples --no-tutorials --no-docs D:\work\usdRig\usd-install).
- usdGenSchema at usd-install/bin/usdGenSchema (needs PYTHONPATH=usd-install/lib/python, PATH+=usd-install/lib, usd-install/bin).

# Method references

These public sources describe dependencies and numerical techniques used in
RigExec. They are references for implementation and review, not claims of
affiliation, endorsement, or ownership of the underlying techniques.

| Area | Public source | RigExec code |
|---|---|---|
| Typed dependency evaluation | [OpenExec introduction](https://openusd.org/release/intro_to_openexec.html) and [system design](https://openusd.org/dev/api/page__execution__system__design.html) | `libs/rigExec` |
| Scene-index publication | [Hydra getting started](https://openusd.org/dev/api/_page__hydra__getting__started__guide.html) | `libs/rigExecImaging` |
| Dual-quaternion skinning | Kavan et al., *Geometric Skinning with Approximate Dual Quaternion Blending* (2008), [author project page](https://users.cs.utah.edu/~ladislav/dq/index.html) | `libs/rigExecMath/dualQuat.cpp` |
| Delta Mush | Mancewicz, Derksen, and Wilson, *Delta Mush: Smoothing Deformations While Preserving Detail* (2014), [publisher entry](https://dl.acm.org/doi/10.1145/2614106.2614144) | `libs/rigExecMath/deltaMushKernel.h` |
| Position projection concepts | Müller, Heidelberger, Hennix, and Ratcliff, *Position Based Dynamics* (2007), [author-hosted paper](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf), [DOI](https://doi.org/10.1016/j.jvcir.2007.01.005) | `libs/rigExecMath/wrinkleKernel.h` |
| Wrinkle reference lengths and bounded offsets | Müller and Chentanez, *Wrinkle Meshes* (2010), [author-hosted paper](https://matthias-research.github.io/pages/publications/wrinkleMeshes.pdf), [DOI](https://doi.org/10.2312/SCA/SCA10/085-091) | `libs/rigExecMath/wrinkleKernel.h`; [method and limitations](concepts/wrinkle-deformation.md) |
| Lattice (free-form) deformation | Sederberg and Parry, *Free-Form Deformation of Solid Geometric Models*, ACM SIGGRAPH Computer Graphics 20(4):151–160 (1986), [DOI](https://doi.org/10.1145/15886.15903) | `libs/rigExecMath/latticeKernel.h` |
| Ray-triangle intersection | Möller and Trumbore, *Fast, Minimum Storage Ray-Triangle Intersection*, Journal of Graphics Tools 2(1):21–28 (1997), [publication entry](https://en-cg-web.coecis.cornell.edu/pubs/1997/MT97.html) | `libs/rigExecMath/surfaceProjectorKernel.h` (surface projector, `RigExecRaycastSurface`) |
| Swing-twist decomposition | Dobrowolski, *Swing-twist decomposition in Clifford algebra* (2015), [arXiv:1506.05481](https://arxiv.org/abs/1506.05481) | `libs/rigExec/solverKernels.cpp` (space switch rotation filters), `libs/rigExecMath/rbf.cpp` |

Implementation details and limitations are documented with each operator.
In particular, referencing Delta Mush does not claim implementation of the
distinct Direct Delta Mush method. Third-party source and asset provenance
is tracked separately in [third-party notices](../THIRD_PARTY_NOTICES.md).
The wrinkle mover independently adapts position projection, reference lengths,
and bounded offsets using a deterministic fold guide and affine edge targets.
Its guide and target equations are documented in the
[method guide](concepts/wrinkle-deformation.md). It does not reproduce the
papers' nonlinear distance projection, the separate refined mesh, or the
persistent offset state of *Wrinkle Meshes*. Its public behavior references
do not imply use of proprietary implementation code.

## Persistent surface bindings

`RigExecSurfaceBindingMover` evaluates generalized barycentric positions and
offsets in an orthonormal tangent/bitangent/normal frame. Polygon normals use
Newell's area vector; optional smooth normals interpolate normalized,
area-weighted vertex normals. The Blender
[Surface Deform evaluator](https://github.com/blender/blender/blob/main/source/blender/modifiers/intern/MOD_surfacedeform.cc)
is the reference for weighted polygon bindings and normal-offset import.
Limit position and tangent derivatives are supplied as factored control-point
stencils, as described by the
[OpenSubdiv limit stencil factory](https://graphics.pixar.com/opensubdiv/docs/far_overview.html).
The runtime consumes these immutable attributes without tessellation or rebinding.

## Extended deformation settings

The independent Blender behavior fixtures cover
[Corrective Smooth](https://github.com/blender/blender/blob/main/source/blender/modifiers/intern/MOD_correctivesmooth.cc),
[nearest-surface shrinkwrap](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/shrinkwrap.cc),
and [lattice deformation](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/lattice_deform.cc).
The shared, stage-free kernels implement the selected mathematical behavior;
Blender-generated fixture coordinates test the results independently.

Newly authored surface, Delta Mush, or lattice settings require updated runtime
consumers. Binary exports mark these semantics using wire revision aliases
19 (SurfaceProject), 20 (DeltaMush), and 21 (Lattice). Current readers normalize
these to the existing operations; older readers reject the unsupported opcode.
Exports using only legacy settings retain their original operation codes.
USD assets must likewise be opened with the updated schema and runtime.


## Affine frame computation contracts

The stage-free `libs/rigExecMath/affineFrameKernels.cpp` implements joint
inheritance, affine frame copying/mapping, weighted rest-to-pose parenting,
constraint space conversion, tracking, stretch and location remapping.
`libs/rigExec/affineFrameComputations.cpp` only reads declared Exec inputs and
adapts frame providers into these value kernels. Source behavior references
include Blender's [constraint evaluator](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/constraint.cc),
[joint inheritance](https://github.com/blender/blender/blob/main/source/blender/blenkernel/intern/armature.cc),
and [rotation channel extraction](https://github.com/blender/blender/blob/main/source/blender/blenlib/intern/math_rotation_c.cc).
The shared schemas define the runtime contracts independently of the file
format that authored them. Independent source fixtures cover local-space
location remapping, inherited nonuniform transforms and sequential skinning.

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

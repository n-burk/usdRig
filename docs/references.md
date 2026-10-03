# Method references

These public sources describe dependencies and numerical techniques used in
RigExec. They are references for implementation and review, not claims of
affiliation, endorsement, or ownership of the underlying techniques.

| Area | Public source | RigExec code |
|---|---|---|
| Typed dependency evaluation | [OpenExec introduction](https://openusd.org/release/intro_to_openexec.html) and [system design](https://openusd.org/dev/api/page__execution__system__design.html) | `libs/rigExec` |
| Scene-index publication | [Hydra getting started](https://openusd.org/dev/api/_page__hydra__getting__started__guide.html) | `libs/rigExecImaging` |
| Curvenet articulation | de Goes, Sheffler, and Fleischer, *Character Articulation through Profile Curves* (2022), [author-hosted paper](https://graphics.pixar.com/library/ProfileMover/paper.pdf), [DOI](https://doi.org/10.1145/3528223.3530060) | `curvenet.cpp`, `cutMesh.cpp`, `profileMover.cpp` in `libs/rigExecMath` |
| Curvenet weight interpolation | Talbot, Sheffler, and de Goes, *Face Rigging through Curvenet Parametrization* (2026), [author publication list](https://fdegoes.github.io/), [DOI](https://doi.org/10.1145/3799818.3812074) | `libs/rigExecMath/curvenetWeights.cpp` |
| Dual-quaternion skinning | Kavan et al., *Geometric Skinning with Approximate Dual Quaternion Blending* (2008), [author project page](https://users.cs.utah.edu/~ladislav/dq/index.html) | `libs/rigExecMath/dualQuat.cpp` |
| Delta Mush | Mancewicz, Derksen, and Wilson, *Delta Mush: Smoothing Deformations While Preserving Detail* (2014), [publisher entry](https://dl.acm.org/doi/10.1145/2614106.2614144) | `libs/rigExecMath/deltaMushKernel.h` |

Implementation details and limitations are documented with each operator.
In particular, referencing Delta Mush does not claim implementation of the
distinct Direct Delta Mush method. Third-party source and asset provenance
is tracked separately in [third-party notices](../THIRD_PARTY_NOTICES.md).

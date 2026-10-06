# Third-party notices and provenance

The root [MIT license](LICENSE) covers original RigExec code and documentation.
It does not replace licenses or grant rights in material owned by others.

## Bundled code

`plugin/usdNoodles` is a locally modified version of the node editor proposed
in [OpenUSD pull request 4156](https://github.com/PixarAnimationStudios/OpenUSD/pull/4156).
Its files carry Meta Platforms copyright notices and retain the
[Tomorrow Open Source Technology License 1.0](plugin/usdNoodles/LICENSE.txt).
Local integration includes rig authoring, layout, picking, and viewer changes.
This component is not relicensed under MIT. Its copyright and license notices
must accompany source and binary distributions.

OpenUSD is an external dependency with its own
[license](https://github.com/PixarAnimationStudios/OpenUSD/blob/v26.08/LICENSE.txt).
Generated schema resources contain inherited upstream schema documentation.
Python, Qt/PySide, and other dependencies retain their respective terms;
the root license does not relicense a dependency installation.

The OpenUSD Gf-derived math in `libs/rigExecRuntime/runtimeMath.h` and the
adapted widgets in `plugin/rigExecUsdview/volumeWeightUI.py` also retain
OpenUSD license terms and their Pixar notices.

The native noodles dependency is pinned at
`ff5d473f10e8c37ceaf0da11ea7cb80805bc8314`. Its copied GLSL shaders retain
Meta's [MIT license](plugin/usdNoodles/NOODLES_LICENSE.txt). The Poppins
font atlases retain the SIL Open Font License 1.1, reproduced in
[upstream third-party licenses](plugin/usdNoodles/THIRD_PARTY_LICENSES.txt).
Those font terms differ from both the root MIT and the editor's OpenUSD terms.
The distribution [NOTICE](NOTICE) records these component boundaries.

## Published methods

Dual-quaternion skinning and Delta Mush have public research references listed in
[Method references](docs/references.md). Those references identify techniques,
not a claim that RigExec invented them or an endorsement by their authors.
A citation alone does not establish rights to source code or assets.

## Provenance requiring owner review

- `examples/biped` contains geometry, corrective shapes, and rig data
  converted from a character and rig by
  [Squarebit Studios](https://www.squarebitstudios.com/)
  ([Matt Schiller](https://github.com/matthewschiller),
  [Walt Yoder](https://github.com/wyoder)). It is retained in the
  repository at the owner's request. A redistribution license for that
  material is not documented here yet.

The biped entry now identifies its source author; its redistribution
terms are still undocumented. Before a public distribution, the owner must document
ownership or permission for the affected material. No claim of an
entirely MIT-licensed distribution is made here.

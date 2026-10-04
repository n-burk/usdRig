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

`thirdparty/flatbuffers` holds unmodified C++ runtime headers of
[Google FlatBuffers](https://github.com/google/flatbuffers) 25.12.19 (tag
`v25.12.19`, commit `7e163021e59cca4f8e1e35a7c828b5c6b7915953`) under the
Apache License 2.0 ([license](thirdparty/flatbuffers/LICENSE)). The generated
headers in `libs/rigExecBinary/generated` are `flatc` output from RigExec
schemas. FlatBuffers is not relicensed under MIT, and its license accompanies
source and binary distributions that contain it, including builds that
compile it in.

The native noodles dependency is pinned at
`ff5d473f10e8c37ceaf0da11ea7cb80805bc8314`. Its copied GLSL shaders retain
Meta's [MIT license](plugin/usdNoodles/NOODLES_LICENSE.txt). The Poppins
font atlases retain the SIL Open Font License 1.1, reproduced in
[upstream third-party licenses](plugin/usdNoodles/THIRD_PARTY_LICENSES.txt).
Those font terms differ from both the root MIT and the editor's OpenUSD terms.
The distribution [NOTICE](NOTICE) records these component boundaries.

The Squarebit Eye shader, `examples/biped/shaders/sbe_eye.glslfx`, is
licensed under the
[Squarebit Eye End User License Agreement](https://www.squarebitstudios.com/squarebit-eye/eula),
as its header states. It is not relicensed under MIT.

## Published methods

Dual-quaternion skinning and Delta Mush have public research references listed in
[Method references](docs/references.md). Those references identify techniques,
not a claim that RigExec invented them or an endorsement by their authors.
A citation alone does not establish rights to source code or assets.


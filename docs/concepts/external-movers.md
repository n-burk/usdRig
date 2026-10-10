---
title: External mover plugins
summary: Build and register point movers from a separate repository for native, frozen and .rigexec playback.
order: 50
---

Movers can live in a separate repository and register with RigExec through a
shared library. The host discovers the library through
OpenUSD's plugin registry. Each registered type supplies a parameter assembly
callback and a points computation; no changes to RigExec's built-in mover list
are required.

Use the same compiler, architecture, build configuration, OpenUSD installation,
and RigExec SDK as the host. Plugin API version 5 combines declared external
input leaves, explicit phased oracle lookup callbacks and the optional
provider-aware playback kernel. Libraries built for API 1, 2 or 3 must be
rebuilt and migrate their assembly callback; libraries built for API 4 must be
rebuilt.

`declareExternalInputs` runs only at compilation and declares ordered typed
`RigExecRevisionLeafKey` records. Each record specifies its canonical property
path, type, time policy, read flavour and missing-value fallback. Declared scalar
inputs receive normal evaluator override numbers; array inputs follow the
existing public and private array admission policy.

`assembleExternal` receives `RigExecExternalInputContext`: the immutable binding,
explicit provider values and sampled input values in declaration order. It must
compute a payload without querying a stage, registry or mutable shared state.
`applyExternal` computes the full-strength candidate from that payload and the
preceding points. The engine owns enable, envelope and failure handling.

Oracle handlers receive `phasedPoints` and `phasedMatrix` lookups with the exact
provider path, authored phase and reader identity. A `preceding` read of the
oracle's own target uses `entering`; the first revision falls back to base.
The lookups expose current-generation finished publications and bound record
versions supplied by the adapter. Cached values alone do not establish a
finished publication.

## Build a mover repository

In the external repository, create a `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.26)
project(MyMovers LANGUAGES CXX)

if(NOT TARGET rigExec::rigExec)
    find_package(rigExec CONFIG REQUIRED)
endif()

rigexec_add_mover_plugin(MyMovers SOURCES mover.cpp)
```

`rigexec_add_mover_plugin` links `rigExec::rigExec`, enables C++17, generates
OpenUSD plugin metadata, and installs the library and metadata. Add
`LIBRARIES otherTarget` for dependencies or `NO_INSTALL` for a build-only plugin.
Use `PLUGIN_ROOT path` to place build metadata under a separate discovery root,
for example `${CMAKE_BINARY_DIR}/test-plugins` for a test fixture. Relative paths
resolve from the current binary directory. The default root is
`${CMAKE_BINARY_DIR}/usd/rigExecMoverPlugins`.
The target property `RIGEXEC_PLUGIN_RESOURCE_DIR` contains its generated
resources path, including a CMake configuration generator expression.

To include that repository in the main usdRig build, set its actual location
before invoking the existing helper. For example, in PowerShell:

```powershell
$env:RIGEXEC_MOVER_PLUGIN_DIRS = 'D:/src/mover-plugins'
bin/build_rigexec.bat
```

Or in a POSIX shell:

```sh
RIGEXEC_MOVER_PLUGIN_DIRS=/path/to/mover-plugins bin/build_rigexec.sh
```

For manual CMake configuration, pass
`-DRIGEXEC_MOVER_PLUGIN_DIRS=/path/to/mover-plugins` with the normal build
arguments. Multiple repositories use a quoted semicolon-separated list. Each
directory must contain a `CMakeLists.txt` that defines its mover plugin targets.
The build and launch helpers include `build/usd/rigExecMoverPlugins` in
`PXR_PLUGINPATH_NAME`, so these plugins are discovered when launching usdview.

The same repository can build independently against an installed SDK:

```sh
cmake -S /path/to/mover-plugins -B mover-build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_PREFIX_PATH=/path/to/rigexec-install;/path/to/usd-install"
cmake --build mover-build
cmake --install mover-build --prefix /path/to/mover-install
```

## Implement and register a mover

This complete `mover.cpp` translates the preceding point revision by an
animated `inputs:offset`. Its payload uses `GfVec3f`, which already supports
the value comparison needed by `VtValue` and the evaluation cache.

```cpp
#include "rigExec/movers/moverRegistry.h"
#include "rigExecGraph/sceneDescriptors.h"

#include <cmath>

PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;

namespace {

bool Validate(const RigExecMoverValidateContext &ctx, std::string *error)
{
    const UsdAttribute attr = ctx.prim.GetAttribute(TfToken("inputs:offset"));
    if (!attr || attr.GetTypeName() != SdfValueTypeNames->Float3) {
        *error = "SampleOffsetMover requires float3 inputs:offset";
        return false;
    }
    return true;
}

void DeclareInputs(const RigExecMoverBindContext &ctx,
                   std::vector<RigExecRevisionLeafKey> *inputs)
{
    inputs->push_back({ctx.moverPrim.GetPath().AppendProperty(TfToken("inputs:offset")),
        RigExecRevisionLeafType::Vec3f, RigExecRevisionLeafTime::AtTime,
        RigExecRevisionLeafFlavour::Resolved, VtValue(GfVec3f(0))});
}

bool CompileScene(const RigExecSceneDescriptors &scene,const SdfPath &mover,
                  const SdfPath &,RigExecRevisionBinding *binding,std::string *error)
{
    const auto path=mover.AppendProperty(TfToken("inputs:offset"));
    const auto found=scene.attributes.find(path);
    if(found==scene.attributes.end() || found->second.fact.type!=SdfValueTypeNames->Float3) {
        *error="SampleOffsetMover requires float3 inputs:offset";return false;
    }
    binding->externalInputs={{path,RigExecRevisionLeafType::Vec3f,
        RigExecRevisionLeafTime::AtTime,RigExecRevisionLeafFlavour::Resolved,VtValue(GfVec3f(0))}};
    return true;
}

bool Assemble(const RigExecExternalInputContext &ctx, VtValue *data)
{
    if (ctx.inputs.size() != 1 || !ctx.inputs[0].IsHolding<GfVec3f>()) return false;
    const auto &offset = ctx.inputs[0].UncheckedGet<GfVec3f>();
    if (!std::isfinite(offset[0]) || !std::isfinite(offset[1]) || !std::isfinite(offset[2])) return false;
    *data = VtValue(offset);
    return true;
}

bool Apply(const VtValue &data, std::vector<GfVec3f> *points)
{
    if (!points || !data.IsHolding<GfVec3f>()) return false;
    const GfVec3f &offset = data.UncheckedGet<GfVec3f>();
    for (GfVec3f &point : *points) point += offset;
    return true;
}

RigExecMoverHandler MakeHandler()
{
    RigExecMoverHandler handler(
        "SampleOffsetMover", RigExecFixedMoverOp<RigExecRevisionOp::External>,
        RigExecMoverDomain::Points);
    handler.singleTarget = true;
    handler.hasScalarOracle = false;
    handler.validate = Validate;
    handler.declareExternalInputs = DeclareInputs;
    handler.compileScene = CompileScene;
    handler.assembleExternal = Assemble;
    handler.applyExternal = Apply;
    return handler;
}

RIGEXEC_REGISTER_MOVER(MakeHandler());
} // namespace
```

Registration keeps a copy of the handler and its type name. Duplicate names,
missing callbacks, and external handlers outside the points domain are rejected.
Hosts that need a returned registration error can call
`RigExecRegisterMoverHandler(handler, &error)` directly. Keep every plugin
library loaded for the lifetime of its evaluators and payload values.

`assembleExternal` computes an immutable, equality-comparable payload from its
sampled declarations. Only compilation and the engine's sampling boundary may
query the source scene. Custom payloads must be copyable and equality-comparable.

`applyExternal` performs pure, thread-safe computation over that payload and
the incoming points. Preserve the point count and produce a full-strength
candidate. RigExec handles `inputs:enabled`, `inputs:defaultWeight`, and the
weight-object envelope. Returning false fails the mover and preserves the
preceding revision. Optional `bind`, `validate`, and `oracle` callbacks support
additional bindings, compile checks, and scalar parity comparisons; omit an
oracle only when reporting that parity comparison is unavailable is acceptable.

## Author a plugin mover

Use the registered type name and apply `RigExecMoverAPI`. A separate generated
USD schema is optional; it can provide defaults and richer authoring tools, but
the evaluator accepts the registered type with explicitly authored attributes.

```usda
#usda 1.0
def Xform "Asset"
{
    def RigExecRoot "Rig"
    {
        def SampleOffsetMover "Offset" (
            prepend apiSchemas = ["RigExecMoverAPI"]
        )
        {
            rel rigExec:moves = </Asset/Geom.points>
            float inputs:defaultWeight = 1
            float3 inputs:offset = (0, 1, 0)
        }
    }
    def Mesh "Geom"
    {
        point3f[] points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        int[] faceVertexCounts = [3]
        int[] faceVertexIndices = [0, 1, 2]
        uniform token subdivisionScheme = "none"
    }
}
```

The generic validator checks the native `UsdGeomPointBased` `point3f[]` target
and the handler's single-target rule before the custom validator runs. Mover
ordering and envelopes follow the same rules as built-in geometry movers.

A plugin schema can give its types a node-editor icon. Declare it on the class
and ship the image in the schema plugin's resources; the usdNoodles editor shows
it on every prim of that type, or of a type derived from it, that authors no
`ui:nodegraph:node:icon`. An authored icon still wins.

```usda
class MySampleOffsetMover "MySampleOffsetMover" (
    inherits = </Typed>
    customData = {
        dictionary extraPlugInfo = {
            string nodeGraphIcon = "icons/sample_offset_mover.png"
        }
    }
)
{
}
```

`usdGenSchema` copies `extraPlugInfo` into the type's `plugInfo.json` entry, and
`nodeGraphIcon` is a path relative to that plugin's resources directory. The
RigExec schema declares its own node types' icons the same way.

## Export and play back a plugin mover

A `.rigexec` file is played by a runtime that has no stage, so it cannot call
`assembleExternal`. Two optional handler fields let a rig holding the mover
export and replay:

- `encodeExternal` runs at export, once per baked frame, on the payload
  `assembleExternal` produced. It splits the payload into epoch bytes, which
  must be identical on every frame and are written once, and frame bytes,
  which are written per frame (identical frames share storage).
- `runtimeKernel` is the playback half, declared in
  `rigExecBinary/external.h` without any USD types. `prepare` decodes a
  revision's epoch bytes once per opened file; `apply` revises the preceding
  points in place from that state, the frame's bytes, and the points the
  binding reads at a declared phase.

The bytes are the plugin's own format; RigExec stores them without reading
them. Start the epoch bytes with a format tag so `prepare` can refuse bytes it
does not understand. Points named in `binding.phases` reach `apply` as playback
evaluated them, so a playback posed through `SetAvar` moves them; frame bytes
can still carry the value the export read, for a phase playback holds no value
at. API4 also passes declared input values, typed and in declaration order,
with their actual availability and element count. Kernels must validate the
input tag and count before reading data, and prefer the current declared
input over encoded frame bytes. Default-time declarations retain their
Default read policy. The runtime applies `inputs:enabled` and the envelope,
and fails the mover for that frame when `apply` returns false or produces a
non-finite point.

A mover whose result follows its influences or its transform provider sets
`runtimeKernel.applyWithProviders` instead of `apply`. Playback then also
hands it `RigExecExternalProviders`: the binding's transform and influence
matrices as playback evaluated them at the binding's phase, and the chain's
base points, the values `assembleExternal` receives as
`RigExecExternalProviderValues`. The built-in `RigExecLayeredSkinMover` and
`RigExecSurfaceBindingMover` play this way: their kernels rebuild the payload
with their own `assembleExternal` logic and deform with `applyExternal`'s, so
playback matches native evaluation bit for bit and a posed playback follows
the providers.

Extending the offset mover above:

```cpp
#include <cstring>

bool Encode(const VtValue &data, const RigExecRevisionBinding &,
            std::vector<uint8_t> *epoch, std::vector<uint8_t> *frame)
{
    if (!data.IsHolding<GfVec3f>()) return false;
    epoch->assign({'O', 'F', 'F', '1'});
    frame->resize(sizeof(GfVec3f));
    std::memcpy(frame->data(), data.UncheckedGet<GfVec3f>().data(),
                frame->size());
    return true;
}

std::shared_ptr<const void> Prepare(const uint8_t *epoch, size_t size,
                                    std::string *error)
{
    if (size != 4 || std::memcmp(epoch, "OFF1", 4) != 0) {
        if (error) *error = "not an OFF1 epoch";
        return nullptr;
    }
    return std::make_shared<int>(1);
}

bool Play(const void *, const uint8_t *frame, size_t frameSize,
          const RigExecExternalPhasedPoints *, size_t,
          const RigExecExternalInputValue *inputs, size_t inputCount,
          float *xyz, size_t count)
{
    float offset[3];
    if (frameSize != sizeof(offset)) return false;
    std::memcpy(offset, frame, sizeof(offset));
    if (inputCount != 1 || inputs[0].type != uint8_t(RigExecExternalInputType::Vec3f) ||
        !inputs[0].hasValue || !inputs[0].data || inputs[0].count != 1) return false;
    std::memcpy(offset, inputs[0].data, sizeof(offset));
    for (size_t i = 0; i < count * 3; ++i) xyz[i] += offset[i % 3];
    return true;
}

// In MakeHandler():
//     handler.encodeExternal = Encode;
//     handler.runtimeKernel.prepare = Prepare;
//     handler.runtimeKernel.apply = Play;
```

Export fails, naming the mover, when its plugin provides no `encodeExternal`
or encodes different epoch bytes on two frames. A frame on which assembly
failed exports as a failed frame, and playback fails the mover there too.

A host that opens a `.rigexec` file installs kernels with
`RigExecRuntimeReader::SetExternalKernel(type, kernel)`, by the type name the
plugin registered. `GetExternalMoverTypes()` lists the types a file holds and
`GetMissingExternalKernels()` the ones still without a prepared kernel. The
usdview playback path installs `runtimeKernel` from every loaded plugin. A mover
whose type has no kernel in the runtime is a no-op: its points pass through
unchanged, and every `Execute` adds a `warning:` line naming the mover to the
reader's diagnostics. A runtime built without USD carries a kernel only if it
links one in; RigExec itself ships none.

## Discovery and supported evaluation

For an independent build, add `mover-build/usd/rigExecMoverPlugins` to
`PXR_PLUGINPATH_NAME`. For an installed plugin, add
`/path/to/mover-install/lib/usd/rigExecMoverPlugins`. Include the plugin and
dependency library directories in the host's library search path, including
`/path/to/mover-install/lib` in Windows `PATH`. Path lists use semicolons on
Windows and colons on Linux/macOS. Keep the host's existing schema and imaging
plugin paths as well.

The generated `plugInfo.json` marks the library with
`"Info": {"RigExecMoverPlugin": 5}`. RigExec loads discoverable mover libraries
on the first unknown handler lookup. Hosts can call
`RigExecLoadMoverPlugins(&diagnostics)` explicitly to inspect version or load
errors. After adding search locations with
`PlugRegistry::GetInstance().RegisterPlugins(path)`, call the loader again to
discover those additional libraries.

API4 external point callbacks execute from compiled immutable bindings and
declared typed inputs in native and frozen jobs. Exported playback also needs
the runtime callbacks. `encodeExternalEpoch` encodes immutable compile facts
without requiring successful initial dynamic input sampling; unavailable inputs
can recover on later frames. Property-mover plugin callbacks are outside this
point-mover API.

A plugin's tests can export and replay its rigs by linking
`rigExec::rigExecBake` and `rigExec::rigExecRuntime`, which are available both
inside the usdRig build and from an installed SDK.

When integrating a mover, check the independent reference and exact golden results, animated and connected
inputs, override updates, envelope weights, disabled behavior, and failure
pass-through. If it exports, check that playback matches the independently captured published pose on
every exported frame and with a posed `SetAvar`. Also check that an unavailable
library or incompatible plugin version produces a diagnostic instead of a
successful deformation.

API4 requires `compileScene` for normalized standalone compilation. It receives
owned composed `RigExecSceneDescriptors`, fills immutable binding facts and the
ordered typed `externalInputs`, and retains no source access. The same
`assembleExternal` and `applyExternal` callbacks run in native, frozen and
detached graphs. Plugins declare any auxiliary source-only schema types in
`sceneDataSchemas`; those records remain data rather than extra operations.

---
title: External mover plugins
summary: Build and register point movers from a separate repository for dynamic, baked and .rigexec playback.
order: 50
---

Movers can live in a separate repository and register with RigExec through a
shared library. The host discovers the library through
OpenUSD's plugin registry. Each registered type supplies a parameter assembly
callback and a points computation; no changes to RigExec's built-in mover list
are required.

Use the same compiler, architecture, build configuration, OpenUSD installation,
and RigExec SDK as the host. Plugin API version 2 identifies the callback
contract; it does not provide binary compatibility across different SDK builds.
Version 2 added the `.rigexec` export and playback callbacks below, so a library
built for version 1 must be rebuilt.

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

bool Assemble(const UsdPrim &prim, const RigExecRevisionBinding &,
              const RigExecProviderValues &values, UsdTimeCode time,
              VtValue *data)
{
    const UsdAttribute attr = prim.GetAttribute(TfToken("inputs:offset"));
    GfVec3f offset;
    const bool read = values.resolved
        ? values.resolved->GetAttribute(attr, time, &offset)
        : attr.Get(&offset, time);
    if (!read || !std::isfinite(offset[0]) ||
        !std::isfinite(offset[1]) || !std::isfinite(offset[2])) {
        return false;
    }
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

`assembleExternal` reads the scene at the requested time and creates an
immutable payload. Read attributes through `values.resolved->GetAttribute`
when available so overrides and connected values follow normal evaluation
semantics. Do not author stage changes during evaluation. Custom payloads must
be copyable and equality-comparable.

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
at. Everything else in the payload replays as the export captured it on that
frame. That is a gap against the USD evaluators: a drag that reaches an input
`assembleExternal` reads moves their result but not a playback's. The runtime applies `inputs:enabled` and the envelope, and fails the mover
for that frame when `apply` returns false or produces a non-finite point.

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
          float *xyz, size_t count)
{
    float offset[3];
    if (frameSize != sizeof(offset)) return false;
    std::memcpy(offset, frame, sizeof(offset));
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
`"Info": {"RigExecMoverPlugin": 2}`. RigExec loads discoverable mover libraries
on the first unknown handler lookup. Hosts can call
`RigExecLoadMoverPlugins(&diagnostics)` explicitly to inspect version or load
errors. After adding search locations with
`PlugRegistry::GetInstance().RegisterPlugins(path)`, call the loader again to
discover those additional libraries.

External point callbacks work in stage-backed dynamic and baked evaluation,
and in `.rigexec` export and playback for plugins that provide the callbacks
above. Frozen evaluation and background frame-cache warming reject rigs
containing external movers; use live evaluation for those rigs. Independent
property-mover callbacks and frozen payload snapshots are not part of this API.

A plugin's tests can export and replay its rigs by linking
`rigExec::rigExecBake` and `rigExec::rigExecRuntime`, which are available both
inside the usdRig build and from an installed SDK.

When integrating a mover, verify dynamic/baked parity, animated and connected
inputs, override updates, envelope weights, disabled behavior, and failure
pass-through. If it exports, check that playback matches the baked program on
every exported frame and with a posed `SetAvar`. Also check that an unavailable
library or incompatible plugin version produces a diagnostic instead of a
successful deformation.

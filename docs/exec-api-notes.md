# OpenExec Plugin-Computation Registration — API Notes (OpenUSD v26.08)

Source studied: `D:\work\usdRig\OpenUSD\pxr\exec\` (exec, execUsd, execGeom, execIr, vdf) and
`extras/exec/examples/definingComputations/`. All signatures below are verbatim from source.

---

## 1. Registration macro, plugInfo.json, plugin discovery

### 1.1 `EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(SchemaType)`
Header: `pxr/exec/exec/registerSchema.h`.

```cpp
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecMySchema)   // token pasted AND stringized
{
    // `self` is an ExecComputationBuilder&
    self.PrimComputation(_tokens->computeThing)
        .Callback<double>(...)
        .Inputs(...);
}
```

Mechanics (exact, from the macro body):
- `SchemaType` is used **both** as an identifier (pasted into the function name
  `Exec_RegisterSchema_##SchemaType`) and stringized (`#SchemaType`) — so it must be the
  **TfType alias/name of the schema class**, e.g. `UsdGeomXformable`,
  `TestExecUsdConstantInputCustomSchema`, `ExecIrFkController`. The string is resolved at
  registry-run time via `TfType::FindByName(schemaTypeName)`
  (`computationBuilders.cpp:428`); unknown name → coding error.
- The macro expands to a `TF_REGISTRY_FUNCTION(ExecDefinitionRegistryTag)` that constructs
  `ExecComputationBuilder self` and calls your block, which becomes the body of
  `static void PXR_NS::exec_registration::Exec_RegisterSchema_<SchemaType>(ExecComputationBuilder &self)`.
- The block lives in namespace `exec_registration`, which is why `AttributeValue`, `Computation`,
  `Relationship`, `Constant`, etc. are usable unqualified inside the block.
- Works for **typed and applied (API) schemas** alike. Only **one macro invocation per schema
  per process** (a second registration for the same schema is an error — see
  `TestExecConflictingPluginRegistration*`). Computations registered for base schema types and
  applied API schemas are all found when resolving a computation on a prim.
- Computation names must NOT start with `__` (reserved prefix for builtins,
  `builtinComputationRegistry.cpp`).

### 1.2 plugInfo.json `Info.Exec.Schemas` block
The library that *defines the computations* (need not be the schema-defining library) must
declare which schemas it registers computations for:

```json
{
    "Plugins": [
        {
            "Info": {
                "Exec": {
                    "Schemas": {
                        "RigExecMySchema":  { "allowsPluginComputations": true },
                        "RigExecOtherAPI":  { }
                    }
                }
            },
            "LibraryPath": "@PLUG_INFO_LIBRARY_PATH@",
            "Name": "rigExec",
            "ResourcePath": "@PLUG_INFO_RESOURCE_PATH@",
            "Root": "@PLUG_INFO_ROOT@",
            "Type": "library"
        }
    ]
}
```

- Keys under `Schemas` are **TfType names** (resolved with `TfType::FindByName`; the schema's
  plugin must already be discoverable so the type exists).
- `allowsPluginComputations` defaults to **true** when omitted. Setting it `false` declares the
  schema *cannot* have plugin computations (registrations for it are errors/ignored).
- Real examples: `pxr/exec/execGeom/plugInfo.json`, `pxr/exec/execIr/plugInfo.json` (the latter
  combines `Exec.Schemas` with the usual usdGenSchema `Types` + `SdfMetadata` blocks in one
  plugin).

### 1.3 Discovery / loading (`pxr/exec/exec/pluginData.cpp`)
- At first use, `Exec_PluginData` iterates `PlugRegistry::GetInstance().GetAllPlugins()` and
  builds a map `schema TfType -> plugin` from every plugin whose metadata has an `Exec` block.
- When exec needs computations for a schema, `LoadPluginComputationsForSchema(schemaType)`
  calls `plugin->Load()` — loading the shared library runs your
  `TF_REGISTRY_FUNCTION(ExecDefinitionRegistryTag)` registrations (the macro) and your
  `TF_REGISTRY_FUNCTION(ExecTypeRegistry)` type registrations.
- So the computation library must be a normal Plug **`"Type": "library"`** plugin (shared lib +
  installed plugInfo.json). The *schema* library itself can be a codeless
  **`"Type": "resource"`** plugin (see
  `execUsd/testenv/testExecUsdAttributeValueInput/resources/plugInfo.json`, which defines the
  schema `Types` only, while the computations are statically linked in the test binary). If your
  computations are linked into the host app instead of a plugin, they register at static-init
  time and no `Exec` plugInfo block is needed — the block exists purely so exec knows *which
  library to load on demand*.
- Standalone tools must make the plugins discoverable, e.g.
  `PlugRegistry::GetInstance().RegisterPlugins(path)` or `PXR_PLUGINPATH_NAME`.

---

## 2. Registering a custom value type (`ExecTypeRegistry`)

Header: `pxr/exec/exec/typeRegistry.h`.

```cpp
template <typename ValueType>
static void ExecTypeRegistry::RegisterType(const ValueType &fallback);   // static!

template <typename ValueType>
TfType ExecTypeRegistry::CheckForRegistration() const;  // fatal error if not registered
```

Correct registry-function tag is exactly `TF_REGISTRY_FUNCTION(ExecTypeRegistry)`:

```cpp
// rigExec/types.h
struct RigExecBezierSegment {
    std::array<GfVec3d, 4> cps;
    uint32_t flags = 0;

    friend bool operator==(const RigExecBezierSegment &a,
                           const RigExecBezierSegment &b) {
        return a.cps == b.cps && a.flags == b.flags;
    }
    friend bool operator!=(const RigExecBezierSegment &a,
                           const RigExecBezierSegment &b) { return !(a == b); }

    // Only needed if the type is ever used with Constant(...) inputs:
    template <typename HashState>
    friend void TfHashAppend(HashState &h, const RigExecBezierSegment &s) {
        h.Append(s.cps, s.flags);
    }
};

// rigExec/types.cpp
#include "pxr/exec/exec/typeRegistry.h"
TF_REGISTRY_FUNCTION(ExecTypeRegistry)
{
    ExecTypeRegistry::RegisterType(RigExecBezierSegment{});   // arg = fallback value
}
```

Requirements / behavior (all verified in source):
- **Equality comparable** — `static_assert(VdfIsEqualityComparable<ValueType>, ...)`.
  (`VdfIsEqualityComparable<T>` in `vdf/traits.h` is `std::equality_comparable` with recursive
  specializations for `std::vector`, `std::pair`, `std::map`, `std::unordered_map`.)
- **Not a VtArray** — `static_assert(!VtIsArray<ValueType>::value, ...)`. VtArray is never an
  exec value type; arrays flow as *vectorized element values* (see §3b).
- **Copyable + a valid fallback value** — the fallback is returned by
  `VdfContext::GetInputValue<T>` when a value is missing (after emitting a coding error), so it
  needs a sane default. Default-constructibility per se is only needed to spell
  `RegisterType(T{})`.
- **TfType registration is automatic**: `VdfExecutionTypeRegistry::Define(fallback)` calls
  `TfType::Define<T>()` if `TfType::Find<T>()` is unknown (`vdf/executionTypeRegistry.h:216`).
  No `TF_REGISTRY_FUNCTION(TfType)` and **no VtValue "traits"/Sdf value-type registration**
  needed — `RegisterType` also installs the VtValue extractor
  (so `ExecUsdCacheView::Get()` returns a `VtValue` holding `T`) and the
  VtValue→VdfVector converter used for `Constant` inputs.
- **Hashable (`VtIsHashable`, i.e. `TfHashAppend` overload) only if used with `Constant(...)`**
  (static_assert lives in the `Constant` accessor, not in `RegisterType`).
- Registering the same type twice is allowed but all calls must pass an equal fallback.
- Registration must be **loaded before use**: put `types.cpp` in the same plugin library as the
  computations (execIr does exactly this: `execIr/types.cpp`).
- The type registry is preloaded with **every Sdf attribute/metadata value type** (double,
  GfVec3d, TfToken, SdfPath, GfMatrix4d, std::string, ...) — do not re-register those.
- Precedent for a container-as-scalar value type: execIr registers
  `using ExecIrResult = TfDenseHashMap<TfToken, VtValue, TfToken::HashFunctor>;` and callbacks
  simply `return ExecIrResult{...};`.

Using it: once registered, the type works anywhere a `ResultType` is accepted —
`.Callback<RigExecBezierSegment>(...)`, `Computation<RigExecBezierSegment>(token)`,
`Relationship(rel).TargetedObjects<RigExecBezierSegment>(token)`,
`ctx.GetInputValue<RigExecBezierSegment>(token)`, `ctx.SetOutput(seg)`. Every DSL specifier
calls `ExecTypeRegistry::GetInstance().CheckForRegistration<T>()` at registration time and
emits a **fatal error** if the type is unknown.

---

## 3. The computation-definition DSL (`computationBuilders.h`)

### 3.1 Computation registrations (methods on `self`)
```cpp
ExecPrimComputationBuilder      PrimComputation(const TfToken &computationName);
ExecAttributeComputationBuilder AttributeComputation(const TfToken &attributeName,
                                                     const TfToken &computationName);
ExecAttributeExpressionBuilder  AttributeExpression(const TfToken &attributeName);
// Dispatched variants (visible cross-prim only via FallsBackToDispatched() inputs):
template <class... Types> ExecPrimComputationBuilder
    DispatchedPrimComputation(const TfToken &name, Types &&...schemaTypes /* TfType... */);
template <class... Types> ExecAttributeComputationBuilder
    DispatchedAttributeComputation(const TfToken &name, Types &&...schemaTypes);
```
`AttributeExpression` overrides the attribute's builtin `computeValue`; to consume the authored
value inside it, take an input `Computation<T>(ExecBuiltinComputations->computeResolvedValue)`.

### 3.2 `.Callback(...)` (on any of the three builders; chainable with `.Inputs`)
```cpp
// (1) Function pointer (incl. non-capturing lambda via unary +):
template <typename ResultType = /*deduced*/, typename ReturnType = /*deduced*/>
Derived& Callback(ReturnType (*callback)(const VdfContext &));
//  - ReturnType == ResultType: deduced, e.g. .Callback(+[](const VdfContext&) -> double {...})
//  - ReturnType convertible to explicit ResultType: .Callback<std::string>(+[]{ return "x"; })
//  - ReturnType void + explicit ResultType: callback MUST call ctx.SetOutput / SetEmptyOutput
//    / write-iterator:  .Callback<double>(+[](const VdfContext &ctx) { ctx.SetOutput(1.0); })

// (2) Function object / capturing lambda: ResultType mandatory, return type MUST be void:
template <typename ResultType, typename FuncType>
Derived& Callback(FuncType &&callback);
```
Constraints (static_asserts): result may not be a reference, may not be a `VtArray`.
Callbacks must be **pure, thread-safe, cache-safe** (all inputs must come through the context).

### 3.3 `.Inputs(...)` — input registrations
`Inputs(Args&&...)` takes any number of registrations; each is
`[accessor(s)] -> value specifier -> [options]`. Default provider (no accessor) is the owning
prim/attribute.

Object accessors (namespace `exec_registration`):
- `Attribute(attrNameToken)` — prim computations: sibling attribute of the prim.
- `Relationship(relNameToken)` — prim computations: relationship on the prim.
- `Prim()` — attribute computations: the owning prim; has `.Attribute(name)`,
  `.Relationship(name)`, `.AttributeValue<T>(name)`.
- `Stage()` — any computation; must be the sole accessor (for
  `ExecBuiltinComputations->computeTime` etc.).

Value specifiers (exactly one per input registration):
- `Computation<ResultType>(computationName)` — computation on the current provider.
- accessor`.Computation<ResultType>(computationName)` — computation on the accessed object.
- accessor`.Metadata<ResultType>(metadataKey)` / `Metadata<T>(key)` — metadata value
  (default input name = key).
- `Relationship(rel).TargetedObjects<ResultType>(computationName)` — see §4c.
- `Attribute(attr).Connections<ResultType>(name)` / `Connections<ResultType>(name)` (attribute
  computations) — computation on objects targeted by the attribute's connections.
- `IncomingConnections<ResultType>(name)` — computation on attributes whose connections target
  the provider (unordered when multiple).
- `NamespaceAncestor<ResultType>(computationName)` — nearest namespace ancestor prim that
  provides the computation.
- `Constant(value).InputName(nameToken)` — bakes a constant (type must be registered AND
  hashable; string literals become `std::string` via deduction guide; `.InputName` mandatory).

Aliases:
- `AttributeValue<T>(attrToken)` ≡
  `Attribute(attrToken).Computation<T>(ExecBuiltinComputations->computeValue).InputName(attrToken)`
- `Prim().AttributeValue<T>(attrToken)` — same, from an attribute computation.

Input options (chainable on a value specifier, return `This&`):
- `.InputName(TfToken)` — overrides default input name (default = computation name, or
  attribute name for `AttributeValue`, or metadata key for `Metadata`).
- `.Required()` — inputs are **optional by default**; `Required()` makes compilation emit an
  error if the input cannot be compiled. There is **no public `.Optional()`** (optional is the
  default; `_SetOptional` is protected) and **no `.Fallback(value)` option** — do fallbacks in
  the callback via `GetInputValuePtr` (see §4d).
- `.FallsBackToDispatched()` — allows the input to resolve to a Dispatched*Computation when no
  local computation of that name exists on the provider.
- Disambiguating ids exist in the specifier base-class ctor
  (`Exec_ComputationBuilderComputationValueSpecifier(name, type, resolution, disambiguatingId = TfToken())`)
  but are **not exposed as a chainable DSL option in v26.08**; two inputs may share the same
  input name and their values simply appear on the same-named input (read them vectorized —
  see `testExecUsdRecompilation.cpp` `computeUsingDuplicateInputNames`).

---

## 4. Reading inputs in callbacks (`VdfContext`, iterators)

`pxr/exec/vdf/context.h` — key members:

```cpp
template <typename T> VdfByValueOrConstRef<T> GetInputValue(const TfToken &name) const;
    // Expects exactly one value; if none: TF_CODING_ERROR + returns registered fallback.
template <typename T> const T *GetInputValuePtr(const TfToken &name) const;          // nullptr if absent
template <typename T> const T *GetInputValuePtr(const TfToken &name, const T *defPtr) const;
bool HasInputValue(const TfToken &name) const;
bool IsOutputRequested(const TfToken &outputName) const;

template <typename T> void SetOutput(const T &value) const;          // sole output
template <typename T> void SetOutput(T &&value) const;               // move
template <typename T> void SetOutput(const TfToken &outputName, const T &value) const;
void SetEmptyOutput() const;                                          // + (outputName) overload
void SetOutputToReferenceInput(const TfToken &inputName) const;       // pass-through optimization
void Warn(const char *fmt, ...) const;  void CodingError(const char *fmt, ...) const;
```

`pxr/exec/vdf/readIterator.h` / `readIteratorRange.h`:
```cpp
template <typename T> class VdfReadIterator {
    VdfReadIterator(const VdfContext &context, const TfToken &inputName);
    VdfReadIterator &operator++();
    reference operator*() const;      // const T&
    bool IsAtEnd() const;
    size_t ComputeSize() const;       // total elements across all connections
    void AdvanceToEnd();
};
template <typename T> class VdfReadIteratorRange;  // VdfReadIteratorRange<T> range(ctx, name);
                                                   // begin()/end() for range-for / std::accumulate
```

`pxr/exec/vdf/readWriteIterator.h` (vectorized/boxed output):
```cpp
template <typename T> class VdfReadWriteIterator {
    VdfReadWriteIterator(const VdfContext &context, const TfToken &name); // in-place r/w
    explicit VdfReadWriteIterator(const VdfContext &context);             // sole output
    static VdfReadWriteIterator Allocate(const VdfContext &context,
                                         const TfToken &name, size_t count);
    static VdfReadWriteIterator Allocate(const VdfContext &context, size_t count); // sole output
    // elements default-initialized; iterate and assign *it
};
```

### (a) Scalar attribute value
```cpp
self.PrimComputation(_tokens->computeDoubleAttr)
    .Callback<double>(+[](const VdfContext &ctx) {
        return ctx.GetInputValue<double>(_tokens->doubleAttr);
    })
    .Inputs(AttributeValue<double>(_tokens->doubleAttr));   // add .Required() to hard-require
```
Input name for `AttributeValue` = the attribute token itself.

### (b) Array-valued attribute (e.g. `point3d[4]`)
Authored `VtArray` attribute values enter exec as **vectorized element values** — you register
`AttributeValue<GfVec3d>` (the *element* type; `AttributeValue<VtVec3dArray>` will not work,
VtArray is rejected) and read the elements with `VdfReadIterator`. Verbatim from
`execUsd/testenv/testExecUsdAttributeValueInput.cpp` (attribute `Vec3f[] arrayAttr`):

```cpp
self.PrimComputation(_tokens->computeArrayAttr)
    .Callback<std::vector<GfVec3f>>(+[](const VdfContext &ctx) {
        VdfReadIterator<GfVec3f> rIt(ctx, _tokens->arrayAttr);
        std::vector<GfVec3f> result;
        result.reserve(rIt.ComputeSize());
        for (size_t i = 0; !rIt.IsAtEnd(); ++rIt, ++i) { result.push_back(*rIt); }
        return result;
    })
    .Inputs(AttributeValue<GfVec3f>(_tokens->arrayAttr));
```
(The test registers `ExecTypeRegistry::RegisterType(std::vector<GfVec3f>{})` to use
`std::vector<GfVec3f>` as the *result* type — VtArray can't be a result type either. For a
`point3d[4]` attribute you'd read 4 `GfVec3d` elements and pack them into your registered
struct, e.g. `RigExecBezierSegment` — this is exactly the intended pattern.)
`GetInputValue<GfVec3d>` on an array input would return only the first element; use the
iterator. `RegisterType` note: extraction of a *VtArray-typed attribute's computed value* back
to `VtValue` (cache view) yields `VtArray<Element>` automatically via the built-in extractors.

### (c) Another prim's computation via relationship targets
Provider resolution `DynamicTraversal::RelationshipTargetedObjects`
(`providerResolution.h`): all relationship targets (with relationship forwarding transitively
expanded to non-relationship objects) become providers. **Multiple targets → multiple values
delivered on the single named input, in target order — read them vectorized with
`VdfReadIterator<T>`.** `GetInputValue`/`GetInputValuePtr` return just the first value.
Verbatim from `testExecUsdRecompilation.cpp`:

```cpp
self.PrimComputation(_tokens->computeUsingCustomRel)
    .Callback(+[](const VdfContext &context) {
        int result = 0;
        VdfReadIterator<int> it(context, ExecBuiltinComputations->computeValue);
        for (; !it.IsAtEnd(); ++it) { result += *it; }
        return result;
    })
    .Inputs(
        Relationship(_tokens->customRel)
        .TargetedObjects<int>(ExecBuiltinComputations->computeValue));
```
With a custom struct result on the targeted prims (RigExec case):
```cpp
.Inputs(Relationship(_tokens->sources)
            .TargetedObjects<RigExecBezierSegment>(_tokens->computeSegment)
            .InputName(_tokens->segments))       // optional rename
// callback: VdfReadIterator<RigExecBezierSegment> it(ctx, _tokens->segments);
```
Targets that don't provide the computation contribute no value (input is optional by default);
`.Required()` errors if nothing can be compiled. `.FallsBackToDispatched()` lets targets match
a `DispatchedPrimComputation` you registered for your own schema.

### (d) Optional inputs and fallbacks
- All inputs are optional unless `.Required()`.
- Preferred patterns (from execGeom/execUsd tests):
```cpp
const GfMatrix4d *const parentToWorld =
    ctx.GetInputValuePtr<GfMatrix4d>(ExecGeomXformableTokens->computeLocalToWorldTransform);
GfMatrix4d out = parentToWorld ? *parentToWorld : GfMatrix4d(1.0);

static const std::string empty;
const std::string *v = ctx.GetInputValuePtr<std::string>(_tokens->attr, &empty); // default ptr
if (ctx.HasInputValue(_tokens->x)) { ... }
```
- Never call `GetInputValue<T>` on a possibly-missing input: it coding-errors and returns the
  type's registered fallback.

---

## 5. Builtin computations — exact tokens

Header: `pxr/exec/exec/builtinComputations.h`; access via the static data pointer
`ExecBuiltinComputations` (type `TfStaticData<Exec_BuiltinComputationTokens>`):

| C++ spelling | Token string | Provider | Result type |
|---|---|---|---|
| `ExecBuiltinComputations->computeValue` | `"__computeValue"` | attribute | attribute's scalar value type (or expression's type); source order: attribute expression → single same-typed connection's computed value → resolved value |
| `ExecBuiltinComputations->computeResolvedValue` | `"__computeResolvedValue"` | attribute | attribute's scalar value type; always the authored/resolved value (bypasses expressions) |
| `ExecBuiltinComputations->computePath` | `"__computePath"` | any object | `SdfPath` |
| `ExecBuiltinComputations->computeTime` | `"__computeTime"` | stage (`Stage()` accessor only) | `EfTime` |

- All builtin names are prefixed `__` internally (`builtinComputationRegistry.cpp`); `__` is a
  reserved prefix for plugin computation names.
- There is **no public `computeConnectedValue` token**: "connected value" is an internal
  definition (`Exec_ComputeConnectedValueComputationDefinition`,
  `builtinAttributeComputations.h`) used by `computeValue` when the attribute has exactly one
  valid same-typed connection. It does not support multiple connections.
- `builtinStageComputations.h`/`builtinObjectComputations.h` implement `computeTime` /
  `computePath` providers.

---

## 6. Real registrations in the repo (models to copy)

### execGeom (`pxr/exec/execGeom/xformable.cpp`) — scalar callback, optional inputs, recursion up namespace
```cpp
static GfMatrix4d _ComputeLocalToWorldTransform(const VdfContext &ctx) {
    const GfMatrix4d *const localToParent =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->xformOpTransform);
    const GfMatrix4d *const parentToWorld =
        ctx.GetInputValuePtr<GfMatrix4d>(
            ExecGeomXformableTokens->computeLocalToWorldTransform);
    if (parentToWorld) {
        return localToParent ? (*localToParent) * (*parentToWorld) : (*parentToWorld);
    }
    return localToParent ? (*localToParent) : GfMatrix4d(1.0);
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(UsdGeomXformable)
{
    self.PrimComputation(ExecGeomXformableTokens->computeLocalToWorldTransform)
        .Callback<GfMatrix4d>(&_ComputeLocalToWorldTransform)
        .Inputs(
            AttributeValue<GfMatrix4d>(_tokens->xformOpTransform),
            NamespaceAncestor<GfMatrix4d>(
                ExecGeomXformableTokens->computeLocalToWorldTransform));
}
```
Note the self-referential `NamespaceAncestor` input — this is how per-prim computations chain
up a hierarchy (each prim's node feeds its children's nodes; exec caches per prim).

### execIr (`fkControllerComputations.cpp` + `types.cpp`) — custom value type as result
```cpp
using ExecIrResult = TfDenseHashMap<TfToken, VtValue, TfToken::HashFunctor>;   // types.h
TF_REGISTRY_FUNCTION(ExecTypeRegistry) { ExecTypeRegistry::RegisterType(ExecIrResult{}); }

static ExecIrResult _Compute(const VdfContext &ctx) {
    const GfMatrix4d outSpaceValue = ExecIr_UtilsCompute(
        ExecIr_ComputeFkParams(ctx),
        ExecIr_UtilsComputeLocalTranslation(ctx),
        ExecIr_UtilsComputeLocalRotation(ctx));
    return ExecIrResult({{ExecIrTokens->outSpace, VtValue(outSpaceValue)}});
}
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(ExecIrFkController)
{
    ExecIrControllerBuilder builder(self, &_Compute, &_Invert);  // helper wrapping the DSL
    builder.InvertibleInputAttribute<double>(ExecIrTokens->inTx);
    ...
}
```
(execIr also demonstrates wrapping the builder in your own helper class — `self` can be passed
around by reference and the sub-builders returned by `PrimComputation` can be stored/composed.)

### Vectorized output (from vdf tests, e.g. `testVdfReadWriteIterator.cpp`)
A callback whose output should hold N elements (rather than one struct):
```cpp
.Callback<GfVec3d>(+[](const VdfContext &ctx) {   // ResultType = element type
    VdfReadWriteIterator<GfVec3d> out =
        VdfReadWriteIterator<GfVec3d>::Allocate(ctx, 4);   // sole output, 4 elements
    for (int i = 0; !out.IsAtEnd(); ++out, ++i) { *out = GfVec3d(i); }
})
```
For RigExec's fixed 4-point data, prefer the registered struct (`RigExecBezierSegment`) as a
single scalar value — simpler extraction and equality-based invalidation.

---

## 7. CMake / build & install

From `execGeom/CMakeLists.txt` and `execIr/CMakeLists.txt`:

```cmake
set(PXR_PREFIX pxr/exec)          # or your own prefix
set(PXR_PACKAGE rigExec)

pxr_library(rigExec
    LIBRARIES            # public+link deps; execGeom uses: gf tf execUsd usdGeom
        gf
        tf
        usd              # if you have generated schema classes
        execUsd          # pulls in exec, esf, esfUsd, vdf, ef transitively
    INCLUDE_SCHEMA_FILES # only if this lib contains usdGenSchema-generated classes (execIr)
    PUBLIC_HEADERS   api.h
    PUBLIC_CLASSES   tokens types      # types.cpp holds TF_REGISTRY_FUNCTION(ExecTypeRegistry)
    CPPFILES         myComputations.cpp   # holds EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA
    RESOURCE_FILES   plugInfo.json
)
```

- **Minimum link set for a computation-only plugin**: `tf` + `exec` (that's all
  `TestExecPluginComputation` links: `pxr_build_test_shared_lib(TestExecPluginComputation
  INSTALL_PREFIX ExecPlugins LIBRARIES tf exec CPPFILES ...)`). Add `gf`/`vt`/`sdf` for value
  types, `usd*` for schema classes, `execUsd` only if you also *evaluate* (ExecUsdSystem /
  BuildRequest / CacheView) from the same lib. `vdf` comes transitively via `exec` (contexts
  and iterators are headers in `pxr/exec/vdf/...`).
- `plugInfo.json` is listed under `RESOURCE_FILES`; `pxr_library`/`_install_resource_files`
  substitutes `@PLUG_INFO_LIBRARY_PATH@`, `@PLUG_INFO_RESOURCE_PATH@`, `@PLUG_INFO_ROOT@` and
  installs it to `<prefix>/lib/usd/rigExec/resources/plugInfo.json` (picked up via the
  top-level `lib/usd/plugInfo.json` include mechanism / `PXR_PLUGINPATH_NAME`).
- Tests: `pxr_build_test(name LIBRARIES tf exec execUsd plug sdf vdf usd CPPFILES ...)` +
  `pxr_register_test(... TESTENV ...)`; codeless schema plugins for tests live in
  `testenv/<test>/resources/plugInfo.json` (`"Type": "resource"`, `"ResourcePath": "."`) and
  are loaded at runtime with `PlugRegistry::GetInstance().RegisterPlugins(TfAbsPath("resources"))`.
- execIr note for Windows/MSVC + generated schemas: it unsets `CMAKE_CXX_VISIBILITY_PRESET`
  before `pxr_library` (generated schema code and hidden visibility issue).
- Test plugins require `BUILD_SHARED_LIBS` (static builds don't support plugin loading of test
  libs — guarded in execGeom's CMakeLists).

---

## 8. Gotchas checklist

1. Macro argument must be the exact TfType name and a valid C++ identifier; the macro is used
   at file scope (it opens/closes `PXR_NAMESPACE` itself). One invocation per schema, ever.
2. `Callback<T>` with capturing lambda must return `void` and call `ctx.SetOutput(...)`;
   only function pointers (`+[]`) may return the value directly.
3. `VtArray<T>` is banned as result and input value type — use element-vectorized flow
   (`VdfReadIterator<Element>`) or register a `std::vector<T>`/custom struct.
4. Inputs are **optional by default**; misspelled input names silently produce "no value" — the
   default input name is the *computation* name except for `AttributeValue`/`Metadata` aliases.
5. `GetInputValue` on an absent input = coding error + fallback; use `GetInputValuePtr`.
6. Multiple values on one input (multiple rel targets, duplicate input names, array elements)
   require `VdfReadIterator`; `GetInputValue*` sees only the first.
7. `Constant(...)` values must be hashable (`TfHashAppend`) and `.InputName(...)` is mandatory.
8. Register custom value types in the same plugin library (`TF_REGISTRY_FUNCTION(ExecTypeRegistry)`);
   they auto-`TfType::Define` — do not also hand-register the TfType with a different name.
9. `Stage()` accessor must be the only accessor in its input registration.
10. Computation names starting with `__` are reserved for builtins.
11. Callbacks must be pure/thread-safe; no reading stage data directly inside a callback — all
    scene dependencies must be declared as inputs or invalidation breaks ("cache safety").

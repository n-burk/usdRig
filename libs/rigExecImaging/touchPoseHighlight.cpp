//
// TouchPose highlight -- see touchPoseHighlight.h.
//
#include "touchPoseHighlight.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/type.h"
#include "pxr/imaging/hd/containerDataSourceEditor.h"
#include "pxr/imaging/hd/dataSourceMaterialNetworkInterface.h"
#include "pxr/imaging/hd/materialBindingSchema.h"
#include "pxr/imaging/hd/materialBindingsSchema.h"
#include "pxr/imaging/hd/materialSchema.h"
#include "pxr/imaging/hd/overlayContainerDataSource.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hio/glslfx.h"
#include "pxr/usd/sdr/registry.h"
#include "pxr/usd/sdr/shaderNode.h"
#include "pxr/usd/sdr/shaderProperty.h"
#include "pxr/usdImaging/usdImaging/sceneIndexPlugin.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <locale>
#include <sstream>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    (rigExecTouchRegion)
    (rigExecTouchTable)
    (glslfx)
    (opacityMode)
    (transparent)
    (presence)
    (primvars)
);

}  // namespace

const TfToken &
RigExecTouchPoseTokens::RegionPrimvar()
{
    return _tokens->rigExecTouchRegion;
}

const TfToken &
RigExecTouchPoseTokens::TablePrimvar()
{
    return _tokens->rigExecTouchTable;
}

// ===========================================================================
// the generated shader
// ===========================================================================

namespace {

std::string
_JsonNumber(double value, bool integral)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    if (integral) {
        out << static_cast<long long>(value);
        return out.str();
    }
    out << std::setprecision(9) << value;
    std::string text = out.str();
    // JSON parses "1" as an int and Sdr turns an int into an Int property,
    // which Storm would then declare as `int` -- a float input has to keep
    // its decimal point.
    if (text.find_first_of(".eE") == std::string::npos &&
        text.find("inf") == std::string::npos &&
        text.find("nan") == std::string::npos) {
        text += ".0";
    }
    return text;
}

std::string
_JsonArray(const double *values, size_t count)
{
    std::string out = "[";
    for (size_t i = 0; i < count; ++i) {
        out += (i ? ", " : "") + _JsonNumber(values[i], false);
    }
    return out + "]";
}

/// A glslfx "default" for one Sdr input, or empty when the type has no glslfx
/// spelling (asset paths, strings other than the known token inputs).
std::string
_JsonDefault(const TfToken &name, const VtValue &value)
{
    if (value.IsHolding<float>()) {
        return _JsonNumber(value.UncheckedGet<float>(), false);
    }
    if (value.IsHolding<double>()) {
        return _JsonNumber(value.UncheckedGet<double>(), false);
    }
    if (value.IsHolding<int>()) {
        return _JsonNumber(value.UncheckedGet<int>(), true);
    }
    if (value.IsHolding<bool>()) {
        return _JsonNumber(value.UncheckedGet<bool>() ? 1 : 0, true);
    }
    if (value.IsHolding<GfVec2f>()) {
        const GfVec2f v = value.UncheckedGet<GfVec2f>();
        const double d[2] = {v[0], v[1]};
        return _JsonArray(d, 2);
    }
    if (value.IsHolding<GfVec3f>()) {
        const GfVec3f v = value.UncheckedGet<GfVec3f>();
        const double d[3] = {v[0], v[1], v[2]};
        return _JsonArray(d, 3);
    }
    if (value.IsHolding<GfVec3d>()) {
        const GfVec3d v = value.UncheckedGet<GfVec3d>();
        const double d[3] = {v[0], v[1], v[2]};
        return _JsonArray(d, 3);
    }
    if (value.IsHolding<GfVec4f>()) {
        const GfVec4f v = value.UncheckedGet<GfVec4f>();
        const double d[4] = {v[0], v[1], v[2], v[3]};
        return _JsonArray(d, 4);
    }
    if (value.IsHolding<VtFloatArray>()) {
        const VtFloatArray &a = value.UncheckedGet<VtFloatArray>();
        std::vector<double> d(a.begin(), a.end());
        return d.empty() ? std::string() : _JsonArray(d.data(), d.size());
    }
    // UsdPreviewSurface's opacityMode is a token input, and Storm only turns
    // it into the int its shader reads for a node literally identified as
    // UsdPreviewSurface (hdSt/materialNetwork.cpp). The wrapper is not, so
    // the conversion is made here for the default and by the scene index
    // for an authored value.
    if (name == _tokens->opacityMode) {
        std::string token;
        if (value.IsHolding<TfToken>()) {
            token = value.UncheckedGet<TfToken>().GetString();
        } else if (value.IsHolding<std::string>()) {
            token = value.UncheckedGet<std::string>();
        }
        return _JsonNumber(token == _tokens->presence.GetString() ? 0 : 1,
                           true);
    }
    return std::string();
}

// The wrapper's own code. `rigExecTouchPose_BaseSurfaceShader` is the
// original terminal, renamed by a #define around its source so its body is
// compiled byte for byte as it was.
//
// The tint is mixed over the LIT colour, which is what the old overlay
// patch did with its alpha, and it is shaded by a headlight term so a lit
// region still reads as a surface with a shape rather than a flat decal.
const char *const _kWrapSource = R"GLSL(
#undef surfaceShader

vec4
surfaceShader(vec4 Peye, vec3 Neye, vec4 color, vec4 patchCoord)
{
    vec4 shaded = rigExecTouchPose_BaseSurfaceShader(
        Peye, Neye, color, patchCoord);
#if defined(HD_HAS_rigExecTouchRegion) && defined(HD_HAS_rigExecTouchTable)
    int slot = int(HdGet_rigExecTouchRegion() + 0.5);
    if (slot > 0) {
        vec4 tint = HdGet_rigExecTouchTable(slot);
        if (tint.a > 0.0) {
            vec3 n = normalize(Neye);
            vec3 v = normalize(-Peye.xyz);
            float facing = clamp(abs(dot(n, v)), 0.0, 1.0);
            vec3 lit = tint.rgb * (0.45 + 0.55 * facing);
            shaded.rgb = mix(shaded.rgb, lit, clamp(tint.a, 0.0, 1.0));
        }
    }
#endif
    return shaded;
}
)GLSL";

struct _Wrapper {
    TfToken identifier;
    std::string source;
};

std::mutex &
_WrapperMutex()
{
    static std::mutex mutex;
    return mutex;
}

std::map<TfToken, _Wrapper> &
_Wrappers()
{
    static std::map<TfToken, _Wrapper> wrappers;
    return wrappers;
}

_Wrapper
_BuildWrapper(const TfToken &base)
{
    _Wrapper result;
    SdrRegistry &registry = SdrRegistry::GetInstance();
    const SdrShaderNodeConstPtr node =
        registry.GetShaderNodeByIdentifierAndType(base, _tokens->glslfx);
    if (!node) {
        return result;
    }
    const std::string &uri = node->GetResolvedImplementationURI();
    if (uri.empty()) {
        // A terminal that is itself source code has no file to re-read.
        return result;
    }
    HioGlslfx glslfx(uri);
    std::string reason;
    if (!glslfx.IsValid(&reason)) {
        TF_WARN("TouchPose: cannot wrap %s (%s): %s", base.GetText(),
                uri.c_str(), reason.c_str());
        return result;
    }
    const std::string surface = glslfx.GetSurfaceSource();
    if (surface.find("surfaceShader") == std::string::npos) {
        return result;
    }
    const std::string displacement = glslfx.GetDisplacementSource();

    std::string parameters;
    for (const TfToken &input : node->GetShaderInputNames()) {
        const SdrShaderPropertyConstPtr property = node->GetShaderInput(input);
        if (!property) {
            continue;
        }
        const std::string value =
            _JsonDefault(input, property->GetDefaultValue());
        if (value.empty()) {
            continue;
        }
        parameters += std::string(parameters.empty() ? "" : ",\n") +
                      "        \"" + input.GetString() +
                      "\": { \"default\": " + value + " }";
    }

    std::string metadata;
    for (const auto &entry : glslfx.GetMetadata()) {
        if (entry.second.IsHolding<std::string>()) {
            metadata += std::string(metadata.empty() ? "" : ",\n") +
                        "        \"" + entry.first + "\": \"" +
                        entry.second.UncheckedGet<std::string>() + "\"";
        }
    }

    std::ostringstream code;
    code << "-- glslfx version 0.1\n\n"
         << "--- Generated by RigExec TouchPose: " << base.GetString()
         << " (" << uri << ") with a region tint mixed over its result.\n\n"
         << "-- configuration\n{\n";
    if (!metadata.empty()) {
        code << "    \"metadata\": {\n" << metadata << "\n    },\n";
    }
    code << "    \"parameters\": {\n" << parameters << "\n    },\n"
         << "    \"techniques\": {\n        \"default\": {\n";
    if (!displacement.empty()) {
        code << "            \"displacementShader\": { \"source\": "
                "[ \"RigExecTouchPose.Displacement\" ] },\n";
    }
    code << "            \"surfaceShader\": { \"source\": "
            "[ \"RigExecTouchPose.Base\", \"RigExecTouchPose.Wrap\" ] }\n"
         << "        }\n    }\n}\n\n";
    if (!displacement.empty()) {
        code << "-- glsl RigExecTouchPose.Displacement\n" << displacement
             << "\n";
    }
    code << "-- glsl RigExecTouchPose.Base\n"
         << "#define surfaceShader rigExecTouchPose_BaseSurfaceShader\n"
         << surface << "\n"
         << "-- glsl RigExecTouchPose.Wrap\n" << _kWrapSource;
    const std::string source = code.str();

    // The primvars go in as node METADATA: Storm keeps a mesh's primvar only
    // if some material asks for it by name (primvar filtering), and this is
    // how a glslfx node asks.
    SdrTokenMap sdrMetadata;
    sdrMetadata[SdrNodeMetadata->Primvars] =
        _tokens->rigExecTouchRegion.GetString() + "|" +
        _tokens->rigExecTouchTable.GetString();
    const SdrShaderNodeConstPtr wrapped = registry.GetShaderNodeFromSourceCode(
        source, _tokens->glslfx, sdrMetadata);
    if (!wrapped) {
        TF_WARN("TouchPose: Sdr rejected the wrapper for %s", base.GetText());
        return result;
    }
    result.identifier = wrapped->GetIdentifier();
    result.source = source;
    return result;
}

const _Wrapper &
_GetWrapper(const TfToken &base)
{
    std::lock_guard<std::mutex> lock(_WrapperMutex());
    auto it = _Wrappers().find(base);
    if (it == _Wrappers().end()) {
        it = _Wrappers().emplace(base, _BuildWrapper(base)).first;
    }
    return it->second;
}

bool
_IsWrapperIdentifier(const TfToken &identifier)
{
    std::lock_guard<std::mutex> lock(_WrapperMutex());
    for (const auto &entry : _Wrappers()) {
        if (entry.second.identifier == identifier) {
            return true;
        }
    }
    return false;
}

}  // namespace

TfToken
RigExecTouchPoseGetWrappedTerminal(const TfToken &base)
{
    return _GetWrapper(base).identifier;
}

std::string
RigExecTouchPoseGetWrappedSource(const TfToken &base)
{
    return _GetWrapper(base).source;
}

// ===========================================================================
// the shared state
// ===========================================================================

RigExecTouchPoseHighlights &
RigExecTouchPoseHighlights::GetInstance()
{
    static RigExecTouchPoseHighlights instance;
    return instance;
}

void
RigExecTouchPoseHighlights::SetMesh(
    const SdfPath &path, const VtFloatArray &faceSlots,
    const VtVec4fArray &table)
{
    // The wrapper for the terminal everyone uses is built HERE, on the
    // caller's thread, so the first material GetPrim a render thread makes
    // finds it already parsed.
    RigExecTouchPoseGetWrappedTerminal(TfToken("UsdPreviewSurface"));

    auto mesh = std::make_shared<RigExecTouchPoseHighlightMesh>();
    mesh->path = path;
    mesh->faceSlots = faceSlots;
    mesh->table = table;
    bool attach = true;
    bool slotsMoved = false;
    bool tableMoved = false;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _meshes.find(path);
        if (it != _meshes.end() && it->second->table.size() == table.size() &&
            it->second->faceSlots.size() == faceSlots.size()) {
            attach = false;
            slotsMoved = it->second->faceSlots != faceSlots;
            tableMoved = it->second->table != table;
            if (!slotsMoved && !tableMoved) {
                return;
            }
        }
        _meshes[path] = mesh;
        _count = _meshes.size();
    }
    if (attach) {
        _Notify(path, Change::Attached);
        return;
    }
    if (slotsMoved) {
        _Notify(path, Change::Slots);
    }
    if (tableMoved) {
        ++_tableUpdates;
        _Notify(path, Change::Table);
    }
}

bool
RigExecTouchPoseHighlights::SetTable(
    const SdfPath &path, const VtVec4fArray &table)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _meshes.find(path);
        if (it == _meshes.end() || it->second->table.size() != table.size()) {
            return false;
        }
        if (it->second->table == table) {
            return true;
        }
        auto mesh = std::make_shared<RigExecTouchPoseHighlightMesh>(*it->second);
        mesh->table = table;
        it->second = mesh;
    }
    ++_tableUpdates;
    _Notify(path, Change::Table);
    return true;
}

bool
RigExecTouchPoseHighlights::SetFaceSlots(
    const SdfPath &path, const VtFloatArray &faceSlots)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _meshes.find(path);
        if (it == _meshes.end() ||
            it->second->faceSlots.size() != faceSlots.size()) {
            return false;
        }
        if (it->second->faceSlots == faceSlots) {
            return true;
        }
        auto mesh = std::make_shared<RigExecTouchPoseHighlightMesh>(*it->second);
        mesh->faceSlots = faceSlots;
        it->second = mesh;
    }
    _Notify(path, Change::Slots);
    return true;
}

void
RigExecTouchPoseHighlights::RemoveMesh(const SdfPath &path)
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (!_meshes.erase(path)) {
            return;
        }
        _count = _meshes.size();
    }
    _Notify(path, Change::Detached);
}

RigExecTouchPoseHighlightMeshConstPtr
RigExecTouchPoseHighlights::Find(const SdfPath &path) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _meshes.find(path);
    return it == _meshes.end() ? nullptr : it->second;
}

std::vector<SdfPath>
RigExecTouchPoseHighlights::GetMeshPaths() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::vector<SdfPath> paths;
    for (const auto &entry : _meshes) {
        paths.push_back(entry.first);
    }
    return paths;
}

void
RigExecTouchPoseHighlights::RegisterSceneIndex(
    const RigExecTouchPoseSceneIndexRefPtr &index)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _indices.erase(
        std::remove_if(_indices.begin(), _indices.end(),
                       [](const auto &weak) { return !weak; }),
        _indices.end());
    _indices.emplace_back(index);
}

size_t
RigExecTouchPoseHighlights::GetSceneIndexCount()
{
    std::lock_guard<std::mutex> lock(_mutex);
    size_t live = 0;
    for (const auto &weak : _indices) {
        live += weak ? 1 : 0;
    }
    return live;
}

void
RigExecTouchPoseHighlights::_Notify(const SdfPath &path, Change change)
{
    std::vector<RigExecTouchPoseSceneIndexRefPtr> live;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (const auto &weak : _indices) {
            if (weak) {
                live.emplace_back(weak);
            }
        }
    }
    for (const auto &index : live) {
        index->HighlightChanged(path, change);
    }
}

// ===========================================================================
// the scene index
// ===========================================================================

namespace {

HdContainerDataSourceHandle
_PrimvarsOverlay(const RigExecTouchPoseHighlightMesh &mesh)
{
    static const HdTokenDataSourceHandle uniform =
        HdPrimvarSchema::BuildInterpolationDataSource(
            HdPrimvarSchemaTokens->uniform);
    static const HdTokenDataSourceHandle constant =
        HdPrimvarSchema::BuildInterpolationDataSource(
            HdPrimvarSchemaTokens->constant);

    const HdDataSourceBaseHandle region =
        HdPrimvarSchema::Builder()
            .SetPrimvarValue(
                HdRetainedTypedSampledDataSource<VtFloatArray>::New(
                    mesh.faceSlots))
            .SetInterpolation(uniform)
            .Build();
    const HdDataSourceBaseHandle table =
        HdPrimvarSchema::Builder()
            .SetPrimvarValue(
                HdRetainedTypedSampledDataSource<VtVec4fArray>::New(
                    mesh.table))
            .SetInterpolation(constant)
            .Build();
    return HdRetainedContainerDataSource::New(
        HdPrimvarsSchema::GetSchemaToken(),
        HdRetainedContainerDataSource::New(
            _tokens->rigExecTouchRegion, region,
            _tokens->rigExecTouchTable, table));
}

/// One material network with its surface terminal wrapped, or null when
/// there is nothing to wrap.
HdContainerDataSourceHandle
_WrapNetwork(const SdfPath &materialPath,
             const HdContainerDataSourceHandle &network,
             const HdContainerDataSourceHandle &primDataSource)
{
    HdDataSourceMaterialNetworkInterface interface(
        materialPath, network, primDataSource);
    const auto terminal =
        interface.GetTerminalConnection(HdMaterialTerminalTokens->surface);
    if (!terminal.first) {
        return nullptr;
    }
    const TfToken node = terminal.second.upstreamNodeName;
    const TfToken base = interface.GetNodeType(node);
    if (base.IsEmpty() || _IsWrapperIdentifier(base)) {
        return nullptr;
    }
    const TfToken wrapped = RigExecTouchPoseGetWrappedTerminal(base);
    if (wrapped.IsEmpty()) {
        return nullptr;
    }
    interface.SetNodeType(node, wrapped);
    // See _JsonDefault: an AUTHORED opacityMode token has to become the int
    // the shader reads, because Storm only converts it for a node named
    // UsdPreviewSurface.
    const VtValue mode =
        interface.GetNodeParameterValue(node, _tokens->opacityMode);
    if (!mode.IsEmpty() && !mode.IsHolding<int>()) {
        std::string token;
        if (mode.IsHolding<TfToken>()) {
            token = mode.UncheckedGet<TfToken>().GetString();
        } else if (mode.IsHolding<std::string>()) {
            token = mode.UncheckedGet<std::string>();
        }
        interface.SetNodeParameterValue(
            node, _tokens->opacityMode,
            VtValue(token == _tokens->presence.GetString() ? 0 : 1));
    }
    return interface.Finish();
}

HdContainerDataSourceHandle
_WrapMaterial(const SdfPath &materialPath,
              const HdContainerDataSourceHandle &primDataSource)
{
    const HdMaterialSchema schema =
        HdMaterialSchema::GetFromParent(primDataSource);
    const HdContainerDataSourceHandle material = schema.GetContainer();
    if (!material) {
        return primDataSource;
    }
    const TfTokenVector contexts = material->GetNames();
    std::vector<TfToken> names;
    std::vector<HdDataSourceBaseHandle> values;
    bool changed = false;
    for (const TfToken &context : contexts) {
        HdDataSourceBaseHandle value = material->Get(context);
        if (const HdContainerDataSourceHandle network =
                HdContainerDataSource::Cast(value)) {
            if (HdContainerDataSourceHandle wrapped =
                    _WrapNetwork(materialPath, network, primDataSource)) {
                value = wrapped;
                changed = true;
            }
        }
        names.push_back(context);
        values.push_back(value);
    }
    if (!changed) {
        return primDataSource;
    }
    return HdContainerDataSourceEditor(primDataSource)
        .Set(HdMaterialSchema::GetDefaultLocator(),
             HdRetainedContainerDataSource::New(
                 names.size(), names.data(), values.data()))
        .Finish();
}

const HdDataSourceLocator &
_RegionLocator()
{
    static const HdDataSourceLocator locator =
        HdPrimvarsSchema::GetDefaultLocator().Append(
            _tokens->rigExecTouchRegion);
    return locator;
}

const HdDataSourceLocator &
_TableLocator()
{
    static const HdDataSourceLocator locator =
        HdPrimvarsSchema::GetDefaultLocator().Append(
            _tokens->rigExecTouchTable);
    return locator;
}

}  // namespace

RigExecTouchPoseSceneIndexRefPtr
RigExecTouchPoseSceneIndex::New(const HdSceneIndexBaseRefPtr &input)
{
    RigExecTouchPoseSceneIndexRefPtr result =
        TfCreateRefPtr(new RigExecTouchPoseSceneIndex(input));
    RigExecTouchPoseHighlights::GetInstance().RegisterSceneIndex(result);
    // A chain built while TouchPose is already on (a renderer switch, a
    // second viewport) starts wrapped rather than waiting for the next
    // attach.
    for (const SdfPath &mesh :
             RigExecTouchPoseHighlights::GetInstance().GetMeshPaths()) {
        HdSceneIndexObserver::DirtiedPrimEntries ignored;
        result->_SetMeshMaterials(
            mesh, result->_ComputeBoundMaterials(mesh), &ignored);
    }
    return result;
}

RigExecTouchPoseSceneIndex::RigExecTouchPoseSceneIndex(
    const HdSceneIndexBaseRefPtr &input)
    : HdSingleInputFilteringSceneIndexBase(input)
{
    SetDisplayName("RigExec TouchPose highlight");
}

HdSceneIndexPrim
RigExecTouchPoseSceneIndex::GetPrim(const SdfPath &primPath) const
{
    HdSceneIndexPrim prim = _GetInputSceneIndex()->GetPrim(primPath);
    if (!prim.dataSource ||
        !RigExecTouchPoseHighlights::GetInstance().HasMeshes()) {
        return prim;
    }
    if (prim.primType == HdPrimTypeTokens->mesh) {
        if (const RigExecTouchPoseHighlightMeshConstPtr mesh =
                RigExecTouchPoseHighlights::GetInstance().Find(primPath)) {
            prim.dataSource = HdOverlayContainerDataSource::New(
                _PrimvarsOverlay(*mesh), prim.dataSource);
        }
    } else if (prim.primType == HdPrimTypeTokens->material) {
        if (_IsWrapped(primPath)) {
            prim.dataSource = _WrapMaterial(primPath, prim.dataSource);
        }
    }
    return prim;
}

SdfPathVector
RigExecTouchPoseSceneIndex::GetChildPrimPaths(const SdfPath &primPath) const
{
    return _GetInputSceneIndex()->GetChildPrimPaths(primPath);
}

bool
RigExecTouchPoseSceneIndex::_IsWrapped(const SdfPath &path) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _wrapCount.find(path);
    return it != _wrapCount.end() && it->second > 0;
}

std::set<SdfPath>
RigExecTouchPoseSceneIndex::GetWrappedMaterials(const SdfPath &mesh) const
{
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _materialsByMesh.find(mesh);
    return it == _materialsByMesh.end() ? std::set<SdfPath>() : it->second;
}

std::set<SdfPath>
RigExecTouchPoseSceneIndex::_ComputeBoundMaterials(const SdfPath &mesh) const
{
    std::set<SdfPath> result;
    const HdSceneIndexBaseRefPtr &input = _GetInputSceneIndex();
    auto collect = [&](const SdfPath &path) {
        const HdSceneIndexPrim prim = input->GetPrim(path);
        if (!prim.dataSource) {
            return;
        }
        const HdMaterialBindingsSchema bindings =
            HdMaterialBindingsSchema::GetFromParent(prim.dataSource);
        if (!bindings) {
            return;
        }
        // Every purpose, not just the one Storm resolves: wrapping a
        // material that ends up unused costs nothing, and guessing the
        // purpose wrong would leave the region dark.
        const HdContainerDataSourceHandle container = bindings.GetContainer();
        for (const TfToken &purpose : container->GetNames()) {
            const HdMaterialBindingSchema binding =
                bindings.GetMaterialBinding(purpose);
            if (const HdPathDataSourceHandle pathSource = binding.GetPath()) {
                const SdfPath bound = pathSource->GetTypedValue(0.0f);
                if (!bound.IsEmpty()) {
                    result.insert(bound);
                }
            }
        }
    };
    collect(mesh);
    for (const SdfPath &child : input->GetChildPrimPaths(mesh)) {
        if (input->GetPrim(child).primType == HdPrimTypeTokens->geomSubset) {
            collect(child);
        }
    }
    return result;
}

void
RigExecTouchPoseSceneIndex::_SetMeshMaterials(
    const SdfPath &mesh, const std::set<SdfPath> &materials,
    HdSceneIndexObserver::DirtiedPrimEntries *dirty)
{
    std::set<SdfPath> changed;
    {
        std::lock_guard<std::mutex> lock(_mutex);
        std::set<SdfPath> before;
        auto it = _materialsByMesh.find(mesh);
        if (it != _materialsByMesh.end()) {
            before = it->second;
        }
        if (before == materials) {
            return;
        }
        for (const SdfPath &path : before) {
            if (!materials.count(path) && --_wrapCount[path] <= 0) {
                _wrapCount.erase(path);
                changed.insert(path);
            }
        }
        for (const SdfPath &path : materials) {
            if (!before.count(path) && ++_wrapCount[path] == 1) {
                changed.insert(path);
            }
        }
        if (materials.empty()) {
            _materialsByMesh.erase(mesh);
        } else {
            _materialsByMesh[mesh] = materials;
        }
    }
    for (const SdfPath &path : changed) {
        dirty->emplace_back(path, HdMaterialSchema::GetDefaultLocator());
    }
}

void
RigExecTouchPoseSceneIndex::HighlightChanged(
    const SdfPath &mesh, RigExecTouchPoseHighlights::Change change)
{
    using Change = RigExecTouchPoseHighlights::Change;
    HdSceneIndexObserver::DirtiedPrimEntries dirty;
    switch (change) {
    case Change::Attached:
    case Change::Detached: {
        // The primvars APPEAR or DISAPPEAR, which is a change to the
        // descriptor set, not to a value. Dirtying the primvar locators
        // themselves (not their primvarValue) is what makes the adapter
        // scene delegate drop its cached descriptors and Storm re-filter
        // the mesh's primvars against the new material.
        _SetMeshMaterials(
            mesh,
            change == Change::Attached ? _ComputeBoundMaterials(mesh)
                                       : std::set<SdfPath>(),
            &dirty);
        HdDataSourceLocatorSet locators;
        locators.insert(_RegionLocator());
        locators.insert(_TableLocator());
        dirty.emplace_back(mesh, locators);
        break;
    }
    case Change::Slots:
        dirty.emplace_back(
            mesh, HdDataSourceLocatorSet{_RegionLocator().Append(
                      HdPrimvarSchemaTokens->primvarValue)});
        break;
    case Change::Table:
        dirty.emplace_back(
            mesh, HdDataSourceLocatorSet{_TableLocator().Append(
                      HdPrimvarSchemaTokens->primvarValue)});
        break;
    }
    if (!dirty.empty() && _IsObserved()) {
        _SendPrimsDirtied(dirty);
    }
}

void
RigExecTouchPoseSceneIndex::_PrimsAdded(
    const HdSceneIndexBase &sender,
    const HdSceneIndexObserver::AddedPrimEntries &entries)
{
    // A resync of an attached mesh (or of one of its subsets) may bring
    // different bindings with it.
    HdSceneIndexObserver::DirtiedPrimEntries dirty;
    if (RigExecTouchPoseHighlights::GetInstance().HasMeshes()) {
        std::set<SdfPath> meshes;
        for (const SdfPath &mesh :
                 RigExecTouchPoseHighlights::GetInstance().GetMeshPaths()) {
            for (const auto &entry : entries) {
                if (entry.primPath.HasPrefix(mesh)) {
                    meshes.insert(mesh);
                    break;
                }
            }
        }
        for (const SdfPath &mesh : meshes) {
            _SetMeshMaterials(mesh, _ComputeBoundMaterials(mesh), &dirty);
        }
    }
    _SendPrimsAdded(entries);
    if (!dirty.empty()) {
        _SendPrimsDirtied(dirty);
    }
}

void
RigExecTouchPoseSceneIndex::_PrimsRemoved(
    const HdSceneIndexBase &sender,
    const HdSceneIndexObserver::RemovedPrimEntries &entries)
{
    _SendPrimsRemoved(entries);
}

void
RigExecTouchPoseSceneIndex::_PrimsDirtied(
    const HdSceneIndexBase &sender,
    const HdSceneIndexObserver::DirtiedPrimEntries &entries)
{
    HdSceneIndexObserver::DirtiedPrimEntries dirty;
    if (RigExecTouchPoseHighlights::GetInstance().HasMeshes()) {
        std::set<SdfPath> meshes;
        for (const SdfPath &mesh :
                 RigExecTouchPoseHighlights::GetInstance().GetMeshPaths()) {
            for (const auto &entry : entries) {
                if (entry.primPath.HasPrefix(mesh) &&
                    entry.dirtyLocators.Intersects(
                        HdMaterialBindingsSchema::GetDefaultLocator())) {
                    meshes.insert(mesh);
                    break;
                }
            }
        }
        for (const SdfPath &mesh : meshes) {
            _SetMeshMaterials(mesh, _ComputeBoundMaterials(mesh), &dirty);
        }
    }
    _SendPrimsDirtied(entries);
    if (!dirty.empty()) {
        _SendPrimsDirtied(dirty);
    }
}

}  // namespace rigExec

// ===========================================================================
// registration
// ===========================================================================

PXR_NAMESPACE_OPEN_SCOPE

/// Appends RigExecTouchPoseSceneIndex to every UsdImaging chain. A plugin of
/// its own rather than a line in RigExecUsdImagingSceneIndexPlugin, so the
/// highlight has no ordering dependency on the rig's filters: it only adds
/// data, and the rig's filters neither read nor replace it.
///
/// In the pxr namespace, like its sibling, so its TfType name is the plain
/// class name plugInfo.json declares.
class RigExecTouchPoseSceneIndexPlugin final
    : public UsdImagingSceneIndexPlugin {
public:
    HdSceneIndexBaseRefPtr AppendSceneIndex(
        HdSceneIndexBaseRefPtr const &inputScene) override
    {
        return rigExec::RigExecTouchPoseSceneIndex::New(inputScene);
    }
};

TF_REGISTRY_FUNCTION(UsdImagingSceneIndexPlugin)
{
    UsdImagingSceneIndexPlugin::Define<RigExecTouchPoseSceneIndexPlugin>();
}

PXR_NAMESPACE_CLOSE_SCOPE

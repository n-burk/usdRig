// RigExec baked playback for Hydra (M2b). See playback.h for the contract.
#include "playback.h"
#include "rigExecRuntime/stageArrayInputs.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/movers/moverRegistry.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace rigExec {

namespace {

TF_DEFINE_PRIVATE_TOKENS(
    _playbackTokens,
    ((asset, "rigExec:asset"))
    ((generated, "__RigExecGenerated"))
    (points)
    (normals)
    (extent)
);

// FNV-1a 64 over the file bytes: the binding-epoch digest. Only stability
// per file is required (a binary's published prim set never changes), so
// a non-cryptographic hash is the whole of it.
uint64_t
_PlaybackDigestBytes(const std::vector<uint8_t> &bytes)
{
    uint64_t hash = 14695981039346656037ULL;
    for (uint8_t byte : bytes) {
        hash ^= uint64_t(byte);
        hash *= 1099511628211ULL;
    }
    return hash ? hash : 1;
}

bool
_PlaybackPathOk(const std::string &path)
{
    return !path.empty() && SdfPath::IsValidPathString(path) &&
           SdfPath(path).IsAbsolutePath();
}

}  // namespace

bool
RigExecPlaybackAssetFor(const UsdPrim &rig, std::string *resolvedPath)
{
    if (!rig || !resolvedPath) {
        return false;
    }
    const UsdAttribute attribute = rig.GetAttribute(_playbackTokens->asset);
    if (!attribute) {
        return false;
    }
    SdfAssetPath asset;
    if (!attribute.Get(&asset)) {
        return false;
    }
    // Resolved against the layer that authored it (the usual asset rule),
    // falling back to the authored string when no resolver anchored it.
    std::string resolved = asset.GetResolvedPath();
    if (resolved.empty()) {
        resolved = asset.GetAssetPath();
    }
    if (resolved.empty()) {
        return false;
    }
    *resolvedPath = resolved;
    return true;
}

RigExecBakedPlayback::RigExecBakedPlayback(
    const UsdStageRefPtr &stage, const SdfPath &rigPath,
    std::shared_ptr<RigExecSnapshotStore> store)
    : _stage(stage)
    , _rigPath(rigPath)
    , _assetRoot(rigPath.GetParentPath())
    , _store(std::move(store))
{
}

RigExecBakedPlayback::~RigExecBakedPlayback() = default;

bool
RigExecBakedPlayback::Open(const std::string &resolvedPath,
                           std::string *error)
{
    std::ifstream stream(resolvedPath, std::ios::binary);
    const std::vector<char> raw((std::istreambuf_iterator<char>(stream)),
                                std::istreambuf_iterator<char>());
    const std::vector<uint8_t> bytes(raw.begin(), raw.end());
    if (bytes.empty()) {
        if (error) {
            *error = "cannot read " + resolvedPath;
        }
        return false;
    }
    std::string why;
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &why);
    if (!reader) {
        if (error) {
            *error = "cannot open " + resolvedPath + ": " + why;
        }
        return false;
    }
    // Plugin movers play through the kernel their plugin registered. One
    // with none loaded here passes its points through: said once here, and
    // in the diagnostics of every frame it does.
    for (const std::string &type : reader->GetExternalMoverTypes()) {
        const RigExecMoverHandler *handler =
            RigExecFindMoverHandler(TfToken(type));
        std::string why;
        if (!handler || !handler->runtimeKernel.IsSet()) {
            TF_WARN("rigExec: %s holds %s movers and no loaded plugin "
                    "provides their playback kernel; they pass their "
                    "points through",
                    resolvedPath.c_str(), type.c_str());
        } else if (!reader->SetExternalKernel(type, handler->runtimeKernel,
                                              &why)) {
            TF_WARN("rigExec: %s: %s; those movers pass their points "
                    "through",
                    resolvedPath.c_str(), why.c_str());
        }
    }
    // The inputs bind once the kernels are in: a binary baked from this
    // stage resolves every one, so a miss is said once, here.
    RigExecInputSampler sampler;
    if (!sampler.Bind(_stage, *reader, &why)) {
        if (error) {
            *error = "cannot play " + resolvedPath + ": " + why;
        }
        return false;
    }
    const std::vector<std::string> &warnings = sampler.GetWarnings();
    if (!warnings.empty()) {
        std::string named;
        for (size_t i = 0; i < warnings.size() && i < 4; ++i) {
            named += (i ? "; " : "") + warnings[i];
        }
        if (warnings.size() > 4) {
            named += "; ...";
        }
        TF_WARN("rigExec: %s: %zu input(s) keep their bake-time value: %s",
                resolvedPath.c_str(), warnings.size(), named.c_str());
    }
    _sampler = std::move(sampler);
    _epochDigest = _PlaybackDigestBytes(bytes);
    _reader = std::move(reader);
    _assetPath = resolvedPath;
    // Admission condition 3 in playback: a listed input, holding its tag's
    // type, including the evaluator's admitted arrays.
    _listedInputs.clear();
    for (size_t i = 0; i < _reader->GetInputCount(); ++i) {
        const RigExecRuntimeInputInfo &info = _reader->GetInputInfo(i);
        if (SdfPath::IsValidPathString(info.name)) {
            _listedInputs.emplace(SdfPath(info.name),
                                  RigExecInputTagType(info.type));
        }
    }
    _upstreamApplied.clear();
    _AdmitUpstream(UsdTimeCode(_reader->GetBakeTime()));
    return true;
}

void
RigExecBakedPlayback::SetUpstreamInputs(
    std::vector<RigExecUpstreamValue> values)
{
    _upstreamRequested = std::move(values);
    _AdmitUpstream(_upstreamTime);
}

std::vector<SdfPath>
RigExecBakedPlayback::GetUpstreamInputPaths() const
{
    std::vector<SdfPath> paths;
    paths.reserve(_upstreamAdmitted.size());
    for (const auto &[path, key] : _upstreamAdmitted) {
        paths.push_back(path);
    }
    return paths;
}

void
RigExecBakedPlayback::_AdmitUpstream(UsdTimeCode time)
{
    _upstreamTime = time;
    _upstreamAdmitted.clear();
    _upstreamDropLines.clear();
    if (!_reader) {
        return;
    }
    // Live's rule over what a playback session has: the file's listed
    // inputs stand for the program's admissible set, which equals them over
    // unconnected attributes with a stage value. Each entry is judged
    // before the last-wins rule, and the reason text is live's.
    for (const RigExecUpstreamValue &entry : _upstreamRequested) {
        const auto drop = [&](const std::string &reason) {
            _upstreamDropLines.push_back("upstream input " +
                                         entry.path.GetString() + ": " +
                                         reason + "; ignored");
        };
        if (!entry.path.IsPrimPropertyPath()) {
            drop("names no attribute");
            continue;
        }
        std::string reason = RigExecUpstreamDropReason(
            _stage, &_listedInputs, entry.path, entry.value, time, nullptr,
            &_listedInputs);
        size_t index = 0;
        if (reason.empty() &&
            !_reader->FindInput(entry.path.GetString(), &index)) {
            reason = "no listed read reaches it";
        }
        // Only a file baked from another stage types an input apart from
        // its attribute.
        if (reason.empty() &&
            entry.value.GetType() !=
                RigExecInputTagType(_reader->GetInputInfo(index).type)) {
            reason = std::string("the file's input holds a ") +
                     RigExecInputTagName(_reader->GetInputInfo(index).type);
        }
        if (!reason.empty()) {
            drop(reason);
            continue;
        }
        // Authored array sets reach both time and rest reads. Their lift
        // restores the stage sample at playback time for Animated slots.
        const RigExecRuntimeInputInfo &info = _reader->GetInputInfo(index);
        _UpstreamKey key;
        key.index = index;
        key.name = info.name;
        key.tag = info.type;
        key.animated = info.animated;
        key.attribute = _stage->GetAttributeAtPath(entry.path);
        key.value = entry.value;
        _upstreamAdmitted[entry.path] = std::move(key);
    }
}

bool
RigExecBakedPlayback::_LiftUpstream(const _UpstreamKey &key,
                                    UsdTimeCode time, bool sampled,
                                    std::string *error)
{
    // A non-Animated input's default is the bake-time stage value, which is
    // the stage value at every time. An Animated one takes the stage at
    // this time: Apply already read it when it sampled, else one read here.
    const bool sampledArray = !RrInputTagIsArray(key.tag) ||
        RigExecRuntimeStageArrayInputs::CanSample(*_reader, key.index);
    if (!key.animated || !sampledArray) {
        return _reader->ResetInput(key.name, error);
    }
    return sampled || RigExecSampleInputAt(key.attribute, key.index,
                                           key.name, key.tag, time,
                                           _reader.get(), error);
}

bool
RigExecBakedPlayback::_ApplyUpstream(UsdTimeCode time, bool sampled,
                                     std::string *error)
{
    for (const auto &[path, key] : _upstreamApplied) {
        if (!_upstreamAdmitted.count(path) &&
            !_LiftUpstream(key, time, sampled, error)) {
            return false;
        }
    }
    std::map<SdfPath, _UpstreamKey> applied;
    std::vector<SdfPath> refused;
    for (const auto &[path, key] : _upstreamAdmitted) {
        const auto was = _upstreamApplied.find(path);
        const bool moved =
            was == _upstreamApplied.end() || was->second.value != key.value;
        // Apply overwrote an Animated input with the stage at the new time.
        if (moved || (key.animated && sampled)) {
            // SetSampledInputAt: the value stands where the stage value
            // stood, and a stage value may be non-finite. A Token is set by
            // its text, which the reader interns when the file lacks it.
            RrInputValue value;
            std::string why;
            RigExecRuntimeArray array;
            const bool set =
                RrInputTagIsArray(key.tag)
                    ? RigExecInputArrayFrom(key.value, key.tag, &array) &&
                          _reader->SetInputArrayAt(key.index, array, &why)
                    : key.tag == RrInputTag::Token
                    ? _reader->SetInputToken(
                          key.name,
                          key.value.UncheckedGet<TfToken>().GetString(),
                          &why)
                    : RigExecInputValueFrom(key.value, key.tag, &value) &&
                          _reader->SetSampledInputAt(key.index, value, &why);
            if (!set) {
                // A refused set lifts the key, reported as live reports a
                // dropped one.
                _upstreamDropLines.push_back(
                    "upstream input " + path.GetString() + ": " +
                    (why.empty() ? std::string("the input refused it")
                                 : why) +
                    "; ignored");
                refused.push_back(path);
                if (!_LiftUpstream(key, time, sampled, error)) {
                    return false;
                }
                continue;
            }
        }
        applied.emplace(path, key);
    }
    for (const SdfPath &path : refused) {
        _upstreamAdmitted.erase(path);
    }
    _upstreamApplied = std::move(applied);
    return true;
}

SdfPath
RigExecBakedPlayback::GetGeneratedScope() const
{
    return _rigPath.AppendChild(_playbackTokens->generated);
}

RigExecImagingBridge::PublishResult
RigExecBakedPlayback::EvaluateAndPublishResult(UsdTimeCode time)
{
    RigExecImagingBridge::PublishResult result;
    if (!_reader) {
        return result;
    }
    std::string why;
    bool sampled = false;
    // Upstream values go in after Apply, never through
    // _sampler.Invalidate(): an invalidated Apply touches every Animated
    // input, so every varying step would re-run for a change that reaches
    // only its own readers.
    bool ran = _sampler.Apply(time, _reader.get(), &why, &sampled);
    if (ran) {
        _AdmitUpstream(time);
        ran = _ApplyUpstream(time, sampled, &why) && _reader->Execute(&why);
    }
    if (!ran) {
        // The bridge's rule: a rig that cannot evaluate stops driving the
        // scene, and the next good generation re-announces its epoch.
        result.dirtied = _store->Publish(nullptr);
        _publishedEpochDigest = 0;
        return result;
    }

    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->generation = ++_generation;
    // The requested time: the merge only accepts a generation that
    // describes the stage/time it asked for.
    snapshot->stage = UsdStageWeakPtr(_stage);
    snapshot->sampleTimeIsDefault = time.IsDefault();
    snapshot->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();
    snapshot->assetRoot = _assetRoot;

    const TfToken &pointsName = _playbackTokens->points;
    const TfToken &normalsName = _playbackTokens->normals;
    const TfToken &extentName = _playbackTokens->extent;
    for (const RigExecRuntimePoints &moved : _reader->GetPoints()) {
        if (!_PlaybackPathOk(moved.path)) {
            continue;
        }
        const SdfPath propertyPath(moved.path);
        if (!propertyPath.IsPropertyPath()) {
            continue;
        }
        const SdfPath primPath = propertyPath.GetPrimPath();
        const TfToken property = propertyPath.GetNameToken();
        RigExecPublishedPrim *published = nullptr;
        if (property == pointsName || property == normalsName ||
            (property == extentName && moved.points.size() == 2)) {
            published = &snapshot->prims[primPath];
            published->assetRoot = _assetRoot;
        } else {
            continue;
        }
        if (property == pointsName) {
            published->hasPoints = true;
            published->points.resize(moved.points.size());
            for (size_t i = 0; i < moved.points.size(); ++i) {
                published->points[i] = GfVec3f(
                    moved.points[i][0], moved.points[i][1],
                    moved.points[i][2]);
            }
        } else if (property == normalsName) {
            published->hasNormals = true;
            published->normals.resize(moved.points.size());
            for (size_t i = 0; i < moved.points.size(); ++i) {
                published->normals[i] = GfVec3f(
                    moved.points[i][0], moved.points[i][1],
                    moved.points[i][2]);
            }
        } else {
            published->hasExtent = true;
            published->extentMin = GfVec3d(
                moved.points[0][0], moved.points[0][1], moved.points[0][2]);
            published->extentMax = GfVec3d(
                moved.points[1][0], moved.points[1][1], moved.points[1][2]);
        }
    }

    // Constraint-driven transforms, published onto the prim itself so
    // parented geometry rides along -- the bridge's _FillProviderXforms,
    // fed by the runtime's revised/base pair instead of the pose.
    std::set<SdfPath> scanned;
    for (const RigExecRuntimeProviderXform &provider :
         _reader->GetProviderXforms()) {
        if (!_PlaybackPathOk(provider.path)) {
            continue;
        }
        const SdfPath providerPath(provider.path);
        if (!providerPath.IsPrimPath()) {
            continue;
        }
        RigExecPublishedPrim &published = snapshot->prims[providerPath];
        published.assetRoot = _assetRoot;
        published.xform = GfMatrix4d(
            provider.matrix[0][0], provider.matrix[0][1],
            provider.matrix[0][2], provider.matrix[0][3],
            provider.matrix[1][0], provider.matrix[1][1],
            provider.matrix[1][2], provider.matrix[1][3],
            provider.matrix[2][0], provider.matrix[2][1],
            provider.matrix[2][2], provider.matrix[2][3],
            provider.matrix[3][0], provider.matrix[3][1],
            provider.matrix[3][2], provider.matrix[3][3]);
        published.xformBase = GfMatrix4d(
            provider.base[0][0], provider.base[0][1],
            provider.base[0][2], provider.base[0][3],
            provider.base[1][0], provider.base[1][1],
            provider.base[1][2], provider.base[1][3],
            provider.base[2][0], provider.base[2][1],
            provider.base[2][2], provider.base[2][3],
            provider.base[3][0], provider.base[3][1],
            provider.base[3][2], provider.base[3][3]);
        published.hasXform = true;
        snapshot->hasDrivenXforms = true;
        // Authored reset boundaries beneath driven roots, captured while
        // the stage is still in reach: flattening erases them later.
        bool covered = false;
        for (const SdfPath &root : scanned) {
            covered = covered || providerPath.HasPrefix(root);
        }
        if (covered) {
            continue;
        }
        scanned.insert(providerPath);
        const UsdPrim root = _stage->GetPrimAtPath(providerPath);
        if (!root) {
            continue;
        }
        for (const UsdPrim &prim : UsdPrimRange(root)) {
            const UsdGeomXformable xform(prim);
            if (xform && xform.GetResetXformStack()) {
                snapshot->xformResetPaths.insert(prim.GetPath());
            }
        }
    }

    // The influence overlay, from the fields the runtime published --
    // what the movers consumed, not a re-derivation, like the bridge.
    if (!_weightOverlay.IsEmpty()) {
        for (const RigExecRuntimeWeightField &field :
             _reader->GetWeightFields()) {
            if (field.path != _weightOverlay.GetString() ||
                field.weights.empty() ||
                !_PlaybackPathOk(field.target)) {
                continue;
            }
            const SdfPath geomPath =
                SdfPath(field.target).GetPrimPath();
            if (geomPath.IsEmpty() || geomPath.IsAbsoluteRootPath()) {
                continue;
            }
            RigExecPublishedPrim &published = snapshot->prims[geomPath];
            published.assetRoot = _assetRoot;
            published.hasWeightOverlay = true;
            published.weightOverlay = VtFloatArray(
                field.weights.begin(), field.weights.end());
            break;
        }
    }

    // One epoch ever: the digest is stable per file, so only the first
    // publication carries it.
    if (_epochDigest != _publishedEpochDigest) {
        auto epoch = std::make_shared<
            RigExecBindingResolvingSceneIndex::BindingEpoch>();
        epoch->id = _epochDigest;
        for (const auto &[primPath, published] : snapshot->prims) {
            epoch->publishedPrims.insert(primPath);
        }
        result.epoch = std::move(epoch);
        _publishedEpochDigest = _epochDigest;
    }

    result.dirtied = _store->Publish(std::move(snapshot));
    result.ok = true;
    return result;
}

}  // namespace rigExec

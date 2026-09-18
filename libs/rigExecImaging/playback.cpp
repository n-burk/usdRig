//
// RigExec baked playback for Hydra (M2b). See playback.h for the contract.
//

#include "playback.h"

#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <cmath>
#include <fstream>
#include <limits>

namespace rigExec {

namespace {

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
    static const TfToken assetName("rigExec:asset");
    const UsdAttribute attribute = rig.GetAttribute(assetName);
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
    if (reader->GetFrameTimes().empty()) {
        if (error) {
            *error = resolvedPath + " carries no baked frames";
        }
        return false;
    }
    _epochDigest = _PlaybackDigestBytes(bytes);
    _reader = std::move(reader);
    _assetPath = resolvedPath;
    return true;
}

SdfPath
RigExecBakedPlayback::GetGeneratedScope() const
{
    return _rigPath.AppendChild(TfToken("__RigExecGenerated"));
}

double
RigExecBakedPlayback::MapTimeToFrame(UsdTimeCode time) const
{
    if (!_reader) {
        return 0.0;
    }
    const std::vector<double> frames = _reader->GetFrameTimes();
    if (frames.empty()) {
        return 0.0;
    }
    const double t = time.IsDefault() ? 0.0 : time.GetValue();
    double best = frames[0];
    double gap = std::fabs(frames[0] - t);
    for (size_t i = 1; i < frames.size(); ++i) {
        const double next = std::fabs(frames[i] - t);
        // Strictly smaller wins, so a tie keeps the lower frame.
        if (next < gap) {
            gap = next;
            best = frames[i];
        }
    }
    return best;
}

RigExecImagingBridge::PublishResult
RigExecBakedPlayback::EvaluateAndPublishResult(UsdTimeCode time)
{
    RigExecImagingBridge::PublishResult result;
    if (!_reader) {
        return result;
    }
    std::string why;
    if (!_reader->SetFrame(MapTimeToFrame(time), &why) ||
        !_reader->Execute(&why)) {
        // The bridge's rule: a rig that cannot evaluate stops driving the
        // scene, and the next good generation re-announces its epoch.
        result.dirtied = _store->Publish(nullptr);
        _publishedEpochDigest = 0;
        return result;
    }

    auto snapshot = std::make_shared<RigExecImagingSnapshot>();
    snapshot->generation = ++_generation;
    // The REQUESTED time, not the baked frame it mapped to: the merge
    // only accepts a generation that describes the stage/time it asked
    // for, and the mapping is this class's internal answer.
    snapshot->stage = UsdStageWeakPtr(_stage);
    snapshot->sampleTimeIsDefault = time.IsDefault();
    snapshot->sampleTime = time.IsDefault() ? 0.0 : time.GetValue();
    snapshot->assetRoot = _assetRoot;

    static const TfToken pointsName("points");
    static const TfToken normalsName("normals");
    static const TfToken extentName("extent");
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

#include "pack.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/sdf/relationshipSpec.h"
#include "pxr/usd/sdf/zipFile.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>

namespace rigExec {
namespace {
const std::string statesKey = "__rigexecPackStates";
const std::string missingKey = "__rigexecPackMissing";
const std::string customKey = "__rigexecPackHadCustomData";
bool Fail(std::string *error, const std::string &message) {
    if (error) *error = message;
    return false;
}
template<class T> bool Read(const VtDictionary &dict, const std::string &key, T *out) {
    const auto found = dict.find(key);
    if (found == dict.end() || !found->second.IsHolding<T>()) return false;
    *out = found->second.UncheckedGet<T>();
    return true;
}
VtDictionary Metadata(const UsdObject &object) {
    VtDictionary result;
    for (const auto &[name, value] : object.GetAllMetadata()) {
        const std::string key = name.GetString();
        if (key != "default" && key != "timeSamples" && key != "spline" &&
            key != "connectionPaths" && key != "targetPaths") result[key] = value;
    }
    return result;
}
void WriteMetadata(const SdfSpecHandle &spec, const VtDictionary &metadata) {
    for (const auto &[key, value] : metadata) spec->SetInfo(TfToken(key), value);
}
VtDictionary ReadMetadata(const SdfSpecHandle &spec) {
    VtDictionary result;
    for (const TfToken &key : spec->ListInfoKeys()) {
        if (key != TfToken("connectionPaths") && key != TfToken("targetPaths") &&
            key != TfToken("default") && key != TfToken("timeSamples") && key != TfToken("spline"))
            result[key.GetString()] = spec->GetInfo(key);
    }
    return result;
}
VtDictionary AssetParts(const SdfAssetPath &asset) {
    return {{"authored", VtValue(asset.GetAuthoredPath())},
            {"evaluated", VtValue(asset.GetEvaluatedPath())},
            {"resolved", VtValue(asset.GetResolvedPath())}};
}
VtValue Encode(const VtValue &value) {
    // Ordinary USD asset serialization drops resolved/evaluated fields. Keep
    // these generic native-value components so a stage-less consumer need not
    // run an asset resolver to recover the exported value.
    if (value.IsHolding<SdfAssetPath>()) return VtValue(AssetParts(value.UncheckedGet<SdfAssetPath>()));
    if (value.IsHolding<VtArray<SdfAssetPath>>()) {
        VtDictionary array;
        const auto &values = value.UncheckedGet<VtArray<SdfAssetPath>>();
        for (size_t i = 0; i < values.size(); ++i) array[std::to_string(i)] = VtValue(AssetParts(values[i]));
        return VtValue(array);
    }
    return value;
}
bool DecodeAsset(const VtValue &value, SdfAssetPath *asset) {
    if (!value.IsHolding<VtDictionary>()) return false;
    std::string authored, evaluated, resolved;
    const auto &parts = value.UncheckedGet<VtDictionary>();
    if (!Read(parts,"authored",&authored) || !Read(parts,"evaluated",&evaluated) || !Read(parts,"resolved",&resolved)) return false;
    *asset = SdfAssetPath(SdfAssetPathParams().Authored(authored).Evaluated(evaluated).Resolved(resolved));
    return true;
}
bool Decode(const VtValue &value, SdfValueTypeName type, VtValue *out) {
    if (type == SdfValueTypeNames->Asset) {
        SdfAssetPath asset;
        if (!DecodeAsset(value, &asset)) return false;
        *out = VtValue(asset);
    } else if (type == SdfValueTypeNames->AssetArray) {
        if (!value.IsHolding<VtDictionary>()) return false;
        const auto &entries = value.UncheckedGet<VtDictionary>();
        VtArray<SdfAssetPath> assets(entries.size());
        for (size_t i = 0; i < assets.size(); ++i) {
            const auto entry = entries.find(std::to_string(i));
            if (entry == entries.end() || !DecodeAsset(entry->second, &assets[i])) return false;
        }
        *out = VtValue(assets);
    } else *out = value;
    return out->GetType() == type.GetType();
}
bool ValidIdentity(const std::string &key) {
    if (key == "default") return true;
    const size_t prefix = key.rfind("pre:",0) == 0 ? 4 : key.rfind("exact:",0) == 0 ? 6 : 0;
    if (!prefix || key.size() != prefix + 16) return false;
    uint64_t bits = 0;
    for (size_t i = prefix; i < key.size(); ++i) {
        const char c = key[i];
        if (!(c >= '0' && c <= '9') && !(c >= 'a' && c <= 'f')) return false;
        bits = bits * 16 + (c <= '9' ? c - '0' : c - 'a' + 10);
    }
    double time;
    std::memcpy(&time,&bits,sizeof(time));
    return RigExecStandaloneTimeKey(prefix == 4 ? UsdTimeCode::PreTime(time) : UsdTimeCode(time)) == key;
}
}

bool RigExecExportRigPack(const UsdStageRefPtr &stage,
    const std::vector<UsdTimeCode> &times, const std::string &packPath,
    const std::string &sourceAssetId, std::string *error)
{
    if (!stage || times.empty() || sourceAssetId.empty()) return Fail(error,"pack export requires a stage, identities and sourceAssetId");
    namespace fs = std::filesystem;
    std::error_code pathError;
    const auto destination = fs::weakly_canonical(fs::absolute(packPath,pathError),pathError);
    if (pathError) return Fail(error,"invalid rigpack destination path");
    for (const auto &layer : stage->GetUsedLayers()) {
        if (!layer->GetRealPath().empty() && fs::exists(destination,pathError) &&
            fs::equivalent(destination,fs::path(layer->GetRealPath()),pathError))
            return Fail(error,"rigpack destination cannot overwrite a source layer");
    }
    RigExecSceneDb db;
    db.prims[SdfPath::AbsoluteRootPath()] = {};
    db.sourceAssetId = sourceAssetId;
    db.timeCodesPerSecond = stage->GetTimeCodesPerSecond();
    db.framesPerSecond = stage->GetFramesPerSecond();
    db.interpolation = TfToken(stage->GetInterpolationType() == UsdInterpolationTypeHeld ? "held" : "linear");
    std::map<std::string,UsdTimeCode> identities;
    for (UsdTimeCode time : times) {
        const auto key = RigExecStandaloneTimeKey(time);
        if (key.empty()) return Fail(error,"pack identities must be finite");
        db.identities.insert(key);
        identities.emplace(key,time);
    }
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (!RigExecStandaloneSupportsPrimType(prim.GetTypeName()))
            return Fail(error,"unsupported provider-only pack type " + prim.GetTypeName().GetString());
        const auto apis = prim.GetAppliedSchemas();
        SdfPathVector moves;
        if (const auto rel = prim.GetRelationship(TfToken("rigExec:moves"))) rel.GetTargets(&moves);
        if (std::find(apis.begin(),apis.end(),TfToken("RigExecMoverAPI")) != apis.end() ||
            !moves.empty())
            return Fail(error,"provider-only pack does not lower mover applications: " + prim.GetPath().GetString());
        SdfPathVector joints;
        if (const auto rel = prim.GetRelationship(TfToken("rigExec:joints"))) rel.GetTargets(&joints);
        if (!joints.empty()) return Fail(error,"provider-only pack does not lower solver-to-joint output bindings: " + prim.GetPath().GetString());
        if (prim.IsInstance() || prim.IsInstanceProxy()) return Fail(error,"provider-only pack does not flatten instance providers");
        db.prims[prim.GetPath()] = {prim.GetTypeName(),apis,Metadata(prim),prim.IsActive()};
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            RigExecStandaloneAttribute value;
            value.type = attr.GetTypeName();
            value.metadata = Metadata(attr);
            attr.GetConnections(&value.connections);
            for (const auto &[key,time] : identities) {
                VtValue resolved;
                if (!attr.Get(&resolved,time)) resolved = VtValue();
                value.resolved.emplace(key,std::move(resolved));
            }
            db.attributes.emplace(attr.GetPath(),std::move(value));
        }
        for (const UsdRelationship &rel : prim.GetRelationships()) {
            RigExecStandaloneRelationship value;
            value.metadata = Metadata(rel);
            rel.GetTargets(&value.targets);
            db.relationships.emplace(rel.GetPath(),std::move(value));
        }
    }
    if (!db.Validate(error) || !db.ValidateCapabilities(error)) return false;
    const SdfLayerRefPtr manifest = SdfLayer::CreateAnonymous("manifest.usda");
    manifest->SetCustomLayerData({{"packVersion",VtValue(1)}, {"scope",VtValue(std::string("providers"))},
        {"sourceAssetId",VtValue(sourceAssetId)}, {"timeCodesPerSecond",VtValue(db.timeCodesPerSecond)},
        {"framesPerSecond",VtValue(db.framesPerSecond)}, {"interpolation",VtValue(db.interpolation)},
        {"identities",VtValue(VtStringArray(db.identities.begin(),db.identities.end()))}});
    TfErrorMark errors;
    for (const auto &[path,prim] : db.prims) {
        if (path == SdfPath::AbsoluteRootPath()) continue;
        const auto spec = SdfCreatePrimInLayer(manifest,path);
        spec->SetSpecifier(SdfSpecifierDef);
        WriteMetadata(spec,prim.metadata);
        spec->SetTypeName(prim.type.GetString());
        spec->SetActive(prim.active);
        spec->SetInfo(TfToken("apiSchemas"),VtValue(SdfTokenListOp::CreateExplicit(prim.appliedSchemas)));
    }
    for (const auto &[path,attr] : db.attributes) {
        const auto spec = SdfAttributeSpec::New(manifest->GetPrimAtPath(path.GetPrimPath()),path.GetName(),attr.type);
        WriteMetadata(spec,attr.metadata);
        spec->GetConnectionPathList().GetExplicitItems() = attr.connections;
        VtDictionary custom, states;
        const bool hadCustom = Read(attr.metadata,"customData",&custom);
        if (custom.count(statesKey) || custom.count(missingKey) || custom.count(customKey))
            return Fail(error,"source uses reserved pack metadata keys");
        VtStringArray missing;
        for (const auto &[key,value] : attr.resolved) {
            if (value.IsEmpty()) missing.push_back(key);
            else states[key] = Encode(value);
        }
        custom[statesKey] = VtValue(states);
        custom[missingKey] = VtValue(missing);
        custom[customKey] = VtValue(hadCustom);
        spec->SetInfo(TfToken("customData"),VtValue(custom));
    }
    for (const auto &[path,rel] : db.relationships) {
        const auto spec = SdfRelationshipSpec::New(manifest->GetPrimAtPath(path.GetPrimPath()),path.GetName());
        WriteMetadata(spec,rel.metadata);
        spec->GetTargetPathList().GetExplicitItems() = rel.targets;
    }
    if (!errors.IsClean()) { errors.Clear(); return Fail(error,"cannot serialize native provider metadata"); }
    static std::atomic<uint64_t> next{0};
    const auto tag = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" + std::to_string(next++);
    const fs::path scratch = fs::temp_directory_path() / ("rigexec-pack-" + tag);
    std::error_code ec;
    if (!fs::create_directory(scratch,ec)) return Fail(error,"cannot create pack scratch directory");
    struct Cleanup { fs::path path; ~Cleanup() { std::error_code ec; fs::remove_all(path,ec); } } cleanup{scratch};
    const auto sourcePath = (scratch / "source.usdc").string(), manifestPath = (scratch / "manifest.usda").string();
    const auto source = stage->Flatten();
    if (!source || !source->Export(sourcePath) || !manifest->Export(manifestPath)) return Fail(error,"could not serialize pack payloads");
    // ZipFileWriter records each payload's local DOS modification timestamp.
    // Normalize to a fixed local date so repeated exports and time zones do
    // not inject incidental build-time bytes into an otherwise identical pack.
    std::tm fixed{}; fixed.tm_year = 100; fixed.tm_mon = 0; fixed.tm_mday = 1;
    fixed.tm_hour = 12; fixed.tm_sec = 1; fixed.tm_isdst = -1;
    const auto fileNow = fs::file_time_type::clock::now();
    const auto wallNow = std::chrono::system_clock::now();
    const auto fixedFile = fileNow + std::chrono::duration_cast<fs::file_time_type::duration>(
        std::chrono::system_clock::from_time_t(std::mktime(&fixed)) - wallNow);
    fs::last_write_time(sourcePath,fixedFile,ec);
    if (ec) return Fail(error,"cannot normalize pack payload timestamp");
    fs::last_write_time(manifestPath,fixedFile,ec);
    if (ec) return Fail(error,"cannot normalize pack payload timestamp");
    auto writer = SdfZipFileWriter::CreateNew(packPath);
    if (!writer) return Fail(error,"cannot create rigpack archive");
    if (writer.AddFile(sourcePath,"source.usdc").empty() || writer.AddFile(manifestPath,"manifest.usda").empty()) {
        writer.Discard(); return Fail(error,"could not add rigpack payloads");
    }
    return writer.Save() || Fail(error,"could not save rigpack archive");
}

std::shared_ptr<RigExecSceneDb> RigExecLoadRigPack(const std::string &packPath, std::string *error)
{
    const auto fail = [&](const std::string &message) -> std::shared_ptr<RigExecSceneDb> { Fail(error,message); return {}; };
    const auto archive = SdfZipFile::Open(packPath);
    if (!archive) return fail("cannot open rigpack archive");
    const auto source = archive.Find("source.usdc"), entry = archive.Find("manifest.usda");
    if (source == archive.end() || entry == archive.end()) return fail("rigpack requires source.usdc and manifest.usda");
    const auto info = entry.GetFileInfo(), sourceInfo = source.GetFileInfo();
    if (!info.size || !entry.GetFile() || !source.GetFile() ||
        info.compressionMethod || info.encrypted || sourceInfo.compressionMethod || sourceInfo.encrypted ||
        sourceInfo.size < 8 || std::memcmp(source.GetFile(),"PXR-USDC",8)) return fail("unsupported rigpack payload encoding");
    const auto manifest = SdfLayer::CreateAnonymous("manifest.usda");
    if (!manifest->ImportFromString(std::string(entry.GetFile(),info.size))) return fail("invalid rigpack manifest");
    auto db = std::make_shared<RigExecSceneDb>();
    const auto header = manifest->GetCustomLayerData();
    int version;
    std::string scope;
    VtStringArray identities;
    if (!Read(header,"packVersion",&version) || version != 1 || !Read(header,"scope",&scope) || scope != "providers" ||
        !Read(header,"sourceAssetId",&db->sourceAssetId) || !Read(header,"timeCodesPerSecond",&db->timeCodesPerSecond) ||
        !Read(header,"framesPerSecond",&db->framesPerSecond) || !Read(header,"interpolation",&db->interpolation) ||
        !Read(header,"identities",&identities)) return fail("invalid rigpack header");
    for (const auto &key : identities) {
        if (!ValidIdentity(key) || !db->identities.insert(key).second) return fail("invalid or duplicate rigpack identity");
    }
    if (db->sourceAssetId.empty() || (db->interpolation != "linear" && db->interpolation != "held")) return fail("invalid rigpack source/policy");
    db->prims[SdfPath::AbsoluteRootPath()] = {};
    bool valid = true;
    std::string invalidRecord;
    manifest->Traverse(SdfPath::AbsoluteRootPath(),[&](const SdfPath &path) {
        if (path == SdfPath::AbsoluteRootPath()) return;
        if (path.IsTargetPath()) return; // connection/relationship target specs are represented by their owner lists
        if (const auto prim = manifest->GetPrimAtPath(path)) {
            auto &value = db->prims[path];
            value.type = TfToken(prim->GetTypeName()); value.active = prim->GetActive();
            value.metadata = ReadMetadata(prim);
            const auto apis = prim->GetInfo(TfToken("apiSchemas"));
            if (apis.IsHolding<SdfTokenListOp>()) value.appliedSchemas = apis.UncheckedGet<SdfTokenListOp>().GetExplicitItems();
        } else if (const auto attr = manifest->GetAttributeAtPath(path)) {
            auto &value = db->attributes[path];
            value.type = attr->GetTypeName(); value.metadata = ReadMetadata(attr);
            value.connections = attr->GetConnectionPathList().GetAppliedItems();
            VtDictionary custom, states;
            VtStringArray missing;
            bool hadCustom;
            if (!Read(value.metadata,"customData",&custom) || !Read(custom,statesKey,&states) ||
                !Read(custom,missingKey,&missing) || !Read(custom,customKey,&hadCustom)) { valid = false; invalidRecord = path.GetString() + ": missing state metadata"; return; }
            for (const auto &[key,state] : states) {
                VtValue decoded;
                if (!db->identities.count(key) || !Decode(state,value.type,&decoded)) {
                    valid = false; invalidRecord = path.GetString() + " " + key + ": state type " + state.GetTypeName() + " does not match " + value.type.GetAsToken().GetString(); return;
                }
                value.resolved.emplace(key,std::move(decoded));
            }
            for (const auto &key : missing) {
                if (!db->identities.count(key) || !value.resolved.emplace(key,VtValue()).second) { valid = false; invalidRecord = path.GetString() + ": duplicate or unknown missing state"; return; }
            }
            custom.erase(statesKey); custom.erase(missingKey); custom.erase(customKey);
            if (hadCustom) value.metadata["customData"] = VtValue(custom);
            else value.metadata.erase("customData");
        } else if (const auto rel = manifest->GetRelationshipAtPath(path)) {
            db->relationships[path] = {ReadMetadata(rel),rel->GetTargetPathList().GetAppliedItems()};
        } else { valid = false; invalidRecord = path.GetString() + ": unknown record"; }
    });
    if (!valid) return fail("invalid generic rigpack state record " + invalidRecord);
    if (!db->Validate(error) || !db->ValidateCapabilities(error)) return {};
    return db;
}
}

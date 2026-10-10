#include "pxr/base/ts/spline.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pack.h"
#include "rigExec/inputReplayValues.h"
#include <stdexcept>
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
#include "pxr/usd/usd/resolveInfo.h"
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
const std::string factsKey = "__rigexecPackSourceFacts";
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
VtValue Encode(const VtValue &value) {
    std::string bytes,error;
    if(!RigExecEncodeInputValue(value,&bytes,&error))throw std::runtime_error(error);
    static const char digits[]="0123456789abcdef";
    std::string hex;hex.reserve(bytes.size()*2);
    for(unsigned char byte:bytes) {hex.push_back(digits[byte>>4]);hex.push_back(digits[byte&15]);}
    return VtValue(hex);
}
bool DecodeExact(const VtValue &value,VtValue *out) {
    if(!value.IsHolding<std::string>())return false;
    const auto &hex=value.UncheckedGet<std::string>();
    if(hex.size()%2)return false;
    auto nibble=[](char c)->int {return c>='0'&&c<='9'?c-'0':c>='a'&&c<='f'?c-'a'+10:-1;};
    std::string bytes;bytes.reserve(hex.size()/2);
    for(size_t i=0;i<hex.size();i+=2) {int a=nibble(hex[i]),b=nibble(hex[i+1]);if(a<0||b<0)return false;bytes.push_back(char((a<<4)|b));}
    std::string error;return RigExecDecodeInputValue(bytes,out,&error);
}
bool Decode(const VtValue &value,SdfValueTypeName type,VtValue *out) {
    return DecodeExact(value,out) && out->GetType()==type.GetType();
}
void WriteMetadata(const SdfSpecHandle &spec,const VtDictionary &metadata) {
    spec->SetInfo(TfToken("customData"),VtValue(VtDictionary{{"__rigexecExactMetadata",Encode(VtValue(metadata))}}));
}
VtDictionary ReadMetadata(const SdfSpecHandle &spec) {
    VtValue decoded;VtDictionary custom;
    if(!spec->GetInfo(TfToken("customData")).IsHolding<VtDictionary>())throw std::runtime_error("missing exact pack metadata");
    custom=spec->GetInfo(TfToken("customData")).UncheckedGet<VtDictionary>();
    auto found=custom.find("__rigexecExactMetadata");
    if(found==custom.end() || !DecodeExact(found->second,&decoded) || !decoded.IsHolding<VtDictionary>())throw std::runtime_error("invalid exact pack metadata");
    return decoded.UncheckedGet<VtDictionary>();
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
try {
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
    for (const auto &child : stage->GetPseudoRoot().GetAllChildren())
        db.prims[SdfPath::AbsoluteRootPath()].children.push_back(child.GetPath());
    db.sourceAssetId = sourceAssetId;
    db.timeCodesPerSecond = stage->GetTimeCodesPerSecond();
    db.framesPerSecond = stage->GetFramesPerSecond();
    db.upAxis = UsdGeomGetStageUpAxis(stage);
    db.interpolation = TfToken(stage->GetInterpolationType() == UsdInterpolationTypeHeld ? "held" : "linear");
    std::map<std::string,UsdTimeCode> identities;
    // Static source reads (rest arrays, operations and element maps) always
    // require exact Default independently of the requested numeric samples.
    db.identities.insert("default");
    identities.emplace("default",UsdTimeCode::Default());
    for (UsdTimeCode time : times) {
        const auto key = RigExecStandaloneTimeKey(time);
        if (key.empty()) return Fail(error,"pack identities must be finite");
        db.identities.insert(key);
        identities.emplace(key,time);
    }
    for (const UsdPrim &prim : stage->TraverseAll()) {
        const auto apis=prim.GetAppliedSchemas();
        if(prim.IsInstance() || prim.IsInstanceProxy())return Fail(error,"scene pack requires flattened instance composition");
        db.prims[prim.GetPath()] = {prim.GetTypeName(),apis,Metadata(prim),prim.IsActive()};
        for (const auto &child : prim.GetAllChildren()) db.prims[prim.GetPath()].children.push_back(child.GetPath());
        for (const UsdAttribute &attr : prim.GetAttributes()) {
            RigExecStandaloneAttribute value;
            value.type = attr.GetTypeName();
            value.metadata = Metadata(attr);
            if(attr.HasSpline())value.spline=VtValue(attr.GetSpline());
            attr.GetConnections(&value.connections);
            value.hasValue = attr.HasValue();
            value.variability = attr.GetVariability();
            value.hasAuthoredValue = attr.HasAuthoredValueOpinion();
            value.hasAuthoredReadableValue = attr.HasAuthoredValue();
            value.hasAuthoredConnections = attr.HasAuthoredConnections();
            value.mightBeTimeVarying = attr.ValueMightBeTimeVarying();
            attr.GetTimeSamples(&value.sampleTimes);
            for (const auto &spec : attr.GetPropertyStack(UsdTimeCode::Default())) {
                if (!spec->HasInfo(TfToken("default"))) continue;
                value.hasAuthoredDefault = true;
                value.authoredDefault=spec->GetInfo(TfToken("default"));
                value.defaultBlocked = value.authoredDefault.IsHolding<SdfValueBlock>();
                break;
            }
            for (const auto &[key,time] : identities) {
                VtValue resolved;
                if (!attr.Get(&resolved,time)) resolved = VtValue();
                if (attr.GetResolveInfo(time).ValueIsBlocked()) value.blockedIdentities.insert(key);
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
    VtDictionary children;
    for (const auto &[path, prim] : db.prims) {
        VtStringArray paths;
        for (const auto &child : prim.children) paths.push_back(child.GetString());
        children[path.GetString()] = VtValue(paths);
    }
    manifest->SetCustomLayerData({{"packVersion",VtValue(3)}, {"scope",VtValue(std::string("scene"))},
        {"sourceAssetId",VtValue(sourceAssetId)}, {"timeCodesPerSecond",Encode(VtValue(db.timeCodesPerSecond))},
        {"framesPerSecond",Encode(VtValue(db.framesPerSecond))}, {"interpolation",VtValue(db.interpolation)},
        {"upAxis",VtValue(db.upAxis)},
        {"identities",VtValue(VtStringArray(db.identities.begin(),db.identities.end()))},
        {"children", VtValue(children)}});
    TfErrorMark errors;
    for (const auto &[path,prim] : db.prims) {
        if (path == SdfPath::AbsoluteRootPath()) continue;
        const auto spec = SdfCreatePrimInLayer(manifest,path);
        spec->SetSpecifier(SdfSpecifierDef);
        WriteMetadata(spec,prim.metadata);
        if(!prim.type.IsEmpty())spec->SetTypeName(prim.type.GetString());
        spec->SetActive(prim.active);
        spec->SetInfo(TfToken("apiSchemas"),VtValue(SdfTokenListOp::CreateExplicit(prim.appliedSchemas)));
    }
    for (const auto &[path,attr] : db.attributes) {
        const auto spec = SdfAttributeSpec::New(manifest->GetPrimAtPath(path.GetPrimPath()),path.GetName(),attr.type);
        WriteMetadata(spec,attr.metadata);

        spec->GetConnectionPathList().GetExplicitItems() = attr.connections;
        VtDictionary custom, states;
        const bool hadCustom = Read(attr.metadata,"customData",&custom);
        if (custom.count(statesKey) || custom.count(missingKey) || custom.count(customKey) || custom.count(factsKey))
            return Fail(error,"source uses reserved pack metadata keys");
        VtStringArray missing;
        for (const auto &[key,value] : attr.resolved) {
            if (value.IsEmpty()) missing.push_back(key);
            else states[key] = Encode(value);
        }
        custom[statesKey] = VtValue(states);
        custom[missingKey] = VtValue(missing);
        custom[customKey] = VtValue(hadCustom);
        custom[factsKey] = Encode(VtValue(VtDictionary{
            {"hasValue", VtValue(attr.hasValue)},
            {"hasSpline", VtValue(!attr.spline.IsEmpty())},
            {"spline",attr.spline},
            {"hasAuthoredValue", VtValue(attr.hasAuthoredValue)},
            {"hasAuthoredReadableValue",VtValue(attr.hasAuthoredReadableValue)},
            {"hasAuthoredConnections", VtValue(attr.hasAuthoredConnections)},
            {"mightBeTimeVarying", VtValue(attr.mightBeTimeVarying)},
            {"hasAuthoredDefault", VtValue(attr.hasAuthoredDefault)},
            {"defaultBlocked", VtValue(attr.defaultBlocked)},
            {"authoredDefault",attr.authoredDefault},
            {"variability", VtValue(int(attr.variability))},
            {"blockedIdentities", VtValue(VtStringArray(attr.blockedIdentities.begin(), attr.blockedIdentities.end()))},
            {"sampleTimes", VtValue(VtDoubleArray(attr.sampleTimes.begin(), attr.sampleTimes.end()))}}));
        auto metadata=attr.metadata;metadata["customData"]=VtValue(custom);WriteMetadata(spec,metadata);
    }
    for (const auto &[path,rel] : db.relationships) {
        const auto spec = SdfRelationshipSpec::New(manifest->GetPrimAtPath(path.GetPrimPath()),path.GetName());
        WriteMetadata(spec,rel.metadata);
        spec->GetTargetPathList().GetExplicitItems() = rel.targets;
    }
    if (!errors.IsClean()) { std::string why="cannot serialize native provider metadata";for(auto it=errors.GetBegin();it!=errors.GetEnd();++it)why+="; "+it->GetCommentary();errors.Clear();return Fail(error,why); }
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
} catch(const std::exception &exception) {return Fail(error,std::string("exact rigpack export: ")+exception.what());}

std::shared_ptr<RigExecSceneDb> RigExecLoadRigPack(const std::string &packPath, std::string *error)
try {
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
    VtDictionary children;
    if (!Read(header,"packVersion",&version) || version != 3)
        return fail("rigpack version 3 required; regenerate archives for exact semantic values");
    VtValue tcps,fps;
    const auto tcpsEntry=header.find("timeCodesPerSecond");
    const auto fpsEntry=header.find("framesPerSecond");
    if(tcpsEntry==header.end() || fpsEntry==header.end() ||
       !DecodeExact(tcpsEntry->second,&tcps) || !tcps.IsHolding<double>() ||
       !DecodeExact(fpsEntry->second,&fps) || !fps.IsHolding<double>())return fail("invalid exact stage rates");
    db->timeCodesPerSecond=tcps.UncheckedGet<double>();db->framesPerSecond=fps.UncheckedGet<double>();
    if (!Read(header,"scope",&scope) || scope != "scene" ||
        !Read(header,"sourceAssetId",&db->sourceAssetId) || !Read(header,"interpolation",&db->interpolation) ||
        !Read(header,"upAxis",&db->upAxis) ||
        !Read(header,"identities",&identities) || !Read(header,"children",&children)) return fail("invalid rigpack header");
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
            VtDictionary facts;
            VtDoubleArray samples;
            int variability;
            bool hasSpline=false;
            VtStringArray blocked;
            VtValue decodedFacts;
            const auto factEntry=custom.find(factsKey);
            if(factEntry==custom.end() || !DecodeExact(factEntry->second,&decodedFacts) || !decodedFacts.IsHolding<VtDictionary>()) {valid=false;invalidRecord="invalid exact source facts";return;}
            facts=decodedFacts.UncheckedGet<VtDictionary>();
            const auto defaultOpinion=facts.find("authoredDefault");if(defaultOpinion==facts.end()){valid=false;invalidRecord="missing exact default opinion";return;}value.authoredDefault=defaultOpinion->second;
            const auto spline=facts.find("spline");if(spline==facts.end()){valid=false;invalidRecord="missing exact spline";return;}value.spline=spline->second;
            if (
                !Read(facts,"hasValue",&value.hasValue) ||
                !Read(facts,"hasSpline",&hasSpline) ||
                hasSpline!=!value.spline.IsEmpty() ||
                !Read(facts,"hasAuthoredValue",&value.hasAuthoredValue) ||
                !Read(facts,"hasAuthoredReadableValue",&value.hasAuthoredReadableValue) ||
                !Read(facts,"hasAuthoredConnections",&value.hasAuthoredConnections) ||
                !Read(facts,"mightBeTimeVarying",&value.mightBeTimeVarying) ||
                !Read(facts,"hasAuthoredDefault",&value.hasAuthoredDefault) ||
                !Read(facts,"defaultBlocked",&value.defaultBlocked) ||
                !Read(facts,"variability",&variability) ||
                !Read(facts,"blockedIdentities",&blocked) ||
                (variability != SdfVariabilityUniform && variability != SdfVariabilityVarying) ||
                !Read(facts,"sampleTimes",&samples)) {
                valid = false; invalidRecord = path.GetString() + ": missing authored source facts"; return;
            }
            value.sampleTimes.assign(samples.begin(), samples.end());
            value.variability = SdfVariability(variability);
            for (const auto &key : blocked)
                if (!value.blockedIdentities.insert(key).second) {
                    valid = false; invalidRecord = path.GetString() + ": duplicate blocked identity"; return;
                }
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
            custom.erase(statesKey); custom.erase(missingKey); custom.erase(customKey); custom.erase(factsKey);
            if (hadCustom) value.metadata["customData"] = VtValue(custom);
            else value.metadata.erase("customData");
        } else if (const auto rel = manifest->GetRelationshipAtPath(path)) {
            db->relationships[path] = {ReadMetadata(rel),rel->GetTargetPathList().GetAppliedItems()};
        } else { valid = false; invalidRecord = path.GetString() + ": unknown record"; }
    });
    if (!valid) return fail("invalid generic rigpack state record " + invalidRecord);
    if (children.size() != db->prims.size()) return fail("incomplete rigpack child order");
    for (auto &[path, prim] : db->prims) {
        VtStringArray paths;
        if (!Read(children,path.GetString(),&paths)) return fail("missing rigpack child order: " + path.GetString());
        std::set<SdfPath> seen;
        for (const auto &childText : paths) {
            const SdfPath child(childText);
            if (!db->prims.count(child) || child.GetParentPath() != path || !seen.insert(child).second)
                return fail("invalid rigpack child order: " + path.GetString());
            prim.children.push_back(child);
        }
        for (const auto &[child, unused] : db->prims)
            if (child != SdfPath::AbsoluteRootPath() && child.GetParentPath() == path && !seen.count(child))
                return fail("incomplete rigpack child order: " + path.GetString());
    }
    if (!db->Validate(error) || !db->ValidateCapabilities(error)) return {};
    return db;
} catch(const std::exception &exception) {Fail(error,std::string("exact rigpack load: ")+exception.what());return {}; }
}

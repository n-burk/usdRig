#include "inputReplay.h"
#include "inputReplayValues.h"
#include "inputReplayWire.h"
#include "rigEvaluator.h"
#include "bakedProgram.h"
#include "heldProgramGolden.h"
#include "compileGolden.h"
#include "tapSet.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/scopeDescription.h"
#include "pxr/usd/ar/defaultResolverContext.h"
#include "pxr/usd/sdf/abstractData.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layerStateDelegate.h"
#include "pxr/usd/sdf/notice.h"
#include "pxr/usd/sdf/payload.h"
#include "pxr/usd/sdf/reference.h"
#include "pxr/usd/usd/notice.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <thread>
#include <typeinfo>

namespace rigExec {
namespace {
using inputReplay::Event;
using inputReplay::Op;
using inputReplay::Reader;
using inputReplay::Writer;
thread_local unsigned suppressed = 0;
thread_local unsigned implementationDepth = 0;
constexpr unsigned noLayer = UINT32_MAX;
constexpr char magic[] = "rigexec-input-actions-5\n";
constexpr char lifetimeMagic[] = "rigexec-input-actions-4\n";
constexpr char heldMagic[] = "rigexec-input-actions-3\n";
constexpr char legacyMagic[] = "rigexec-input-actions-2\n";

[[noreturn]] void CaptureFailure(const std::string &reason) {
    std::fprintf(stderr,"input replay capture refused: %s\n",reason.c_str());
    std::fflush(stderr);std::_Exit(EXIT_FAILURE);
}
std::string ReadFile(const std::string &path) {
    std::ifstream stream(path,std::ios::binary);
    if(!stream)throw std::runtime_error("cannot read input/provenance file: "+path);
    std::string bytes{std::istreambuf_iterator<char>(stream),std::istreambuf_iterator<char>()};
    if(stream.bad())throw std::runtime_error("cannot finish reading input file: "+path);
    return bytes;
}
std::string ValueBytes(const VtValue &value) {
    std::string bytes,error;
    if(!RigExecEncodeInputValue(value,&bytes,&error))throw std::runtime_error(error);
    return bytes;
}
VtValue ReadValue(Reader &reader) {
    VtValue value;std::string error;
    if(!RigExecDecodeInputValue(reader.Text(),&value,&error))throw std::runtime_error(error);
    return value;
}
void Value(Writer &writer,const VtValue &value) { writer.Text(ValueBytes(value)); }

// Anonymous identifiers are object references, not new authored asset names.
// File-backed identifiers remain unchanged so relative asset anchors survive.
using Aliases=std::map<std::string,std::string>;
std::string Alias(const std::string &value,const Aliases &aliases) {
    const auto found=aliases.find(value);return found==aliases.end()?value:found->second;
}
VtValue Rebind(const VtValue &value,const Aliases &aliases);
SdfReference RebindOne(const SdfReference &v,const Aliases &aliases) {
    return SdfReference(Alias(v.GetAssetPath(),aliases),v.GetPrimPath(),v.GetLayerOffset(),
        Rebind(VtValue(v.GetCustomData()),aliases).Get<VtDictionary>());
}
SdfPayload RebindOne(const SdfPayload &v,const Aliases &aliases) {
    return SdfPayload(Alias(v.GetAssetPath(),aliases),v.GetPrimPath(),v.GetLayerOffset());
}
template<class T> VtValue RebindList(const SdfListOp<T> &source,const Aliases &aliases) {
    auto map=[&](const std::vector<T> &items){auto out=items;for(auto &item:out)item=RebindOne(item,aliases);return out;};
    SdfListOp<T> out;
    if(source.IsExplicit())out.SetExplicitItems(map(source.GetExplicitItems()));
    else {out.SetAddedItems(map(source.GetAddedItems()));out.SetPrependedItems(map(source.GetPrependedItems()));
        out.SetAppendedItems(map(source.GetAppendedItems()));out.SetDeletedItems(map(source.GetDeletedItems()));out.SetOrderedItems(map(source.GetOrderedItems()));}
    return VtValue(out);
}
VtValue Rebind(const VtValue &value,const Aliases &aliases) {
    if(value.IsHolding<SdfAssetPath>()) {const auto &v=value.Get<SdfAssetPath>();
        return VtValue(SdfAssetPath(SdfAssetPathParams().Authored(Alias(v.GetAuthoredPath(),aliases))
            .Evaluated(Alias(v.GetEvaluatedPath(),aliases)).Resolved(Alias(v.GetResolvedPath(),aliases))));}
    if(value.IsHolding<VtArray<SdfAssetPath>>()) {auto out=value.Get<VtArray<SdfAssetPath>>();
        for(auto &v:out)v=Rebind(VtValue(v),aliases).Get<SdfAssetPath>();return VtValue(out);}
    if(value.IsHolding<SdfReference>())return VtValue(RebindOne(value.Get<SdfReference>(),aliases));
    if(value.IsHolding<SdfPayload>())return VtValue(RebindOne(value.Get<SdfPayload>(),aliases));
    if(value.IsHolding<SdfReferenceListOp>())return RebindList(value.Get<SdfReferenceListOp>(),aliases);
    if(value.IsHolding<SdfPayloadListOp>())return RebindList(value.Get<SdfPayloadListOp>(),aliases);
    if(value.IsHolding<VtDictionary>()) {auto out=value.Get<VtDictionary>();for(auto &v:out)v.second=Rebind(v.second,aliases);return VtValue(out);}
    if(value.IsHolding<SdfTimeSampleMap>()) {auto out=value.Get<SdfTimeSampleMap>();for(auto &v:out)v.second=Rebind(v.second,aliases);return VtValue(out);}
    if(value.IsHolding<std::vector<VtValue>>()) {auto out=value.Get<std::vector<VtValue>>();for(auto &v:out)v=Rebind(v,aliases);return VtValue(out);}
    return value;
}
VtValue RebindField(const TfToken &field,const VtValue &value,const Aliases &aliases) {
    // Only Sdf's sublayer field assigns layer identity semantics to strings.
    // Ordinary string inputs and customData remain byte-for-byte authored.
    if(field==TfToken("subLayers")) {
        if(value.IsEmpty())return value;
        if(value.IsHolding<std::vector<std::string>>()) {auto out=value.Get<std::vector<std::string>>();for(auto &v:out)v=Alias(v,aliases);return VtValue(out);}
        throw std::runtime_error("unsupported sublayer field value type: "+value.GetTypeName());
    }
    return Rebind(value,aliases);
}

uint32_t Flags(const SdfChangeList::Entry &entry) {
    const auto &f=entry.flags;uint32_t bits=0;unsigned n=0;
#define FLAG(name) bits|=uint32_t(f.name)<<n++
    FLAG(didChangeIdentifier);FLAG(didChangeResolvedPath);FLAG(didReplaceContent);FLAG(didReloadContent);
    FLAG(didReorderChildren);FLAG(didReorderProperties);FLAG(didRename);FLAG(didChangePrimVariantSets);
    FLAG(didChangePrimInheritPaths);FLAG(didChangePrimSpecializes);FLAG(didChangePrimReferences);
    FLAG(didChangeAttributeTimeSamples);FLAG(didChangeAttributeConnection);FLAG(didChangeRelationshipTargets);
    FLAG(didAddTarget);FLAG(didRemoveTarget);FLAG(didAddInertPrim);FLAG(didAddNonInertPrim);
    FLAG(didRemoveInertPrim);FLAG(didRemoveNonInertPrim);FLAG(didAddPropertyWithOnlyRequiredFields);
    FLAG(didAddProperty);FLAG(didRemovePropertyWithOnlyRequiredFields);FLAG(didRemoveProperty);
#undef FLAG
    return bits;
}
using Layers=std::map<SdfLayerHandle,unsigned>;
std::string NoticeBytes(const SdfNotice::LayersDidChange &notice,const Layers &layers,const Aliases &aliases,bool allowReplace=false) {
    // Sdf may order its layer collection by process-specific pointer identity.
    // Audit semantic layer IDs; ordinary edits retain path/change entry order.
    std::vector<const SdfLayerChangeListVec::value_type *> ordered;
    for(const auto &item:notice.GetChangeListVec())if(layers.count(item.first))ordered.push_back(&item);
    std::sort(ordered.begin(),ordered.end(),[&](const auto *a,const auto *b){return layers.at(a->first)<layers.at(b->first);});
    Writer out;out.U32(unsigned(ordered.size()));
    for(const auto *pointer:ordered) {
        const auto &item=*pointer;const auto found=layers.find(item.first);
        out.U32(found->second);out.U32(unsigned(item.second.GetEntryList().size()));
        std::vector<const SdfChangeList::EntryList::value_type *> entries;
        for(const auto &entry:item.second.GetEntryList())entries.push_back(&entry);
        // A known ImportFromString action computes its field diff through
        // SdfAbstractData::VisitSpecs, whose traversal order is undefined.
        // Normalize only that single replacement action's spec enumeration.
        if(allowReplace)std::sort(entries.begin(),entries.end(),[](const auto *a,const auto *b){return a->first.GetString()<b->first.GetString();});
        for(const auto *entryPointer:entries) {
            const auto &entry=*entryPointer;
            if(Flags(entry.second)&(allowReplace?11u:15u))throw std::runtime_error("unsupported bulk reload/replacement or layer identity edit: "+item.first->GetIdentifier());
            out.Text(entry.first.GetString());out.U32(Flags(entry.second));out.Text(entry.second.oldPath.GetString());
            out.Text(Alias(entry.second.oldIdentifier,aliases));out.U32(unsigned(entry.second.infoChanged.size()));
            for(const auto &info:entry.second.infoChanged) {out.Text(info.first.GetString());
                Value(out,RebindField(info.first,info.second.first,aliases));Value(out,RebindField(info.first,info.second.second,aliases));}
            out.U32(unsigned(entry.second.subLayerChanges.size()));
            for(const auto &change:entry.second.subLayerChanges){out.Text(Alias(change.first,aliases));out.U32(unsigned(change.second));}
        }
    }
    return out.bytes;
}
std::string NormalizeImportNotice(const std::string &bytes) {
    Reader r(bytes);Writer out;const auto layers=r.U32();out.U32(layers);
    for(unsigned layer=0;layer<layers;++layer) {
        const auto id=r.U32(),count=r.U32();out.U32(id);out.U32(count);
        std::vector<std::pair<std::string,std::string>> entries;
        for(unsigned entry=0;entry<count;++entry) {
            const size_t begin=r.at;const auto path=r.Text();r.U32();r.Text();r.Text();
            const auto fields=r.U32();
            for(unsigned field=0;field<fields;++field){r.Text();r.Text();r.Text();}
            const auto sublayers=r.U32();
            for(unsigned sublayer=0;sublayer<sublayers;++sublayer){r.Text();r.U32();}
            entries.emplace_back(path,bytes.substr(begin,r.at-begin));
        }
        std::sort(entries.begin(),entries.end(),[](const auto &a,const auto &b){return a.first<b.first;});
        for(size_t i=0;i<entries.size();++i) {
            if(i&&entries[i-1].first==entries[i].first)throw std::runtime_error("duplicate known-import notice path");
            out.bytes+=entries[i].second;
        }
    }
    r.Finish();return out.bytes;
}
std::string NoticeSummary(const std::string &bytes) {
    try {
        Reader r(bytes);std::string result;const auto count=r.U32();
        result="layers="+std::to_string(count);
        for(unsigned layer=0;layer<count;++layer) {
            const auto id=r.U32(),entries=r.U32();
            result+=" layer="+std::to_string(id)+" entries="+std::to_string(entries);
            for(unsigned i=0;i<entries;++i) {
                const auto path=r.Text();const auto flags=r.U32();
                const auto oldPath=r.Text(),oldIdentifier=r.Text();
                result+=" ["+path+" flags="+std::to_string(flags);
                if(!oldPath.empty())result+=" oldPath="+oldPath;
                if(!oldIdentifier.empty())result+=" oldIdentifier="+oldIdentifier;
                const auto fields=r.U32();
                for(unsigned field=0;field<fields;++field) {
                    result+=" field="+r.Text();r.Text();r.Text();
                }
                const auto sublayers=r.U32();
                for(unsigned sublayer=0;sublayer<sublayers;++sublayer) {
                    const auto path=r.Text();const auto change=r.U32();
                    result+=" sublayer="+path+":"+std::to_string(change);
                }
                result+="]";
            }
        }
        r.Finish();return result;
    } catch(const std::exception &e) {return std::string("invalid notice encoding: ")+e.what();}
}
std::string NoticeDifference(const std::string &expected,const std::vector<std::string> &actual) {
    std::string result="; expected "+NoticeSummary(expected)+"; actual notice count="+std::to_string(actual.size());
    for(const auto &value:actual)result+=" {"+NoticeSummary(value)+"}";
    if(actual.size()==1) {
        const auto &value=actual.front();size_t offset=0;
        while(offset<expected.size()&&offset<value.size()&&expected[offset]==value[offset])++offset;
        result+="; first differing byte="+std::to_string(offset)+" expected size="+std::to_string(expected.size())+" actual size="+std::to_string(value.size());
        if(offset<expected.size())result+=" expected byte="+std::to_string(uint8_t(expected[offset]));
        if(offset<value.size())result+=" actual byte="+std::to_string(uint8_t(value[offset]));
    }
    return result;
}
struct Operation {
    unsigned layer;Op kind;SdfPath path;TfToken field,key;SdfPath other;
    VtValue value;double time=0;SdfSpecType type=SdfSpecTypeUnknown;bool inert=false;
};
std::string EncodeOperation(const Operation &op) {
    Writer w;w.U32(op.layer);w.U8(uint8_t(op.kind));w.Text(op.path.GetString());w.Text(op.field.GetString());
    w.Text(op.key.GetString());w.Text(op.other.GetString());Value(w,op.value);w.Double(op.time);w.U32(unsigned(op.type));w.U8(op.inert);return w.bytes;
}
Operation DecodeOperation(Reader &r) {
    Operation op;op.layer=r.U32();op.kind=Op(r.U8());op.path=SdfPath(r.Text());op.field=TfToken(r.Text());
    op.key=TfToken(r.Text());op.other=SdfPath(r.Text());op.value=ReadValue(r);op.time=r.Double();op.type=SdfSpecType(r.U32());
    const auto inert=r.U8();if(inert>1)throw std::runtime_error("invalid operation inert flag");op.inert=bool(inert);r.Finish();return op;
}
void ApplyOperation(const Operation &op,const SdfLayerRefPtr &layer,const Aliases &aliases,const std::vector<SdfLayerRefPtr> &layers) {
    const auto value=op.kind==Op::Field?RebindField(op.field,op.value,aliases):Rebind(op.value,aliases);const auto delegate=layer->GetStateDelegate();
    switch(op.kind) {
    case Op::Field: if(value.IsEmpty())layer->EraseField(op.path,op.field);else layer->SetField(op.path,op.field,value);break;
    case Op::Dictionary: if(value.IsEmpty())layer->EraseFieldDictValueByKey(op.path,op.field,op.key);else layer->SetFieldDictValueByKey(op.path,op.field,op.key,value);break;
    case Op::Sample: if(value.IsEmpty())layer->EraseTimeSample(op.path,op.time);else layer->SetTimeSample(op.path,op.time,value);break;
    case Op::Create:delegate->CreateSpec(op.path,op.type,op.inert);break;
    case Op::Delete:delegate->DeleteSpec(op.path,op.inert);break;
    case Op::Move:delegate->MoveSpec(op.path,op.other);break;
    case Op::PushToken:delegate->PushChild(op.path,op.field,value.Get<TfToken>());break;
    case Op::PushPath:delegate->PushChild(op.path,op.field,value.Get<SdfPath>());break;
    case Op::PopToken:delegate->PopChild(op.path,op.field,value.Get<TfToken>());break;
    case Op::PopPath:delegate->PopChild(op.path,op.field,value.Get<SdfPath>());break;
    case Op::Import:if(!layer->ImportFromString(value.Get<std::string>()))throw std::runtime_error("replayed authored ImportFromString failed");break;
    case Op::Clear:layer->Clear();break;
    case Op::Transfer:layer->TransferContent(layers.at(value.Get<unsigned>()));break;
    default:throw std::runtime_error("unknown input authoring operation");
    }
}
class Recorder;
std::unique_ptr<Recorder> recorder;
class SourceDelegate final:public SdfSimpleLayerStateDelegate {
public:
    static SdfLayerStateDelegateBaseRefPtr New(unsigned id){return TfCreateRefPtr(new SourceDelegate(id));}
private:
    explicit SourceDelegate(unsigned id):_id(id){}
    unsigned _id;
    void Record(Op,const SdfPath &,const TfToken & = {},const VtValue & = {},
                const TfToken & = {},const SdfPath & = {},double=0,SdfSpecType=SdfSpecTypeUnknown,bool=false);
    void _OnSetField(const SdfPath &p,const TfToken &f,const VtValue &v) override {SdfSimpleLayerStateDelegate::_OnSetField(p,f,v);Record(Op::Field,p,f,v);}
    void _OnSetField(const SdfPath &p,const TfToken &f,const SdfAbstractDataConstValue &v) override {VtValue out;if(!v.GetValue(&out))CaptureFailure("type-erased field cannot be copied");_OnSetField(p,f,out);}
    void _OnSetFieldDictValueByKey(const SdfPath &p,const TfToken &f,const TfToken &k,const VtValue &v) override {SdfSimpleLayerStateDelegate::_OnSetFieldDictValueByKey(p,f,k,v);Record(Op::Dictionary,p,f,v,k);}
    void _OnSetFieldDictValueByKey(const SdfPath &p,const TfToken &f,const TfToken &k,const SdfAbstractDataConstValue &v) override {VtValue out;if(!v.GetValue(&out))CaptureFailure("type-erased dictionary cannot be copied");_OnSetFieldDictValueByKey(p,f,k,out);}
    void _OnSetTimeSample(const SdfPath &p,double t,const VtValue &v) override {SdfSimpleLayerStateDelegate::_OnSetTimeSample(p,t,v);Record(Op::Sample,p,{},v,{},{},t);}
    void _OnSetTimeSample(const SdfPath &p,double t,const SdfAbstractDataConstValue &v) override {VtValue out;if(!v.GetValue(&out))CaptureFailure("type-erased sample cannot be copied");_OnSetTimeSample(p,t,out);}
    void _OnCreateSpec(const SdfPath &p,SdfSpecType t,bool i) override {SdfSimpleLayerStateDelegate::_OnCreateSpec(p,t,i);Record(Op::Create,p,{},{},{},{},0,t,i);}
    void _OnDeleteSpec(const SdfPath &p,bool i) override {SdfSimpleLayerStateDelegate::_OnDeleteSpec(p,i);Record(Op::Delete,p,{},{},{},{},0,SdfSpecTypeUnknown,i);}
    void _OnMoveSpec(const SdfPath &p,const SdfPath &q) override {SdfSimpleLayerStateDelegate::_OnMoveSpec(p,q);Record(Op::Move,p,{},{},{},q);}
    void _OnPushChild(const SdfPath &p,const TfToken &f,const TfToken &v) override {SdfSimpleLayerStateDelegate::_OnPushChild(p,f,v);Record(Op::PushToken,p,f,VtValue(v));}
    void _OnPushChild(const SdfPath &p,const TfToken &f,const SdfPath &v) override {SdfSimpleLayerStateDelegate::_OnPushChild(p,f,v);Record(Op::PushPath,p,f,VtValue(v));}
    void _OnPopChild(const SdfPath &p,const TfToken &f,const TfToken &v) override {SdfSimpleLayerStateDelegate::_OnPopChild(p,f,v);Record(Op::PopToken,p,f,VtValue(v));}
    void _OnPopChild(const SdfPath &p,const TfToken &f,const SdfPath &v) override {SdfSimpleLayerStateDelegate::_OnPopChild(p,f,v);Record(Op::PopPath,p,f,VtValue(v));}
};

class Recorder:public TfWeakBase {
public:
    struct Layer {SdfLayerHandle layer;SdfLayerStateDelegateBaseRefPtr previous,observer;bool edit;bool retired=false;};
    struct Stage {UsdStageWeakPtr stage;std::string state,configuration;bool retired=false;};
    struct Owner {unsigned stage;bool closed=false;size_t visits=0,actions=0;};
    struct Program {unsigned owner;bool closed=false,router=false;size_t runs=0;};
    std::vector<Program> programs;
    std::thread::id thread=std::this_thread::get_id();
    std::vector<Layer> layers;Layers layerIds;std::vector<Stage> stages;
    std::map<UsdStageWeakPtr,unsigned> stageIds;std::vector<Owner> owners;
    std::ofstream stream;uint64_t events=0,comparisons=0;unsigned pending=0,numerical=0,bulk=0,pendingReplacements=0;
    TfNotice::Key notice,stageNotice;
    explicit Recorder(const std::string &file) {
        if(std::filesystem::exists(file))throw std::runtime_error("input transcript already exists: "+file);
        const char *provenance=std::getenv("RIGEXEC_INPUT_REPLAY_PROVENANCE");
        if(!provenance||!*provenance)throw std::runtime_error("RIGEXEC_INPUT_REPLAY_PROVENANCE source/plugin manifest is required");
        const auto metadata=ReadFile(provenance);
        stream.open(file,std::ios::binary);if(!stream)throw std::runtime_error("cannot create input transcript: "+file);
        stream.write(magic,sizeof(magic)-1);Writer header;header.Text(provenance);header.Text(metadata);
        stream.write(header.bytes.data(),std::streamsize(header.bytes.size()));
        Writer initial;initial.U8(RigExecUpstreamArrayAdmission());
        Emit(Event::ArrayAdmission,initial.bytes);
        notice=TfNotice::Register(TfCreateWeakPtr(this),&Recorder::OnNotice);
        stageNotice=TfNotice::Register(TfCreateWeakPtr(this),&Recorder::OnStageNotice);
    }
    ~Recorder(){TfNotice::Revoke(notice);TfNotice::Revoke(stageNotice);for(auto &layer:layers)if(layer.layer)layer.layer->SetStateDelegate(layer.previous);}
    void Check() const {if(thread!=std::this_thread::get_id())CaptureFailure("source history must be serial on its owning thread");}
    void Emit(Event event,const std::string &payload) {
        Check();Writer row;row.U64(events++);row.U8(uint8_t(event));row.Text(payload);
        stream.write(row.bytes.data(),std::streamsize(row.bytes.size()));if(!stream)CaptureFailure("input transcript write failed");
    }
    void Boundary() {Check();if(numerical)CaptureFailure("caller source action during numerical evaluator implementation");if(pending)CaptureFailure("caller action inside an unfinished SdfChangeBlock");
        // Observe actual source expiry; owner destruction alone does not imply
        // that its caller released a stage or a separately held layer.
        for(unsigned id=0;id<stages.size();++id) {
            auto &stage=stages[id];
            if(!stage.retired&&!stage.stage) {
                for(const auto &owner:owners)if(owner.stage==id&&!owner.closed)
                    CaptureFailure("source stage expired with a live evaluator");
                stage.retired=true;stageIds.erase(stage.stage);
                Writer out;out.U32(id);Emit(Event::StageRetire,out.bytes);
            }
        }
        for(unsigned id=0;id<layers.size();++id) {
            auto &layer=layers[id];
            if(!layer.retired&&!layer.layer) {
                layer.retired=true;layerIds.erase(layer.layer);
                Writer out;out.U32(id);Emit(Event::LayerRetire,out.bytes);
            }
        }
        for(const auto &layer:layers)if(layer.layer&&(get_pointer(layer.layer->GetStateDelegate())!=get_pointer(layer.observer)||layer.layer->PermissionToEdit()!=layer.edit))
            CaptureFailure("source delegate/edit permission changed without a supported action: "+layer.layer->GetIdentifier());}
    unsigned AddLayer(const SdfLayerHandle &handle) {
        const auto found=layerIds.find(handle);if(found!=layerIds.end())return found->second;
        if(!handle)throw std::runtime_error("null source layer");
        const auto previous=handle->GetStateDelegate();
        if(!previous||typeid(*previous)!=typeid(SdfSimpleLayerStateDelegate))throw std::runtime_error("unsupported custom source layer state delegate: "+handle->GetIdentifier());
        const unsigned id=unsigned(layers.size());layerIds.emplace(handle,id);
        Layer layer{handle,SdfLayerStateDelegateBaseRefPtr(previous),SourceDelegate::New(id),handle->PermissionToEdit()};
        layers.push_back(layer);
        Writer out;out.U32(id);out.Text(handle->GetIdentifier());out.U8(handle->IsAnonymous());out.U8(layer.edit);
        const std::string real=handle->GetRealPath();out.Text(real);out.Text(real.empty()?std::string():ReadFile(real));
        std::vector<SdfPath> specs{SdfPath::AbsoluteRootPath()};
        handle->Traverse(SdfPath::AbsoluteRootPath(),[&](const SdfPath &p){if(p!=SdfPath::AbsoluteRootPath())specs.push_back(p);});
        std::sort(specs.begin(),specs.end(),[](const SdfPath &a,const SdfPath &b){return std::make_pair(a.GetPathElementCount(),a)<std::make_pair(b.GetPathElementCount(),b);});
        out.U32(unsigned(specs.size()));for(const auto &path:specs) {out.Text(path.GetString());out.U32(unsigned(handle->GetSpecType(path)));
            const auto fields=handle->ListFields(path);out.U32(unsigned(fields.size()));for(const auto &field:fields){out.Text(field.GetString());
                try{Value(out,handle->GetField(path,field));}catch(const std::exception &error){throw std::runtime_error(handle->GetIdentifier()+" "+path.GetString()+" "+field.GetString()+": "+error.what());}}}
        Emit(Event::Layer,out.bytes);handle->SetStateDelegate(layer.observer);return id;
    }
    std::string Configuration(const UsdStageRefPtr &stage) {
        Writer out;
        const auto context=stage->GetPathResolverContext();const auto *standard=context.Get<ArDefaultResolverContext>();
        if(!context.IsEmpty()&&(!standard||context!=ArResolverContext(*standard)))throw std::runtime_error("unsupported custom resolver context");
        const auto search=standard?standard->GetSearchPath():std::vector<std::string>();out.U8(bool(standard));out.U32(unsigned(search.size()));for(const auto &path:search)out.Text(path);
        const auto mask=stage->GetPopulationMask().GetPaths();out.U32(unsigned(mask.size()));for(const auto &path:mask)out.Text(path.GetString());
        const auto &rules=stage->GetLoadRules().GetRules();out.U32(unsigned(rules.size()));for(const auto &rule:rules){out.Text(rule.first.GetString());out.U32(unsigned(rule.second));}
        const auto &muted=stage->GetMutedLayers();out.U32(unsigned(muted.size()));for(const auto &path:muted)out.Text(path);
        return out.bytes;
    }
    std::string State(const UsdStageRefPtr &stage) {
        Writer out;out.U32(AddLayer(stage->GetRootLayer()));out.U32(stage->GetSessionLayer()?AddLayer(stage->GetSessionLayer()):noLayer);
        const auto target=stage->GetEditTarget();if(!target.GetMapFunction().IsIdentity())throw std::runtime_error("unsupported mapped/variant edit target");
        out.U32(target.GetLayer()?AddLayer(target.GetLayer()):noLayer);out.bytes+=Configuration(stage);return out.bytes;
    }
    void Sync(unsigned id) {
        Boundary();auto &entry=stages.at(id);
        if(entry.retired||!entry.stage)CaptureFailure("caller action on expired source stage");
        const UsdStageRefPtr stage(entry.stage);
        // Reconstructing a newly discovered layer after an evaluator exists
        // can introduce notices absent from its original edit history.
        for(const auto &layer:stage->GetUsedLayers())if(!layerIds.count(layer))
            CaptureFailure("newly reachable source layer needs an entry adapter: "+layer->GetIdentifier());
        const auto edit=stage->GetEditTarget().GetLayer();
        if(edit&&!layerIds.count(edit))CaptureFailure("new edit-target layer needs an entry adapter: "+edit->GetIdentifier());
        if(Configuration(stage)!=entry.configuration)
            CaptureFailure("population/load/mute or resolver mutation needs an entry adapter");
        auto state=State(stage);
        if(state!=entry.state){entry.state=state;Writer out;out.U32(id);out.Text(state);Emit(Event::StageState,out.bytes);}
    }
    unsigned AddStage(const UsdStageRefPtr &stage) {
        const auto found=stageIds.find(UsdStageWeakPtr(stage));if(found!=stageIds.end()){Sync(found->second);return found->second;}
        if(!stage)throw std::runtime_error("null input stage");
        Boundary();for(const auto &layer:stage->GetUsedLayers())AddLayer(layer);
        const unsigned id=unsigned(stages.size());auto state=State(stage);stageIds.emplace(UsdStageWeakPtr(stage),id);stages.push_back({stage,state,Configuration(stage)});
        Writer out;out.U32(id);out.Text(state);Emit(Event::Stage,out.bytes);return id;
    }
    void Action(unsigned owner,Event event,Writer payload={}) {
        const auto &record=owners.at(owner);if(record.closed)CaptureFailure("action after evaluator destruction");
        Sync(record.stage);Writer out;out.U32(owner);out.bytes+=payload.bytes;++owners[owner].actions;Emit(event,out.bytes);
    }
    void OperationEvent(const Operation &operation) {
        Check();if(numerical)CaptureFailure("source authoring during numerical evaluator implementation: "+operation.path.GetString());
        if(bulk)return;
        // USD 26.08 annotates _SetData with this official caller scope. An
        // unadapted nonstreaming import otherwise looks like ordinary edits.
        // Only opt-in source authoring reaches this SDK registry accessor.
        for(const auto &scope:TfGetThisThreadScopeDescriptionStack())
            if(scope=="Setting layer data")CaptureFailure("bulk source replacement requires an exact caller-entry adapter");
        try{Emit(Event::Operation,EncodeOperation(operation));++pending;}
        catch(const std::exception &error){CaptureFailure(operation.path.GetString()+" "+operation.field.GetString()+": "+error.what());}
    }
    void OnNotice(const SdfNotice::LayersDidChange &value) {
        bool relevant=false;for(const auto &item:value.GetChangeListVec())if(layerIds.count(item.first))relevant=true;
        if(!relevant)return;
        Check();if(numerical)CaptureFailure("source layer notice during numerical evaluator implementation");
        try{if(!pending)CaptureFailure("source notice without intercepted authoring operations (bulk/custom edit)");
            if(pendingReplacements&&(pendingReplacements!=1||pending!=1))CaptureFailure("known source replacement must be the sole action in its source notice batch");
            Writer out;out.U32(pending);out.U8(pendingReplacements!=0);out.Text(NoticeBytes(value,layerIds,{},pendingReplacements!=0));Emit(Event::Batch,out.bytes);pending=0;pendingReplacements=0;}
        catch(const std::exception &error){CaptureFailure(error.what());}
    }
    void OnStageNotice(const UsdNotice::StageContentsChanged &value) {
        const auto found=stageIds.find(UsdStageWeakPtr(value.GetStage()));if(found==stageIds.end())return;
        Check();
        try {
            const auto &entry=stages[found->second];
            if(Configuration(UsdStageRefPtr(entry.stage))!=entry.configuration)
                CaptureFailure("population/load/mute or resolver mutation needs an entry adapter");
        }catch(const std::exception &error){CaptureFailure(error.what());}
    }
    bool finished=false;
    void Finish() {
        if(finished)return;
        Boundary();if(numerical)CaptureFailure("unclosed numerical Evaluate");
        Writer out;out.U64(events);out.U32(unsigned(layers.size()));out.U32(unsigned(stages.size()));out.U32(unsigned(owners.size()));
        for(const auto &owner:owners){if(!owner.closed)CaptureFailure("unclosed evaluator in input history");out.U64(owner.actions);out.U64(owner.visits);}
        out.U32(unsigned(programs.size()));
        for(const auto &program:programs){if(!program.closed||program.router)CaptureFailure("unclosed held program/router");out.U32(program.owner);out.U64(program.runs);}
        Emit(Event::End,out.bytes);stream.close();if(!stream)CaptureFailure("cannot close input transcript");
        TfNotice::Revoke(notice);notice=TfNotice::Key();
        TfNotice::Revoke(stageNotice);stageNotice=TfNotice::Key();
        for(auto &layer:layers){if(layer.layer)layer.layer->SetStateDelegate(layer.previous);layer.observer=layer.previous;}
        finished=true;
    }
};
void SourceDelegate::Record(Op kind,const SdfPath &path,const TfToken &field,const VtValue &value,
    const TfToken &key,const SdfPath &other,double time,SdfSpecType type,bool inert) {
    recorder->OperationEvent({_id,kind,path,field,key,other,value,time,type,inert});
}
void FinishCapture(){if(recorder)recorder->Finish();}
void Requests(Writer &out,const std::vector<RigExecValueOverride> &requests) {
    out.U32(unsigned(requests.size()));for(const auto &request:requests){out.Text(request.prim.GetString());out.Text(request.computation.GetString());out.Text(request.attribute.GetString());Value(out,request.value);}
}
std::vector<RigExecValueOverride> ReadRequests(Reader &r,const Aliases &aliases) {
    const auto count=r.U32();if(count>r.bytes.size())throw std::runtime_error("invalid input request count");
    std::vector<RigExecValueOverride> requests;requests.reserve(count);
    for(unsigned i=0;i<count;++i){const SdfPath prim(r.Text());const TfToken computation(r.Text()),attribute(r.Text());requests.push_back({prim,computation,attribute,Rebind(ReadValue(r),aliases)});}return requests;
}
}

void RigExecFinalizeInputReplay(){FinishCapture();RigExecFinalizeHeldProgramGolden();RigExecFinalizeCompileGolden();}
void RigExecInputReplayObserver::ArrayAdmission(bool on) {
    if(suppressed)return;
    const char *mode=std::getenv("RIGEXEC_INPUT_REPLAY");
    if(!mode||!*mode)return;
    try {
        const std::string request(mode);
        if(request.rfind("capture:",0)!=0||request.size()==8)throw std::runtime_error("RIGEXEC_INPUT_REPLAY needs capture:<new file>");
        if(!recorder){recorder=std::make_unique<Recorder>(request.substr(8));if(std::atexit(FinishCapture)!=0)throw std::runtime_error("cannot register input history completion");}
        recorder->Boundary();Writer out;out.U8(on);recorder->Emit(Event::ArrayAdmission,out.bytes);
    }catch(const std::exception &error){CaptureFailure(error.what());}
}
std::unique_ptr<RigExecInputReplayObserver> RigExecInputReplayObserver::Create(const UsdStageRefPtr &stage,const SdfPath &rig) {
    const char *mode=std::getenv("RIGEXEC_INPUT_REPLAY");
    if(suppressed)return nullptr;
    if(!mode||!*mode) {
        const auto *compile=std::getenv("RIGEXEC_COMPILE_GOLDEN");
        if(compile&&*compile)CaptureFailure("Compile witness requires a recorded caller identity");
        return nullptr;
    }
    try {
        const std::string request(mode);if(request.rfind("capture:",0)!=0||request.size()==8)throw std::runtime_error("RIGEXEC_INPUT_REPLAY needs capture:<new file>");
        if(!recorder){recorder=std::make_unique<Recorder>(request.substr(8));if(std::atexit(FinishCapture)!=0)throw std::runtime_error("cannot register input history completion");}
        recorder->Boundary();const unsigned stageId=recorder->AddStage(stage),owner=unsigned(recorder->owners.size());
        recorder->owners.push_back({stageId});Writer out;out.U32(stageId);out.U32(owner);out.Text(rig.GetString());recorder->Emit(Event::Create,out.bytes);
        return std::unique_ptr<RigExecInputReplayObserver>(new RigExecInputReplayObserver(stageId,owner,rig));
    }catch(const std::exception &error){CaptureFailure(error.what());}
}
RigExecInputReplayObserver::RigExecInputReplayObserver(unsigned stage,unsigned owner,const SdfPath &rig):_stage(stage),_evaluator(owner),_rig(rig){}
void RigExecInputReplayObserver::UnsupportedAction(const char *action) {
    const char *mode=std::getenv("RIGEXEC_INPUT_REPLAY");
    if(!mode||!*mode||suppressed||implementationDepth||(recorder&&recorder->numerical))return;
    CaptureFailure(std::string("unsupported unrecorded public entry: ")+action);
}
RigExecInputReplayObserver::~RigExecInputReplayObserver(){try{if(_compilePending)CaptureFailure("evaluator destroyed during Compile");if(_evaluating)CaptureFailure("evaluator destroyed during Evaluate");for(const auto &program:recorder->programs)if(program.owner==_evaluator&&!program.closed)CaptureFailure("evaluator destroyed before held program");recorder->Action(_evaluator,Event::Destroy);recorder->owners[_evaluator].closed=true;}catch(const std::exception &error){CaptureFailure(error.what());}}
RigExecInputReplayCompileCall RigExecInputReplayObserver::Compile(const std::vector<std::string> *diagnostics) {
    if(_evaluating||suppressed||implementationDepth)return {};
    try {
        if(_compilePending)CaptureFailure("nested caller Compile");
        RigExecInputReplayCompileCall call;call.recorded=true;call.owner=_evaluator;
        call.ordinal=_compileCalls++;call.diagnostics=diagnostics;
        Writer payload;payload.U8(diagnostics!=nullptr);
        if(diagnostics) {
            if(diagnostics->size()>UINT32_MAX)CaptureFailure("Compile seed exceeds wire count");
            payload.U32(uint32_t(diagnostics->size()));
            for(const auto &line:*diagnostics)payload.Text(line);
        }
        recorder->Action(_evaluator,Event::CompileObserved,payload);
        const std::vector<std::string> empty;
        const auto golden=RigExecBeginCompileGolden(_evaluator,_rig,call.ordinal,
            diagnostics!=nullptr,diagnostics?*diagnostics:empty);
        call.goldenEnabled=golden.enabled;_pendingCompile=call;_compilePending=true;
        return call;
    }catch(const std::exception &error){CaptureFailure(error.what());}
}
void RigExecInputReplayObserver::CompileResult(const RigExecInputReplayCompileCall &call,
    bool returned,const std::vector<std::string> *diagnostics) {
    if(!call.recorded)return;
    try {
        if(!_compilePending||call.owner!=_evaluator||call.ordinal!=_pendingCompile.ordinal||
           call.diagnostics!=diagnostics||call.diagnostics!=_pendingCompile.diagnostics||
           call.goldenEnabled!=_pendingCompile.goldenEnabled)
            CaptureFailure("Compile completion token/pointer differs");
        RigExecRecordCompileGolden({call.goldenEnabled,call.owner,call.ordinal},returned,diagnostics);
        _compilePending=false;
    }catch(const std::exception &error){CaptureFailure(error.what());}
}
void RigExecInputReplayObserver::Interactive(const std::vector<RigExecValueOverride> &v){try{Writer out;Requests(out,v);recorder->Action(_evaluator,Event::Interactive,out);}catch(const std::exception &error){CaptureFailure(error.what());}}
void RigExecInputReplayObserver::ClearInteractive(){try{recorder->Action(_evaluator,Event::Clear);}catch(const std::exception &error){CaptureFailure(error.what());}}
void RigExecInputReplayObserver::Upstream(const std::vector<RigExecValueOverride> &v){try{Writer out;Requests(out,v);recorder->Action(_evaluator,Event::Upstream,out);}catch(const std::exception &error){CaptureFailure(error.what());}}
void RigExecInputReplayObserver::Options(bool guides,bool fields){if(_evaluating||suppressed||implementationDepth)return;try{Writer out;out.U8(guides);out.U8(fields);recorder->Action(_evaluator,Event::Options,out);}catch(const std::exception &error){CaptureFailure(error.what());}}
void RigExecInputReplayObserver::BeginEvaluate(UsdTimeCode time){try{if(_evaluating||recorder->numerical)CaptureFailure("nested/concurrent Evaluate unsupported in serial input capture");Writer out;out.U8(time.IsDefault());out.Double(time.IsDefault()?0:time.GetValue());recorder->Action(_evaluator,Event::Evaluate,out);++recorder->owners[_evaluator].visits;_evaluating=true;++recorder->numerical;}catch(const std::exception &error){CaptureFailure(error.what());}}
void RigExecInputReplayObserver::EndEvaluate(){if(!_evaluating)CaptureFailure("unmatched EndEvaluate");recorder->Check();_evaluating=false;--recorder->numerical;}
namespace {
std::map<RigExecBakedProgram *, RigExecInputReplayHeldProgram *> heldPrograms;
unsigned nextHeldProgramId=0;
bool heldProgramThreadSet=false;
std::thread::id heldProgramThread;
const char *ProgramBailName(RigExecBakedBail bail) {
    switch(bail) {
    case RigExecBakedBail::None:return "None";
    case RigExecBakedBail::StageFrames:return "StageFrames";
    case RigExecBakedBail::Step:return "Step";
    case RigExecBakedBail::Publish:return "Publish";
    }
    return "Unknown";
}
}
RigExecInputReplayHeldProgram::RigExecInputReplayHeldProgram(
    RigExecRigEvaluator *owner,unsigned ownerId,unsigned programId,bool recording)
    : _owner(ownerId),_id(programId),_recording(recording) {
    _golden=RigExecHeldProgramGoldenObserver::Create(programId,owner->GetRigPath());
}
std::unique_ptr<RigExecInputReplayHeldProgram>
RigExecInputReplayHeldProgram::Build(RigExecRigEvaluator *owner,
                                      std::vector<std::string> *reasons) {
    if(!owner)throw std::runtime_error("null held-program owner");
    if(reasons&&!reasons->empty())throw std::runtime_error("held-program adapter requires initially empty reasons output");
    auto *observer=owner->_inputReplayObserver.get();
    const bool recording=bool(observer);
    unsigned ownerId=0,id=0;
    const auto *goldenMode=std::getenv("RIGEXEC_GOLDEN_SUITE");
    if(recording||(goldenMode&&*goldenMode)) {
        if(heldProgramThreadSet&&heldProgramThread!=std::this_thread::get_id())
            CaptureFailure("held program observation requires serial owning-thread calls");
        heldProgramThread=std::this_thread::get_id();heldProgramThreadSet=true;
        id=nextHeldProgramId++;
    }
    if(recording) {
        recorder->Boundary();ownerId=observer->_evaluator;
        observer->Options(owner->GetSolverGuidesEnabled(),owner->GetPublishWeightFields());
        if(id!=recorder->programs.size())CaptureFailure("held program capture/golden identity differs");
        recorder->programs.push_back({ownerId});
        Writer payload;payload.U32(id);payload.U8(reasons!=nullptr);recorder->Action(ownerId,Event::ProgramBuild,payload);
    }
    auto result=std::unique_ptr<RigExecInputReplayHeldProgram>(
        new RigExecInputReplayHeldProgram(owner,ownerId,id,recording));
    {RigExecInputReplayImplementationScope scope(recording);
     result->_program=RigExecBakedProgram::Build(owner,reasons);}
    if(result->_golden)result->_golden->RecordBuild(bool(result->_program), reasons ? *reasons : std::vector<std::string>());
    if(result->_program)heldPrograms.emplace(result->_program.get(),result.get());
    return result;
}
RigExecInputReplayHeldProgram::~RigExecInputReplayHeldProgram() {
    if(_router)CaptureFailure("held program destroyed before caller router");
    if(_recording) {Writer payload;payload.U32(_id);
        recorder->Action(_owner,Event::ProgramDestroy,payload);
        recorder->programs.at(_id).closed=true;}
    heldPrograms.erase(_program.get());
    _program.reset();_golden.reset();
}
bool RigExecInputReplayHeldProgram::Run(UsdTimeCode time,RigExecRigPose *pose,
                                        bool requestedTimeSeed) {
    if(!pose||!_program)throw std::runtime_error("null held-program output/program");
    *pose=RigExecRigPose();if(requestedTimeSeed)pose->time=time;
    if(_recording) {Writer payload;payload.U32(_id);payload.U8(time.IsDefault());
        payload.Double(time.IsDefault()?0:time.GetValue());payload.U8(requestedTimeSeed);
        recorder->Action(_owner,Event::ProgramRun,payload);++recorder->programs.at(_id).runs;}
    bool returned;
    {RigExecInputReplayImplementationScope scope(_recording);returned=_program->Run(time,pose);}
    if(_golden)_golden->RecordRun(time,*pose,returned,ProgramBailName(_program->GetLastBail()));
    return returned;
}
void RigExecInputReplayHeldProgram::Router(RigExecBakedProgram *program,bool enabled) {
    const auto found=heldPrograms.find(program);if(found==heldPrograms.end())return;
    auto &held=*found->second;
    if(held._router==enabled)CaptureFailure("duplicate held-program router transition");
    if(held._recording) {Writer payload;payload.U32(held._id);payload.U8(enabled);
        recorder->Action(held._owner,Event::ProgramRouter,payload);
        recorder->programs.at(held._id).router=enabled;}
    held._router=enabled;
}
RigExecInputReplaySuppression::RigExecInputReplaySuppression(){++suppressed;}
RigExecInputReplaySuppression::~RigExecInputReplaySuppression(){--suppressed;}
RigExecInputReplayImplementationScope::RigExecInputReplayImplementationScope(bool enabled):_enabled(enabled){if(_enabled){recorder->Check();++implementationDepth;++recorder->numerical;}}
RigExecInputReplayImplementationScope::~RigExecInputReplayImplementationScope(){if(_enabled){--recorder->numerical;--implementationDepth;}}
RigExecInputReplayComparisonScope::RigExecInputReplayComparisonScope(const char *action,bool equivalent) {
    const char *mode=std::getenv("RIGEXEC_INPUT_REPLAY");
    if(!mode||!*mode||suppressed||implementationDepth||(recorder&&recorder->numerical))return;
    const char *native=std::getenv("RIGEXEC_INPUT_REPLAY_NATIVE_HISTORIES");
    if(!native||std::string(native)!="1")RigExecInputReplayObserver::UnsupportedAction(action);
    if(!recorder)CaptureFailure("comparison scope has no captured native evaluator");
    try {
        recorder->Boundary();_ordinal=recorder->comparisons++;
        Writer out;out.U64(_ordinal);out.Text(action);out.U8(equivalent);
        recorder->Emit(Event::ComparisonBegin,out.bytes);++recorder->numerical;_enabled=true;
    }catch(const std::exception &error){CaptureFailure(error.what());}
}
RigExecInputReplayComparisonScope::~RigExecInputReplayComparisonScope(){if(_enabled){
    recorder->Check();--recorder->numerical;Writer out;out.U64(_ordinal);recorder->Emit(Event::ComparisonEnd,out.bytes);
}}
bool RigExecInputReplayImportFromString(const SdfLayerHandle &layer,const std::string &text) {
    if(!recorder||!recorder->layerIds.count(layer))return layer->ImportFromString(text);
    try {
        recorder->Boundary();
        recorder->OperationEvent({recorder->layerIds.at(layer),Op::Import,SdfPath::AbsoluteRootPath(),{}, {}, {},VtValue(text)});
        ++recorder->pendingReplacements;++recorder->bulk;
        const bool result=layer->ImportFromString(text);--recorder->bulk;
        if(!result)CaptureFailure("authored ImportFromString request failed; transcript cannot claim completion");
        return result;
    }catch(const std::exception &error){CaptureFailure(error.what());}
}

void RigExecInputReplayClearLayer(const SdfLayerHandle &layer) {
    if(!recorder||!recorder->layerIds.count(layer)){layer->Clear();return;}
    try {
        recorder->Boundary();
        recorder->OperationEvent({recorder->layerIds.at(layer),Op::Clear,SdfPath::AbsoluteRootPath()});
        ++recorder->pendingReplacements;++recorder->bulk;layer->Clear();--recorder->bulk;
    }catch(const std::exception &error){CaptureFailure(error.what());}
}
void RigExecInputReplayTransferLayerContent(const SdfLayerHandle &target,const SdfLayerHandle &source) {
    if(!recorder||!recorder->layerIds.count(target)){target->TransferContent(source);return;}
    try {
        recorder->Boundary();const unsigned sourceId=recorder->AddLayer(source);
        recorder->OperationEvent({recorder->layerIds.at(target),Op::Transfer,SdfPath::AbsoluteRootPath(),{}, {}, {},VtValue(sourceId)});
        ++recorder->pendingReplacements;++recorder->bulk;target->TransferContent(source);--recorder->bulk;
    }catch(const std::exception &error){CaptureFailure(error.what());}
}

namespace {
class ReplayAudit:public TfWeakBase {
public:
    Layers ids;Aliases reverse;bool enabled=false,numerical=false,allowReplace=false;std::vector<std::string> notices;TfNotice::Key key;
    ReplayAudit(){key=TfNotice::Register(TfCreateWeakPtr(this),&ReplayAudit::OnNotice);}
    ~ReplayAudit(){TfNotice::Revoke(key);}
    void OnNotice(const SdfNotice::LayersDidChange &notice){if(enabled||numerical){bool relevant=false;for(const auto &v:notice.GetChangeListVec())if(ids.count(v.first))relevant=true;
        if(relevant&&numerical)throw std::runtime_error("source authoring during replayed numerical execution");
        if(relevant)notices.push_back(NoticeBytes(notice,ids,reverse,allowReplace));}}
};
struct NumericalReplayScope {ReplayAudit &audit;explicit NumericalReplayScope(ReplayAudit &a):audit(a){audit.numerical=true;}~NumericalReplayScope(){audit.numerical=false;}};
struct StageState {unsigned root,session,edit;ArResolverContext context;UsdStagePopulationMask mask;UsdStageLoadRules rules;std::vector<std::string> muted;};
StageState ReadStageState(const std::string &bytes) {
    Reader r(bytes);StageState s;s.root=r.U32();s.session=r.U32();s.edit=r.U32();const auto hasContext=r.U8();if(hasContext>1)throw std::runtime_error("invalid resolver tag");
    std::vector<std::string> paths;const auto n=r.U32();for(unsigned i=0;i<n;++i)paths.push_back(r.Text());if(!hasContext&&!paths.empty())throw std::runtime_error("invalid empty resolver context");
    if(hasContext)s.context=ArResolverContext(ArDefaultResolverContext(paths));
    const auto masks=r.U32();for(unsigned i=0;i<masks;++i)s.mask.Add(SdfPath(r.Text()));
    std::vector<std::pair<SdfPath,UsdStageLoadRules::Rule>> rules;const auto nr=r.U32();
    for(unsigned i=0;i<nr;++i){SdfPath path(r.Text());const auto rule=r.U32();if(rule>UsdStageLoadRules::NoneRule)throw std::runtime_error("invalid load rule");rules.emplace_back(path,UsdStageLoadRules::Rule(rule));}
    s.rules.SetRules(rules);const auto muted=r.U32();for(unsigned i=0;i<muted;++i)s.muted.push_back(r.Text());r.Finish();return s;
}
void SetStageState(const UsdStageRefPtr &stage,const StageState &s,const std::vector<SdfLayerRefPtr> &layers,const Aliases &aliases) {
    if(stage->GetRootLayer()!=layers.at(s.root)||(s.session==noLayer?bool(stage->GetSessionLayer()):stage->GetSessionLayer()!=layers.at(s.session)))throw std::runtime_error("stage layer identity changed");
    if(stage->GetPathResolverContext()!=s.context)throw std::runtime_error("resolver context mutation unsupported");
    if(stage->GetPopulationMask()!=s.mask)stage->SetPopulationMask(s.mask);
    if(stage->GetLoadRules()!=s.rules)stage->SetLoadRules(s.rules);
    const auto old=stage->GetMutedLayers();std::vector<std::string> desired;
    for(const auto &path:s.muted)desired.push_back(Alias(path,aliases));
    for(const auto &path:old)if(std::find(desired.begin(),desired.end(),path)==desired.end())stage->UnmuteLayer(path);
    for(const auto &path:desired)if(std::find(old.begin(),old.end(),path)==old.end())stage->MuteLayer(path);
    if(s.edit==noLayer)throw std::runtime_error("null edit target unsupported");
    if(stage->GetEditTarget().GetLayer()!=layers.at(s.edit))stage->SetEditTarget(layers.at(s.edit));
}
struct InitialSpec {SdfPath path;SdfSpecType type;std::vector<std::pair<TfToken,VtValue>> fields;};
}
namespace {
class HeldReplayRouter : public TfWeakBase {
public:
    HeldReplayRouter(const UsdStageRefPtr &stage,RigExecBakedProgram *program)
        : _program(program) {
        _key=TfNotice::Register(TfCreateWeakPtr(this),&HeldReplayRouter::Changed,stage);
    }
    ~HeldReplayRouter(){TfNotice::Revoke(_key);}
private:
    void Changed(const UsdNotice::ObjectsChanged &notice,const UsdStageWeakPtr &) {
        if(!_program->ApplyValueEdits(notice))_program->BumpProgramStamp();
    }
    RigExecBakedProgram *_program;TfNotice::Key _key;
};
struct HeldReplayProgram {
    unsigned owner;bool closed=false;size_t runs=0;
    std::unique_ptr<RigExecBakedProgram> program;
    std::unique_ptr<RigExecHeldProgramGoldenObserver> golden;
    std::unique_ptr<HeldReplayRouter> router;
};
}
bool RigExecReplayInputActions(const std::string &file,std::string *error) {
    try {
        RigExecInputReplaySuppression replayCaptureSuppression;
        struct ArrayAdmissionRestore {
            bool saved=RigExecUpstreamArrayAdmission();
            ~ArrayAdmissionRestore(){RigExecSetUpstreamArrayAdmissionForTesting(saved);}
        } arrayAdmissionRestore;
        if(const auto *capture=std::getenv("RIGEXEC_INPUT_REPLAY"))if(*capture)throw std::runtime_error("input capture must be disabled in replay host");
        const auto bytes=ReadFile(file);
        const bool controls=bytes.compare(0,sizeof(magic)-1,magic)==0;
        const bool lifetimes=controls||bytes.compare(0,sizeof(lifetimeMagic)-1,lifetimeMagic)==0;
        const bool extended=lifetimes||bytes.compare(0,sizeof(heldMagic)-1,heldMagic)==0;
        if(!extended&&bytes.compare(0,sizeof(legacyMagic)-1,legacyMagic)!=0)throw std::runtime_error("unknown input action format");
        const auto body=bytes.substr(lifetimes?sizeof(magic)-1:extended?sizeof(heldMagic)-1:sizeof(legacyMagic)-1);Reader input(body);
        const auto provenancePath=input.Text(),provenance=input.Text();if(provenance.empty()||ReadFile(provenancePath)!=provenance)throw std::runtime_error("source/plugin provenance changed");
        std::vector<SdfLayerRefPtr> layers;std::vector<bool> permissions;std::vector<UsdStageRefPtr> stages;
        std::vector<std::string> layerIdentifiers;
        std::vector<bool> retiredLayers,retiredStages;
        std::vector<std::vector<InitialSpec>> initialSpecs;std::vector<bool> initialized;
        std::vector<std::unique_ptr<RigExecRigEvaluator>> owners;std::vector<size_t> actions,visits,compileCalls;
        std::vector<unsigned> ownerStages;std::vector<HeldReplayProgram> programs;
        std::vector<Operation> pending;Aliases aliases;ReplayAudit audit;uint64_t sequence=0,nextComparison=0,openComparison=0;bool end=false,comparisonOpen=false;
        size_t compileRefusals=0,invalidVisits=0;
        auto initializeLayers=[&] {
                for(size_t i=0;i<layers.size();++i)if(!initialized[i]&&!retiredLayers[i]) {
                    layers[i]->SetPermissionToEdit(true);
                    {SdfChangeBlock block;for(const auto &spec:initialSpecs[i])for(const auto &field:spec.fields)
                        layers[i]->SetField(spec.path,field.first,RebindField(field.first,field.second,aliases));}
                    for(const auto &spec:initialSpecs[i]) {
                        if(layers[i]->GetSpecType(spec.path)!=spec.type||layers[i]->ListFields(spec.path).size()!=spec.fields.size())
                            throw std::runtime_error("initial source spec/type/presence differs: "+spec.path.GetString());
                        for(const auto &field:spec.fields)if(ValueBytes(layers[i]->GetField(spec.path,field.first))!=ValueBytes(RebindField(field.first,field.second,aliases)))
                            throw std::runtime_error("initial source typed bits differ: "+spec.path.GetString()+" "+field.first.GetString());
                    }
                    layers[i]->SetPermissionToEdit(permissions[i]);initialized[i]=true;
                }
        };
        while(!input.Done()) {
            const auto ordinal=input.U64();if(ordinal!=sequence++)throw std::runtime_error("missing/reordered input action");
            const Event event=Event(input.U8());const auto payload=input.Text();Reader r(payload);
            if(controls&&ordinal==0&&event!=Event::ArrayAdmission)throw std::runtime_error("missing initial array admission state");
            if(end)throw std::runtime_error("extra event after complete input index");
            if(comparisonOpen&&event!=Event::ComparisonEnd)throw std::runtime_error("caller/source action inside a comparison scope");
            if(!pending.empty()&&event!=Event::Operation&&event!=Event::Batch)throw std::runtime_error("unclosed source notice batch before caller action");
            if(event==Event::ArrayAdmission) {
                if(!controls)throw std::runtime_error("array admission action requires input format 5");
                const auto on=r.U8();if(on>1)throw std::runtime_error("invalid array admission state");
                r.Finish();RigExecSetUpstreamArrayAdmissionForTesting(on!=0);
            } else if(event==Event::Layer) {
                const auto id=r.U32();if(id!=layers.size())throw std::runtime_error("duplicate/missing initial layer");
                const auto identifier=r.Text();const auto anonymous=r.U8(),permission=r.U8();if(anonymous>1||permission>1)throw std::runtime_error("invalid layer flags");
                const auto real=r.Text(),source=r.Text();if(!real.empty()&&ReadFile(real)!=source)throw std::runtime_error("source layer bytes changed: "+real);
                auto layer=anonymous?SdfLayer::CreateAnonymous(".usda"):SdfLayer::FindOrOpen(identifier);
                if(!layer)throw std::runtime_error("cannot reconstruct source layer: "+identifier);
                if(audit.ids.count(layer))throw std::runtime_error("initial layers alias unexpectedly");
                aliases[identifier]=layer->GetIdentifier();audit.reverse[layer->GetIdentifier()]=identifier;
                audit.ids[layer]=id;layers.push_back(layer);permissions.push_back(bool(permission));
                layerIdentifiers.push_back(identifier);retiredLayers.push_back(false);
                layer->SetPermissionToEdit(true);layer->Clear();const auto count=r.U32();
                std::vector<InitialSpec> specs;
                for(unsigned i=0;i<count;++i){InitialSpec spec;spec.path=SdfPath(r.Text());spec.type=SdfSpecType(r.U32());const auto nf=r.U32();for(unsigned f=0;f<nf;++f){const TfToken key(r.Text());spec.fields.emplace_back(key,ReadValue(r));}specs.push_back(std::move(spec));}
                {SdfChangeBlock block;for(const auto &spec:specs)if(spec.path!=SdfPath::AbsoluteRootPath())layer->GetStateDelegate()->CreateSpec(spec.path,spec.type,false);
                    for(const auto &spec:specs)for(const auto &field:spec.fields)layer->SetField(spec.path,field.first,RebindField(field.first,field.second,aliases));}
                layer->SetPermissionToEdit(bool(permission));initialSpecs.push_back(std::move(specs));initialized.push_back(false);
            } else if(event==Event::Stage||event==Event::StageState) {
                const auto id=r.U32();const auto state=ReadStageState(r.Text());
                for(const auto layer:{state.root,state.session,state.edit})
                    if(layer!=noLayer&&(retiredLayers.at(layer)||!layers.at(layer)))
                        throw std::runtime_error("stage references retired source layer");
                // Forward anonymous-layer references are bound after every
                // initial layer has an identity, before any evaluator exists.
                // Already initialized layers retain their actual edit history.
                initializeLayers();
                if(event==Event::Stage) {if(id!=stages.size())throw std::runtime_error("duplicate/missing stage");
                    const auto session=state.session==noLayer?SdfLayerHandle():SdfLayerHandle(layers.at(state.session));
                    auto stage=UsdStage::Open(layers.at(state.root),session,state.context,UsdStage::LoadNone);if(!stage)throw std::runtime_error("initial source stage failed");stages.push_back(stage);retiredStages.push_back(false);}
                if(retiredStages.at(id)||!stages.at(id))throw std::runtime_error("state action on retired source stage");
                audit.enabled=false;SetStageState(stages.at(id),state,layers,aliases);
            } else if(event==Event::StageRetire) {
                if(!lifetimes)throw std::runtime_error("source lifetime event in legacy transcript");
                const auto id=r.U32();
                if(retiredStages.at(id)||!stages.at(id))throw std::runtime_error("duplicate source stage retirement");
                for(size_t owner=0;owner<owners.size();++owner)
                    if(ownerStages[owner]==id&&owners[owner])throw std::runtime_error("source stage retirement with a live evaluator");
                stages[id].Reset();retiredStages[id]=true;
            } else if(event==Event::LayerRetire) {
                if(!lifetimes)throw std::runtime_error("source lifetime event in legacy transcript");
                const auto id=r.U32();auto &layer=layers.at(id);
                if(retiredLayers.at(id)||!layer)throw std::runtime_error("duplicate source layer retirement");
                for(const auto &stage:stages)if(stage) {
                    const auto used=stage->GetUsedLayers();
                    if(stage->GetRootLayer()==layer||stage->GetSessionLayer()==layer||stage->GetEditTarget().GetLayer()==layer||
                       std::find(used.begin(),used.end(),layer)!=used.end())
                        throw std::runtime_error("source layer retirement while a live stage retains it");
                }
                const std::string liveIdentifier=layer->GetIdentifier();
                audit.ids.erase(SdfLayerHandle(layer));audit.reverse.erase(liveIdentifier);
                const auto alias=aliases.find(layerIdentifiers[id]);
                if(alias!=aliases.end()&&alias->second==liveIdentifier)aliases.erase(alias);
                layer.Reset();retiredLayers[id]=true;initialized[id]=true;
            } else if(event==Event::Create) {
                const auto stage=r.U32(),owner=r.U32();const SdfPath rig(r.Text());if(owner!=owners.size())throw std::runtime_error("duplicate/missing evaluator");
                if(retiredStages.at(stage)||!stages.at(stage))throw std::runtime_error("evaluator creation on retired source stage");
                owners.emplace_back(new RigExecRigEvaluator(stages.at(stage),rig));actions.push_back(0);visits.push_back(0);compileCalls.push_back(0);ownerStages.push_back(stage);
            } else if(event==Event::Operation) {
                auto operation=DecodeOperation(r);
                if(retiredLayers.at(operation.layer)||!layers.at(operation.layer))throw std::runtime_error("source operation on retired layer");
                if(operation.kind==Op::Transfer&&
                   (retiredLayers.at(operation.value.Get<unsigned>())||!layers.at(operation.value.Get<unsigned>())))
                    throw std::runtime_error("source transfer from retired layer");
                pending.push_back(std::move(operation));
            }
            else if(event==Event::Batch) {
                const auto count=r.U32();const auto replace=r.U8();if(replace>1)throw std::runtime_error("invalid bulk adapter tag");
                const bool hasReplacement=std::any_of(pending.begin(),pending.end(),[](const auto &v){return v.kind==Op::Import||v.kind==Op::Clear||v.kind==Op::Transfer;});
                const auto recorded=r.Text();const auto expected=replace?NormalizeImportNotice(recorded):recorded;if(count!=pending.size()||!count||bool(replace)!=hasReplacement||(replace&&count!=1))throw std::runtime_error("source batch operation count/adapter differs");
                initializeLayers();audit.notices.clear();audit.enabled=true;audit.allowReplace=bool(replace);
                {SdfChangeBlock block;for(const auto &operation:pending)ApplyOperation(operation,layers.at(operation.layer),aliases,layers);}
                audit.enabled=false;if(audit.notices.size()!=1||audit.notices.front()!=expected)throw std::runtime_error("source notice batch audit differs at event "+std::to_string(ordinal)+NoticeDifference(expected,audit.notices));pending.clear();
            } else if(event==Event::ComparisonBegin) {
                openComparison=r.U64();if(openComparison!=nextComparison++)throw std::runtime_error("missing/reordered comparison scope");
                if(r.Text().empty()||r.U8()>1)throw std::runtime_error("invalid named comparison scope");comparisonOpen=true;
            } else if(event==Event::ComparisonEnd) {
                if(!comparisonOpen||r.U64()!=openComparison)throw std::runtime_error("unmatched comparison scope end");comparisonOpen=false;
            } else if(event==Event::End) {
                if(r.U64()!=ordinal||r.U32()!=layers.size()||r.U32()!=stages.size()||r.U32()!=owners.size())throw std::runtime_error("input inventory total differs");
                for(size_t i=0;i<owners.size();++i)if(r.U64()!=actions[i]||r.U64()!=visits[i]||owners[i])throw std::runtime_error("unclosed evaluator/action/visit count mismatch");
                if(extended){if(r.U32()!=programs.size())throw std::runtime_error("held program inventory differs");
                    for(const auto &program:programs)if(r.U32()!=program.owner||r.U64()!=program.runs||!program.closed||program.router||program.program)throw std::runtime_error("unclosed held program/run index differs");}
                end=true;
            } else {
                const auto owner=r.U32();auto &ptr=owners.at(owner);if(!ptr)throw std::runtime_error("caller action after evaluator close");++actions[owner];
                switch(event) {
                case Event::Compile:{std::vector<std::string> diagnostics;
                    NumericalReplayScope scope(audit);
                    if(!ptr->Compile(&diagnostics)){++compileRefusals;std::fprintf(stderr,"input replay Compile refused at event %llu\n",static_cast<unsigned long long>(ordinal));
                        for(const auto &line:diagnostics)std::fprintf(stderr,"  %s\n",line.c_str());}break;}
                case Event::CompileObserved:{
                    if(!lifetimes)throw std::runtime_error("Compile witness needs input action version 4");
                    const auto requested=r.U8();if(requested>1)throw std::runtime_error("invalid Compile diagnostics flag");
                    std::vector<std::string> diagnostics;
                    if(requested){const auto count=r.U32();for(uint32_t i=0;i<count;++i)diagnostics.push_back(r.Text());}
                    r.Finish();
                    const auto call=compileCalls.at(owner)++;
                    const auto golden=RigExecBeginCompileGolden(owner,ptr->GetRigPath(),call,
                        requested!=0,diagnostics);
                    bool returned;
                    {NumericalReplayScope scope(audit);returned=ptr->Compile(requested?&diagnostics:nullptr);}
                    RigExecRecordCompileGolden(golden,returned,requested?&diagnostics:nullptr);
                    if(!returned){++compileRefusals;std::fprintf(stderr,"input replay Compile refused at event %llu\n",static_cast<unsigned long long>(ordinal));
                        if(requested)for(const auto &line:diagnostics)std::fprintf(stderr,"  %s\n",line.c_str());}
                    break;}
                case Event::Options:{const auto guides=r.U8(),fields=r.U8();if(guides>1||fields>1)throw std::runtime_error("invalid publication option");ptr->SetSolverGuidesEnabled(bool(guides));ptr->SetPublishWeightFields(bool(fields));break;}
                case Event::Interactive:ptr->SetInteractiveOverrides(ReadRequests(r,aliases));break;
                case Event::Clear:ptr->ClearInteractiveOverrides();break;
                case Event::Upstream:ptr->SetUpstreamInputs(ReadRequests(r,aliases));break;
                case Event::Evaluate:{const auto atDefault=r.U8();const auto time=r.Double();if(atDefault>1)throw std::runtime_error("invalid time identity");
                    NumericalReplayScope scope(audit);
                    const auto pose=ptr->Evaluate(atDefault?UsdTimeCode::Default():UsdTimeCode(time));
                    if(!pose.valid){++invalidVisits;std::fprintf(stderr,"input replay invalid visit at event %llu\n",static_cast<unsigned long long>(ordinal));}
                    ++visits[owner];break;}
                case Event::ProgramBuild:{
                    if(!extended)throw std::runtime_error("held event in legacy transcript");
                    const auto id=r.U32();const auto reasonsRequested=r.U8();
                    if(id!=programs.size())throw std::runtime_error("duplicate/missing held program");
                    if(reasonsRequested>1)throw std::runtime_error("invalid held Build reasons request");
                    HeldReplayProgram program;program.owner=owner;
                    program.golden=RigExecHeldProgramGoldenObserver::Create(id,ptr->GetRigPath());
                    std::vector<std::string> reasons;
                    {NumericalReplayScope scope(audit);program.program=RigExecBakedProgram::Build(ptr.get(),reasonsRequested?&reasons:nullptr);}
                    if(program.golden)program.golden->RecordBuild(bool(program.program),reasons);
                    programs.push_back(std::move(program));break;}
                case Event::ProgramRun:case Event::ProgramRouter:case Event::ProgramDestroy:{
                    if(!extended)throw std::runtime_error("held event in legacy transcript");
                    const auto id=r.U32();auto &program=programs.at(id);
                    if(program.owner!=owner||program.closed)throw std::runtime_error("held program owner/lifetime differs");
                    if(event==Event::ProgramDestroy){
                        if(program.router)throw std::runtime_error("held program destroyed with active router");
                        program.program.reset();program.golden.reset();program.closed=true;break;}
                    if(!program.program)throw std::runtime_error("held action after refused Build");
                    if(event==Event::ProgramRouter){const auto enabled=r.U8();
                        if(enabled>1||bool(program.router)==bool(enabled))throw std::runtime_error("invalid/duplicate held router transition");
                        if(enabled)program.router=std::make_unique<HeldReplayRouter>(stages.at(ownerStages.at(owner)),program.program.get());
                        else program.router.reset();break;}
                    const auto atDefault=r.U8();const auto numericTime=r.Double();const auto seed=r.U8();
                    if(atDefault>1||seed>1)throw std::runtime_error("invalid held run time/seed");
                    const auto time=atDefault?UsdTimeCode::Default():UsdTimeCode(numericTime);
                    RigExecRigPose pose;if(seed)pose.time=time;bool returned;
                    {NumericalReplayScope scope(audit);returned=program.program->Run(time,&pose);}
                    if(program.golden)program.golden->RecordRun(time,pose,returned,ProgramBailName(program.program->GetLastBail()));
                    ++program.runs;break;}
                case Event::Destroy:
                    for(const auto &program:programs)if(program.owner==owner&&!program.closed)throw std::runtime_error("owner destroyed before held program");
                    ptr.reset();break;
                default:throw std::runtime_error("unknown input action event");
                }
            }
            r.Finish();
        }
        if(!end||!pending.empty()||comparisonOpen)throw std::runtime_error("incomplete input transcript (missing strict end index)");
        if(compileRefusals||invalidVisits) {
            if(error)*error="input chronology complete, but numerical execution had "+std::to_string(compileRefusals)+
                " Compile refusals and "+std::to_string(invalidVisits)+" invalid visits; preserve original captures and report separately";
            return false;
        }
        return true;
    }catch(const std::exception &failure){if(error)*error=failure.what();return false;}
}
}

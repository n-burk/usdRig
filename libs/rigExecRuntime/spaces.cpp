// Portable provider kernels share the native arithmetic definition.
#include "spaces.h"
#include "poseInternal.h"
#include <algorithm>
#include "rigExecGraph/providerArithmetic.h"
#include "rigExecGraph/providerRefresh.h"
#include <cstring>
#include <type_traits>
namespace rigExec {
namespace {
struct Math {
    using Matrix=RrMat4d; using Vector=RrVec3d;
    using Rotation=RrRotation; using Frame=RrPointFrame;
    static bool PointsToMatrix(const std::array<Vector,4> &a,
                              const std::array<Vector,4> &b,Matrix *out) {
        return RrPointsToMatrix(a,b,out);
    }
};
struct Store {
    std::vector<RigExecProviderPlainState> &values;
    const std::vector<uint64_t> *inputs = nullptr;
    RrProviderConversionScratch *scratch = nullptr;
    explicit Store(std::vector<RigExecProviderPlainState> &v):values(v) {}
    Store(std::vector<RigExecProviderPlainState> &v,
          const std::vector<uint64_t> &ids,RrProviderConversionScratch &converted)
        :values(v),inputs(&ids),scratch(&converted) {}
    size_t ConversionIndex(uint64_t id) const {
        if (!inputs) return size_t(-1);
        const auto found = std::find(inputs->begin(),inputs->end(),id);
        return found == inputs->end() ? size_t(-1) : size_t(found-inputs->begin());
    }
    bool Present(uint64_t id) const {
        return id<values.size() && values[id].initialized;
    }
    bool Authoritative(uint64_t id) const {
        return id<values.size() && values[id].authoritative;
    }
    const RrMat4d *ReadMatrix(uint64_t id) {
        if(!Present(id) || values[id].blocked) return nullptr;
        const auto *v=std::get_if<std::array<double,16>>(&values[id].value);
        if(!v) return nullptr;
        const size_t index=ConversionIndex(id);
        if(index==size_t(-1) || !scratch) return nullptr;
        std::memcpy(scratch->matrices[index]._mtx,v->data(),sizeof(double)*16);
        return &scratch->matrices[index];
    }
    const RrPointFrame *ReadFrame(uint64_t id) {
        if(!Present(id) || values[id].blocked) return nullptr;
        const auto *v=std::get_if<RigExecProviderPlainFrame>(&values[id].value);
        if(!v) return nullptr;
        const size_t index=ConversionIndex(id);
        if(index==size_t(-1) || !scratch) return nullptr;
        for(size_t p=0;p<4;++p) for(size_t a=0;a<3;++a)
            scratch->frames[index].points[p][a]=v->points[p][a];
        scratch->frames[index].flags=v->flags;
        return &scratch->frames[index];
    }
    const double *ReadScalar(uint64_t id) {
        if(!Present(id) || values[id].blocked) return nullptr;
        const size_t index=ConversionIndex(id);
        if(index==size_t(-1) || !scratch) return nullptr;
        if(const auto *v=std::get_if<double>(&values[id].value)) scratch->scalars[index]=*v;
        else if(const auto *v=std::get_if<float>(&values[id].value)) scratch->scalars[index]=*v;
        else return nullptr;
        return &scratch->scalars[index];
    }
    const RrVec3d *ReadVector(uint64_t id) {
        if(!Present(id) || values[id].blocked) return nullptr;
        const auto *v=std::get_if<std::array<double,3>>(&values[id].value);
        if(!v) return nullptr;
        const size_t index=ConversionIndex(id);
        if(index==size_t(-1) || !scratch) return nullptr;
        for(size_t a=0;a<3;++a) scratch->vectors[index][a]=(*v)[a];
        return &scratch->vectors[index];
    }
    std::string_view ReadToken(uint64_t id) const {
        if(!Present(id) || values[id].blocked) return {};
        const auto *v=std::get_if<std::string>(&values[id].value);
        return v?std::string_view(*v):std::string_view();
    }
    bool ResetXformStack(uint64_t id) const {
        if(!Present(id) || values[id].blocked) return false;
        const auto *tokens=std::get_if<std::vector<std::string>>(&values[id].value);
        return tokens && std::find(tokens->begin(),tokens->end(),"!resetXformStack!")!=tokens->end();
    }
    void Invalidate(uint64_t output,uint64_t source) {
        const bool blocked=source<values.size() && values[source].blocked;
        auto &out=values[output]; out.value=std::monostate{};
        out.initialized=true; out.blocked=blocked; out.authoritative=false;
        out.count=0; out.error="provider input unavailable";
    }
    void Copy(uint64_t to,uint64_t from,bool authority) {
        values[to]=values[from]; values[to].authoritative=authority || values[from].authoritative;
    }
    void PublishMatrix(uint64_t id,const RrMat4d &matrix,bool authority) {
        auto &out=values[id]; std::array<double,16> value;
        std::memcpy(value.data(),matrix._mtx,sizeof(double)*16);
        out.value=value; out.initialized=true; out.blocked=false;
        out.authoritative=authority; out.count=1; out.error.clear();
    }
    void PublishFrame(uint64_t id,const RrPointFrame &frame) {
        RigExecProviderPlainFrame value;
        for(size_t p=0;p<4;++p) for(size_t a=0;a<3;++a)
            value.points[p][a]=frame.points[p][a];
        value.flags=frame.flags;
        auto &out=values[id]; out.value=value; out.initialized=true;
        out.blocked=false; out.authoritative=false; out.count=1; out.error.clear();
    }
};
}
bool RrReadCrossDomain(const RrProgram *program,int32_t index,RrWireValue *out) {
    if(index<0 || size_t(index)>=program->file->crossDomainReads.size()) return false;
    const auto &read=program->file->crossDomainReads[size_t(index)];
    if(read.kind==fb::CrossDomainReadKind::PointElement) {
        RrVec3f point;
        if(!RrReadCrossDomainPoint(program,read,&point)) return false;
        out->tag=RigExecWireInputTag::Vec3f;
        out->vec3f={point[0],point[1],point[2]}; return true;
    }
    if(read.kind==fb::CrossDomainReadKind::PoseFrame) {
        const auto &frames=read.baseFrame?program->store.base:program->store.fin;
        const std::array<RrVec3d,4> identity={RrVec3d(0),RrVec3d(1,0,0),RrVec3d(0,1,0),RrVec3d(0,0,1)};
        for(auto version:read.frames) {
            const auto &frame=frames[version];
            if(!frame.IsValid() || frame.IsDegenerate()) continue;
            RrMat4d matrix(1);
            if(!RrPointsToMatrix(identity,frame,&matrix)) continue;
            out->tag=RigExecWireInputTag::Matrix4d;
            std::memcpy(out->matrix.data(),matrix._mtx,sizeof(double)*16); return true;
        }
        return false;
    }
    if(read.kind==fb::CrossDomainReadKind::PropertyResult) {
        if(!program->store.propertyVersionValid[read.propertyVersion]) return false;
        const auto &v=program->store.propertyVersions[read.propertyVersion];
        switch(v.tag) {
        case RrPropertyValue::Tag::Float: {
            out->tag=RigExecWireInputTag::Float; uint32_t bits;
            std::memcpy(&bits,&v.f32,sizeof(bits)); out->bits=bits; break;
        }
        case RrPropertyValue::Tag::Double:
            out->tag=RigExecWireInputTag::Double;
            std::memcpy(&out->bits,&v.f64,sizeof(v.f64)); break;
        case RrPropertyValue::Tag::Vec3f:
            out->tag=RigExecWireInputTag::Vec3f; out->vec3f={v.vec[0],v.vec[1],v.vec[2]}; break;
        case RrPropertyValue::Tag::Matrix4d:
            out->tag=RigExecWireInputTag::Matrix4d;
            std::memcpy(out->matrix.data(),v.matrix._mtx,sizeof(double)*16); break;
        }
        return true;
    }
    if(read.spaceValue<0) return false;
    const auto &state=program->store.providerValues[size_t(read.spaceValue)];
    if(!state.initialized || state.blocked) return false;
    if (const auto *matrix=std::get_if<std::array<double,16>>(&state.value)) {
        out->tag=RigExecWireInputTag::Matrix4d; out->matrix=*matrix; return true;
    }
    if (const auto *value=std::get_if<double>(&state.value)) {
        out->tag=RigExecWireInputTag::Double; std::memcpy(&out->bits,value,sizeof(double)); return true;
    }
    if (const auto *value=std::get_if<float>(&state.value)) {
        out->tag=RigExecWireInputTag::Float; uint32_t bits; std::memcpy(&bits,value,sizeof(float)); out->bits=bits; return true;
    }
    if (const auto *value=std::get_if<std::array<double,3>>(&state.value)) {
        out->tag=RigExecWireInputTag::Vec3d; out->vec3d=*value; return true;
    }
    if (const auto *value=std::get_if<std::array<float,3>>(&state.value)) {
        out->tag=RigExecWireInputTag::Vec3f; out->vec3f=*value; return true;
    }
    if (const auto *value=std::get_if<std::array<int32_t,3>>(&state.value)) {
        out->tag=RigExecWireInputTag::Vec3i; out->vec3i=*value; return true;
    }
    if (const auto *value=std::get_if<bool>(&state.value)) {
        out->tag=RigExecWireInputTag::Bool; out->bits=*value?1:0; return true;
    }
    if (const auto *value=std::get_if<int32_t>(&state.value)) {
        out->tag=RigExecWireInputTag::Int; out->bits=uint32_t(*value); return true;
    }
    if (const auto *value=std::get_if<std::string>(&state.value)) {
        // Token ownership is established at Open/sampling, never by a consumer.
        const auto found=program->inputState.tokenIds.find(*value);
        if(found==program->inputState.tokenIds.end()) return false;
        out->tag=RigExecWireInputTag::Token; out->bits=found->second; return true;
    }
    if (const auto *value=std::get_if<RigExecProviderPlainFrame>(&state.value)) {
        RrPointFrame frame;
        for(size_t k=0;k<4;++k) frame.points[k]=RrVec3d(value->points[k][0],value->points[k][1],value->points[k][2]);
        frame.flags=value->flags;
        const std::array<RrVec3d,4> identity={RrVec3d(0),RrVec3d(1,0,0),RrVec3d(0,1,0),RrVec3d(0,0,1)};
        RrMat4d matrix(1);
        if(!frame.IsValid() || frame.IsDegenerate() || !RrPointsToMatrix(identity,frame,&matrix)) return false;
        out->tag=RigExecWireInputTag::Matrix4d;
        std::memcpy(out->matrix.data(),matrix._mtx,sizeof(double)*16); return true;
    }
    return false;
}

bool RrOpenProviderProgram(RrProgram *program,std::string *error) {
    if(!program->file || !program->file->providerProgram) return true;
    const auto &wire=*program->file->providerProgram;
    auto &plain=program->providerProgram;
    plain.valueKeys=wire.valueKeys; plain.leaves=wire.leaves;
    const auto internToken=[&](const std::string &text) {
        auto &state=program->inputState;
        if(state.tokenIds.find(text)!=state.tokenIds.end()) return;
        const uint32_t id=state.extraTokenBase+uint32_t(state.extraTokens.size());
        state.extraTokens.push_back(text); state.tokenIds.emplace(text,id);
    };
    for(const auto &row:wire.defaults) {
        RigExecProviderPlainState out;
        out.initialized=row.initialized; out.blocked=row.blocked;
        out.authoritative=row.authoritative; out.count=row.count; out.error=row.error;
        switch(row.kind) {
        case fb::ProviderValueKind::Empty: break;
        case fb::ProviderValueKind::Double: out.value=*row.scalarDouble; break;
        case fb::ProviderValueKind::Float: out.value=*row.scalarFloat; break;
        case fb::ProviderValueKind::Vector: out.value=*row.vector; break;
        case fb::ProviderValueKind::Matrix: out.value=*row.matrix; break;
        case fb::ProviderValueKind::Token: internToken(row.token); out.value=row.token; break;
        case fb::ProviderValueKind::Frame: {
            RigExecProviderPlainFrame frame;
            std::copy(row.framePoints.begin(),row.framePoints.end(),frame.points.begin());
            frame.flags=row.frameFlags; out.value=frame; break;
        }
        case fb::ProviderValueKind::Vec3f: out.value=*row.vec3f; break;
        case fb::ProviderValueKind::Bool: out.value=row.boolean; break;
        case fb::ProviderValueKind::Int: out.value=int32_t(row.integer); break;
        case fb::ProviderValueKind::FloatArray: out.value=row.floats; break;
        case fb::ProviderValueKind::DoubleArray: out.value=row.doubles; break;
        case fb::ProviderValueKind::Vec3fArray: out.value=row.vec3fs; break;
        case fb::ProviderValueKind::Vec3dArray: out.value=row.vec3ds; break;
        case fb::ProviderValueKind::IntArray: out.value=row.ints; break;
        case fb::ProviderValueKind::MatrixArray: out.value=row.matrices; break;
        case fb::ProviderValueKind::TokenArray:
            for(const auto &token:row.tokens) internToken(token);
            out.value=row.tokens; break;
        case fb::ProviderValueKind::BoolArray: {
            std::vector<bool> bits; for(uint8_t bit:row.bools) bits.push_back(bit!=0);
            out.value=std::move(bits); break;
        }
        case fb::ProviderValueKind::Vec2f: out.value=*row.vec2f; break;
        case fb::ProviderValueKind::Vec2fArray: out.value=row.vec2fs; break;
        case fb::ProviderValueKind::Vec3i: out.value=*row.vec3i; break;
        default: if(error)*error="unknown provider value kind"; return false;
        }
        plain.defaults.push_back(std::move(out));
    }
    for(const auto &row:wire.ops) {
        RigExecProviderPlainOp op;
        op.kind=RigExecProviderOpKind(row.kind); op.owner=row.owner;
        op.output=row.output; op.inputs=row.inputs; op.scaleAvars=row.scaleAvars;
        plain.ops.push_back(std::move(op));
    }
    const auto leaves=[](const auto &wire,auto *out) {
        for(const auto &row:wire) out->push_back({row.value,row.path,row.computation});
    };
    leaves(wire.sampled,&plain.sampled); leaves(wire.externalInputs,&plain.externalInputs);
    program->store.providerValues=plain.defaults;
    auto &converted=program->store.providerConversionScratch;
    converted.resize(plain.ops.size());
    for(size_t i=0;i<plain.ops.size();++i) {
        const size_t count=plain.ops[i].inputs.size();
        converted[i].matrices.resize(count);
        converted[i].frames.resize(count);
        converted[i].vectors.resize(count);
        converted[i].scalars.resize(count);
    }
    auto &refresh=program->store.providerRefreshScratch;
    refresh.resize(program->poses->providerRefreshes.size());
    for(size_t i=0;i<refresh.size();++i) {
        const size_t count=program->poses->providerRefreshes[i].carries.size();
        refresh[i].baseInputs.resize(count); refresh[i].finInputs.resize(count);
        refresh[i].baseOutputs.resize(count); refresh[i].finOutputs.resize(count);
        refresh[i].blocked.resize(count);
    }
    return true;
}
bool RrProviderEffectiveInputMemo(const RrProgram *program,const RigExecWireStep &step,
                                  std::string *key,std::vector<uint32_t> *covered) {
    if(step.part==2) {
        const auto &leaf=program->file->providerProgram->sampled[size_t(step.object)];
        if(leaf.inputSlot>=0) {
            RigExecWireInput raw;
            raw.tag=program->file->inputs[size_t(leaf.inputSlot)].type();
            raw.mode=RigExecWireReadMode::Raw; raw.walk={uint32_t(leaf.inputSlot)};
            raw.constant=program->file->inputs[size_t(leaf.inputSlot)].value();
            RrSourceReadMemo(program,raw,key);
        }
        return true;
    }
    if(step.part==4) return true; // Only declared typed frame inputs.
    if(step.part!=0) return false;
    const auto &op=program->providerProgram.ops[size_t(step.object)];
    const auto &values=program->store.providerValues;
    const auto append=[&](const auto &value) { key->append(reinterpret_cast<const char *>(&value),sizeof(value)); };
    const auto text=[&](const std::string &value) { append(uint64_t(value.size())); key->append(value); };
    const auto state=[&](uint64_t id) {
        const bool exists=id<values.size(); append(exists); if(!exists) return;
        const auto &value=values[size_t(id)];
        append(value.initialized); append(value.blocked); append(value.authoritative);
        append(value.count); text(value.error); append(uint32_t(value.value.index()));
        std::visit([&](const auto &payload) {
            using T=std::decay_t<decltype(payload)>;
            if constexpr(std::is_same_v<T,std::monostate>) {}
            else if constexpr(std::is_same_v<T,std::string>) text(payload);
            else if constexpr(std::is_same_v<T,RigExecProviderPlainFrame>) {
                for(const auto &point:payload.points) for(double x:point) append(x);
                append(payload.flags);
            } else if constexpr(std::is_arithmetic_v<T>) append(payload);
            else {
                append(uint64_t(payload.size()));
                for(const auto &item:payload) {
                    using E=std::decay_t<decltype(item)>;
                    if constexpr(std::is_same_v<E,std::string>) text(item);
                    else if constexpr(std::is_same_v<T,std::vector<bool>>) append(bool(item));
                    else append(item);
                }
            }
        },value.value);
    };
    const auto input=[&](size_t index) { return index<op.inputs.size()?op.inputs[index]:UINT64_MAX; };
    const auto matrix=[&](uint64_t id) -> const std::array<double,16> * {
        return id<values.size() && values[size_t(id)].initialized && !values[size_t(id)].blocked?
            std::get_if<std::array<double,16>>(&values[size_t(id)].value):nullptr;
    };
    const auto frame=[&](uint64_t id) {
        return id<values.size() && values[size_t(id)].initialized && !values[size_t(id)].blocked &&
            std::holds_alternative<RigExecProviderPlainFrame>(values[size_t(id)].value);
    };
    if(op.kind==RigExecProviderOpKind::RestFrame) {
        state(input(7));
        if(input(7)!=UINT64_MAX && !frame(input(7))) return true;
        state(input(8));
        if(input(8)!=UINT64_MAX && !matrix(input(8))) return true;
        for(size_t i=0;i<std::min(size_t(7),op.inputs.size());++i) state(op.inputs[i]);
        return true;
    }
    if(op.kind==RigExecProviderOpKind::DefaultSpace || op.kind==RigExecProviderOpKind::JointMatrix) {
        const size_t count=op.kind==RigExecProviderOpKind::DefaultSpace?3:2;
        for(size_t i=0;i<count;++i) {
            state(input(i));
            if(input(i)!=UINT64_MAX && !(i<2?frame(input(i)):bool(matrix(input(i))))) return true;
        }
        for(size_t i=count;i<op.inputs.size();++i) state(op.inputs[i]);
        return true;
    }
    if(op.kind==RigExecProviderOpKind::Attribute) {
        const uint64_t raw=input(0),connected=input(1);
        const bool authority=raw<values.size() && values[size_t(raw)].authoritative;
        append(authority); state(authority || connected==UINT64_MAX?raw:connected);
    } else if(op.kind==RigExecProviderOpKind::SpaceExpression) {
        const uint64_t raw=input(0),connected=input(1),fallback=input(2);
        const auto *authored=matrix(raw); const auto *bound=matrix(connected);
        const bool authority=raw<values.size() && values[size_t(raw)].authoritative;
        bool identity=true;
        if(authored) for(size_t i=0;i<16;++i) if((*authored)[i]!=(i%5==0?1.0:0.0)) identity=false;
        uint8_t selected=authority&&authored?0:bound?1:authored&&!identity?2:3;
        append(selected); state(selected==0||selected==2?raw:selected==1?connected:fallback);
    } else if(op.kind==RigExecProviderOpKind::RelativeXform) {
        for(size_t i=0;i<op.inputs.size();i+=2) {
            state(op.inputs[i]);
            if(!matrix(op.inputs[i])) break;
            if(i+1<op.inputs.size()) {
                const uint64_t order=op.inputs[i+1]; state(order);
                const auto *tokens=order<values.size() && values[size_t(order)].initialized && !values[size_t(order)].blocked?
                    std::get_if<std::vector<std::string>>(&values[size_t(order)].value):nullptr;
                if(tokens && std::find(tokens->begin(),tokens->end(),"!resetXformStack!")!=tokens->end()) break;
            }
        }
    } else if(op.kind==RigExecProviderOpKind::PosedFrame) {
        const uint64_t posed=input(0); const auto *value=matrix(posed);
        bool identity=true;
        if(value) for(size_t i=0;i<16;++i) if((*value)[i]!=(i%5==0?1.0:0.0)) identity=false;
        const bool selected=value && (values[size_t(posed)].authoritative || !identity);
        append(selected);
        if(selected) state(posed);
        else {
            bool missing=false;
            for(size_t i=1;i<=3;++i) {
                state(input(i));
                if(input(i)!=UINT64_MAX && !matrix(input(i))) { missing=true; break; }
            }
            if(!missing) for(size_t i=4;i<op.inputs.size();++i)
                if(op.scaleAvars || i<8 || i>10) state(op.inputs[i]);
        }
    } else if(op.kind==RigExecProviderOpKind::AvarMatrix) {
        for(size_t i=0;i<op.inputs.size();++i)
            if(op.scaleAvars || i<4 || i>6) state(op.inputs[i]);
    } else {
        for(uint64_t id:op.inputs) state(id);
    }
    (void)covered;
    return true;
}
bool RrRunSpaceExpression(RrProgram *program,const RigExecWireStep &step,std::string *error) {
    if (step.part == 2) {
        const auto &leaf = program->file->providerProgram->sampled[size_t(step.object)];
        auto &out = program->store.providerValues[size_t(leaf.value)];
        const int slot = leaf.inputSlot;
        out = RigExecProviderPlainState();
        if (slot >= 0 && program->inputState.slotHasValue[size_t(slot)]) {
            const auto &v = program->inputState.slotCurrent[size_t(slot)];
            switch (v.tag) {
            case RigExecWireInputTag::Double: {
                double d; std::memcpy(&d, &v.bits, sizeof(d)); out.value=d; break;
            }
            case RigExecWireInputTag::Float: {
                uint32_t bits=uint32_t(v.bits); float f;
                std::memcpy(&f, &bits, sizeof(f)); out.value=f; break;
            }
            case RigExecWireInputTag::Matrix4d: out.value=v.matrix; break;
            case RigExecWireInputTag::Vec3d: out.value=v.vec3d; break;
            case RigExecWireInputTag::Vec3i: out.value=v.vec3i; break;
            case RigExecWireInputTag::Vec3f:
                out.value=v.vec3f; break;
            case RigExecWireInputTag::Token: out.value=program->TextOrEmpty(uint32_t(v.bits)); break;
            case RigExecWireInputTag::Bool: out.value=bool(v.bits); break;
            case RigExecWireInputTag::Int: out.value=int32_t(uint32_t(v.bits)); break;
            default: break;
            }
            if(RigExecFormatIsArrayTag(v.tag)) {
                const auto copy=[&](const auto *array) { if(array) out.value=*array; };
                switch(v.tag) {
                case RigExecWireInputTag::IntArray: copy(RrInputArray<int32_t>(program,uint32_t(slot))); break;
                case RigExecWireInputTag::FloatArray: copy(RrInputArray<float>(program,uint32_t(slot))); break;
                case RigExecWireInputTag::DoubleArray: copy(RrInputArray<double>(program,uint32_t(slot))); break;
                case RigExecWireInputTag::Vec3dArray: copy(RrInputArray<RigExecWireVec3d>(program,uint32_t(slot))); break;
                case RigExecWireInputTag::Matrix4dArray: copy(RrInputArray<RigExecWireMatrix4d>(program,uint32_t(slot))); break;
                case RigExecWireInputTag::TokenArray: {
                    const auto *array=RrInputArray<uint32_t>(program,uint32_t(slot));
                    if(array) { std::vector<std::string> text;
                        for(uint32_t id:*array) text.push_back(program->TextOrEmpty(id)); out.value=std::move(text); }
                    break;
                }
                case RigExecWireInputTag::BoolArray: {
                    const auto *array=RrInputArray<uint8_t>(program,uint32_t(slot));
                    if(array) { std::vector<bool> bits; for(uint8_t bit:*array) bits.push_back(bit!=0); out.value=std::move(bits); }
                    break;
                }
                case RigExecWireInputTag::Vec3fArray: {
                    const auto *array=RrInputArray<RrVec3f>(program,uint32_t(slot));
                    if(array) { std::vector<std::array<float,3>> values;
                        for(const auto &point:*array) values.push_back({point[0],point[1],point[2]}); out.value=std::move(values); }
                    break;
                }
                case RigExecWireInputTag::Vec2fArray: {
                    const auto *array=RrInputArray<RrVec2f>(program,uint32_t(slot));
                    if(array) { std::vector<std::array<float,2>> values;
                        for(const auto &point:*array) values.push_back({point[0],point[1]}); out.value=std::move(values); }
                    break;
                }
                default: break;
                }
            }
            out.initialized=!std::holds_alternative<std::monostate>(out.value);
            out.count=out.initialized?1:0;
            if (out.initialized && slot >= 0 &&
                RigExecFormatIsArrayTag(program->file->inputs[size_t(slot)].type())) {
                out.count=std::visit([](const auto &value)->size_t {
                    using T=std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<T,std::vector<float>> ||
                        std::is_same_v<T,std::vector<double>> ||
                        std::is_same_v<T,std::vector<std::array<float,3>>> ||
                        std::is_same_v<T,std::vector<std::array<double,3>>> ||
                        std::is_same_v<T,std::vector<int32_t>> ||
                        std::is_same_v<T,std::vector<std::array<double,16>>> ||
                        std::is_same_v<T,std::vector<std::string>> ||
                        std::is_same_v<T,std::vector<bool>> ||
                        std::is_same_v<T,std::vector<std::array<float,2>>>) return value.size();
                    return 0;
                },out.value);
            }
        }
        // A missing sampled source is a current published monostate, not a
        // never-produced value; native PublishSource also initializes it.
        out.initialized=true;
        out.blocked=slot>=0 && program->inputState.slotBlocked[size_t(slot)]!=0;
        out.authoritative=slot>=0 && program->inputState.slotAuthored[size_t(slot)];
        return true;
    }
    if(step.part==1) {
        const auto &leaf=program->file->providerProgram->externalInputs[size_t(step.object)];
        Store store(program->store.providerValues);
        const int slot=leaf.providerSlot;
        const auto *pose=static_cast<const RrPoseScratch *>(program->pose.get());
        if(leaf.computation=="computeRestFrame")
            store.PublishFrame(leaf.value,slot>=0?pose->restFrames[size_t(slot)]:RrFrameFromMatrix(RrMat4d(1)));
        else if(leaf.computation=="computePointFrame" || leaf.computation=="computeBasePointFrame") {
            // Initial provider composition reads its initial base frame.
            // Contextual refreshed versions are explicit part4 bridges.
            store.PublishFrame(leaf.value,slot>=0?program->store.base[size_t(slot)]:RrFrameFromMatrix(RrMat4d(1)));
        }
        else {
            RrMat4d matrix(1);
            if(leaf.interveningRead) {
                const auto value=RrReadInput(program,*leaf.interveningRead);
                std::memcpy(matrix._mtx,value.matrix.data(),sizeof(double)*16);
            }
            store.PublishMatrix(leaf.value,matrix,false);
        }
        return true;
    }
    if(step.part==3) {
        const auto &route=program->file->providerProgram->routedInputs[size_t(step.object)];
        auto &out=program->store.providerValues[size_t(route.value)];
        RrWireValue value;
        out=RigExecProviderPlainState(); out.authoritative=true;
        const auto &cross=program->file->crossDomainReads[size_t(route.crossRead)];
        if(cross.kind==fb::CrossDomainReadKind::Points) {
            if(const auto *points=RrReadCrossDomainPoints(program,cross)) {
                std::vector<std::array<float,3>> current;
                for(const auto &point:*points) current.push_back({point[0],point[1],point[2]});
                out.value=std::move(current); out.initialized=true; out.count=points->size();
            }
        } else if(RrReadCrossDomain(program,route.crossRead,&value)) {
            if(value.tag==RigExecWireInputTag::Matrix4d) out.value=value.matrix;
            else if(value.tag==RigExecWireInputTag::Vec3f) out.value=value.vec3f;
            else if(value.tag==RigExecWireInputTag::Vec3d) out.value=value.vec3d;
            else if(value.tag==RigExecWireInputTag::Vec3i) out.value=value.vec3i;
            else if(value.tag==RigExecWireInputTag::Bool) out.value=bool(value.bits);
            else if(value.tag==RigExecWireInputTag::Int) out.value=int32_t(uint32_t(value.bits));
            else if(value.tag==RigExecWireInputTag::Token) out.value=program->TextOrEmpty(uint32_t(value.bits));
            else if(value.tag==RigExecWireInputTag::Double) {
                double d; std::memcpy(&d,&value.bits,sizeof(d)); out.value=d;
            } else if(value.tag==RigExecWireInputTag::Float) {
                uint32_t bits=uint32_t(value.bits); float f;
                std::memcpy(&f,&bits,sizeof(f)); out.value=f;
            }
            out.initialized=!std::holds_alternative<std::monostate>(out.value);
            out.count=out.initialized?1:0;
        }
        out.blocked=!out.initialized;
        out.initialized=true;
        if(out.blocked && !cross.unavailable.empty()) {
            const size_t physical=size_t(&step-program->steps->data());
            program->store.stepOutputs[physical].diagnostics.push_back(cross.unavailable);
        }
        return true;
    }
    if(step.part==4) {
        const auto &leaf=program->poses->providerFrameInputs[size_t(step.object)];
        Store bridge(program->store.providerValues);
        bridge.PublishFrame(leaf.value,leaf.base?program->store.base[leaf.version]:program->store.fin[leaf.version]);
        return true;
    }
    if(step.part!=0) {
        if(error)*error="SpaceExpression has an unsupported bridge part";
        return false;
    }
    if(step.object<0 || size_t(step.object)>=program->providerProgram.ops.size()) {
        if(error)*error="SpaceExpression names no portable op";
        return false;
    }
    const auto &op=program->providerProgram.ops[size_t(step.object)];
    Store store(program->store.providerValues,op.inputs,
                program->store.providerConversionScratch[size_t(step.object)]);
    if(!RigExecRunProviderArithmetic<Math>(op,store)) {
        auto &out=program->store.providerValues[size_t(op.output)];
        out.initialized=false; out.blocked=true; out.value=std::monostate();
        out.count=0; out.error="provider input unavailable";
    }
    return true;
}
namespace {
struct RefreshMath {
    using Frame=RrPointFrame; using Matrix=RrMat4d;
    static bool Usable(const Frame &frame) { return RrFrameUsable(frame); }
    static bool PointsToMatrix(const Frame &a,const Frame &b,Matrix *out) {
        return RrPointsToMatrix(a.points,b,out);
    }
    static Frame Transform(const Frame &frame,const Matrix &matrix) {
        return RrMatrixToPoints(frame.points,matrix);
    }
};
}
bool RrRunProviderRefresh(RrProgram *program,const RigExecWireStep &step,std::string *error)
{
    if(step.object<0 || size_t(step.object)>=program->poses->providerRefreshes.size()) {
        if(error)*error="ProviderRefresh names no compiled context";
        return false;
    }
    const auto &record=program->poses->providerRefreshes[size_t(step.object)];
    auto &store=program->store;
    auto &scratch=store.providerRefreshScratch[size_t(step.object)];
    bool constrained=false;
    for(const auto &prior:record.priorConstraints) {
        const auto &commit=store.commits[prior.first];
        constrained=constrained || (!commit.abandoned && prior.second<commit.present.size() && commit.present[prior.second]);
    }
    for(size_t k=0;k<record.carries.size();++k) {
        const auto &carry=record.carries[k];
        scratch.baseInputs[k]=store.base[carry.baseRead];
        scratch.finInputs[k]=store.fin[carry.finRead];
        scratch.blocked[k]=0;
        for(int slot:carry.blockingSlots) {
            const auto raw=RrValueFromWire(RrReadProviderRefreshBlocker(program,slot));
            if(raw.matrix!=RrMat4d(1.0))scratch.blocked[k]=1;
        }
    }
    const auto frame=[&](uint64_t id,RrPointFrame *out)->const RrPointFrame * {
        if(id>=store.providerValues.size())return nullptr;
        const auto &state=store.providerValues[size_t(id)];
        if(!state.initialized || state.blocked)return nullptr;
        const auto *value=std::get_if<RigExecProviderPlainFrame>(&state.value);
        if(!value)return nullptr;
        for(size_t p=0;p<4;++p)for(size_t a=0;a<3;++a)out->points[p][a]=value->points[p][a];
        out->flags=value->flags;return out;
    };
    RrPointFrame raw,current,baseOutput,finOutput;
    size_t failed=0;
    const auto outcome=RigExecRefreshProviderFrames<RefreshMath>(
        store.base[record.baseRead],store.fin[record.finRead],frame(record.baseValue,&raw),
        frame(record.currentValue,&current),constrained,scratch.baseInputs.data(),scratch.finInputs.data(),
        scratch.blocked.data(),record.carries.size(),&baseOutput,&finOutput,
        scratch.baseOutputs.data(),scratch.finOutputs.data(),&failed);
    store.base[record.baseWrite]=baseOutput; store.fin[record.finWrite]=finOutput;
    for(size_t k=0;k<record.carries.size();++k) {
        store.base[record.carries[k].baseWrite]=scratch.baseOutputs[k];
        store.fin[record.carries[k].finWrite]=scratch.finOutputs[k];
    }
    const size_t physical=size_t(&step-program->steps->data());
    auto &diagnostics=store.stepOutputs[physical].diagnostics;
    const std::string &path=program->TextOrEmpty(program->slotMeta->paths[size_t(record.slot)]);
    if(outcome==RigExecProviderRefreshOutcome::MissingBase)
        diagnostics.push_back("connected base pose input incomplete: "+path);
    else if(outcome==RigExecProviderRefreshOutcome::MissingCurrent)
        diagnostics.push_back("connected final pose input incomplete: "+path);
    else if(outcome==RigExecProviderRefreshOutcome::InvalidCurrent)
        diagnostics.push_back(path+" produced an invalid or degenerate frame for "+path+"; constraint passed through");
    else if(outcome==RigExecProviderRefreshOutcome::SingularCurrent)
        diagnostics.push_back(path+" produced a singular hierarchy delta; constraint passed through");
    else if(outcome==RigExecProviderRefreshOutcome::InvalidDescendant)
        diagnostics.push_back(path+" could not propagate its pose revision through "+
            program->TextOrEmpty(program->slotMeta->paths[size_t(record.carries[failed].slot)])+"; constraint passed through");
    else if(outcome==RigExecProviderRefreshOutcome::InvalidTransformedDescendant)
        diagnostics.push_back(path+" produced an invalid descendant frame for "+
            program->TextOrEmpty(program->slotMeta->paths[size_t(record.carries[failed].slot)])+"; constraint passed through");
    return true;
}

}

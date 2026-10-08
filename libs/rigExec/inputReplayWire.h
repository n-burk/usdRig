#ifndef RIGEXEC_INPUT_REPLAY_WIRE_H
#define RIGEXEC_INPUT_REPLAY_WIRE_H

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace rigExec { namespace inputReplay {
enum class Event : uint8_t {
    Layer = 1, Stage, Create, Compile, Options, Interactive, Clear,
    Upstream, Evaluate, Destroy, Operation, Batch, StageState, End,
    ComparisonBegin, ComparisonEnd,
    ProgramBuild, ProgramRun, ProgramDestroy, ProgramRouter, CompileObserved,
    StageRetire, LayerRetire, ArrayAdmission
};
enum class Op : uint8_t { Field = 1, Dictionary, Sample, Create, Delete,
                         Move, PushToken, PushPath, PopToken, PopPath, Import, Clear, Transfer };
struct Writer {
    std::string bytes;
    void U8(uint8_t v) { bytes.push_back(char(v)); }
    void U32(uint32_t v) { for(unsigned i=0;i<4;++i) U8(uint8_t(v>>(8*i))); }
    void U64(uint64_t v) { for(unsigned i=0;i<8;++i) U8(uint8_t(v>>(8*i))); }
    void Double(double v) { uint64_t bits;std::memcpy(&bits,&v,8);U64(bits); }
    void Text(const std::string &v) { U64(v.size());bytes+=v; }
};
struct Reader {
    const std::string &bytes;size_t at=0;
    explicit Reader(const std::string &v):bytes(v) {}
    void Need(size_t n) const { if(n>bytes.size()-at)throw std::runtime_error("truncated input action stream"); }
    uint8_t U8() { Need(1);return uint8_t(bytes[at++]); }
    uint32_t U32() { uint32_t v=0;for(unsigned i=0;i<4;++i)v|=uint32_t(U8())<<(8*i);return v; }
    uint64_t U64() { uint64_t v=0;for(unsigned i=0;i<8;++i)v|=uint64_t(U8())<<(8*i);return v; }
    double Double() { const auto bits=U64();double v;std::memcpy(&v,&bits,8);return v; }
    std::string Text() { const auto n=U64();if(n>bytes.size()-at)throw std::runtime_error("invalid input action length");auto v=bytes.substr(at,size_t(n));at+=size_t(n);return v; }
    bool Done() const { return at==bytes.size(); }
    void Finish() const { if(!Done())throw std::runtime_error("extra input action payload"); }
};
}}
#endif

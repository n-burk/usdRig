#include "rigExecBinary/transport.h"
#include "LzmaEnc.h"
#include "LzmaDec.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>
namespace rigExec { namespace transport { namespace {
constexpr size_t MaxRaw = size_t(INT32_MAX); // Original FlatBuffers strict upper bound.
constexpr size_t SDKBudget = 128u * 1024u * 1024u; // Fixed 1MiB profile scratch, not raw-file cap.
constexpr uint8_t Magic[8] = {'R','X','E','L','Z','1',0,0};
struct Context { ISzAlloc sdk; Stats stats; Faults faults; };
static_assert(std::is_standard_layout<Context>::value && offsetof(Context,sdk)==0,"allocator owner");
struct alignas(std::max_align_t) Allocation { size_t size; };
void *Allocate(ISzAllocPtr owner, size_t n) {
    auto &c=*reinterpret_cast<Context *>(const_cast<ISzAlloc *>(owner));
    ++c.stats.sdkCalls;
    if ((c.faults.failSDKCall && c.stats.sdkCalls==c.faults.failSDKCall) ||
        n>SDKBudget-c.stats.sdkActive || n>SIZE_MAX-sizeof(Allocation)) return nullptr;
    auto *p=static_cast<Allocation *>(std::malloc(n+sizeof(Allocation)));
    if (!p) return nullptr;
    p->size=n; c.stats.sdkActive+=n;
    c.stats.sdkPeak=std::max(c.stats.sdkPeak,c.stats.sdkActive); return p+1;
}
void Free(ISzAllocPtr owner,void *ptr) {
    if (!ptr) return;
    auto &c=*reinterpret_cast<Context *>(const_cast<ISzAlloc *>(owner));
    auto *p=static_cast<Allocation *>(ptr)-1;c.stats.sdkActive-=p->size;std::free(p);
}
struct Report { Context &context; Stats *out; ~Report(){if(out)*out=context.stats;} };
uint32_t CRC(const uint8_t *p,size_t n) {
    uint32_t c=~0u;
    for(size_t i=0;i<n;++i){c^=p[i];for(unsigned k=0;k<8;++k)c=(c>>1)^(0xEDB88320u&uint32_t(0-int(c&1)));}
    return ~c;
}
uint64_t Get(const uint8_t *p,unsigned n){uint64_t v=0;for(unsigned i=0;i<n;++i)v|=uint64_t(p[i])<<(8*i);return v;}
void Put(uint8_t *p,uint64_t v,unsigned n){for(unsigned i=0;i<n;++i)p[i]=uint8_t(v>>(8*i));}
bool Fail(std::string *error,const char *message){if(error)*error=message;return false;}
} // namespace
bool IsEnvelope(const uint8_t *p,size_t n){return p&&n>=8&&!std::memcmp(p,Magic,8);}
bool Decode(const uint8_t *p,size_t n,Buffer *out,std::string *error,Stats *stats,Faults faults) {
    Context c{{Allocate,Free},{},faults};Report report{c,stats};
    if(!out||!IsEnvelope(p,n)||n<HeaderSize||Get(p+8,4)!=1||Get(p+12,4)!=HeaderSize)
        return Fail(error,"invalid .rigexec transport header");
    const uint64_t raw=Get(p+16,8),packed=Get(p+24,8);
    if(!raw||raw>=MaxRaw||!packed||packed>=MaxRaw||packed!=n-HeaderSize)
        return Fail(error,"invalid .rigexec transport lengths");
    if(p[32]!=0x5D||Get(p+33,4)!=(1u<<20)||p[37]||p[38]||p[39])
        return Fail(error,"unsupported .rigexec transport properties");
    if(Get(p+44,4)!=CRC(p+HeaderSize,size_t(packed)))
        return Fail(error,"corrupt .rigexec compressed payload");
    ++c.stats.outputAttempts;
    Buffer candidate;
    if(!faults.failOutput && faults.failOutputCall!=c.stats.outputAttempts)candidate.data.reset(static_cast<uint8_t *>(std::malloc(size_t(raw))));
    if(!candidate.data)return Fail(error,"cannot decode .rigexec: out of memory");
    SizeT dest=size_t(raw),source=size_t(packed);ELzmaStatus status=LZMA_STATUS_NOT_SPECIFIED;
    const SRes result=LzmaDecode(candidate.data.get(),&dest,p+HeaderSize,&source,p+32,5,LZMA_FINISH_END,&status,&c.sdk);
    if(result!=SZ_OK||status!=LZMA_STATUS_FINISHED_WITH_MARK||dest!=raw||source!=packed||CRC(candidate.data.get(),size_t(raw))!=Get(p+40,4))
        return Fail(error,"invalid .rigexec compressed stream");
    candidate.size=size_t(raw);*out=std::move(candidate);return true;
}
bool Encode(const uint8_t *raw,size_t n,std::vector<uint8_t> *out,std::string *error,Stats *stats,Faults faults) {
    Context c{{Allocate,Free},{},faults};Report report{c,stats};
    if(!out||!raw||!n||n>=MaxRaw)return Fail(error,"invalid .rigexec raw payload size");
    if(n<=HeaderSize+1){out->clear();return true;}
    ++c.stats.outputAttempts;
    std::unique_ptr<uint8_t,FreeBuffer> candidate;
    if(!faults.failOutput && faults.failOutputCall!=c.stats.outputAttempts)candidate.reset(static_cast<uint8_t *>(std::malloc(n-1)));
    if(!candidate)return Fail(error,"cannot encode .rigexec: out of memory");
    CLzmaEncProps props;LzmaEncProps_Init(&props);props.level=1;props.dictSize=1u<<20;
    props.algo=0;props.btMode=0;props.fb=128;props.mc=32;props.numThreads=1;props.lc=3;props.lp=0;props.pb=2;
    SizeT packed=n-HeaderSize-1,propSize=5;
    const SRes result=LzmaEncode(candidate.get()+HeaderSize,&packed,raw,n,&props,candidate.get()+32,&propSize,1,nullptr,&c.sdk,&c.sdk);
    if(result==SZ_ERROR_OUTPUT_EOF){out->clear();return true;}
    if(result!=SZ_OK||propSize!=5)return Fail(error,"cannot encode .rigexec compressed stream");
    std::memcpy(candidate.get(),Magic,8);Put(candidate.get()+8,1,4);Put(candidate.get()+12,HeaderSize,4);
    Put(candidate.get()+16,n,8);Put(candidate.get()+24,packed,8);
    candidate.get()[37]=candidate.get()[38]=candidate.get()[39]=0;
    Put(candidate.get()+40,CRC(raw,n),4);Put(candidate.get()+44,CRC(candidate.get()+HeaderSize,packed),4);
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    try {
#endif
        ++c.stats.outputAttempts;
        if(faults.failOutputCall==c.stats.outputAttempts)
            return Fail(error,"cannot publish .rigexec: out of memory");
        std::vector<uint8_t> bytes(candidate.get(),candidate.get()+HeaderSize+packed);out->swap(bytes);
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    }catch(const std::bad_alloc &){return Fail(error,"cannot encode .rigexec: out of memory");}
#endif
    return true;
}
} }

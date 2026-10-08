#ifndef RIGEXEC_PHASE_QUERY_OBSERVER_H
#define RIGEXEC_PHASE_QUERY_OBSERVER_H
#include "pxr/base/vt/value.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace rigExecOriginalQuery {
inline std::string Text(const std::string &s) {
    std::ostringstream out; out << std::hex << std::setfill('0');
    for (unsigned char c : s) out << std::setw(2) << unsigned(c);
    return out.str();
}
template<class T> inline void Bits(std::ostringstream &out,T value) {
    static_assert(sizeof(T)==4 || sizeof(T)==8,"scalar raw bits");
    uint64_t bits=0; std::memcpy(&bits,&value,sizeof(T));
    out << std::hex << std::setfill('0') << std::setw(sizeof(T)*2) << bits;
}
inline std::string Value(const pxr::VtValue *value) {
    if (!value) return "missing";
    if (value->IsEmpty()) return "empty";
    std::ostringstream out;
    if(value->IsHolding<float>()) {out<<"f:";Bits(out,value->UncheckedGet<float>());}
    else if(value->IsHolding<double>()) {out<<"d:";Bits(out,value->UncheckedGet<double>());}
    else if(value->IsHolding<pxr::GfMatrix4d>()) {
        out<<"m:";const auto &m=value->UncheckedGet<pxr::GfMatrix4d>();
        for(int r=0;r<4;++r)for(int c=0;c<4;++c)Bits(out,m[r][c]);
    } else if(value->IsHolding<pxr::VtVec3fArray>()) {
        const auto &a=value->UncheckedGet<pxr::VtVec3fArray>();
        out<<"a3f:"<<a.size()<<":";
        for(const auto &p:a)for(int c=0;c<3;++c)Bits(out,p[c]);
    } else throw std::runtime_error("original query observer unsupported typed value");
    return out.str();
}
struct Configuration {
    bool capture=false, check=false;
    std::string filename,suite;
    std::vector<std::string> expected;
    Configuration() {
        const char *mode=std::getenv("RIGEXEC_PHASE_QUERY_OBSERVER");
        const char *label=std::getenv("RIGEXEC_PHASE_QUERY_SUITE");
        if(label)suite=label;
        if(!mode || !*mode)return;
        const std::string setting(mode);
        capture=setting.compare(0,8,"capture:")==0;
        check=setting.compare(0,6,"check:")==0;
        if(!capture && !check)throw std::runtime_error("invalid phase query observer mode");
        filename=setting.substr(capture?8:6);
        if(filename.empty())throw std::runtime_error("empty phase query observer file");
        if(check) {
            std::ifstream input(filename);
            if(!input)throw std::runtime_error("cannot read phase query observer corpus");
            std::string line;while(std::getline(input,line)) {
                if(!line.empty() && line.back()=='\r')line.pop_back();
                expected.push_back(line);
            }
        }
    }
};
inline const Configuration configuration;
inline uint64_t sequence=0;
// Observation only: original callers pass the returned independent lookup
// value and separately captured bound answer. Neither is replaced by this hook.
inline void Emit(const std::string &where,const std::string &reader,
                 const std::string &path,const std::string &phase,bool isDefault,
                 double time,const pxr::VtValue *value,const pxr::VtValue *bound=nullptr) {
    if(!configuration.capture && !configuration.check)return;
    std::ostringstream timeBits;Bits(timeBits,time);
    std::ostringstream row;
    row<<"query1\t"<<sequence<<'\t'<<Text(configuration.suite)<<'\t'
       <<Text(where)<<'\t'<<Text(reader)<<'\t'<<Text(path)<<'\t'<<Text(phase)
       <<'\t'<<(isDefault?"default":timeBits.str())<<'\t'<<Value(value)
       <<'\t'<<Value(bound);
    if(configuration.capture) {
        std::ofstream out(configuration.filename,std::ios::app);
        if(!out)throw std::runtime_error("cannot write original query observer");
        out<<row.str()<<'\n';
        if(!out)throw std::runtime_error("original query observer write failed");
    } else if(sequence>=configuration.expected.size() ||
              configuration.expected[size_t(sequence)].substr(0,configuration.expected[size_t(sequence)].rfind('\t'))!=row.str().substr(0,row.str().rfind('\t'))) {
        throw std::runtime_error("phase query differs from independent original capture at " +
                                 std::to_string(sequence));
    }
    ++sequence;
}
inline void Finish() {
    if(configuration.check && sequence!=configuration.expected.size())
        throw std::runtime_error("phase query corpus has unconsumed original queries");
}

}
#endif

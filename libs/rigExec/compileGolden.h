#ifndef RIGEXEC_COMPILE_GOLDEN_H
#define RIGEXEC_COMPILE_GOLDEN_H

// Passive metadata only. Caller arguments and actual returned results remain
// distinct. A matching negative attestation never changes a native exit code.
#include "rigExec/heldProgramGolden.h"
#include <cstdint>
#include <map>
#include <thread>
#include <utility>

namespace rigExec {
struct RigExecCompileGoldenToken {
    bool enabled = false;
    unsigned owner = 0;
    uint64_t call = 0;
};

namespace compileGolden {
[[noreturn]] inline void Fail(const std::string &message) {
    std::fprintf(stderr,"compile golden refused: %s\n",message.c_str());
    std::fflush(stderr);std::_Exit(EXIT_FAILURE);
}
struct Node {
    std::string rig,bytes;
    uint64_t calls=0;
    bool pending=false,requested=false;
};
struct Registry {
    bool enabled=false,capture=false,finalized=false;
    uint64_t refused=0;
    std::thread::id thread;
    std::filesystem::path directory;
    std::map<unsigned,Node> nodes;
    std::vector<std::pair<unsigned,uint64_t>> order;
    Registry() {
        const char *mode=std::getenv("RIGEXEC_COMPILE_GOLDEN");
        if(!mode||!*mode)return;
        const std::string value(mode);size_t prefix=0;
        if(value.rfind("capture:",0)==0){capture=true;prefix=8;}
        else if(value.rfind("check:",0)==0)prefix=6;
        else Fail("unsupported RIGEXEC_COMPILE_GOLDEN mode");
        const char *unit=std::getenv("RIGEXEC_GOLDEN_SUITE_NAME");
        if(!unit||!*unit||value.size()==prefix)Fail("missing compile unit/root");
        const std::filesystem::path name(unit);
        if(name.has_parent_path()||name=="."||name=="..")Fail("invalid compile unit");
        directory=std::filesystem::path(value.substr(prefix))/name/"compiles";
        enabled=true;thread=std::this_thread::get_id();
    }
    void CheckThread() const {
        if(enabled&&thread!=std::this_thread::get_id())Fail("compile observation changed owning thread");
    }
    void Finalize() {
        if(finalized)return;
        CheckThread();finalized=true;
        if(!enabled)return;
        try {
            std::string bytes="rigexec-compile-index 1\n";
            std::set<std::string> expected;
            for(const auto &entry:nodes) {
                const auto &node=entry.second;
                if(node.pending)Fail("unfinished Compile owner "+std::to_string(entry.first));
                const std::string file=std::to_string(entry.first)+".golden";
                expected.insert(file);
                heldProgramGolden::Store(directory/file,node.bytes+"end calls "+std::to_string(node.calls)+"\n",capture);
                bytes+="owner "+std::to_string(entry.first)+" rig "+RigExecGoldenEscape(node.rig)+" calls "+std::to_string(node.calls)+"\n";
            }
            for(size_t i=0;i<order.size();++i)
                bytes+="ordinal "+std::to_string(i)+" owner "+std::to_string(order[i].first)+" call "+std::to_string(order[i].second)+"\n";
            bytes+="end owners "+std::to_string(nodes.size())+" calls "+std::to_string(order.size())+" refused "+std::to_string(refused)+"\n";
            if(!capture)expected.insert("index.golden");
            std::set<std::string> actual;
            if(std::filesystem::exists(directory))for(const auto &entry:std::filesystem::directory_iterator(directory)) {
                if(!entry.is_regular_file())Fail("non-file in compile inventory");
                actual.insert(entry.path().filename().string());
            }
            if(actual!=expected)Fail("compile file inventory differs");
            heldProgramGolden::Store(directory/"index.golden",bytes,capture);
        } catch(const std::exception &error) {Fail(error.what());}
    }
    ~Registry(){Finalize();}
};
inline Registry &Index(){static Registry value;return value;}
}

inline RigExecCompileGoldenToken RigExecBeginCompileGolden(
    unsigned owner,const SdfPath &rig,uint64_t call,bool requested,
    const std::vector<std::string> &initialSeed) {
    auto &index=compileGolden::Index();
    if(!index.enabled)return {false,owner,call};
    index.CheckThread();
    if(index.finalized)compileGolden::Fail("Compile begun after finalization");
    if(!requested&&!initialSeed.empty())compileGolden::Fail("nullptr diagnostics with initial seed");
    auto found=index.nodes.find(owner);
    if(found==index.nodes.end()) {
        compileGolden::Node node;node.rig=rig.GetString();
        node.bytes="rigexec-compile-golden 1\nowner "+std::to_string(owner)+" rig "+RigExecGoldenEscape(node.rig)+"\n";
        found=index.nodes.emplace(owner,std::move(node)).first;
    }
    auto &node=found->second;
    if(node.rig!=rig.GetString()||node.pending||call!=node.calls)compileGolden::Fail("missing/reordered/pending Compile identity");
    node.pending=true;node.requested=requested;
    node.bytes+="call "+std::to_string(call)+" requested "+(requested?"1":"0")+" initial "+std::to_string(initialSeed.size())+"\n";
    for(size_t i=0;i<initialSeed.size();++i)
        node.bytes+="initial "+std::to_string(i)+" "+RigExecGoldenEscape(initialSeed[i])+"\n";
    index.order.emplace_back(owner,call);
    return {true,owner,call};
}

inline void RigExecRecordCompileGolden(const RigExecCompileGoldenToken &token,
    bool returned,const std::vector<std::string> *post) {
    if(!token.enabled)return;
    auto &index=compileGolden::Index();index.CheckThread();
    if(index.finalized)compileGolden::Fail("Compile result after finalization");
    auto found=index.nodes.find(token.owner);
    if(found==index.nodes.end())compileGolden::Fail("Compile result without owner");
    auto &node=found->second;
    if(!node.pending||token.call!=node.calls||node.requested!=bool(post))compileGolden::Fail("duplicate/mismatched Compile result");
    node.bytes+="returned "+std::string(returned?"1":"0")+" requested "+(post?"1":"0")+" diagnostics "+std::to_string(post?post->size():0)+"\n";
    if(post)for(size_t i=0;i<post->size();++i)
        node.bytes+="diagnostic "+std::to_string(i)+" "+RigExecGoldenEscape((*post)[i])+"\n";
    if(!returned)++index.refused;
    ++node.calls;node.pending=false;
}
inline void RigExecFinalizeCompileGolden(){compileGolden::Index().Finalize();}
}
#endif

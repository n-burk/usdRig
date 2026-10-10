#ifndef RIGEXEC_HELD_PROGRAM_GOLDEN_H
#define RIGEXEC_HELD_PROGRAM_GOLDEN_H

// Passive, shared CURRENT/ORIGINAL observation. No numerical inputs or state
// are changed here. In particular, actual refused pose/time remains intact.
#include "rigExec/goldenPose.h"
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace rigExec {
namespace heldProgramGolden {
[[noreturn]] inline void Fail(const std::string &message) {
    std::fprintf(stderr,"held-program golden refused: %s\n",message.c_str());
    std::fflush(stderr); std::_Exit(EXIT_FAILURE);
}
inline std::string Read(const std::filesystem::path &path) {
    std::ifstream in(path,std::ios::binary);
    if(!in)Fail("cannot read "+path.string());
    std::string result{std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>()};
    if(in.bad())Fail("cannot finish "+path.string());
    return result;
}
inline void Write(const std::filesystem::path &path,const std::string &bytes) {
    const auto pending=std::filesystem::path(path.string()+".pending");
    if(std::filesystem::exists(path)||std::filesystem::exists(pending))Fail("existing output "+path.string());
    std::filesystem::create_directories(path.parent_path());
    {std::ofstream out(pending,std::ios::binary);out.write(bytes.data(),std::streamsize(bytes.size()));out.flush();
        if(!out)Fail("cannot finish output "+pending.string());}
    std::filesystem::rename(pending,path);
}
inline void Store(const std::filesystem::path &path,const std::string &bytes,bool capture) {
    if(capture)Write(path,bytes);
    else {std::string error;if(!RigExecCompareGolden(Read(path),bytes,&error))Fail(path.string()+": "+error);}
}
struct Node { std::string rig;size_t runs=0;bool built=false,closed=false; };
struct Registry {
    bool enabled=false,capture=false,raw=false,finalized=false;
    std::filesystem::path directory;
    std::vector<Node> nodes;
    Registry() {
        const char *mode=std::getenv("RIGEXEC_GOLDEN_SUITE");
        if(!mode||!*mode)return;
        const std::string value(mode);size_t prefix=0;
        if(value.rfind("capture:",0)==0){capture=true;prefix=8;}
        else if(value.rfind("check:",0)==0)prefix=6;
        else Fail("unsupported RIGEXEC_GOLDEN_SUITE mode");
        const char *unit=std::getenv("RIGEXEC_GOLDEN_SUITE_NAME");
        if(!unit||!*unit||value.size()==prefix)Fail("missing held-program golden unit/root");
        const char *full=std::getenv("RIGEXEC_GOLDEN_SUITE_RAW");raw=full&&std::string(full)=="1";
        directory=std::filesystem::path(value.substr(prefix))/unit/"programs";
        enabled=true;
    }
    void Finalize() {
        if(finalized)return;
        finalized=true;
        if(!enabled)return;
        std::string bytes="rigexec-held-program-index 1\n";size_t visits=0;
        for(size_t i=0;i<nodes.size();++i){const auto &node=nodes[i];
            if(!node.built||!node.closed)Fail("unclosed/unbuilt held program "+std::to_string(i));
            visits+=node.runs;bytes+="program "+std::to_string(i)+" rig "+RigExecGoldenEscape(node.rig)+" runs "+std::to_string(node.runs)+"\n";
        }
        bytes+="end programs "+std::to_string(nodes.size())+" runs "+std::to_string(visits)+"\n";
        std::set<std::string> expected;
        for(size_t i=0;i<nodes.size();++i)expected.insert(std::to_string(i)+".golden");
        if(!capture)expected.insert("index.golden");
        std::set<std::string> actual;
        if(std::filesystem::exists(directory))for(const auto &entry:std::filesystem::directory_iterator(directory)) {
            if(!entry.is_regular_file())Fail("non-file in held program inventory");
            actual.insert(entry.path().filename().string());
        }
        if(actual!=expected)Fail("held program file inventory differs");
        Store(directory/"index.golden",bytes,capture);
    }
    ~Registry() {Finalize();}
};
inline Registry &Index(){static Registry value;return value;}
inline std::string Time(UsdTimeCode time) {return time.IsDefault()?"default":RigExecGoldenDouble(time.GetValue());}
}

inline void RigExecFinalizeHeldProgramGolden() {heldProgramGolden::Index().Finalize();}

class RigExecHeldProgramGoldenObserver {
public:
    static std::unique_ptr<RigExecHeldProgramGoldenObserver> Create(unsigned id,const SdfPath &rig) {
        auto &index=heldProgramGolden::Index();if(!index.enabled)return nullptr;
        if(index.finalized)heldProgramGolden::Fail("held program created after finalization");
        if(id!=index.nodes.size())heldProgramGolden::Fail("missing/reordered held program ID");
        index.nodes.push_back({rig.GetString(),0,false,false});
        return std::unique_ptr<RigExecHeldProgramGoldenObserver>(new RigExecHeldProgramGoldenObserver(id));
    }
    void RecordBuild(bool okay,const std::vector<std::string> &reasons) {
        if(heldProgramGolden::Index().finalized)heldProgramGolden::Fail("held Build after finalization");
        auto &node=heldProgramGolden::Index().nodes.at(_id);
        if(node.built||node.closed)heldProgramGolden::Fail("duplicate/closed held Build");
        node.built=true;_bytes+="build returned "+std::string(okay?"1":"0")+" reasons "+std::to_string(reasons.size())+"\n";
        for(size_t i=0;i<reasons.size();++i)_bytes+="reason "+std::to_string(i)+" "+RigExecGoldenEscape(reasons[i])+"\n";
    }
    void RecordRun(UsdTimeCode requested,const RigExecRigPose &pose,bool okay,const char *bail) {
        if(heldProgramGolden::Index().finalized)heldProgramGolden::Fail("held Run after finalization");
        if(!bail)heldProgramGolden::Fail("missing actual bail name");
        auto &node=heldProgramGolden::Index().nodes.at(_id);
        if(!node.built||node.closed)heldProgramGolden::Fail("held Run before Build/after close");
        std::vector<RigExecGoldenValue> values;std::string error;
        if(!RigExecEncodeGoldenPose(pose,&values,&error))heldProgramGolden::Fail(error);
        _bytes+="run "+std::to_string(node.runs)+" requested-time "+heldProgramGolden::Time(requested)+" returned "+(okay?"1":"0")+" bail "+RigExecGoldenEscape(bail)+"\n";
        _bytes+=RigExecGoldenVisit("held-program",node.runs,pose,values,!heldProgramGolden::Index().raw);
        ++node.runs;
    }
    void Close() {
        auto &index=heldProgramGolden::Index();auto &node=index.nodes.at(_id);
        if(node.closed)return;
        if(index.finalized)heldProgramGolden::Fail("held Close after finalization");
        if(!node.built)heldProgramGolden::Fail("held program closed without Build result");
        _bytes+="end runs "+std::to_string(node.runs)+"\n";
        heldProgramGolden::Store(index.directory/(std::to_string(_id)+".golden"),_bytes,index.capture);
        node.closed=true;
    }
    ~RigExecHeldProgramGoldenObserver(){Close();}
    RigExecHeldProgramGoldenObserver(const RigExecHeldProgramGoldenObserver &)=delete;
    RigExecHeldProgramGoldenObserver &operator=(const RigExecHeldProgramGoldenObserver &)=delete;
private:
    explicit RigExecHeldProgramGoldenObserver(unsigned id):_id(id) {
        _bytes="rigexec-held-program-golden 1 encoding "+std::string(heldProgramGolden::Index().raw?"full-values":"domain-digests")+"\n";
        _bytes+="program "+std::to_string(id)+" rig "+RigExecGoldenEscape(heldProgramGolden::Index().nodes.at(id).rig)+"\n";
    }
    unsigned _id;std::string _bytes;
};
}
#endif

#include "rigExec/inputReplay.h"
#include "rigExec/moverGraph.h"
#include "rigExec/movers/moverRegistry.h"
#include "pxr/base/plug/registry.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE
int main(int argc,char **argv) {
    if(argc<3) {
        std::fprintf(stderr,"usage: rigExecReplayInputs <input-actions> <core-schema-resources> [mover-plugin-resources ...]\n");
        return 2;
    }
    if(const char *capture=std::getenv("RIGEXEC_INPUT_REPLAY"))if(*capture) {
        std::fprintf(stderr,"Disable RIGEXEC_INPUT_REPLAY before replaying an immutable input transcript\n");return 2;
    }
    const char *judge=std::getenv("RIGEXEC_GOLDEN_SUITE");
#ifdef RIGEXEC_INPUT_REPLAY_ORIGINAL
    if(!judge||std::string(judge).rfind("capture:",0)!=0) {
        std::fprintf(stderr,"The original numerical host requires G5 capture into a fresh judge directory\n");return 2;
    }
#else
    if(!judge||std::string(judge).rfind("check:",0)!=0) {
        std::fprintf(stderr,"The current numerical host requires G5 check against an original judge\n");return 2;
    }
#endif
    for(int i=2;i<argc;++i)PlugRegistry::GetInstance().RegisterPlugins(argv[i]);
    rigExec::RigExecLoadComputations();
    std::vector<std::string> diagnostics;
    if(!rigExec::RigExecLoadMoverPlugins(&diagnostics)) {
        for(const auto &message:diagnostics)std::fprintf(stderr,"plugin refusal: %s\n",message.c_str());return 1;
    }
    std::string error;
    if(!rigExec::RigExecReplayInputActions(argv[1],&error)) {
        std::fprintf(stderr,"input replay failed: %s\n",error.c_str());return 1;
    }
    std::printf("Input chronology and completeness replayed; numerical verdict uses the independent G5 original judge\n");
    return 0;
}

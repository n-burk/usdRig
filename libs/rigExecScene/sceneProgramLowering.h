#ifndef RIGEXEC_GRAPH_SCENE_PROGRAM_LOWERING_H
#define RIGEXEC_GRAPH_SCENE_PROGRAM_LOWERING_H
#include "sceneProgram.h"
namespace rigExec {
bool RigExecLowerSceneProgram(const RigExecSceneDescriptors &,RigExecSceneProgram *,
    std::string *error=nullptr);
}
#endif

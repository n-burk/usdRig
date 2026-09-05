#ifndef RIGEXEC_STANDALONE_ADAPTER_H
#define RIGEXEC_STANDALONE_ADAPTER_H
#include "sceneDb.h"
#include "pxr/exec/esf/stage.h"
#include "pxr/exec/esf/object.h"
namespace rigExec {
EsfStage RigExecAdaptStandaloneStage(const RigExecSceneDb *db);
EsfObject RigExecAdaptStandaloneObject(const RigExecSceneDb *db, const SdfPath &path);
} // namespace rigExec
#endif

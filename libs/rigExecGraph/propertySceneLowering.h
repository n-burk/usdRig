#ifndef RIGEXEC_GRAPH_PROPERTY_SCENE_LOWERING_H
#define RIGEXEC_GRAPH_PROPERTY_SCENE_LOWERING_H
#include "sceneCompileInputs.h"
#include "sceneTypedReads.h"
#include "propertyProgram.h"
#include <variant>
namespace rigExec {
using RigExecPropertyRecord=std::variant<RigExecFloatPropertyRecord,RigExecVec3PropertyRecord,RigExecMatrixPropertyRecord>;
struct RigExecScenePropertyDescriptor {
    SdfPath mover,target,weightObject;
    SdfValueTypeName targetType;
    RigExecPropertyRecord record;
    std::map<std::string,RigExecSceneBoundInput> inputs;
    std::map<std::string,RigExecSceneTypedRead> typedReads;
};
bool RigExecLowerSceneProperty(const RigExecSceneDescriptors &,const SdfPath &mover,
    const SdfPath &target,RigExecScenePropertyDescriptor *,std::string *error=nullptr);
/// Resolves graph-selected inputs into retained typed record storage. Weight
/// field selection and envelope validation are domain operations before apply.
bool RigExecResolveSceneProperty(const RigExecSceneDescriptors &,
    RigExecScenePropertyDescriptor *,UsdTimeCode,const std::map<SdfPath,VtValue> &delivered,
    bool *enabled,float *defaultWeight,std::string *error=nullptr);
}
#endif

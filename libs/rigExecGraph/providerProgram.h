#ifndef RIGEXEC_GRAPH_PROVIDER_PROGRAM_H
#define RIGEXEC_GRAPH_PROVIDER_PROGRAM_H
#include "sceneDescriptors.h"
#include "typedValues.h"
#include "providerRecords.h"
#include <limits>
#include <set>

namespace rigExec {
constexpr RigExecValueId RigExecNoProviderValue=std::numeric_limits<RigExecValueId>::max();
struct RigExecProviderXformInput {
    TfToken name;
    RigExecValueId raw=RigExecNoProviderValue;
    int type=0;
};
constexpr uint32_t RigExecNoProviderOwnerText=std::numeric_limits<uint32_t>::max();
struct RigExecProviderOp {
    RigExecProviderOpKind kind;
    SdfPath owner;
    RigExecValueId output=RigExecNoProviderValue;
    std::vector<RigExecValueId> inputs;
    std::vector<RigExecProviderXformInput> xforms;
    bool scaleAvars=true;
    /// `owner`'s entry in RigExecProviderProgram::ownerTexts.
    uint32_t ownerText=RigExecNoProviderOwnerText;
};
struct RigExecProviderLeaf {
    RigExecValueId value;
    SdfPath attribute;
};
struct RigExecProviderExternalInput {
    RigExecValueId value;
    SdfPath owner;
    std::string computation;
};
struct RigExecProviderRoutedInput {
    RigExecValueId value;
    SdfPath consumer,source;
    TfToken readPhase;
};
struct RigExecProviderProgram {
    std::vector<std::string> valueKeys;
    std::map<std::string,RigExecValueId> valueIds;
    std::vector<RigExecProviderLeaf> sampled;
    std::map<SdfPath,RigExecValueId> rawInputs, attributeValues;
    std::vector<RigExecProviderExternalInput> externalInputs;
    std::vector<RigExecProviderRoutedInput> routedInputs;
    std::vector<RigExecValueId> leaves;
    std::vector<RigExecProviderOp> ops;
    std::vector<RigExecOpDescriptor> descriptors;
    /// Each distinct op owner, spelled once on the thread that builds or
    /// extends the program. An op body names its owner only through this
    /// table: SdfPath::GetString interns under Sdf's table locks.
    std::vector<std::string> ownerTexts;
    RigExecValueId FindValue(const std::string &key) const;
};
/// Spells the owner of every op that has no text yet into ownerTexts, once
/// per distinct owner. The owning thread calls it after adding ops; a copied
/// op keeps its text.
void RigExecSpellProviderOwners(RigExecProviderProgram *program);
std::string RigExecProviderValueKey(const SdfPath &owner,const std::string &computation);
std::string RigExecProviderRawKey(const SdfPath &attribute);
std::string RigExecProviderAttributeKey(const SdfPath &attribute);
std::string RigExecProviderRoutedKey(const SdfPath &consumer,const TfToken &phase);
/// With composeFrames=false, native rest/posed frames are explicit leaves;
/// each expression kernel can enter the native graph as one ordinary op.
bool RigExecBuildProviderProgram(const RigExecSceneDescriptors &scene,
    bool composeFrames,RigExecProviderProgram *program,std::string *error=nullptr,
    const std::set<SdfPath> *selectedProviders=nullptr);
/// Clone one connected expression closure into a consumer's selected frame
/// context. Explicit replacements are typed SSA IDs; computation name remaps
/// create external bridge leaves. Other external/raw leaves remain shared.
bool RigExecCloneProviderContext(RigExecProviderProgram *,RigExecValueId result,
    const std::string &prefix,const std::map<std::string,std::string> &externalNames,
    const std::map<RigExecValueId,RigExecValueId> &replacements,
    RigExecValueId *output,std::string *error=nullptr);
bool RigExecRunProviderOp(const RigExecProviderProgram &program,uint32_t originalIndex,
    RigExecTypedValueStore *store,std::string *error=nullptr);
/// Sampling/overlays publish raw slots before entering the readiness loop.
bool RigExecSampleProviderProgram(const RigExecProviderProgram &program,
    const RigExecSceneDescriptors &scene,size_t identity,
    const std::map<SdfPath,VtValue> &overlays,RigExecTypedValueStore *store,
    std::vector<RigExecValueId> *changed,std::string *error=nullptr);
}
#endif

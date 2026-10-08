#ifndef RIGEXEC_GRAPH_SCENE_COMPILE_INPUTS_H
#define RIGEXEC_GRAPH_SCENE_COMPILE_INPUTS_H
#include "sceneDescriptors.h"
namespace rigExec {
enum class RigExecSceneReadRoute { Raw, ConnectionResolved };
struct RigExecSceneBoundInput {
    SdfPath consumer, source;
    SdfPathVector walk;
    SdfValueTypeName type;
    TfToken readPhase;
    RigExecSceneReadRoute route = RigExecSceneReadRoute::Raw;
    bool varies = false;
    std::vector<VtValue> values;
    std::vector<char> available, blocked;
    /// Computed expressions require a graph producer, never a raw fallback.
    std::vector<char> computed;
};
/// Build-time reads over an owned snapshot. Both USD and SceneDb capture this
/// same input model; compiled bindings retain no source/query handles.
class RigExecSceneCompileInputs {
public:
    explicit RigExecSceneCompileInputs(const RigExecSceneDescriptors &scene)
        : _scene(scene) {}
    const RigExecSceneNodeDescriptor *Node(const SdfPath &) const;
    const RigExecSceneAttributeDescriptor *Attribute(const SdfPath &) const;
    const RigExecSceneRelationshipDescriptor *Relationship(const SdfPath &) const;
    bool Bind(const SdfPath &, RigExecSceneReadRoute,
              RigExecSceneBoundInput *, std::string *error = nullptr) const;
    bool Read(const RigExecSceneBoundInput &, UsdTimeCode, VtValue *,
              bool *blocked = nullptr, std::string *error = nullptr) const;
    template<class T> bool Read(const RigExecSceneBoundInput &input,
        UsdTimeCode identity, T *value, std::string *error = nullptr) const {
        VtValue boxed;
        if (!value || !Read(input,identity,&boxed,nullptr,error)) return false;
        if (!boxed.IsHolding<T>()) {
            if (error) *error = "input type mismatch: " + input.consumer.GetString();
            return false;
        }
        *value = boxed.UncheckedGet<T>();
        return true;
    }
    SdfPathVector Targets(const SdfPath &) const;
    /// Relationship targets keep original cardinality including missing paths.
    bool TargetExists(const SdfPath &, size_t target) const;
    const std::vector<UsdTimeCode> &Identities() const { return _scene.identities; }
private:
    const RigExecSceneDescriptors &_scene;
};
}
#endif

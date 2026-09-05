#include "adapter.h"
#include "pxr/exec/esf/attribute.h"
#include "pxr/exec/esf/attributeQuery.h"
#include "pxr/exec/esf/prim.h"
#include "pxr/exec/esf/property.h"
#include "pxr/exec/esf/relationship.h"
#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include <algorithm>
#include <type_traits>

namespace rigExec {
namespace {
EsfPrim Prim(const RigExecSceneDb *, const SdfPath &);
EsfAttribute Attribute(const RigExecSceneDb *, const SdfPath &);
EsfProperty Property(const RigExecSceneDb *, const SdfPath &);
EsfRelationship Relationship(const RigExecSceneDb *, const SdfPath &);

class StageImpl : public EsfStageInterface {
public:
    explicit StageImpl(const RigExecSceneDb *db) : _db(db) {}
private:
    EsfAttribute _GetAttributeAtPath(const SdfPath &path) const override { return Attribute(_db, path); }
    EsfObject _GetObjectAtPath(const SdfPath &path) const override { return RigExecAdaptStandaloneObject(_db, path); }
    EsfPrim _GetPrimAtPath(const SdfPath &path) const override { return Prim(_db, path); }
    EsfProperty _GetPropertyAtPath(const SdfPath &path) const override { return Property(_db, path); }
    EsfRelationship _GetRelationshipAtPath(const SdfPath &path) const override { return Relationship(_db, path); }
    std::pair<TfToken, TfToken> _GetTypeNameAndInstance(const TfToken &name) const override {
        return UsdSchemaRegistry::GetTypeNameAndInstance(name);
    }
    TfType _GetAPITypeFromSchemaTypeName(const TfToken &name) const override {
        return UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(name);
    }
    const RigExecSceneDb *_db;
};

template<class Base>
class ObjectImpl : public Base {
public:
    ObjectImpl(const RigExecSceneDb *db, const SdfPath &path) : Base(path), _db(db) {}
    bool IsPrim() const override { return _db->prims.count(this->_GetPath()) != 0; }
    bool IsAttribute() const override { return _db->attributes.count(this->_GetPath()) != 0; }
    bool IsRelationship() const override { return _db->relationships.count(this->_GetPath()) != 0; }
    EsfObject AsObject() const override { return RigExecAdaptStandaloneObject(_db, this->_GetPath()); }
    EsfAttribute AsAttribute() const override { return Attribute(_db, IsAttribute() ? this->_GetPath() : SdfPath()); }
    EsfRelationship AsRelationship() const override { return Relationship(_db, IsRelationship() ? this->_GetPath() : SdfPath()); }
    EsfPrim AsPrim() const override { return Prim(_db, IsPrim() ? this->_GetPath() : SdfPath()); }
protected:
    EsfStage _GetStage() const override { return RigExecAdaptStandaloneStage(_db); }
    const RigExecSceneDb *_db;
private:
    bool _IsValid() const override {
        if (!_db->IsActive(this->_GetPath())) return false;
        if constexpr (std::is_base_of_v<EsfPrimInterface, Base>) return IsPrim();
        if constexpr (std::is_base_of_v<EsfAttributeInterface, Base>) return IsAttribute();
        if constexpr (std::is_base_of_v<EsfRelationshipInterface, Base>) return IsRelationship();
        if constexpr (std::is_base_of_v<EsfPropertyInterface, Base>) return IsAttribute() || IsRelationship();
        return IsPrim() || IsAttribute() || IsRelationship();
    }
    TfToken _GetName() const override { return this->_GetPath().GetNameToken(); }
    EsfPrim _GetPrim() const override { return Prim(_db, this->_GetPath().GetPrimPath()); }
    SdfPathVector _GetIncomingConnections() const override {
        return _db->IncomingConnections(this->_GetPath());
    }
    EsfSchemaConfigKey _GetSchemaConfigKey() const override {
        return this->CreateSchemaConfigKey(_db->SchemaKey(this->_GetPath().GetPrimPath()));
    }
    VtValue _GetMetadata(const TfToken &key) const override {
        const VtDictionary *metadata = nullptr;
        const auto prim = _db->prims.find(this->_GetPath());
        if (prim != _db->prims.end()) metadata = &prim->second.metadata;
        const auto attribute = _db->attributes.find(this->_GetPath());
        if (attribute != _db->attributes.end()) metadata = &attribute->second.metadata;
        const auto relationship = _db->relationships.find(this->_GetPath());
        if (relationship != _db->relationships.end()) metadata = &relationship->second.metadata;
        if (metadata) {
            const auto value = metadata->find(key.GetString());
            if (value != metadata->end()) return value->second;
        }
        const auto *field = SdfSchema::GetInstance().GetFieldDefinition(key);
        return field ? field->GetFallbackValue() : VtValue();
    }
    bool _IsValidMetadataKey(const TfToken &key) const override {
        const SdfSpecType type = IsAttribute() ? SdfSpecTypeAttribute :
            IsRelationship() ? SdfSpecTypeRelationship : SdfSpecTypePrim;
        const auto *spec = SdfSchema::GetInstance().GetSpecDefinition(type);
        return spec && spec->IsMetadataField(key);
    }
    TfType _GetMetadataValueType(const TfToken &key) const override {
        const auto *field = SdfSchema::GetInstance().GetFieldDefinition(key);
        if (!field) return TfType();
        const VtValue fallback = field->GetFallbackValue();
        return fallback.IsEmpty() ? TfType::Find<VtValue>() : fallback.GetType();
    }
};

template<class Base>
class PropertyImpl : public ObjectImpl<Base> {
public:
    using ObjectImpl<Base>::ObjectImpl;
private:
    TfToken _GetBaseName() const override {
        const std::string name = this->_GetPath().GetName();
        const auto split = name.rfind(':');
        return TfToken(split == std::string::npos ? name : name.substr(split + 1));
    }
    TfToken _GetNamespace() const override {
        const std::string name = this->_GetPath().GetName();
        const auto split = name.rfind(':');
        return split == std::string::npos ? TfToken() : TfToken(name.substr(0, split));
    }
};

class PrimImpl : public ObjectImpl<EsfPrimInterface> {
public:
    using ObjectImpl::ObjectImpl;
    bool IsPseudoRoot() const override { return _GetPath() == SdfPath::AbsoluteRootPath(); }
private:
    const TfTokenVector &_GetAppliedSchemas() const override {
        static const TfTokenVector empty;
        const auto prim = _db->prims.find(_GetPath());
        return prim == _db->prims.end() ? empty : prim->second.appliedSchemas;
    }
    EsfAttribute _GetAttribute(const TfToken &name) const override { return Attribute(_db, _GetPath().AppendProperty(name)); }
    EsfRelationship _GetRelationship(const TfToken &name) const override { return Relationship(_db, _GetPath().AppendProperty(name)); }
    EsfPrim _GetParent() const override { return Prim(_db, _GetPath().GetParentPath()); }
    TfType _GetType() const override {
        const auto prim = _db->prims.find(_GetPath());
        return prim == _db->prims.end() ? TfType() :
            UsdSchemaRegistry::GetTypeFromSchemaTypeName(prim->second.type);
    }
};

class QueryImpl : public EsfAttributeQueryInterface {
public:
    QueryImpl(const RigExecSceneDb *db, const SdfPath &path) : _db(db), _path(path) {}
private:
    bool _IsValid() const override { return _db->attributes.count(_path) && _db->IsActive(_path); }
    SdfPath _GetPath() const override { return _path; }
    void _Initialize() override {} // path lookup revives against the current Db row
    bool _Get(VtValue *value, UsdTimeCode time) const override { return _db->Get(_path, time, value); }
    std::optional<TsSpline> _GetSpline() const override { return std::nullopt; }
    bool _ValueMightBeTimeVarying() const override { return true; }
    bool _IsTimeVarying(UsdTimeCode from, UsdTimeCode to) const override {
        VtValue a, b;
        return _db->Get(_path, from, &a) != _db->Get(_path, to, &b) || a != b;
    }
    const RigExecSceneDb *_db;
    SdfPath _path;
};

class AttributeImpl : public PropertyImpl<EsfAttributeInterface> {
public:
    using PropertyImpl::PropertyImpl;
private:
    SdfValueTypeName _GetValueTypeName() const override {
        const auto value = _db->attributes.find(_GetPath());
        return value == _db->attributes.end() ? SdfValueTypeName() : value->second.type;
    }
    EsfAttributeQuery _GetQuery() const override { return {std::in_place_type<QueryImpl>, _db, _GetPath()}; }
    SdfPathVector _GetConnections() const override {
        const auto value = _db->attributes.find(_GetPath());
        return value == _db->attributes.end() ? SdfPathVector() : value->second.connections;
    }
};
class RelationshipImpl : public PropertyImpl<EsfRelationshipInterface> {
public:
    using PropertyImpl::PropertyImpl;
private:
    SdfPathVector _GetTargets() const override {
        const auto value = _db->relationships.find(_GetPath());
        return value == _db->relationships.end() ? SdfPathVector() : value->second.targets;
    }
};

EsfPrim Prim(const RigExecSceneDb *db, const SdfPath &path) { return {std::in_place_type<PrimImpl>, db, path}; }
EsfAttribute Attribute(const RigExecSceneDb *db, const SdfPath &path) { return {std::in_place_type<AttributeImpl>, db, path}; }
EsfProperty Property(const RigExecSceneDb *db, const SdfPath &path) { return {std::in_place_type<PropertyImpl<EsfPropertyInterface>>, db, path}; }
EsfRelationship Relationship(const RigExecSceneDb *db, const SdfPath &path) { return {std::in_place_type<RelationshipImpl>, db, path}; }
} // namespace

EsfStage RigExecAdaptStandaloneStage(const RigExecSceneDb *db) { return {std::in_place_type<StageImpl>, db}; }
EsfObject RigExecAdaptStandaloneObject(const RigExecSceneDb *db, const SdfPath &path) {
    return {std::in_place_type<ObjectImpl<EsfObjectInterface>>, db, path};
}
} // namespace rigExec

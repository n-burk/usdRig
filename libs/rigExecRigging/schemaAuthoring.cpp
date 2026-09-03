//
// Strict, schema-backed authoring for codeless RigExec schemas.
//
#include "schemaAuthoring.h"

#include "rigExecMath/avarScale.h"

#include "pxr/usd/sdf/schema.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primDefinition.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/schemaRegistry.h"

#include <stdexcept>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

namespace {

const TfToken _readPhaseMetadata("rigExecReadPhase");
const TfToken _controlType("RigExecControl");
const TfToken _jointType("RigExecJoint");
const TfToken _avarSx("avars:sx");
const TfToken _avarSy("avars:sy");
const TfToken _avarSz("avars:sz");

std::string
_Quoted(const TfToken &token)
{
    return "'" + token.GetString() + "'";
}

std::string
_At(const SdfPath &path)
{
    return " at " + path.GetString();
}

void
_RequireStageAndPath(const UsdStageRefPtr &stage, const SdfPath &path)
{
    if (!stage) {
        throw std::invalid_argument(
            "RigExecSchemaPrim requires a valid UsdStage");
    }
    if (path.IsEmpty() || !path.IsAbsolutePath() || !path.IsPrimPath() ||
        path == SdfPath::AbsoluteRootPath()) {
        throw std::invalid_argument(
            "RigExecSchemaPrim requires an absolute prim path, got " +
            path.GetString());
    }
}

const UsdPrimDefinition &
_RequireConcreteDefinition(const TfToken &schemaType)
{
    if (schemaType.IsEmpty()) {
        throw std::invalid_argument(
            "concrete schema type must not be empty");
    }
    const UsdPrimDefinition *definition =
        UsdSchemaRegistry::GetInstance().FindConcretePrimDefinition(schemaType);
    if (!definition) {
        throw std::runtime_error(
            "no registered concrete prim definition for " +
            _Quoted(schemaType) +
            "; register the RigExec schema plugin before authoring");
    }
    return *definition;
}

const UsdPrimDefinition &
_RequireAppliedDefinition(const TfToken &schemaIdentifier)
{
    if (schemaIdentifier.IsEmpty()) {
        throw std::invalid_argument(
            "applied API schema identifier must not be empty");
    }
    const UsdSchemaRegistry::SchemaInfo *schemaInfo =
        UsdSchemaRegistry::FindSchemaInfo(schemaIdentifier);
    if (!schemaInfo || schemaInfo->kind != UsdSchemaKind::SingleApplyAPI) {
        throw std::runtime_error(
            _Quoted(schemaIdentifier) +
            " is not a registered single-apply API schema identifier");
    }
    const UsdPrimDefinition *definition =
        UsdSchemaRegistry::GetInstance().FindAppliedAPIPrimDefinition(
            schemaIdentifier);
    if (!definition) {
        throw std::runtime_error(
            "no registered applied API prim definition for " +
            _Quoted(schemaIdentifier) +
            "; register the providing schema plugin before authoring");
    }
    return *definition;
}

void
_RequireExpectedType(
    const UsdPrim &prim, const TfToken &schemaType, const SdfPath &path)
{
    if (!prim) {
        throw std::runtime_error("no prim exists" + _At(path));
    }
    if (prim.GetTypeName() != schemaType) {
        throw std::invalid_argument(
            "expected " + _Quoted(schemaType) + _At(path) + ", found " +
            _Quoted(prim.GetTypeName()));
    }

    // A raw typeName opinion is legal USD even when its schema plugin is
    // absent. Require the recognized schema type as well, so a bare string can
    // never masquerade as a schema-backed prim.
    const TfToken resolvedType =
        prim.GetPrimTypeInfo().GetSchemaTypeName();
    if (resolvedType != schemaType) {
        throw std::runtime_error(
            "prim " + path.GetString() + " authors type " +
            _Quoted(schemaType) + " but OpenUSD resolves schema type " +
            _Quoted(resolvedType) +
            "; the concrete schema is not active in this stage");
    }
}

UsdPrimDefinition::Attribute
_RequireAttributeDefinition(const UsdPrim &prim, const TfToken &name)
{
    if (name.IsEmpty()) {
        throw std::invalid_argument("attribute name must not be empty");
    }
    const UsdPrimDefinition &definition = prim.GetPrimDefinition();
    const UsdPrimDefinition::Attribute attribute =
        definition.GetAttributeDefinition(name);
    if (attribute) {
        return attribute;
    }
    if (definition.GetRelationshipDefinition(name)) {
        throw std::invalid_argument(
            _Quoted(name) + " is a declared relationship, not an attribute" +
            _At(prim.GetPath()));
    }
    throw std::invalid_argument(
        "attribute " + _Quoted(name) + " is not declared by the composed "
        "schema definition" + _At(prim.GetPath()));
}

UsdPrimDefinition::Relationship
_RequireRelationshipDefinition(const UsdPrim &prim, const TfToken &name)
{
    if (name.IsEmpty()) {
        throw std::invalid_argument("relationship name must not be empty");
    }
    const UsdPrimDefinition &definition = prim.GetPrimDefinition();
    const UsdPrimDefinition::Relationship relationship =
        definition.GetRelationshipDefinition(name);
    if (relationship) {
        return relationship;
    }
    if (definition.GetAttributeDefinition(name)) {
        throw std::invalid_argument(
            _Quoted(name) + " is a declared attribute, not a relationship" +
            _At(prim.GetPath()));
    }
    throw std::invalid_argument(
        "relationship " + _Quoted(name) +
        " is not declared by the composed schema definition" +
        _At(prim.GetPath()));
}

bool
_IsControlOrJointAvarScale(const UsdPrim &prim, const TfToken &name)
{
    const TfToken typeName = prim.GetTypeName();
    return (typeName == _controlType || typeName == _jointType) &&
           (name == _avarSx || name == _avarSy || name == _avarSz);
}

}  // namespace

RigExecSchemaPrim
RigExecSchemaPrim::Define(
    UsdStageRefPtr stage, const SdfPath &path, const TfToken &schemaType)
{
    _RequireStageAndPath(stage, path);
    // Resolve before DefinePrim: an unavailable or misspelled schema must not
    // leave even a raw type-name prim behind.
    _RequireConcreteDefinition(schemaType);

    UsdPrim prim = stage->GetPrimAtPath(path);
    if (prim) {
        _RequireExpectedType(prim, schemaType, path);
    } else {
        prim = stage->DefinePrim(path, schemaType);
        if (!prim) {
            throw std::runtime_error(
                "failed to define " + _Quoted(schemaType) + _At(path));
        }
        _RequireExpectedType(prim, schemaType, path);
    }
    return RigExecSchemaPrim(std::move(stage), path, schemaType);
}

RigExecSchemaPrim
RigExecSchemaPrim::Get(
    UsdStageRefPtr stage, const SdfPath &path,
    const TfToken &expectedSchemaType)
{
    _RequireStageAndPath(stage, path);
    _RequireConcreteDefinition(expectedSchemaType);
    const UsdPrim prim = stage->GetPrimAtPath(path);
    _RequireExpectedType(prim, expectedSchemaType, path);
    return RigExecSchemaPrim(
        std::move(stage), path, expectedSchemaType);
}

bool
RigExecSchemaPrim::IsValid() const
{
    if (!_stage || _path.IsEmpty() || _schemaType.IsEmpty() ||
        !UsdSchemaRegistry::GetInstance().FindConcretePrimDefinition(
            _schemaType)) {
        return false;
    }
    const UsdPrim prim = _stage->GetPrimAtPath(_path);
    return prim && prim.GetTypeName() == _schemaType &&
           prim.GetPrimTypeInfo().GetSchemaTypeName() == _schemaType;
}

UsdPrim
RigExecSchemaPrim::_RequirePrim() const
{
    _RequireStageAndPath(_stage, _path);
    _RequireConcreteDefinition(_schemaType);
    const UsdPrim prim = _stage->GetPrimAtPath(_path);
    _RequireExpectedType(prim, _schemaType, _path);
    return prim;
}

UsdPrim
RigExecSchemaPrim::GetPrim() const
{
    return _RequirePrim();
}

bool
RigExecSchemaPrim::HasAPI(const TfToken &schemaIdentifier) const
{
    _RequireAppliedDefinition(schemaIdentifier);
    return _RequirePrim().HasAPI(schemaIdentifier);
}

void
RigExecSchemaPrim::ApplyAPI(const TfToken &schemaIdentifier) const
{
    const UsdPrimDefinition &apiDefinition =
        _RequireAppliedDefinition(schemaIdentifier);
    const UsdPrim prim = _RequirePrim();

    if (!prim.HasAPI(schemaIdentifier)) {
        std::string whyNot;
        if (!prim.CanApplyAPI(schemaIdentifier, &whyNot)) {
            throw std::invalid_argument(
                "cannot apply API " + _Quoted(schemaIdentifier) + _At(_path) +
                (whyNot.empty() ? std::string() : ": " + whyNot));
        }
        if (!prim.ApplyAPI(schemaIdentifier)) {
            throw std::runtime_error(
                "OpenUSD failed to apply API " + _Quoted(schemaIdentifier) +
                _At(_path));
        }
    }
    if (!prim.HasAPI(schemaIdentifier)) {
        throw std::runtime_error(
            "API " + _Quoted(schemaIdentifier) +
            " was not composed after ApplyAPI" + _At(_path));
    }

    // Verify that applying the API really extended this prim's composed
    // definition. This catches incomplete/mismatched plugin registration even
    // if an apiSchemas token was authored successfully.
    const UsdPrimDefinition &composed = prim.GetPrimDefinition();
    for (const TfToken &property : apiDefinition.GetPropertyNames()) {
        if (!composed.GetPropertyDefinition(property)) {
            throw std::runtime_error(
                "API " + _Quoted(schemaIdentifier) + " is applied" +
                _At(_path) + " but its declared property " +
                _Quoted(property) + " is absent from the composed definition");
        }
    }
}

void
RigExecSchemaPrim::SetAttribute(
    const TfToken &name, const VtValue &value, UsdTimeCode time) const
{
    const UsdPrim prim = _RequirePrim();
    const UsdPrimDefinition::Attribute definition =
        _RequireAttributeDefinition(prim, name);
    if (value.IsEmpty()) {
        throw std::invalid_argument(
            "cannot set declared attribute " + _Quoted(name) +
            " from an empty VtValue" + _At(_path));
    }

    const SdfValueTypeName expectedType = definition.GetTypeName();
    if (!expectedType.CanRepresent(value)) {
        throw std::invalid_argument(
            "attribute " + _Quoted(name) + _At(_path) + " requires " +
            expectedType.GetAsToken().GetString() + ", got " +
            value.GetTypeName());
    }
    if (definition.GetVariability() == SdfVariabilityUniform &&
        !time.IsDefault()) {
        throw std::invalid_argument(
            "uniform attribute " + _Quoted(name) + _At(_path) +
            " cannot be authored at a numeric time code");
    }

    VtValue authoredValue = value;
    if (_IsControlOrJointAvarScale(prim, name)) {
        const double scale = value.UncheckedGet<double>();
        if (!RigExecIsFiniteAvarScale(scale)) {
            throw std::invalid_argument(
                "attribute " + _Quoted(name) + _At(_path) +
                " requires a finite control/joint avar scale");
        }
        authoredValue = VtValue(RigExecNormalizeAvarScale(scale));
    }

    // GetAttribute returns a schema-backed property proxy for a declared
    // builtin even before it has an authored property spec. Set() asks USD to
    // materialize that declaration in the current edit target.
    const UsdAttribute attribute = prim.GetAttribute(name);
    if (!attribute || attribute.IsCustom() ||
        attribute.GetTypeName() != expectedType ||
        attribute.GetVariability() != definition.GetVariability()) {
        throw std::runtime_error(
            "OpenUSD did not expose " + _Quoted(name) +
            " with the type, variability, and non-custom status declared by "
            "the composed schema" + _At(_path));
    }
    if (!attribute.Set(authoredValue, time)) {
        throw std::runtime_error(
            "failed to set declared attribute " + _Quoted(name) +
            _At(_path));
    }
}

void
RigExecSchemaPrim::SetRelationship(
    const TfToken &name, const SdfPathVector &targets) const
{
    const UsdPrim prim = _RequirePrim();
    _RequireRelationshipDefinition(prim, name);
    const UsdRelationship relationship = prim.GetRelationship(name);
    if (!relationship || relationship.IsCustom()) {
        throw std::runtime_error(
            "OpenUSD did not expose the declared non-custom relationship " +
            _Quoted(name) + _At(_path));
    }
    if (!relationship.SetTargets(targets)) {
        throw std::runtime_error(
            "failed to set declared relationship " + _Quoted(name) +
            _At(_path));
    }
}

void
RigExecSchemaPrim::ClearAttribute(const TfToken &name) const
{
    const UsdPrim prim = _RequirePrim();
    _RequireAttributeDefinition(prim, name);
    const UsdAttribute attribute = prim.GetAttribute(name);
    if (!attribute) {
        throw std::runtime_error(
            "OpenUSD did not expose the declared attribute " + _Quoted(name) +
            _At(_path));
    }
    if (!attribute.Clear()) {
        throw std::runtime_error(
            "failed to clear declared attribute " + _Quoted(name) +
            _At(_path));
    }
}

void
RigExecSchemaPrim::SetReadPhase(
    const TfToken &propertyName, const std::string &phase) const
{
    if (phase.empty()) {
        throw std::invalid_argument("read phase must not be empty");
    }
    const UsdPrim prim = _RequirePrim();
    const UsdPrimDefinition &definition = prim.GetPrimDefinition();
    const UsdPrimDefinition::Property property =
        definition.GetPropertyDefinition(propertyName);
    if (!property) {
        throw std::invalid_argument(
            "property " + _Quoted(propertyName) +
            " is not declared by the composed schema definition" +
            _At(_path));
    }
    const SdfSchema &sdfSchema = SdfSchema::GetInstance();
    if (!sdfSchema.IsRegistered(_readPhaseMetadata) ||
        !sdfSchema.IsValidFieldForSpec(
            _readPhaseMetadata, property.GetSpecType())) {
        throw std::runtime_error(
            "metadata field 'rigExecReadPhase' is not registered for this "
            "property kind; register the RigExec schema plugin before "
            "authoring");
    }

    bool authored = false;
    if (property.IsAttribute()) {
        const UsdAttribute attribute = prim.GetAttribute(propertyName);
        authored = attribute && attribute.SetMetadata(
            _readPhaseMetadata, VtValue(phase));
    } else if (property.IsRelationship()) {
        const UsdRelationship relationship = prim.GetRelationship(propertyName);
        authored = relationship && relationship.SetMetadata(
            _readPhaseMetadata, VtValue(phase));
    }
    if (!authored) {
        throw std::runtime_error(
            "failed to set rigExecReadPhase on declared property " +
            _Quoted(propertyName) + _At(_path));
    }
}

}  // namespace rigExec

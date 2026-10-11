#ifndef RIGEXEC_GRAPH_SCENE_INPUT_TYPES_H
#define RIGEXEC_GRAPH_SCENE_INPUT_TYPES_H
#include "pxr/usd/sdf/types.h"
#include "pxr/base/vt/types.h"
#include <type_traits>
namespace rigExec {
PXR_NAMESPACE_USING_DIRECTIVE
/// Expected native typed read type, independent of a malformed source's type.
template<class T> SdfValueTypeName RigExecSceneInputType() {
    if constexpr(std::is_same_v<T,double>)return SdfValueTypeNames->Double;
    else if constexpr(std::is_same_v<T,float>)return SdfValueTypeNames->Float;
    else if constexpr(std::is_same_v<T,bool>)return SdfValueTypeNames->Bool;
    else if constexpr(std::is_same_v<T,int>)return SdfValueTypeNames->Int;
    else if constexpr(std::is_same_v<T,GfVec3d>)return SdfValueTypeNames->Double3;
    else if constexpr(std::is_same_v<T,GfVec3f>)return SdfValueTypeNames->Float3;
    else if constexpr(std::is_same_v<T,GfMatrix4d>)return SdfValueTypeNames->Matrix4d;
    else if constexpr(std::is_same_v<T,TfToken>)return SdfValueTypeNames->Token;
    else if constexpr(std::is_same_v<T,VtFloatArray>)return SdfValueTypeNames->FloatArray;
    else if constexpr(std::is_same_v<T,VtDoubleArray>)return SdfValueTypeNames->DoubleArray;
    else if constexpr(std::is_same_v<T,VtIntArray>)return SdfValueTypeNames->IntArray;
    else if constexpr(std::is_same_v<T,VtVec2fArray>)return SdfValueTypeNames->Float2Array;
    else if constexpr(std::is_same_v<T,VtVec3fArray>)return SdfValueTypeNames->Float3Array;
    else if constexpr(std::is_same_v<T,VtVec3dArray>)return SdfValueTypeNames->Double3Array;
    else if constexpr(std::is_same_v<T,VtMatrix4dArray>)return SdfValueTypeNames->Matrix4dArray;
    else if constexpr(std::is_same_v<T,VtTokenArray>)return SdfValueTypeNames->TokenArray;
    else if constexpr(std::is_same_v<T,VtBoolArray>)return SdfValueTypeNames->BoolArray;
    else return {};
}
}
#endif

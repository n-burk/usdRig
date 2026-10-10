#include "weightProgram.h"
#include <algorithm>
#include <cmath>
namespace rigExec {
namespace {
bool Resolve(const std::vector<RigExecWeightRecord> &, const std::vector<RigExecWeightFieldInputs> &, size_t, size_t,
             const RigExecWeightPointView *, RigExecWeightFieldWorkspace &, size_t, std::vector<float> *, std::string *);
bool Volume(const std::vector<RigExecWeightRecord> &records, const std::vector<RigExecWeightFieldInputs> &inputs, size_t id, size_t count,
            const RigExecWeightPointView *current, RigExecWeightFieldWorkspace &workspace, size_t depth, std::vector<float> *out,
            std::string *error)
{
    if (id >= records.size() || id >= inputs.size()) { *error = "weight input unavailable"; return false; }
    const auto &object = records[id];
    const auto &input = inputs[id];
    const auto &who = object.name;
    const auto read = [&](int member, float fallback) {
        return member >= 0 && member < 19 ? input.scalars[size_t(member)] : fallback;
    };
    if (object.kind == 2) {
        auto &child = workspace.children[depth];
        const bool combined = RigExecCombineWeightFieldsStreamed(
            RigExecWeightCombine(object.combineMode),object.inputs.size(),count,
            [&](size_t k,const float **data,size_t *size) {
                if (!Resolve(records,inputs,size_t(object.inputs[k]),count,current,
                    workspace,depth+1,&child,error)) return false;
                *data=child.data();*size=child.size();return true;
            },out);
        if (!combined) {
            if(error->empty())*error = who + ": combine inputs disagree on element count";
            return false;
        }
        const float strength = read(4, 1), invert = read(5, 0);
        for (float &w : *out) w = (w + (1.0f - 2.0f * w) * invert) * strength;
        return true;
    }
    if (!input.hasPlacement) {
        *error = who + ": no resolved placement for this volume weight"; return false;
    }
    const GfMatrix4d rigid = input.placement.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        *error = who + ": degenerate volume placement"; return false;
    }
    GfMatrix4d worldToLocal = rigid.GetInverse();
    if (!object.phaseError.empty()) { *error = object.phaseError; return false; }
    const RigExecWeightPointInput *raw = nullptr;
    RigExecWeightPointView samples;
    const auto &target = input.phasedPoints[1];
    const auto &primary = input.phasedPoints[0];
    if (target.declared) {
        if (!target.available) { *error = who + ": could not read the points to sample"; return false; }
        samples = {target.data,target.count};
    } else if (object.samplesInFlight) {
        if (!current) { *error = who + ": rigExec:weightTarget reads `preceding` but no in-flight points were supplied"; return false; }
        samples = *current;
    } else if (primary.declared && primary.available) {
        samples = {primary.data,primary.count};
    } else {
        raw = primary.declared ? nullptr : &input.rawPoints[0];
        if (!raw || !raw->available) raw = &input.rawPoints[1];
        if (!raw->available) { *error = who + ": could not read the points to sample"; return false; }
        samples = {raw->data,raw->count};
    }    if (samples.count != count) { *error = who + ": sampled point count does not match the target"; return false; }
    RigExecFalloffParams params;
    params.falloffMin = read(6,0); params.falloffMax = read(7,1);
    params.invert = read(5,0); params.strength = read(4,1); params.curveData = object.falloffCurve.data(); params.curveCount = object.falloffCurve.size();
    const auto &axisName = input.axis;
    const auto &boundsName = input.bounds;
    const int axis = axisName == "x" ? 0 : axisName == "y" ? 1 : axisName == "z" ? 2 : -1;
    if (object.kind == 4) {
        if (axis < 0) { *error = who + ": unknown rigExec:planeAxis " + axisName; return false; }
        RigExecPlaneBounds bounds;
        const RigExecPlaneBounds *clip = nullptr;
        if (boundsName == "bounded") {
            bounds.extentU = read(17,1); bounds.extentV = read(18,1);
            for (float extent : {bounds.extentU,bounds.extentV}) if (!std::isfinite(extent) || extent <= 0) {
                *error = who + ": inputs:extentU/V must be finite and positive when rigExec:planeBounds is `bounded`"; return false;
            }
            clip = &bounds;
        } else if (boundsName != "unbounded") { *error = who + ": unknown rigExec:planeBounds " + boundsName; return false; }
        RigExecPlaneWeightField(samples, worldToLocal, axis, params, out, clip);
        return true;
    }
    const GfVec3f scale(read(8,1),read(9,1),read(10,1));
    for (int axis=0;axis<3;++axis) if (!std::isfinite(scale[axis]) || scale[axis] <= 0) {
        *error = who + ": inputs:scaleX/Y/Z must be finite and positive"; return false;
    }
    GfMatrix4d divide(1.0); divide.SetScale(GfVec3d(1.0/double(scale[0]),1.0/double(scale[1]),1.0/double(scale[2])));
    worldToLocal = worldToLocal * divide;
    if (object.kind == 3) {
        const GfVec3f positive(read(11,1),read(12,1),read(13,1));
        const GfVec3f negative(read(14,1),read(15,1),read(16,1));
        for (int axis=0;axis<3;++axis) if (!std::isfinite(positive[axis]) || positive[axis] <= 0 ||
            !std::isfinite(negative[axis]) || negative[axis] <= 0) {
            *error = who + ": signed axis scales must be finite and positive"; return false;
        }
        RigExecSphereWeightField(samples,worldToLocal,params,out,positive,negative); return true;
    }
    const auto curveInput = input.phasedPoints[2];
    const auto *curve = curveInput.declared ? nullptr : &input.rawPoints[2];
    const auto &curvePoints = curveInput.declared ? curveInput : *curve;
    if (!curvePoints.available || !curvePoints.count) {
        *error = who + ": rigExec:curve must name exactly one points source"; return false;
    }
    RigExecCurveWeightField(samples,{curvePoints.data,curvePoints.count},worldToLocal,
        params,out,&workspace.localCurve); return true;
}
bool Resolve(const std::vector<RigExecWeightRecord> &records, const std::vector<RigExecWeightFieldInputs> &inputs, size_t id, size_t count,
             const RigExecWeightPointView *current, RigExecWeightFieldWorkspace &workspace, size_t depth, std::vector<float> *out,
             std::string *error)
{
    if (id >= records.size() || id >= inputs.size()) { *error = "weight input unavailable"; return false; }
    const auto &object = records[id];
    const auto &input = inputs[id];
    const auto &who = object.name;
    if(input.blocked){*error="operation cycle";return false;}
    if (!object.staticError.empty()) { *error = object.staticError; return false; }
    if (object.kind >= 2) {
        if (!Volume(records,inputs,id,count,current,workspace,depth,out,error)) return false;
        for (float &w : *out) {
            if (!std::isfinite(w)) { *error = "non-finite weight on " + who; return false; }
            if (w < 0 || w > 1) {
                if (!object.clamp) { *error = "strict range violation on " + who; return false; }
                w = std::min(std::max(w,0.0f),1.0f);
            }
        }
        return true;
    }
    const float defaultWeight = input.scalars[0];
    if (object.kind == 1) {
        if (object.base >= 0) {
            if (!Resolve(records,inputs,size_t(object.base),count,current,workspace,depth,out,error)) return false;
        } else out->assign(count,1.0f);
        const float driver = input.scalars[1], scale = input.scalars[2], bias = input.scalars[3];
        for (size_t i=0;i<count;++i) {
            float w = ((*out)[i]*driver)*scale+bias;
            if (!std::isfinite(w)) { *error = "non-finite dynamic weight on " + who; return false; }
            if (w < 0 || w > 1) {
                if (!object.clamp) { *error = "strict range violation on " + who; return false; }
                w = std::min(std::max(w,0.0f),1.0f);
            }
            (*out)[i] = w;
        }
        return true;
    }
    const float *values=input.paintedArraysDeclared?input.paintedValues:object.values.data();
    const size_t valueCount=input.paintedArraysDeclared?input.paintedValueCount:object.values.size();
    const int *indices=input.paintedArraysDeclared?input.paintedIndices:object.indices.data();
    const size_t indexCount=input.paintedArraysDeclared?input.paintedIndexCount:object.indices.size();
    if (object.representation == 0) {
        if (valueCount != 0) { *error = "constant weight must not author values on " + who; return false; }
        out->assign(count,defaultWeight);
    } else if (object.representation == 1) {
        if (valueCount != count) { *error = "dense weight cardinality mismatch on " + who; return false; }
        if (defaultWeight != 0) { *error = "dense weight requires canonical defaultWeight 0 on " + who; return false; }
        if(count)out->assign(values,values+count);else out->clear();
    } else if (object.representation == 2) {
        if (indexCount != valueCount) { *error = "sparse index/value size mismatch on " + who; return false; }
        out->assign(count,defaultWeight);
        auto &seen=workspace.seen;
        seen.resize(count,0);
        if(++workspace.seenGeneration==0) {
            std::fill(seen.begin(),seen.end(),0);workspace.seenGeneration=1;
        }
        const uint32_t generation=workspace.seenGeneration;
        for (size_t i=0;i<valueCount;++i) {
            const int index = indices[i];
            if (index < 0 || size_t(index) >= count) { *error = "sparse index out of range on " + who; return false; }
            if (seen[size_t(index)]==generation) { *error = "duplicate sparse index on " + who; return false; }
            seen[size_t(index)] = generation; (*out)[size_t(index)] = values[i];
        }
    } else { *error = "unknown weight representation on " + who; return false; }
    for (float &w : *out) {
        if (!std::isfinite(w) || (!object.clamp && (w < 0 || w > 1))) {
            *error = "weight range violation on " + who; return false;
        }
        if (object.clamp) w = std::min(std::max(w,0.0f),1.0f);
    }
    return true;
}
}
bool RigExecRunWeightField(const std::vector<RigExecWeightRecord> &records,int root,
    const std::vector<RigExecWeightFieldInputs> &inputs,size_t count,
    const RigExecWeightPointView *current,RigExecWeightFieldWorkspace *workspace,
    std::vector<float> *result,std::string *error) {
    if (!result || !error || !workspace) return false;
    error->clear();
    workspace->children.resize(records.size()+1);
    const bool ok = root >= 0 && Resolve(records,inputs,size_t(root),count,current,
        *workspace,0,result,error);
    if (!ok) { result->clear(); if(error->empty())*error="weight input unavailable"; }
    return ok;
}
bool RigExecRunWeightField(const std::vector<RigExecWeightRecord> &records,int root,
    const std::vector<RigExecWeightFieldInputs> &inputs,size_t count,
    const std::vector<GfVec3f> *current,std::vector<float> *result,std::string *error) {
    RigExecWeightFieldWorkspace workspace;
    const RigExecWeightPointView view=current?RigExecWeightPointView{current->data(),current->size()}:RigExecWeightPointView{};
    return RigExecRunWeightField(records,root,inputs,count,current?&view:nullptr,&workspace,result,error);
}
bool RigExecRunEffectiveWeightField(const std::vector<RigExecWeightRecord> &records,int root,
    const std::vector<RigExecWeightFieldInputs> &inputs,size_t count,
    const std::vector<GfVec3f> *current,float defaultWeight,
    std::vector<float> *result,std::string *error) {
    if(root>=0)return RigExecRunWeightField(records,root,inputs,count,current,result,error);
    if(!result || !error)return false;
    error->clear();
    if(!std::isfinite(defaultWeight) || defaultWeight<0.0f || defaultWeight>1.0f) {
        result->clear();*error="inputs:defaultWeight must be finite and in [0, 1]";return false;
    }
    result->assign(count,defaultWeight);return true;
}
RigExecWeightPacket RigExecRunWeightPacket(const RigExecWeightRecord &record,
    const RigExecWeightPacketInputs &input) {
    if (record.kind == 0) return RigExecBuildStaticWeightPacket(input.painted,input.workspace);
    if (record.kind == 1) return RigExecBuildDynamicWeightPacket(input.dynamic,input.base);
    if (record.kind == 2) return RigExecBuildCombineWeightPacket(record.representationToken,
        record.rangePolicy,record.combineToken,input.borrowedInputs?*input.borrowedInputs:input.inputs,
        input.targetCount,input.strength,input.invert,input.workspace);
    if (record.kind >= 3 && record.kind <= 5) return RigExecBuildVolumeWeightPacket(record.type,input.volume);
    return RigExecWeightPacket();
}
std::vector<float> RigExecBakeWeightFalloff(const TfToken &profile,const TsSpline *spline) {
    if (profile == "linear") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    if (profile == "smooth") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
    }
    if (profile == "easeIn") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseIn);
    }
    if (profile == "easeOut") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseOut);
    }
    if (profile == "constant") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Constant);
    }
    if (profile != "curve") {
        return {};  // unknown token: linear, never coerced to a preset
    }

    if (!spline) {
        // `curve` with nothing drawn is linear, not empty: the profile
        // token is a promise about SHAPE, and an author who selects it
        // before touching the editor should see the identity ramp.
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    std::vector<float> lut(RigExecFalloffLutSize);
    for (size_t i = 0; i < RigExecFalloffLutSize; ++i) {
        const double x = double(i) / double(RigExecFalloffLutSize - 1);
        float value = 0.0f;
        // Ts extrapolates HELD outside the authored knot range, so a
        // curve drawn over a shorter span still yields a total field.
        if (!spline->Eval(x, &value) || !std::isfinite(value)) {
            value = float(x);
        }
        lut[i] = value;
    }
    return lut;
}

}

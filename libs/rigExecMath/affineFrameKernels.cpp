// Numerical contracts and primary sources: docs/references.md.
#include "affineFrameKernels.h"
#include <stdexcept>
namespace rigExec {
using Kernel = RigExecGfAffineFrameKernel;
GfMatrix4d RigExecComputeArmatureParent(const RigExecAffineFrameInputs &inputs) {return Kernel::ArmatureParent(inputs);}
GfMatrix4d RigExecComputeBoneFrame(const RigExecAffineFrameInputs &inputs) {return Kernel::BoneFrame(inputs);}
GfMatrix4d RigExecComputeSkinInfluence(const RigExecAffineFrameInputs &inputs) {return Kernel::SkinInfluence(inputs);}
GfMatrix4d RigExecComputeMappedFrame(const RigExecAffineFrameInputs &inputs) {return Kernel::MappedFrame(inputs);}
GfMatrix4d RigExecComputeCopyTransforms(const RigExecAffineFrameInputs &inputs) {return Kernel::CopyTransforms(inputs);}
GfMatrix4d RigExecComputeConstraintFrame(const RigExecAffineFrameInputs &inputs) {
    const char *failure=nullptr;
    const GfMatrix4d result=Kernel::ConstraintFrame(inputs,&failure);
    if(failure)throw std::runtime_error(failure);
    return result;
}
}

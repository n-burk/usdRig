// rigExecSampler: feeds a .rigexec's inputs from the stage it plays over.
// A .rigexec's inputs are attributes of the stage it was baked from, each
// starting at its value at the bake time. Playing the binary along the
// stage's timeline means handing the reader, at every new time, the values
// the stage holds there. Only the inputs the file marks Animated can hold
// another value at another time, so only those are read: raw typed Gets of
// the attribute itself (the runtime performs the connection walks). A time
// equal to the last one makes no calls, so the reader's closure sees time
// move exactly when it did.
// The sampler holds attribute handles, no locks and no shared state: one
// sampler per reader, used on the thread that owns both.
#ifndef RIGEXEC_SAMPLER_INPUT_SAMPLER_H
#define RIGEXEC_SAMPLER_INPUT_SAMPLER_H

#include "rigExecRuntime/runtime.h"

#include "pxr/base/tf/type.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstddef>
#include <string>
#include <vector>

namespace rigExec {

/// The value type an input of \p tag holds, the way the bake typed its
/// slot: the attribute's own scalar type, any role.
PXR_NS::TfType RigExecInputTagType(RrInputTag tag);

/// The name of \p tag's type as a stage spells it ("double", "matrix4d").
const char *RigExecInputTagName(RrInputTag tag);

/// \p value as an input value of \p tag, for SetSampledInputAt. False when
/// \p value does not hold exactly RigExecInputTagType(tag), and for a Token
/// input, which is set by its text (SetInputToken).
bool RigExecInputValueFrom(const PXR_NS::VtValue &value, RrInputTag tag,
                           RrInputValue *out);

/// Sets input \p index of \p reader, named \p name and of type \p tag, to
/// \p attribute's typed value at \p time, as Apply sets an Animated input:
/// SetSampledInputAt, which takes a non-finite value too, SetInputToken for
/// a Token, and ClearInputAt when the read fails. False with the reader's
/// reason when it refuses.
bool RigExecSampleInputAt(const PXR_NS::UsdAttribute &attribute,
                          size_t index, const std::string &name,
                          RrInputTag tag, PXR_NS::UsdTimeCode time,
                          RigExecRuntimeReader *reader, std::string *error);

class RigExecInputSampler {
public:
    /// Resolves every listed scalar input of \p reader to the attribute of
    /// \p stage at the input's path. An input the stage lacks, or whose
    /// attribute holds another value type, keeps its bake-time default and
    /// gets one warning (GetWarnings). Notes the inputs the file marks
    /// Animated. Array inputs are not sampled: they keep their value until
    /// something sets them. The reader's defaults count as sampled at its
    /// bake time. False with the reason when \p stage is null.
    bool Bind(const PXR_NS::UsdStagePtr &stage,
              const RigExecRuntimeReader &reader, std::string *error);

    /// At \p time: a typed read of every bound Animated input, set on
    /// \p reader as the stage holds it (SetSampledInputAt, which takes a
    /// non-finite value too), then TouchAnimatedInputs. An input whose read
    /// fails there (a blocked sample; UsdTimeCode::Default, a time like any
    /// other, on an attribute keyed alone) holds no value (ClearInputAt),
    /// so the binary reads it as the evaluators read the stage. No call at
    /// all when \p time is the time of the last Apply, or the bake time
    /// right after Bind. False with the reader's reason when it refuses a
    /// value; the next Apply samples again. \p sampled, when given, says
    /// whether this call sampled (false for the no-call case).
    bool Apply(PXR_NS::UsdTimeCode time, RigExecRuntimeReader *reader,
               std::string *error, bool *sampled = nullptr);

    /// The next Apply samples whatever its time.
    void Invalidate() { _sampled = false; }

    /// One line per input Bind could not resolve, naming it and why.
    const std::vector<std::string> &GetWarnings() const { return _warnings; }

    /// The inputs Apply reads.
    size_t GetAnimatedCount() const { return _animated.size(); }

private:
    struct _Bound {
        size_t index = 0;
        std::string name;
        RrInputTag type = RrInputTag::Double;
        PXR_NS::UsdAttribute attribute;
    };

    std::vector<_Bound> _animated;
    std::vector<std::string> _warnings;
    PXR_NS::UsdTimeCode _last;
    bool _sampled = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_SAMPLER_INPUT_SAMPLER_H

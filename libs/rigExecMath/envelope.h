//
// Exact common-envelope blending shared by every mover output domain.
//
#ifndef RIGEXEC_MATH_ENVELOPE_H
#define RIGEXEC_MATH_ENVELOPE_H

namespace rigExec {

/// Blends a full-strength candidate back toward its preceding revision.
///
/// The explicit endpoint branches are part of the mover contract, not merely
/// an optimization.  `preceding + (full - preceding) * 1` can cancel to a
/// value different from `full` (and can overflow in the subtraction), while a
/// zero-weight arithmetic blend can disturb signed zero or turn an infinite
/// dormant candidate into NaN.  Normalized common envelopes therefore return
/// the selected endpoint without doing any interpolation arithmetic.
template <class T, class Weight>
T
RigExecBlendEnvelope(const T &preceding, const T &full, Weight weight)
{
    if (weight <= Weight(0)) {
        return preceding;
    }
    if (weight >= Weight(1)) {
        return full;
    }
    return preceding + (full - preceding) * weight;
}

}  // namespace rigExec

#endif  // RIGEXEC_MATH_ENVELOPE_H

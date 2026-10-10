// .rigexec time-variance audit: the static data a bake holds at its one time
// while the stage animates the source. A binary replays such a datum at the
// bake time whatever time a client drives it to, so a rig with one is exact
// at the bake time (and under input drags) and nowhere else on its
// timeline. A report, never a refusal.
#ifndef RIGEXEC_BAKE_STATIC_REPORT_H
#define RIGEXEC_BAKE_STATIC_REPORT_H

#include <string>
#include <vector>

namespace rigExec {

class RigExecRigEvaluator;

/// One static datum whose source the stage animates.
struct RigExecBakeStaticEntry {
    /// The datum, as "<kind> <owner path>": "xform base", "native frame"
    /// and "delta base" (a transform a constraint reads off the stage),
    /// "constraint arrays", "chain base" and "derived base" (a point
    /// chain's authored points), "blend sample points", "blend sample
    /// layout" (a sparse sample's offsets or point indices), "ribbon
    /// points", "revision binding" (a mover's cage, surface, driver curve,
    /// bind coordinates or topology), "revision read" (any other path read
    /// a mover's assembly makes at the time that no input slot evaluates:
    /// a structural token, lattice divisions, a wire's dropoff, extent
    /// widths), and "weight object <path> oracle" or "weight object <path>
    /// gather". A plugin mover's own reads are not listed.
    std::string field;
    /// The animated attribute, or for a transform the prim whose transform
    /// might vary.
    std::string source;
};

/// Lists every static datum of \p evaluator's baked program whose source
/// the stage animates, sorted by field then source, without duplicates. A
/// transform source is the prim or any ancestor below the asset root whose
/// transform might vary; any other source is an attribute that is animated
/// (time samples, a spline, or a value that might vary) or connected.
///
/// Compiles when no program stands; never evaluates. False with the reason
/// when the rig does not compile, has no compiled program for the epoch,
/// or holds interactive overrides.
bool RigExecBakeStaticReport(RigExecRigEvaluator &evaluator,
                             std::vector<RigExecBakeStaticEntry> *entries,
                             std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_STATIC_REPORT_H

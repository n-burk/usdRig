// Private evaluator implementation types and helpers.
#ifndef RIGEXEC_RIG_EVALUATOR_PROPERTYBINDINGS_H
#define RIGEXEC_RIG_EVALUATOR_PROPERTYBINDINGS_H

#include "rigEvaluatorInternal.h"
#include "pxr/usd/usd/attributeQuery.h"

namespace rigExec {

// Bound property inputs and cached chain results. Notices mark affected chains
// stale; a new binding epoch drops the bindings wholesale.
struct RigExecPropertyChainBindings
{
    /// One input the revision loop reads by value.
    struct Input {
        UsdAttribute attribute;
        UsdAttributeQuery query;
        SdfPath path;
        /// Authored connections make the read a WALK over the connection
        /// chain, which only RigExecResolvedInputs::GetAttribute knows how
        /// to perform; such an input is handed back to it unchanged.
        bool connected = false;
        /// Whether the attribute cannot change until the stage does -- no
        /// authored connections, no time samples, no time-varying opinion --
        /// which is the admission test RigExecStaticInputCache applies, and
        /// then `constantValue` is what it read once. Exactly as safe as
        /// that cache and for the same reasons: a standing property override
        /// is consulted FIRST and outranks it, and a stage edit on the
        /// attribute rebinds the chain.
        bool constant = false;
        VtValue constantValue;
        explicit operator bool() const { return bool(attribute); }
    };

    /// One revision of one chain, in the chain's own order.
    struct Revision {
        UsdPrim moverPrim;
        SdfPathVector weightObjects;
        Input enabled;
        Input defaultWeight;
        Input operation;
        Input value;
        Input minimum;
        Input maximum;
        Input keys;
        Input tangents;
    };

    /// One target, in _propertyChainOrder's order. An entry whose target
    /// attribute did not resolve carries an invalid `target`, which is the
    /// same thing the per-frame lookup used to report.
    struct Chain {
        SdfPath targetPath;
        UsdAttribute target;
        UsdAttributeQuery targetQuery;
        SdfValueTypeName valueType;
        std::vector<Revision> revisions;

        // What the chain's answer can depend on, so a frame in which none of
        // it moved republishes the last answer instead of recomputing it.
        //  * `watch`: every input attribute, and every attribute along an
        //    input's connection chain -- where an interactive override can
        //    stand (the phased readers' hops and the target are watched for
        //    overrides too, below);
        //  * `upstream`: the chains whose targets are among those
        //    attributes;
        //  * `varying`: whether any of them, or the target's own authored
        //    base, can change with time;
        //  * `alwaysDirty`: a weight object is read through its own
        //    resolution, which this does not follow.
        std::vector<SdfPath> watch;
        std::vector<size_t> upstream;
        bool varying = false;
        bool alwaysDirty = false;

        // What a stage edit has to reach to make the binding wrong, beside
        // `watch` and `targetPath`: every mover prim (its inputs, the
        // weight-object relationship, the prim itself), and each connection
        // source an input's walk named but could not find, which authoring
        // it would bring into the walk. `stale` is a notice's verdict,
        // consumed by the next run, which rebinds the chain from scratch.
        std::vector<SdfPath> moverPaths;
        std::vector<SdfPath> missingSources;
        bool stale = false;

        // Operator inputs that read this chain at a phase
        // (RigExecPhasedConnection): each is published on the consumer with
        // the value after `applied` revisions, cast to the consumer's own
        // type -- unless an interactive override stands on one of its
        // `hops`, which the overlay walk then meets instead. An override on
        // the target is what a `final` reader gets.
        struct Phased {
            SdfPath consumer;
            SdfValueTypeName consumerType;
            size_t applied = 0;
            SdfPathVector hops;
            bool final = false;
        };
        std::vector<Phased> phased;

        // The last run: whether it published, what, and the diagnostics it
        // pushed, all replayed verbatim when the chain is clean. lastPhased
        // parallels `phased`, empty where nothing was published.
        bool cached = false;
        bool published = false;
        VtValue lastValue;
        std::vector<VtValue> lastPhased;
        std::vector<std::string> lastDiagnostics;
        bool changedThisRun = false;
    };

    std::vector<Chain> chains;
    /// Whether any chain above is stale.
    bool anyStale = false;

    // The interactive overrides and the time the last run saw.
    bool haveLast = false;
    UsdTimeCode lastTime;
    std::unordered_map<SdfPath, VtValue, SdfPath::Hash> lastOverrides;
};

} // namespace rigExec

#endif

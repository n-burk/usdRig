//
// Proving a cone: RIGEXEC_BAKED_VERIFY_CONES.
//
// Cone re-execution is an argument -- "nothing outside the closure could
// have moved, so last run's values are this run's" -- and an argument about
// floating-point state is worth exactly what a machine can check of it. So
// this file lets one frame run twice from one starting point: the cone run's
// whole answer is shadowed, the starting point is put back, every step runs,
// and the two answers are compared slot by slot, counter by counter and
// diagnostic by diagnostic.
//
// Nothing here is on a production path. It is deliberately a deep copy of
// everything a step can touch, because a shadow that left a field out would
// agree with the cone run about the one thing the cone got wrong.
//
#include "bakedProgramImpl.h"

#include "pxr/base/tf/getenv.h"

#include <cstdio>
#include <string>
#include <vector>

namespace rigExec {

bool
RigExecBakedVerifyConesRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_BAKED_VERIFY_CONES", false);
    return requested;
}

namespace {

/// Appends one line naming \p what, and counts it.
void
Differ(std::vector<std::string> *differences, size_t *count,
       const std::string &what)
{
    ++*count;
    if (differences && differences->size() < 64) {
        differences->push_back("baked cone mismatch: " + what);
    }
}

template <class T>
void
CompareValue(std::vector<std::string> *differences, size_t *count,
             const std::string &what, const T &shadow, const T &current)
{
    if (!(shadow == current)) {
        Differ(differences, count, what);
    }
}

template <class T>
void
CompareVector(std::vector<std::string> *differences, size_t *count,
              const std::string &what, const std::vector<T> &shadow,
              const std::vector<T> &current)
{
    if (shadow.size() != current.size()) {
        Differ(differences, count, what + " size");
        return;
    }
    for (size_t i = 0; i < shadow.size(); ++i) {
        if (!(shadow[i] == current[i])) {
            Differ(differences, count, what + "[" + std::to_string(i) + "]");
            return;
        }
    }
}

void
CaptureRevision(const RigExecBakedProgramImpl::GeomRevision &revision,
                RigExecBakedRunShadow::RevisionState *state)
{
    state->output = revision.output;
    state->influences = revision.influences;
    state->rows = revision.rows;
    state->envelope = revision.envelope;
    state->palette = revision.palette;
    state->parameters = revision.parameters;
    state->lastParameters = revision.lastParameters;
    state->status = revision.status;
    state->lastStatus = revision.lastStatus;
    state->resultStatus = revision.resultStatus;
    state->transform = revision.transform;
    state->precedingCount = revision.precedingCount;
    state->currentSource = revision.currentSource;
    state->haveTransform = revision.haveTransform;
    state->ran = revision.ran;
    state->executed = revision.executed;
    state->influencesValid = revision.influencesValid;
    state->influencesChanged = revision.influencesChanged;
    state->staticDirty = revision.staticDirty;
    state->partitionStale = revision.partitionStale;
    state->layoutUsable = revision.layoutUsable;
    state->envelopeOk = revision.envelopeOk;
    state->fullStrength = revision.fullStrength;
    state->chunks.resize(revision.chunks.size());
    for (size_t k = 0; k < revision.chunks.size(); ++k) {
        state->chunks[k].transforms = revision.chunks[k].transforms;
        state->chunks[k].keyChanged = revision.chunks[k].keyChanged;
        state->chunks[k].ok = revision.chunks[k].ok;
    }
}

void
RestoreRevision(const RigExecBakedRunShadow::RevisionState &state,
                RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->output = state.output;
    revision->influences = state.influences;
    revision->rows = state.rows;
    revision->envelope = state.envelope;
    revision->palette = state.palette;
    revision->parameters = state.parameters;
    revision->lastParameters = state.lastParameters;
    revision->status = state.status;
    revision->lastStatus = state.lastStatus;
    revision->resultStatus = state.resultStatus;
    revision->transform = state.transform;
    revision->precedingCount = state.precedingCount;
    revision->currentSource = state.currentSource;
    revision->haveTransform = state.haveTransform;
    revision->ran = state.ran;
    revision->executed = state.executed;
    revision->influencesValid = state.influencesValid;
    revision->influencesChanged = state.influencesChanged;
    revision->staticDirty = state.staticDirty;
    revision->partitionStale = state.partitionStale;
    revision->layoutUsable = state.layoutUsable;
    revision->envelopeOk = state.envelopeOk;
    revision->fullStrength = state.fullStrength;
    for (size_t k = 0; k < revision->chunks.size() && k < state.chunks.size();
         ++k) {
        revision->chunks[k].transforms = state.chunks[k].transforms;
        revision->chunks[k].keyChanged = state.chunks[k].keyChanged;
        revision->chunks[k].ok = state.chunks[k].ok;
    }
}

void
CompareRevision(std::vector<std::string> *differences, size_t *count,
                const std::string &where,
                const RigExecBakedRunShadow::RevisionState &shadow,
                const RigExecBakedProgramImpl::GeomRevision &revision)
{
    CompareVector(differences, count, where + " output", shadow.output,
                  revision.output);
    CompareVector(differences, count, where + " influences", shadow.influences,
                  revision.influences);
    CompareVector(differences, count, where + " envelope", shadow.envelope,
                  revision.envelope);
    CompareValue(differences, count, where + " parameters", shadow.parameters,
                 revision.parameters);
    CompareValue(differences, count, where + " status", shadow.status,
                 revision.status);
    CompareValue(differences, count, where + " lastParameters",
                 shadow.lastParameters, revision.lastParameters);
    CompareValue(differences, count, where + " lastStatus", shadow.lastStatus,
                 revision.lastStatus);
    CompareValue(differences, count, where + " resultStatus",
                 shadow.resultStatus, revision.resultStatus);
    CompareValue(differences, count, where + " currentSource",
                 shadow.currentSource, revision.currentSource);
    CompareValue(differences, count, where + " ran", shadow.ran, revision.ran);
    CompareValue(differences, count, where + " executed", shadow.executed,
                 revision.executed);
    CompareValue(differences, count, where + " influencesValid",
                 shadow.influencesValid, revision.influencesValid);
    CompareValue(differences, count, where + " influencesChanged",
                 shadow.influencesChanged, revision.influencesChanged);
    CompareValue(differences, count, where + " staticDirty",
                 shadow.staticDirty, revision.staticDirty);
    CompareValue(differences, count, where + " transform", shadow.transform,
                 revision.transform);
    for (size_t k = 0;
         k < shadow.chunks.size() && k < revision.chunks.size(); ++k) {
        const std::string what =
            where + " chunk " + std::to_string(k);
        CompareValue(differences, count, what + " ok", shadow.chunks[k].ok,
                     revision.chunks[k].ok);
        CompareVector(differences, count, what + " transforms",
                      shadow.chunks[k].transforms,
                      revision.chunks[k].transforms);
    }
}

}  // namespace

void
RigExecBakedRunShadow::Capture(const RigExecBakedProgramImpl &program)
{
    avars = program.avars;
    posedM = program.posedM;
    finalMatrix = program.finalMatrix;
    baseMatrix = program.baseMatrix;
    base = program.base;
    fin = program.fin;
    aggregates = program.aggregates;
    constraintDeltas = program.constraintDeltas;
    avarsDisturbed = program.avarsDisturbed;

    solvers.resize(program.solvers.size());
    for (size_t s = 0; s < program.solvers.size(); ++s) {
        solvers[s].outFrames = program.solvers[s].outFrames;
        solvers[s].outPresent = program.solvers[s].outPresent;
        solvers[s].fallbackJoints = program.solvers[s].fallbackJoints;
    }
    commits.resize(program.commits.size());
    for (size_t c = 0; c < program.commits.size(); ++c) {
        commits[c].present = program.commits[c].present;
        commits[c].deltaOk = program.commits[c].deltaOk;
        commits[c].frames = program.commits[c].frames;
        commits[c].staged = program.commits[c].staged;
        commits[c].deltas = program.commits[c].deltas;
        commits[c].outcome = program.commits[c].outcome;
        commits[c].sources = program.commits[c].sources;
        commits[c].abandoned = program.commits[c].abandoned;
    }
    chains.resize(program.chains.size());
    for (size_t c = 0; c < program.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = program.chains[c];
        chains[c].lastBase = chain.lastBase;
        chains[c].result = chain.result;
        chains[c].spare = chain.spare;
        chains[c].haveResult = chain.haveResult;
        chains[c].haveBase = chain.haveBase;
        chains[c].baseDirty = chain.baseDirty;
        chains[c].revisions.resize(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            CaptureRevision(chain.revisions[r], &chains[c].revisions[r]);
        }
        chains[c].derived.resize(chain.derived.size());
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            CaptureRevision(chain.derived[d].revision,
                            &chains[c].derived[d].revision);
            chains[c].derived[d].result = chain.derived[d].result;
            chains[c].derived[d].spare = chain.derived[d].spare;
            chains[c].derived[d].lastBase = chain.derived[d].lastBase;
            chains[c].derived[d].haveResult = chain.derived[d].haveResult;
            chains[c].derived[d].haveBase = chain.derived[d].haveBase;
            chains[c].derived[d].baseDirty = chain.derived[d].baseDirty;
        }
    }
    steps.resize(program.steps.size());
    for (size_t k = 0; k < program.steps.size(); ++k) {
        steps[k].diagnostics = program.steps[k].diagnostics;
        steps[k].counters = program.steps[k].counters;
        steps[k].bail = program.steps[k].bail;
    }
}

void
RigExecBakedRunShadow::Restore(RigExecBakedProgramImpl *program) const
{
    RigExecBakedProgramImpl &B = *program;
    B.avars = avars;
    B.posedM = posedM;
    B.finalMatrix = finalMatrix;
    B.baseMatrix = baseMatrix;
    B.base = base;
    B.fin = fin;
    B.aggregates = aggregates;
    B.constraintDeltas = constraintDeltas;
    B.avarsDisturbed = avarsDisturbed;
    // Run-local by construction: the prologue empties it, so a second run
    // over one frame has to start with it empty too or every record lands in
    // it twice.
    B.runSnapshots.Clear();

    for (size_t s = 0; s < B.solvers.size() && s < solvers.size(); ++s) {
        B.solvers[s].outFrames = solvers[s].outFrames;
        B.solvers[s].outPresent = solvers[s].outPresent;
        B.solvers[s].fallbackJoints = solvers[s].fallbackJoints;
    }
    for (size_t c = 0; c < B.commits.size() && c < commits.size(); ++c) {
        B.commits[c].present = commits[c].present;
        B.commits[c].deltaOk = commits[c].deltaOk;
        B.commits[c].frames = commits[c].frames;
        B.commits[c].staged = commits[c].staged;
        B.commits[c].deltas = commits[c].deltas;
        B.commits[c].outcome = commits[c].outcome;
        B.commits[c].sources = commits[c].sources;
        B.commits[c].abandoned = commits[c].abandoned;
    }
    for (size_t c = 0; c < B.chains.size() && c < chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        chain.lastBase = chains[c].lastBase;
        chain.result = chains[c].result;
        chain.spare = chains[c].spare;
        chain.haveResult = chains[c].haveResult;
        chain.haveBase = chains[c].haveBase;
        chain.baseDirty = chains[c].baseDirty;
        for (size_t r = 0;
             r < chain.revisions.size() && r < chains[c].revisions.size();
             ++r) {
            RestoreRevision(chains[c].revisions[r], &chain.revisions[r]);
        }
        for (size_t d = 0;
             d < chain.derived.size() && d < chains[c].derived.size(); ++d) {
            RestoreRevision(chains[c].derived[d].revision,
                            &chain.derived[d].revision);
            chain.derived[d].result = chains[c].derived[d].result;
            chain.derived[d].spare = chains[c].derived[d].spare;
            chain.derived[d].lastBase = chains[c].derived[d].lastBase;
            chain.derived[d].haveResult = chains[c].derived[d].haveResult;
            chain.derived[d].haveBase = chains[c].derived[d].haveBase;
            chain.derived[d].baseDirty = chains[c].derived[d].baseDirty;
        }
    }
    for (size_t k = 0; k < B.steps.size() && k < steps.size(); ++k) {
        B.steps[k].diagnostics = steps[k].diagnostics;
        B.steps[k].counters = steps[k].counters;
        B.steps[k].bail = steps[k].bail;
    }
}

size_t
RigExecBakedRunShadow::Compare(const RigExecBakedProgramImpl &program,
                               std::vector<std::string> *differences) const
{
    size_t count = 0;
    CompareVector(differences, &count, "posedM", posedM, program.posedM);
    CompareVector(differences, &count, "base", base, program.base);
    CompareVector(differences, &count, "fin", fin, program.fin);
    CompareVector(differences, &count, "finalMatrix", finalMatrix,
                  program.finalMatrix);
    CompareVector(differences, &count, "baseMatrix", baseMatrix,
                  program.baseMatrix);
    CompareVector(differences, &count, "aggregates", aggregates,
                  program.aggregates);
    for (size_t s = 0; s < program.solvers.size() && s < solvers.size(); ++s) {
        const std::string where = "solver " + program.solvers[s].path.GetString();
        CompareVector(differences, &count, where + " outFrames",
                      solvers[s].outFrames, program.solvers[s].outFrames);
        CompareVector(differences, &count, where + " outPresent",
                      solvers[s].outPresent, program.solvers[s].outPresent);
        CompareVector(differences, &count, where + " fallbackJoints",
                      solvers[s].fallbackJoints,
                      program.solvers[s].fallbackJoints);
    }
    for (size_t c = 0; c < program.commits.size() && c < commits.size(); ++c) {
        const std::string where = "commit " + std::to_string(c);
        CompareVector(differences, &count, where + " present",
                      commits[c].present, program.commits[c].present);
        CompareVector(differences, &count, where + " frames",
                      commits[c].frames, program.commits[c].frames);
        CompareVector(differences, &count, where + " staged",
                      commits[c].staged, program.commits[c].staged);
        CompareVector(differences, &count, where + " outcome",
                      commits[c].outcome, program.commits[c].outcome);
        CompareValue(differences, &count, where + " abandoned",
                     commits[c].abandoned, program.commits[c].abandoned);
    }
    for (size_t c = 0; c < program.chains.size() && c < chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = program.chains[c];
        const std::string where = "chain " + chain.target.GetString();
        if (!(chains[c].result == chain.result)) {
            Differ(differences, &count, where + " points");
        }
        CompareValue(differences, &count, where + " haveResult",
                     chains[c].haveResult, chain.haveResult);
        for (size_t r = 0;
             r < chain.revisions.size() && r < chains[c].revisions.size();
             ++r) {
            CompareRevision(differences, &count,
                            where + " revision " +
                                chain.revisions[r].moverPath.GetString(),
                            chains[c].revisions[r], chain.revisions[r]);
        }
        for (size_t d = 0;
             d < chain.derived.size() && d < chains[c].derived.size(); ++d) {
            if (!(chains[c].derived[d].result == chain.derived[d].result)) {
                Differ(differences, &count,
                       where + " derived " +
                           chain.derived[d].target.GetString() + " points");
            }
            CompareRevision(differences, &count,
                            where + " derived " +
                                chain.derived[d].target.GetString(),
                            chains[c].derived[d].revision,
                            chain.derived[d].revision);
        }
    }
    for (size_t k = 0; k < program.steps.size() && k < steps.size(); ++k) {
        const RigExecBakedStep &step = program.steps[k];
        const std::string where = "step " + step.label;
        CompareVector(differences, &count, where + " diagnostics",
                      steps[k].diagnostics, step.diagnostics);
        if (steps[k].counters.revisionsExecuted !=
                step.counters.revisionsExecuted ||
            steps[k].counters.revisionsCreated !=
                step.counters.revisionsCreated ||
            steps[k].counters.schedulesBuilt !=
                step.counters.schedulesBuilt ||
            steps[k].counters.chainsBuilt != step.counters.chainsBuilt ||
            steps[k].counters.revisionsBuilt != step.counters.revisionsBuilt) {
            Differ(differences, &count, where + " counters");
        }
        CompareValue(differences, &count, where + " bail", steps[k].bail,
                     step.bail);
    }
    return count;
}

}  // namespace rigExec

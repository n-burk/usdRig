// AtPrim transform reads held to the dynamic walk's phased-read store, for
// the tests. The baked program answers an AtPrim read phase on
// rigExec:transform (and on a matrix mover's reference influences) from a
// list of frame records bound at Build and keeps no store of its own. In a
// BakedWithParityCheck generation the dynamic walk runs after the program
// over the same evaluator, so the evaluator's store (the program's
// `chainSnapshots`) then holds the walk's records. It holds every pose
// record before any geometry reader runs, so each list's first valid record
// must equal the store's answer, in presence and bit for bit.
// Reports through a caller-owned failure counter, like rigExecPoseCompare.h.
#ifndef RIGEXEC_TESTS_FRAME_RECORD_CHECK_H
#define RIGEXEC_TESTS_FRAME_RECORD_CHECK_H

#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/usd/sdf/path.h"

#include <cstdio>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExecTest {

/// The first geometry revision of \p mover in any chain.
inline const rigExec::RigExecBakedProgramImpl::GeomRevision *
FindRevision(const rigExec::RigExecBakedProgramImpl &B, const SdfPath &mover)
{
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.moverPath == mover) {
                return &revision;
            }
        }
    }
    return nullptr;
}

/// The writers (constraints or solvers) of \p records, in list order.
inline std::vector<SdfPath>
RecordMovers(const rigExec::RigExecBakedProgramImpl &B,
             const std::vector<int> &records)
{
    std::vector<SdfPath> movers;
    for (const int record : records) {
        movers.push_back(size_t(record) < B.frameRecords.size()
                             ? B.frameRecords[size_t(record)].mover
                             : SdfPath());
    }
    return movers;
}

/// What the fold takes from \p records: the first valid one.
inline bool
FirstValidRecord(const rigExec::RigExecBakedProgramImpl &B,
                 const std::vector<int> &records, GfMatrix4d *matrix)
{
    for (const int record : records) {
        if (B.frameMatrixValid[size_t(record)]) {
            *matrix = B.frameMatrix[size_t(record)];
            return true;
        }
    }
    return false;
}

/// Holds every AtPrim transform reader of \p evaluator's program to the
/// dynamic walk's store after a BakedWithParityCheck generation, and every
/// other reader to an empty list. Returns the readers a record answered.
inline size_t
CheckFrameRecords(int *failures, const std::string &where,
                  const rigExec::RigExecRigEvaluator &evaluator)
{
    const auto expect = [failures](const std::string &at, bool ok,
                                   const char *what) {
        if (!ok) {
            ++*failures;
            std::printf("FAIL [%s]: %s\n", at.c_str(), what);
        }
    };
    size_t answered = 0;
    const rigExec::RigExecBakedProgram *program = evaluator.GetBakedProgram();
    expect(where, program != nullptr, "the rig runs a baked program");
    if (!program) {
        return answered;
    }
    const rigExec::RigExecBakedProgramImpl &B = program->GetStepGraph();
    expect(where,
           evaluator.GetEvaluationMode() ==
               rigExec::RigExecEvaluationMode::BakedWithParityCheck,
           "the walk ran beside the program");
    expect(where, B.chainSnapshots != nullptr, "the evaluator's store");
    if (!B.chainSnapshots) {
        return answered;
    }
    const rigExec::RigExecChainSnapshots &walk = *B.chainSnapshots;
    expect(where, B.frameMatrix.size() == B.frameRecords.size(),
           "one frameMatrix per record");
    expect(where, B.frameMatrixValid.size() == B.frameRecords.size(),
           "one frameMatrixValid per record");
    const auto check =
        [&](const std::string &at, int slot,
            const rigExec::RigExecBakedProgramImpl::GeomRevision &r,
            const std::vector<int> &records) {
        const VtValue *stored =
            slot >= 0 ? walk.Lookup(B.paths[size_t(slot)],
                                    r.binding.transformPhase, r.moverPath)
                      : nullptr;
        const bool storeAnswered = stored && stored->IsHolding<GfMatrix4d>();
        GfMatrix4d bound(1.0);
        const bool recordAnswered = FirstValidRecord(B, records, &bound);
        expect(at, storeAnswered == recordAnswered,
               "the store and the records answer alike");
        if (storeAnswered && recordAnswered) {
            expect(at, stored->UncheckedGet<GfMatrix4d>() == bound,
                   "the store's matrix is the record's");
        }
        return recordAnswered;
    };
    for (const auto &chain : B.chains) {
        for (const auto &revision : chain.revisions) {
            if (revision.binding.transformPhase.kind !=
                rigExec::RigExecReadPhaseKind::AtPrim) {
                expect(where + " " + revision.moverPath.GetString(),
                       revision.transformRecords.empty(),
                       "a reader at no AtPrim phase lists no record");
                continue;
            }
            const std::string at =
                where + " " + revision.moverPath.GetString();
            if (check(at, revision.transformSlot, revision,
                      revision.transformRecords)) {
                ++answered;
            }
            for (size_t k = 0; k < revision.influenceRecords.size(); ++k) {
                check(at + " influence " + std::to_string(k),
                      revision.influenceSlots[k], revision,
                      revision.influenceRecords[k]);
            }
        }
    }
    return answered;
}

}  // namespace rigExecTest

#endif

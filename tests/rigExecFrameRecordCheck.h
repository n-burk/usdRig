// AtPrim typed producer provenance and optional literal checkpoint facts.
// This helper checks binding/storage conformance, not an independent math oracle.
#ifndef RIGEXEC_TESTS_FRAME_RECORD_CHECK_H
#define RIGEXEC_TESTS_FRAME_RECORD_CHECK_H

#include "rigExecPhaseQueryObserver.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/usd/sdf/path.h"

#include <cstdio>
#include <map>
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
        movers.push_back(record >= 0 && size_t(record) < B.frameRecords.size()
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
        if (record >= 0 && size_t(record) < B.frameMatrixValid.size() &&
            size_t(record) < B.frameMatrix.size() && B.frameMatrixValid[size_t(record)]) {
            *matrix = B.frameMatrix[size_t(record)];
            return true;
        }
    }
    return false;
}

struct LiteralFrameFact {
    rigExec::RigExecPointFrame frame;
    GfMatrix4d matrix{1.0};
    bool hasValue = false;
};

/// Checks exact provider/version provenance and every caller-supplied literal
/// fact. Literal expectations must come from fixture arithmetic or independent
/// reference capture, never this program's output tables.
inline size_t
CheckFrameRecords(int *failures, const std::string &where,
                  const rigExec::RigExecRigEvaluator &evaluator,
                  const std::map<int,LiteralFrameFact> *literalFacts = nullptr)
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
    expect(where, B.frameMatrix.size() == B.frameRecords.size(),
           "one frameMatrix per record");
    expect(where, B.frameMatrixValid.size() == B.frameRecords.size(),
           "one frameMatrixValid per record");
    const auto check =
        [&](const std::string &at, int slot,
            const rigExec::RigExecBakedProgramImpl::GeomRevision &r,
            const std::vector<int> &records) {
        expect(at, slot >= 0 && size_t(slot) < B.paths.size(), "valid provider slot");
        for (const int id : records) {
            const bool validId = id >= 0 && size_t(id) < B.frameRecords.size();
            expect(at,validId,"record index is in range");
            if (!validId) continue;
            if (size_t(id) >= B.frameMatrixValid.size() || size_t(id) >= B.frameMatrix.size()) continue;
            const auto &record = B.frameRecords[size_t(id)];
            expect(at,record.slot == slot,"record belongs to the requested provider");
            expect(at,record.version < B.fin.size(),"record names a typed final-frame version");
            size_t producers = 0;
            for (const auto &step : B.steps) {
                if (step.kind != rigExec::RigExecBakedStepKind::FrameMatrix || step.object != id) continue;
                ++producers;
                bool readsVersion = false;
                for (const auto &range : step.reads)
                    readsVersion = readsVersion ||
                        (range.domain == rigExec::RigExecBakedSlotDomain::PoseFin &&
                         record.version >= range.begin && record.version < range.end);
                expect(at,readsVersion,"checkpoint producer reads the exact frame version");
            }
            expect(at,producers == 1,"checkpoint has exactly one declared producer");
            if (literalFacts) {
                const auto fact = literalFacts->find(id);
                if (fact != literalFacts->end() && record.version < B.fin.size()) {
                    expect(at,B.fin[record.version] == fact->second.frame,"literal checkpoint frame");
                    expect(at,bool(B.frameMatrixValid[size_t(id)]) == fact->second.hasValue,
                           "literal checkpoint presence");
                    if (fact->second.hasValue)
                        expect(at,B.frameMatrix[size_t(id)] == fact->second.matrix,"literal checkpoint matrix");
                }
            }
        }
        GfMatrix4d bound(1.0);
        const bool recordAnswered = FirstValidRecord(B,records,&bound);
        if (rigExecOriginalQuery::configuration.check && !literalFacts) {
            const VtValue value = recordAnswered ? VtValue(bound) : VtValue();
            rigExecOriginalQuery::Emit(at,r.moverPath.GetString(),
                slot >= 0 && size_t(slot) < B.paths.size() ? B.paths[size_t(slot)].GetString() : std::string(),
                r.binding.transformPhase.GetAsString(),B.lastTime.IsDefault(),
                B.lastTime.GetValue(),recordAnswered ? &value : nullptr);
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

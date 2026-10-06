// Bake side of the .rigexec PropertyChains section (see
// rigExecBinary/propertyChains.h).
#ifndef RIGEXEC_BAKE_PROPERTY_CHAINS_BAKE_H
#define RIGEXEC_BAKE_PROPERTY_CHAINS_BAKE_H

#include "rigExecBinary/container.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/propertyChains.h"

#include <string>
#include <vector>

namespace rigExec {

class RigExecRigEvaluator;

/// Builds the property chains of \p evaluator's compiled epoch as programs,
/// with each walk's fallback and each target's base sampled at \p frames,
/// and the consumers matched against \p table's directory heads. Paths are
/// interned into \p writer.
///
/// A chain the runtime cannot reproduce exactly -- a weight object, a
/// value type other than float or double, an operation or curve that is not
/// constant -- is left out and named in \p skipped; the runtime keeps
/// replaying its recorded value. False only when the chains cannot be
/// named at all (see RigExecBindChainSampleInputs).
bool RigExecBakePropertyChains(const RigExecRigEvaluator &evaluator,
                               const std::vector<double> &frames,
                               const RigExecWireInputTable &table,
                               RigExecBinaryWriter *writer,
                               RigExecWirePropertyChains *out,
                               std::vector<std::string> *skipped,
                               std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_PROPERTY_CHAINS_BAKE_H

// Compiled into each consumer of librigExecOracle. The call keeps the
// shared library on the link line when the consumer only reaches the
// hooks through librigExec, so the load-time registration runs.
namespace rigExec {
void RigExecOracleLinkAnchor();
namespace {
struct Anchor {
    Anchor() { RigExecOracleLinkAnchor(); }
};
const Anchor anchor;
}
}

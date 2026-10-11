// Compiled into each consumer of librigExecScene. The call keeps the
// shared library on the link line when the consumer only reaches the
// lowering hooks through librigExec, so the load-time registration runs.
namespace rigExec {
void RigExecSceneLinkAnchor();
namespace {
struct Anchor {
    Anchor() { RigExecSceneLinkAnchor(); }
};
const Anchor anchor;
}
}

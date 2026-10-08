#ifndef RIGEXEC_GRAPH_PROVIDER_RECORDS_H
#define RIGEXEC_GRAPH_PROVIDER_RECORDS_H
#include <array>
#include <cstdint>
#include <cstddef>
#include <string>
#include <variant>
#include <vector>
namespace rigExec {
enum class RigExecProviderOpKind {
    Attribute, SpaceExpression, RestFrame, DefaultSpace, FrameToSpace,
    MatrixToFrame, PosedFrame, JointMatrix, LocalXform, InterveningXform, AvarMatrix, RelativeXform
};
struct RigExecProviderPlainOp {
    RigExecProviderOpKind kind;
    std::string owner;
    uint64_t output=UINT64_MAX;
    std::vector<uint64_t> inputs;
    bool scaleAvars=true;
};
struct RigExecProviderPlainFrame {
    std::array<std::array<double,3>,4> points;
    uint32_t flags=0;
};
using RigExecProviderPlainValue=std::variant<std::monostate,double,float,
    std::array<double,3>,std::array<double,16>,std::string,RigExecProviderPlainFrame,
    std::array<float,3>,bool,int32_t,std::vector<float>,std::vector<double>,
    std::vector<std::array<float,3>>,std::vector<std::array<double,3>>,
    std::vector<int32_t>,std::vector<std::array<double,16>>,std::vector<std::string>,
    std::vector<bool>,std::array<float,2>,std::vector<std::array<float,2>>,std::array<int32_t,3>>;
struct RigExecProviderPlainState {
    RigExecProviderPlainValue value;
    bool initialized=false,blocked=false,authoritative=false;
    uint64_t revision=0;
    size_t count=0;
    std::string error;
};
struct RigExecProviderPlainLeaf {
    uint64_t value;
    std::string path,computation;
};
struct RigExecProviderPlainRoutedInput {
    uint64_t value;
    std::string consumer,source,readPhase;
};
/// Wire-neutral provider program. Numeric precision and exact empty/block/
/// identity-authoritative states survive translation without USD objects.
struct RigExecProviderPlainProgram {
    std::vector<std::string> valueKeys;
    std::vector<RigExecProviderPlainState> defaults;
    std::vector<RigExecProviderPlainOp> ops;
    std::vector<RigExecProviderPlainLeaf> sampled,externalInputs;
    std::vector<RigExecProviderPlainRoutedInput> routedInputs;
    std::vector<uint64_t> leaves;
};
}
#endif

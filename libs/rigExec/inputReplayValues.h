#ifndef RIGEXEC_INPUT_REPLAY_VALUES_H
#define RIGEXEC_INPUT_REPLAY_VALUES_H
#include "pxr/base/vt/value.h"
#include <string>
#if defined(_WIN32) && defined(RIGEXEC_INPUT_VALUES_EXPORTS)
# define RIGEXEC_INPUT_VALUES_API __declspec(dllexport)
#else
# define RIGEXEC_INPUT_VALUES_API
#endif
namespace rigExec {
// Input-only binary values; no source queries or produced-value substitution.
// Failure leaves the caller's output unchanged. Decode rejects trailing bytes.
RIGEXEC_INPUT_VALUES_API bool RigExecEncodeInputValue(const pxr::VtValue&, std::string*, std::string*);
RIGEXEC_INPUT_VALUES_API bool RigExecDecodeInputValue(const std::string&, pxr::VtValue*, std::string*);
}
#undef RIGEXEC_INPUT_VALUES_API
#endif

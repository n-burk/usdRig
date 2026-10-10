#ifndef RIGEXEC_BINARY_TRANSPORT_H
#define RIGEXEC_BINARY_TRANSPORT_H
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
namespace rigExec { namespace transport {
constexpr size_t HeaderSize = 48;
struct FreeBuffer { void operator()(uint8_t *p) const { std::free(p); } };
struct Buffer { std::unique_ptr<uint8_t, FreeBuffer> data; size_t size = 0; };
// Private fault hooks: per-call, no global allocator or shared instrumentation.
struct Faults { size_t failSDKCall = 0; bool failOutput = false; size_t failOutputCall = 0; };
struct Stats { size_t sdkCalls = 0, sdkActive = 0, sdkPeak = 0, outputAttempts = 0; };
bool IsEnvelope(const uint8_t *, size_t);
bool Decode(const uint8_t *, size_t, Buffer *, std::string *, Stats * = nullptr, Faults = {});
// Empty successful result means raw fallback; never encode an equal/larger container.
bool Encode(const uint8_t *, size_t, std::vector<uint8_t> *, std::string *, Stats * = nullptr, Faults = {});
} }
#endif

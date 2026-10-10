// RigExec pulled upstream inputs. See upstreamTable.h for the contract.
#include "rigExecImaging/upstreamTable.h"

#include "rigExec/frameCache.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/types.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

namespace {

// USD's linear step (usd/interpolators.cpp, Usd_Interpolate): alpha from the
// bracketing times, then GfLerp per value or per array element, so a source
// sampled where a stage authors the same samples reconstructs bit for bit.
template <class T>
bool
_LerpAs(const VtValue &lower, const VtValue &upper, double alpha,
        VtValue *out)
{
    if (!lower.IsHolding<T>() || !upper.IsHolding<T>()) {
        return false;
    }
    *out = VtValue(GfLerp(alpha, lower.UncheckedGet<T>(),
                          upper.UncheckedGet<T>()));
    return true;
}

template <class T>
bool
_LerpArrayAs(const VtValue &lower, const VtValue &upper, double alpha,
             VtValue *out)
{
    if (!lower.IsHolding<VtArray<T>>() || !upper.IsHolding<VtArray<T>>()) {
        return false;
    }
    const VtArray<T> &a = lower.UncheckedGet<VtArray<T>>();
    const VtArray<T> &b = upper.UncheckedGet<VtArray<T>>();
    // Arrays of different lengths hold, as USD's do.
    if (a.size() != b.size()) {
        *out = lower;
        return true;
    }
    if (alpha == 1.0) {
        *out = upper;
        return true;
    }
    VtArray<T> result(a.size());
    T *data = result.data();
    for (size_t i = 0; i < a.size(); ++i) {
        data[i] = GfLerp(alpha, a[i], b[i]);
    }
    *out = VtValue(std::move(result));
    return true;
}

// Linear between \p lower and \p upper for the interpolating types; false
// for every other type, which holds.
bool
_Lerp(const VtValue &lower, const VtValue &upper, double alpha, VtValue *out)
{
    if (lower.GetTypeid() != upper.GetTypeid()) {
        return false;
    }
    return _LerpAs<double>(lower, upper, alpha, out) ||
           _LerpAs<float>(lower, upper, alpha, out) ||
           _LerpAs<GfVec3d>(lower, upper, alpha, out) ||
           _LerpAs<GfVec3f>(lower, upper, alpha, out) ||
           _LerpAs<GfMatrix4d>(lower, upper, alpha, out) ||
           _LerpArrayAs<float>(lower, upper, alpha, out) ||
           _LerpArrayAs<double>(lower, upper, alpha, out) ||
           _LerpArrayAs<GfVec2f>(lower, upper, alpha, out) ||
           _LerpArrayAs<GfVec3f>(lower, upper, alpha, out);
}

// The first element's address of an array value, null for a scalar.
const void *
_ArrayData(const VtValue &value)
{
    if (value.IsHolding<VtIntArray>()) {
        return value.UncheckedGet<VtIntArray>().cdata();
    }
    if (value.IsHolding<VtFloatArray>()) {
        return value.UncheckedGet<VtFloatArray>().cdata();
    }
    if (value.IsHolding<VtDoubleArray>()) {
        return value.UncheckedGet<VtDoubleArray>().cdata();
    }
    if (value.IsHolding<VtVec2fArray>()) {
        return value.UncheckedGet<VtVec2fArray>().cdata();
    }
    if (value.IsHolding<VtVec3fArray>()) {
        return value.UncheckedGet<VtVec3fArray>().cdata();
    }
    return nullptr;
}

}  // namespace

VtValue
RigExecReconstructUpstream(
    const std::vector<RigExecUpstreamTableSample> &samples, double t,
    bool held)
{
    if (samples.empty()) {
        return VtValue();
    }
    if (t <= samples.front().time) {
        return samples.front().value;
    }
    if (t >= samples.back().time) {
        return samples.back().value;
    }
    // The first sample after t; the one before it brackets from below.
    const auto upper = std::upper_bound(
        samples.begin(), samples.end(), t,
        [](double time, const RigExecUpstreamTableSample &sample) {
            return time < sample.time;
        });
    const auto lower = upper - 1;
    if (lower->time == t || held) {
        return lower->value;
    }
    const double alpha = (t - lower->time) / (upper->time - lower->time);
    if (alpha == 0.0) {
        return lower->value;
    }
    VtValue lerped;
    if (_Lerp(lower->value, upper->value, alpha, &lerped)) {
        return lerped;
    }
    return lower->value;
}

bool
RigExecUpstreamTable::AnyVaries() const
{
    for (const auto &[path, entry] : entries) {
        if (entry.varies) {
            return true;
        }
    }
    return false;
}

bool
RigExecUpstreamTable::ValueAt(const SdfPath &path,
                              const RigExecUpstreamTableEntry &entry,
                              double t, RigExecUpstreamValue *out) const
{
    out->path = path;
    if (!entry.varies || t == t0) {
        out->value = entry.atT0;
        out->foldHash = entry.hashAtT0;
        return !out->value.IsEmpty();
    }
    if (t < windowLo || t > windowHi) {
        return false;
    }
    const double frame = std::floor(t);
    if (frame == t && frame >= double(firstFrame) &&
        frame < double(firstFrame) + double(frameCount)) {
        const size_t index = size_t(int64_t(frame) - firstFrame);
        if (entry.isArray) {
            out->value = RigExecReconstructUpstream(entry.samples, t, held);
            out->foldHash = index < entry.frameHashes.size()
                                ? entry.frameHashes[index]
                                : RigExecUpstreamFoldHash(out->value);
        } else {
            out->value = index < entry.frameValues.size()
                             ? entry.frameValues[index]
                             : RigExecReconstructUpstream(entry.samples, t,
                                                          held);
            out->foldHash = 0;
        }
        return !out->value.IsEmpty();
    }
    // Between window frames: reconstructed, and an array's hash is left for
    // the sampler to fold (RigExecUpstreamValue::foldHash 0).
    out->value = RigExecReconstructUpstream(entry.samples, t, held);
    out->foldHash = 0;
    return !out->value.IsEmpty();
}

bool
RigExecUpstreamTable::ValuesAt(double t,
                               std::vector<RigExecUpstreamValue> *out) const
{
    out->clear();
    out->reserve(entries.size());
    bool complete = true;
    for (const auto &[path, entry] : entries) {
        RigExecUpstreamValue value;
        if (ValueAt(path, entry, t, &value)) {
            out->push_back(std::move(value));
        } else {
            complete = false;
        }
    }
    return complete;
}

bool
RigExecFillUniformUpstreamEntry(const VtValue &value,
                                RigExecUpstreamTableEntry *entry)
{
    if (value.IsEmpty()) {
        return false;
    }
    *entry = RigExecUpstreamTableEntry();
    entry->isArray = value.IsArrayValued();
    entry->atT0 = value;
    entry->hashAtT0 = RigExecUpstreamFoldHash(value);
    return true;
}

bool
RigExecFillVaryingUpstreamEntry(
    const RigExecUpstreamTable &table,
    std::vector<RigExecUpstreamTableSample> samples,
    RigExecUpstreamTableEntry *entry)
{
    samples.erase(std::remove_if(samples.begin(), samples.end(),
                                 [](const RigExecUpstreamTableSample &s) {
                                     return s.value.IsEmpty();
                                 }),
                  samples.end());
    if (samples.empty()) {
        return false;
    }
    std::stable_sort(samples.begin(), samples.end(),
                     [](const RigExecUpstreamTableSample &a,
                        const RigExecUpstreamTableSample &b) {
                         return a.time < b.time;
                     });
    // One value per time: the first returned wins.
    samples.erase(std::unique(samples.begin(), samples.end(),
                              [](const RigExecUpstreamTableSample &a,
                                 const RigExecUpstreamTableSample &b) {
                                  return a.time == b.time;
                              }),
                  samples.end());
    *entry = RigExecUpstreamTableEntry();
    entry->varies = true;
    entry->isArray = samples.front().value.IsArrayValued();
    entry->samples = std::move(samples);
    std::vector<uint64_t> sampleHashes;
    if (entry->isArray) {
        sampleHashes.reserve(entry->samples.size());
        for (const RigExecUpstreamTableSample &sample : entry->samples) {
            sampleHashes.push_back(RigExecUpstreamFoldHash(sample.value));
        }
    }
    // The fold hash of a reconstructed array: a sample's own when the
    // reconstruction is that sample's buffer (held, boundary or exact).
    const auto hashOf = [&](const VtValue &value) -> uint64_t {
        const void *data = _ArrayData(value);
        for (size_t i = 0; data && i < entry->samples.size(); ++i) {
            if (_ArrayData(entry->samples[i].value) == data &&
                entry->samples[i].value.GetTypeid() == value.GetTypeid()) {
                return sampleHashes[i];
            }
        }
        return RigExecUpstreamFoldHash(value);
    };
    entry->atT0 =
        RigExecReconstructUpstream(entry->samples, table.t0, table.held);
    entry->hashAtT0 = entry->isArray ? hashOf(entry->atT0) : 0;
    if (entry->isArray) {
        entry->frameHashes.reserve(table.frameCount);
    } else {
        entry->frameValues.reserve(table.frameCount);
    }
    for (size_t i = 0; i < table.frameCount; ++i) {
        const double frame = double(table.firstFrame + int64_t(i));
        VtValue value =
            RigExecReconstructUpstream(entry->samples, frame, table.held);
        if (entry->isArray) {
            // The array is dropped once its hash is kept.
            entry->frameHashes.push_back(hashOf(value));
        } else {
            entry->frameValues.push_back(std::move(value));
        }
    }
    return true;
}

void
RigExecReanchorUpstreamTable(RigExecUpstreamTable *table, double t0)
{
    for (auto &[path, entry] : table->entries) {
        if (!entry.varies) {
            continue;
        }
        // At the old T0 this is the old value; elsewhere the frame tables
        // or the samples answer.
        RigExecUpstreamValue value;
        if (table->ValueAt(path, entry, t0, &value)) {
            entry.atT0 = value.value;
            entry.hashAtT0 = entry.isArray
                                 ? (value.foldHash != 0
                                        ? value.foldHash
                                        : RigExecUpstreamFoldHash(value.value))
                                 : 0;
        }
    }
    table->t0 = t0;
}

bool
RigExecSameUpstreamSources(const RigExecUpstreamTable &a,
                           const RigExecUpstreamTable &b)
{
    if (a.entries.size() != b.entries.size() || a.held != b.held) {
        return false;
    }
    const bool varies = a.AnyVaries();
    if (varies != b.AnyVaries()) {
        return false;
    }
    // A window bounds only the per-frame tables of varying sources.
    if (varies &&
        (a.windowLo != b.windowLo || a.windowHi != b.windowHi ||
         a.firstFrame != b.firstFrame || a.frameCount != b.frameCount)) {
        return false;
    }
    auto ia = a.entries.begin();
    auto ib = b.entries.begin();
    for (; ia != a.entries.end(); ++ia, ++ib) {
        const RigExecUpstreamTableEntry &x = ia->second;
        const RigExecUpstreamTableEntry &y = ib->second;
        if (ia->first != ib->first || x.varies != y.varies ||
            x.isArray != y.isArray) {
            return false;
        }
        if (!x.varies) {
            if (x.hashAtT0 != y.hashAtT0 || x.atT0 != y.atT0) {
                return false;
            }
            continue;
        }
        if (x.samples.size() != y.samples.size() ||
            x.frameValues != y.frameValues ||
            x.frameHashes != y.frameHashes) {
            return false;
        }
        for (size_t i = 0; i < x.samples.size(); ++i) {
            if (x.samples[i].time != y.samples[i].time ||
                x.samples[i].value != y.samples[i].value) {
                return false;
            }
        }
    }
    return true;
}

size_t
RigExecUpstreamTableBufferCount(const RigExecUpstreamTable &table)
{
    std::set<const void *> buffers;
    const auto note = [&](const VtValue &value) {
        if (const void *data = _ArrayData(value)) {
            buffers.insert(data);
        }
    };
    for (const auto &[path, entry] : table.entries) {
        note(entry.atT0);
        for (const RigExecUpstreamTableSample &sample : entry.samples) {
            note(sample.value);
        }
        for (const VtValue &value : entry.frameValues) {
            note(value);
        }
    }
    return buffers.size();
}

}  // namespace rigExec

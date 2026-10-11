// USD-free. Execute shares a retained point or weight buffer instead of
// copying it. A later Write or swap leaves that share's bytes alone,
// including a NaN payload. A unique buffer still recycles on swap.
#include "rigExecRuntime/runtime.h"
#include "rigExecRuntime/store.h"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

namespace {

int Fail(const char *message)
{
    std::cerr << message << "\n";
    return 1;
}

rigExec::RrVec3f Point(float x, float y, float z)
{
    return rigExec::RrVec3f(x, y, z);
}

}  // namespace

int main()
{
    using namespace rigExec;

    uint32_t payload = 0x7fc01234u;
    RrVec3f nanPoint = Point(1.f, 0.f, 3.f);
    std::memcpy(&nanPoint[1], &payload, sizeof(payload));

    RrRetainedArray<RrVec3f> held;
    held.Write().push_back(nanPoint);
    held.Write().push_back(Point(4.f, 5.f, 6.f));
    const std::shared_ptr<const std::vector<RrVec3f>> share = held.Share();
    if (!share || share->data() != held.Read().data() || share.use_count() < 2) {
        return Fail("point share copied the buffer");
    }
    if (std::memcmp(share->data(), held.Read().data(),
                    share->size() * sizeof(RrVec3f)) != 0) {
        return Fail("point share bytes differ");
    }

    const RrVec3f *snapshot = share->data();
    held.Write()[1] = Point(7.f, 8.f, 9.f);
    if (share->data() != snapshot || (*share)[1][0] != 4.f ||
        std::memcmp(&(*share)[0], &nanPoint, sizeof(RrVec3f)) != 0) {
        return Fail("write mutated a shared point snapshot");
    }
    if (held.Read().data() == snapshot || held.Read()[1][0] != 7.f) {
        return Fail("write did not copy on write");
    }

    RrRetainedArray<RrVec3f> chain;
    chain.Write().push_back(Point(1.f, 0.f, 0.f));
    chain.Write().push_back(Point(0.f, 1.f, 0.f));
    RigExecSharedArray<RrVec3f> published(chain.Share());
    const RrVec3f *publishedData = published.data();
    std::vector<RrVec3f> spare;
    spare.push_back(Point(9.f, 9.f, 9.f));
    spare.push_back(Point(8.f, 8.f, 8.f));
    chain.swap(spare);
    if (published.data() != publishedData || published[0][0] != 1.f ||
        std::memcmp(published.data(), publishedData, 2 * sizeof(RrVec3f)) != 0) {
        return Fail("swap moved a shared point buffer");
    }
    if (chain.Read().size() != 2 || chain.Read()[0][0] != 9.f || !spare.empty()) {
        return Fail("shared swap did not install the spare");
    }
    RigExecSharedArray<RrVec3f> again(published.Share());
    if (published != again || published.Vector()[0][0] != 1.f) {
        return Fail("published view lost its points");
    }
    RigExecSharedArray<RrVec3f> nanView(share);
    if (nanView == nanView ||
        std::memcmp(nanView.data(), &nanPoint, sizeof(nanPoint)) != 0) {
        return Fail("NaN payload did not survive the share");
    }

    RrRetainedArray<RrVec3f> unique;
    unique.Write().push_back(Point(1.f, 2.f, 3.f));
    const RrVec3f *uniqueData = unique.Read().data();
    std::vector<RrVec3f> recycled;
    recycled.push_back(Point(4.f, 5.f, 6.f));
    unique.swap(recycled);
    if (recycled.data() != uniqueData || unique.Read()[0][0] != 4.f) {
        return Fail("unique swap did not recycle the buffer");
    }

    RrRetainedArray<float> weights;
    weights.Write().push_back(0.25f);
    weights.Write().push_back(0.5f);
    RigExecSharedArray<float> weightView(weights.Share());
    if (weightView.data() != weights.data() || weightView.size() != 2) {
        return Fail("weight share copied the buffer");
    }
    std::vector<float> weightSpare(1, 1.f);
    weights.swap(weightSpare);
    if (weightView[0] != 0.25f || weights.Read().size() != 1 ||
        weights.Read()[0] != 1.f || !weightSpare.empty()) {
        return Fail("shared weight swap changed the snapshot");
    }

    RrRetainedArray<RrVec3f> emptyPoints;
    RrRetainedArray<float> emptyWeights;
    if (emptyPoints.Share().get() != RrEmptySharedPoints.get() ||
        !emptyPoints.Share()->empty() ||
        emptyWeights.Share().get() != RrEmptySharedWeights.get()) {
        return Fail("empty share is not the stable empty buffer");
    }
    RigExecSharedArray<float> noWeights;
    if (!noWeights.empty() || !(noWeights == RigExecSharedArray<float>())) {
        return Fail("default weight view is not empty");
    }

    // A reader that does not keep the frame: the next spare reuses it.
    RrRetainedArray<RrVec3f> replay;
    replay.Write().push_back(Point(1.f, 2.f, 3.f));
    replay.Write().push_back(Point(4.f, 5.f, 6.f));
    RigExecSharedArray<RrVec3f> frame(replay.Share());
    const RrVec3f *firstData = frame.data();
    std::vector<RrVec3f> replaySpare;
    RrRetainedPrepare(&replaySpare, 2);
    replaySpare[0] = Point(7.f, 7.f, 7.f);
    replaySpare[1] = Point(8.f, 8.f, 8.f);
    replay.swap(replaySpare);
    if (frame.data() != firstData || frame[0][0] != 1.f) {
        return Fail("replay swap changed the held frame");
    }
    RrRetainedRecycle(frame.Release());
    frame = RigExecSharedArray<RrVec3f>(replay.Share());
    std::vector<RrVec3f> reused;
    RrRetainedPrepare(&reused, 2);
    if (reused.data() != firstData) {
        return Fail("spare did not reuse the released frame");
    }
    reused[0] = Point(0.f, 0.f, 0.f);
    if (frame.data() == reused.data() || frame[0][0] != 7.f) {
        return Fail("reused spare aliased the new frame");
    }

    // A caller-held share is not recycled, and its bytes stay put.
    RigExecSharedArray<RrVec3f> kept(frame.Share());
    const RrVec3f *keptData = kept.data();
    const float keptX = kept[0][0];
    RrRetainedRecycle(frame.Release());
    if (kept.data() != keptData || kept.size() != 2 || kept[0][0] != keptX) {
        return Fail("recycle stole a caller-held share");
    }
    std::vector<RrVec3f> fresh;
    RrRetainedPrepare(&fresh, 2);
    if (fresh.data() == keptData) {
        return Fail("prepare reused a caller-held share");
    }

    std::cout << "shared buffers ok\n";
    return 0;
}

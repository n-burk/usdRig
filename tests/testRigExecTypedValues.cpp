#include "rigExecGraph/typedValues.h"
#include "rigExec/weightPackets.h"
#include <cstdint>
#include <cstring>
#include <iostream>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(condition) do { if(!(condition)) { std::cerr<<"failed line "<<__LINE__<<": "<<#condition<<'\n';return 1; } } while(false)
namespace {
float FloatBits(uint32_t bits) { float value;std::memcpy(&value,&bits,sizeof(value));return value; }
}
int main() {
    RigExecTypedValueStore store(4);
    const float nan=FloatBits(0x7fc00001u);
    VtFloatArray values{nan,-0.0f};
    CHECK(store.PublishSource(0,VtValue(values)));
    CHECK(!store.PublishSource(0,VtValue(values)));
    CHECK(store.values[0].count==2);
    values[1]=0.0f;
    CHECK(store.PublishSource(0,VtValue(values)));
    values[0]=FloatBits(0x7fc00002u);
    CHECK(store.PublishSource(0,VtValue(values)));
    CHECK(store.PublishSource(0,VtValue(VtFloatArray{})));
    CHECK(store.values[0].count==0 && store.Read<VtValue>(0));
    CHECK(store.PublishSource(0,VtValue(VtFloatArray{}),true));
    CHECK(!store.Read<VtValue>(0));
    CHECK(store.PublishSource(0,VtValue(VtFloatArray{}),false));
    CHECK(!store.PublishSource(0,VtValue(VtFloatArray{}),false));
    CHECK(store.PublishSource(0,VtValue(VtDoubleArray{}),false));

    RigExecWeightPacket packet;
    packet.valid=true;packet.representation=TfToken("dense");packet.rangePolicy=TfToken("strict");
    packet.defaultWeight=-0.0f;packet.values={nan};
    CHECK(store.Publish(1,VtValue(packet),false,true,1));
    CHECK(!store.Publish(1,VtValue(packet),false,true,1));
    packet.defaultWeight=0.0f;
    CHECK(store.Publish(1,VtValue(packet),false,true,1));
    packet.valid=false;
    CHECK(store.Publish(1,VtValue(packet),false,true,1,"weight unavailable"));
    CHECK(store.values[1].error=="weight unavailable");
    CHECK(!store.Publish(1,VtValue(packet),false,true,1,"weight unavailable"));

    RigExecPointFrameArray frames;frames.frames.resize(1);frames.rests={std::array<GfVec3d,4>{GfVec3d(0),GfVec3d(1,0,0),GfVec3d(0,1,0),GfVec3d(0,0,1)}};
    frames.frames[0].flags=RigExecPointFrameValid;
    CHECK(store.Publish(2,VtValue(frames),false,false,1));
    CHECK(!store.Publish(2,VtValue(frames),false,false,1));
    frames.frames[0].flags=RigExecPointFrameDegenerate;
    CHECK(store.Publish(2,VtValue(frames),false,false,1));
    frames.rests[0][0][0]=-0.0;
    CHECK(store.Publish(2,VtValue(frames),false,false,1));
    CHECK(store.Copy(3,2));
    CHECK(!store.Copy(3,2));
    CHECK(store.values[3].count==1);
    RigExecPointFrame frame=frames.frames[0];
    CHECK(store.Publish(3,VtValue(frame),false,false,1));
    CHECK(!store.Publish(3,VtValue(frame),false,false,1));
    frame.points[0][0]=-0.0;
    CHECK(store.Publish(3,VtValue(frame),false,false,1));
    CHECK(!store.Publish(3,VtValue(frame),false,false,1));
    uint64_t doubleBits=0x7ff8000000001234ULL;
    std::memcpy(&frame.points[1][0],&doubleBits,sizeof(doubleBits));
    CHECK(store.Publish(3,VtValue(frame),false,false,1));
    CHECK(!store.Publish(3,VtValue(frame),false,false,1));
    ++doubleBits;
    std::memcpy(&frame.points[1][0],&doubleBits,sizeof(doubleBits));
    CHECK(store.Publish(3,VtValue(frame),false,false,1));
    frame.flags=RigExecPointFrameValid;
    CHECK(store.Publish(3,VtValue(frame),false,false,1));
    return 0;
}

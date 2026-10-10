// USD-free playback of the checked-in minimal program.
// The bytes are the smallest file the format validator accepts: empty
// tables, rig path /Rig, no steps. This test rebuilds that file and
// compares it to the fixture, then opens and executes the fixture.
#include "rigExecBinary/format.h"
#include "rigExecRuntime/runtime.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace {

rigExec::fb::RigExecWireFile MinimalFile()
{
    using namespace rigExec;
    using namespace rigExec::fb;
    RigExecWireFile file;
    file.formatVersion = RigExecFormatVersion;
    file.names = {"", "Rig"};
    file.paths = {PathNode(0, 0, PathKind::None),
                  PathNode(0, 1, PathKind::Prim)};
    file.rig = 1;
    file.values.resize(1);
    file.intArrays.resize(1);
    file.floatArrays.resize(1);
    file.doubleArrays.resize(1);
    file.vec2fArrays.resize(1);
    file.vec3fArrays.resize(1);
    file.vec3dArrays.resize(1);
    file.matrix4dArrays.resize(1);
    file.tokenArrays.resize(1);
    file.boolArrays.resize(1);
    file.slotMeta = std::make_unique<RigExecWireSlotMeta>();
    file.constants = std::make_unique<RigExecWireConstants>();
    file.clustering = std::make_unique<RigExecWireClustering>();
    file.cones = std::make_unique<RigExecWireCones>();
    file.cones->always = std::make_unique<RigExecWireClusterSet>();
    file.cones->poseClusters = std::make_unique<RigExecWireClusterSet>();
    file.pose = std::make_unique<RigExecWireDomainPose>();
    file.pose->requiredStageFramesAdmission =
        std::make_unique<RigExecWireRequiredStageFramesAdmission>();
    file.geometry = std::make_unique<RigExecWireDomainGeometry>();
    file.commonGraph = std::make_unique<RigExecWireCommonGraph>();
    return file;
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "expected the minimal.rigexec fixture\n";
        return 2;
    }
    std::ifstream file(argv[1], std::ios::binary);
    if (!file) {
        std::cerr << "cannot open " << argv[1] << "\n";
        return 1;
    }
    const std::vector<uint8_t> fixture{
        std::istreambuf_iterator<char>(file), {}};
    std::vector<uint8_t> rebuilt;
    std::string error;
    if (!rigExec::RigExecFormatWrite(MinimalFile(), &rebuilt, &error) ||
        rebuilt != fixture) {
        std::cerr << "fixture is not the minimal program: " << error << "\n";
        return 1;
    }
    auto reader = rigExec::RigExecRuntimeReader::Open(
        fixture.data(), fixture.size(), &error);
    if (!reader || !reader->Execute(&error)) {
        std::cerr << error << "\n";
        return 1;
    }
    if (!reader->GetPoints().empty() || !reader->GetJointMatrices().empty()) {
        std::cerr << "minimal program published outputs\n";
        return 1;
    }
    return 0;
}

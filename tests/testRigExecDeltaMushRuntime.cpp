// This executable deliberately links only rigExecRuntime/rigExecBinary.
// The companion bake fixture supplies the authored sample baked at frame 1;
// no USD is loaded. Playback sets the mover's inputs:defaultWeight to the
// values its stage keys at frames 1, 24 and 48 (0, 1, 0).
#include "rigExecRuntime/runtime.h"
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

int main(int argc, char **argv)
{
    try {
        if (argc != 2) throw std::runtime_error("expected a .rigexec file");
        std::ifstream file(argv[1], std::ios::binary);
        if (!file) throw std::runtime_error("cannot open binary");
        std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(file), {}};
        std::string error;
        auto reader = rigExec::RigExecRuntimeReader::Open(
            bytes.data(), bytes.size(), &error);
        if (!reader) throw std::runtime_error(error);
        const std::string weight =
            "/Rig/Movers/Deform/Detail.inputs:defaultWeight";
        if (reader->GetBakeTime() != 1 || !reader->FindInput(weight, nullptr))
            throw std::runtime_error("expected a bake at frame 1 with " +
                                     weight + " as an input");
        auto evaluate = [&](double value) {
            if (!reader->SetInput(weight, value, &error) ||
                !reader->Execute(&error))
                throw std::runtime_error(error);
            for (const auto &value : reader->GetPoints()) {
                if (value.path == "/Rig/Body.points" && value.points.size() == 6)
                    return value.points;
            }
            throw std::runtime_error("missing deltaMush mesh");
        };
        const auto before = evaluate(0), full = evaluate(1), reset = evaluate(0);
        if (before == full || before != reset || evaluate(1) != full)
            throw std::runtime_error("deltaMush playback/seek/reset failed");
        std::cout << "DeltaMush binary playback passed without USD\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

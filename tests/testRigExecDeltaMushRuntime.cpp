// This executable deliberately links only rigExecRuntime/rigExecBinary.
// The companion bake fixture supplies the authored sample; no USD is loaded.
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
        auto evaluate = [&](double frame) {
            if (!reader->SetFrame(frame, &error) || !reader->Execute(&error))
                throw std::runtime_error(error);
            for (const auto &value : reader->GetPoints()) {
                if (value.path == "/Rig/Body.points" && value.points.size() == 6)
                    return value.points;
            }
            throw std::runtime_error("missing deltaMush mesh");
        };
        const auto before = evaluate(1), full = evaluate(24), reset = evaluate(48);
        if (before == full || before != reset || evaluate(24) != full)
            throw std::runtime_error("deltaMush playback/seek/reset failed");
        std::cout << "DeltaMush binary playback passed without USD\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}

// This quasistatic wrinkle fixture links rigExecRuntime only; no USD is loaded.
#include "rigExecRuntime/runtime.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <stdexcept>

int main(int argc, char **argv) {
    try {
        if (argc != 2) throw std::runtime_error("expected a .rigexec fixture");
        std::ifstream file(argv[1], std::ios::binary);
        if (!file) throw std::runtime_error("cannot open wrinkle fixture");
        std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(file), {}};
        std::string error;
        auto reader = rigExec::RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
        if (!reader) throw std::runtime_error(error);
        auto evaluate = [&](double frame) {
            if (!reader->SetFrame(frame, &error) || !reader->Execute(&error))
                throw std::runtime_error(error);
            for (const auto &value : reader->GetPoints())
                if (value.path == "/Rig/Body.points" && value.points.size() == 91)
                    return value.points;
            throw std::runtime_error("missing wrinkle mesh");
        };
        const auto rest = evaluate(1), compressed = evaluate(24), reset = evaluate(48);
        float height = 0.0f;
        for (const auto &point : compressed) {
            for (int axis = 0; axis < 3; ++axis)
                if (!std::isfinite(point[axis])) throw std::runtime_error("nonfinite wrinkle point");
            height = std::max(height, std::abs(point[2]));
        }
        if (height <= 0.001f || rest == compressed || rest != reset || evaluate(24) != compressed ||
            evaluate(1) != rest)
            throw std::runtime_error("wrinkle playback/seek/reset failed");

        const auto &frames = reader->GetFrameTimes();
        if (frames.size() != 471 || frames.front() != 1 || frames.back() != 48)
            throw std::runtime_error("missing tenth-frame wrinkle animation");
        std::vector<decltype(evaluate(1))> expected;
        for (size_t index = 0; index < frames.size(); ++index) {
            if (frames[index] != 1 + index / 10.0)
                throw std::runtime_error("incorrect wrinkle sample time");
            expected.push_back(evaluate(frames[index]));
        }
        const auto compressionAt = [](double frame) {
            return frame <= 24 ? 1 - 0.35 * (frame - 1) / 23 :
                                 0.65 + 0.35 * (frame - 24) / 24;
        };
        double maxOffsetStep = 0;
        for (size_t index = 1; index < frames.size(); ++index) {
            const double scaleStep = compressionAt(frames[index]) - compressionAt(frames[index - 1]);
            for (size_t point = 0; point < rest.size(); ++point) {
                double lengthSquared = 0;
                for (int axis = 0; axis < 3; ++axis) {
                    const double current = expected[index][point][axis];
                    const double previous = expected[index - 1][point][axis];
                    if (!std::isfinite(current) || !std::isfinite(previous))
                        throw std::runtime_error("nonfinite animated wrinkle point");
                    const double delta = current - previous - (axis == 0 ? rest[point][0] * scaleStep : 0);
                    lengthSquared += delta * delta;
                }
                maxOffsetStep = std::max(maxOffsetStep, std::sqrt(lengthSquared));
            }
        }
        if (maxOffsetStep >= 0.01)
            throw std::runtime_error("wrinkle folds pop between subframes");
        for (size_t index = frames.size(); index-- > 0;)
            if (evaluate(frames[index]) != expected[index])
                throw std::runtime_error("wrinkle reverse playback changed folds");
        std::vector<size_t> order(frames.size());
        std::iota(order.begin(), order.end(), size_t(0));
        std::mt19937 random(617);
        std::shuffle(order.begin(), order.end(), random);
        for (size_t index : order)
            if (evaluate(frames[index]) != expected[index])
                throw std::runtime_error("wrinkle random seek changed folds");

        // A new reader must not need earlier frames to reproduce a subframe.
        const auto fresh = rigExec::RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
        if (!fresh || !fresh->SetFrame(frames[163], &error) || !fresh->Execute(&error))
            throw std::runtime_error(error);
        bool found = false;
        for (const auto &value : fresh->GetPoints()) {
            if (value.path != "/Rig/Body.points") continue;
            found = true;
            if (value.points != expected[163])
                throw std::runtime_error("wrinkle direct subframe seek changed folds");
        }
        if (!found) throw std::runtime_error("missing wrinkle mesh on direct seek");
        std::cout << "Wrinkle binary playback passed without USD\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

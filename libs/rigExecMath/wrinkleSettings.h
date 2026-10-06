#ifndef RIGEXEC_MATH_WRINKLE_SETTINGS_H
#define RIGEXEC_MATH_WRINKLE_SETTINGS_H

#include <vector>

namespace rigExec {

enum class RigExecWrinkleTopology { Cloth = 0, SurfaceStruts = 1 };

/// Quasistatic distance-constraint wrinkle solve in target-local coordinates.
struct RigExecWrinkleSettings {
    int iterations = 80;
    RigExecWrinkleTopology topology = RigExecWrinkleTopology::Cloth;
    int neighborDistance = 2;
    float restLengthScale = 1.0f;
    float stretchStiffness = 1.0f;
    float compressionStiffness = 1.0f;
    float bendStiffness = 0.1f;
    float maxDisplacement = 0.2f;
    bool pinBorders = true;
    bool tangentPlaneCollisions = true;
    float tangentPlaneInset = 0.0f;
    float wrinkleScale = 1.0f;
    int smoothingIterations = 0;
    std::vector<int> pinPoints;

    bool operator==(const RigExecWrinkleSettings &other) const
    {
        return iterations == other.iterations && topology == other.topology &&
            neighborDistance == other.neighborDistance &&
            restLengthScale == other.restLengthScale &&
            stretchStiffness == other.stretchStiffness &&
            compressionStiffness == other.compressionStiffness &&
            bendStiffness == other.bendStiffness &&
            maxDisplacement == other.maxDisplacement &&
            pinBorders == other.pinBorders &&
            tangentPlaneCollisions == other.tangentPlaneCollisions &&
            tangentPlaneInset == other.tangentPlaneInset &&
            wrinkleScale == other.wrinkleScale &&
            smoothingIterations == other.smoothingIterations &&
            pinPoints == other.pinPoints;
    }
    bool operator!=(const RigExecWrinkleSettings &other) const { return !(*this == other); }
};

} // namespace rigExec
#endif // RIGEXEC_MATH_WRINKLE_SETTINGS_H

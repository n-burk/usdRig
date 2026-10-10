// USD-free. Compares the runtime's serial skin loop with the parallel
// split, including a NaN payload, and prints the 26,276-point timings.
#include "rigExecRuntime/runtime.h"

#include <iostream>
#include <string>

int main()
{
    std::string error;
    double linearSerialUs = 0;
    double linearParallelUs = 0;
    double dualSerialUs = 0;
    double dualParallelUs = 0;
    if (!rigExec::RrGeoCompareSkinSplitForTesting(
            &error, &linearSerialUs, &linearParallelUs, &dualSerialUs,
            &dualParallelUs)) {
        std::cerr << error << "\n";
        return 1;
    }
    std::cout << "skin linear 26276 pts serial_us=" << linearSerialUs
              << " parallel_us=" << linearParallelUs << "\n"
              << "skin dualQuaternion 26276 pts serial_us=" << dualSerialUs
              << " parallel_us=" << dualParallelUs << "\n";
    return 0;
}

"""Numeric inverse correctness, bounded failure and pure callback contract."""

import math
import sys

from test_rigexec_python import _setup_environment
_setup_environment()

from rigexec import solve_parameters


def main():
    initial = [0.0, 0.0]
    desired = [5.0, 1.0]
    forward = lambda p: (2 * p[0] + p[1], p[0] - p[1])
    solved = solve_parameters(forward, initial, desired)
    assert solved.converged and solved.reason == "target_reached", solved
    assert abs(solved.parameters[0] - 2.0) < 1e-5, solved
    assert abs(solved.parameters[1] - 1.0) < 1e-5, solved
    assert initial == [0.0, 0.0] and desired == [5.0, 1.0]
    analytic = solve_parameters(forward, initial, desired,
                                jacobian=lambda p: ((2, 1), (1, -1)))
    assert analytic.converged and analytic.evaluations < solved.evaluations

    # A planar two-bone endpoint, with two continuous joint-angle parameters.
    def endpoint(p):
        assert isinstance(p, tuple)
        return (math.cos(p[0]) + math.cos(p[0] + p[1]),
                math.sin(p[0]) + math.sin(p[0] + p[1]))
    desired_endpoint = endpoint((0.35, 1.1))
    ik = solve_parameters(endpoint, (0.0, 0.5), desired_endpoint,
                          bounds=((-math.pi, math.pi), (0.0, math.pi)))
    assert ik.converged and ik.residual_norm < 1e-5, ik
    assert max(abs(a - b) for a, b in zip(ik.parameters, (0.35, 1.1))) < 1e-4, ik

    # Rank-deficient maps remain solvable because damping stabilizes J^T J.
    rank_deficient = solve_parameters(lambda p: (p[0] + p[1],), (0, 0), (3,))
    assert rank_deficient.converged, rank_deficient
    bounded = solve_parameters(lambda p: p, (0,), (2,), bounds=((0, 1),))
    assert not bounded.converged and bounded.parameters == (1.0,), bounded
    assert bounded.reason in ("stalled", "relative_improvement"), bounded
    fixed = solve_parameters(lambda p: p, (0,), (2,), bounds=((0, 0),))
    assert not fixed.converged and fixed.parameters == (0.0,), fixed
    limited = solve_parameters(endpoint, (0, 0.5), desired_endpoint, max_iterations=0)
    assert limited.reason == "iteration_limit" and limited.evaluations == 1
    invalid = solve_parameters(lambda p: (float("nan"),), (0,), (1,))
    assert invalid.reason == "non_finite_forward" and not invalid.converged
    weighted = solve_parameters(lambda p: (p[0], 100), (0,), (2, 0), weights=(1, 0))
    assert weighted.converged, weighted
    regularized = solve_parameters(lambda p: p, (0,), (2,), regularization=1.0)
    assert abs(regularized.parameters[0] - 1.0) < 1e-4, regularized
    assert not regularized.converged
    for kwargs in ({"bounds": ((2, 1),)}, {"weights": (-1,)},
                   {"damping": 0}, {"character_scale": float("inf")}):
        try:
            solve_parameters(lambda p: p, (0,), (1,), **kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid inputs were accepted: %r" % kwargs)
    print("RIGEXEC_INVERSE_OK")


if __name__ == "__main__":
    main()

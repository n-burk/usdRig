"""Pure bounded inverse parameter solving for explicitly supplied forward maps.

The callback receives an immutable parameter tuple and returns a flat landmark
vector. It must not author USD or retain simulation state. Selecting landmarks,
evaluation overrides, committing channels and undo belong to the caller.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class InverseResult:
    parameters: tuple
    output: tuple
    converged: bool
    reason: str
    iterations: int
    evaluations: int
    residual_norm: float
    objective: float


def _linear_solve(matrix, rhs):
    """Partial-pivot elimination; damping makes the normal matrix positive."""
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    size = len(rhs)
    for col in range(size):
        pivot = max(range(col, size), key=lambda r: abs(rows[r][col]))
        if abs(rows[pivot][col]) < 1e-30:
            return None
        rows[col], rows[pivot] = rows[pivot], rows[col]
        for row in range(col + 1, size):
            factor = rows[row][col] / rows[col][col]
            for k in range(col + 1, size + 1):
                rows[row][k] -= factor * rows[col][k]
    result = [0.0] * size
    for row in range(size - 1, -1, -1):
        result[row] = (rows[row][size] - sum(
            rows[row][col] * result[col] for col in range(row + 1, size))) / rows[row][row]
    return result


def solve_parameters(forward, initial, desired, *, bounds=None, weights=None,
                     regularization=0.0, reference=None, jacobian=None,
                     character_scale=1.0, tolerance=None,
                     relative_tolerance=1e-6, max_iterations=64,
                     damping=1e-3, difference_step=1e-5):
    """Minimize weighted landmark error plus distance from a reference pose.

    Uses a supplied analytic Jacobian (rows are output components, columns are
    parameters), or bounded central finite differences for a prototype forward
    callback. ``weights`` multiply residual components, ``regularization`` is
    the nonnegative coefficient of squared distance from ``reference`` (the
    initial pose by default), and bounds are inclusive ``(lower, upper)`` pairs.
    The initial pose must satisfy its bounds. No input sequence is modified.

    Success means weighted landmark error is below ``tolerance``, which defaults
    to 1e-5 * character_scale. Stationarity, unreachable bounded targets, invalid
    forward values and the iteration limit return an explicit unsuccessful
    result. Invalid arguments or callback exceptions raise without swallowing
    the caller's error. This dense solver is intended for small channel subsets.
    """
    x = tuple(float(v) for v in initial)
    target = tuple(float(v) for v in desired)
    if not x or not target or not all(math.isfinite(v) for v in x + target):
        raise ValueError("initial and desired must be nonempty finite vectors")
    n, m = len(x), len(target)
    w = tuple(float(v) for v in weights) if weights is not None else (1.0,) * m
    ref = tuple(float(v) for v in reference) if reference is not None else x
    limits = tuple((float(a), float(b)) for a, b in bounds) if bounds is not None \
        else ((-math.inf, math.inf),) * n
    if len(w) != m or any(not math.isfinite(v) or v < 0 for v in w):
        raise ValueError("weights must be finite, nonnegative and match desired")
    if len(ref) != n or not all(math.isfinite(v) for v in ref):
        raise ValueError("reference must be finite and match initial")
    if len(limits) != n or any(math.isnan(a) or math.isnan(b) or not a <= v <= b
                              for v, (a, b) in zip(x, limits)):
        raise ValueError("bounds must contain each initial parameter")
    tolerance = 1e-5 * character_scale if tolerance is None else tolerance
    if any(not math.isfinite(v) or v <= 0
           for v in (character_scale, tolerance, damping, difference_step)):
        raise ValueError("scale, tolerance, damping and difference step must be positive")
    if (not math.isfinite(regularization) or regularization < 0
            or not math.isfinite(relative_tolerance) or relative_tolerance < 0
            or not isinstance(max_iterations, int) or max_iterations < 0):
        raise ValueError("regularization, relative tolerance and iteration count must be nonnegative")
    evaluations = 0

    def evaluate(values):
        nonlocal evaluations
        evaluations += 1
        result = tuple(float(v) for v in forward(tuple(values)))
        if len(result) != m:
            raise ValueError("forward output cardinality must match desired")
        return result

    def scores(values, output):
        residuals = [weight * (value - wanted) for weight, value, wanted in zip(w, output, target)]
        norm = math.hypot(*residuals)
        penalty = (regularization * sum((a - b) * (a - b) for a, b in zip(values, ref))
                   if regularization else 0.0)
        return norm, norm * norm + penalty

    output = evaluate(x)
    norm, objective = (scores(x, output) if all(math.isfinite(v) for v in output)
                       else (math.inf, math.inf))

    def result(reason, iteration):
        return InverseResult(x, output, norm <= tolerance, reason, iteration,
                             evaluations, norm, objective)

    if not math.isfinite(objective):
        return result("non_finite_forward", 0)
    for iteration in range(max_iterations + 1):
        if norm <= tolerance:
            return result("target_reached", iteration)
        if iteration == max_iterations:
            return result("iteration_limit", iteration)
        if jacobian is not None:
            j = [list(map(float, row)) for row in jacobian(x)]
            if len(j) != m or any(len(row) != n for row in j):
                raise ValueError("Jacobian must have output rows and parameter columns")
        else:
            j = [[0.0] * n for _ in range(m)]
            for col, (lower, upper) in enumerate(limits):
                step = difference_step * max(1.0, abs(x[col]))
                lo, hi = max(lower, x[col] - step), min(upper, x[col] + step)
                if lo == hi:
                    continue
                left, right = list(x), list(x)
                left[col], right[col] = lo, hi
                a = output if lo == x[col] else evaluate(left)
                b = output if hi == x[col] else evaluate(right)
                for row in range(m):
                    j[row][col] = (b[row] - a[row]) / (hi - lo)
        if any(not math.isfinite(v) for row in j for v in row):
            return result("non_finite_jacobian", iteration)
        normal = [[sum(w[k] * w[k] * j[k][a] * j[k][b] for k in range(m))
                   + (regularization if a == b else 0.0)
                   for b in range(n)] for a in range(n)]
        gradient = [sum(w[k] * w[k] * j[k][a] * (output[k] - target[k]) for k in range(m))
                    + regularization * (x[a] - ref[a]) for a in range(n)]
        if (any(not math.isfinite(v) for row in normal for v in row)
                or any(not math.isfinite(v) for v in gradient)):
            return result("non_finite_normal_equations", iteration)
        accepted = False
        for _ in range(12):
            damped = [row[:] for row in normal]
            for col in range(n):
                damped[col][col] += damping * max(1.0, normal[col][col])
            delta = _linear_solve(damped, [-v for v in gradient])
            if delta is None or not all(math.isfinite(v) for v in delta):
                damping *= 10.0
                continue
            candidate = tuple(min(b, max(a, v + step))
                              for v, step, (a, b) in zip(x, delta, limits))
            proposed = evaluate(candidate)
            if not all(math.isfinite(v) for v in proposed):
                damping *= 10.0
                continue
            next_norm, next_objective = scores(candidate, proposed)
            if next_objective < objective:
                improvement = (objective - next_objective) / max(objective, 1e-30)
                x, output, norm, objective = candidate, proposed, next_norm, next_objective
                damping = max(1e-15, damping / 3.0)
                accepted = True
                break
            damping *= 10.0
        if not accepted:
            return result("stalled", iteration + 1)
        if improvement <= relative_tolerance and norm > tolerance:
            return result("relative_improvement", iteration + 1)
    raise AssertionError("unreachable iteration state")

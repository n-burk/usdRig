//
// Sparse symmetric positive-definite direct solver.
//
// The Profile Mover factorizes one matrix per binding epoch and reuses it for
// twelve right-hand sides every frame (nine gradient columns, three position
// columns), which is exactly the shape a direct method is for. The USD build
// carries no sparse linear algebra -- no Eigen, no CHOLMOD, which the paper
// itself uses -- so this supplies the minimum: a fill-reducing ordering, a
// symbolic analysis, and an up-looking sparse Cholesky.
//
// Only what the solver needs is here. It is not a general linear algebra
// library and deliberately does not try to be one.
//
#ifndef RIGEXEC_MATH_SPARSE_SOLVE_H
#define RIGEXEC_MATH_SPARSE_SOLVE_H

#include <string>
#include <vector>

namespace rigExec {

/// Accumulates (row, column, value) triplets and compresses them into the
/// lower triangle of a symmetric matrix. Duplicate entries sum, which is what
/// lets a caller assemble element-by-element without tracking whether a slot
/// was already touched.
class RigExecSparseBuilder
{
public:
    explicit RigExecSparseBuilder(int size) : _size(size) {}

    /// Adds \p value at (row, column).
    ///
    /// The matrix is symmetric and only its lower triangle is stored, so
    /// entries strictly above the diagonal are DISCARDED rather than
    /// mirrored. That is what lets a caller scatter a dense symmetric element
    /// matrix over all its index pairs -- as the cut-face assembly does --
    /// without every off-diagonal being counted twice.
    void Add(int row, int column, double value);

    int GetSize() const { return _size; }
    size_t GetTripletCount() const { return _rows.size(); }

private:
    friend class RigExecSparseCholesky;
    int _size = 0;
    std::vector<int> _rows;
    std::vector<int> _columns;
    std::vector<double> _values;
};

/// Sparse Cholesky factorization A = L L^T with a minimum-degree ordering.
class RigExecSparseCholesky
{
public:
    /// Analyzes and factorizes.
    ///
    /// \p regularization is added to every diagonal entry before factoring.
    /// Zero is the honest default; a caller that knows its matrix is only
    /// semi-definite (a mesh component no curve constrains) should exclude
    /// those rows rather than regularize them into a fictitious answer.
    ///
    /// Returns false and fills \p error when a pivot is not positive, naming
    /// the column -- which for the Profile Mover means a set of vertices the
    /// curvenet does not reach.
    bool Factorize(const RigExecSparseBuilder &matrix, double regularization,
                   std::string *error);

    /// Solves A x = b for \p columns right-hand sides.
    ///
    /// \p rhs and \p out are column-major: entry (i, c) at [c * n + i]. \p out
    /// is resized. Safe to call concurrently on one factorization.
    void Solve(const std::vector<double> &rhs, int columns,
               std::vector<double> *out) const;

    int GetSize() const { return _size; }
    /// Nonzeros in L, for reporting.
    size_t GetFactorNonzeros() const { return _values.size(); }
    bool IsFactorized() const { return _factorized; }

private:
    int _size = 0;
    bool _factorized = false;
    std::vector<int> _permutation;         ///< new index -> old index
    std::vector<int> _inversePermutation;  ///< old index -> new index
    std::vector<int> _columnStart;         ///< L, CSC, size+1
    std::vector<int> _rowIndex;
    std::vector<double> _values;
    std::vector<double> _diagonal;
    std::vector<int> _parent;              ///< elimination tree
};

/// Minimum-degree ordering of a symmetric pattern given as adjacency lists.
/// Exposed for testing; Factorize calls it internally.
std::vector<int> RigExecMinimumDegreeOrder(
    const std::vector<std::vector<int>> &adjacency);

}  // namespace rigExec

#endif  // RIGEXEC_MATH_SPARSE_SOLVE_H

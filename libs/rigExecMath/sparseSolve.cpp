//
// Sparse Cholesky: minimum-degree ordering, elimination tree, up-looking
// numeric factorization.
//
// The algorithms are the standard ones (Davis, "Direct Methods for Sparse
// Linear Systems"): an elimination tree, `ereach` to find the nonzero pattern
// of one row of L by walking that tree, and an up-looking factorization that
// produces L one row at a time. Up-looking is chosen over left-looking or
// supernodal because it needs no column counts and no dense frontal work, and
// the matrices here -- one-ring sparsity on a surface mesh -- are exactly the
// case where that simplicity costs nothing.
//
#include "sparseSolve.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <numeric>
#include <queue>

namespace rigExec {

void RigExecSparseBuilder::Add(int row, int column, double value)
{
    if (row < 0 || column < 0 || row >= _size || column >= _size) {
        return;
    }
    if (value == 0.0) {
        return;
    }
    if (row < column) {
        return;  // symmetric: the lower-triangle twin carries this value
    }
    _rows.push_back(row);
    _columns.push_back(column);
    _values.push_back(value);
}

namespace {

/// Compresses triplets into lower-triangular CSC with duplicates summed.
void _Compress(int n, const std::vector<int> &rows,
               const std::vector<int> &columns,
               const std::vector<double> &values,
               std::vector<int> *columnStart, std::vector<int> *rowIndex,
               std::vector<double> *out)
{
    std::vector<int> counts(n + 1, 0);
    for (int c : columns) {
        ++counts[c + 1];
    }
    for (int i = 0; i < n; ++i) {
        counts[i + 1] += counts[i];
    }
    std::vector<int> cursor(counts.begin(), counts.end() - 1);
    std::vector<int> scratchRow(rows.size());
    std::vector<double> scratchValue(rows.size());
    for (size_t k = 0; k < rows.size(); ++k) {
        const int slot = cursor[columns[k]]++;
        scratchRow[slot] = rows[k];
        scratchValue[slot] = values[k];
    }

    columnStart->assign(n + 1, 0);
    rowIndex->clear();
    out->clear();
    rowIndex->reserve(rows.size());
    out->reserve(rows.size());
    std::vector<std::pair<int, double>> column;
    for (int c = 0; c < n; ++c) {
        column.clear();
        for (int k = counts[c]; k < counts[c + 1]; ++k) {
            column.push_back({scratchRow[k], scratchValue[k]});
        }
        std::sort(column.begin(), column.end(),
                  [](const std::pair<int, double> &a,
                     const std::pair<int, double> &b) {
                      return a.first < b.first;
                  });
        for (size_t k = 0; k < column.size();) {
            size_t j = k;
            double sum = 0.0;
            while (j < column.size() && column[j].first == column[k].first) {
                sum += column[j].second;
                ++j;
            }
            rowIndex->push_back(column[k].first);
            out->push_back(sum);
            k = j;
        }
        (*columnStart)[c + 1] = int(rowIndex->size());
    }
}

/// Elimination tree of a lower-triangular CSC pattern (Liu's algorithm with
/// path compression).
std::vector<int> _EliminationTree(int n, const std::vector<int> &columnStart,
                                  const std::vector<int> &rowIndex)
{
    std::vector<int> parent(n, -1);
    std::vector<int> ancestor(n, -1);
    // The pattern is stored by column of the lower triangle, i.e. for column
    // j it holds rows i >= j. The tree needs, for each row i, the columns
    // j < i that appear in it -- which is the same set read the other way,
    // so walk columns and treat each stored (i, j) as row i touching j.
    std::vector<std::vector<int>> rowsOf(n);
    for (int j = 0; j < n; ++j) {
        for (int p = columnStart[j]; p < columnStart[j + 1]; ++p) {
            const int i = rowIndex[p];
            if (i > j) {
                rowsOf[i].push_back(j);
            }
        }
    }
    for (int k = 0; k < n; ++k) {
        for (int j : rowsOf[k]) {
            int i = j;
            while (i != -1 && i < k) {
                const int next = ancestor[i];
                ancestor[i] = k;
                if (next == -1) {
                    parent[i] = k;
                    break;
                }
                i = next;
            }
        }
    }
    return parent;
}

/// Strictly-below-diagonal entries of A by ROW.
///
/// Both the up-looking factorization and `ereach` want row k of A restricted
/// to columns j < k. Lower-triangular CSC stores that as "column j contains
/// row k", so the row view is the transpose, built once rather than searched
/// for per row -- searching every column per row is what makes a textbook
/// transcription of this quadratic.
struct _RowView {
    std::vector<int> start;   ///< n+1
    std::vector<int> column;
    std::vector<double> value;
};

_RowView _BuildRowView(int n, const std::vector<int> &columnStart,
                       const std::vector<int> &rowIndex,
                       const std::vector<double> &values)
{
    _RowView view;
    view.start.assign(n + 1, 0);
    for (int j = 0; j < n; ++j) {
        for (int p = columnStart[j]; p < columnStart[j + 1]; ++p) {
            if (rowIndex[p] > j) {
                ++view.start[rowIndex[p] + 1];
            }
        }
    }
    for (int i = 0; i < n; ++i) {
        view.start[i + 1] += view.start[i];
    }
    view.column.assign(view.start[n], 0);
    view.value.assign(view.start[n], 0.0);
    std::vector<int> cursor(view.start.begin(), view.start.end() - 1);
    for (int j = 0; j < n; ++j) {
        for (int p = columnStart[j]; p < columnStart[j + 1]; ++p) {
            const int i = rowIndex[p];
            if (i > j) {
                const int slot = cursor[i]++;
                view.column[slot] = j;
                view.value[slot] = values[p];
            }
        }
    }
    return view;
}

/// Nonzero pattern of row k of L, in topological order (Davis `cs_ereach`).
/// Returns the index of the first written entry in \p stack; entries
/// [top, n) are the pattern.
int _EReach(int k, const _RowView &rows, const std::vector<int> &parent,
            std::vector<int> *stack, std::vector<char> *marked)
{
    const int n = int(stack->size());
    int top = n;
    (*marked)[k] = 1;
    for (int p = rows.start[k]; p < rows.start[k + 1]; ++p) {
        int i = rows.column[p];
        int length = 0;
        std::vector<int> &s = *stack;
        while (i != -1 && !(*marked)[i]) {
            s[length++] = i;
            (*marked)[i] = 1;
            i = parent[i];
        }
        while (length > 0) {
            s[--top] = s[--length];
        }
    }
    for (int p = top; p < n; ++p) {
        (*marked)[(*stack)[p]] = 0;
    }
    (*marked)[k] = 0;
    return top;
}

}  // namespace

std::vector<int> RigExecMinimumDegreeOrder(
    const std::vector<std::vector<int>> &adjacency)
{
    const int n = int(adjacency.size());
    std::vector<std::vector<int>> neighbors(n);
    for (int i = 0; i < n; ++i) {
        neighbors[i] = adjacency[i];
        std::sort(neighbors[i].begin(), neighbors[i].end());
        neighbors[i].erase(
            std::unique(neighbors[i].begin(), neighbors[i].end()),
            neighbors[i].end());
        neighbors[i].erase(
            std::remove(neighbors[i].begin(), neighbors[i].end(), i),
            neighbors[i].end());
    }

    std::vector<char> eliminated(n, 0);
    // Lazy-deletion heap: an entry is stale when its recorded degree no
    // longer matches the vertex's current one.
    using Entry = std::pair<int, int>;  // (degree, vertex)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    for (int i = 0; i < n; ++i) {
        queue.push({int(neighbors[i].size()), i});
    }

    std::vector<int> order;
    order.reserve(n);
    std::vector<int> merged;
    while (!queue.empty()) {
        const Entry top = queue.top();
        queue.pop();
        const int v = top.second;
        if (eliminated[v] || top.first != int(neighbors[v].size())) {
            continue;
        }
        eliminated[v] = 1;
        order.push_back(v);

        // Eliminating v makes its remaining neighbours a clique.
        std::vector<int> live;
        live.reserve(neighbors[v].size());
        for (int u : neighbors[v]) {
            if (!eliminated[u]) {
                live.push_back(u);
            }
        }
        for (int u : live) {
            merged.clear();
            // union(neighbors[u], live) minus u and eliminated vertices
            std::set_union(neighbors[u].begin(), neighbors[u].end(),
                           live.begin(), live.end(),
                           std::back_inserter(merged));
            merged.erase(std::remove_if(merged.begin(), merged.end(),
                                        [&](int x) {
                                            return x == u || eliminated[x];
                                        }),
                         merged.end());
            merged.erase(std::unique(merged.begin(), merged.end()),
                         merged.end());
            neighbors[u].swap(merged);
            queue.push({int(neighbors[u].size()), u});
        }
        neighbors[v].clear();
        neighbors[v].shrink_to_fit();
    }
    // Any vertex the heap never surfaced (isolated after elimination).
    for (int i = 0; i < n; ++i) {
        if (!eliminated[i]) {
            order.push_back(i);
        }
    }
    return order;
}

bool RigExecSparseCholesky::Factorize(const RigExecSparseBuilder &matrix,
                                      double regularization,
                                      std::string *error)
{
    _factorized = false;
    _size = matrix._size;
    const int n = _size;
    if (n <= 0) {
        _columnStart.assign(1, 0);
        _rowIndex.clear();
        _values.clear();
        _diagonal.clear();
        _permutation.clear();
        _inversePermutation.clear();
        _factorized = true;
        return true;
    }

    std::vector<int> inputStart, inputRow;
    std::vector<double> inputValue;
    _Compress(n, matrix._rows, matrix._columns, matrix._values, &inputStart,
              &inputRow, &inputValue);

    // Ordering from the symmetric pattern.
    std::vector<std::vector<int>> adjacency(n);
    for (int j = 0; j < n; ++j) {
        for (int p = inputStart[j]; p < inputStart[j + 1]; ++p) {
            const int i = inputRow[p];
            if (i != j) {
                adjacency[i].push_back(j);
                adjacency[j].push_back(i);
            }
        }
    }
    _permutation = RigExecMinimumDegreeOrder(adjacency);
    _inversePermutation.assign(n, 0);
    for (int k = 0; k < n; ++k) {
        _inversePermutation[_permutation[k]] = k;
    }

    // Permute into the lower triangle of the reordered matrix.
    RigExecSparseBuilder permuted(n);
    for (int j = 0; j < n; ++j) {
        for (int p = inputStart[j]; p < inputStart[j + 1]; ++p) {
            const int i = inputRow[p];
            double value = inputValue[p];
            if (i == j) {
                value += regularization;
            }
            // The input holds each off-diagonal once, in its lower triangle;
            // the permutation can send it above the diagonal, where Add would
            // discard it. Reflect it back -- the matrix is symmetric, so this
            // preserves exactly one copy.
            int row = _inversePermutation[i];
            int column = _inversePermutation[j];
            if (row < column) {
                std::swap(row, column);
            }
            permuted.Add(row, column, value);
        }
    }
    std::vector<int> start, row;
    std::vector<double> value;
    _Compress(n, permuted._rows, permuted._columns, permuted._values, &start,
              &row, &value);

    _parent = _EliminationTree(n, start, row);
    const _RowView rows = _BuildRowView(n, start, row, value);
    std::vector<double> inputDiagonal(n, 0.0);
    for (int j = 0; j < n; ++j) {
        for (int p = start[j]; p < start[j + 1]; ++p) {
            if (row[p] == j) {
                inputDiagonal[j] += value[p];
            }
        }
    }

    // Up-looking numeric factorization. Row k of L is solved against the
    // already-computed rows named by ereach(k).
    _columnStart.assign(n + 1, 0);
    _rowIndex.clear();
    _values.clear();
    _diagonal.assign(n, 0.0);

    // L is built by columns, so rows are accumulated then scattered. Collect
    // per-column entries first.
    std::vector<std::vector<std::pair<int, double>>> columns(n);
    std::vector<double> work(n, 0.0);
    std::vector<int> stack(n, 0);
    std::vector<char> marked(n, 0);
    // Column-major view of the partially built L for the triangular solve.
    std::vector<std::vector<std::pair<int, double>>> &lower = columns;

    for (int k = 0; k < n; ++k) {
        // Scatter row k of A into the dense workspace.
        double diagonalValue = inputDiagonal[k];
        for (int p = rows.start[k]; p < rows.start[k + 1]; ++p) {
            work[rows.column[p]] += rows.value[p];
        }

        const int top = _EReach(k, rows, _parent, &stack, &marked);

        // Forward-solve L(0:k-1, 0:k-1) * y = A(k, 0:k-1)^T over the pattern.
        for (int s = top; s < n; ++s) {
            const int j = stack[s];
            const double y = work[j] / _diagonal[j];
            work[j] = 0.0;
            for (const auto &entry : lower[j]) {
                if (entry.first > j) {
                    work[entry.first] -= entry.second * y;
                }
            }
            columns[j].push_back({k, y});
            diagonalValue -= y * y;
        }

        if (!(diagonalValue > 0.0) || !std::isfinite(diagonalValue)) {
            if (error) {
                char buffer[224];
                std::snprintf(
                    buffer, sizeof(buffer),
                    "matrix is not positive definite: pivot %d (original index "
                    "%d) is %g. For the Profile Mover this means a set of mesh "
                    "vertices no curvenet constrains.",
                    k, _permutation[k], diagonalValue);
                *error = buffer;
            }
            return false;
        }
        _diagonal[k] = std::sqrt(diagonalValue);
    }

    // Flatten into CSC. The diagonal is kept separately; stored columns hold
    // strictly-below-diagonal entries.
    _columnStart[0] = 0;
    for (int j = 0; j < n; ++j) {
        std::sort(columns[j].begin(), columns[j].end());
        for (const auto &entry : columns[j]) {
            _rowIndex.push_back(entry.first);
            _values.push_back(entry.second);
        }
        _columnStart[j + 1] = int(_rowIndex.size());
    }

    _factorized = true;
    return true;
}

void RigExecSparseCholesky::Solve(const std::vector<double> &rhs, int columns,
                                  std::vector<double> *out) const
{
    const int n = _size;
    out->assign(size_t(n) * size_t(std::max(columns, 0)), 0.0);
    if (!_factorized || n == 0 || columns <= 0) {
        return;
    }
    std::vector<double> x(n, 0.0);
    for (int c = 0; c < columns; ++c) {
        const double *b = rhs.data() + size_t(c) * size_t(n);
        for (int k = 0; k < n; ++k) {
            x[k] = b[_permutation[k]];
        }
        // L y = Pb
        for (int j = 0; j < n; ++j) {
            x[j] /= _diagonal[j];
            const double y = x[j];
            for (int p = _columnStart[j]; p < _columnStart[j + 1]; ++p) {
                x[_rowIndex[p]] -= _values[p] * y;
            }
        }
        // L^T z = y
        for (int j = n - 1; j >= 0; --j) {
            double sum = x[j];
            for (int p = _columnStart[j]; p < _columnStart[j + 1]; ++p) {
                sum -= _values[p] * x[_rowIndex[p]];
            }
            x[j] = sum / _diagonal[j];
        }
        double *result = out->data() + size_t(c) * size_t(n);
        for (int k = 0; k < n; ++k) {
            result[_permutation[k]] = x[k];
        }
    }
}

}  // namespace rigExec

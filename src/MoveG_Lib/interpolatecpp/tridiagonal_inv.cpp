/**
 * @file tridiagonal_solver.cpp
 * @brief Implementation of the tridiagonal system solver using the Thomas algorithm
  *
 * @author Giorgio Medico
 * @date April 09, 2025
 */

#include "interpolatecpp/tridiagonal_inv.h"

namespace InterpolateCpp
{

std::vector<double> solve_tridiagonal(std::vector<double> lower_diagonal,
                                      std::vector<double> main_diagonal,
                                      std::vector<double> upper_diagonal,
                                      std::vector<double> right_hand_side)
{
    // Check input dimensions
    const size_t n = right_hand_side.size();
    if (main_diagonal.size() != n or lower_diagonal.size() != n or upper_diagonal.size() != n)
    {
        throw std::invalid_argument("All input arrays must have the same length");
    }

    auto &a = lower_diagonal;
    auto &b = main_diagonal;
    auto &c = upper_diagonal;
    auto &d = right_hand_side;

    // Check for zero pivot
    if (n > 0 && b[0] == 0.0)
    {
        throw std::runtime_error(
            "Pivot cannot be zero. The system cannot be solved with this method.");
    }

    // Forward elimination
    for (size_t k = 1; k < n; ++k)
    {
        // Ensure we don't divide by zero
        if (b[k - 1] == 0.0)
        {
            throw std::runtime_error("Encountered zero pivot during elimination at index " +
                                     std::to_string(k - 1));
        }

        double m = a[k] / b[k - 1];
        b[k] -= m * c[k - 1];
        d[k] -= m * d[k - 1];

        // Check for zero pivot during elimination
        if (b[k] == 0.0)
        {
            throw std::runtime_error("Encountered zero pivot during elimination at index " +
                                     std::to_string(k));
        }
    }

    // Back substitution
    std::vector<double> x(n, 0.0);

    // Handle empty matrix case
    if (n == 0)
    {
        return x;
    }

    x[n - 1] = d[n - 1] / b[n - 1];

    for (size_t i = 0; i < n - 1; ++i)
    {
        // Calculate k in reverse order: n-2, n-3, ..., 0
        size_t k = n - 2 - i;
        x[k] = (d[k] - c[k] * x[k + 1]) / b[k];
    }

    return x;
}

} // namespace InterpolateCpp

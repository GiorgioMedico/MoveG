/**
 * @file tridiagonal_solver.h
 * @brief Tridiagonal matrix solver implementation using the Thomas algorithm
 *
 * @author Giorgio Medico
 * @date April 09, 2025
 */

#pragma once

#include <stdexcept>
#include <vector>

namespace InterpolateCpp
{

/**
 * @brief Solve a tridiagonal system using the Thomas algorithm.
 *
 * This function solves the equation Ax = b where A is a tridiagonal matrix.
 * The system is solved efficiently using the Thomas algorithm (also known as
 * the tridiagonal matrix algorithm).
 *
 * The algorithm has O(n) time complexity, much more efficient than general
 * Gaussian elimination which has O(n³) complexity.
 *
 * For a system where the matrix A is:
 * [b₀ c₀ 0  0  0 ]
 * [a₁ b₁ c₁ 0  0 ]
 * [0  a₂ b₂ c₂ 0 ]
 * [0  0  a₃ b₃ c₃]
 * [0  0  0  a₄ b₄]
 *
 * @param lower_diagonal Lower diagonal elements (first element is not used).
 *                       Must have the same length as main_diagonal.
 * @param main_diagonal Main diagonal elements.
 * @param upper_diagonal Upper diagonal elements (last element is not used).
 *                       Must have the same length as main_diagonal.
 * @param right_hand_side Right-hand side vector of the equation.
 *
 * @return Solution vector x.
 *
 * @throws std::invalid_argument If input dimensions are inconsistent.
 * @throws std::runtime_error If a pivot is zero during forward elimination.
 *
 * @example
 * std::vector<double> a = {0, 1, 2, 3}; // Lower diagonal (a[0] is not used)
 * std::vector<double> b = {2, 3, 4, 5}; // Main diagonal
 * std::vector<double> c = {1, 2, 3, 0}; // Upper diagonal (c[n-1] is not used)
 * std::vector<double> d = {1, 2, 3, 4}; // Right hand side
 * std::vector<double> x = InterpolateCpp::solve_tridiagonal(a, b, c, d);
 */
std::vector<double> solve_tridiagonal(std::vector<double> lower_diagonal,
                                      std::vector<double> main_diagonal,
                                      std::vector<double> upper_diagonal,
                                      std::vector<double> right_hand_side);

} // namespace InterpolateCpp

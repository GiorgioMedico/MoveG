#define CATCH_CONFIG_MAIN
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <vector>

#include "interpolatecpp/tridiagonal_inv.h"

using namespace InterpolateCpp;

TEST_CASE("Tridiagonal Solver: 3x3 System", "[tridiagonal]")
{
    // Define a simple 3x3 tridiagonal system
    // [ 2 -1  0 ] [ x0 ]   [ 1 ]
    // [-1  2 -1 ] [ x1 ] = [ 0 ]
    // [ 0 -1  2 ] [ x2 ]   [ 1 ]

    std::vector<double> lower_diagonal = {0.0, -1.0, -1.0}; // First element not used
    std::vector<double> main_diagonal = {2.0, 2.0, 2.0};
    std::vector<double> upper_diagonal = {-1.0, -1.0, 0.0}; // Last element not used
    std::vector<double> right_hand_side = {1.0, 0.0, 1.0};

    // Solve using our tridiagonal solver
    std::vector<double> result =
        solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side);

    // Compare with Eigen solver
    Eigen::Matrix3d A;
    A << 2.0, -1.0, 0.0, -1.0, 2.0, -1.0, 0.0, -1.0, 2.0;

    Eigen::Vector3d b(1.0, 0.0, 1.0);
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);

    for (size_t i = 0; i < 3; ++i)
    {
        REQUIRE(result[i] == Catch::Approx(x[i]));
    }
}

TEST_CASE("Tridiagonal Solver: 1x1 System", "[tridiagonal]")
{
    // Simple 1x1 system: [a] [x] = [b]
    std::vector<double> lower_diagonal = {0.0}; // Not used for 1x1
    std::vector<double> main_diagonal = {2.0};
    std::vector<double> upper_diagonal = {0.0}; // Not used for 1x1
    std::vector<double> right_hand_side = {4.0};

    std::vector<double> result =
        solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side);

    REQUIRE(result.size() == 1);
    REQUIRE(result[0] == Catch::Approx(2.0)); // x = b/a = 4/2 = 2
}

TEST_CASE("Tridiagonal Solver: 2x2 System", "[tridiagonal]")
{
    // 2x2 system:
    // [ 2 -1 ] [ x0 ] = [ 1 ]
    // [-1  2 ] [ x1 ] = [ 2 ]
    std::vector<double> lower_diagonal = {0.0, -1.0}; // First element not used
    std::vector<double> main_diagonal = {2.0, 2.0};
    std::vector<double> upper_diagonal = {-1.0, 0.0}; // Last element not used
    std::vector<double> right_hand_side = {1.0, 2.0};

    std::vector<double> result =
        solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side);

    // Calculate expected result
    Eigen::Matrix2d A;
    A << 2.0, -1.0, -1.0, 2.0;

    Eigen::Vector2d b(1.0, 2.0);
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

    REQUIRE(result.size() == 2);
    REQUIRE(result[0] == Catch::Approx(x[0]));
    REQUIRE(result[1] == Catch::Approx(x[1]));
}

TEST_CASE("Tridiagonal Solver: Larger System (10x10)", "[tridiagonal]")
{
    const int n = 10;

    // Create a 10x10 tridiagonal system
    std::vector<double> lower_diagonal(n, -1.0);
    lower_diagonal[0] = 0.0; // First element not used

    std::vector<double> main_diagonal(n, 2.0);

    std::vector<double> upper_diagonal(n, -1.0);
    upper_diagonal[n - 1] = 0.0; // Last element not used

    // Create a right-hand side
    std::vector<double> right_hand_side(n, 0.0);
    right_hand_side[0] = 1.0;
    right_hand_side[n - 1] = 1.0;

    // Solve using our tridiagonal solver
    std::vector<double> result =
        solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side);

    // Compare with Eigen solver
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i)
    {
        A(i, i) = 2.0;
        if (i > 0)
            A(i, i - 1) = -1.0;
        if (i < n - 1)
            A(i, i + 1) = -1.0;
    }

    Eigen::VectorXd b = Eigen::VectorXd::Zero(n);
    b[0] = 1.0;
    b[n - 1] = 1.0;

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    REQUIRE(result.size() == static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        REQUIRE(result[i] == Catch::Approx(x[i]));
    }
}

TEST_CASE("Tridiagonal Solver: Error Cases", "[tridiagonal]")
{
    SECTION("Zero Pivot")
    {
        // System with a zero pivot
        std::vector<double> lower_diagonal = {0.0, -1.0, -1.0};
        std::vector<double> main_diagonal = {0.0, 2.0, 2.0}; // First element is zero
        std::vector<double> upper_diagonal = {-1.0, -1.0, 0.0};
        std::vector<double> right_hand_side = {1.0, 0.0, 1.0};

        // Using a try-catch block to catch any specific exceptions or buffer overflows
        try
        {
            solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side);
            FAIL("Expected std::runtime_error exception but none was thrown");
        }
        catch (const std::runtime_error &e)
        {
            // This is the expected exception
            REQUIRE(std::string(e.what()).find("Pivot cannot be zero") != std::string::npos);
        }
    }

    SECTION("Zero Pivot During Elimination")
    {
        // System that will encounter a zero pivot during elimination
        std::vector<double> lower_diagonal = {0.0, 1.0, -1.0};
        std::vector<double> main_diagonal = {1.0, 1.0, 2.0};
        std::vector<double> upper_diagonal = {1.0, -1.0, 0.0};
        std::vector<double> right_hand_side = {1.0, 0.0, 1.0};

        // This will lead to a zero pivot during the forward elimination step
        // because the second pivot will become: main_diagonal[1] - lower_diagonal[1] * upper_diagonal[0] / main_diagonal[0]
        // = 1.0 - 1.0 * 1.0 / 1.0 = 0.0
        REQUIRE_THROWS_AS(
            solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side),
            std::runtime_error);
    }

    SECTION("Inconsistent Input Sizes")
    {
        std::vector<double> lower_diagonal = {0.0, -1.0};
        std::vector<double> main_diagonal = {2.0, 2.0, 2.0}; // One more element
        std::vector<double> upper_diagonal = {-1.0, 0.0};    // One fewer element
        std::vector<double> right_hand_side = {1.0, 0.0, 1.0};

        REQUIRE_THROWS_AS(
            solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side),
            std::invalid_argument);
    }
}

TEST_CASE("Tridiagonal Solver: Random System", "[tridiagonal]")
{
    // Create a random-sized system
    const int n = 100;

    // Create random diagonals
    std::vector<double> lower_diagonal(n, 0.0);
    std::vector<double> main_diagonal(n, 0.0);
    std::vector<double> upper_diagonal(n, 0.0);

    // Fill with random values, but ensure diagonally dominant for stability
    srand(42); // Fixed seed for reproducibility
    for (int i = 0; i < n; ++i)
    {
        main_diagonal[i] = 10.0 + rand() % 10; // Strong diagonal dominance

        if (i > 0)
        {
            lower_diagonal[i] = -1.0 - (rand() % 5); // Negative values
        }

        if (i < n - 1)
        {
            upper_diagonal[i] = -1.0 - (rand() % 5); // Negative values
        }
    }

    // Create a random right-hand side
    std::vector<double> right_hand_side(n);
    for (int i = 0; i < n; ++i)
    {
        right_hand_side[i] = rand() % 100;
    }

    // Solve using our tridiagonal solver
    std::vector<double> result =
        solve_tridiagonal(lower_diagonal, main_diagonal, upper_diagonal, right_hand_side);

    // Create equivalent Eigen matrix
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i)
    {
        A(i, i) = main_diagonal[i];
        if (i > 0)
            A(i, i - 1) = lower_diagonal[i];
        if (i < n - 1)
            A(i, i + 1) = upper_diagonal[i];
    }

    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i)
    {
        b[i] = right_hand_side[i];
    }

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    // Compare our result with Eigen's
    REQUIRE(result.size() == static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        REQUIRE(result[i] == Catch::Approx(x[i]));
    }

    // Also verify that the solution satisfies the original equation
    for (int i = 0; i < n; ++i)
    {
        double lhs = 0.0;
        if (i > 0)
            lhs += lower_diagonal[i] * result[i - 1];
        lhs += main_diagonal[i] * result[i];
        if (i < n - 1)
            lhs += upper_diagonal[i] * result[i + 1];

        REQUIRE(lhs == Catch::Approx(right_hand_side[i]));
    }
}

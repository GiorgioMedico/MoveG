/**
 * @file trapezoidal.h
 * @brief Class for generating and managing trapezoidal velocity profiles for trajectory planning
 *
 * @author Giorgio Medico
 * @date April 09, 2025
 */

#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace MoveG
{

/**
  * @struct TrajectoryParams
  * @brief Parameters for trapezoidal trajectory generation
  */
struct TrajectoryParams
{
    double q0 = 0.0;                  ///< Initial position
    double q1 = 0.0;                  ///< Final position
    double t0 = 0.0;                  ///< Start time
    double v0 = 0.0;                  ///< Initial velocity
    double v1 = 0.0;                  ///< Final velocity
    std::optional<double> amax{};     ///< Maximum acceleration
    std::optional<double> vmax{};     ///< Maximum velocity
    std::optional<double> duration{}; ///< Desired duration
};

/**
  * @struct CalculationParams
  * @brief Parameters for trajectory calculations
  */
struct CalculationParams
{
    double q0 = 0.0;   ///< Initial position
    double q1 = 0.0;   ///< Final position
    double v0 = 0.0;   ///< Initial velocity
    double v1 = 0.0;   ///< Final velocity
    double amax = 0.0; ///< Maximum acceleration
};

/**
  * @struct InterpolationParams
  * @brief Parameters for multi-point interpolation
  */
struct InterpolationParams
{
    std::vector<double> points;                            ///< Waypoints
    double v0 = 0.0;                                       ///< Initial velocity
    double vn = 0.0;                                       ///< Final velocity
    std::optional<std::vector<double>> inter_velocities{}; ///< Intermediate velocities
    std::optional<std::vector<double>> times{};            ///< Timestamps for waypoints
    double amax = 10.0;                                    ///< Maximum acceleration
    std::optional<double> vmax{};                          ///< Maximum velocity
};

/**
  * @class TrapezoidalTrajectory
  * @brief Generate trapezoidal velocity profiles for trajectory planning
  *
  * This class provides methods to create trapezoidal velocity profiles for various
  * trajectory planning scenarios, including single segment trajectories and
  * multi-point interpolation.
  */
class TrapezoidalTrajectory
{
public:
    /**
      * @brief Default constructor
      */
    TrapezoidalTrajectory() = default;

    /**
      * @brief Default destructor
      */
    ~TrapezoidalTrajectory() = default;

    /**
      * @brief Generate a trapezoidal trajectory with non-null initial and final velocities
      *
      * @param params Parameters for trajectory generation
      * @return A tuple containing:
      *         - Function that computes position, velocity, and acceleration at time t
      *         - Duration of trajectory
      */
    static std::tuple<std::function<std::tuple<double, double, double>(double)>, double>
    generateTrajectory(const TrajectoryParams &params);

    /**
      * @brief Calculate velocities based on height differences
      *
      * @param q_list List of height values [q0, q1, ..., qn]
      * @param v0 Initial velocity (assigned)
      * @param vn Final velocity (assigned)
      * @param v_max Maximum velocity value (positive magnitude)
      * @param amax Maximum acceleration
      * @return Calculated velocities [v0, v1, ..., vn]
      */
    static std::vector<double> calculateHeuristicVelocities(
        const std::vector<double> &q_list,
        double v0,
        double vn,
        std::optional<double> v_max = std::nullopt,
        std::optional<double> amax = std::nullopt);

    /**
      * @brief Generate a trajectory through a sequence of points using trapezoidal velocity profiles
      *
      * @param params Parameters for interpolation
      * @return A tuple containing:
      *         - Function that returns position, velocity, and acceleration at any time t
      *         - Total duration of the trajectory
      */
    static std::tuple<std::function<std::tuple<double, double, double>(double)>, double>
    interpolateWaypoints(const InterpolationParams &params);

private:
    // Constants
    static constexpr int MIN_POINTS = 2;     ///< Minimum number of points needed for interpolation
    static constexpr double EPSILON = 1e-10; ///< Small value to prevent division by zero

    /**
      * @brief Calculate trajectory parameters for duration-based constraints
      *
      * @param params Basic trajectory parameters
      * @param duration Desired duration
      * @return Tuple containing cruise velocity, acceleration time, deceleration time
      */
    static std::tuple<double, double, double> calculateDurationBasedTrajectory(
        const CalculationParams &params,
        double duration);

    /**
      * @brief Calculate trajectory parameters for velocity-based constraints
      *
      * @param params Basic trajectory parameters
      * @param vmax Maximum velocity
      * @return Tuple containing cruise velocity, acceleration time, deceleration time, total duration
      */
    static std::tuple<double, double, double, double> calculateVelocityBasedTrajectory(
        const CalculationParams &params,
        double vmax);
};

} // namespace MoveG

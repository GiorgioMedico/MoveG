/**
 * @file trapezoidal.cpp
 * @brief Implementation of trapezoidal velocity profiles for trajectory planning
 *
 * @author Giorgio Medico
 * @date April 09, 2025
 */

#include "trapezoidal.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace MoveG
{

std::tuple<double, double, double> TrapezoidalTrajectory::calculateDurationBasedTrajectory(
    const CalculationParams &params,
    double duration)
{
    double q0 = params.q0;
    double q1 = params.q1;
    double v0 = params.v0;
    double v1 = params.v1;
    double amax = params.amax;
    double h = q1 - q0;

    // Check feasibility using equation (3.14)
    if (amax * h < std::abs(v0 * v0 - v1 * v1) / 2)
    {
        throw std::runtime_error(
            "Trajectory not feasible. Try increasing amax or reducing velocities.");
    }

    // Check minimum required acceleration (equation 3.15)
    double term_under_sqrt =
        4 * h * h - 4 * h * (v0 + v1) * duration + 2 * (v0 * v0 + v1 * v1) * duration * duration;

    // Ensure term under sqrt is non-negative to avoid numerical issues
    if (term_under_sqrt < 0)
    {
        if (term_under_sqrt > -EPSILON) // Very close to zero, likely numerical error
        {
            term_under_sqrt = 0;
        }
        else
        {
            throw std::runtime_error(
                "Trajectory not feasible with given duration. Try increasing duration.");
        }
    }

    double alim = (2 * h - duration * (v0 + v1) + std::sqrt(term_under_sqrt)) /
                  std::max(duration * duration, EPSILON);

    if (amax < alim)
    {
        // Adjust amax to minimum required
        amax = alim;
        // Note: In production code, you might want to use a logging system here
        // std::cout << "Warning: Using minimum required acceleration: " << alim << std::endl;
    }

    // Calculate constant velocity (vv) from equation in section 3.2.7
    double sqrt_term = amax * amax * duration * duration - 4 * amax * h +
                       2 * amax * (v0 + v1) * duration - (v0 - v1) * (v0 - v1);

    // Ensure sqrt term is non-negative
    if (sqrt_term < 0)
    {
        if (sqrt_term > -EPSILON) // Very close to zero, likely numerical error
        {
            sqrt_term = 0;
        }
        else
        {
            throw std::runtime_error("Numerical issue in trajectory calculation. "
                                     "The parameters may lead to an invalid trajectory.");
        }
    }

    double vv = 0.5 * (v0 + v1 + amax * duration - std::sqrt(sqrt_term));

    // Calculate acceleration and deceleration times with numerical stability
    double ta = (vv - v0) / (amax + EPSILON);
    double td = (vv - v1) / (amax + EPSILON);

    return std::make_tuple(vv, ta, td);
}

std::tuple<double, double, double, double> TrapezoidalTrajectory::calculateVelocityBasedTrajectory(
    const CalculationParams &params,
    double vmax)
{
    double q0 = params.q0;
    double q1 = params.q1;
    double v0 = params.v0;
    double v1 = params.v1;
    double amax = params.amax;
    double h = q1 - q0;

    double vv = 0.0;
    double ta = 0.0;
    double td = 0.0;
    double duration = 0.0;

    // Determine if vmax is reached (Case 1 or Case 2)
    if (h * amax > vmax * vmax - (v0 * v0 + v1 * v1) / 2)
    {
        // vmax is reached
        vv = vmax;

        // Calculate acceleration and deceleration times with numerical stability
        ta = (vmax - v0) / (amax + EPSILON);
        td = (vmax - v1) / (amax + EPSILON);

        // Calculate total duration with numerical stability
        double v0_vmax_ratio = v0 / std::max(vmax, EPSILON);
        double v1_vmax_ratio = v1 / std::max(vmax, EPSILON);

        // Ensure ratios are within valid range to avoid numerical issues
        v0_vmax_ratio = std::clamp(v0_vmax_ratio, -1.0 + EPSILON, 1.0 - EPSILON);
        v1_vmax_ratio = std::clamp(v1_vmax_ratio, -1.0 + EPSILON, 1.0 - EPSILON);

        duration = (h / std::max(vmax, EPSILON)) +
                   (vmax / (2 * amax + EPSILON)) * (1 - v0_vmax_ratio) * (1 - v0_vmax_ratio) +
                   (vmax / (2 * amax + EPSILON)) * (1 - v1_vmax_ratio) * (1 - v1_vmax_ratio);
    }
    else
    {
        // vmax is not reached (triangular profile)
        // Ensure the term under sqrt is non-negative
        double sqrt_term = h * amax + (v0 * v0 + v1 * v1) / 2;
        if (sqrt_term < 0)
        {
            if (sqrt_term > -EPSILON) // Very close to zero, likely numerical error
            {
                sqrt_term = 0;
            }
            else
            {
                throw std::runtime_error(
                    "Invalid trajectory parameters. The calculation resulted in "
                    "a negative value under a square root.");
            }
        }

        double vlim = std::sqrt(sqrt_term);
        vv = vlim;

        // Calculate acceleration and deceleration times with numerical stability
        ta = (vlim - v0) / (amax + EPSILON);
        td = (vlim - v1) / (amax + EPSILON);

        // Total duration
        duration = ta + td;
    }

    return std::make_tuple(vv, ta, td, duration);
}

std::tuple<std::function<std::tuple<double, double, double>(double)>, double>
TrapezoidalTrajectory::generateTrajectory(const TrajectoryParams &params)
{
    // Local variables for better readability
    double q0_orig = params.q0;
    double q1_orig = params.q1;
    double t0_orig = params.t0;
    double v0_orig = params.v0;
    double v1_orig = params.v1;

    // Parameter validation
    if (!params.amax)
    {
        throw std::runtime_error("Maximum acceleration (amax) must be provided");
    }

    if (!params.duration && !params.vmax)
    {
        throw std::runtime_error("Either duration or maximum velocity (vmax) must be provided");
    }

    // Ensure amax and vmax are positive using absolute values if provided
    double amax_val = std::abs(params.amax.value());
    std::optional<double> vmax_val;
    if (params.vmax)
    {
        vmax_val = std::abs(params.vmax.value());
    }

    // Calculate displacement
    double h_orig = q1_orig - q0_orig;

    // Handle negative displacement (q1 < q0) according to section 3.4.2
    bool invert_flag = false;
    double q0_temp = q0_orig;
    double q1_temp = q1_orig;
    double v0_temp = v0_orig;
    double v1_temp = v1_orig;

    if (h_orig < 0)
    {
        invert_flag = true;
        // Transform initial and final positions/velocities with opposite signs
        q0_temp = -q0_orig;
        q1_temp = -q1_orig;
        v0_temp = -v0_orig;
        v1_temp = -v1_orig;
    }

    // Create calculation parameters
    CalculationParams calc_params{q0_temp, q1_temp, v0_temp, v1_temp, amax_val};

    // Pre-declare variables used in the trajectory function
    double ta_val = 0.0;       // Acceleration time
    double td_val = 0.0;       // Deceleration time
    double vv_val = 0.0;       // Cruise velocity
    double duration_val = 0.0; // Total duration

    // Determine which case to use based on provided parameters
    if (params.duration && !params.vmax)
    {
        // Case 1: Preassigned duration and acceleration
        std::tie(vv_val, ta_val, td_val) =
            calculateDurationBasedTrajectory(calc_params, params.duration.value());
        duration_val = params.duration.value();
    }
    else if (params.vmax && !params.duration)
    {
        // Case 2: Preassigned acceleration and velocity
        std::tie(vv_val, ta_val, td_val, duration_val) =
            calculateVelocityBasedTrajectory(calc_params, vmax_val.value());
    }
    else
    {
        // This should not happen due to the parameter validation above
        throw std::runtime_error(
            "Invalid parameter combination. Provide either (amax, duration) or (amax, vmax).");
    }

    double t1_val = t0_orig + duration_val;

    // Define the trajectory function
    auto trajectory_data = std::make_shared<std::tuple<double,
                                                       double,
                                                       double,
                                                       double,
                                                       double,
                                                       double,
                                                       double,
                                                       double,
                                                       double,
                                                       double,
                                                       bool>>(q0_temp,
                                                              q1_temp,
                                                              v0_temp,
                                                              v1_temp,
                                                              vv_val,
                                                              t0_orig,
                                                              t1_val,
                                                              ta_val,
                                                              td_val,
                                                              EPSILON,
                                                              invert_flag);

    // Create a trajectory function that captures the parameters
    auto trajectory_function =
        [data = std::move(trajectory_data)](double t) -> std::tuple<double, double, double> {
        const auto &[q0_cap,
                     q1_cap,
                     v0_cap,
                     v1_cap,
                     vv_cap,
                     t0_cap,
                     t1_cap,
                     ta_cap,
                     td_cap,
                     eps_cap,
                     invert_res_cap] = *data;

        // Ensure t is within bounds
        t = std::clamp(t, t0_cap, t1_cap);

        // Initialize variables
        double position = 0.0;
        double velocity = 0.0;
        double acceleration = 0.0;

        // Ensure ta and td are not too small to avoid numerical issues
        double ta_safe = std::max(ta_cap, eps_cap);
        double td_safe = std::max(td_cap, eps_cap);

        // Calculate trajectory for the given time
        if (t0_cap <= t && t < t0_cap + ta_safe)
        {
            // Acceleration phase
            double dt = t - t0_cap;
            position = q0_cap + v0_cap * dt + (vv_cap - v0_cap) / (2 * ta_safe) * dt * dt;
            velocity = v0_cap + (vv_cap - v0_cap) / ta_safe * dt;
            acceleration = (vv_cap - v0_cap) / ta_safe;
        }
        else if (t0_cap + ta_safe <= t && t < t1_cap - td_safe)
        {
            // Constant velocity phase
            position = q0_cap + v0_cap * ta_safe / 2 + vv_cap * (t - t0_cap - ta_safe / 2);
            velocity = vv_cap;
            acceleration = 0;
        }
        else // t1_cap - td_safe <= t <= t1_cap
        {
            // Deceleration phase
            double dt = t1_cap - t;
            position = q1_cap - v1_cap * dt - (vv_cap - v1_cap) / (2 * td_safe) * dt * dt;
            velocity = v1_cap + (vv_cap - v1_cap) / td_safe * dt;
            acceleration = -(vv_cap - v1_cap) / td_safe;
        }

        // If we had a negative displacement, invert the resulting profiles
        if (invert_res_cap)
        {
            return std::make_tuple(-position, -velocity, -acceleration);
        }
        else
        {
            return std::make_tuple(position, velocity, acceleration);
        }
    };

    return std::make_tuple(trajectory_function, duration_val);
}

std::vector<double> TrapezoidalTrajectory::calculateHeuristicVelocities(
    const std::vector<double> &q_list,
    double v0,
    double vn,
    std::optional<double> v_max,
    std::optional<double> amax)
{
    // Calculate height differences h_k = q_k - q_(k-1)
    std::vector<double> h_values;
    for (size_t k = 1; k < q_list.size(); ++k)
    {
        h_values.push_back(q_list[k] - q_list[k - 1]);
    }

    double v_max_value = 0.0;

    // If v_max is not provided, compute it heuristically
    if (!v_max)
    {
        if (!amax)
        {
            throw std::runtime_error("Either v_max or amax must be provided");
        }

        // Ensure amax is positive
        double amax_value = std::abs(amax.value());

        // OPTION 1: Time-Based Approach
        // Estimate a reasonable total duration for the path and derive velocity
        double total_distance = 0.0;
        for (const auto &h : h_values)
        {
            total_distance += std::abs(h);
        }
        double estimated_duration =
            std::sqrt(2 * total_distance / amax_value); // From acceleration equation
        double v_max_time = total_distance / estimated_duration * 0.75; // 75% of average velocity

        // OPTION 2: Segment-Optimized Approach
        // Calculate optimal velocity for each segment based on its length
        std::vector<double> segment_velocities;
        for (const auto &h : h_values)
        {
            // Calculate velocity that allows comfortable acceleration/deceleration
            double segment_length = std::abs(h);
            // Distance to accelerate from 0 to v and decelerate back to 0 is (v^2)/a
            // We want this to be less than the segment length, solving for v:
            double v_segment = std::sqrt(amax_value * segment_length / 2);
            segment_velocities.push_back(v_segment);
        }

        // Choose a velocity that works well for all segments
        double v_max_segments = 0.0;
        if (!segment_velocities.empty())
        {
            v_max_segments =
                *std::min_element(segment_velocities.begin(), segment_velocities.end()) * 0.8;
        }

        // OPTION 3: Curvature-Based Approach
        // Look at changes in direction to determine velocity
        std::vector<double> direction_changes;
        for (size_t i = 0; i < h_values.size() - 1; ++i)
        {
            // Calculate angle between consecutive segments
            if (h_values[i] * h_values[i + 1] < 0) // Direction change
            {
                direction_changes.push_back(1.0); // Full direction change
            }
            else
            {
                // Calculate relative change in slope
                double rel_change = std::abs(h_values[i + 1] - h_values[i]) /
                                    (std::abs(h_values[i]) + std::abs(h_values[i + 1]));
                direction_changes.push_back(rel_change);
            }
        }

        // More direction changes or sharper changes suggest lower velocity
        double avg_change = 0.0;
        if (!direction_changes.empty())
        {
            avg_change = std::accumulate(direction_changes.begin(), direction_changes.end(), 0.0) /
                         static_cast<double>(direction_changes.size());
        }
        double v_max_curvature = std::sqrt(amax_value * total_distance /
                                           (static_cast<double>(h_values.size()) + 5 * avg_change));

        // Choose the minimum of all approaches for safety
        v_max_value = std::min({v_max_time, v_max_segments, v_max_curvature});
    }
    else
    {
        v_max_value = v_max.value();
    }

    // Ensure v_max is valid
    if (v_max_value <= 0)
    {
        throw std::runtime_error("Failed to calculate a valid maximum velocity");
    }

    // Initialize velocities array with v0 as the first element
    std::vector<double> velocities = {v0};

    // Calculate intermediate velocities (v1 to v_(n-1))
    for (size_t k = 0; k < h_values.size() - 1; ++k)
    {
        if (std::signbit(h_values[k]) != std::signbit(h_values[k + 1]))
        {
            velocities.push_back(0.0);
        }
        else
        {
            velocities.push_back(std::signbit(h_values[k]) ? -v_max_value : v_max_value);
        }
    }

    // Add the final velocity vn
    velocities.push_back(vn);

    return velocities;
}

std::tuple<std::function<std::tuple<double, double, double>(double)>, double>
TrapezoidalTrajectory::interpolateWaypoints(const InterpolationParams &params)
{
    // Ensure input is valid
    if (params.points.size() < MIN_POINTS)
    {
        throw std::runtime_error("At least two points are required for interpolation");
    }

    // Calculate intermediate velocities if not provided
    std::vector<double> velocities;
    if (!params.inter_velocities)
    {
        velocities = calculateHeuristicVelocities(params.points,
                                                  params.v0,
                                                  params.vn,
                                                  params.vmax,
                                                  params.amax);
    }
    else if (params.inter_velocities->size() != params.points.size() - 2)
    {
        throw std::runtime_error("Expected " + std::to_string(params.points.size() - 2) +
                                 " intermediate velocities, "
                                 "got " +
                                 std::to_string(params.inter_velocities->size()));
    }
    else
    {
        // Use provided velocities
        velocities.push_back(params.v0);
        velocities.insert(velocities.end(),
                          params.inter_velocities->begin(),
                          params.inter_velocities->end());
        velocities.push_back(params.vn);
    }

    // If vmax was computed in the heuristic, use it for the trajectories
    std::optional<double> vmax_val = params.vmax;
    if (!vmax_val && !params.inter_velocities)
    {
        // Extract the computed vmax from the heuristic (maximum absolute velocity)
        double computed_vmax = 0.0;
        for (const auto &v : velocities)
        {
            computed_vmax = std::max(computed_vmax, std::abs(v));
        }
        vmax_val = computed_vmax;
    }

    // Initialize containers for combined trajectory
    std::vector<std::function<std::tuple<double, double, double>(double)>> traj_functions;
    double cumulative_time = 0.0;
    std::vector<double> seg_end_times = {0.0}; // Start with initial time

    // Generate individual segment trajectories
    for (size_t i = 0; i < params.points.size() - 1; ++i)
    {
        double q0_seg = params.points[i];
        double q1_seg = params.points[i + 1];
        double v_start = velocities[i];
        double v_end = velocities[i + 1];

        std::function<std::tuple<double, double, double>(double)> traj_func;
        double segment_duration = 0.0;

        if (!params.times)
        {
            // Calculate trajectory with velocity/acceleration constraints
            TrajectoryParams traj_params;
            traj_params.q0 = q0_seg;
            traj_params.q1 = q1_seg;
            traj_params.t0 = cumulative_time;
            traj_params.v0 = v_start;
            traj_params.v1 = v_end;
            traj_params.amax = params.amax;
            traj_params.vmax = vmax_val;

            std::tie(traj_func, segment_duration) = generateTrajectory(traj_params);
        }
        else
        {
            // Use specified time for this segment
            segment_duration = (*params.times)[i + 1] - (*params.times)[i];
            TrajectoryParams traj_params;
            traj_params.q0 = q0_seg;
            traj_params.q1 = q1_seg;
            traj_params.t0 = cumulative_time;
            traj_params.v0 = v_start;
            traj_params.v1 = v_end;
            traj_params.amax = params.amax;
            traj_params.duration = segment_duration;

            std::tie(traj_func, std::ignore) = generateTrajectory(traj_params);
        }

        cumulative_time += segment_duration;
        seg_end_times.push_back(cumulative_time);
        traj_functions.push_back(traj_func);
    }

    // Total duration of the trajectory
    double total_duration_val = cumulative_time;
    double final_pos = params.points.back();

    // Create a closure to hold all the necessary data
    auto trajectory_data = std::make_shared<
        std::tuple<std::vector<std::function<std::tuple<double, double, double>(double)>>,
                   std::vector<double>,
                   double,
                   double>>(std::move(traj_functions),
                            std::move(seg_end_times),
                            final_pos,
                            total_duration_val);

    // Function to evaluate trajectory at any time t
    auto trajectory_function =
        [traj_data = std::move(trajectory_data)](double t) -> std::tuple<double, double, double> {
        const auto &[traj_funcs, segment_times, final_pos_val, duration_val] = *traj_data;

        // Clip time to valid range
        t = std::clamp(t, 0.0, duration_val);

        // Determine which segment this time belongs to
        size_t segment_idx = 0;
        for (size_t i = 1; i < segment_times.size(); ++i)
        {
            if (t < segment_times[i])
            {
                segment_idx = i - 1;
                break;
            }
        }

        if (segment_idx < traj_funcs.size())
        {
            return traj_funcs[segment_idx](t);
        }
        // If beyond the end, return final position with zero velocity and acceleration
        return std::make_tuple(final_pos_val, 0.0, 0.0);
    };

    return std::make_tuple(trajectory_function, total_duration_val);
}

} // namespace MoveG

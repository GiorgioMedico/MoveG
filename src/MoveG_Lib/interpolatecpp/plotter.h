#pragma once

#include "sciplot/sciplot.hpp"
#include <string>
#include <vector>

// /**
//  * @brief Class for plotting motion data using sciplot library
//  *
//  * This class takes vectors of position, velocity, acceleration, and optional jerk data
//  * and creates plots for visualizing motion profiles.
//  */
// class MotionPlotter {
// public:
//     /**
//      * @brief Constructor that takes motion data
//      *
//      * @param time Vector of time points
//      * @param position Vector of position values
//      * @param velocity Vector of velocity values
//      * @param acceleration Vector of acceleration values
//      * @param jerk Optional vector of jerk values
//      * @throws std::invalid_argument if vectors have inconsistent sizes
//      */
//     MotionPlotter(const std::vector<double>& time,
//                  const std::vector<double>& position,
//                  const std::vector<double>& velocity,
//                  const std::vector<double>& acceleration,
//                  const std::vector<double>& jerk = {});

//     /**
//      * @brief Display plots of the motion data
//      *
//      * @param title Main title for the plots
//      * @param position_title Title for position plot (if empty, uses main title + " - Position")
//      * @param velocity_title Title for velocity plot (if empty, uses main title + " - Velocity")
//      * @param acceleration_title Title for acceleration plot (if empty, uses main title + " - Acceleration")
//      * @param jerk_title Title for jerk plot (if empty, uses main title + " - Jerk")
//      */
//     void show(const std::string& title = "Motion Data",
//               const std::string& position_title = "",
//               const std::string& velocity_title = "",
//               const std::string& acceleration_title = "",
//               const std::string& jerk_title = "");

//     /**
//      * @brief Save plots to files
//      *
//      * @param filename_prefix Prefix for the output filenames
//      * @param title Main title for the plots
//      * @param position_title Title for position plot (if empty, uses main title + " - Position")
//      * @param velocity_title Title for velocity plot (if empty, uses main title + " - Velocity")
//      * @param acceleration_title Title for acceleration plot (if empty, uses main title + " - Acceleration")
//      * @param jerk_title Title for jerk plot (if empty, uses main title + " - Jerk")
//      */
//     void save(const std::string& filename_prefix,
//               const std::string& title = "Motion Data",
//               const std::string& position_title = "",
//               const std::string& velocity_title = "",
//               const std::string& acceleration_title = "",
//               const std::string& jerk_title = "");

// private:
//     // Input data
//     std::vector<double> m_time;
//     std::vector<double> m_position;
//     std::vector<double> m_velocity;
//     std::vector<double> m_acceleration;
//     std::vector<double> m_jerk;
//     bool m_has_jerk;
// };

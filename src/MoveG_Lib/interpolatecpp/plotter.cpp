#include "plotter.h"
#include <stdexcept>

// MotionPlotter::MotionPlotter(const std::vector<double>& time,
//                            const std::vector<double>& position,
//                            const std::vector<double>& velocity,
//                            const std::vector<double>& acceleration,
//                            const std::vector<double>& jerk)
//     : m_time(time), m_position(position), m_velocity(velocity), m_acceleration(acceleration), m_jerk(jerk)
// {
//     // Check if jerk data is provided
//     m_has_jerk = !jerk.empty();

//     // Validate that all vectors have the same size
//     if (m_time.size() != m_position.size() ||
//         m_time.size() != m_velocity.size() ||
//         m_time.size() != m_acceleration.size())
//     {
//         throw std::invalid_argument("All data vectors must have the same size");
//     }

//     if (m_has_jerk && m_time.size() != m_jerk.size())
//     {
//         throw std::invalid_argument("Jerk vector must have the same size as other data vectors");
//     }
// }

// void MotionPlotter::show(const std::string& title,
//                         const std::string& position_title,
//                         const std::string& velocity_title,
//                         const std::string& acceleration_title,
//                         const std::string& jerk_title)
// {
//     using namespace sciplot;

//     // Create plots for position, velocity, acceleration, and jerk
//     Plot2D position_plot;
//     position_plot.drawCurve(m_time, m_position).label("Position");
//     position_plot.xlabel("Time");
//     position_plot.ylabel("Position");
//     position_plot.title(position_title.empty() ? (title + " - Position") : position_title);
//     position_plot.legend().atTopRight();

//     Plot2D velocity_plot;
//     velocity_plot.drawCurve(m_time, m_velocity).label("Velocity");
//     velocity_plot.xlabel("Time");
//     velocity_plot.ylabel("Velocity");
//     velocity_plot.title(velocity_title.empty() ? (title + " - Velocity") : velocity_title);
//     velocity_plot.legend().atTopRight();

//     Plot2D acceleration_plot;
//     acceleration_plot.drawCurve(m_time, m_acceleration).label("Acceleration");
//     acceleration_plot.xlabel("Time");
//     acceleration_plot.ylabel("Acceleration");
//     acceleration_plot.title(acceleration_title.empty() ? (title + " - Acceleration") : acceleration_title);
//     acceleration_plot.legend().atTopRight();

//     // Create a figure with the three plots
//     Figure fig({ position_plot, velocity_plot, acceleration_plot });
//     fig.layout(3, 1);  // 3 rows, 1 column
//     fig.show();

//     // Create and show jerk plot if jerk data is provided
//     if (m_has_jerk)
//     {
//         Plot2D jerk_plot;
//         jerk_plot.drawCurve(m_time, m_jerk).label("Jerk");
//         jerk_plot.xlabel("Time");
//         jerk_plot.ylabel("Jerk");
//         jerk_plot.title(jerk_title.empty() ? (title + " - Jerk") : jerk_title);
//         jerk_plot.legend().atTopRight();

//         Figure jerk_fig({ jerk_plot });
//         jerk_fig.show();
//     }
// }

// void MotionPlotter::save(const std::string& filename_prefix,
//                         const std::string& title,
//                         const std::string& position_title,
//                         const std::string& velocity_title,
//                         const std::string& acceleration_title,
//                         const std::string& jerk_title)
// {
//     using namespace sciplot;

//     // Create plots for position, velocity, acceleration, and jerk
//     Plot2D position_plot;
//     position_plot.drawCurve(m_time, m_position).label("Position");
//     position_plot.xlabel("Time");
//     position_plot.ylabel("Position");
//     position_plot.title(position_title.empty() ? (title + " - Position") : position_title);
//     position_plot.legend().atTopRight();

//     Plot2D velocity_plot;
//     velocity_plot.drawCurve(m_time, m_velocity).label("Velocity");
//     velocity_plot.xlabel("Time");
//     velocity_plot.ylabel("Velocity");
//     velocity_plot.title(velocity_title.empty() ? (title + " - Velocity") : velocity_title);
//     velocity_plot.legend().atTopRight();

//     Plot2D acceleration_plot;
//     acceleration_plot.drawCurve(m_time, m_acceleration).label("Acceleration");
//     acceleration_plot.xlabel("Time");
//     acceleration_plot.ylabel("Acceleration");
//     acceleration_plot.title(acceleration_title.empty() ? (title + " - Acceleration") : acceleration_title);
//     acceleration_plot.legend().atTopRight();

//     // Save combined figure
//     Figure fig({ position_plot, velocity_plot, acceleration_plot });
//     fig.layout(3, 1);  // 3 rows, 1 column
//     fig.save(filename_prefix + "_combined.pdf");

//     // Save individual plots
//     Figure pos_fig({ position_plot });
//     Figure vel_fig({ velocity_plot });
//     Figure acc_fig({ acceleration_plot });

//     pos_fig.save(filename_prefix + "_position.pdf");
//     vel_fig.save(filename_prefix + "_velocity.pdf");
//     acc_fig.save(filename_prefix + "_acceleration.pdf");

//     // Create and save jerk plot if jerk data is provided
//     if (m_has_jerk)
//     {
//         Plot2D jerk_plot;
//         jerk_plot.drawCurve(m_time, m_jerk).label("Jerk");
//         jerk_plot.xlabel("Time");
//         jerk_plot.ylabel("Jerk");
//         jerk_plot.title(jerk_title.empty() ? (title + " - Jerk") : jerk_title);
//         jerk_plot.legend().atTopRight();

//         Figure jerk_fig({ jerk_plot });
//         jerk_fig.save(filename_prefix + "_jerk.pdf");

//         // Save combined figure with jerk
//         Figure combined_fig({ position_plot, velocity_plot, acceleration_plot, jerk_plot });
//         combined_fig.layout(2, 2);  // 2 rows, 2 columns
//         combined_fig.save(filename_prefix + "_all.pdf");
//     }
// }

/**
 * @file trajectory.h
 * @brief Class for representing trajectories
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometric_path.h"
#include "motion_law.h"
#include "waypoint.h"

namespace MoveG
{

/**
   * @class Trajectory
   * @brief Class representing a trajectory
   *
   * This class combines waypoints, geometric paths, and motion laws
   * to define complete trajectories for robotic manipulators.
   */
class Trajectory
{
public:
    /**
       * @brief Default constructor
       * @param name Trajectory name
       */
    explicit Trajectory(const std::string &name = "");

    /**
       * @brief Copy constructor
       * @param other Trajectory to copy
       */
    Trajectory(const Trajectory &other);

    /**
       * @brief Move constructor
       * @param other Trajectory to move
       */
    Trajectory(Trajectory &&other) noexcept;

    /**
       * @brief Copy assignment operator
       * @param other Trajectory to copy
       * @return Reference to this Trajectory
       */
    Trajectory &operator=(const Trajectory &other);

    /**
       * @brief Move assignment operator
       * @param other Trajectory to move
       * @return Reference to this Trajectory
       */
    Trajectory &operator=(Trajectory &&other) noexcept;

    /**
       * @brief Virtual destructor
       */
    virtual ~Trajectory() = default;

    /**
       * @brief Add a waypoint to the trajectory
       * @param waypoint Waypoint to add
       */
    void addWaypoint(const Waypoint &waypoint);

    /**
       * @brief Get a specific waypoint
       * @param index Waypoint index
       * @return Reference to the waypoint
       * @throws std::out_of_range If the index is out of range
       */
    Waypoint &getWaypoint(size_t index);

    /**
       * @brief Get a specific waypoint (const)
       * @param index Waypoint index
       * @return Constant reference to the waypoint
       * @throws std::out_of_range If the index is out of range
       */
    const Waypoint &getWaypoint(size_t index) const;

    /**
       * @brief Get the number of waypoints
       * @return Number of waypoints
       */
    size_t getNumWaypoints() const
    {
        return waypoints_.size();
    }

    /**
       * @brief Set the geometric path type
       * @param type Path type
       */
    void setGeometricPathType(GeometricPathType type);

    /**
       * @brief Configure a circular path
       * @param center Circle center
       * @param normal Plane normal
       * @param radius Circle radius
       * @param arcAngle Arc angle
       */
    void configureCircularPath(const Eigen::Vector3d &center,
                               const Eigen::Vector3d &normal,
                               double radius,
                               double arcAngle);

    /**
       * @brief Set the orientation interpolation type
       * @param type Interpolation type
       */
    void setOrientationInterpolationType(OrientationInterpolationType type)
    {
        orientationInterpolationType_ = type;
    }

    /**
       * @brief Get the orientation interpolation type
       * @return Interpolation type
       */
    OrientationInterpolationType getOrientationInterpolationType() const
    {
        return orientationInterpolationType_;
    }

    /**
       * @brief Set the motion law type
       * @param type Motion law type
       */
    void setMotionLawType(MotionLawType type);

    /**
       * @brief Configure the motion law
       * @param duration Duration
       * @param maxVelocity Maximum velocity
       * @param maxAcceleration Maximum acceleration
       * @param maxJerk Maximum jerk
       */
    void configureMotionLaw(double duration,
                            double maxVelocity,
                            double maxAcceleration,
                            double maxJerk);

    /**
       * @brief Generate the trajectory
       * @return true if generation was successful, false otherwise
       */
    bool generateTrajectory();

    /**
       * @brief Evaluate the trajectory at a given time
       * @param time Time in seconds
       * @return Waypoint at the specified position
       */
    Waypoint evaluateAtTime(double time) const;

    /**
       * @brief Set the trajectory duration
       * @param duration Duration in seconds
       */
    void setDuration(double duration)
    {
        duration_ = duration;
    }

    /**
       * @brief Get the trajectory duration
       * @return Duration in seconds
       */
    double getDuration() const
    {
        return duration_;
    }

    /**
       * @brief Set the trajectory name
       * @param name Trajectory name
       */
    void setName(const std::string &name)
    {
        name_ = name;
    }

    /**
       * @brief Get the trajectory name
       * @return Trajectory name
       */
    std::string getName() const
    {
        return name_;
    }

    /**
       * @brief Set the maximum velocity
       * @param maxVel Maximum velocity
       */
    void setMaxVelocity(const Eigen::Vector3d &maxVel)
    {
        maxVelocity_ = maxVel;
    }

    /**
       * @brief Set the maximum acceleration
       * @param maxAcc Maximum acceleration
       */
    void setMaxAcceleration(const Eigen::Vector3d &maxAcc)
    {
        maxAcceleration_ = maxAcc;
    }

    /**
       * @brief Set the maximum jerk
       * @param maxJerk Maximum jerk
       */
    void setMaxJerk(const Eigen::Vector3d &maxJerk)
    {
        maxJerk_ = maxJerk;
    }

    /**
       * @brief Export the trajectory as a series of samples at regular frequency
       * @param samplingFrequency Sampling frequency in Hz
       * @return Vector of sampled waypoints
       */
    std::vector<Waypoint> sampleTrajectory(double samplingFrequency) const;

    /**
       * @brief Concatenate two trajectories
       * @param other Trajectory to append
       * @param blendTime Blending time between trajectories
       * @return Resulting trajectory
       */
    Trajectory concatenate(const Trajectory &other, double blendTime = 0.0) const;

    /**
       * @brief Check if the trajectory respects kinematic constraints
       * @return true if all constraints are respected, false otherwise
       */
    bool checkKinematicConstraints() const;

    /**
       * @brief Stream operator for printing
       * @param os Output stream
       * @param trajectory Trajectory to print
       * @return Reference to the output stream
       */
    friend std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory);

private:
    std::string name_;                ///< Trajectory name
    std::vector<Waypoint> waypoints_; ///< Waypoints
    double duration_ = 0.0;           ///< Duration in seconds

    // Geometric path and motion law
    GeometricPath geometricPath_; ///< Geometric path
    MotionLaw motionLaw_;         ///< Motion law

    // Orientation interpolation method
    OrientationInterpolationType orientationInterpolationType_ =
        OrientationInterpolationType::SLERP;

    // Kinematic constraints
    std::optional<Eigen::Vector3d> maxVelocity_;     ///< Maximum velocity
    std::optional<Eigen::Vector3d> maxAcceleration_; ///< Maximum acceleration
    std::optional<Eigen::Vector3d> maxJerk_;         ///< Maximum jerk

    // Trajectory generation flag
    bool trajectoryGenerated_ = false; ///< Flag indicating if the trajectory has been generated

    // Trajectory segments
    struct TrajectorySegment
    {
        size_t startWaypointIndex; ///< Start waypoint index
        size_t endWaypointIndex;   ///< End waypoint index
        double startTime;          ///< Start time
        double endTime;            ///< End time
        GeometricPath path;        ///< Geometric path
        MotionLaw motion;          ///< Motion law
    };

    std::vector<TrajectorySegment> segments_; ///< Trajectory segments

    // Helper functions for trajectory evaluation
    size_t findSegmentIndex(double time) const;

    // Orientation interpolation functions
    Eigen::Quaterniond interpolateSLERP(const Eigen::Quaterniond &q1,
                                        const Eigen::Quaterniond &q2,
                                        double t) const;

    Eigen::Quaterniond interpolateSQUAD(const Eigen::Quaterniond &q1,
                                        const Eigen::Quaterniond &q2,
                                        const Eigen::Quaterniond &a,
                                        const Eigen::Quaterniond &b,
                                        double t) const;
};

} // namespace MoveG

/**
 * @file waypoint.h
 * @brief Class for representing waypoints in a trajectory
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "pose_lib.h"
#include "rotation_lib.h"

namespace MoveG
{

/**
   * @brief Enumeration of orientation interpolation methods
   */
enum class OrientationInterpolationType
{
    SLERP, ///< Spherical Linear Interpolation
    SQUAD  ///< Spherical Cubic Interpolation
};

/**
   * @class Waypoint
   * @brief Class representing a waypoint in a trajectory
   *
   * The Waypoint class contains information about position, orientation,
   * and derivatives (velocity, acceleration, jerk) for a point in the trajectory.
   */
class Waypoint
{
public:
    /**
       * @brief Default constructor
       */
    Waypoint();

    /**
       * @brief Constructor with position and orientation
       * @param position 3D position of the waypoint
       * @param orientation Orientation as quaternion
       */
    Waypoint(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

    /**
       * @brief Constructor with Pose
       * @param pose Position and orientation of the waypoint
       */
    explicit Waypoint(const Pose &pose);

    /**
       * @brief Copy constructor
       * @param other Waypoint to copy
       */
    Waypoint(const Waypoint &other);

    /**
       * @brief Move constructor
       * @param other Waypoint to move
       */
    Waypoint(Waypoint &&other) noexcept;

    /**
       * @brief Copy assignment operator
       * @param other Waypoint to copy
       * @return Reference to this Waypoint
       */
    Waypoint &operator=(const Waypoint &other);

    /**
       * @brief Move assignment operator
       * @param other Waypoint to move
       * @return Reference to this Waypoint
       */
    Waypoint &operator=(Waypoint &&other) noexcept;

    /**
       * @brief Virtual destructor
       */
    virtual ~Waypoint() = default;

    // Getters for position and orientation
    Eigen::Vector3d getPosition() const
    {
        return position_;
    }
    Eigen::Quaterniond getOrientation() const
    {
        return orientation_;
    }
    Pose getPose() const
    {
        return Pose(position_, orientation_);
    }

    // Getters for velocity
    std::optional<Eigen::Vector3d> getLinearVelocity() const
    {
        return linearVelocity_;
    }
    std::optional<Eigen::Vector3d> getAngularVelocity() const
    {
        return angularVelocity_;
    }

    // Getters for acceleration
    std::optional<Eigen::Vector3d> getLinearAcceleration() const
    {
        return linearAcceleration_;
    }
    std::optional<Eigen::Vector3d> getAngularAcceleration() const
    {
        return angularAcceleration_;
    }

    // Getters for jerk
    std::optional<Eigen::Vector3d> getLinearJerk() const
    {
        return linearJerk_;
    }
    std::optional<Eigen::Vector3d> getAngularJerk() const
    {
        return angularJerk_;
    }

    // Getters for blending parameters
    double getBlendRadius() const
    {
        return blendRadius_;
    }
    double getBlendTolerance() const
    {
        return blendTolerance_;
    }

    // Getter for timestamp
    std::optional<double> getTimeStamp() const
    {
        return timeStamp_;
    }

    // Setters for position and orientation
    void setPosition(const Eigen::Vector3d &position)
    {
        position_ = position;
    }
    void setOrientation(const Eigen::Quaterniond &orientation)
    {
        orientation_ = orientation.normalized();
    }
    void setPose(const Pose &pose);

    // Setters for velocity
    void setLinearVelocity(const Eigen::Vector3d &velocity)
    {
        linearVelocity_ = velocity;
    }
    void setAngularVelocity(const Eigen::Vector3d &velocity)
    {
        angularVelocity_ = velocity;
    }

    // Setters for acceleration
    void setLinearAcceleration(const Eigen::Vector3d &acceleration)
    {
        linearAcceleration_ = acceleration;
    }
    void setAngularAcceleration(const Eigen::Vector3d &acceleration)
    {
        angularAcceleration_ = acceleration;
    }

    // Setters for jerk
    void setLinearJerk(const Eigen::Vector3d &jerk)
    {
        linearJerk_ = jerk;
    }
    void setAngularJerk(const Eigen::Vector3d &jerk)
    {
        angularJerk_ = jerk;
    }

    // Setters for blending parameters
    void setBlendRadius(double radius)
    {
        blendRadius_ = radius;
    }
    void setBlendTolerance(double tolerance)
    {
        blendTolerance_ = tolerance;
    }

    // Setter for timestamp
    void setTimeStamp(double timeStamp)
    {
        timeStamp_ = timeStamp;
    }

    /**
       * @brief Stream operator for printing
       * @param os Output stream
       * @param waypoint Waypoint to print
       * @return Reference to the output stream
       */
    friend std::ostream &operator<<(std::ostream &os, const Waypoint &waypoint);

    /**
       * @brief Linear interpolation between two waypoints
       * @param start Initial waypoint
       * @param end Final waypoint
       * @param t Interpolation parameter (0.0 - 1.0)
       * @param orientationType Orientation interpolation type
       * @return Interpolated waypoint
       */
    static Waypoint interpolate(
        const Waypoint &start,
        const Waypoint &end,
        double t,
        OrientationInterpolationType orientationType = OrientationInterpolationType::SLERP);

private:
    Eigen::Vector3d position_;       ///< 3D position of the waypoint
    Eigen::Quaterniond orientation_; ///< Orientation as quaternion

    std::optional<Eigen::Vector3d> linearVelocity_;  ///< Linear velocity (optional)
    std::optional<Eigen::Vector3d> angularVelocity_; ///< Angular velocity (optional)

    std::optional<Eigen::Vector3d> linearAcceleration_;  ///< Linear acceleration (optional)
    std::optional<Eigen::Vector3d> angularAcceleration_; ///< Angular acceleration (optional)

    std::optional<Eigen::Vector3d> linearJerk_;  ///< Linear jerk (optional)
    std::optional<Eigen::Vector3d> angularJerk_; ///< Angular jerk (optional)

    double blendRadius_ = 0.0;      ///< Blending radius
    double blendTolerance_ = 0.001; ///< Blending tolerance

    std::optional<double> timeStamp_; ///< Timestamp (optional)
};

} // namespace MoveG

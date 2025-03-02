/**
 * @file waypoint.cpp
 * @brief Implementation of the Waypoint class for trajectory representation
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#include "waypoint.h"

namespace MoveG
{

// Default Constructor
Waypoint::Waypoint()
    : position_(Eigen::Vector3d::Zero()), orientation_(Eigen::Quaterniond::Identity())
{
}

// Constructor with position and quaternion orientation
Waypoint::Waypoint(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
    : position_(position), orientation_(orientation.normalized())
{
}

// Constructor with Pose
Waypoint::Waypoint(const Pose &pose)
    : position_(pose.getPosition()), orientation_(pose.getQuaternion())
{
}

// Copy Constructor
Waypoint::Waypoint(const Waypoint &other)
    : position_(other.position_), orientation_(other.orientation_),
      linearVelocity_(other.linearVelocity_), angularVelocity_(other.angularVelocity_),
      linearAcceleration_(other.linearAcceleration_),
      angularAcceleration_(other.angularAcceleration_), linearJerk_(other.linearJerk_),
      angularJerk_(other.angularJerk_), blendRadius_(other.blendRadius_),
      blendTolerance_(other.blendTolerance_), timeStamp_(other.timeStamp_)
{
}

// Move Constructor
Waypoint::Waypoint(Waypoint &&other) noexcept
    : position_(std::move(other.position_)), orientation_(std::move(other.orientation_)),
      linearVelocity_(std::move(other.linearVelocity_)),
      angularVelocity_(std::move(other.angularVelocity_)),
      linearAcceleration_(std::move(other.linearAcceleration_)),
      angularAcceleration_(std::move(other.angularAcceleration_)),
      linearJerk_(std::move(other.linearJerk_)), angularJerk_(std::move(other.angularJerk_)),
      blendRadius_(std::move(other.blendRadius_)),
      blendTolerance_(std::move(other.blendTolerance_)), timeStamp_(std::move(other.timeStamp_))
{
}

// Copy Assignment Operator
Waypoint &Waypoint::operator=(const Waypoint &other)
{
    if (this != &other)
    {
        position_ = other.position_;
        orientation_ = other.orientation_;
        linearVelocity_ = other.linearVelocity_;
        angularVelocity_ = other.angularVelocity_;
        linearAcceleration_ = other.linearAcceleration_;
        angularAcceleration_ = other.angularAcceleration_;
        linearJerk_ = other.linearJerk_;
        angularJerk_ = other.angularJerk_;
        blendRadius_ = other.blendRadius_;
        blendTolerance_ = other.blendTolerance_;
        timeStamp_ = other.timeStamp_;
    }
    return *this;
}

// Move Assignment Operator
Waypoint &Waypoint::operator=(Waypoint &&other) noexcept
{
    if (this != &other)
    {
        position_ = std::move(other.position_);
        orientation_ = std::move(other.orientation_);
        linearVelocity_ = std::move(other.linearVelocity_);
        angularVelocity_ = std::move(other.angularVelocity_);
        linearAcceleration_ = std::move(other.linearAcceleration_);
        angularAcceleration_ = std::move(other.angularAcceleration_);
        linearJerk_ = std::move(other.linearJerk_);
        angularJerk_ = std::move(other.angularJerk_);
        blendRadius_ = std::move(other.blendRadius_);
        blendTolerance_ = std::move(other.blendTolerance_);
        timeStamp_ = std::move(other.timeStamp_);
    }
    return *this;
}

// Set Pose
void Waypoint::setPose(const Pose &pose)
{
    position_ = pose.getPosition();
    orientation_ = pose.getQuaternion();
}

// Stream operator implementation
std::ostream &operator<<(std::ostream &os, const Waypoint &waypoint)
{
    os << "Waypoint:" << std::endl;
    os << "  Position: [" << waypoint.position_.x() << ", " << waypoint.position_.y() << ", "
       << waypoint.position_.z() << "]" << std::endl;
    os << "  Orientation (w,x,y,z): [" << waypoint.orientation_.w() << ", "
       << waypoint.orientation_.x() << ", " << waypoint.orientation_.y() << ", "
       << waypoint.orientation_.z() << "]" << std::endl;

    if (waypoint.linearVelocity_)
    {
        os << "  Linear Velocity: [" << waypoint.linearVelocity_->x() << ", "
           << waypoint.linearVelocity_->y() << ", " << waypoint.linearVelocity_->z() << "]"
           << std::endl;
    }

    if (waypoint.angularVelocity_)
    {
        os << "  Angular Velocity: [" << waypoint.angularVelocity_->x() << ", "
           << waypoint.angularVelocity_->y() << ", " << waypoint.angularVelocity_->z() << "]"
           << std::endl;
    }

    if (waypoint.timeStamp_)
    {
        os << "  Time: " << *waypoint.timeStamp_ << " s" << std::endl;
    }

    os << "  Blend Radius: " << waypoint.blendRadius_ << std::endl;

    return os;
}

// Interpolation between waypoints
Waypoint Waypoint::interpolate(const Waypoint &start,
                               const Waypoint &end,
                               double t,
                               OrientationInterpolationType orientationType)
{
    // Ensure t is in [0, 1]
    t = std::max(0.0, std::min(1.0, t));

    // Linearly interpolate position
    Eigen::Vector3d position = start.position_ + t * (end.position_ - start.position_);

    // Interpolate orientation
    Eigen::Quaterniond orientation;

    if (orientationType == OrientationInterpolationType::SLERP)
    {
        orientation = start.orientation_.slerp(t, end.orientation_);
    }
    else // SQUAD
    {
        // For SQUAD, we use a simplified approach assuming it's a double SLERP
        // (in a complete implementation, we would have a true SQUAD with tangent control)
        double t1 = 2.0 * t * (1.0 - t);
        double t2 = t * t;

        Eigen::Quaterniond q1 = start.orientation_.slerp(t, end.orientation_);
        Eigen::Quaterniond q2 = start.orientation_.slerp(t + 0.01, end.orientation_).normalized();
        Eigen::Quaterniond q3 = start.orientation_.slerp(t - 0.01, end.orientation_).normalized();

        // Calculate angular acceleration (approximation)
        Eigen::Quaterniond acc = q3.inverse() * q1 * q1.inverse() * q2;

        // Apply acceleration to get a smoother interpolation
        orientation = q1 * Eigen::Quaterniond::Identity().slerp(t1, acc).normalized();
    }

    Waypoint result(position, orientation);

    // Interpolate velocity, acceleration and jerk if present in both waypoints
    if (start.linearVelocity_ && end.linearVelocity_)
    {
        result.linearVelocity_ =
            *start.linearVelocity_ + t * (*end.linearVelocity_ - *start.linearVelocity_);
    }

    if (start.angularVelocity_ && end.angularVelocity_)
    {
        result.angularVelocity_ =
            *start.angularVelocity_ + t * (*end.angularVelocity_ - *start.angularVelocity_);
    }

    if (start.linearAcceleration_ && end.linearAcceleration_)
    {
        result.linearAcceleration_ = *start.linearAcceleration_ +
                                     t * (*end.linearAcceleration_ - *start.linearAcceleration_);
    }

    if (start.angularAcceleration_ && end.angularAcceleration_)
    {
        result.angularAcceleration_ = *start.angularAcceleration_ +
                                      t * (*end.angularAcceleration_ - *start.angularAcceleration_);
    }

    if (start.linearJerk_ && end.linearJerk_)
    {
        result.linearJerk_ = *start.linearJerk_ + t * (*end.linearJerk_ - *start.linearJerk_);
    }

    if (start.angularJerk_ && end.angularJerk_)
    {
        result.angularJerk_ = *start.angularJerk_ + t * (*end.angularJerk_ - *start.angularJerk_);
    }

    // Interpolate timestamp if present
    if (start.timeStamp_ && end.timeStamp_)
    {
        result.timeStamp_ = *start.timeStamp_ + t * (*end.timeStamp_ - *start.timeStamp_);
    }

    // Interpolate blending parameters
    result.blendRadius_ = start.blendRadius_ + t * (end.blendRadius_ - start.blendRadius_);
    result.blendTolerance_ =
        start.blendTolerance_ + t * (end.blendTolerance_ - start.blendTolerance_);

    return result;
}

} // namespace MoveG

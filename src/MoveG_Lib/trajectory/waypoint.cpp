/**
 * @file waypoint.cpp
 * @brief Implementation of the Waypoint class for trajectory representation
 *
 * @author Giorgio Medico
 * @date 02/03/2025
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
      linearVelocity_(other.linearVelocity_), linearAcceleration_(other.linearAcceleration_),
      linearJerk_(other.linearJerk_), blendRadius_(other.blendRadius_),
      blendTolerance_(other.blendTolerance_), timeStamp_(other.timeStamp_)
#if ENABLE_ANGULAR_FEATURES
      ,
      angularVelocity_(other.angularVelocity_), angularAcceleration_(other.angularAcceleration_),
      angularJerk_(other.angularJerk_)
#endif
{
}

// Move Constructor
Waypoint::Waypoint(Waypoint &&other) noexcept
    : position_(std::move(other.position_)), orientation_(std::move(other.orientation_)),
      linearVelocity_(std::move(other.linearVelocity_)),
      linearAcceleration_(std::move(other.linearAcceleration_)),
      linearJerk_(std::move(other.linearJerk_)), blendRadius_(std::move(other.blendRadius_)),
      blendTolerance_(std::move(other.blendTolerance_)), timeStamp_(std::move(other.timeStamp_))
#if ENABLE_ANGULAR_FEATURES
      ,
      angularVelocity_(std::move(other.angularVelocity_)),
      angularAcceleration_(std::move(other.angularAcceleration_)),
      angularJerk_(std::move(other.angularJerk_))
#endif
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
        linearAcceleration_ = other.linearAcceleration_;
        linearJerk_ = other.linearJerk_;
        blendRadius_ = other.blendRadius_;
        blendTolerance_ = other.blendTolerance_;
        timeStamp_ = other.timeStamp_;

#if ENABLE_ANGULAR_FEATURES
        angularVelocity_ = other.angularVelocity_;
        angularAcceleration_ = other.angularAcceleration_;
        angularJerk_ = other.angularJerk_;
#endif
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
        linearAcceleration_ = std::move(other.linearAcceleration_);
        linearJerk_ = std::move(other.linearJerk_);
        blendRadius_ = std::move(other.blendRadius_);
        blendTolerance_ = std::move(other.blendTolerance_);
        timeStamp_ = std::move(other.timeStamp_);

#if ENABLE_ANGULAR_FEATURES
        angularVelocity_ = std::move(other.angularVelocity_);
        angularAcceleration_ = std::move(other.angularAcceleration_);
        angularJerk_ = std::move(other.angularJerk_);
#endif
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
    os << "  Orientation (x,y,z,w): [" << waypoint.orientation_.x() << ", "
       << waypoint.orientation_.y() << ", " << waypoint.orientation_.z() << ", "
       << waypoint.orientation_.w() << "]" << std::endl;

    if (waypoint.linearVelocity_)
    {
        os << "  Linear Velocity: [" << waypoint.linearVelocity_->x() << ", "
           << waypoint.linearVelocity_->y() << ", " << waypoint.linearVelocity_->z() << "]"
           << std::endl;
    }

#if ENABLE_ANGULAR_FEATURES
    if (waypoint.angularVelocity_)
    {
        os << "  Angular Velocity: [" << waypoint.angularVelocity_->x() << ", "
           << waypoint.angularVelocity_->y() << ", " << waypoint.angularVelocity_->z() << "]"
           << std::endl;
    }
#endif

    if (waypoint.timeStamp_)
    {
        os << "  Time: " << *waypoint.timeStamp_ << " s" << std::endl;
    }

    os << "  Blend Radius: " << waypoint.blendRadius_ << std::endl;

    return os;
}

} // namespace MoveG

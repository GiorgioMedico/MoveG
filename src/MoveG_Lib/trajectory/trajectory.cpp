/**
 * @file trajectory.cpp
 * @brief Implementation of the Trajectory class
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#include "trajectory.h"

namespace MoveG
{

// Default Constructor
Trajectory::Trajectory(const std::string &name)
    : name_(name), geometricPath_(GeometricPathType::LINEAR),
      motionLaw_(MotionLawType::CUBIC_POLYNOMIAL)
{
}

// Copy Constructor
Trajectory::Trajectory(const Trajectory &other)
    : name_(other.name_), waypoints_(other.waypoints_), duration_(other.duration_),
      geometricPath_(other.geometricPath_), motionLaw_(other.motionLaw_),
      orientationInterpolationType_(other.orientationInterpolationType_),
      maxVelocity_(other.maxVelocity_), maxAcceleration_(other.maxAcceleration_),
      maxJerk_(other.maxJerk_), trajectoryGenerated_(other.trajectoryGenerated_),
      segments_(other.segments_)
{
}

// Move Constructor
Trajectory::Trajectory(Trajectory &&other) noexcept
    : name_(std::move(other.name_)), waypoints_(std::move(other.waypoints_)),
      duration_(std::move(other.duration_)), geometricPath_(std::move(other.geometricPath_)),
      motionLaw_(std::move(other.motionLaw_)),
      orientationInterpolationType_(std::move(other.orientationInterpolationType_)),
      maxVelocity_(std::move(other.maxVelocity_)),
      maxAcceleration_(std::move(other.maxAcceleration_)), maxJerk_(std::move(other.maxJerk_)),
      trajectoryGenerated_(std::move(other.trajectoryGenerated_)),
      segments_(std::move(other.segments_))
{
}

// Copy Assignment Operator
Trajectory &Trajectory::operator=(const Trajectory &other)
{
    if (this != &other)
    {
        name_ = other.name_;
        waypoints_ = other.waypoints_;
        duration_ = other.duration_;
        geometricPath_ = other.geometricPath_;
        motionLaw_ = other.motionLaw_;
        orientationInterpolationType_ = other.orientationInterpolationType_;
        maxVelocity_ = other.maxVelocity_;
        maxAcceleration_ = other.maxAcceleration_;
        maxJerk_ = other.maxJerk_;
        trajectoryGenerated_ = other.trajectoryGenerated_;
        segments_ = other.segments_;
    }
    return *this;
}

// Move Assignment Operator
Trajectory &Trajectory::operator=(Trajectory &&other) noexcept
{
    if (this != &other)
    {
        name_ = std::move(other.name_);
        waypoints_ = std::move(other.waypoints_);
        duration_ = std::move(other.duration_);
        geometricPath_ = std::move(other.geometricPath_);
        motionLaw_ = std::move(other.motionLaw_);
        orientationInterpolationType_ = std::move(other.orientationInterpolationType_);
        maxVelocity_ = std::move(other.maxVelocity_);
        maxAcceleration_ = std::move(other.maxAcceleration_);
        maxJerk_ = std::move(other.maxJerk_);
        trajectoryGenerated_ = std::move(other.trajectoryGenerated_);
        segments_ = std::move(other.segments_);
    }
    return *this;
}

// Add waypoint
void Trajectory::addWaypoint(const Waypoint &waypoint)
{
    waypoints_.push_back(waypoint);
    trajectoryGenerated_ = false; // Invalidate the generated trajectory
}

// Get waypoint
Waypoint &Trajectory::getWaypoint(size_t index)
{
    if (index >= waypoints_.size())
    {
        throw std::out_of_range("Waypoint index out of range");
    }
    return waypoints_[index];
}

// Get waypoint (const)
const Waypoint &Trajectory::getWaypoint(size_t index) const
{
    if (index >= waypoints_.size())
    {
        throw std::out_of_range("Waypoint index out of range");
    }
    return waypoints_[index];
}

// Set geometric path type
void Trajectory::setGeometricPathType(GeometricPathType type)
{
    geometricPath_.setPathType(type);
    trajectoryGenerated_ = false; // Invalidate the generated trajectory
}

// Configure circular path
void Trajectory::configureCircularPath(const Eigen::Vector3d &center,
                                       const Eigen::Vector3d &normal,
                                       double radius,
                                       double arcAngle)
{
    geometricPath_.setPathType(GeometricPathType::CIRCULAR);
    geometricPath_.setCircleCenter(center);
    geometricPath_.setCircleNormal(normal);
    geometricPath_.setCircleRadius(radius);
    geometricPath_.setArcAngle(arcAngle);
    trajectoryGenerated_ = false; // Invalidate the generated trajectory
}

// Set motion law type
void Trajectory::setMotionLawType(MotionLawType type)
{
    motionLaw_.setMotionType(type);
    trajectoryGenerated_ = false; // Invalidate the generated trajectory
}

// Configure motion law
void Trajectory::configureMotionLaw(double duration,
                                    double maxVelocity,
                                    double maxAcceleration,
                                    double maxJerk)
{
    motionLaw_.setDuration(duration);
    motionLaw_.setMaxVelocity(maxVelocity);
    motionLaw_.setMaxAcceleration(maxAcceleration);
    motionLaw_.setMaxJerk(maxJerk);
    duration_ = duration;
    trajectoryGenerated_ = false; // Invalidate the generated trajectory
}

// Generate trajectory
bool Trajectory::generateTrajectory()
{
    if (waypoints_.size() < 2)
    {
        std::cerr << "Error: At least 2 waypoints are required to generate a trajectory"
                  << std::endl;
        return false;
    }

    // Calculate duration if not already set
    if (duration_ <= 0.0)
    {
        // Assign duration based on distance and velocities
        double totalLength = 0.0;
        for (size_t i = 1; i < waypoints_.size(); ++i)
        {
            totalLength += (waypoints_[i].getPosition() - waypoints_[i - 1].getPosition()).norm();
        }

        // Estimate duration (assuming average velocity of 1.0 m/s)
        duration_ = totalLength;
        motionLaw_.setDuration(duration_);
    }

    // Calculate timestamps for each waypoint if not already set
    double currentTime = 0.0;

    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        if (!waypoints_[i].getTimeStamp())
        {
            if (i == 0)
            {
                // First waypoint has timestamp 0
                waypoints_[i].setTimeStamp(0.0);
            }
            else if (i == waypoints_.size() - 1)
            {
                // Last waypoint has timestamp equal to total duration
                waypoints_[i].setTimeStamp(duration_);
            }
            else
            {
                // Intermediate waypoints have evenly distributed timestamps
                waypoints_[i].setTimeStamp(duration_ * static_cast<double>(i) /
                                           (waypoints_.size() - 1));
            }
        }
    }

    // Create segments between each pair of waypoints
    segments_.clear();
    for (size_t i = 0; i < waypoints_.size() - 1; ++i)
    {
        TrajectorySegment segment;
        segment.startWaypointIndex = i;
        segment.endWaypointIndex = i + 1;
        segment.startTime = *waypoints_[i].getTimeStamp();
        segment.endTime = *waypoints_[i + 1].getTimeStamp();

        // Create geometric path for the segment
        Eigen::Vector3d startPos = waypoints_[i].getPosition();
        Eigen::Vector3d endPos = waypoints_[i + 1].getPosition();

        if (geometricPath_.getPathType() == GeometricPathType::LINEAR)
        {
            segment.path = GeometricPath::createLinePath(startPos, endPos);
        }
        else if (geometricPath_.getPathType() == GeometricPathType::CIRCULAR)
        {
            // For a circular path, use the global configuration
            segment.path = geometricPath_;
        }
        else if (geometricPath_.getPathType() == GeometricPathType::SPLINE_CUBIC)
        {
            // For a cubic spline, use waypoints as control points
            std::vector<Eigen::Vector3d> controlPoints;
            for (size_t j = i; j <= i + 1; ++j)
            {
                controlPoints.push_back(waypoints_[j].getPosition());
            }
            segment.path = GeometricPath::createCubicSplinePath(controlPoints);
        }
        else if (geometricPath_.getPathType() == GeometricPathType::SPLINE_BSPLINE)
        {
            // For a B-spline, use waypoints as control points
            std::vector<Eigen::Vector3d> controlPoints;
            for (size_t j = i; j <= i + 1; ++j)
            {
                controlPoints.push_back(waypoints_[j].getPosition());
            }
            segment.path = GeometricPath::createBSplinePath(controlPoints, 3);
        }
        else
        {
            // Unsupported path type, use linear as fallback
            segment.path = GeometricPath::createLinePath(startPos, endPos);
        }

        // Create motion law for the segment
        double segmentDuration = segment.endTime - segment.startTime;

        if (motionLaw_.getMotionType() == MotionLawType::CUBIC_POLYNOMIAL)
        {
            double initialVel = 0.0;
            double finalVel = 0.0;

            // Use velocities from waypoints if available
            if (waypoints_[i].getLinearVelocity() && waypoints_[i + 1].getLinearVelocity())
            {
                initialVel = waypoints_[i].getLinearVelocity()->norm();
                finalVel = waypoints_[i + 1].getLinearVelocity()->norm();
            }

            segment.motion =
                MotionLaw::createCubicPolynomial(segmentDuration, initialVel, finalVel);
        }
        else if (motionLaw_.getMotionType() == MotionLawType::QUINTIC_POLYNOMIAL)
        {
            double initialVel = 0.0;
            double finalVel = 0.0;
            double initialAcc = 0.0;
            double finalAcc = 0.0;

            // Use velocities and accelerations from waypoints if available
            if (waypoints_[i].getLinearVelocity() && waypoints_[i + 1].getLinearVelocity())
            {
                initialVel = waypoints_[i].getLinearVelocity()->norm();
                finalVel = waypoints_[i + 1].getLinearVelocity()->norm();
            }

            if (waypoints_[i].getLinearAcceleration() && waypoints_[i + 1].getLinearAcceleration())
            {
                initialAcc = waypoints_[i].getLinearAcceleration()->norm();
                finalAcc = waypoints_[i + 1].getLinearAcceleration()->norm();
            }

            segment.motion = MotionLaw::createQuinticPolynomial(segmentDuration,
                                                                initialVel,
                                                                finalVel,
                                                                initialAcc,
                                                                finalAcc);
        }
        else if (motionLaw_.getMotionType() == MotionLawType::TRAPEZOIDAL)
        {
            double maxVel = 1.0; // Default value
            double maxAcc = 2.0; // Default value

            if (maxVelocity_)
            {
                maxVel = maxVelocity_->norm();
            }

            if (maxAcceleration_)
            {
                maxAcc = maxAcceleration_->norm();
            }

            segment.motion = MotionLaw::createTrapezoidal(segmentDuration, maxVel, maxAcc);
        }
        else
        {
            // Unsupported motion law type, use cubic as fallback
            segment.motion = MotionLaw::createCubicPolynomial(segmentDuration, 0.0, 0.0);
        }

        segments_.push_back(segment);
    }

    trajectoryGenerated_ = true;
    return true;
}

// Evaluate trajectory at time
Waypoint Trajectory::evaluateAtTime(double time) const
{
    if (!trajectoryGenerated_)
    {
        throw std::runtime_error("Trajectory not generated. Call generateTrajectory() first");
    }

    // Limit time to [0, duration_] range
    time = std::max(0.0, std::min(time, duration_));

    // Find the appropriate segment
    size_t segmentIndex = findSegmentIndex(time);
    const TrajectorySegment &segment = segments_[segmentIndex];

    // Normalize time within the segment
    double normalizedTime = (time - segment.startTime) / (segment.endTime - segment.startTime);

    // Calculate the motion law parameter
    double motionParameter = segment.motion.evaluateParameter(time - segment.startTime);

    // Calculate the position along the path
    Eigen::Vector3d position = segment.path.evaluatePosition(motionParameter);

    // Interpolate orientation
    Eigen::Quaterniond startOrientation = waypoints_[segment.startWaypointIndex].getOrientation();
    Eigen::Quaterniond endOrientation = waypoints_[segment.endWaypointIndex].getOrientation();

    Eigen::Quaterniond orientation;

    if (orientationInterpolationType_ == OrientationInterpolationType::SLERP)
    {
        orientation = interpolateSLERP(startOrientation, endOrientation, motionParameter);
    }
    else // SQUAD
    {
        // For a true SQUAD, we should calculate control quaternions
        // Here we use a double SLERP as an approximation
        orientation = interpolateSLERP(startOrientation, endOrientation, motionParameter);
    }

    // Create the resulting waypoint
    Waypoint result(position, orientation);

    // Calculate velocity, acceleration, and jerk if needed
    double motionVelocity = segment.motion.evaluateFirstDerivative(time - segment.startTime);
    double motionAcceleration = segment.motion.evaluateSecondDerivative(time - segment.startTime);
    double motionJerk = segment.motion.evaluateThirdDerivative(time - segment.startTime);

    // Calculate path derivatives
    Eigen::Vector3d pathVelocity = segment.path.evaluateDerivative(motionParameter, 1);
    Eigen::Vector3d pathAcceleration = segment.path.evaluateDerivative(motionParameter, 2);

    // Set linear velocity and acceleration
    result.setLinearVelocity(pathVelocity * motionVelocity);
    result.setLinearAcceleration(pathVelocity * motionAcceleration +
                                 pathAcceleration * motionVelocity * motionVelocity);
    result.setLinearJerk(Eigen::Vector3d(pathVelocity * motionJerk));

    // Set timestamp
    result.setTimeStamp(time);

    return result;
}

// Sample trajectory
std::vector<Waypoint> Trajectory::sampleTrajectory(double samplingFrequency) const
{
    if (!trajectoryGenerated_)
    {
        throw std::runtime_error("Trajectory not generated. Call generateTrajectory() first");
    }

    // Calculate number of samples
    double dt = 1.0 / samplingFrequency;
    size_t numSamples = static_cast<size_t>(std::ceil(duration_ * samplingFrequency)) + 1;

    // Sample the trajectory
    std::vector<Waypoint> samples;
    samples.reserve(numSamples);

    for (size_t i = 0; i < numSamples; ++i)
    {
        double time = i * dt;
        if (time > duration_)
        {
            time = duration_; // Ensure the last sample is exactly at the end
        }

        samples.push_back(evaluateAtTime(time));
    }

    return samples;
}

// Concatenate trajectories
Trajectory Trajectory::concatenate(const Trajectory &other, double blendTime) const
{
    if (!trajectoryGenerated_ || !other.trajectoryGenerated_)
    {
        throw std::runtime_error("Trajectories must be generated before concatenation");
    }

    Trajectory result(name_ + "_" + other.name_);

    // Copy all waypoints from this trajectory
    for (const auto &wp : waypoints_)
    {
        result.addWaypoint(wp);
    }

    // Calculate the time at which the second trajectory starts
    double startTimeSecond = duration_;

    // Add waypoints from the second trajectory with adjusted timestamps
    for (size_t i = 0; i < other.waypoints_.size(); ++i)
    {
        Waypoint wp = other.waypoints_[i];

        if (wp.getTimeStamp())
        {
            wp.setTimeStamp(*wp.getTimeStamp() + startTimeSecond);
        }

        // Skip the first waypoint of the second trajectory if blending time is zero
        if (i > 0 || blendTime == 0.0)
        {
            result.addWaypoint(wp);
        }
    }

    // Set total duration
    result.setDuration(duration_ + other.duration_);

    // Generate the trajectory
    result.setGeometricPathType(geometricPath_.getPathType());
    result.setMotionLawType(motionLaw_.getMotionType());
    result.setOrientationInterpolationType(orientationInterpolationType_);

    if (maxVelocity_)
    {
        result.setMaxVelocity(*maxVelocity_);
    }

    if (maxAcceleration_)
    {
        result.setMaxAcceleration(*maxAcceleration_);
    }

    if (maxJerk_)
    {
        result.setMaxJerk(*maxJerk_);
    }

    result.generateTrajectory();

    return result;
}

// Check kinematic constraints
bool Trajectory::checkKinematicConstraints() const
{
    if (!trajectoryGenerated_)
    {
        throw std::runtime_error("Trajectory not generated. Call generateTrajectory() first");
    }

    if (!maxVelocity_ && !maxAcceleration_ && !maxJerk_)
    {
        // No constraints to check
        return true;
    }

    // Sample the trajectory to check constraints
    const double dt = 0.01; // 100 Hz
    size_t numSamples = static_cast<size_t>(std::ceil(duration_ / dt)) + 1;

    for (size_t i = 0; i < numSamples; ++i)
    {
        double time = i * dt;
        if (time > duration_)
        {
            time = duration_;
        }

        Waypoint wp = evaluateAtTime(time);

        // Check velocity constraints
        if (maxVelocity_ && wp.getLinearVelocity())
        {
            double vx = std::abs(wp.getLinearVelocity()->x());
            double vy = std::abs(wp.getLinearVelocity()->y());
            double vz = std::abs(wp.getLinearVelocity()->z());

            if (vx > maxVelocity_->x() || vy > maxVelocity_->y() || vz > maxVelocity_->z())
            {
                std::cerr << "Velocity constraint violated at time t = " << time << " s"
                          << std::endl;
                return false;
            }
        }

        // Check acceleration constraints
        if (maxAcceleration_ && wp.getLinearAcceleration())
        {
            double ax = std::abs(wp.getLinearAcceleration()->x());
            double ay = std::abs(wp.getLinearAcceleration()->y());
            double az = std::abs(wp.getLinearAcceleration()->z());

            if (ax > maxAcceleration_->x() || ay > maxAcceleration_->y() ||
                az > maxAcceleration_->z())
            {
                std::cerr << "Acceleration constraint violated at time t = " << time << " s"
                          << std::endl;
                return false;
            }
        }

        // Check jerk constraints
        if (maxJerk_ && wp.getLinearJerk())
        {
            double jx = std::abs(wp.getLinearJerk()->x());
            double jy = std::abs(wp.getLinearJerk()->y());
            double jz = std::abs(wp.getLinearJerk()->z());

            if (jx > maxJerk_->x() || jy > maxJerk_->y() || jz > maxJerk_->z())
            {
                std::cerr << "Jerk constraint violated at time t = " << time << " s" << std::endl;
                return false;
            }
        }
    }

    return true;
}

// Stream operator implementation
std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory)
{
    os << "Trajectory: " << trajectory.name_ << std::endl;
    os << "Duration: " << trajectory.duration_ << " s" << std::endl;
    os << "Number of waypoints: " << trajectory.waypoints_.size() << std::endl;

    for (size_t i = 0; i < trajectory.waypoints_.size(); ++i)
    {
        os << "Waypoint " << i << ":" << std::endl;
        os << trajectory.waypoints_[i] << std::endl;
    }

    return os;
}

// Find segment index
size_t Trajectory::findSegmentIndex(double time) const
{
    // Find the segment containing the specified time
    for (size_t i = 0; i < segments_.size(); ++i)
    {
        if (time >= segments_[i].startTime && time <= segments_[i].endTime)
        {
            return i;
        }
    }

    // If time is beyond duration, return the last segment
    if (time >= duration_ && !segments_.empty())
    {
        return segments_.size() - 1;
    }

    throw std::runtime_error("Could not find a segment for the specified time");
}

// SLERP interpolation
Eigen::Quaterniond Trajectory::interpolateSLERP(const Eigen::Quaterniond &q1,
                                                const Eigen::Quaterniond &q2,
                                                double t) const
{
    // Use Eigen's implementation for SLERP
    return q1.slerp(t, q2);
}

// SQUAD interpolation
Eigen::Quaterniond Trajectory::interpolateSQUAD(const Eigen::Quaterniond &q1,
                                                const Eigen::Quaterniond &q2,
                                                const Eigen::Quaterniond &a,
                                                const Eigen::Quaterniond &b,
                                                double t) const
{
    // Implementation of SQUAD (Spherical Cubic Interpolation)
    Eigen::Quaterniond slerp1 = interpolateSLERP(q1, q2, t);
    Eigen::Quaterniond slerp2 = interpolateSLERP(a, b, t);
    return interpolateSLERP(slerp1, slerp2, 2.0 * t * (1.0 - t));
}

} // namespace MoveG

/**
 * @file pose_lib.cpp
 * @brief Class for representing Poses
 *
 * @author Giorgio Medico
 * @date 28/02/2025
 */

#include "pose_lib.h"

namespace MoveG
{
// Default Constructor
Pose::Pose() noexcept
    : position_(Eigen::Vector3d::Zero()), orientation_(Eigen::Quaterniond::Identity())
{
}

// Constructor with position and quaternion orientation
Pose::Pose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
    : position_(position),
      orientation_(orientation.normalized()) // Ensure the quaternion is normalized
{
}

// Constructor with position and rotation matrix
Pose::Pose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation_matrix)
    : position_(position), orientation_(Eigen::Quaterniond(rotation_matrix))
{
    orientation_.normalize(); // Ensure the quaternion is normalized
}

// Constructor with Affine3d transformation
Pose::Pose(const Eigen::Affine3d &transformation)
    : position_(transformation.translation()),
      orientation_(Eigen::Quaterniond(transformation.rotation()))
{
    orientation_.normalize(); // Ensure the quaternion is normalized
}

// Constructor with position and custom Rotation class
Pose::Pose(const Eigen::Vector3d &position, const Rotation &orientation)
    : position_(position), orientation_(orientation.toQuaternion())
{
    orientation_.normalize(); // Ensure the quaternion is normalized
}

// Constructor with Homogeneous Transformation Matrix
Pose::Pose(const Eigen::Matrix4d &homogeneousT)
{
    validateHomogeneousMatrix(homogeneousT);

    // Extract position
    position_ = homogeneousT.block<3, 1>(0, 3);

    // Extract rotation matrix
    const Eigen::Matrix3d rotation_matrix = homogeneousT.block<3, 3>(0, 0);
    orientation_ = Eigen::Quaterniond(rotation_matrix).normalized(); // Ensure normalization
}

// Destructor
Pose::~Pose()
{
    // No dynamic memory to clean up
}

// Copy Constructor
Pose::Pose(const Pose &other) : position_(other.position_), orientation_(other.orientation_)
{
}

// Move Constructor
Pose::Pose(Pose &&other) noexcept
    : position_(std::move(other.position_)), orientation_(std::move(other.orientation_))
{
}

// Copy Assignment Operator
Pose &Pose::operator=(const Pose &other)
{
    if (this != &other)
    {
        position_ = other.position_;
        orientation_ = other.orientation_;
    }
    return *this;
}

// Move Assignment Operator
Pose &Pose::operator=(Pose &&other) noexcept
{
    if (this != &other)
    {
        position_ = std::move(other.position_);
        orientation_ = std::move(other.orientation_);
    }
    return *this;
}

// Getters

Eigen::Vector3d Pose::getPosition() const
{
    return position_;
}

Eigen::Quaterniond Pose::getQuaternion() const
{
    return orientation_;
}

Eigen::Matrix3d Pose::getRotationMatrix() const
{
    return orientation_.toRotationMatrix();
}

Eigen::Affine3d Pose::getAffineTransformation() const
{
    Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
    transformation.translation() = position_;
    transformation.linear() = orientation_.toRotationMatrix();
    return transformation;
}

Eigen::Matrix4d Pose::getHomogeneousT() const
{
    Eigen::Matrix4d homogeneous = Eigen::Matrix4d::Identity();
    homogeneous.block<3, 3>(0, 0) = orientation_.toRotationMatrix();
    homogeneous.block<3, 1>(0, 3) = position_;
    return homogeneous;
}

// Additional Getters for individual components

double Pose::getX() const
{
    return position_.x();
}

double Pose::getY() const
{
    return position_.y();
}

double Pose::getZ() const
{
    return position_.z();
}

double Pose::getQx() const
{
    return orientation_.x();
}

double Pose::getQy() const
{
    return orientation_.y();
}

double Pose::getQz() const
{
    return orientation_.z();
}

double Pose::getQw() const
{
    return orientation_.w();
}

// Setters

void Pose::setPosition(const Eigen::Vector3d &position)
{
    position_ = position;
}

void Pose::setOrientation(const Eigen::Quaterniond &orientation)
{
    orientation_ = orientation.normalized(); // Ensure the quaternion is normalized
}

void Pose::setRotationMatrix(const Eigen::Matrix3d &rotation_matrix)
{
    orientation_ = Eigen::Quaterniond(rotation_matrix).normalized(); // Ensure normalization
}

void Pose::setAffineTransformation(const Eigen::Affine3d &transformation)
{
    position_ = transformation.translation();
    orientation_ = Eigen::Quaterniond(transformation.rotation()).normalized();
}

void Pose::setHomogeneousT(const Eigen::Matrix4d &homogeneousT)
{
    validateHomogeneousMatrix(homogeneousT);

    // Extract position
    position_ = homogeneousT.block<3, 1>(0, 3);

    // Extract rotation matrix
    const Eigen::Matrix3d rotation_matrix = homogeneousT.block<3, 3>(0, 0);
    orientation_ = Eigen::Quaterniond(rotation_matrix).normalized();
}

// Distance metrics
double Pose::positionDistance(const Pose &other) const
{
    return (position_ - other.position_).norm();
}

double Pose::orientationDistance(const Pose &other) const
{
    // Compute the angle between quaternions
    double dot = std::abs(orientation_.dot(other.orientation_));
    // Clamp dot to [-1, 1] to avoid numerical issues
    dot = std::max(-1.0, std::min(1.0, dot));
    return 2.0 * std::acos(dot);
}

// Compose two poses
Pose Pose::operator*(const Pose &other) const
{
    const Eigen::Quaterniond combined_orientation = orientation_ * other.orientation_;
    const Eigen::Vector3d combined_position = position_ + orientation_ * other.position_;
    return Pose(combined_position, combined_orientation);
}

// Inverse of the pose
Pose Pose::inverse() const
{
    const Eigen::Quaterniond inv_orientation = orientation_.conjugate();
    const Eigen::Vector3d inv_position = -(inv_orientation * position_);
    return Pose(inv_position, inv_orientation);
}

// Validate homogeneous transformation matrix
void Pose::validateHomogeneousMatrix(const Eigen::Matrix4d &homogeneousT)
{
    // Check the last row
    Eigen::RowVector4d expected_last_row(0, 0, 0, 1);
    Eigen::RowVector4d actual_last_row = homogeneousT.block<1, 4>(3, 0);

    if (!actual_last_row.isApprox(expected_last_row, 1e-6))
    {
        std::stringstream error_msg;
        error_msg << "Invalid homogeneous transformation matrix. The last row must be [0, 0, 0, "
                     "1], but got ["
                  << actual_last_row(0) << ", " << actual_last_row(1) << ", " << actual_last_row(2)
                  << ", " << actual_last_row(3) << "].";
        throw std::invalid_argument(error_msg.str());
    }

    // Extract the rotation matrix part
    const Eigen::Matrix3d rotation_part = homogeneousT.block<3, 3>(0, 0);

    // Check if the rotation matrix is orthogonal
    // A rotation matrix multiplied by its transpose should give the identity matrix
    Eigen::Matrix3d orthogonality_check = rotation_part * rotation_part.transpose();
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    if (!orthogonality_check.isApprox(identity, 1e-6))
    {
        std::stringstream error_msg;
        error_msg
            << "Invalid homogeneous transformation matrix. The rotation part is not orthogonal.";
        throw std::invalid_argument(error_msg.str());
    }

    // Additionally, check the determinant is approximately 1 (proper rotation)
    double det = rotation_part.determinant();
    if (std::abs(det - 1.0) > 1e-6)
    {
        std::stringstream error_msg;
        error_msg << "Invalid homogeneous transformation matrix. The rotation part has determinant "
                  << det << ", which differs from 1.0 (expected for a proper rotation).";
        throw std::invalid_argument(error_msg.str());
    }
}

// Overload the output stream operator
std::ostream &operator<<(std::ostream &os, const Pose &pose)
{
    os << "Position: [" << pose.position_.x() << ", " << pose.position_.y() << ", "
       << pose.position_.z() << "]\n";
    os << "Orientation (quaternion): [" << pose.orientation_.x() << ", " << pose.orientation_.y()
       << ", " << pose.orientation_.z() << ", " << pose.orientation_.w() << "]";
    return os;
}

// Implementation of coordinate transformation methods
Pose Pose::transformPose(const Pose &transform) const
{
    // This method transforms this pose by the given transformation
    // This is useful for changing coordinate frames
    return transform * (*this);
}

Eigen::Vector3d Pose::localToGlobal(const Eigen::Vector3d &local_point) const
{
    // Transform a point from the local coordinate frame to the global frame
    // This applies the rotation and then the translation
    return position_ + orientation_ * local_point;
}

Eigen::Vector3d Pose::globalToLocal(const Eigen::Vector3d &global_point) const
{
    // Transform a point from the global coordinate frame to the local frame
    // This applies the inverse transformation (first translate, then rotate)
    return orientation_.conjugate() * (global_point - position_);
}
} // namespace MoveG

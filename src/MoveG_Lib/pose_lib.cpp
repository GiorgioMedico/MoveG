/**
 * @file pose_lib.cpp
 * @brief Classe per la rappresentazione di Pose
 *
 * @author Giorgio Medico
 * @date 4/10/2024
 */

#include "pose_lib.h"

namespace MoveG
{
// Default Constructor
Pose::Pose() : position_(Eigen::Vector3d::Zero()), orientation_(Eigen::Quaterniond::Identity())
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
    : position_(position),
      orientation_(orientation.toQuaternion()) // Assuming Rotation has toQuaternion()
{
    orientation_.normalize(); // Ensure the quaternion is normalized
}

// Constructor with Homogeneous Transformation Matrix
Pose::Pose(const Eigen::Matrix4d &homogeneousT)
{
    // Validate the homogeneous transformation matrix
    if (!homogeneousT.block<1, 4>(3, 0).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-6))
    {
        throw std::invalid_argument(
            "Invalid homogeneous transformation matrix. The last row must be [0, 0, 0, 1].");
    }

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

// Getters

Eigen::Vector3d Pose::getPosition() const
{
    return position_;
}

Eigen::Quaterniond Pose::getQaternion() const
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
    // Reuse the homogeneous matrix constructor for validation and extraction
    *this = Pose(homogeneousT);
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

// Overload the output stream operator
std::ostream &operator<<(std::ostream &os, const Pose &pose)
{
    os << "Position: [" << pose.position_.x() << ", " << pose.position_.y() << ", "
       << pose.position_.z() << "]\n";
    os << "Orientation (quaternion): [" << pose.orientation_.x() << ", " << pose.orientation_.y()
       << ", " << pose.orientation_.z() << ", " << pose.orientation_.w() << "]";
    return os;
}

} // namespace MoveG

/**
 * @file rotation_lib.cpp
 * @brief Class for representing and manipulating 3D rotations
 *
 * @author Giorgio Medico
 * @date 28/02/2025
 */

#include "rotation_lib.h"

namespace MoveG
{

// Implementation of the Rotation class methods

// Default constructor (identity rotation)
Rotation::Rotation() : q(Eigen::Quaterniond::Identity())
{
}

// Copy constructor
Rotation::Rotation(const Rotation &other) : q(other.q)
{
}

// Move constructor
Rotation::Rotation(Rotation &&other) noexcept : q(std::move(other.q))
{
}

// Constructor from rotation matrix
Rotation::Rotation(const Eigen::Matrix3d &rotation_matrix) : q(Eigen::Quaterniond(rotation_matrix))
{
}

// Constructor from quaternion
Rotation::Rotation(const Eigen::Quaterniond &quaternion) : q(quaternion.normalized())
{
}

// Constructor from axis-angle
Rotation::Rotation(const Eigen::AngleAxisd &angle_axis) : q(Eigen::Quaterniond(angle_axis))
{
}

// Constructor from Euler angles using axis-angle representations
Rotation::Rotation(double angle1,
                   double angle2,
                   double angle3,
                   bool intrinsic,
                   const std::string &sequence,
                   bool degree)
    : q(Eigen::Quaterniond::Identity())
{
    checkSequence(sequence);

    if (degree)
    {
        angle1 = deg2rad(angle1);
        angle2 = deg2rad(angle2);
        angle3 = deg2rad(angle3);
    }

    const Eigen::AngleAxisd aa1 = angleAxis(sequence[0], angle1);
    const Eigen::AngleAxisd aa2 = angleAxis(sequence[1], angle2);
    const Eigen::AngleAxisd aa3 = angleAxis(sequence[2], angle3);

    if (intrinsic)
    {
        // For intrinsic rotations, multiply in the order q3 * q2 * q1
        q = Eigen::Quaterniond(aa3) * Eigen::Quaterniond(aa2) * Eigen::Quaterniond(aa1);
    }
    else
    {
        // For extrinsic rotations, multiply in the order q1 * q2 * q3
        q = Eigen::Quaterniond(aa1) * Eigen::Quaterniond(aa2) * Eigen::Quaterniond(aa3);
    }

    q.normalize();
}

// Static methods
Rotation Rotation::fromRotationMatrix(const Eigen::Matrix3d &rotation_matrix)
{
    return Rotation(rotation_matrix);
}

Rotation Rotation::fromQuaternion(const Eigen::Quaterniond &quaternion)
{
    return Rotation(quaternion.normalized());
}

Rotation Rotation::fromAngleAxis(const Eigen::AngleAxisd &angle_axis)
{
    return Rotation(angle_axis);
}

Rotation Rotation::fromEulerAngles(double angle1,
                                   double angle2,
                                   double angle3,
                                   bool intrinsic,
                                   const std::string &sequence,
                                   bool degree)
{
    return Rotation(angle1, angle2, angle3, intrinsic, sequence, degree);
}

// Converter to rotation matrix
Eigen::Matrix3d Rotation::toRotationMatrix() const
{
    return q.toRotationMatrix();
}

// Converter to quaternion
Eigen::Quaterniond Rotation::toQuaternion() const
{
    return q;
}

// Converter to axis-angle
Eigen::AngleAxisd Rotation::toAngleAxis() const
{
    return Eigen::AngleAxisd(q);
}

// Converter to Euler angles
Eigen::Vector3d Rotation::toEulerAngles(bool intrinsic, const std::string &sequence) const
{
    checkSequence(sequence);

    const int a0 = axisToIndex(sequence[0]);
    const int a1 = axisToIndex(sequence[1]);
    const int a2 = axisToIndex(sequence[2]);

    Eigen::Vector3d angles = Eigen::Vector3d::Zero();

    if (intrinsic)
    {
        // For intrinsic rotations, reverse the order of axes
        angles = q.toRotationMatrix().eulerAngles(a2, a1, a0).reverse();
    }
    else
    {
        // For extrinsic rotations, use the axes as they are
        angles = q.toRotationMatrix().eulerAngles(a0, a1, a2);
    }

    return angles;
}

// Overloading the * operator (composition of rotations)
Rotation Rotation::operator*(const Rotation &other) const
{
    return Rotation(q * other.q);
}

Rotation &Rotation::operator=(const Rotation &other)
{
    if (this != &other)
    {
        q = other.q;
    }
    return *this;
}

Rotation &Rotation::operator=(Rotation &&other) noexcept
{
    q = std::move(other.q);
    return *this;
}

// Implementation of the output stream operator
std::ostream &operator<<(std::ostream &os, const Rotation &rotation)
{
    Eigen::Quaterniond quat = rotation.toQuaternion();
    os << "Rotation (quaternion x,y,z,w): [" << quat.x() << ", " << quat.y() << ", " << quat.z()
       << ", " << quat.w() << "]";
    return os;
}

// Angle normalization methods
double Rotation::normalizeAngle(double angle)
{
    // Use fmod to get the remainder of division by 2π
    angle = std::fmod(angle, 2 * M_PI);

    // Adjust to the range [-π, π]
    if (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }

    return angle;
}

// Normalize angle to a specified range [minVal, maxVal]
double Rotation::normalizeAngleRange(double angle, double minVal, double maxVal)
{
    const double width = maxVal - minVal;

    // Validate range width
    if (width <= 0)
    {
        throw std::invalid_argument("Invalid range: maxVal must be greater than minVal");
    }

    // Get a value in range [0, width) using modulo
    const double offsetAngle = std::fmod(angle - minVal, width);

    // Handle negative angles
    if (offsetAngle < 0)
    {
        return offsetAngle + maxVal;
    }
    else
    {
        return offsetAngle + minVal;
    }
}

// Normalize Euler angles
Eigen::Vector3d Rotation::normalizeEulerAngles(const Eigen::Vector3d &angles)
{
    return Eigen::Vector3d(normalizeAngle(angles.x()),
                           normalizeAngle(angles.y()),
                           normalizeAngle(angles.z()));
}

// Auxiliary function to create an axis-angle representation
Eigen::AngleAxisd Rotation::angleAxis(char axis, double angle)
{
    switch (axis)
    {
    case 'X':
    case 'x':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
    case 'Y':
    case 'y':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
    case 'Z':
    case 'z':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    default:
        throw std::invalid_argument("Invalid axis. Valid axes are 'X', 'Y', or 'Z'.");
    }
}

// Auxiliary function to convert axis to index
int Rotation::axisToIndex(char axis)
{
    switch (axis)
    {
    case 'X':
    case 'x':
        return 0;
    case 'Y':
    case 'y':
        return 1;
    case 'Z':
    case 'z':
        return 2;
    default:
        throw std::invalid_argument("Invalid axis. Valid axes are 'X', 'Y', or 'Z'.");
    }
}

// Auxiliary function to check the Euler angle sequence
int Rotation::checkSequence(const std::string &sequence)
{
    if (sequence.length() != 3 || sequence.find_first_not_of("XYZxyz") != std::string::npos)
    {
        throw std::invalid_argument("The axis sequence must be a string of three characters "
                                    "from 'X', 'Y', and 'Z' or 'x', 'y', and 'z'.");
    }
    return 1;
}

// Conversion from degrees to radians
double Rotation::deg2rad(double degree)
{
    return degree * M_PI / 180.0;
}

// Conversion from radians to degrees
double Rotation::rad2deg(double radian)
{
    return radian * 180.0 / M_PI;
}

Eigen::Quaterniond Rotation::quaternion_difference(const Eigen::Quaterniond &a,
                                                   const Eigen::Quaterniond &b)
{
    return a.inverse() * b;
}

Eigen::Quaterniond Rotation::quaternion_negation(const Eigen::Quaterniond &v)
{
    return Eigen::Quaterniond(-v.w(), -v.x(), -v.y(), -v.z());
}

Eigen::Quaterniond Rotation::scalar_product(const Eigen::Quaterniond &v, double t)
{
    return Eigen::Quaterniond(t * v.w(), t * v.x(), t * v.y(), t * v.z());
}

Eigen::Quaterniond Rotation::quaternion_minus(const Eigen::Quaterniond &v1,
                                              const Eigen::Quaterniond &v0)
{
    return Eigen::Quaterniond(v1.w() - v0.w(), v1.x() - v0.x(), v1.y() - v0.y(), v1.z() - v0.z());
}

Eigen::Quaterniond Rotation::quaternion_plus(const Eigen::Quaterniond &v1,
                                             const Eigen::Quaterniond &v0)
{
    return Eigen::Quaterniond(v1.w() + v0.w(), v1.x() + v0.x(), v1.y() + v0.y(), v1.z() + v0.z());
}

// Elementary rotation matrices
Eigen::Matrix3d Rotation::rotationX(double angle)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);
    return rot;
}

Eigen::Matrix3d Rotation::rotationY(double angle)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle);
    return rot;
}

Eigen::Matrix3d Rotation::rotationZ(double angle)
{
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;
    return rot;
}

// Matrix S
Eigen::Matrix3d Rotation::matrixS(const Eigen::Vector3d &omega)
{
    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    S << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(), omega.x(), 0;
    return S;
}

// Matrix R_dot
Eigen::Matrix3d Rotation::matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Matrix3d &S)
{
    return S * R;
}

Eigen::Matrix3d Rotation::matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Vector3d &omega)
{
    const Eigen::Matrix3d S = matrixS(omega);
    return matrixR_dot(R, S);
}

Eigen::Matrix3d Rotation::matrixT(const Eigen::Vector3d &angles, const std::string &sequence)
{
    checkSequence(sequence);

    Eigen::Matrix3d T = Eigen::Matrix3d::Zero();

    std::array<Eigen::Vector3d, 3> theta;

    for (std::size_t i = 0; i < 3; ++i)
    {
        const char c = sequence[i];
        switch (c)
        {
        case 'X':
        case 'x':
            theta[i] = Eigen::Vector3d::UnitX();
            break;
        case 'Y':
        case 'y':
            theta[i] = Eigen::Vector3d::UnitY();
            break;
        case 'Z':
        case 'z':
            theta[i] = Eigen::Vector3d::UnitZ();
            break;
        default:
            break;
        }
    }

    T.col(0) = theta[0];

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    for (std::size_t i = 0; i < 2; ++i)
    {
        Eigen::Matrix3d Ri;

        const char c = sequence[i];
        switch (c)
        {
        case 'X':
        case 'x':
            Ri = Rotation::rotationX(angles[static_cast<Eigen::Index>(i)]);
            break;
        case 'Y':
        case 'y':
            Ri = Rotation::rotationY(angles[static_cast<Eigen::Index>(i)]);
            break;
        case 'Z':
        case 'z':
            Ri = Rotation::rotationZ(angles[static_cast<Eigen::Index>(i)]);
            break;
        default:
            Ri = Eigen::Matrix3d::Identity();
            break;
        }

        R *= Ri;
        T.col(static_cast<Eigen::Index>(i + 1)) = R * theta[i + 1];
    }

    // Check if it is singular and so the determinant is near zero
    if (std::abs(T.determinant()) < 1e-6)
    {
        std::cerr << "WARNING: The matrix is singular\n";
        throw std::runtime_error("Matrix T is singular, which may cause numerical instability.");
    }

    return T;
}

} // namespace MoveG

/**
 * @file rotation_lib.h
 * @brief Class for representing and manipulating 3D rotations
 *
 * @author Giorgio Medico
 * @date 24/09/2024
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Additional comments to clarify the behavior of intrinsic and extrinsic rotations

/**
  * Extrinsic vs intrinsic rotations.
  *
  * - Extrinsic: all rotations refer to a fixed/global coordinate system xyz.
  *   Each rotation applies to the global coordinate system.
  *
  * - Intrinsic: each rotation refers to the previously rotated coordinate system.
  *   For example, an intrinsic Yaw-Pitch'-Roll'' (z-y'-x'') rotation:
  *   1) Rotation around the global z-axis.
  *   2) Rotation around the new y'-axis.
  *   3) Rotation around the new x''-axis.
  *
  * Matrix multiplication occurs in order of application (the matrices are the same for both types):
  * Intrinsic: R = R3 * R2 * R1
  * Extrinsic: R = R1 * R2 * R3
  */

namespace MoveG
{

/**
  * @class Rotation
  * @brief Class for representing and managing rotations in three-dimensional space.
  *
  * This class supports various rotation representations, including rotation matrices,
  * quaternions, axis-angle, and Euler angles. It also allows composition of rotations.
  */
class Rotation
{
public:
    /**
      * @brief Default constructor.
      *
      * Initializes the rotation as identity.
      */
    Rotation();

    /**
      * @brief Copy constructor.
      * @param other Another rotation to copy.
      */
    Rotation(const Rotation &other);

    /**
      * @brief Move constructor for the Rotation class.
      *
      * This constructor allows the creation of a Rotation object by transferring
      * the resources from another Rotation object. It is marked as noexcept to
      * indicate that it does not throw exceptions.
      *
      * @param other The Rotation object to be moved.
      */
    Rotation(Rotation &&other) noexcept;

    /**
      * @brief Constructor from rotation matrix.
      * @param rotation_matrix 3x3 rotation matrix.
      */
    Rotation(const Eigen::Matrix3d &rotation_matrix);

    /**
      * @brief Constructor from quaternion.
      * @param quaternion Quaternion representing the rotation.
      */
    Rotation(const Eigen::Quaterniond &quaternion);

    /**
      * @brief Constructor from axis-angle.
      * @param angle_axis Axis-angle representation of the rotation.
      */
    Rotation(const Eigen::AngleAxisd &angle_axis);

    /**
      * @brief Constructor from Euler angles.
      * @param angle1 First Euler angle.
      * @param angle2 Second Euler angle.
      * @param angle3 Third Euler angle.
      * @param intrinsic If true, uses intrinsic rotations; otherwise, extrinsic.
      * @param sequence Sequence of axes (e.g., "ZYX").
      * @param degree If true, angles are in degrees; otherwise, in radians.
      */
    Rotation(double angle1,
             double angle2,
             double angle3,
             bool intrinsic = true,
             const std::string &sequence = "ZYX",
             bool degree = false);

    /**
      * @brief Default destructor.
      */
    ~Rotation() = default;

    /** Static methods to define a rotation
      * Static methods allow creating a rotation without having to instantiate an object.
      *
      * Example:
      * Rotation rot = Rotation::fromEulerAngles(0, 0, 0, true, "XYZ", false);
      * Otherwise you would have to do:
      *
      * Rotation rot;
      * rot = rot.fromEulerAngles(0, 0, 0, true, "XYZ", false);
      *
      */

    /**
      * @brief Creates a rotation from a rotation matrix.
      * @param rotation_matrix 3x3 rotation matrix.
      * @return Rotation object.
      */
    static Rotation fromRotationMatrix(const Eigen::Matrix3d &rotation_matrix);

    /**
      * @brief Creates a rotation from a quaternion.
      * @param quaternion Quaternion representing the rotation.
      * @return Rotation object.
      */
    static Rotation fromQuaternion(const Eigen::Quaterniond &quaternion);

    /**
      * @brief Creates a rotation from an axis-angle representation.
      * @param angle_axis Axis-angle representation of the rotation.
      * @return Rotation object.
      */
    static Rotation fromAngleAxis(const Eigen::AngleAxisd &angle_axis);

    /**
      * @brief Creates a rotation from Euler angles.
      * @param angle1 First Euler angle.
      * @param angle2 Second Euler angle.
      * @param angle3 Third Euler angle.
      * @param intrinsic If true, uses intrinsic rotations; otherwise, extrinsic.
      * @param sequence Sequence of axes (e.g., "XYZ").
      * @param degree If true, angles are in degrees; otherwise, in radians.
      * @return Rotation object.
      */
    static Rotation fromEulerAngles(double angle1,
                                    double angle2,
                                    double angle3,
                                    bool intrinsic = true,
                                    const std::string &sequence = "XYZ",
                                    bool degree = false);

    // Converters

    /**
      * @brief Converts the rotation to a rotation matrix.
      * @return 3x3 rotation matrix.
      */
    Eigen::Matrix3d toRotationMatrix() const;

    /**
      * @brief Converts the rotation to a quaternion.
      * @return Quaternion representing the rotation.
      */
    Eigen::Quaterniond toQuaternion() const;

    /**
      * @brief Converts the rotation to an axis-angle representation.
      * @return Axis-angle representation of the rotation.
      */
    Eigen::AngleAxisd toAngleAxis() const;

    /**
      * @brief Converts the rotation to Euler angles.
      * @param intrinsic If true, uses intrinsic rotations; otherwise, extrinsic.
      * @param sequence Sequence of axes (e.g., "ZYX").
      * @return Vector containing the three Euler angles.
      */
    Eigen::Vector3d toEulerAngles(bool intrinsic = true, const std::string &sequence = "ZYX") const;

    // Overloading of operators * and =

    /**
      * @brief Composition of two rotations.
      * @param other Rotation to compose with.
      * @return New rotation resulting from the composition.
      */
    Rotation operator*(const Rotation &other) const;

    /**
      * @brief Assignment operator.
      * @param other Rotation to assign.
      * @return Reference to the current object.
      */
    Rotation &operator=(const Rotation &other);

    /**
      * @brief Move assignment operator.
      * @param other Rotation to assign.
      * @return Reference to the current object.
      */
    Rotation &operator=(Rotation &&other) noexcept;

    // Static methods for quaternion operations
    /**
      * @brief Calculates the difference between two quaternions.
      * @param a Reference quaternion.
      * @param b Quaternion to subtract.
      * @return Quaternion resulting from the difference.
      */
    static Eigen::Quaterniond quaternion_difference(const Eigen::Quaterniond &a,
                                                    const Eigen::Quaterniond &b);

    /**
      * @brief Negates the quaternion.
      * @param v Quaternion to negate.
      * @return Negated quaternion.
      */
    static Eigen::Quaterniond quaternion_negation(const Eigen::Quaterniond &v);

    /**
      * @brief Calculates the scalar product of a quaternion by a scalar.
      * @param v Quaternion to scale.
      * @param t Scale to multiply by.
      * @return Scaled quaternion.
      */
    static Eigen::Quaterniond scalar_product(const Eigen::Quaterniond &v, double t);

    /**
      * @brief Calculates the difference between two quaternions (v1 - v0).
      * @param v1 First quaternion.
      * @param v0 Second quaternion.
      * @return Quaternion resulting from the subtraction.
      */
    static Eigen::Quaterniond quaternion_minus(const Eigen::Quaterniond &v1,
                                               const Eigen::Quaterniond &v0);

    /**
      * @brief Calculates the sum of two quaternions.
      * @param v1 First quaternion.
      * @param v0 Second quaternion.
      * @return Quaternion resulting from the addition.
      */
    static Eigen::Quaterniond quaternion_plus(const Eigen::Quaterniond &v1,
                                              const Eigen::Quaterniond &v0);

    /**
      * @brief Converts degrees to radians.
      * @param degree Angle in degrees.
      * @return Angle in radians.
      */
    static double deg2rad(double degree);

    /**
      * @brief Converts radians to degrees.
      * @param radian Angle in radians.
      * @return Angle in degrees.
      */
    static double rad2deg(double radian);

    // Elementary Rotations Matrices

    /**
      * @brief Rotation matrix for a rotation around the X axis.
      * @param angle Rotation angle in radians.
      * @return 3x3 rotation matrix.
      */
    static Eigen::Matrix3d rotationX(double angle);

    /**
      * @brief Rotation matrix for a rotation around the Y axis.
      * @param angle Rotation angle in radians.
      * @return 3x3 rotation matrix.
      */

    static Eigen::Matrix3d rotationY(double angle);

    /**
      * @brief Rotation matrix for a rotation around the Z axis.
      * @param angle Rotation angle in radians.
      * @return 3x3 rotation matrix.
      */
    static Eigen::Matrix3d rotationZ(double angle);

    // Definition of Matrix S and R_dot

    /**
      * @brief Calculates the S matrix
      * @param omega Angular velocity vector. (Different from Euler angle velocities)
      * @return S matrix
      *
      */
    static Eigen::Matrix3d matrixS(const Eigen::Vector3d &omega);

    /**
      * @brief Calculates the R_dot matrix
      * @param R Rotation matrix.
      * @param S S matrix.
      * @return R_dot matrix
      *
      */
    static Eigen::Matrix3d matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Matrix3d &S);

    /**
      * @brief Calculates the R_dot matrix from an angular velocity vector.
      * @param omega Angular velocity vector.
      * @param R Rotation matrix.
      * @return R matrix
      *
      */
    static Eigen::Matrix3d matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Vector3d &omega);

    // Matrix T Mapping Between Body Angular Velocity Vector and the Euler Angle Rates

    /**
      * @brief Calculates the T matrix
      * @param angles Euler angles vector.
      * @param sequence Sequence of axes (e.g., "ZYX", "ZYZ").
      * @return T matrix
      *
      */
    static Eigen::Matrix3d matrixT(const Eigen::Vector3d &angles, const std::string &sequence);

private:
    /**
      * @brief Internal representation of rotation via quaternion.
      */
    Eigen::Quaterniond q;

    // Auxiliary functions

    /**
      * @brief Creates an axis-angle representation.
      * @param axis Rotation axis ('X', 'Y', 'Z' or lowercase).
      * @param angle Rotation angle in radians.
      * @return Axis-angle representation.
      */
    static Eigen::AngleAxisd angleAxis(char axis, double angle);

    /**
      * @brief Converts an axis character to index.
      * @param axis Rotation axis ('X', 'Y', 'Z' or lowercase).
      * @return Index corresponding to the axis (0 for X, 1 for Y, 2 for Z).
      * @throws std::invalid_argument If the axis is not valid.
      */
    static int axisToIndex(char axis);

    /**
      * @brief Checks the validity of the Euler angle sequence.
      * @param sequence Sequence of axes (e.g., "XYZ").
      * @return Integer value (always 1 if valid).
      * @throws std::invalid_argument If the sequence is not valid.
      */
    static int checkSequence(const std::string &sequence);
};

} // namespace MoveG

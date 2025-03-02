/**
 * @file pose_lib.h
 * @brief Class for representing Poses
 *
 * @author Giorgio Medico
 * @date 28/02/2025
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rotation_lib.h"

/**
  * @brief Namespace for movement and manipulation of poses.
  */
namespace MoveG
{
/**
      * @class Pose
      * @brief Class representing a pose in 3D space.
      *
      * The Pose class encapsulates a position and orientation in three-dimensional space.
      * It uses Eigen for mathematical operations related to geometry.
      */
class Pose
{
public:
    // Constructors

    /**
          * @brief Default constructor.
          *
          * Initializes the pose at position (0,0,0) with neutral orientation (identity quaternion).
          */
    Pose() noexcept;

    /**
          * @brief Constructor with position and quaternion orientation.
          *
          * @param position The position in space.
          * @param orientation The quaternion representing the orientation.
          */
    Pose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

    /**
          * @brief Constructor with position and rotation matrix.
          *
          * @param position The position in space.
          * @param rotation_matrix The 3x3 rotation matrix.
          */
    Pose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation_matrix);

    /**
          * @brief Constructor with affine transformation.
          *
          * @param transformation The 3D affine transformation.
          */
    explicit Pose(const Eigen::Affine3d &transformation);

    /**
          * @brief Constructor with position and Rotation object.
          *
          * @param position The position in space.
          * @param orientation The Rotation object representing the orientation.
          */
    Pose(const Eigen::Vector3d &position, const Rotation &orientation);

    /**
          * @brief Constructor with homogeneous transformation matrix 4x4.
          *
          * @param homogeneousT The 4x4 homogeneous transformation matrix.
          * @throws std::invalid_argument If the matrix is not a valid homogeneous transformation matrix.
          */
    explicit Pose(const Eigen::Matrix4d &homogeneousT);

    /**
          * @brief Destructor.
          */
    ~Pose();

    /**
          * @brief Copy constructor.
          *
          * @param other The Pose object to copy.
          */
    Pose(const Pose &other);

    /**
          * @brief Move constructor.
          *
          * @param other The Pose object to move.
          */
    Pose(Pose &&other) noexcept;

    /**
          * @brief Copy assignment operator.
          *
          * @param other The Pose object to assign.
          * @return Reference to the assigned object.
          */
    Pose &operator=(const Pose &other);

    /**
          * @brief Move assignment operator.
          *
          * @param other The Pose object to move and assign.
          * @return Reference to the assigned object.
          */
    Pose &operator=(Pose &&other) noexcept;

    /**
          * @brief Overloads the stream insertion operator.
          *
          * @param os The output stream.
          * @param pose The Pose object to insert into the stream.
          * @return Reference to the output stream.
          */
    friend std::ostream &operator<<(std::ostream &os, const Pose &pose);

    // Getters

    /**
          * @brief Gets the position.
          *
          * @return The position as a 3D vector.
          */
    [[nodiscard]] Eigen::Vector3d getPosition() const;

    /**
          * @brief Gets the orientation quaternion.
          *
          * @return The quaternion representing the orientation.
          */
    [[nodiscard]] Eigen::Quaterniond getQuaternion() const;

    /**
          * @brief Gets the rotation matrix.
          *
          * @return The 3x3 rotation matrix.
          */
    [[nodiscard]] Eigen::Matrix3d getRotationMatrix() const;

    /**
          * @brief Gets the affine transformation.
          *
          * @return The 3D affine transformation.
          */
    [[nodiscard]] Eigen::Affine3d getAffineTransformation() const;

    /**
          * @brief Gets the homogeneous transformation matrix.
          *
          * @return The 4x4 homogeneous matrix.
          */
    [[nodiscard]] Eigen::Matrix4d getHomogeneousT() const;

    /**
          * @brief Gets the X coordinate of the position.
          *
          * @return The X coordinate.
          */
    [[nodiscard]] double getX() const;

    /**
          * @brief Gets the Y coordinate of the position.
          *
          * @return The Y coordinate.
          */
    [[nodiscard]] double getY() const;

    /**
          * @brief Gets the Z coordinate of the position.
          *
          * @return The Z coordinate.
          */
    [[nodiscard]] double getZ() const;

    /**
          * @brief Gets the X component of the quaternion.
          *
          * @return The X component.
          */
    [[nodiscard]] double getQx() const;

    /**
          * @brief Gets the Y component of the quaternion.
          *
          * @return The Y component.
          */
    [[nodiscard]] double getQy() const;

    /**
          * @brief Gets the Z component of the quaternion.
          *
          * @return The Z component.
          */
    [[nodiscard]] double getQz() const;

    /**
          * @brief Gets the W component of the quaternion.
          *
          * @return The W component.
          */
    [[nodiscard]] double getQw() const;

    // Setters

    /**
          * @brief Sets the position.
          *
          * @param position The new position as a 3D vector.
          */
    void setPosition(const Eigen::Vector3d &position);

    /**
          * @brief Sets the orientation via quaternion.
          *
          * @param orientation The new quaternion representing the orientation.
          */
    void setOrientation(const Eigen::Quaterniond &orientation);

    /**
          * @brief Sets the rotation matrix.
          *
          * @param rotation_matrix The new 3x3 rotation matrix.
          */
    void setRotationMatrix(const Eigen::Matrix3d &rotation_matrix);

    /**
          * @brief Sets the affine transformation.
          *
          * @param transformation The new 3D affine transformation.
          */
    void setAffineTransformation(const Eigen::Affine3d &transformation);

    /**
          * @brief Sets the homogeneous transformation matrix.
          *
          * @param homogeneousT The new 4x4 homogeneous matrix.
          * @throws std::invalid_argument If the matrix is not a valid homogeneous transformation matrix.
          */
    void setHomogeneousT(const Eigen::Matrix4d &homogeneousT);

    // Distance metrics

    /**
          * @brief Calculates the Euclidean distance between the positions of two poses.
          *
          * @param other The pose to calculate the distance with.
          * @return The Euclidean distance between the positions.
          */
    [[nodiscard]] double positionDistance(const Pose &other) const;

    /**
          * @brief Calculates the angular distance between the orientations of two poses.
          *
          * @param other The pose to calculate the angular distance with.
          * @return The angular distance in radians.
          */
    [[nodiscard]] double orientationDistance(const Pose &other) const;

    // Operations

    /**
          * @brief Composition of two poses.
          *
          * @param other The second pose to compose.
          * @return A new pose resulting from the composition.
          */
    [[nodiscard]] Pose operator*(const Pose &other) const; // Compose poses

    /**
          * @brief Calculates the inverse of the pose.
          *
          * @return The inverse pose.
          */
    [[nodiscard]] Pose inverse() const;

    /**
     * @brief Transforms this pose from one coordinate frame to another.
     *
     * @param transform The transformation between the two coordinate frames.
     * @return A new pose expressed in the target coordinate frame.
     */
    [[nodiscard]] Pose transformPose(const Pose &transform) const;

    /**
     * @brief Transforms a point from the pose's local frame to the global frame.
     *
     * @param local_point The point in the local coordinate frame.
     * @return The point expressed in the global coordinate frame.
     */
    [[nodiscard]] Eigen::Vector3d localToGlobal(const Eigen::Vector3d &local_point) const;

    /**
     * @brief Transforms a point from the global frame to the pose's local frame.
     *
     * @param global_point The point in the global coordinate frame.
     * @return The point expressed in the local coordinate frame.
     */
    [[nodiscard]] Eigen::Vector3d globalToLocal(const Eigen::Vector3d &global_point) const;

private:
    Eigen::Vector3d position_;       ///< Position in 3D space.
    Eigen::Quaterniond orientation_; ///< Orientation represented as quaternion.

    /**
     * @brief Validates a homogeneous transformation matrix.
     *
     * Checks that the last row is [0,0,0,1] and that the rotation part is orthogonal.
     *
     * @param homogeneousT The homogeneous transformation matrix to validate.
     * @throws std::invalid_argument If the matrix is not a valid homogeneous transformation matrix.
     */
    static void validateHomogeneousMatrix(const Eigen::Matrix4d &homogeneousT);
};

} // namespace MoveG

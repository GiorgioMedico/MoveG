/*
    Author: Giorgio Medico
    Date: 4/10/2024
    Description: Classe per la rappresentazione di Pose
    File : pose_lib.h
*/

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "rotation_lib.h"

/**
 * Descrizione di Eigen Affine3d
 *
 *
 */

namespace MoveG
{
class Pose
{
public:
    // Constructors
    Pose();
    Pose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);
    Pose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation_matrix);
    Pose(const Eigen::Affine3d &transformation);
    Pose(const Eigen::Vector3d &position, const Rotation &orientation);
    Pose(const Eigen::Matrix4d &homogeneousT);

    // Destructor
    ~Pose();

    // Copy constructor and assignment operator
    Pose(const Pose &other);
    Pose &operator=(const Pose &other);

    // << operator overload to define the output stream
    friend std::ostream &operator<<(std::ostream &os, const Pose &pose);

    // Getters
    Eigen::Vector3d getPosition() const;
    Eigen::Quaterniond getQaternion() const;
    Eigen::Matrix3d getRotationMatrix() const;
    Eigen::Affine3d getAffineTransformation() const;
    Eigen::Matrix4d getHomogeneousT() const;

    // Get x, y, z position and quaternion components x, y, z, w
    double getX() const;
    double getY() const;
    double getZ() const;
    double getQx() const;
    double getQy() const;
    double getQz() const;
    double getQw() const;


    // Setters
    void setPosition(const Eigen::Vector3d &position);
    void setOrientation(const Eigen::Quaterniond &orientation);
    void setRotationMatrix(const Eigen::Matrix3d &rotation_matrix);
    void setAffineTransformation(const Eigen::Affine3d &transformation);
    void setHomogeneousT(const Eigen::Matrix4d &homogeneousT);

    // Operations
    Pose operator*(const Pose &other) const; // Compose poses
    Pose inverse() const;

    // Friend functions for input/output
    friend std::ostream &operator<<(std::ostream &os, const Pose &pose);

private:
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
};

} // namespace MoveG

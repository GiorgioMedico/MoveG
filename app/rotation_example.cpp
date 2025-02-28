/**
 * @file rotation_example.cpp
 * @brief Example demonstrating usage of the MoveG Rotation library
 */

#include "rotation_lib.h"
#include <iomanip>
#include <iostream>

using namespace MoveG;

// Utility function to print Euler angles in degrees
void printEulerAngles(const Eigen::Vector3d &angles,
                      const std::string &description,
                      const std::string &sequence)
{
    std::cout << description << " (" << sequence << "): [" << std::fixed << std::setprecision(2)
              << Rotation::rad2deg(angles[0]) << "°, " << Rotation::rad2deg(angles[1]) << "°, "
              << Rotation::rad2deg(angles[2]) << "°]" << std::endl;
}

// Utility function to print quaternion
void printQuaternion(const Eigen::Quaterniond &quat, const std::string &description)
{
    std::cout << description << " [w, x, y, z]: [" << std::fixed << std::setprecision(4) << quat.w()
              << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << "]" << std::endl;
}

int main()
{
    std::cout << "==============================================" << std::endl;
    std::cout << "MoveG Rotation Library Example Usage" << std::endl;
    std::cout << "==============================================" << std::endl;

    // Example 1: Creating rotations using different constructors
    std::cout << "\n1. Creating rotations using different constructors:" << std::endl;

    // Default constructor (identity rotation)
    Rotation identity;
    std::cout << "Identity rotation matrix:\n" << identity.toRotationMatrix() << std::endl;

    // From Euler angles (ZYX sequence, intrinsic)
    double roll = Rotation::deg2rad(30.0);  // Around X-axis
    double pitch = Rotation::deg2rad(45.0); // Around Y-axis
    double yaw = Rotation::deg2rad(60.0);   // Around Z-axis

    Rotation rot_euler(yaw, pitch, roll, true, "ZYX", false);
    std::cout << "Rotation from Euler angles (ZYX, intrinsic):\n"
              << rot_euler.toRotationMatrix() << std::endl;

    // From rotation matrix
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Rotation rot_matrix(R);
    std::cout << "Rotation from matrix (45° around Z):\n"
              << rot_matrix.toRotationMatrix() << std::endl;

    // From quaternion
    Eigen::Quaterniond q =
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY()));
    Rotation rot_quat(q);
    std::cout << "Rotation from quaternion (60° around Y):\n"
              << rot_quat.toRotationMatrix() << std::endl;

    // From angle-axis
    Eigen::AngleAxisd aa(M_PI / 6, Eigen::Vector3d::UnitX());
    Rotation rot_aa(aa);
    std::cout << "Rotation from angle-axis (30° around X):\n"
              << rot_aa.toRotationMatrix() << std::endl;

    // Example 2: Using static methods to create rotations
    std::cout << "\n2. Using static methods to create rotations:" << std::endl;

    Rotation rot_static_euler = Rotation::fromEulerAngles(yaw, pitch, roll, true, "ZYX", false);
    std::cout << "Static method Euler angles rotation:\n"
              << rot_static_euler.toRotationMatrix() << std::endl;

    Rotation rot_static_quat = Rotation::fromQuaternion(q);
    std::cout << "Static method quaternion rotation:\n"
              << rot_static_quat.toRotationMatrix() << std::endl;

    // Example 3: Converting between different rotation representations
    std::cout << "\n3. Converting between different rotation representations:" << std::endl;

    // Create a rotation from Euler angles
    Rotation rot_convert(M_PI / 4, M_PI / 6, M_PI / 3, true, "ZYX", false);

    // Convert to different representations
    Eigen::Matrix3d rot_mat = rot_convert.toRotationMatrix();
    Eigen::Quaterniond rot_q = rot_convert.toQuaternion();
    Eigen::AngleAxisd rot_aa_convert = rot_convert.toAngleAxis();
    Eigen::Vector3d rot_euler_angles = rot_convert.toEulerAngles(true, "ZYX");

    std::cout << "Original rotation from Euler angles (ZYX):" << std::endl;
    printEulerAngles(Eigen::Vector3d(M_PI / 4, M_PI / 6, M_PI / 3), "Input angles", "ZYX");

    std::cout << "\nAs a rotation matrix:\n" << rot_mat << std::endl;

    printQuaternion(rot_q, "As a quaternion");

    std::cout << "As an angle-axis: " << Rotation::rad2deg(rot_aa_convert.angle())
              << "° around axis [" << rot_aa_convert.axis().x() << ", " << rot_aa_convert.axis().y()
              << ", " << rot_aa_convert.axis().z() << "]" << std::endl;

    printEulerAngles(rot_euler_angles, "As Euler angles", "ZYX");

    // Example 4: Composition of rotations
    std::cout << "\n4. Composition of rotations:" << std::endl;

    // Create two rotations
    Rotation rot1 = Rotation::fromEulerAngles(0, 0, M_PI / 2, true, "ZYX", false); // 90° around Z
    Rotation rot2 = Rotation::fromEulerAngles(0, M_PI / 2, 0, true, "ZYX", false); // 90° around Y

    // Compose them
    Rotation rot_combined = rot1 * rot2;

    std::cout << "Rotation 1 (90° around Z):\n" << rot1.toRotationMatrix() << std::endl;
    std::cout << "Rotation 2 (90° around Y):\n" << rot2.toRotationMatrix() << std::endl;
    std::cout << "Combined rotation (rot1 * rot2):\n"
              << rot_combined.toRotationMatrix() << std::endl;

    // Order matters!
    Rotation rot_combined2 = rot2 * rot1;
    std::cout << "Different order (rot2 * rot1):\n"
              << rot_combined2.toRotationMatrix() << std::endl;

    // Example 5: Utility functions
    std::cout << "\n5. Utility functions:" << std::endl;

    // Converting between degrees and radians
    double angle_deg = 45.0;
    double angle_rad = Rotation::deg2rad(angle_deg);
    std::cout << angle_deg << "° = " << angle_rad << " radians" << std::endl;
    std::cout << angle_rad << " radians = " << Rotation::rad2deg(angle_rad) << "°" << std::endl;

    // Normalizing angles
    double big_angle = 5 * M_PI; // 5π radians (900°)
    double normalized = Rotation::normalizeAngle(big_angle);
    std::cout << "Normalizing " << big_angle << " radians to [-π, π]: " << normalized
              << " radians (" << Rotation::rad2deg(normalized) << "°)" << std::endl;

    // Elementary rotation matrices
    std::cout << "\nElementary rotation matrices:" << std::endl;
    std::cout << "Rotation around X (30°):\n" << Rotation::rotationX(M_PI / 6) << std::endl;
    std::cout << "Rotation around Y (45°):\n" << Rotation::rotationY(M_PI / 4) << std::endl;
    std::cout << "Rotation around Z (60°):\n" << Rotation::rotationZ(M_PI / 3) << std::endl;

    // Matrix S and R_dot
    Eigen::Vector3d omega(0.1, 0.2, 0.3); // Angular velocity
    std::cout << "\nMatrix S for angular velocity [0.1, 0.2, 0.3]:\n"
              << Rotation::matrixS(omega) << std::endl;

    Eigen::Matrix3d R_current = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_dot = Rotation::matrixR_dot(R_current, omega);
    std::cout << "Matrix R_dot:\n" << R_dot << std::endl;

    // Matrix T
    try
    {
        Eigen::Vector3d angles_for_T(M_PI / 4, M_PI / 6, M_PI / 3);
        Eigen::Matrix3d T = Rotation::matrixT(angles_for_T, "ZYX");
        std::cout << "\nMatrix T for ZYX Euler angles [" << Rotation::rad2deg(angles_for_T[0])
                  << "°, " << Rotation::rad2deg(angles_for_T[1]) << "°, "
                  << Rotation::rad2deg(angles_for_T[2]) << "°]:\n"
                  << T << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error calculating matrix T: " << e.what() << std::endl;
    }

    std::cout << "==============================================" << std::endl;

    return 0;
}

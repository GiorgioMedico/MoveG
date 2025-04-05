/**
 * @file rotation_example.cpp
 * @brief Example demonstrating usage of the MoveG Rotation library with fmt formatting
 */

#include "pose/rotation_lib.h"
#include "utils/fmt_formatter.h"
#include <fmt/core.h>
#include <fmt/format.h>

using namespace MoveG;

int main()
{
    fmt::print("==============================================\n");
    fmt::print("MoveG Rotation Library Example Usage\n");
    fmt::print("==============================================\n");

    // Example 1: Creating rotations using different constructors
    fmt::print("\n1. Creating rotations using different constructors:\n");

    // Default constructor (identity rotation)
    Rotation identity;
    fmt::print("Identity rotation matrix:\n{}\n", identity.toRotationMatrix());

    // From Euler angles (ZYX sequence, intrinsic)
    double roll = Rotation::deg2rad(30.0);  // Around X-axis
    double pitch = Rotation::deg2rad(45.0); // Around Y-axis
    double yaw = Rotation::deg2rad(60.0);   // Around Z-axis

    Rotation rot_euler(yaw, pitch, roll, true, "ZYX", false);
    fmt::print("Rotation from Euler angles (ZYX, intrinsic):\n{}\n", rot_euler.toRotationMatrix());

    // From rotation matrix
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Rotation rot_matrix(R);
    fmt::print("Rotation from matrix (45° around Z):\n{}\n", rot_matrix.toRotationMatrix());

    // From quaternion
    Eigen::Quaterniond q =
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY()));
    Rotation rot_quat(q);
    fmt::print("Rotation from quaternion (60° around Y):\n{}\n", rot_quat.toRotationMatrix());

    // From angle-axis
    Eigen::AngleAxisd aa(M_PI / 6, Eigen::Vector3d::UnitX());
    Rotation rot_aa(aa);
    fmt::print("Rotation from angle-axis (30° around X):\n{}\n", rot_aa.toRotationMatrix());

    // Example 2: Using static methods to create rotations
    fmt::print("\n2. Using static methods to create rotations:\n");

    Rotation rot_static_euler = Rotation::fromEulerAngles(yaw, pitch, roll, true, "ZYX", false);
    fmt::print("Static method Euler angles rotation:\n{}\n", rot_static_euler.toRotationMatrix());

    Rotation rot_static_quat = Rotation::fromQuaternion(q);
    fmt::print("Static method quaternion rotation:\n{}\n", rot_static_quat.toRotationMatrix());

    // Example 3: Converting between different rotation representations
    fmt::print("\n3. Converting between different rotation representations:\n");

    // Create a rotation from Euler angles
    Rotation rot_convert(M_PI / 4, M_PI / 6, M_PI / 3, true, "ZYX", false);

    // Convert to different representations
    Eigen::Matrix3d rot_mat = rot_convert.toRotationMatrix();
    Eigen::Quaterniond rot_q = rot_convert.toQuaternion();
    Eigen::AngleAxisd rot_aa_convert = rot_convert.toAngleAxis();
    Eigen::Vector3d rot_euler_angles = rot_convert.toEulerAngles(true, "ZYX");

    fmt::print("Original rotation from Euler angles (ZYX):\n");
    fmt::print("Input angles (ZYX): [{:.2f}°, {:.2f}°, {:.2f}°]\n",
               Rotation::rad2deg(M_PI / 4),
               Rotation::rad2deg(M_PI / 6),
               Rotation::rad2deg(M_PI / 3));

    fmt::print("\nAs a rotation matrix:\n{}\n", rot_mat);
    fmt::print("As a quaternion: {}\n", rot_q);
    fmt::print("As an angle-axis: {}\n", rot_aa_convert);
    fmt::print("As Euler angles (ZYX): [{:.2f}°, {:.2f}°, {:.2f}°]\n",
               Rotation::rad2deg(rot_euler_angles[0]),
               Rotation::rad2deg(rot_euler_angles[1]),
               Rotation::rad2deg(rot_euler_angles[2]));

    // Example 4: Composition of rotations
    fmt::print("\n4. Composition of rotations:\n");

    // Create two rotations
    Rotation rot1 = Rotation::fromEulerAngles(0, 0, M_PI / 2, true, "ZYX", false); // 90° around Z
    Rotation rot2 = Rotation::fromEulerAngles(0, M_PI / 2, 0, true, "ZYX", false); // 90° around Y

    // Compose them
    Rotation rot_combined = rot1 * rot2;

    fmt::print("Rotation 1 (90° around Z):\n{}\n", rot1.toRotationMatrix());
    fmt::print("Rotation 2 (90° around Y):\n{}\n", rot2.toRotationMatrix());
    fmt::print("Combined rotation (rot1 * rot2):\n{}\n", rot_combined.toRotationMatrix());

    // Order matters!
    Rotation rot_combined2 = rot2 * rot1;
    fmt::print("Different order (rot2 * rot1):\n{}\n", rot_combined2.toRotationMatrix());

    // Example 5: Utility functions
    fmt::print("\n5. Utility functions:\n");

    // Converting between degrees and radians
    double angle_deg = 45.0;
    double angle_rad = Rotation::deg2rad(angle_deg);
    fmt::print("{:.1f}° = {:.6f} radians\n", angle_deg, angle_rad);
    fmt::print("{:.6f} radians = {:.1f}°\n", angle_rad, Rotation::rad2deg(angle_rad));

    // Normalizing angles
    double big_angle = 5 * M_PI; // 5π radians (900°)
    double normalized = Rotation::normalizeAngle(big_angle);
    fmt::print("Normalizing {:.6f} radians to [-π, π]: {:.6f} radians ({:.2f}°)\n",
               big_angle,
               normalized,
               Rotation::rad2deg(normalized));

    // Elementary rotation matrices
    fmt::print("\nElementary rotation matrices:\n");
    fmt::print("Rotation around X (30°):\n{}\n", Rotation::rotationX(M_PI / 6));
    fmt::print("Rotation around Y (45°):\n{}\n", Rotation::rotationY(M_PI / 4));
    fmt::print("Rotation around Z (60°):\n{}\n", Rotation::rotationZ(M_PI / 3));

    // Matrix S and R_dot
    Eigen::Vector3d omega(0.1, 0.2, 0.3); // Angular velocity
    fmt::print("\nMatrix S for angular velocity {}:\n{}\n", omega, Rotation::matrixS(omega));

    Eigen::Matrix3d R_current = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_dot = Rotation::matrixR_dot(R_current, omega);
    fmt::print("Matrix R_dot:\n{}\n", R_dot);

    // Matrix T
    try
    {
        Eigen::Vector3d angles_for_T(M_PI / 4, M_PI / 6, M_PI / 3);
        Eigen::Matrix3d T = Rotation::matrixT(angles_for_T, "ZYX");
        fmt::print("\nMatrix T for ZYX Euler angles [{:.2f}°, {:.2f}°, {:.2f}°]:\n{}\n",
                   Rotation::rad2deg(angles_for_T[0]),
                   Rotation::rad2deg(angles_for_T[1]),
                   Rotation::rad2deg(angles_for_T[2]),
                   T);
    }
    catch (const std::exception &e)
    {
        fmt::print(stderr, "Error calculating matrix T: {}\n", e.what());
    }

    fmt::print("==============================================\n");

    return 0;
}

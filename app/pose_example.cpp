/**
 * @file pose_example.cpp
 * @brief Example demonstrating usage of the MoveG Pose library with fmt formatting
 */

#include "pose/pose_lib.h"
#include "pose/rotation_lib.h"
#include "utils/fmt_formatter.h"
#include <fmt/core.h>
#include <fmt/format.h>
#include <vector>

using namespace MoveG;

int main()
{
    fmt::print("==============================================\n");
    fmt::print("MoveG Pose Library Example Usage\n");
    fmt::print("==============================================\n");

    // Example 1: Creating poses using different constructors
    fmt::print("\n1. Creating poses using different constructors:\n");

    // Default constructor (identity pose at origin)
    Pose identity_pose;
    fmt::print("Identity pose at origin:\n");
    fmt::print("Position: {}\n", identity_pose.getPosition());
    fmt::print("Orientation: {}\n", identity_pose.getQuaternion());

    // From position and quaternion
    Eigen::Vector3d position(1.0, 2.0, 3.0);
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

    Pose pose_from_quat(position, orientation);
    fmt::print("\nPose from position and quaternion:\n");
    fmt::print("Position: {}\n", pose_from_quat.getPosition());
    fmt::print("Orientation: {}\n", pose_from_quat.getQuaternion());

    // From position and rotation matrix
    Eigen::Matrix3d rot_matrix =
        Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Pose pose_from_matrix(position, rot_matrix);
    fmt::print("\nPose from position and rotation matrix:\n");
    fmt::print("Position: {}\n", pose_from_matrix.getPosition());
    fmt::print("Rotation matrix:\n{}\n", pose_from_matrix.getRotationMatrix());

    // From Affine3d transformation
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.translation() = Eigen::Vector3d(4.0, 5.0, 6.0);
    affine.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

    Pose pose_from_affine(affine);
    fmt::print("\nPose from Affine3d transformation:\n");
    fmt::print("Position: {}\n", pose_from_affine.getPosition());
    fmt::print("Rotation matrix:\n{}\n", pose_from_affine.getRotationMatrix());

    // From position and Rotation object
    Rotation rotation =
        Rotation::fromAngleAxis(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitX()));
    Pose pose_from_rotation(position, rotation);
    fmt::print("\nPose from position and Rotation object:\n");
    fmt::print("Position: {}\n", pose_from_rotation.getPosition());
    fmt::print("Orientation: {}\n", pose_from_rotation.getQuaternion());

    // From homogeneous transformation matrix
    Eigen::Matrix4d homogeneous = Eigen::Matrix4d::Identity();
    homogeneous.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    homogeneous.block<3, 1>(0, 3) = Eigen::Vector3d(7.0, 8.0, 9.0);

    Pose pose_from_homogeneous(homogeneous);
    fmt::print("\nPose from homogeneous transformation matrix:\n");
    fmt::print("Position: {}\n", pose_from_homogeneous.getPosition());
    fmt::print("Orientation: {}\n", pose_from_homogeneous.getQuaternion());

    // Example 2: Getting different representations of a pose
    fmt::print("\n2. Getting different representations of a pose:\n");

    Pose example_pose(Eigen::Vector3d(1.0, 2.0, 3.0),
                      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())));

    // Get individual position components
    fmt::print("Individual position components: x={:.4f}, y={:.4f}, z={:.4f}\n",
               example_pose.getX(),
               example_pose.getY(),
               example_pose.getZ());

    // Get individual quaternion components
    fmt::print("Individual quaternion components: w={:.4f}, x={:.4f}, y={:.4f}, z={:.4f}\n",
               example_pose.getQw(),
               example_pose.getQx(),
               example_pose.getQy(),
               example_pose.getQz());

    // Get as rotation matrix
    fmt::print("Rotation matrix:\n{}\n", example_pose.getRotationMatrix());

    // Get as affine transformation
    Eigen::Affine3d example_affine = example_pose.getAffineTransformation();
    fmt::print("Affine transformation:\n{}\n", example_affine);

    // Get as homogeneous transformation matrix
    Eigen::Matrix4d example_homogeneous = example_pose.getHomogeneousT();
    fmt::print("Homogeneous transformation matrix:\n{}\n", example_homogeneous);

    // Example 3: Modifying poses
    fmt::print("\n3. Modifying poses:\n");

    Pose modifiable_pose;
    fmt::print("Initial pose:\n");
    fmt::print("Position: {}\n", modifiable_pose.getPosition());
    fmt::print("Orientation: {}\n", modifiable_pose.getQuaternion());

    // Set position
    modifiable_pose.setPosition(Eigen::Vector3d(5.0, 6.0, 7.0));

    // Set orientation with quaternion
    modifiable_pose.setOrientation(
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY())));

    fmt::print("\nAfter modification:\n");
    fmt::print("Position: {}\n", modifiable_pose.getPosition());
    fmt::print("Orientation: {}\n", modifiable_pose.getQuaternion());

    // Set orientation with rotation matrix
    Eigen::Matrix3d new_rotation =
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    modifiable_pose.setRotationMatrix(new_rotation);

    fmt::print("\nAfter setting rotation matrix:\n");
    fmt::print("Position: {}\n", modifiable_pose.getPosition());
    fmt::print("Rotation matrix:\n{}\n", modifiable_pose.getRotationMatrix());

    // Set with homogeneous transformation
    Eigen::Matrix4d new_homogeneous = Eigen::Matrix4d::Identity();
    new_homogeneous.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitX()).toRotationMatrix();
    new_homogeneous.block<3, 1>(0, 3) = Eigen::Vector3d(10.0, 11.0, 12.0);

    modifiable_pose.setHomogeneousT(new_homogeneous);

    fmt::print("\nAfter setting homogeneous transformation:\n");
    fmt::print("Position: {}\n", modifiable_pose.getPosition());
    fmt::print("Orientation: {}\n", modifiable_pose.getQuaternion());

    // Example 4: Pose operations
    fmt::print("\n4. Pose operations:\n");

    // Create two poses for operations
    Pose pose1(Eigen::Vector3d(1.0, 0.0, 0.0),
               Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

    Pose pose2(Eigen::Vector3d(0.0, 1.0, 0.0),
               Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

    fmt::print("Pose 1:\n");
    fmt::print("Position: {}\n", pose1.getPosition());
    fmt::print("Orientation: {}\n", pose1.getQuaternion());

    fmt::print("\nPose 2:\n");
    fmt::print("Position: {}\n", pose2.getPosition());
    fmt::print("Orientation: {}\n", pose2.getQuaternion());

    // Compose poses
    Pose composed = pose1 * pose2;
    fmt::print("\nComposed pose (pose1 * pose2):\n");
    fmt::print("Position: {}\n", composed.getPosition());
    fmt::print("Orientation: {}\n", composed.getQuaternion());

    // Inverse pose
    Pose inverse = pose1.inverse();
    fmt::print("\nInverse of pose1:\n");
    fmt::print("Position: {}\n", inverse.getPosition());
    fmt::print("Orientation: {}\n", inverse.getQuaternion());

    // Check that pose * inverse = identity
    Pose identity_check = pose1 * inverse;
    fmt::print("\nPose1 * Inverse (should be identity):\n");
    fmt::print("Position: {}\n", identity_check.getPosition());
    fmt::print("Orientation: {}\n", identity_check.getQuaternion());

    // Example 5: Distance metrics
    fmt::print("\n5. Distance metrics:\n");

    Pose pose_a(Eigen::Vector3d(1.0, 2.0, 3.0),
                Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())));

    Pose pose_b(Eigen::Vector3d(4.0, 6.0, 8.0),
                Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

    double position_distance = pose_a.positionDistance(pose_b);
    double orientation_distance = pose_a.orientationDistance(pose_b);

    fmt::print("Position distance: {:.4f} meters\n", position_distance);
    fmt::print("Orientation distance: {:.4f} radians ({:.4f} degrees)\n",
               orientation_distance,
               Rotation::rad2deg(orientation_distance));

    // Example 6: Coordinate transformations
    fmt::print("\n6. Coordinate transformations:\n");

    // Create a pose representing a robot's pose in the world
    Pose robot_in_world(
        Eigen::Vector3d(2.0, 3.0, 0.0),                       // Position in world frame
        Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()) // 45° rotation around Z
    );

    // Create a point in the robot's local frame
    Eigen::Vector3d point_in_robot(1.0, 0.0, 0.0); // 1 meter in front of the robot

    // Transform point to world coordinates
    Eigen::Vector3d point_in_world = robot_in_world.localToGlobal(point_in_robot);

    fmt::print("Robot pose in world:\n");
    fmt::print("Position: {}\n", robot_in_world.getPosition());
    fmt::print("Rotation: {}\n", Eigen::AngleAxisd(robot_in_world.getQuaternion()));

    fmt::print("\nPoint in robot's local frame: {}\n", point_in_robot);
    fmt::print("Same point in world frame: {}\n", point_in_world);

    // Transform point back to robot coordinates
    Eigen::Vector3d point_back_in_robot = robot_in_world.globalToLocal(point_in_world);
    fmt::print("Point transformed back to robot frame: {}\n", point_back_in_robot);

    // Transform between coordinate frames
    // Create a second pose representing a sensor mounted on the robot
    Pose sensor_in_robot(
        Eigen::Vector3d(0.0, 0.0, 0.5),                  // Sensor is 0.5m above the robot's origin
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) // No rotation relative to robot
    );

    // Calculate sensor pose in world frame
    Pose sensor_in_world = robot_in_world.transformPose(sensor_in_robot);

    fmt::print("\nSensor pose in robot frame:\n");
    fmt::print("Position: {}\n", sensor_in_robot.getPosition());
    fmt::print("Orientation: {}\n", sensor_in_robot.getQuaternion());

    fmt::print("\nSensor pose in world frame:\n");
    fmt::print("Position: {}\n", sensor_in_world.getPosition());
    fmt::print("Orientation: {}\n", sensor_in_world.getQuaternion());

    fmt::print("==============================================\n");

    return 0;
}

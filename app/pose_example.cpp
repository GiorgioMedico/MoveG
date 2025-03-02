/**
 * @file pose_example.cpp
 * @brief Example demonstrating usage of the MoveG Pose library
 */

#include "pose/pose_lib.h"
#include "pose/rotation_lib.h"
#include <iomanip>
#include <iostream>
#include <vector>

using namespace MoveG;

// Utility function to print a vector
void printVector3d(const Eigen::Vector3d &vec, const std::string &description)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << description << ": [" << vec.x() << ", " << vec.y() << ", " << vec.z() << "]"
              << std::endl;
}

// Utility function to print a quaternion
void printQuaternion(const Eigen::Quaterniond &quat, const std::string &description)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << description << " [w, x, y, z]: [" << quat.w() << ", " << quat.x() << ", "
              << quat.y() << ", " << quat.z() << "]" << std::endl;
}

// Utility function to print a homogeneous transformation matrix
void printHomogeneousMatrix(const Eigen::Matrix4d &mat, const std::string &description)
{
    std::cout << std::fixed << std::setprecision(4);
    std::cout << description << ":\n" << mat << std::endl;
}

int main()
{
    std::cout << "==============================================" << std::endl;
    std::cout << "MoveG Pose Library Example Usage" << std::endl;
    std::cout << "==============================================" << std::endl;

    // Example 1: Creating poses using different constructors
    std::cout << "\n1. Creating poses using different constructors:" << std::endl;

    // Default constructor (identity pose at origin)
    Pose identity_pose;
    std::cout << "Identity pose at origin:" << std::endl;
    printVector3d(identity_pose.getPosition(), "Position");
    printQuaternion(identity_pose.getQuaternion(), "Orientation");

    // From position and quaternion
    Eigen::Vector3d position(1.0, 2.0, 3.0);
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));

    Pose pose_from_quat(position, orientation);
    std::cout << "\nPose from position and quaternion:" << std::endl;
    printVector3d(pose_from_quat.getPosition(), "Position");
    printQuaternion(pose_from_quat.getQuaternion(), "Orientation");

    // From position and rotation matrix
    Eigen::Matrix3d rot_matrix =
        Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Pose pose_from_matrix(position, rot_matrix);
    std::cout << "\nPose from position and rotation matrix:" << std::endl;
    printVector3d(pose_from_matrix.getPosition(), "Position");
    std::cout << "Rotation matrix:\n" << pose_from_matrix.getRotationMatrix() << std::endl;

    // From Affine3d transformation
    Eigen::Affine3d affine = Eigen::Affine3d::Identity();
    affine.translation() = Eigen::Vector3d(4.0, 5.0, 6.0);
    affine.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));

    Pose pose_from_affine(affine);
    std::cout << "\nPose from Affine3d transformation:" << std::endl;
    printVector3d(pose_from_affine.getPosition(), "Position");
    std::cout << "Rotation matrix:\n" << pose_from_affine.getRotationMatrix() << std::endl;

    // From position and Rotation object
    Rotation rotation =
        Rotation::fromAngleAxis(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitX()));
    Pose pose_from_rotation(position, rotation);
    std::cout << "\nPose from position and Rotation object:" << std::endl;
    printVector3d(pose_from_rotation.getPosition(), "Position");
    printQuaternion(pose_from_rotation.getQuaternion(), "Orientation");

    // From homogeneous transformation matrix
    Eigen::Matrix4d homogeneous = Eigen::Matrix4d::Identity();
    homogeneous.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    homogeneous.block<3, 1>(0, 3) = Eigen::Vector3d(7.0, 8.0, 9.0);

    Pose pose_from_homogeneous(homogeneous);
    std::cout << "\nPose from homogeneous transformation matrix:" << std::endl;
    printVector3d(pose_from_homogeneous.getPosition(), "Position");
    printQuaternion(pose_from_homogeneous.getQuaternion(), "Orientation");

    // Example 2: Getting different representations of a pose
    std::cout << "\n2. Getting different representations of a pose:" << std::endl;

    Pose example_pose(Eigen::Vector3d(1.0, 2.0, 3.0),
                      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ())));

    // Get individual position components
    std::cout << "Individual position components: x=" << example_pose.getX()
              << ", y=" << example_pose.getY() << ", z=" << example_pose.getZ() << std::endl;

    // Get individual quaternion components
    std::cout << "Individual quaternion components: w=" << example_pose.getQw()
              << ", x=" << example_pose.getQx() << ", y=" << example_pose.getQy()
              << ", z=" << example_pose.getQz() << std::endl;

    // Get as rotation matrix
    std::cout << "Rotation matrix:\n" << example_pose.getRotationMatrix() << std::endl;

    // Get as affine transformation
    Eigen::Affine3d example_affine = example_pose.getAffineTransformation();
    std::cout << "Affine transformation translation: [" << example_affine.translation().x() << ", "
              << example_affine.translation().y() << ", " << example_affine.translation().z() << "]"
              << std::endl;
    std::cout << "Affine transformation rotation:\n" << example_affine.rotation() << std::endl;

    // Get as homogeneous transformation matrix
    Eigen::Matrix4d example_homogeneous = example_pose.getHomogeneousT();
    printHomogeneousMatrix(example_homogeneous, "Homogeneous transformation matrix");

    // Example 3: Modifying poses
    std::cout << "\n3. Modifying poses:" << std::endl;

    Pose modifiable_pose;
    std::cout << "Initial pose:" << std::endl;
    printVector3d(modifiable_pose.getPosition(), "Position");
    printQuaternion(modifiable_pose.getQuaternion(), "Orientation");

    // Set position
    modifiable_pose.setPosition(Eigen::Vector3d(5.0, 6.0, 7.0));

    // Set orientation with quaternion
    modifiable_pose.setOrientation(
        Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY())));

    std::cout << "\nAfter modification:" << std::endl;
    printVector3d(modifiable_pose.getPosition(), "Position");
    printQuaternion(modifiable_pose.getQuaternion(), "Orientation");

    // Set orientation with rotation matrix
    Eigen::Matrix3d new_rotation =
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    modifiable_pose.setRotationMatrix(new_rotation);

    std::cout << "\nAfter setting rotation matrix:" << std::endl;
    printVector3d(modifiable_pose.getPosition(), "Position");
    std::cout << "Rotation matrix:\n" << modifiable_pose.getRotationMatrix() << std::endl;

    // Set with homogeneous transformation
    Eigen::Matrix4d new_homogeneous = Eigen::Matrix4d::Identity();
    new_homogeneous.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitX()).toRotationMatrix();
    new_homogeneous.block<3, 1>(0, 3) = Eigen::Vector3d(10.0, 11.0, 12.0);

    modifiable_pose.setHomogeneousT(new_homogeneous);

    std::cout << "\nAfter setting homogeneous transformation:" << std::endl;
    printVector3d(modifiable_pose.getPosition(), "Position");
    printQuaternion(modifiable_pose.getQuaternion(), "Orientation");

    // Example 4: Pose operations
    std::cout << "\n4. Pose operations:" << std::endl;

    // Create two poses for operations
    Pose pose1(Eigen::Vector3d(1.0, 0.0, 0.0),
               Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

    Pose pose2(Eigen::Vector3d(0.0, 1.0, 0.0),
               Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

    std::cout << "Pose 1:" << std::endl;
    printVector3d(pose1.getPosition(), "Position");
    printQuaternion(pose1.getQuaternion(), "Orientation");

    std::cout << "\nPose 2:" << std::endl;
    printVector3d(pose2.getPosition(), "Position");
    printQuaternion(pose2.getQuaternion(), "Orientation");

    // Compose poses
    Pose composed = pose1 * pose2;
    std::cout << "\nComposed pose (pose1 * pose2):" << std::endl;
    printVector3d(composed.getPosition(), "Position");
    printQuaternion(composed.getQuaternion(), "Orientation");

    // Inverse pose
    Pose inverse = pose1.inverse();
    std::cout << "\nInverse of pose1:" << std::endl;
    printVector3d(inverse.getPosition(), "Position");
    printQuaternion(inverse.getQuaternion(), "Orientation");

    // Check that pose * inverse = identity
    Pose identity_check = pose1 * inverse;
    std::cout << "\nPose1 * Inverse (should be identity):" << std::endl;
    printVector3d(identity_check.getPosition(), "Position");
    printQuaternion(identity_check.getQuaternion(), "Orientation");

    // Example 5: Distance metrics
    std::cout << "\n5. Distance metrics:" << std::endl;

    Pose pose_a(Eigen::Vector3d(1.0, 2.0, 3.0),
                Eigen::Quaterniond(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())));

    Pose pose_b(Eigen::Vector3d(4.0, 6.0, 8.0),
                Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

    double position_distance = pose_a.positionDistance(pose_b);
    double orientation_distance = pose_a.orientationDistance(pose_b);

    std::cout << "Position distance: " << position_distance << " meters" << std::endl;
    std::cout << "Orientation distance: " << orientation_distance << " radians ("
              << Rotation::rad2deg(orientation_distance) << " degrees)" << std::endl;

    // Example 6: Coordinate transformations
    std::cout << "\n6. Coordinate transformations:" << std::endl;

    // Create a pose representing a robot's pose in the world
    Pose robot_in_world(
        Eigen::Vector3d(2.0, 3.0, 0.0),                       // Position in world frame
        Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()) // 45° rotation around Z
    );

    // Create a point in the robot's local frame
    Eigen::Vector3d point_in_robot(1.0, 0.0, 0.0); // 1 meter in front of the robot

    // Transform point to world coordinates
    Eigen::Vector3d point_in_world = robot_in_world.localToGlobal(point_in_robot);

    std::cout << "Robot pose in world:" << std::endl;
    printVector3d(robot_in_world.getPosition(), "Position");
    std::cout << "Rotation: "
              << Rotation::rad2deg(Eigen::AngleAxisd(robot_in_world.getQuaternion()).angle())
              << "° around " << Eigen::AngleAxisd(robot_in_world.getQuaternion()).axis().transpose()
              << std::endl;

    std::cout << "\nPoint in robot's local frame: [" << point_in_robot.x() << ", "
              << point_in_robot.y() << ", " << point_in_robot.z() << "]" << std::endl;

    std::cout << "Same point in world frame: [" << point_in_world.x() << ", " << point_in_world.y()
              << ", " << point_in_world.z() << "]" << std::endl;

    // Transform point back to robot coordinates
    Eigen::Vector3d point_back_in_robot = robot_in_world.globalToLocal(point_in_world);

    std::cout << "Point transformed back to robot frame: [" << point_back_in_robot.x() << ", "
              << point_back_in_robot.y() << ", " << point_back_in_robot.z() << "]" << std::endl;

    // Transform between coordinate frames
    // Create a second pose representing a sensor mounted on the robot
    Pose sensor_in_robot(
        Eigen::Vector3d(0.0, 0.0, 0.5),                  // Sensor is 0.5m above the robot's origin
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) // No rotation relative to robot
    );

    // Calculate sensor pose in world frame
    Pose sensor_in_world = robot_in_world.transformPose(sensor_in_robot);

    std::cout << "\nSensor pose in robot frame:" << std::endl;
    printVector3d(sensor_in_robot.getPosition(), "Position");
    printQuaternion(sensor_in_robot.getQuaternion(), "Orientation");

    std::cout << "\nSensor pose in world frame:" << std::endl;
    printVector3d(sensor_in_world.getPosition(), "Position");
    printQuaternion(sensor_in_world.getQuaternion(), "Orientation");

    std::cout << "==============================================" << std::endl;

    return 0;
}

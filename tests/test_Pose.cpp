#define CATCH_CONFIG_MAIN
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "pose_lib.h"
#include "rotation_lib.h"

using namespace MoveG;

TEST_CASE("Pose Class Constructors", "[Pose]")
{
    SECTION("Default Constructor")
    {
        Pose pose;
        REQUIRE(pose.getPosition().isApprox(Eigen::Vector3d::Zero()));
        REQUIRE(pose.getQuaternion().coeffs().isApprox(Eigen::Quaterniond::Identity().coeffs()));
    }

    SECTION("Constructor with Position and Quaternion")
    {
        Eigen::Vector3d position(1.0, 2.0, 3.0);
        Eigen::Quaterniond orientation(0.0, 0.0, 0.0, 1.0); // Identity quaternion
        Pose pose(position, orientation);
        REQUIRE(pose.getPosition().isApprox(position));
        REQUIRE(pose.getQuaternion().coeffs().isApprox(orientation.coeffs()));
    }

    SECTION("Constructor with Position and Rotation Matrix")
    {
        Eigen::Vector3d position(4.0, 5.0, 6.0);
        Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Pose pose(position, rotation);
        REQUIRE(pose.getPosition().isApprox(position));
        REQUIRE(pose.getRotationMatrix().isApprox(rotation, 1e-6));
        REQUIRE(pose.getQuaternion().normalized().coeffs().isApprox(
            Eigen::Quaterniond(rotation).normalized().coeffs()));
    }

    SECTION("Constructor with Affine3d Transformation")
    {
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        affine.translation() = Eigen::Vector3d(7.0, 8.0, 9.0);
        affine.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
        Pose pose(affine);
        REQUIRE(pose.getPosition().isApprox(affine.translation()));
        REQUIRE(pose.getRotationMatrix().isApprox(affine.rotation(), 1e-6));
    }

    SECTION("Constructor with Position and Rotation object")
    {
        Eigen::Vector3d position(10.0, 11.0, 12.0);
        Rotation rotation =
            Rotation::fromAngleAxis(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()));
        Pose pose(position, rotation);
        REQUIRE(pose.getPosition().isApprox(position));
        REQUIRE(pose.getQuaternion().coeffs().isApprox(rotation.toQuaternion().coeffs()));
    }

    SECTION("Constructor with Homogeneous Transformation Matrix")
    {
        Eigen::Matrix4d homogeneous = Eigen::Matrix4d::Identity();
        homogeneous(0, 3) = 10.0;
        homogeneous(1, 3) = 11.0;
        homogeneous(2, 3) = 12.0;
        homogeneous.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX()).toRotationMatrix();
        Pose pose(homogeneous);
        REQUIRE(pose.getHomogeneousT().isApprox(homogeneous, 1e-6));
    }

    SECTION("Constructor with Invalid Homogeneous Transformation Matrix - Last Row")
    {
        Eigen::Matrix4d invalid_homogeneous = Eigen::Matrix4d::Identity();
        invalid_homogeneous(3, 0) = 0.1; // Invalid last row
        REQUIRE_THROWS_AS(Pose(invalid_homogeneous), std::invalid_argument);
    }

    SECTION("Constructor with Invalid Homogeneous Transformation Matrix - Non-orthogonal")
    {
        Eigen::Matrix4d invalid_homogeneous = Eigen::Matrix4d::Identity();
        invalid_homogeneous.block<3, 3>(0, 0) << 1.0, 0.1, 0.1, 0.1, 1.0, 0.1, 0.1, 0.1,
            0.5; // Non-orthogonal
        REQUIRE_THROWS_AS(Pose(invalid_homogeneous), std::invalid_argument);
    }

    SECTION("Constructor with Invalid Homogeneous Transformation Matrix - Bad Determinant")
    {
        Eigen::Matrix4d invalid_homogeneous = Eigen::Matrix4d::Identity();
        invalid_homogeneous.block<3, 3>(0, 0) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            -1.0; // Reflection, det = -1
        REQUIRE_THROWS_AS(Pose(invalid_homogeneous), std::invalid_argument);
    }

    SECTION("Copy Constructor")
    {
        Eigen::Vector3d position(13.0, 14.0, 15.0);
        Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()));
        Pose original(position, orientation);
        Pose copy(original);
        REQUIRE(copy.getPosition().isApprox(original.getPosition()));
        REQUIRE(copy.getQuaternion().coeffs().isApprox(original.getQuaternion().coeffs()));
    }

    SECTION("Move Constructor")
    {
        Eigen::Vector3d position(16.0, 17.0, 18.0);
        Eigen::Quaterniond orientation(Eigen::AngleAxisd(M_PI / 5, Eigen::Vector3d::UnitY()));
        Pose original(position, orientation);

        // Store values for later comparison
        Eigen::Vector3d original_position = original.getPosition();
        Eigen::Quaterniond original_orientation = original.getQuaternion();

        // Move construct
        Pose moved(std::move(original));

        // Verify moved pose has correct values
        REQUIRE(moved.getPosition().isApprox(original_position));
        REQUIRE(moved.getQuaternion().coeffs().isApprox(original_orientation.coeffs()));
    }
}

TEST_CASE("Pose Class Assignment Operators", "[Pose]")
{
    SECTION("Copy Assignment Operator")
    {
        Pose pose1;
        Pose pose2;
        Eigen::Vector3d new_position(13.0, 14.0, 15.0);
        Eigen::Quaterniond new_orientation(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
        pose2.setPosition(new_position);
        pose2.setOrientation(new_orientation);
        pose1 = pose2;
        REQUIRE(pose1.getPosition().isApprox(new_position));
        REQUIRE(pose1.getQuaternion().coeffs().isApprox(new_orientation.normalized().coeffs()));
    }

    SECTION("Move Assignment Operator")
    {
        Pose pose1;
        Eigen::Vector3d new_position(16.0, 17.0, 18.0);
        Eigen::Quaterniond new_orientation(Eigen::AngleAxisd(M_PI / 7, Eigen::Vector3d::UnitZ()));

        Pose pose2(new_position, new_orientation);

        // Store values for later comparison
        Eigen::Vector3d original_position = pose2.getPosition();
        Eigen::Quaterniond original_orientation = pose2.getQuaternion();

        // Move assign
        pose1 = std::move(pose2);

        // Verify pose1 has correct values
        REQUIRE(pose1.getPosition().isApprox(original_position));
        REQUIRE(pose1.getQuaternion().coeffs().isApprox(original_orientation.coeffs()));
    }

    SECTION("Self Assignment Safety")
    {
        Pose pose(Eigen::Vector3d(1.0, 2.0, 3.0),
                  Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitX())));

        Eigen::Vector3d original_position = pose.getPosition();
        Eigen::Quaterniond original_orientation = pose.getQuaternion();

        // Self assignment
        pose = pose;

        REQUIRE(pose.getPosition().isApprox(original_position));
        REQUIRE(pose.getQuaternion().coeffs().isApprox(original_orientation.coeffs()));
    }
}

TEST_CASE("Pose Class Getters and Setters", "[Pose]")
{
    Pose pose;

    SECTION("Set and Get Position")
    {
        Eigen::Vector3d new_position(16.0, 17.0, 18.0);
        pose.setPosition(new_position);
        REQUIRE(pose.getPosition().isApprox(new_position));
        REQUIRE(pose.getX() == Catch::Approx(16.0));
        REQUIRE(pose.getY() == Catch::Approx(17.0));
        REQUIRE(pose.getZ() == Catch::Approx(18.0));
    }

    SECTION("Set and Get Orientation (Quaternion)")
    {
        Eigen::Quaterniond new_orientation(Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitX()));
        pose.setOrientation(new_orientation);
        REQUIRE(pose.getQuaternion().normalized().coeffs().isApprox(
            new_orientation.normalized().coeffs()));
        REQUIRE(pose.getQx() == Catch::Approx(new_orientation.x()));
        REQUIRE(pose.getQy() == Catch::Approx(new_orientation.y()));
        REQUIRE(pose.getQz() == Catch::Approx(new_orientation.z()));
        REQUIRE(pose.getQw() == Catch::Approx(new_orientation.w()));
    }

    SECTION("Set and Get Rotation Matrix")
    {
        Eigen::Matrix3d new_rotation =
            Eigen::AngleAxisd(M_PI / 5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        pose.setRotationMatrix(new_rotation);
        REQUIRE(pose.getRotationMatrix().isApprox(new_rotation, 1e-6));
    }

    SECTION("Set and Get Affine Transformation")
    {
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        affine.translation() = Eigen::Vector3d(19.0, 20.0, 21.0);
        affine.rotate(Eigen::AngleAxisd(M_PI / 7, Eigen::Vector3d::UnitX()));
        pose.setAffineTransformation(affine);
        REQUIRE(pose.getAffineTransformation().isApprox(affine, 1e-6));
    }

    SECTION("Set and Get Homogeneous Transformation Matrix")
    {
        Eigen::Matrix4d homogeneous = Eigen::Matrix4d::Identity();
        homogeneous(0, 3) = 22.0;
        homogeneous(1, 3) = 23.0;
        homogeneous(2, 3) = 24.0;
        homogeneous.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(M_PI / 9, Eigen::Vector3d::UnitY()).toRotationMatrix();
        pose.setHomogeneousT(homogeneous);
        REQUIRE(pose.getHomogeneousT().isApprox(homogeneous, 1e-6));
    }

    SECTION("Set Invalid Homogeneous Transformation Matrix")
    {
        Eigen::Matrix4d invalid_homogeneous = Eigen::Matrix4d::Identity();
        invalid_homogeneous(3, 1) = 0.5; // Invalid last row
        REQUIRE_THROWS_AS(pose.setHomogeneousT(invalid_homogeneous), std::invalid_argument);
    }
}

TEST_CASE("Pose Class Operators and Composition", "[Pose]")
{
    Eigen::Vector3d pos1(1.0, 0.0, 0.0);
    Eigen::Quaterniond ori1(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    Pose pose1(pos1, ori1);

    Eigen::Vector3d pos2(0.0, 1.0, 0.0);
    Eigen::Quaterniond ori2(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
    Pose pose2(pos2, ori2);

    SECTION("Composition Operator (*)")
    {
        Pose composed = pose1 * pose2;

        // Expected orientation
        Eigen::Quaterniond expected_orientation = ori1 * ori2;
        expected_orientation.normalize();

        // Expected position: pos1 + ori1 * pos2
        Eigen::Vector3d expected_position = pos1 + ori1 * pos2;

        REQUIRE(composed.getQuaternion().coeffs().isApprox(expected_orientation.coeffs()));
        REQUIRE(composed.getPosition().isApprox(expected_position, 1e-6));
    }

    SECTION("Multiple Compositions")
    {
        // Create a third pose
        Eigen::Vector3d pos3(0.0, 0.0, 1.0);
        Eigen::Quaterniond ori3(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()));
        Pose pose3(pos3, ori3);

        // Test associativity: (pose1 * pose2) * pose3 == pose1 * (pose2 * pose3)
        Pose composed1 = (pose1 * pose2) * pose3;
        Pose composed2 = pose1 * (pose2 * pose3);

        REQUIRE(composed1.getPosition().isApprox(composed2.getPosition(), 1e-6));
        REQUIRE(
            composed1.getQuaternion().coeffs().isApprox(composed2.getQuaternion().coeffs(), 1e-6));
    }

    SECTION("Inverse Function")
    {
        Pose inv = pose1.inverse();

        // Compose pose with its inverse should yield identity
        Pose identity = pose1 * inv;

        REQUIRE(identity.getPosition().isApprox(Eigen::Vector3d::Zero(), 1e-6));
        REQUIRE(identity.getQuaternion().isApprox(Eigen::Quaterniond::Identity(), 1e-6));

        // Also check inverse * pose
        Pose identity2 = inv * pose1;
        REQUIRE(identity2.getPosition().isApprox(Eigen::Vector3d::Zero(), 1e-6));
        REQUIRE(identity2.getQuaternion().isApprox(Eigen::Quaterniond::Identity(), 1e-6));
    }
}

TEST_CASE("Pose Class Distance Metrics", "[Pose]")
{
    SECTION("Position Distance")
    {
        Pose pose1(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Quaterniond::Identity());
        Pose pose2(Eigen::Vector3d(4.0, 6.0, 8.0), Eigen::Quaterniond::Identity());

        double expected_distance =
            std::sqrt(std::pow(3.0, 2) + std::pow(4.0, 2) + std::pow(5.0, 2));
        REQUIRE(pose1.positionDistance(pose2) == Catch::Approx(expected_distance));

        // Distance should be commutative
        REQUIRE(pose2.positionDistance(pose1) == Catch::Approx(expected_distance));

        // Distance to self should be zero
        REQUIRE(pose1.positionDistance(pose1) == Catch::Approx(0.0));
    }

    SECTION("Orientation Distance")
    {
        // Create poses with different orientations
        Pose pose1(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

        // 90 degrees rotation around Z axis
        Pose pose2(Eigen::Vector3d::Zero(),
                   Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

        // Expected angular distance for 90 degrees is π/2 radians
        REQUIRE(pose1.orientationDistance(pose2) == Catch::Approx(M_PI / 2).epsilon(1e-6));

        // Distance should be commutative
        REQUIRE(pose2.orientationDistance(pose1) == Catch::Approx(M_PI / 2).epsilon(1e-6));

        // Distance to self should be zero
        REQUIRE(pose1.orientationDistance(pose1) == Catch::Approx(0.0).margin(1e-6));

        // Test with 180 degrees rotation
        Pose pose3(Eigen::Vector3d::Zero(),
                   Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())));
        REQUIRE(pose1.orientationDistance(pose3) == Catch::Approx(M_PI).epsilon(1e-6));
    }
}

TEST_CASE("Pose Class Coordinate Transformations", "[Pose]")
{
    SECTION("Transform Pose")
    {
        // Create a base pose
        Pose base_pose(Eigen::Vector3d(1.0, 0.0, 0.0),
                       Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

        // Create a transformation
        Pose transform(Eigen::Vector3d(0.0, 2.0, 0.0),
                       Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

        // Apply the transformation
        Pose transformed = base_pose.transformPose(transform);

        // Expected results: position at (0.0, 3.0, 0.0), orientation rotated 180 degrees around Z
        Eigen::Vector3d expected_position(0.0, 3.0, 0.0);
        Eigen::Quaterniond expected_orientation(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

        REQUIRE(transformed.getPosition().isApprox(expected_position, 1e-6));

        // For quaternions, we need to check that they represent the same rotation
        // The quaternions q and -q represent the same rotation
        double dot_product = std::abs(transformed.getQuaternion().dot(expected_orientation));
        REQUIRE(dot_product > 0.999); // Should be close to 1 or -1
    }

    SECTION("Local to Global Coordinates")
    {
        // Create a pose at position (1,2,3) with 90-degree rotation around Z
        Pose pose(Eigen::Vector3d(1.0, 2.0, 3.0),
                  Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

        // Point (1,0,0) in local frame should be (1,3,3) in global frame
        Eigen::Vector3d local_point(1.0, 0.0, 0.0);
        Eigen::Vector3d global_point = pose.localToGlobal(local_point);

        Eigen::Vector3d expected_global(1.0, 3.0, 3.0);
        REQUIRE(global_point.isApprox(expected_global, 1e-6));
    }

    SECTION("Global to Local Coordinates")
    {
        // Create a pose at position (1,2,3) with 90-degree rotation around Z
        Pose pose(Eigen::Vector3d(1.0, 2.0, 3.0),
                  Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())));

        // Point (1,3,3) in global frame should be (1,0,0) in local frame
        Eigen::Vector3d global_point(1.0, 3.0, 3.0);
        Eigen::Vector3d local_point = pose.globalToLocal(global_point);

        Eigen::Vector3d expected_local(1.0, 0.0, 0.0);
        REQUIRE(local_point.isApprox(expected_local, 1e-6));
    }

    SECTION("Round Trip Transformations")
    {
        // Create a random pose
        Eigen::Vector3d position(3.5, -2.1, 4.7);
        Eigen::Quaterniond orientation(
            Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.3, 0.5, 0.8).normalized()));
        Pose pose(position, orientation);

        // Create a random point
        Eigen::Vector3d original_local(1.2, -3.4, 5.6);

        // Convert local to global and back
        Eigen::Vector3d global = pose.localToGlobal(original_local);
        Eigen::Vector3d round_trip_local = pose.globalToLocal(global);

        // Should get the original point back
        REQUIRE(round_trip_local.isApprox(original_local, 1e-6));
    }
}

TEST_CASE("Pose Class Stream Operator", "[Pose]")
{
    Eigen::Vector3d position(3.0, 4.0, 5.0);
    Eigen::Quaterniond orientation(Eigen::Quaterniond::Identity());
    Pose pose(position, orientation);

    std::ostringstream oss;
    oss << pose;

    // Check that the output contains the position and orientation values
    std::string output = oss.str();
    REQUIRE(output.find("Position: [3") != std::string::npos);
    REQUIRE(output.find("4") != std::string::npos);
    REQUIRE(output.find("5") != std::string::npos);
    REQUIRE(output.find("Orientation") != std::string::npos);
    REQUIRE(output.find("0, 0, 0, 1") != std::string::npos);
}

TEST_CASE("Pose Class Edge Cases", "[Pose]")
{
    SECTION("Normalization of Quaternion in Constructor")
    {
        Eigen::Vector3d position(6.0, 7.0, 8.0);
        Eigen::Quaterniond orientation(2.0, 0.0, 0.0, 0.0); // Non-normalized
        Pose pose(position, orientation);

        // Check that the quaternion was normalized
        REQUIRE(pose.getQuaternion().norm() == Catch::Approx(1.0).epsilon(1e-6));
        REQUIRE(pose.getQuaternion().normalized().isApprox(pose.getQuaternion(), 1e-6));
    }

    SECTION("Normalization of Quaternion in Setters")
    {
        Pose pose;
        Eigen::Quaterniond non_normalized(0.0, 0.0, 0.0, 2.0);
        pose.setOrientation(non_normalized);

        // Check that the quaternion was normalized
        REQUIRE(pose.getQuaternion().norm() == Catch::Approx(1.0).epsilon(1e-6));
        REQUIRE(pose.getQuaternion().isApprox(non_normalized.normalized(), 1e-6));

        Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(M_PI / 10, Eigen::Vector3d::UnitX()).toRotationMatrix();
        pose.setRotationMatrix(rotation);

        // Check that the quaternion was normalized
        REQUIRE(pose.getQuaternion().norm() == Catch::Approx(1.0).epsilon(1e-6));
        REQUIRE(pose.getQuaternion().isApprox(Eigen::Quaterniond(rotation).normalized(), 1e-6));
    }

    SECTION("Zero Vector Handling")
    {
        // Test with a zero vector for position
        Pose pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
        REQUIRE(pose.getPosition().isApprox(Eigen::Vector3d::Zero()));

        // Test transform with zero vector
        Eigen::Vector3d local_zero = Eigen::Vector3d::Zero();
        Eigen::Vector3d global_zero = pose.localToGlobal(local_zero);
        REQUIRE(global_zero.isApprox(Eigen::Vector3d::Zero()));
    }
}

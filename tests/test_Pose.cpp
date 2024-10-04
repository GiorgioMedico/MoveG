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
        REQUIRE(pose.getQaternion().coeffs().isApprox(Eigen::Quaterniond::Identity().coeffs()));
    }

    SECTION("Constructor with Position and Quaternion")
    {
        Eigen::Vector3d position(1.0, 2.0, 3.0);
        Eigen::Quaterniond orientation(0.0, 0.0, 0.0, 1.0); // Identity quaternion
        Pose pose(position, orientation);
        REQUIRE(pose.getPosition() == position);
        REQUIRE(pose.getQaternion().coeffs() == orientation.coeffs());
    }

    SECTION("Constructor with Position and Rotation Matrix")
    {
        Eigen::Vector3d position(4.0, 5.0, 6.0);
        Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Pose pose(position, rotation);
        REQUIRE(pose.getPosition() == position);
        REQUIRE(pose.getRotationMatrix().isApprox(rotation, 1e-6));
        REQUIRE(pose.getQaternion().normalized().coeffs() ==
                Eigen::Quaterniond(rotation).normalized().coeffs());
    }

    SECTION("Constructor with Affine3d Transformation")
    {
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        affine.translation() = Eigen::Vector3d(7.0, 8.0, 9.0);
        affine.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));
        Pose pose(affine);
        REQUIRE(pose.getPosition() == affine.translation());
        REQUIRE(pose.getRotationMatrix().isApprox(affine.rotation(), 1e-6));
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

    SECTION("Constructor with Invalid Homogeneous Transformation Matrix")
    {
        Eigen::Matrix4d invalid_homogeneous = Eigen::Matrix4d::Identity();
        invalid_homogeneous(3, 0) = 0.1; // Invalid last row
        REQUIRE_THROWS_AS(Pose(invalid_homogeneous), std::invalid_argument);
    }

    SECTION("Copy Constructor")
    {
        Pose original;
        Pose copy(original);
        REQUIRE(copy.getPosition() == original.getPosition());
        REQUIRE(copy.getQaternion().coeffs() == original.getQaternion().coeffs());
    }

    SECTION("Copy Assignment Operator")
    {
        Pose pose1;
        Pose pose2;
        Eigen::Vector3d new_position(13.0, 14.0, 15.0);
        Eigen::Quaterniond new_orientation(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
        pose2.setPosition(new_position);
        pose2.setOrientation(new_orientation);
        pose1 = pose2;
        REQUIRE(pose1.getPosition() == new_position);
        REQUIRE(pose1.getQaternion().coeffs() == new_orientation.normalized().coeffs());
    }
}

TEST_CASE("Pose Class Getters and Setters", "[Pose]")
{
    Pose pose;

    SECTION("Set and Get Position")
    {
        Eigen::Vector3d new_position(16.0, 17.0, 18.0);
        pose.setPosition(new_position);
        REQUIRE(pose.getPosition() == new_position);
        REQUIRE(pose.getX() == Catch::Approx(16.0));
        REQUIRE(pose.getY() == Catch::Approx(17.0));
        REQUIRE(pose.getZ() == Catch::Approx(18.0));
    }

    SECTION("Set and Get Orientation (Quaternion)")
    {
        Eigen::Quaterniond new_orientation(Eigen::AngleAxisd(M_PI / 8, Eigen::Vector3d::UnitX()));
        pose.setOrientation(new_orientation);
        REQUIRE(pose.getQaternion().normalized().coeffs() == new_orientation.normalized().coeffs());
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
}

TEST_CASE("Pose Class Operators", "[Pose]")
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

        REQUIRE(composed.getQaternion().coeffs() == expected_orientation.coeffs());
        REQUIRE(composed.getPosition().isApprox(expected_position, 1e-6));
    }

    SECTION("Inverse Function")
    {
        Pose inv = pose1.inverse();

        // Compose pose with its inverse should yield identity
        Pose identity = pose1 * inv;

        REQUIRE(identity.getPosition().isApprox(Eigen::Vector3d::Zero(), 1e-6));
        REQUIRE(identity.getQaternion().isApprox(Eigen::Quaterniond::Identity(), 1e-6));
    }
}

TEST_CASE("Pose Class Stream Operator", "[Pose]")
{
    Eigen::Vector3d position(3.0, 4.0, 5.0);
    Eigen::Quaterniond orientation(Eigen::Quaterniond::Identity());
    Pose pose(position, orientation);

    std::ostringstream oss;
    oss << pose;

    std::string expected_output = "Position: [3, 4, 5]\nOrientation (quaternion): [0, 0, 0, 1]";
    REQUIRE(oss.str() == expected_output);
}

TEST_CASE("Pose Class Additional Functionalities", "[Pose]")
{
    SECTION("Normalization of Quaternion in Constructor")
    {
        Eigen::Vector3d position(6.0, 7.0, 8.0);
        Eigen::Quaterniond orientation(2.0, 0.0, 0.0, 0.0); // Non-normalized
        Pose pose(position, orientation);
        REQUIRE(pose.getQaternion().normalized().isApprox(pose.getQaternion(), 1e-6));
    }

    SECTION("Normalization of Quaternion in Setters")
    {
        Pose pose;
        Eigen::Quaterniond non_normalized(0.0, 0.0, 0.0, 2.0);
        pose.setOrientation(non_normalized);
        REQUIRE(pose.getQaternion().isApprox(non_normalized.normalized(), 1e-6));

        Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(M_PI / 10, Eigen::Vector3d::UnitX()).toRotationMatrix();
        pose.setRotationMatrix(rotation);
        REQUIRE(pose.getQaternion().isApprox(Eigen::Quaterniond(rotation).normalized(), 1e-6));
    }
}

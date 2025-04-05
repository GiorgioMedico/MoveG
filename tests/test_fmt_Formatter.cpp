#define CATCH_CONFIG_MAIN
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>
#include <fmt/core.h>
#include <fmt/format.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "utils/fmt_formatter.h"

TEST_CASE("Vector Formatter", "[fmt_formatter][vector]")
{
    SECTION("Vector3d")
    {
        Eigen::Vector3d vec(1.0, 2.0, 3.0);
        std::string formatted = fmt::format("{}", vec);
        REQUIRE(formatted == "[1, 2, 3]");
    }

    SECTION("Vector2d")
    {
        Eigen::Vector2d vec(4.5, -6.7);
        std::string formatted = fmt::format("{}", vec);
        REQUIRE(formatted == "[4.5, -6.7]");
    }

    SECTION("Vector4d")
    {
        Eigen::Vector4d vec(1.0, 2.0, 3.0, 4.0);
        std::string formatted = fmt::format("{}", vec);
        REQUIRE(formatted == "[1, 2, 3, 4]");
    }

    SECTION("VectorXd")
    {
        Eigen::VectorXd vec(5);
        vec << 1.0, 2.0, 3.0, 4.0, 5.0;
        std::string formatted = fmt::format("{}", vec);
        REQUIRE(formatted == "[1, 2, 3, 4, 5]");
    }

    SECTION("Small/Large Values")
    {
        Eigen::Vector3d vec(1e-9, 1e9, M_PI);
        std::string formatted = fmt::format("{}", vec);
        REQUIRE(formatted.find("1e-09") != std::string::npos);
        REQUIRE(formatted.find("1e+09") != std::string::npos);
        REQUIRE(formatted.find("3.14159") != std::string::npos);
    }
}

TEST_CASE("Matrix Formatter", "[fmt_formatter][matrix]")
{
    SECTION("Matrix2d")
    {
        Eigen::Matrix2d mat;
        mat << 1.0, 2.0, 3.0, 4.0;
        std::string formatted = fmt::format("{}", mat);
        // Check that the formatted string contains the matrix elements and proper formatting
        REQUIRE(formatted.find("⎡") != std::string::npos); // Top-left corner
        REQUIRE(formatted.find("⎤") != std::string::npos); // Top-right corner
        REQUIRE(formatted.find("⎣") != std::string::npos); // Bottom-left corner
        REQUIRE(formatted.find("⎦") != std::string::npos); // Bottom-right corner
        REQUIRE(formatted.find("1.0000") != std::string::npos);
        REQUIRE(formatted.find("2.0000") != std::string::npos);
        REQUIRE(formatted.find("3.0000") != std::string::npos);
        REQUIRE(formatted.find("4.0000") != std::string::npos);
    }

    SECTION("Matrix3d")
    {
        Eigen::Matrix3d mat;
        mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
        std::string formatted = fmt::format("{}", mat);
        // Check that it's formatted as a matrix with the correct number of rows
        int rows = 0;
        for (char c : formatted)
        {
            if (c == '\n')
                rows++;
        }
        REQUIRE(rows == 2); // A 3x3 matrix should have 2 newlines (3 rows)
        REQUIRE(formatted.find("1.0000") != std::string::npos);
        REQUIRE(formatted.find("5.0000") != std::string::npos);
        REQUIRE(formatted.find("9.0000") != std::string::npos);
    }

    SECTION("MatrixXd")
    {
        Eigen::MatrixXd mat(2, 3);
        mat << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6;
        std::string formatted = fmt::format("{}", mat);
        REQUIRE(formatted.find("1.1000") != std::string::npos);
        REQUIRE(formatted.find("3.3000") != std::string::npos);
        REQUIRE(formatted.find("6.6000") != std::string::npos);
    }
}

TEST_CASE("Quaternion Formatter", "[fmt_formatter][quaternion]")
{
    SECTION("Identity Quaternion")
    {
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        std::string formatted = fmt::format("{}", q);
        REQUIRE(formatted == "[w=1, x=0, y=0, z=0]");
    }

    SECTION("Rotation Quaternion")
    {
        // 90-degree rotation around Z axis
        Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
        std::string formatted = fmt::format("{}", q);
        // Check that the components are formatted correctly
        REQUIRE(formatted.find("w=0.7071") != std::string::npos);
        REQUIRE(formatted.find("x=0") != std::string::npos);
        REQUIRE(formatted.find("y=0") != std::string::npos);
        REQUIRE(formatted.find("z=0.7071") != std::string::npos);
    }

    SECTION("Non-Normalized Quaternion")
    {
        Eigen::Quaterniond q(2.0, 1.0, 0.5, 0.1);
        std::string formatted = fmt::format("{}", q);
        REQUIRE(formatted.find("w=2") != std::string::npos);
        REQUIRE(formatted.find("x=1") != std::string::npos);
        REQUIRE(formatted.find("y=0.5") != std::string::npos);
        REQUIRE(formatted.find("z=0.1") != std::string::npos);
    }
}

TEST_CASE("Affine Transformation Formatter", "[fmt_formatter][affine]")
{
    SECTION("Identity Transformation")
    {
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        std::string formatted = fmt::format("{}", affine);
        // Should be formatted as a 4x4 homogeneous matrix
        REQUIRE(formatted.find("1.0000") != std::string::npos);
        REQUIRE(formatted.find("0.0000") != std::string::npos);
    }

    SECTION("Translation")
    {
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        affine.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
        std::string formatted = fmt::format("{}", affine);
        REQUIRE(formatted.find("1.0000") != std::string::npos); // Translation components
        REQUIRE(formatted.find("2.0000") != std::string::npos);
        REQUIRE(formatted.find("3.0000") != std::string::npos);
    }

    SECTION("Rotation and Translation")
    {
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        affine.translate(Eigen::Vector3d(4.0, 5.0, 6.0));
        affine.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
        std::string formatted = fmt::format("{}", affine);
        // Check that rotation and translation components are present
        REQUIRE(formatted.find("0.0000") != std::string::npos); // Rotation zeros
        REQUIRE(formatted.find("1.0000") != std::string::npos); // Rotation ones
        REQUIRE(formatted.find("4.0000") != std::string::npos); // Translation
        REQUIRE(formatted.find("5.0000") != std::string::npos);
        REQUIRE(formatted.find("6.0000") != std::string::npos);
    }
}

TEST_CASE("AngleAxis Formatter", "[fmt_formatter][angle_axis]")
{
    SECTION("Zero Rotation")
    {
        Eigen::AngleAxisd aa(0.0, Eigen::Vector3d::UnitX());
        std::string formatted = fmt::format("{}", aa);
        REQUIRE(formatted.find("angle=0") != std::string::npos);
        REQUIRE(formatted.find("axis=[1") != std::string::npos);
    }

    SECTION("90-Degree Rotation")
    {
        Eigen::AngleAxisd aa(M_PI / 2, Eigen::Vector3d::UnitY());
        std::string formatted = fmt::format("{}", aa);
        REQUIRE(formatted.find("angle=90") != std::string::npos); // Degrees
        REQUIRE(formatted.find("axis=[0, 1, 0]") != std::string::npos);
    }

    SECTION("Arbitrary Axis")
    {
        Eigen::Vector3d axis(1.0, 2.0, 3.0);
        axis.normalize();
        Eigen::AngleAxisd aa(M_PI / 4, axis);
        std::string formatted = fmt::format("{}", aa);
        REQUIRE(formatted.find("angle=45") != std::string::npos); // 45 degrees
        REQUIRE(formatted.find("axis=[") != std::string::npos);
        // Normalized axis components should be present
        REQUIRE(formatted.find(fmt::format("{:.6g}", axis.x())) != std::string::npos);
        REQUIRE(formatted.find(fmt::format("{:.6g}", axis.y())) != std::string::npos);
        REQUIRE(formatted.find(fmt::format("{:.6g}", axis.z())) != std::string::npos);
    }
}

TEST_CASE("Formatter Integration", "[fmt_formatter][integration]")
{
    SECTION("Compound Formatting")
    {
        Eigen::Vector3d position(1.0, 2.0, 3.0);
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

        // Format two objects together
        std::string formatted = fmt::format("Position: {}, Orientation: {}", position, orientation);
        REQUIRE(formatted.find("Position: [1, 2, 3]") != std::string::npos);
        REQUIRE(formatted.find("Orientation: [w=1, x=0, y=0, z=0]") != std::string::npos);
    }

    SECTION("Format with Other Types")
    {
        Eigen::Matrix2d mat;
        mat << 1.0, 2.0, 3.0, 4.0;

        std::string formatted = fmt::format("Matrix: {}, Value: {}", mat, 42);
        REQUIRE(formatted.find("Matrix: ") != std::string::npos);
        REQUIRE(formatted.find("Value: 42") != std::string::npos);
    }
}

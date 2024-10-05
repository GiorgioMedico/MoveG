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

#include "rotation_lib.h"

using namespace MoveG;

// Funzione per generare un angolo casuale in un intervallo specificato
double randomAngle(double min, double max, std::mt19937 &gen)
{
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

TEST_CASE("Rotation: Identity", "[rotation]")
{
    Rotation rot;
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    REQUIRE(rot.toRotationMatrix().isApprox(identity));
}

TEST_CASE("Rotation: From Rotation Matrix", "[rotation]")
{
    Eigen::AngleAxisd aa(M_PI / 4, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot_matrix = aa.toRotationMatrix();
    Rotation rot(rot_matrix);
    REQUIRE(rot.toRotationMatrix().isApprox(rot_matrix));
}

TEST_CASE("Rotation: From Quaternion", "[rotation]")
{
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()));
    Rotation rot(q);
    REQUIRE(rot.toQuaternion().coeffs().isApprox(q.normalized().coeffs()));
}

TEST_CASE("Rotation: From Angle-Axis", "[rotation]")
{
    Eigen::AngleAxisd aa(M_PI / 6, Eigen::Vector3d::UnitX());
    Rotation rot(aa);
    REQUIRE(rot.toAngleAxis().angle() == Catch::Approx(M_PI / 6));
    REQUIRE(rot.toAngleAxis().axis().isApprox(Eigen::Vector3d::UnitX()));
}

TEST_CASE("Rotation: From Euler Angles Intrinsic - Randomized", "[rotation][random]")
{
    // Inizializza il generatore di numeri casuali con un seme fisso per la ripetibilità
    std::mt19937 gen(42);

    // Definisci gli intervalli per gli angoli (evitando gimbal lock per l'angolo di pitch)
    double min_yaw = -M_PI;
    double max_yaw = M_PI;
    double min_pitch = -M_PI / 3; // -60 gradi
    double max_pitch = M_PI / 3;  // +60 gradi
    double min_roll = -M_PI;
    double max_roll = M_PI;

    std::string sequence = "ZYX";

    // Esegui più iterazioni per testare diversi set di angoli
    const int num_tests = 100;
    for (int i = 0; i < num_tests; ++i)
    {
        double angle1 = randomAngle(min_yaw, max_yaw, gen);     // Yaw
        double angle2 = randomAngle(min_pitch, max_pitch, gen); // Pitch
        double angle3 = randomAngle(min_roll, max_roll, gen);   // Roll

        Rotation rot(angle1, angle2, angle3, true, sequence, false);

        // Per rotazioni intrinseche, moltiplica nell'ordine inverso
        Eigen::Matrix3d expected_matrix =
            Eigen::AngleAxisd(angle3, Eigen::Vector3d::UnitX()).toRotationMatrix() *
            Eigen::AngleAxisd(angle2, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(angle1, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        REQUIRE(rot.toRotationMatrix().isApprox(expected_matrix, 1e-6));
    }
}

TEST_CASE("Rotation: From Euler Angles Extrinsic - Randomized", "[rotation][random]")
{
    // Inizializza il generatore di numeri casuali con un seme fisso per la ripetibilità
    std::mt19937 gen(42);

    // Definisci gli intervalli per gli angoli (evitando gimbal lock per l'angolo di pitch)
    double min_yaw = -M_PI;
    double max_yaw = M_PI;
    double min_pitch = -M_PI / 3; // -60 gradi
    double max_pitch = M_PI / 3;  // +60 gradi
    double min_roll = -M_PI;
    double max_roll = M_PI;

    std::string sequence = "ZYX";

    // Esegui più iterazioni per testare diversi set di angoli
    const int num_tests = 100;
    for (int i = 0; i < num_tests; ++i)
    {
        double angle1 = randomAngle(min_yaw, max_yaw, gen);     // Yaw
        double angle2 = randomAngle(min_pitch, max_pitch, gen); // Pitch
        double angle3 = randomAngle(min_roll, max_roll, gen);   // Roll

        Rotation rot(angle1, angle2, angle3, false, sequence, false);

        // Per rotazioni estrinseche, moltiplica nell'ordine della sequenza
        Eigen::Matrix3d expected_matrix =
            Eigen::AngleAxisd(angle1, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(angle2, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(angle3, Eigen::Vector3d::UnitX()).toRotationMatrix();

        REQUIRE(rot.toRotationMatrix().isApprox(expected_matrix, 1e-6));
    }
}

TEST_CASE("Rotation: Conversion to Euler Angles Intrinsic - Randomized", "[rotation][random]")
{
    // Inizializza il generatore di numeri casuali con un seme fisso per la ripetibilità
    std::mt19937 gen(42);

    // Definisci gli intervalli per gli angoli (evitando gimbal lock per l'angolo di pitch)
    double min_yaw = -M_PI;
    double max_yaw = M_PI;
    double min_pitch = -M_PI / 3; // -60 gradi
    double max_pitch = M_PI / 3;  // +60 gradi
    double min_roll = -M_PI;
    double max_roll = M_PI;

    std::string sequence = "ZYX";

    // Esegui più iterazioni per testare diversi set di angoli
    const int num_tests = 100;
    for (int i = 0; i < num_tests; ++i)
    {
        double angle1 = randomAngle(min_yaw, max_yaw, gen);     // Yaw
        double angle2 = randomAngle(min_pitch, max_pitch, gen); // Pitch
        double angle3 = randomAngle(min_roll, max_roll, gen);   // Roll

        Rotation rot(angle1, angle2, angle3, true, sequence, false);

        // Estrai gli angoli di Eulero
        Eigen::Vector3d euler_angles = rot.toEulerAngles(true, sequence);

        // Ricostruisci la rotazione dagli angoli estratti
        Rotation rot_reconstructed(euler_angles[0],
                                   euler_angles[1],
                                   euler_angles[2],
                                   true,
                                   sequence,
                                   false);

        // Confronta la matrice di rotazione ricostruita con l'originale
        REQUIRE(rot.toRotationMatrix().isApprox(rot_reconstructed.toRotationMatrix(), 1e-6));
    }
}

TEST_CASE("Rotation: Conversion to Euler Angles Extrinsic - Randomized", "[rotation][random]")
{
    // Inizializza il generatore di numeri casuali con un seme fisso per la ripetibilità
    std::mt19937 gen(42);

    // Definisci gli intervalli per gli angoli (evitando gimbal lock per l'angolo di pitch)
    double min_yaw = -M_PI;
    double max_yaw = M_PI;
    double min_pitch = -M_PI / 3; // -60 gradi
    double max_pitch = M_PI / 3;  // +60 gradi
    double min_roll = -M_PI;
    double max_roll = M_PI;

    std::string sequence = "ZYX";

    // Esegui più iterazioni per testare diversi set di angoli
    const int num_tests = 100;
    for (int i = 0; i < num_tests; ++i)
    {
        double angle1 = randomAngle(min_yaw, max_yaw, gen);     // Yaw
        double angle2 = randomAngle(min_pitch, max_pitch, gen); // Pitch
        double angle3 = randomAngle(min_roll, max_roll, gen);   // Roll

        Rotation rot(angle1, angle2, angle3, false, sequence, false);

        // Estrai gli angoli di Eulero
        Eigen::Vector3d euler_angles = rot.toEulerAngles(false, sequence);

        // Ricostruisci la rotazione dagli angoli estratti
        Rotation rot_reconstructed(euler_angles[0],
                                   euler_angles[1],
                                   euler_angles[2],
                                   false,
                                   sequence,
                                   false);

        // Confronta la matrice di rotazione ricostruita con l'originale
        REQUIRE(rot.toRotationMatrix().isApprox(rot_reconstructed.toRotationMatrix(), 1e-6));
    }
}

TEST_CASE("Rotation: Operator Overloading", "[rotation]")
{
    Rotation rot1(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));
    Rotation rot2(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
    Rotation rot_combined = rot1 * rot2;

    Eigen::Matrix3d expected_matrix = rot1.toRotationMatrix() * rot2.toRotationMatrix();

    REQUIRE(rot_combined.toRotationMatrix().isApprox(expected_matrix));
}

TEST_CASE("Rotation: Exception Handling", "[rotation]")
{
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "ABCD"), std::invalid_argument);
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "X1Z"), std::invalid_argument);
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "X"), std::invalid_argument);
}

TEST_CASE("Rotation: Print", "[rotation]")
{
    Eigen::Quaterniond q1(1, 0, 0, 0); // Identità
    Eigen::Quaterniond q2(0, 1, 0, 0); // 180 gradi attorno all'asse X

    // Differenza tra q1 e q2
    Eigen::Quaterniond q_diff = Rotation::quaternion_difference(q1, q2);
    std::cout << "Differenza Quaternion: " << q_diff.coeffs().transpose() << std::endl;

    // Negazione di q2
    Eigen::Quaterniond q_neg = Rotation::quaternion_negation(q2);
    std::cout << "Negazione Quaternion: " << q_neg.coeffs().transpose() << std::endl;

    // Prodotto scalare di q2 per 2
    Eigen::Quaterniond q_scaled = Rotation::scalar_product(q2, 2.0);
    std::cout << "Prodotto Scalari Quaternion: " << q_scaled.coeffs().transpose() << std::endl;

    // Somma di q1 e q2
    Eigen::Quaterniond q_sum = Rotation::quaternion_plus(q1, q2);
    std::cout << "Somma Quaternion: " << q_sum.coeffs().transpose() << std::endl;

    // Differenza di q1 e q2
    Eigen::Quaterniond q_sub = Rotation::quaternion_minus(q1, q2);
    std::cout << "Sottrazione Quaternion: " << q_sub.coeffs().transpose() << std::endl;
}

TEST_CASE("Rotation: Matrix T", "[rotation]")
{
    Eigen::Vector3d angles(M_PI / 4, M_PI / 6, M_PI / 3);
    std::string sequence = "ZYX";

    Eigen::Matrix3d T = Rotation::matrixT(angles, sequence);

    // Calcola la matrice T manualmente
    Eigen::Matrix3d T_manual;
    T_manual << 0, -sin(angles[0]), cos(angles[0]) * cos(angles[1]), 0, cos(angles[0]),
        sin(angles[0]) * cos(angles[1]), 1, 0, -sin(angles[1]);

    // std::cout << "T:\n" << T << std::endl;
    // std::cout << "T_manual:\n" << T_manual << std::endl;

    REQUIRE(T.isApprox(T_manual));

    sequence = "ZYZ";
    T = Rotation::matrixT(angles, sequence);
    T_manual << 0, -sin(angles[0]), cos(angles[0]) * sin(angles[1]), 0, cos(angles[0]),
        sin(angles[0]) * sin(angles[1]), 1, 0, cos(angles[1]);

    REQUIRE(T.isApprox(T_manual));
}

TEST_CASE("Rotation: Copy Constructor", "[rotation][copy]")
{
    Rotation rot1(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitX()));
    Rotation rot2(rot1); // Use copy constructor

    REQUIRE(rot2.toRotationMatrix().isApprox(rot1.toRotationMatrix()));
}

TEST_CASE("Rotation: Assignment Operator", "[rotation][assignment]")
{
    Rotation rot1(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()));
    Rotation rot2;
    rot2 = rot1; // Use assignment operator

    REQUIRE(rot2.toRotationMatrix().isApprox(rot1.toRotationMatrix()));
}

TEST_CASE("Rotation: Degree-Radian Conversion", "[rotation][conversion]")
{
    // Test cases: degrees to radians and back
    std::vector<double> degrees = {0.0, 90.0, 180.0, 270.0, 360.0};
    for (double deg : degrees)
    {
        double rad = Rotation::deg2rad(deg);
        double deg_converted = Rotation::rad2deg(rad);
        REQUIRE(deg_converted == Catch::Approx(deg).margin(1e-9));
    }

    // Test cases: radians to degrees and back
    std::vector<double> radians = {0.0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};
    for (double rad : radians)
    {
        double deg = Rotation::rad2deg(rad);
        double rad_converted = Rotation::deg2rad(deg);
        REQUIRE(rad_converted == Catch::Approx(rad).margin(1e-9));
    }
}

TEST_CASE("Rotation: Quaternion Operations", "[rotation][quaternion]")
{
    Eigen::Quaterniond q1(1, 0, 0, 0); // Identity quaternion
    Eigen::Quaterniond q2(0, 1, 0, 0); // 180 degrees around X-axis

    // Quaternion Difference: q1^{-1} * q2
    Eigen::Quaterniond q_diff = Rotation::quaternion_difference(q1, q2);
    Eigen::Quaterniond expected_diff = q1.inverse() * q2;
    REQUIRE(q_diff.coeffs().isApprox(expected_diff.coeffs(), 1e-6));

    // Quaternion Negation: -q2
    Eigen::Quaterniond q_neg = Rotation::quaternion_negation(q2);
    Eigen::Quaterniond expected_neg(-q2.w(), -q2.x(), -q2.y(), -q2.z());
    REQUIRE(q_neg.coeffs().isApprox(expected_neg.coeffs(), 1e-6));

    // Scalar Product: q2 * 2
    Eigen::Quaterniond q_scaled = Rotation::scalar_product(q2, 2.0);
    Eigen::Quaterniond expected_scaled(0, 2, 0, 0);
    REQUIRE(q_scaled.coeffs().isApprox(expected_scaled.coeffs(), 1e-6));

    // Quaternion Addition: q1 + q2
    Eigen::Quaterniond q_sum = Rotation::quaternion_plus(q1, q2);
    Eigen::Quaterniond expected_sum(1, 1, 0, 0);
    REQUIRE(q_sum.coeffs().isApprox(expected_sum.coeffs(), 1e-6));

    // Quaternion Subtraction: q1 - q2
    Eigen::Quaterniond q_sub = Rotation::quaternion_minus(q1, q2);
    Eigen::Quaterniond expected_sub(1, -1, 0, 0);
    REQUIRE(q_sub.coeffs().isApprox(expected_sub.coeffs(), 1e-6));
}

TEST_CASE("Rotation: Elementary Rotation Matrices", "[rotation][matrices]")
{
    double angle = M_PI / 4; // 45 degrees

    // Rotation around X-axis
    Eigen::Matrix3d rotX = Rotation::rotationX(angle);
    Eigen::Matrix3d expectedRotX;
    expectedRotX << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);
    REQUIRE(rotX.isApprox(expectedRotX, 1e-6));

    // Rotation around Y-axis
    Eigen::Matrix3d rotY = Rotation::rotationY(angle);
    Eigen::Matrix3d expectedRotY;
    expectedRotY << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle);
    REQUIRE(rotY.isApprox(expectedRotY, 1e-6));

    // Rotation around Z-axis
    Eigen::Matrix3d rotZ = Rotation::rotationZ(angle);
    Eigen::Matrix3d expectedRotZ;
    expectedRotZ << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;
    REQUIRE(rotZ.isApprox(expectedRotZ, 1e-6));
}

TEST_CASE("Rotation: Matrix S and Matrix R_dot", "[rotation][matrix_operations]")
{
    Eigen::Vector3d omega(1.0, 2.0, 3.0);
    Eigen::Matrix3d S = Rotation::matrixS(omega);
    Eigen::Matrix3d expectedS;
    expectedS << 0, -omega.z(), omega.y(), omega.z(), 0, -omega.x(), -omega.y(), omega.x(), 0;
    REQUIRE(S.isApprox(expectedS, 1e-6));

    Eigen::Matrix3d R = Rotation::rotationX(M_PI / 6);
    Eigen::Matrix3d R_dot = Rotation::matrixR_dot(R, S);
    Eigen::Matrix3d expectedR_dot = S * R;
    REQUIRE(R_dot.isApprox(expectedR_dot, 1e-6));

    // Test the overloaded matrixR_dot with Vector3d
    Eigen::Matrix3d R_dot_vec = Rotation::matrixR_dot(R, omega);
    REQUIRE(R_dot_vec.isApprox(expectedR_dot, 1e-6));
}

TEST_CASE("Rotation: Invalid Sequences", "[rotation][exception]")
{
    // Invalid sequence lengths
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "XY"), std::invalid_argument);
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "XYZW"), std::invalid_argument);

    // Invalid characters in sequence
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "ABCD"), std::invalid_argument);
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "X1Z"), std::invalid_argument);
    REQUIRE_THROWS_AS(Rotation::fromEulerAngles(0, 0, 0, true, "X@Z"), std::invalid_argument);
}

TEST_CASE("Rotation: Gimbal Lock Scenario", "[rotation][gimbal_lock]")
{
    // Gimbal lock occurs when pitch is +/-90 degrees (or pi/2 radians)
    // However, your current pitch limits are -60 to +60 degrees to avoid gimbal lock
    // Here, we'll test what happens when angles are near the limits

    double near_gimbal_pitch = M_PI / 3 - 1e-6; // Just below 60 degrees
    Rotation rot(0.0, near_gimbal_pitch, 0.0, true, "ZYX", false);

    // Ensure that toEulerAngles does not throw and returns expected values
    Eigen::Vector3d euler = rot.toEulerAngles(true, "ZYX");
    REQUIRE(euler[1] == Catch::Approx(near_gimbal_pitch).margin(1e-6));
}

TEST_CASE("Rotation: Euler Angles with 360 Degrees", "[rotation][euler_angles]")
{
    // Rotations with angles equal to 360 degrees should result in identity
    Rotation rot(Eigen::AngleAxisd(2 * M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    REQUIRE(rot.toRotationMatrix().isApprox(identity, 1e-6));

    // Similarly, using Euler angles
    Rotation rot_euler(360.0, 0.0, 0.0, true, "ZYX", true); // Degrees
    REQUIRE(rot_euler.toRotationMatrix().isApprox(identity, 1e-6));
}

TEST_CASE("Rotation: Quaternion Normalization", "[rotation][quaternion]")
{
    Eigen::Quaterniond q_unormalized(2.0, 0.0, 0.0, 0.0); // Non-unit quaternion
    Rotation rot(q_unormalized);

    Eigen::Quaterniond q_normalized = q_unormalized.normalized();
    REQUIRE(rot.toQuaternion().coeffs().isApprox(q_normalized.coeffs(), 1e-6));
}

TEST_CASE("Rotation: Euler Angles Edge Cases", "[rotation][euler_angles]")
{
    // All angles zero
    Rotation rot_zero(0.0, 0.0, 0.0, true, "ZYX", false);
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    REQUIRE(rot_zero.toRotationMatrix().isApprox(identity, 1e-6));

    // Negative angles
    Rotation rot_negative(-M_PI / 4, -M_PI / 6, -M_PI / 3, true, "ZYX", false);
    Eigen::Matrix3d expected_neg =
        Eigen::AngleAxisd(-M_PI / 3, Eigen::Vector3d::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    REQUIRE(rot_negative.toRotationMatrix().isApprox(expected_neg, 1e-6));
}

TEST_CASE("Rotation: To Angle-Axis Conversion", "[rotation][angle_axis]")
{
    // 90 degrees around Y-axis
    Eigen::AngleAxisd aa(M_PI / 2, Eigen::Vector3d::UnitY());
    Rotation rot(aa);
    Eigen::AngleAxisd rot_aa = rot.toAngleAxis();

    REQUIRE(rot_aa.angle() == Catch::Approx(aa.angle()).margin(1e-6));
    REQUIRE(rot_aa.axis().isApprox(aa.axis(), 1e-6));

    // Zero rotation (identity)
    Rotation rot_identity;
    Eigen::AngleAxisd aa_identity = rot_identity.toAngleAxis();
    REQUIRE(aa_identity.angle() == Catch::Approx(0.0).margin(1e-6));
    // The axis can be arbitrary when angle is zero; Eigen defaults to (1,0,0)
    REQUIRE(aa_identity.axis().isApprox(Eigen::Vector3d::UnitX(), 1e-6));
}

TEST_CASE("Rotation: Rotation Matrix Consistency", "[rotation][consistency]")
{
    // Generate random rotations
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    const int num_tests = 100;
    for (int i = 0; i < num_tests; ++i)
    {
        Eigen::AngleAxisd aa(dist(gen), Eigen::Vector3d::Random().normalized());
        Eigen::Matrix3d R = aa.toRotationMatrix();
        Rotation rot(R);
        Eigen::Matrix3d R_reconstructed = rot.toRotationMatrix();
        REQUIRE(R_reconstructed.isApprox(R, 1e-6));
    }
}

TEST_CASE("Rotation: Quaternion Consistency", "[rotation][consistency]")
{
    // Generate random quaternions
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    const int num_tests = 100;
    for (int i = 0; i < num_tests; ++i)
    {
        Eigen::Quaterniond q(dist(gen), dist(gen), dist(gen), dist(gen));
        q.normalize();
        Rotation rot(q);
        Eigen::Quaterniond q_reconstructed = rot.toQuaternion();
        REQUIRE(q_reconstructed.coeffs().isApprox(q.coeffs(), 1e-6));
    }
}

TEST_CASE("Rotation: Euler Angles Reconstruction", "[rotation][euler_angles][reconstruction]")
{
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(-M_PI, M_PI);

    std::string sequence = "ZYX";
    bool intrinsic = true;

    const int num_tests = 100;
    for (int i = 0; i < num_tests; ++i)
    {
        double yaw = dist(gen);
        double pitch = dist(gen);
        double roll = dist(gen);

        Rotation rot_original(yaw, pitch, roll, intrinsic, sequence, false);
        Eigen::Vector3d euler_angles = rot_original.toEulerAngles(intrinsic, sequence);
        Rotation rot_reconstructed(euler_angles[0],
                                   euler_angles[1],
                                   euler_angles[2],
                                   intrinsic,
                                   sequence,
                                   false);

        REQUIRE(
            rot_original.toRotationMatrix().isApprox(rot_reconstructed.toRotationMatrix(), 1e-6));
    }
}

TEST_CASE("Rotation: Operator* Associativity", "[rotation][operator*][associativity]")
{
    Rotation rot1(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()));
    Rotation rot2(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));
    Rotation rot3(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ()));

    // (rot1 * rot2) * rot3
    Rotation combined1 = (rot1 * rot2) * rot3;

    // rot1 * (rot2 * rot3)
    Rotation combined2 = rot1 * (rot2 * rot3);

    REQUIRE(combined1.toRotationMatrix().isApprox(combined2.toRotationMatrix(), 1e-6));
}

TEST_CASE("Rotation: Operator* Non-Commutativity", "[rotation][operator*][non_commutativity]")
{
    Rotation rot1(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()));
    Rotation rot2(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY()));

    Rotation combined1 = rot1 * rot2;
    Rotation combined2 = rot2 * rot1;

    REQUIRE_FALSE(combined1.toRotationMatrix().isApprox(combined2.toRotationMatrix(), 1e-6));
}

TEST_CASE("Rotation: Static Method fromEulerAngles", "[rotation][static][fromEulerAngles]")
{
    double yaw = M_PI / 4;
    double pitch = M_PI / 6;
    double roll = M_PI / 3;
    std::string sequence = "ZYX";
    bool intrinsic = true;

    Rotation rot = Rotation::fromEulerAngles(yaw, pitch, roll, intrinsic, sequence, false);

    // Compare with constructing directly
    Rotation rot_direct(yaw, pitch, roll, intrinsic, sequence, false);
    REQUIRE(rot.toRotationMatrix().isApprox(rot_direct.toRotationMatrix(), 1e-6));
}

TEST_CASE("Rotation: Static Method fromRotationMatrix", "[rotation][static][fromRotationMatrix]")
{
    Eigen::Matrix3d rot_matrix =
        Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Rotation rot = Rotation::fromRotationMatrix(rot_matrix);

    REQUIRE(rot.toRotationMatrix().isApprox(rot_matrix, 1e-6));
}

TEST_CASE("Rotation: Static Method fromQuaternion", "[rotation][static][fromQuaternion]")
{
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitY()));
    Rotation rot = Rotation::fromQuaternion(q);

    REQUIRE(rot.toQuaternion().isApprox(q.normalized(), 1e-6));
}

TEST_CASE("Rotation: matrixT Invalid Sequence", "[rotation][exception][matrixT]")
{
    Eigen::Vector3d angles(M_PI / 4, M_PI / 6, M_PI / 3);
    REQUIRE_THROWS_AS(Rotation::matrixT(angles, "ABCD"), std::invalid_argument);
    REQUIRE_THROWS_AS(Rotation::matrixT(angles, "X1Z"), std::invalid_argument);
    REQUIRE_THROWS_AS(Rotation::matrixT(angles, "X"), std::invalid_argument);
}

TEST_CASE("Rotation: Quaternion Addition and Subtraction with Non-Unit Quaternions",
          "[rotation][quaternion]")
{
    Eigen::Quaterniond q1(2.0, 1.0, 0.0, 0.0);
    Eigen::Quaterniond q2(1.0, -1.0, 0.0, 0.0);

    Eigen::Quaterniond q_sum = Rotation::quaternion_plus(q1, q2);
    Eigen::Quaterniond expected_sum(3.0, 0.0, 0.0, 0.0);
    REQUIRE(q_sum.coeffs().isApprox(expected_sum.coeffs(), 1e-6));

    Eigen::Quaterniond q_sub = Rotation::quaternion_minus(q1, q2);
    Eigen::Quaterniond expected_sub(1.0, 2.0, 0.0, 0.0);
    REQUIRE(q_sub.coeffs().isApprox(expected_sub.coeffs(), 1e-6));
}

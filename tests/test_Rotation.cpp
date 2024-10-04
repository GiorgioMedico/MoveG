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

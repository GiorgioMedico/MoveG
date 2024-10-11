#define CATCH_CONFIG_MAIN
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pose_lib.h"
#include "rotation_lib.h"

using namespace MoveG;


TEST_CASE("Pose: Affine Example", "[pose]")
{
    // 1. Creazione di un oggetto Affine3d
    // Affine3d rappresenta una trasformazione affine nello spazio 3D (combinazione di rotazione, traslazione, scala, ecc.)
    Eigen::Affine3d affine =
        Eigen::Affine3d::Identity(); // Inizializza la trasformazione come identità

    // 2. Applicare una traslazione
    // Trasla l'oggetto di (1, 2, 3) lungo gli assi x, y, z rispettivamente
    affine.translate(Eigen::Vector3d(1.0, 2.0, 3.0));

    // 3. Applicare una rotazione
    // Ruota l'oggetto di 45 gradi attorno all'asse z
    double angle = M_PI / 4; // 45 gradi in radianti
    affine.rotate(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));

    // 4. Applicare una scala
    // Scala l'oggetto di 2 lungo tutti gli assi
    affine.scale(2.0);

    // 5. Combinare trasformazioni
    // È possibile concatenare più trasformazioni usando gli operatori
    Eigen::Affine3d additional_transform = Eigen::Affine3d::Identity();
    additional_transform.translate(Eigen::Vector3d(-1.0, 0.0, 1.0));
    additional_transform.rotate(
        Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY())); // 30 gradi attorno all'asse y

    // Combina le trasformazioni
    affine = affine * additional_transform;

    // 6. Applicare la trasformazione a un punto
    Eigen::Vector3d point(1.0, 1.0, 1.0);
    Eigen::Vector3d transformed_point = affine * point;

    // Stampare i risultati
    std::cout << "Trasformazione Affine:\n" << affine.matrix() << "\n\n";
    std::cout << "Punto originale: " << point.transpose() << "\n";
    std::cout << "Punto trasformato: " << transformed_point.transpose() << "\n\n";

    // 7. Invertire una trasformazione
    Eigen::Affine3d inverse_affine = affine.inverse();
    Eigen::Vector3d original_point = inverse_affine * transformed_point;
    std::cout << "Punto dopo la trasformazione inversa: " << original_point.transpose() << "\n";

    // 8. Uso di Affine3d con vettori omogenei
    // Affine3d può anche essere utilizzato con vettori omogenei (Vector4d)
    Eigen::Vector4d point_homogeneous(1.0, 1.0, 1.0, 1.0); // Il quarto componente è 1 per punti
    Eigen::Vector4d transformed_point_homogeneous = affine.matrix() * point_homogeneous;
    std::cout << "\nPunto omogeneo trasformato: " << transformed_point_homogeneous.transpose()
              << "\n";
}

/**
 * @file traj_lib.h
 * @brief Classe per la rappresentazione di Traiettorie
 *
 * @author Giorgio Medico
 * @date 5/10/2024
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "pose_lib.h"
#include "rotation_lib.h"

namespace MoveG
{
// Workspace Representation

// Classe Waypoint per la rappresentazione di un punto di passaggio con velocità e accelerazione
class Waypoint
{
public:
    // Costruttori
    Waypoint();

    // Distruttore
    ~Waypoint();

    // Metodi di accesso


private:
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d acceleration_;
};


} // namespace MoveG

// Classe Trajectory per la rappresentazione di una traiettoria nel workspace


// Joint Space Representation


// Classe JointWaypoint per la rappresentazione di un punto di passaggio nello spazio delle giunzioni

// Classe JointTrajectory per la rappresentazione di una traiettoria nello spazio delle giunzioni

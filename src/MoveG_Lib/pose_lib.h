/**
 * @file pose_lib.h
 * @brief Classe per la rappresentazione di Pose
 *
 * @author Giorgio Medico
 * @date 4/10/2024
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "rotation_lib.h"

/**
 * @brief Namespace per il movimento e la manipolazione delle pose.
 */
namespace MoveG
{
/**
     * @class Pose
     * @brief Classe per rappresentare una posa nello spazio 3D.
     *
     * La classe Pose incapsula una posizione e un'orientazione nello spazio tridimensionale.
     * Utilizza Eigen per le operazioni matematiche relative alla geometria.
     */
class Pose
{
public:
    // Constructors

    /**
         * @brief Costruttore di default.
         *
         * Inizializza la posa alla posizione (0,0,0) con orientamento neutro (quaternion identità).
         */
    Pose();

    /**
         * @brief Costruttore con posizione e orientamento tramite quaternion.
         *
         * @param position La posizione nel spazio.
         * @param orientation Il quaternion che rappresenta l'orientamento.
         */
    Pose(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

    /**
         * @brief Costruttore con posizione e matrice di rotazione.
         *
         * @param position La posizione nel spazio.
         * @param rotation_matrix La matrice di rotazione 3x3.
         */
    Pose(const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation_matrix);

    /**
         * @brief Costruttore con trasformazione affine.
         *
         * @param transformation La trasformazione affine 3D.
         */
    Pose(const Eigen::Affine3d &transformation);

    /**
         * @brief Costruttore con posizione e oggetto Rotation.
         *
         * @param position La posizione nel spazio.
         * @param orientation L'oggetto Rotation che rappresenta l'orientamento.
         */
    Pose(const Eigen::Vector3d &position, const Rotation &orientation);

    /**
         * @brief Costruttore con matrice omogenea 4x4.
         *
         * @param homogeneousT La matrice di trasformazione omogenea 4x4.
         */
    Pose(const Eigen::Matrix4d &homogeneousT);

    // Destructor

    /**
         * @brief Distruttore.
         */
    ~Pose();

    // Copy constructor and assignment operator

    /**
         * @brief Costruttore di copia.
         *
         * @param other L'oggetto Pose da copiare.
         */
    Pose(const Pose &other);

    /**
         * @brief Operatore di assegnazione.
         *
         * @param other L'oggetto Pose da assegnare.
         * @return Riferimento all'oggetto assegnato.
         */
    Pose &operator=(const Pose &other);

    // << operator overload to define the output stream

    /**
         * @brief Sovraccarica l'operatore di inserimento nello stream.
         *
         * @param os Il flusso di output.
         * @param pose L'oggetto Pose da inserire nel flusso.
         * @return Riferimento al flusso di output.
         */
    friend std::ostream &operator<<(std::ostream &os, const Pose &pose);

    // Getters

    /**
         * @brief Ottiene la posizione.
         *
         * @return La posizione come vettore 3D.
         */
    Eigen::Vector3d getPosition() const;

    /**
         * @brief Ottiene il quaternion dell'orientamento.
         *
         * @return Il quaternion che rappresenta l'orientamento.
         */
    Eigen::Quaterniond getQaternion() const;

    /**
         * @brief Ottiene la matrice di rotazione.
         *
         * @return La matrice di rotazione 3x3.
         */
    Eigen::Matrix3d getRotationMatrix() const;

    /**
         * @brief Ottiene la trasformazione affine.
         *
         * @return La trasformazione affine 3D.
         */
    Eigen::Affine3d getAffineTransformation() const;

    /**
         * @brief Ottiene la matrice di trasformazione omogenea.
         *
         * @return La matrice omogenea 4x4.
         */
    Eigen::Matrix4d getHomogeneousT() const;

    /**
         * @brief Ottiene la coordinata X della posizione.
         *
         * @return La coordinata X.
         */
    double getX() const;

    /**
         * @brief Ottiene la coordinata Y della posizione.
         *
         * @return La coordinata Y.
         */
    double getY() const;

    /**
         * @brief Ottiene la coordinata Z della posizione.
         *
         * @return La coordinata Z.
         */
    double getZ() const;

    /**
         * @brief Ottiene la componente X del quaternion.
         *
         * @return La componente X.
         */
    double getQx() const;

    /**
         * @brief Ottiene la componente Y del quaternion.
         *
         * @return La componente Y.
         */
    double getQy() const;

    /**
         * @brief Ottiene la componente Z del quaternion.
         *
         * @return La componente Z.
         */
    double getQz() const;

    /**
         * @brief Ottiene la componente W del quaternion.
         *
         * @return La componente W.
         */
    double getQw() const;

    // Setters

    /**
         * @brief Imposta la posizione.
         *
         * @param position La nuova posizione come vettore 3D.
         */
    void setPosition(const Eigen::Vector3d &position);

    /**
         * @brief Imposta l'orientamento tramite quaternion.
         *
         * @param orientation Il nuovo quaternion che rappresenta l'orientamento.
         */
    void setOrientation(const Eigen::Quaterniond &orientation);

    /**
         * @brief Imposta la matrice di rotazione.
         *
         * @param rotation_matrix La nuova matrice di rotazione 3x3.
         */
    void setRotationMatrix(const Eigen::Matrix3d &rotation_matrix);

    /**
         * @brief Imposta la trasformazione affine.
         *
         * @param transformation La nuova trasformazione affine 3D.
         */
    void setAffineTransformation(const Eigen::Affine3d &transformation);

    /**
         * @brief Imposta la matrice di trasformazione omogenea.
         *
         * @param homogeneousT La nuova matrice omogenea 4x4.
         */
    void setHomogeneousT(const Eigen::Matrix4d &homogeneousT);

    // Operations

    /**
         * @brief Composizione di due pose.
         *
         * @param other La seconda posa da comporre.
         * @return Una nuova posa risultante dalla composizione.
         */
    Pose operator*(const Pose &other) const; // Compose poses

    /**
         * @brief Calcola l'inverso della posa.
         *
         * @return La posa inversa.
         */
    Pose inverse() const;

    // Friend functions for input/output

    /**
         * @brief Sovraccarica l'operatore di inserimento nello stream.
         *
         * @param os Il flusso di output.
         * @param pose L'oggetto Pose da inserire nel flusso.
         * @return Riferimento al flusso di output.
         */
    friend std::ostream &operator<<(std::ostream &os, const Pose &pose);

private:
    Eigen::Vector3d position_;       ///< Posizione nello spazio 3D.
    Eigen::Quaterniond orientation_; ///< Orientamento rappresentato come quaternion.
};

} // namespace MoveG

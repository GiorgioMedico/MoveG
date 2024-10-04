/*
    Author: Giorgio Medico
    Date: 24/09/2024
    Description: Classe per la rappresentazione e manipolazione di rotazioni in 3D
    File : rotation_lib.h
*/
#pragma once

#include "eigen3/Eigen/Dense"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// Commenti aggiuntivi per chiarire il comportamento delle rotazioni intrinseche ed estrinseche

/**
 * Rotazioni estrinseche vs intrinseche.
 *
 * - Estrinseche: tutte le rotazioni si riferiscono a un sistema di coordinate fisso/globale xyz.
 *   Ogni rotazione si applica al sistema di coordinate globale.
 *
 * - Intrinseche: ogni rotazione si riferisce al sistema di coordinate ruotato precedentemente.
 *   Ad esempio, una rotazione intrinseca Yaw-Pitch'-Roll'' (z-y'-x''):
 *   1) Rotazione attorno all'asse z globale.
 *   2) Rotazione attorno al nuovo asse y'.
 *   3) Rotazione attorno al nuovo asse x''.
 *
 * La moltiplicazione delle matrici avviene in ordine di applicazione (le matrici sono le stesse per entrambi i tipi):
 * Intrinseche: R = R3 * R2 * R1
 * Estrinseche: R = R1 * R2 * R3
 */

namespace MoveG
{

/**
 * @class Rotation
 * @brief Classe per rappresentare e gestire rotazioni nello spazio tridimensionale.
 *
 * Questa classe supporta diverse rappresentazioni di rotazioni, inclusi matrici di rotazione,
 * quaternioni, asse-angolo e angoli di Eulero. Permette inoltre la composizione di rotazioni.
 */
class Rotation
{
public:
    /**
     * @brief Costruttore di default.
     *
     * Inizializza la rotazione come l'identità.
     */
    Rotation();

    /**
     * @brief Costruttore da matrice di rotazione.
     * @param rotation_matrix Matrice di rotazione 3x3.
     */
    Rotation(const Eigen::Matrix3d &rotation_matrix);

    /**
     * @brief Costruttore da quaternione.
     * @param quaternion Quaternione rappresentante la rotazione.
     */
    Rotation(const Eigen::Quaterniond &quaternion);

    /**
     * @brief Costruttore da asse-angolo.
     * @param angle_axis Rappresentazione asse-angolo della rotazione.
     */
    Rotation(const Eigen::AngleAxisd &angle_axis);

    /**
     * @brief Costruttore da angoli di Eulero.
     * @param angle1 Primo angolo di Eulero.
     * @param angle2 Secondo angolo di Eulero.
     * @param angle3 Terzo angolo di Eulero.
     * @param intrinsic Se true, utilizza rotazioni intrinseche; altrimenti, estrinseche.
     * @param sequence Sequenza degli assi (es. "ZYX").
     * @param degree Se true, gli angoli sono in gradi; altrimenti, in radianti.
     */
    Rotation(double angle1,
             double angle2,
             double angle3,
             bool intrinsic = true,
             const std::string &sequence = "ZYX",
             bool degree = false);

    /**
     * @brief Distruttore di default.
     */
    ~Rotation() = default;

    /** Metodi statiti per definire una rotazione
     * I metodi statici permettono di creare una rotazione senza dover istanziare un oggetto.
     *
     * Esempio:
     * Rotation rot = Rotation::fromEulerAngles(0, 0, 0, true, "XYZ", false);
     * Altrimeti si sarebbe dovuto fare:
     *
     * Rotation rot;
     * rot = rot.fromEulerAngles(0, 0, 0, true, "XYZ", false);
     *
     */

    /**
     * @brief Crea una rotazione da una matrice di rotazione.
     * @param rotation_matrix Matrice di rotazione 3x3.
     * @return Oggetto Rotation.
     */
    static Rotation fromRotationMatrix(const Eigen::Matrix3d &rotation_matrix);

    /**
     * @brief Crea una rotazione da un quaternione.
     * @param quaternion Quaternione rappresentante la rotazione.
     * @return Oggetto Rotation.
     */
    static Rotation fromQuaternion(const Eigen::Quaterniond &quaternion);

    /**
     * @brief Crea una rotazione da una rappresentazione asse-angolo.
     * @param angle_axis Rappresentazione asse-angolo della rotazione.
     * @return Oggetto Rotation.
     */
    static Rotation fromAngleAxis(const Eigen::AngleAxisd &angle_axis);

    /**
     * @brief Crea una rotazione da angoli di Eulero.
     * @param angle1 Primo angolo di Eulero.
     * @param angle2 Secondo angolo di Eulero.
     * @param angle3 Terzo angolo di Eulero.
     * @param intrinsic Se true, utilizza rotazioni intrinseche; altrimenti, estrinseche.
     * @param sequence Sequenza degli assi (es. "XYZ").
     * @param degree Se true, gli angoli sono in gradi; altrimenti, in radianti.
     * @return Oggetto Rotation.
     */
    static Rotation fromEulerAngles(double angle1,
                                    double angle2,
                                    double angle3,
                                    bool intrinsic = true,
                                    const std::string &sequence = "XYZ",
                                    bool degree = false);

    // Convertitori

    /**
     * @brief Converte la rotazione in una matrice di rotazione.
     * @return Matrice di rotazione 3x3.
     */
    Eigen::Matrix3d toRotationMatrix() const;

    /**
     * @brief Converte la rotazione in un quaternione.
     * @return Quaternione rappresentante la rotazione.
     */
    Eigen::Quaterniond toQuaternion() const;

    /**
     * @brief Converte la rotazione in una rappresentazione asse-angolo.
     * @return Rappresentazione asse-angolo della rotazione.
     */
    Eigen::AngleAxisd toAngleAxis() const;

    /**
     * @brief Converte la rotazione in angoli di Eulero.
     * @param intrinsic Se true, utilizza rotazioni intrinseche; altrimenti, estrinseche.
     * @param sequence Sequenza degli assi (es. "ZYX").
     * @return Vettore contenente i tre angoli di Eulero.
     */
    Eigen::Vector3d toEulerAngles(bool intrinsic = true, const std::string &sequence = "ZYX") const;

    // Sovraccarico dell'operatore *

    /**
     * @brief Composizione di due rotazioni.
     * @param other Rotazione da comporre.
     * @return Nuova rotazione risultante dalla composizione.
     */
    Rotation operator*(const Rotation &other) const;

    // Metodi statici per le operazioni sui quaternioni
    /**
     * @brief Calcola la differenza tra due quaternioni.
     * @param a Quaternione di riferimento.
     * @param b Quaternione da sottrarre.
     * @return Quaternione risultante dalla differenza.
     */
    static Eigen::Quaterniond quaternion_difference(const Eigen::Quaterniond &a,
                                                    const Eigen::Quaterniond &b);

    /**
     * @brief Neutra il quaternione.
     * @param v Quaternione da negare.
     * @return Quaternione negato.
     */
    static Eigen::Quaterniond quaternion_negation(const Eigen::Quaterniond &v);

    /**
     * @brief Calcola il prodotto scalare di un quaternione per uno scalare.
     * @param v Quaternione da scalare.
     * @param t Scala da moltiplicare.
     * @return Quaternione scalato.
     */
    static Eigen::Quaterniond scalar_product(const Eigen::Quaterniond &v, double t);

    /**
     * @brief Calcola la differenza tra due quaternioni (v1 - v0).
     * @param v1 Primo quaternione.
     * @param v0 Secondo quaternione.
     * @return Quaternione risultante dalla sottrazione.
     */
    static Eigen::Quaterniond quaternion_minus(const Eigen::Quaterniond &v1,
                                               const Eigen::Quaterniond &v0);

    /**
     * @brief Calcola la somma di due quaternioni.
     * @param v1 Primo quaternione.
     * @param v0 Secondo quaternione.
     * @return Quaternione risultante dall'addizione.
     */
    static Eigen::Quaterniond quaternion_plus(const Eigen::Quaterniond &v1,
                                              const Eigen::Quaterniond &v0);

    /**
     * @brief Converte gradi in radianti.
     * @param degree Angolo in gradi.
     * @return Angolo in radianti.
     */
    static double deg2rad(double degree);

    /**
     * @brief Converte radianti in gradi.
     * @param radian Angolo in radianti.
     * @return Angolo in gradi.
     */
    static double rad2deg(double radian);

    // Elementary Rotations Matrices

    /**
     * @brief Matrice di rotazione per una rotazione attorno all'asse X.
     * @param angle Angolo di rotazione in radianti.
     * @return Matrice di rotazione 3x3.
     */
    static Eigen::Matrix3d rotationX(double angle);

    /**
     * @brief Matrice di rotazione per una rotazione attorno all'asse Y.
     * @param angle Angolo di rotazione in radianti.
     * @return Matrice di rotazione 3x3.
     */

    static Eigen::Matrix3d rotationY(double angle);

    /**
     * @brief Matrice di rotazione per una rotazione attorno all'asse Z.
     * @param angle Angolo di rotazione in radianti.
     * @return Matrice di rotazione 3x3.
     */
    static Eigen::Matrix3d rotationZ(double angle);

    // Definizione di Matrice S e R_dot

    /**
     * @brief Calcola la matrice S
     * @param omega Vettore di velocità angolare.(Diverso da la velocità degli angoli di Eulero)
     * @return Matrice S
     *
     */
    static Eigen::Matrix3d matrixS(const Eigen::Vector3d &omega);

    /**
     * @brief Calcola la matrice R_dot
     * @param R Matrice di rotazione.
     * @param S Matrice S.
     * @return Matrice R_dot
     *
     */
    static Eigen::Matrix3d matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Matrix3d &S);

    /**
     * @brief Calcola la matrice R_dot da un vettore di velocità angolare.
     * @param omega Vettore di velocità angolare.
     * @param R Matrice di rotazione.
     * @return Matrice R
     *
     */
    static Eigen::Matrix3d matrixR_dot(const Eigen::Matrix3d &R, const Eigen::Vector3d &omega);

    // Matrix T Mapping Between Body Angular Velocity Vector and the Euler Angle Rates

    /**
     * @brief Calcola la matrice T
     * @param angles Vettore di angoli di Eulero.
     * @param sequence Sequenza degli assi (es. "ZYX", "ZYZ").
     * @return Matrice T
     *
     */
    static Eigen::Matrix3d matrixT(const Eigen::Vector3d &angles, const std::string &sequence);

private:
    /**
     * @brief Rappresentazione interna della rotazione tramite quaternione.
     */
    Eigen::Quaterniond q;

    // Funzioni ausiliarie

    /**
     * @brief Crea una rappresentazione asse-angolo.
     * @param axis Asse di rotazione ('X', 'Y', 'Z' o minuscoli).
     * @param angle Angolo di rotazione in radianti.
     * @return Rappresentazione asse-angolo.
     */
    static Eigen::AngleAxisd angleAxis(char axis, double angle);

    /**
     * @brief Converte un carattere dell'asse in indice.
     * @param axis Asse di rotazione ('X', 'Y', 'Z' o minuscoli).
     * @return Indice corrispondente all'asse (0 per X, 1 per Y, 2 per Z).
     * @throws std::invalid_argument Se l'asse non è valido.
     */
    static int axisToIndex(char axis);

    /**
     * @brief Controlla la validità della sequenza di angoli di Eulero.
     * @param sequence Sequenza degli assi (es. "XYZ").
     * @return Valore intero (sempre 1 se valido).
     * @throws std::invalid_argument Se la sequenza non è valida.
     */
    static int checkSequence(const std::string &sequence);
};

} // namespace MoveG

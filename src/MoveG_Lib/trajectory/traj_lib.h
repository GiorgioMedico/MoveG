/**
 * @file traj_lib.h
 * @brief Classe per la rappresentazione di Traiettorie
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "pose_lib.h"
#include "rotation_lib.h"

namespace MoveG
{
/**
  * @brief Enumerazione dei tipi di path geometrici supportati
  */
enum class GeometricPathType
{
    LINEAR,         ///< Percorso lineare tra waypoints
    CIRCULAR,       ///< Percorso circolare o arco
    SPLINE_CUBIC,   ///< Interpolazione con spline cubica
    SPLINE_QUINTIC, ///< Interpolazione con spline di quinto ordine
    SPLINE_BSPLINE  ///< Interpolazione con B-spline
};

/**
  * @brief Enumerazione dei metodi di interpolazione dell'orientamento
  */
enum class OrientationInterpolationType
{
    SLERP, ///< Spherical Linear Interpolation
    SQUAD  ///< Spherical Cubic Interpolation
};

/**
  * @brief Enumerazione delle leggi di moto supportate
  */
enum class MotionLawType
{
    CUBIC_POLYNOMIAL,   ///< Polinomiale di 3° ordine
    QUINTIC_POLYNOMIAL, ///< Polinomiale di 5° ordine
    TRAPEZOIDAL,        ///< Velocità trapezoidale
    CUBIC_SPLINE,       ///< Cubic spline
    B_SPLINE            ///< B-spline
};

/**
  * @class Waypoint
  * @brief Classe che rappresenta un waypoint per una traiettoria
  *
  * La classe Waypoint contiene informazioni sulla posizione, orientamento,
  * e derivate (velocità, accelerazione, jerk) per un punto della traiettoria.
  */
class Waypoint
{
public:
    /**
      * @brief Costruttore di default
      */
    Waypoint();

    /**
      * @brief Costruttore con posizione e orientamento
      * @param position Posizione 3D del waypoint
      * @param orientation Orientamento come quaternione
      */
    Waypoint(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation);

    /**
      * @brief Costruttore con Pose
      * @param pose Posizione e orientamento del waypoint
      */
    explicit Waypoint(const Pose &pose);

    /**
      * @brief Costruttore di copia
      * @param other Waypoint da copiare
      */
    Waypoint(const Waypoint &other);

    /**
      * @brief Costruttore di spostamento
      * @param other Waypoint da spostare
      */
    Waypoint(Waypoint &&other) noexcept;

    /**
      * @brief Operatore di assegnamento di copia
      * @param other Waypoint da copiare
      * @return Riferimento a questo Waypoint
      */
    Waypoint &operator=(const Waypoint &other);

    /**
      * @brief Operatore di assegnamento di spostamento
      * @param other Waypoint da spostare
      * @return Riferimento a questo Waypoint
      */
    Waypoint &operator=(Waypoint &&other) noexcept;

    /**
      * @brief Distruttore virtuale
      */
    virtual ~Waypoint() = default;

    // Getters per posizione e orientamento
    Eigen::Vector3d getPosition() const
    {
        return position_;
    }
    Eigen::Quaterniond getOrientation() const
    {
        return orientation_;
    }
    Pose getPose() const
    {
        return Pose(position_, orientation_);
    }

    // Getters per velocità
    std::optional<Eigen::Vector3d> getLinearVelocity() const
    {
        return linearVelocity_;
    }
    std::optional<Eigen::Vector3d> getAngularVelocity() const
    {
        return angularVelocity_;
    }

    // Getters per accelerazione
    std::optional<Eigen::Vector3d> getLinearAcceleration() const
    {
        return linearAcceleration_;
    }
    std::optional<Eigen::Vector3d> getAngularAcceleration() const
    {
        return angularAcceleration_;
    }

    // Getters per jerk
    std::optional<Eigen::Vector3d> getLinearJerk() const
    {
        return linearJerk_;
    }
    std::optional<Eigen::Vector3d> getAngularJerk() const
    {
        return angularJerk_;
    }

    // Getters per parametri di smussamento
    double getBlendRadius() const
    {
        return blendRadius_;
    }
    double getBlendTolerance() const
    {
        return blendTolerance_;
    }

    // Getter per timestamp
    std::optional<double> getTimeStamp() const
    {
        return timeStamp_;
    }

    // Setters per posizione e orientamento
    void setPosition(const Eigen::Vector3d &position)
    {
        position_ = position;
    }
    void setOrientation(const Eigen::Quaterniond &orientation)
    {
        orientation_ = orientation.normalized();
    }
    void setPose(const Pose &pose);

    // Setters per velocità
    void setLinearVelocity(const Eigen::Vector3d &velocity)
    {
        linearVelocity_ = velocity;
    }
    void setAngularVelocity(const Eigen::Vector3d &velocity)
    {
        angularVelocity_ = velocity;
    }

    // Setters per accelerazione
    void setLinearAcceleration(const Eigen::Vector3d &acceleration)
    {
        linearAcceleration_ = acceleration;
    }
    void setAngularAcceleration(const Eigen::Vector3d &acceleration)
    {
        angularAcceleration_ = acceleration;
    }

    // Setters per jerk
    void setLinearJerk(const Eigen::Vector3d &jerk)
    {
        linearJerk_ = jerk;
    }
    void setAngularJerk(const Eigen::Vector3d &jerk)
    {
        angularJerk_ = jerk;
    }

    // Setters per parametri di smussamento
    void setBlendRadius(double radius)
    {
        blendRadius_ = radius;
    }
    void setBlendTolerance(double tolerance)
    {
        blendTolerance_ = tolerance;
    }

    // Setter per timestamp
    void setTimeStamp(double timeStamp)
    {
        timeStamp_ = timeStamp;
    }

    /**
      * @brief Operatore di stream per la stampa
      * @param os Stream di output
      * @param waypoint Waypoint da stampare
      * @return Riferimento allo stream di output
      */
    friend std::ostream &operator<<(std::ostream &os, const Waypoint &waypoint);

    /**
      * @brief Interpolazione lineare tra due waypoints
      * @param start Waypoint iniziale
      * @param end Waypoint finale
      * @param t Parametro di interpolazione (0.0 - 1.0)
      * @param orientationType Tipo di interpolazione per l'orientamento
      * @return Waypoint interpolato
      */
    static Waypoint interpolate(
        const Waypoint &start,
        const Waypoint &end,
        double t,
        OrientationInterpolationType orientationType = OrientationInterpolationType::SLERP);

private:
    Eigen::Vector3d position_;       ///< Posizione 3D del waypoint
    Eigen::Quaterniond orientation_; ///< Orientamento come quaternione

    std::optional<Eigen::Vector3d> linearVelocity_;  ///< Velocità lineare (opzionale)
    std::optional<Eigen::Vector3d> angularVelocity_; ///< Velocità angolare (opzionale)

    std::optional<Eigen::Vector3d> linearAcceleration_;  ///< Accelerazione lineare (opzionale)
    std::optional<Eigen::Vector3d> angularAcceleration_; ///< Accelerazione angolare (opzionale)

    std::optional<Eigen::Vector3d> linearJerk_;  ///< Jerk lineare (opzionale)
    std::optional<Eigen::Vector3d> angularJerk_; ///< Jerk angolare (opzionale)

    double blendRadius_ = 0.0;      ///< Raggio di smussamento
    double blendTolerance_ = 0.001; ///< Tolleranza di smussamento

    std::optional<double> timeStamp_; ///< Timestamp (opzionale)
};

/**
  * @class GeometricPath
  * @brief Classe che rappresenta un percorso geometrico
  *
  * Questa classe definisce la geometria del percorso tra waypoints,
  * supportando percorsi lineari, circolari e basati su spline.
  */
class GeometricPath
{
public:
    /**
      * @brief Costruttore di default
      * @param type Tipo di percorso geometrico
      */
    explicit GeometricPath(GeometricPathType type = GeometricPathType::LINEAR);

    /**
      * @brief Costruttore di copia
      * @param other GeometricPath da copiare
      */
    GeometricPath(const GeometricPath &other);

    /**
      * @brief Costruttore di spostamento
      * @param other GeometricPath da spostare
      */
    GeometricPath(GeometricPath &&other) noexcept;

    /**
      * @brief Operatore di assegnamento di copia
      * @param other GeometricPath da copiare
      * @return Riferimento a questo GeometricPath
      */
    GeometricPath &operator=(const GeometricPath &other);

    /**
      * @brief Operatore di assegnamento di spostamento
      * @param other GeometricPath da spostare
      * @return Riferimento a questo GeometricPath
      */
    GeometricPath &operator=(GeometricPath &&other) noexcept;

    /**
      * @brief Distruttore virtuale
      */
    virtual ~GeometricPath() = default;

    /**
      * @brief Imposta il tipo di percorso
      * @param type Tipo di percorso geometrico
      */
    void setPathType(GeometricPathType type)
    {
        pathType_ = type;
    }

    /**
      * @brief Ottiene il tipo di percorso
      * @return Tipo di percorso geometrico
      */
    GeometricPathType getPathType() const
    {
        return pathType_;
    }

    /**
      * @brief Imposta il centro del cerchio (per percorsi circolari)
      * @param center Centro del cerchio
      */
    void setCircleCenter(const Eigen::Vector3d &center)
    {
        circleCenter_ = center;
    }

    /**
      * @brief Imposta la normale al piano del cerchio (per percorsi circolari)
      * @param normal Vettore normale al piano del cerchio
      */
    void setCircleNormal(const Eigen::Vector3d &normal)
    {
        circleNormal_ = normal;
    }

    /**
      * @brief Imposta il raggio del cerchio (per percorsi circolari)
      * @param radius Raggio del cerchio
      */
    void setCircleRadius(double radius)
    {
        circleRadius_ = radius;
    }

    /**
      * @brief Imposta l'angolo dell'arco (per percorsi circolari)
      * @param angle Angolo dell'arco in radianti
      */
    void setArcAngle(double angle)
    {
        arcAngle_ = angle;
    }

    /**
      * @brief Imposta i punti di controllo (per percorsi basati su spline)
      * @param points Vettore di punti di controllo
      */
    void setControlPoints(const std::vector<Eigen::Vector3d> &points)
    {
        controlPoints_ = points;
    }

    /**
      * @brief Imposta i nodi (per B-spline)
      * @param knots Vettore di nodi
      */
    void setKnots(const std::vector<double> &knots)
    {
        knots_ = knots;
    }

    /**
      * @brief Imposta il grado della spline
      * @param degree Grado della spline
      */
    void setSplineDegree(int degree)
    {
        splineDegree_ = degree;
    }

    /**
      * @brief Ottiene una posizione sul percorso
      * @param parameter Parametro normalizzato (0.0 - 1.0)
      * @return Posizione 3D
      */
    Eigen::Vector3d evaluatePosition(double parameter) const;

    /**
      * @brief Ottiene una derivata della posizione sul percorso
      * @param parameter Parametro normalizzato (0.0 - 1.0)
      * @param order Ordine della derivata (1 = velocità, 2 = accelerazione)
      * @return Derivata della posizione
      */
    Eigen::Vector3d evaluateDerivative(double parameter, int order = 1) const;

    /**
      * @brief Lunghezza del percorso
      * @return Lunghezza in metri
      */
    double getLength() const;

    /**
      * @brief Crea un percorso lineare tra due punti
      * @param start Punto iniziale
      * @param end Punto finale
      * @return Percorso geometrico
      */
    static GeometricPath createLinePath(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

    /**
      * @brief Crea un percorso circolare
      * @param center Centro del cerchio
      * @param normal Normale al piano del cerchio
      * @param radius Raggio del cerchio
      * @param startPoint Punto di inizio
      * @param arcAngle Angolo dell'arco in radianti
      * @return Percorso geometrico
      */
    static GeometricPath createCircularPath(const Eigen::Vector3d &center,
                                            const Eigen::Vector3d &normal,
                                            double radius,
                                            const Eigen::Vector3d &startPoint,
                                            double arcAngle);

    /**
      * @brief Crea un percorso basato su spline cubica
      * @param controlPoints Punti di controllo
      * @return Percorso geometrico
      */
    static GeometricPath createCubicSplinePath(const std::vector<Eigen::Vector3d> &controlPoints);

    /**
      * @brief Crea un percorso basato su B-spline
      * @param controlPoints Punti di controllo
      * @param degree Grado della spline
      * @param knots Nodi (opzionale)
      * @return Percorso geometrico
      */
    static GeometricPath createBSplinePath(const std::vector<Eigen::Vector3d> &controlPoints,
                                           int degree = 3,
                                           const std::vector<double> &knots = {});

private:
    GeometricPathType pathType_; ///< Tipo di percorso geometrico

    // Parametri per percorsi circolari
    std::optional<Eigen::Vector3d> circleCenter_; ///< Centro del cerchio
    std::optional<Eigen::Vector3d> circleNormal_; ///< Normale al piano del cerchio
    std::optional<double> circleRadius_;          ///< Raggio del cerchio
    std::optional<double> arcAngle_;              ///< Angolo dell'arco

    // Parametri per spline
    std::vector<Eigen::Vector3d> controlPoints_; ///< Punti di controllo
    std::vector<double> knots_;                  ///< Nodi per B-spline
    int splineDegree_ = 3;                       ///< Grado della spline

    // Funzioni di valutazione
    Eigen::Vector3d evaluateLinearPath(double parameter) const;
    Eigen::Vector3d evaluateCircularPath(double parameter) const;
    Eigen::Vector3d evaluateCubicSplinePath(double parameter) const;
    Eigen::Vector3d evaluateBSplinePath(double parameter) const;

    // Funzioni per le derivate
    Eigen::Vector3d evaluateLinearPathDerivative(double parameter, int order) const;
    Eigen::Vector3d evaluateCircularPathDerivative(double parameter, int order) const;
    Eigen::Vector3d evaluateCubicSplinePathDerivative(double parameter, int order) const;
    Eigen::Vector3d evaluateBSplinePathDerivative(double parameter, int order) const;
};

/**
  * @class MotionLaw
  * @brief Classe che rappresenta una legge di moto
  *
  * Questa classe definisce come varia la velocità lungo il percorso,
  * implementando diverse leggi di moto come polinomiali, trapezoidali e spline.
  */
class MotionLaw
{
public:
    /**
      * @brief Costruttore di default
      * @param type Tipo di legge di moto
      */
    explicit MotionLaw(MotionLawType type = MotionLawType::CUBIC_POLYNOMIAL);

    /**
      * @brief Costruttore di copia
      * @param other MotionLaw da copiare
      */
    MotionLaw(const MotionLaw &other);

    /**
      * @brief Costruttore di spostamento
      * @param other MotionLaw da spostare
      */
    MotionLaw(MotionLaw &&other) noexcept;

    /**
      * @brief Operatore di assegnamento di copia
      * @param other MotionLaw da copiare
      * @return Riferimento a questo MotionLaw
      */
    MotionLaw &operator=(const MotionLaw &other);

    /**
      * @brief Operatore di assegnamento di spostamento
      * @param other MotionLaw da spostare
      * @return Riferimento a questo MotionLaw
      */
    MotionLaw &operator=(MotionLaw &&other) noexcept;

    /**
      * @brief Distruttore virtuale
      */
    virtual ~MotionLaw() = default;

    /**
      * @brief Imposta il tipo di legge di moto
      * @param type Tipo di legge di moto
      */
    void setMotionType(MotionLawType type)
    {
        motionType_ = type;
    }

    /**
      * @brief Ottiene il tipo di legge di moto
      * @return Tipo di legge di moto
      */
    MotionLawType getMotionType() const
    {
        return motionType_;
    }

    /**
      * @brief Imposta la durata della legge di moto
      * @param duration Durata in secondi
      */
    void setDuration(double duration)
    {
        duration_ = duration;
    }

    /**
      * @brief Ottiene la durata della legge di moto
      * @return Durata in secondi
      */
    double getDuration() const
    {
        return duration_;
    }

    /**
      * @brief Imposta la velocità massima
      * @param velocity Velocità massima
      */
    void setMaxVelocity(double velocity)
    {
        maxVelocity_ = velocity;
    }

    /**
      * @brief Imposta l'accelerazione massima
      * @param acceleration Accelerazione massima
      */
    void setMaxAcceleration(double acceleration)
    {
        maxAcceleration_ = acceleration;
    }

    /**
      * @brief Imposta il jerk massimo
      * @param jerk Jerk massimo
      */
    void setMaxJerk(double jerk)
    {
        maxJerk_ = jerk;
    }

    /**
      * @brief Imposta le condizioni al contorno per la legge di moto
      * @param initialVelocity Velocità iniziale
      * @param finalVelocity Velocità finale
      * @param initialAcceleration Accelerazione iniziale
      * @param finalAcceleration Accelerazione finale
      */
    void setBoundaryConditions(double initialVelocity = 0.0,
                               double finalVelocity = 0.0,
                               double initialAcceleration = 0.0,
                               double finalAcceleration = 0.0);

    /**
      * @brief Calcola il parametro normalizzato per un dato tempo
      * @param time Tempo in secondi
      * @return Parametro normalizzato (0.0 - 1.0)
      */
    double evaluateParameter(double time) const;

    /**
      * @brief Calcola la prima derivata del parametro (velocità)
      * @param time Tempo in secondi
      * @return Prima derivata del parametro
      */
    double evaluateFirstDerivative(double time) const;

    /**
      * @brief Calcola la seconda derivata del parametro (accelerazione)
      * @param time Tempo in secondi
      * @return Seconda derivata del parametro
      */
    double evaluateSecondDerivative(double time) const;

    /**
      * @brief Calcola la terza derivata del parametro (jerk)
      * @param time Tempo in secondi
      * @return Terza derivata del parametro
      */
    double evaluateThirdDerivative(double time) const;

    /**
      * @brief Crea una legge di moto polinomiale cubica
      * @param duration Durata in secondi
      * @param initialVelocity Velocità iniziale
      * @param finalVelocity Velocità finale
      * @return Legge di moto
      */
    static MotionLaw createCubicPolynomial(double duration,
                                           double initialVelocity = 0.0,
                                           double finalVelocity = 0.0);

    /**
      * @brief Crea una legge di moto polinomiale quintica
      * @param duration Durata in secondi
      * @param initialVelocity Velocità iniziale
      * @param finalVelocity Velocità finale
      * @param initialAcceleration Accelerazione iniziale
      * @param finalAcceleration Accelerazione finale
      * @return Legge di moto
      */
    static MotionLaw createQuinticPolynomial(double duration,
                                             double initialVelocity = 0.0,
                                             double finalVelocity = 0.0,
                                             double initialAcceleration = 0.0,
                                             double finalAcceleration = 0.0);

    /**
      * @brief Crea una legge di moto trapezoidale
      * @param duration Durata in secondi
      * @param maxVelocity Velocità massima
      * @param maxAcceleration Accelerazione massima
      * @return Legge di moto
      */
    static MotionLaw createTrapezoidal(double duration, double maxVelocity, double maxAcceleration);

private:
    MotionLawType motionType_; ///< Tipo di legge di moto
    double duration_ = 1.0;    ///< Durata in secondi

    // Limiti cinematici
    std::optional<double> maxVelocity_;     ///< Velocità massima
    std::optional<double> maxAcceleration_; ///< Accelerazione massima
    std::optional<double> maxJerk_;         ///< Jerk massimo

    // Condizioni al contorno
    double initialVelocity_ = 0.0;     ///< Velocità iniziale
    double finalVelocity_ = 0.0;       ///< Velocità finale
    double initialAcceleration_ = 0.0; ///< Accelerazione iniziale
    double finalAcceleration_ = 0.0;   ///< Accelerazione finale

    // Parametri per la legge di moto trapezoidale
    double accelTime_ = 0.0;    ///< Tempo di accelerazione
    double decelTime_ = 0.0;    ///< Tempo di decelerazione
    double constVelTime_ = 0.0; ///< Tempo a velocità costante

    // Coefficienti polinomiali
    std::vector<double> coefficients_; ///< Coefficienti dei polinomi

    // Parametri per B-spline
    std::vector<double> knotVector_;    ///< Vettore dei nodi per B-spline
    std::vector<double> controlPoints_; ///< Punti di controllo per B-spline
    int splineDegree_ = 3;              ///< Grado della spline

    // Funzioni di valutazione
    double evaluateCubicPolynomial(double time) const;
    double evaluateQuinticPolynomial(double time) const;
    double evaluateTrapezoidal(double time) const;
    double evaluateCubicSpline(double time) const;
    double evaluateBSpline(double time) const;

    // Funzioni per le derivate
    double evaluateCubicPolynomialDerivative(double time, int order) const;
    double evaluateQuinticPolynomialDerivative(double time, int order) const;
    double evaluateTrapezoidalDerivative(double time, int order) const;
    double evaluateCubicSplineDerivative(double time, int order) const;
    double evaluateBSplineDerivative(double time, int order) const;

    // Inizializzazione dei coefficienti
    void initCubicPolynomial();
    void initQuinticPolynomial();
    void initTrapezoidal();
};

/**
  * @class Trajectory
  * @brief Classe che rappresenta una traiettoria
  *
  * Questa classe combina waypoints, percorsi geometrici e leggi di moto
  * per definire traiettorie complete per robot manipolatori.
  */
class Trajectory
{
public:
    /**
      * @brief Costruttore di default
      * @param name Nome della traiettoria
      */
    explicit Trajectory(const std::string &name = "");

    /**
      * @brief Costruttore di copia
      * @param other Trajectory da copiare
      */
    Trajectory(const Trajectory &other);

    /**
      * @brief Costruttore di spostamento
      * @param other Trajectory da spostare
      */
    Trajectory(Trajectory &&other) noexcept;

    /**
      * @brief Operatore di assegnamento di copia
      * @param other Trajectory da copiare
      * @return Riferimento a questa Trajectory
      */
    Trajectory &operator=(const Trajectory &other);

    /**
      * @brief Operatore di assegnamento di spostamento
      * @param other Trajectory da spostare
      * @return Riferimento a questa Trajectory
      */
    Trajectory &operator=(Trajectory &&other) noexcept;

    /**
      * @brief Distruttore virtuale
      */
    virtual ~Trajectory() = default;

    /**
      * @brief Aggiunge un waypoint alla traiettoria
      * @param waypoint Waypoint da aggiungere
      */
    void addWaypoint(const Waypoint &waypoint);

    /**
      * @brief Ottiene un waypoint specifico
      * @param index Indice del waypoint
      * @return Riferimento al waypoint
      * @throws std::out_of_range Se l'indice è fuori range
      */
    Waypoint &getWaypoint(size_t index);

    /**
      * @brief Ottiene un waypoint specifico (const)
      * @param index Indice del waypoint
      * @return Riferimento costante al waypoint
      * @throws std::out_of_range Se l'indice è fuori range
      */
    const Waypoint &getWaypoint(size_t index) const;

    /**
      * @brief Ottiene il numero di waypoints
      * @return Numero di waypoints
      */
    size_t getNumWaypoints() const
    {
        return waypoints_.size();
    }

    /**
      * @brief Imposta il tipo di percorso geometrico
      * @param type Tipo di percorso
      */
    void setGeometricPathType(GeometricPathType type);

    /**
      * @brief Configura un percorso circolare
      * @param center Centro del cerchio
      * @param normal Normale al piano
      * @param radius Raggio del cerchio
      * @param arcAngle Angolo dell'arco
      */
    void configureCircularPath(const Eigen::Vector3d &center,
                               const Eigen::Vector3d &normal,
                               double radius,
                               double arcAngle);

    /**
      * @brief Imposta il tipo di interpolazione dell'orientamento
      * @param type Tipo di interpolazione
      */
    void setOrientationInterpolationType(OrientationInterpolationType type)
    {
        orientationInterpolationType_ = type;
    }

    /**
      * @brief Ottiene il tipo di interpolazione dell'orientamento
      * @return Tipo di interpolazione
      */
    OrientationInterpolationType getOrientationInterpolationType() const
    {
        return orientationInterpolationType_;
    }

    /**
      * @brief Imposta il tipo di legge di moto
      * @param type Tipo di legge di moto
      */
    void setMotionLawType(MotionLawType type);

    /**
      * @brief Configura la legge di moto
      * @param duration Durata
      * @param maxVelocity Velocità massima
      * @param maxAcceleration Accelerazione massima
      * @param maxJerk Jerk massimo
      */
    void configureMotionLaw(double duration,
                            double maxVelocity,
                            double maxAcceleration,
                            double maxJerk);

    /**
      * @brief Genera la traiettoria
      * @return true se la generazione è avvenuta con successo, false altrimenti
      */
    bool generateTrajectory();

    /**
      * @brief Valuta la traiettoria ad un dato istante
      * @param time Tempo in secondi
      * @return Waypoint alla posizione specificata
      */
    Waypoint evaluateAtTime(double time) const;

    /**
      * @brief Imposta la durata della traiettoria
      * @param duration Durata in secondi
      */
    void setDuration(double duration)
    {
        duration_ = duration;
    }

    /**
      * @brief Ottiene la durata della traiettoria
      * @return Durata in secondi
      */
    double getDuration() const
    {
        return duration_;
    }

    /**
      * @brief Imposta il nome della traiettoria
      * @param name Nome della traiettoria
      */
    void setName(const std::string &name)
    {
        name_ = name;
    }

    /**
      * @brief Ottiene il nome della traiettoria
      * @return Nome della traiettoria
      */
    std::string getName() const
    {
        return name_;
    }

    /**
      * @brief Imposta la velocità massima
      * @param maxVel Velocità massima
      */
    void setMaxVelocity(const Eigen::Vector3d &maxVel)
    {
        maxVelocity_ = maxVel;
    }

    /**
      * @brief Imposta l'accelerazione massima
      * @param maxAcc Accelerazione massima
      */
    void setMaxAcceleration(const Eigen::Vector3d &maxAcc)
    {
        maxAcceleration_ = maxAcc;
    }

    /**
      * @brief Imposta il jerk massimo
      * @param maxJerk Jerk massimo
      */
    void setMaxJerk(const Eigen::Vector3d &maxJerk)
    {
        maxJerk_ = maxJerk;
    }

    /**
      * @brief Esporta la traiettoria come serie di campioni a frequenza regolare
      * @param samplingFrequency Frequenza di campionamento in Hz
      * @return Vettore di waypoints campionati
      */
    std::vector<Waypoint> sampleTrajectory(double samplingFrequency) const;

    /**
      * @brief Concatena due traiettorie
      * @param other Traiettoria da appendere
      * @param blendTime Tempo di fusione tra le traiettorie
      * @return Traiettoria risultante
      */
    Trajectory concatenate(const Trajectory &other, double blendTime = 0.0) const;

    /**
      * @brief Verifica se la traiettoria rispetta i vincoli cinematici
      * @return true se tutti i vincoli sono rispettati, false altrimenti
      */
    bool checkKinematicConstraints() const;

    /**
      * @brief Operatore di stream per la stampa
      * @param os Stream di output
      * @param trajectory Traiettoria da stampare
      * @return Riferimento allo stream di output
      */
    friend std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory);

private:
    std::string name_;                ///< Nome della traiettoria
    std::vector<Waypoint> waypoints_; ///< Waypoints
    double duration_ = 0.0;           ///< Durata in secondi

    // Percorso geometrico e legge di moto
    GeometricPath geometricPath_; ///< Percorso geometrico
    MotionLaw motionLaw_;         ///< Legge di moto

    // Metodo di interpolazione dell'orientamento
    OrientationInterpolationType orientationInterpolationType_ =
        OrientationInterpolationType::SLERP;

    // Vincoli cinematici
    std::optional<Eigen::Vector3d> maxVelocity_;     ///< Velocità massima
    std::optional<Eigen::Vector3d> maxAcceleration_; ///< Accelerazione massima
    std::optional<Eigen::Vector3d> maxJerk_;         ///< Jerk massimo

    // Flag di traiettoria generata
    bool trajectoryGenerated_ = false; ///< Flag che indica se la traiettoria è stata generata

    // Segmenti di traiettoria
    struct TrajectorySegment
    {
        size_t startWaypointIndex; ///< Indice del waypoint iniziale
        size_t endWaypointIndex;   ///< Indice del waypoint finale
        double startTime;          ///< Tempo di inizio
        double endTime;            ///< Tempo di fine
        GeometricPath path;        ///< Percorso geometrico
        MotionLaw motion;          ///< Legge di moto
    };

    std::vector<TrajectorySegment> segments_; ///< Segmenti di traiettoria

    // Funzioni ausiliarie per la valutazione della traiettoria
    size_t findSegmentIndex(double time) const;

    // Funzioni di interpolazione dell'orientamento
    Eigen::Quaterniond interpolateSLERP(const Eigen::Quaterniond &q1,
                                        const Eigen::Quaterniond &q2,
                                        double t) const;

    Eigen::Quaterniond interpolateSQUAD(const Eigen::Quaterniond &q1,
                                        const Eigen::Quaterniond &q2,
                                        const Eigen::Quaterniond &a,
                                        const Eigen::Quaterniond &b,
                                        double t) const;
};

} // namespace MoveG

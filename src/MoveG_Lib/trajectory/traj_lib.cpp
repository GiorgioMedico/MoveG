/**
 * @file traj_lib.cpp
 * @brief Classe per la rappresentazione di Traiettorie
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#include "traj_lib.h"

namespace MoveG
{

//
// Implementazione della classe Waypoint
//

Waypoint::Waypoint()
    : position_(Eigen::Vector3d::Zero()), orientation_(Eigen::Quaterniond::Identity())
{
}

Waypoint::Waypoint(const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation)
    : position_(position), orientation_(orientation.normalized())
{
}

Waypoint::Waypoint(const Pose &pose)
    : position_(pose.getPosition()), orientation_(pose.getQuaternion())
{
}

Waypoint::Waypoint(const Waypoint &other)
    : position_(other.position_), orientation_(other.orientation_),
      linearVelocity_(other.linearVelocity_), angularVelocity_(other.angularVelocity_),
      linearAcceleration_(other.linearAcceleration_),
      angularAcceleration_(other.angularAcceleration_), linearJerk_(other.linearJerk_),
      angularJerk_(other.angularJerk_), blendRadius_(other.blendRadius_),
      blendTolerance_(other.blendTolerance_), timeStamp_(other.timeStamp_)
{
}

Waypoint::Waypoint(Waypoint &&other) noexcept
    : position_(std::move(other.position_)), orientation_(std::move(other.orientation_)),
      linearVelocity_(std::move(other.linearVelocity_)),
      angularVelocity_(std::move(other.angularVelocity_)),
      linearAcceleration_(std::move(other.linearAcceleration_)),
      angularAcceleration_(std::move(other.angularAcceleration_)),
      linearJerk_(std::move(other.linearJerk_)), angularJerk_(std::move(other.angularJerk_)),
      blendRadius_(std::move(other.blendRadius_)),
      blendTolerance_(std::move(other.blendTolerance_)), timeStamp_(std::move(other.timeStamp_))
{
}

Waypoint &Waypoint::operator=(const Waypoint &other)
{
    if (this != &other)
    {
        position_ = other.position_;
        orientation_ = other.orientation_;
        linearVelocity_ = other.linearVelocity_;
        angularVelocity_ = other.angularVelocity_;
        linearAcceleration_ = other.linearAcceleration_;
        angularAcceleration_ = other.angularAcceleration_;
        linearJerk_ = other.linearJerk_;
        angularJerk_ = other.angularJerk_;
        blendRadius_ = other.blendRadius_;
        blendTolerance_ = other.blendTolerance_;
        timeStamp_ = other.timeStamp_;
    }
    return *this;
}

Waypoint &Waypoint::operator=(Waypoint &&other) noexcept
{
    if (this != &other)
    {
        position_ = std::move(other.position_);
        orientation_ = std::move(other.orientation_);
        linearVelocity_ = std::move(other.linearVelocity_);
        angularVelocity_ = std::move(other.angularVelocity_);
        linearAcceleration_ = std::move(other.linearAcceleration_);
        angularAcceleration_ = std::move(other.angularAcceleration_);
        linearJerk_ = std::move(other.linearJerk_);
        angularJerk_ = std::move(other.angularJerk_);
        blendRadius_ = std::move(other.blendRadius_);
        blendTolerance_ = std::move(other.blendTolerance_);
        timeStamp_ = std::move(other.timeStamp_);
    }
    return *this;
}

void Waypoint::setPose(const Pose &pose)
{
    position_ = pose.getPosition();
    orientation_ = pose.getQuaternion();
}

std::ostream &operator<<(std::ostream &os, const Waypoint &waypoint)
{
    os << "Waypoint:" << std::endl;
    os << "  Position: [" << waypoint.position_.x() << ", " << waypoint.position_.y() << ", "
       << waypoint.position_.z() << "]" << std::endl;
    os << "  Orientation (w,x,y,z): [" << waypoint.orientation_.w() << ", "
       << waypoint.orientation_.x() << ", " << waypoint.orientation_.y() << ", "
       << waypoint.orientation_.z() << "]" << std::endl;

    if (waypoint.linearVelocity_)
    {
        os << "  Linear Velocity: [" << waypoint.linearVelocity_->x() << ", "
           << waypoint.linearVelocity_->y() << ", " << waypoint.linearVelocity_->z() << "]"
           << std::endl;
    }

    if (waypoint.angularVelocity_)
    {
        os << "  Angular Velocity: [" << waypoint.angularVelocity_->x() << ", "
           << waypoint.angularVelocity_->y() << ", " << waypoint.angularVelocity_->z() << "]"
           << std::endl;
    }

    if (waypoint.timeStamp_)
    {
        os << "  Time: " << *waypoint.timeStamp_ << " s" << std::endl;
    }

    os << "  Blend Radius: " << waypoint.blendRadius_ << std::endl;

    return os;
}

Waypoint Waypoint::interpolate(const Waypoint &start,
                               const Waypoint &end,
                               double t,
                               OrientationInterpolationType orientationType)
{
    // Assicura che t sia in [0, 1]
    t = std::max(0.0, std::min(1.0, t));

    // Interpola posizione linearmente
    Eigen::Vector3d position = start.position_ + t * (end.position_ - start.position_);

    // Interpola orientamento
    Eigen::Quaterniond orientation;

    if (orientationType == OrientationInterpolationType::SLERP)
    {
        orientation = start.orientation_.slerp(t, end.orientation_);
    }
    else // SQUAD
    {
        // Per SQUAD, usiamo una semplificazione assumendo che sia una doppia SLERP
        // (in una implementazione completa, qui ci sarebbe un vero SQUAD con controllo della tangente)
        double t1 = 2.0 * t * (1.0 - t);
        double t2 = t * t;

        Eigen::Quaterniond q1 = start.orientation_.slerp(t, end.orientation_);
        Eigen::Quaterniond q2 = start.orientation_.slerp(t + 0.01, end.orientation_).normalized();
        Eigen::Quaterniond q3 = start.orientation_.slerp(t - 0.01, end.orientation_).normalized();

        // Calcola l'accelerazione angolare (approssimazione)
        Eigen::Quaterniond acc = q3.inverse() * q1 * q1.inverse() * q2;

        // Applica l'accelerazione per ottenere un'interpolazione più fluida
        orientation = q1 * Eigen::Quaterniond::Identity().slerp(t1, acc).normalized();
    }

    Waypoint result(position, orientation);

    // Interpola velocità, accelerazione e jerk se presenti in entrambi i waypoint
    if (start.linearVelocity_ && end.linearVelocity_)
    {
        result.linearVelocity_ =
            *start.linearVelocity_ + t * (*end.linearVelocity_ - *start.linearVelocity_);
    }

    if (start.angularVelocity_ && end.angularVelocity_)
    {
        result.angularVelocity_ =
            *start.angularVelocity_ + t * (*end.angularVelocity_ - *start.angularVelocity_);
    }

    if (start.linearAcceleration_ && end.linearAcceleration_)
    {
        result.linearAcceleration_ = *start.linearAcceleration_ +
                                     t * (*end.linearAcceleration_ - *start.linearAcceleration_);
    }

    if (start.angularAcceleration_ && end.angularAcceleration_)
    {
        result.angularAcceleration_ = *start.angularAcceleration_ +
                                      t * (*end.angularAcceleration_ - *start.angularAcceleration_);
    }

    if (start.linearJerk_ && end.linearJerk_)
    {
        result.linearJerk_ = *start.linearJerk_ + t * (*end.linearJerk_ - *start.linearJerk_);
    }

    if (start.angularJerk_ && end.angularJerk_)
    {
        result.angularJerk_ = *start.angularJerk_ + t * (*end.angularJerk_ - *start.angularJerk_);
    }

    // Interpola il timestamp se presente
    if (start.timeStamp_ && end.timeStamp_)
    {
        result.timeStamp_ = *start.timeStamp_ + t * (*end.timeStamp_ - *start.timeStamp_);
    }

    // Interpola i parametri di blending
    result.blendRadius_ = start.blendRadius_ + t * (end.blendRadius_ - start.blendRadius_);
    result.blendTolerance_ =
        start.blendTolerance_ + t * (end.blendTolerance_ - start.blendTolerance_);

    return result;
}


//
// Implementazione della classe GeometricPath
//

GeometricPath::GeometricPath(GeometricPathType type) : pathType_(type)
{
}

GeometricPath::GeometricPath(const GeometricPath &other)
    : pathType_(other.pathType_), circleCenter_(other.circleCenter_),
      circleNormal_(other.circleNormal_), circleRadius_(other.circleRadius_),
      arcAngle_(other.arcAngle_), controlPoints_(other.controlPoints_), knots_(other.knots_),
      splineDegree_(other.splineDegree_)
{
}

GeometricPath::GeometricPath(GeometricPath &&other) noexcept
    : pathType_(std::move(other.pathType_)), circleCenter_(std::move(other.circleCenter_)),
      circleNormal_(std::move(other.circleNormal_)), circleRadius_(std::move(other.circleRadius_)),
      arcAngle_(std::move(other.arcAngle_)), controlPoints_(std::move(other.controlPoints_)),
      knots_(std::move(other.knots_)), splineDegree_(std::move(other.splineDegree_))
{
}

GeometricPath &GeometricPath::operator=(const GeometricPath &other)
{
    if (this != &other)
    {
        pathType_ = other.pathType_;
        circleCenter_ = other.circleCenter_;
        circleNormal_ = other.circleNormal_;
        circleRadius_ = other.circleRadius_;
        arcAngle_ = other.arcAngle_;
        controlPoints_ = other.controlPoints_;
        knots_ = other.knots_;
        splineDegree_ = other.splineDegree_;
    }
    return *this;
}

GeometricPath &GeometricPath::operator=(GeometricPath &&other) noexcept
{
    if (this != &other)
    {
        pathType_ = std::move(other.pathType_);
        circleCenter_ = std::move(other.circleCenter_);
        circleNormal_ = std::move(other.circleNormal_);
        circleRadius_ = std::move(other.circleRadius_);
        arcAngle_ = std::move(other.arcAngle_);
        controlPoints_ = std::move(other.controlPoints_);
        knots_ = std::move(other.knots_);
        splineDegree_ = std::move(other.splineDegree_);
    }
    return *this;
}

Eigen::Vector3d GeometricPath::evaluatePosition(double parameter) const
{
    // Limita il parametro all'intervallo [0, 1]
    parameter = std::max(0.0, std::min(1.0, parameter));

    switch (pathType_)
    {
    case GeometricPathType::LINEAR:
        return evaluateLinearPath(parameter);
    case GeometricPathType::CIRCULAR:
        return evaluateCircularPath(parameter);
    case GeometricPathType::SPLINE_CUBIC:
        return evaluateCubicSplinePath(parameter);
    case GeometricPathType::SPLINE_QUINTIC:
    case GeometricPathType::SPLINE_BSPLINE:
        return evaluateBSplinePath(parameter);
    default:
        throw std::runtime_error("Tipo di percorso geometrico non supportato");
    }
}

Eigen::Vector3d GeometricPath::evaluateDerivative(double parameter, int order) const
{
    // Limita il parametro all'intervallo [0, 1]
    parameter = std::max(0.0, std::min(1.0, parameter));

    switch (pathType_)
    {
    case GeometricPathType::LINEAR:
        return evaluateLinearPathDerivative(parameter, order);
    case GeometricPathType::CIRCULAR:
        return evaluateCircularPathDerivative(parameter, order);
    case GeometricPathType::SPLINE_CUBIC:
        return evaluateCubicSplinePathDerivative(parameter, order);
    case GeometricPathType::SPLINE_QUINTIC:
    case GeometricPathType::SPLINE_BSPLINE:
        return evaluateBSplinePathDerivative(parameter, order);
    default:
        throw std::runtime_error("Tipo di percorso geometrico non supportato");
    }
}

double GeometricPath::getLength() const
{
    switch (pathType_)
    {
    case GeometricPathType::LINEAR:
    {
        if (controlPoints_.size() < 2)
        {
            return 0.0;
        }
        return (controlPoints_[1] - controlPoints_[0]).norm();
    }
    case GeometricPathType::CIRCULAR:
    {
        if (!circleRadius_ || !arcAngle_)
        {
            return 0.0;
        }
        return (*circleRadius_) * std::abs(*arcAngle_);
    }
    case GeometricPathType::SPLINE_CUBIC:
    case GeometricPathType::SPLINE_QUINTIC:
    case GeometricPathType::SPLINE_BSPLINE:
    {
        // Calcolo numerico della lunghezza mediante campionamento
        const int numSamples = 100;
        double length = 0.0;
        Eigen::Vector3d prevPoint = evaluatePosition(0.0);

        for (int i = 1; i <= numSamples; ++i)
        {
            double t = static_cast<double>(i) / numSamples;
            Eigen::Vector3d currentPoint = evaluatePosition(t);
            length += (currentPoint - prevPoint).norm();
            prevPoint = currentPoint;
        }

        return length;
    }
    default:
        return 0.0;
    }
}

GeometricPath GeometricPath::createLinePath(const Eigen::Vector3d &start,
                                            const Eigen::Vector3d &end)
{
    GeometricPath path(GeometricPathType::LINEAR);
    path.controlPoints_ = {start, end};
    return path;
}

GeometricPath GeometricPath::createCircularPath(const Eigen::Vector3d &center,
                                                const Eigen::Vector3d &normal,
                                                double radius,
                                                const Eigen::Vector3d &startPoint,
                                                double arcAngle)
{
    GeometricPath path(GeometricPathType::CIRCULAR);
    path.circleCenter_ = center;
    path.circleNormal_ = normal.normalized(); // Assicurati che la normale sia normalizzata
    path.circleRadius_ = radius;
    path.arcAngle_ = arcAngle;

    // Calcola il punto di inizio relativo al centro
    Eigen::Vector3d relStart = startPoint - center;

    // Calcola la direzione radiale e la direzione tangenziale
    Eigen::Vector3d radialDir = relStart.normalized();
    Eigen::Vector3d tangentDir = (*path.circleNormal_).cross(radialDir).normalized();

    // Memorizza i vettori base per calcolare i punti sull'arco
    path.controlPoints_ = {radialDir, tangentDir, *path.circleNormal_};

    return path;
}

GeometricPath GeometricPath::createCubicSplinePath(
    const std::vector<Eigen::Vector3d> &controlPoints)
{
    if (controlPoints.size() < 2)
    {
        throw std::invalid_argument(
            "Sono necessari almeno 2 punti di controllo per una spline cubica");
    }

    GeometricPath path(GeometricPathType::SPLINE_CUBIC);
    path.controlPoints_ = controlPoints;

    return path;
}

GeometricPath GeometricPath::createBSplinePath(const std::vector<Eigen::Vector3d> &controlPoints,
                                               int degree,
                                               const std::vector<double> &knots)
{
    if (controlPoints.size() < degree + 1)
    {
        throw std::invalid_argument(
            "Sono necessari almeno (degree + 1) punti di controllo per una B-spline");
    }

    GeometricPath path(GeometricPathType::SPLINE_BSPLINE);
    path.controlPoints_ = controlPoints;
    path.splineDegree_ = degree;

    // Se i nodi non sono forniti, genera una sequenza uniforme
    if (knots.empty())
    {
        int numKnots = controlPoints.size() + degree + 1;
        path.knots_.resize(numKnots);

        // I primi 'degree' nodi sono 0
        for (int i = 0; i <= degree; ++i)
        {
            path.knots_[i] = 0.0;
        }

        // Nodi interni equidistanti
        for (int i = 1; i < controlPoints.size() - degree; ++i)
        {
            path.knots_[i + degree] = static_cast<double>(i) / (controlPoints.size() - degree);
        }

        // Gli ultimi 'degree' nodi sono 1
        for (int i = numKnots - degree - 1; i < numKnots; ++i)
        {
            path.knots_[i] = 1.0;
        }
    }
    else
    {
        path.knots_ = knots;
    }

    return path;
}

Eigen::Vector3d GeometricPath::evaluateLinearPath(double parameter) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Percorso lineare non inizializzato correttamente");
    }

    return controlPoints_[0] + parameter * (controlPoints_[1] - controlPoints_[0]);
}

Eigen::Vector3d GeometricPath::evaluateCircularPath(double parameter) const
{
    if (!circleCenter_ || !circleNormal_ || !circleRadius_ || !arcAngle_ ||
        controlPoints_.size() < 3)
    {
        throw std::runtime_error("Percorso circolare non inizializzato correttamente");
    }

    // Calcola l'angolo corrente
    double angle = (*arcAngle_) * parameter;

    // Ottieni i vettori base precedentemente memorizzati
    const Eigen::Vector3d &radialDir = controlPoints_[0];
    const Eigen::Vector3d &tangentDir = controlPoints_[1];

    // Calcola il punto sull'arco
    Eigen::Vector3d position = *circleCenter_ + (*circleRadius_) * (radialDir * std::cos(angle) +
                                                                    tangentDir * std::sin(angle));

    return position;
}

Eigen::Vector3d GeometricPath::evaluateCubicSplinePath(double parameter) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Percorso con spline cubica non inizializzato correttamente");
    }

    // Spline cubica di base (implementazione semplificata)
    // In un'implementazione completa, qui ci sarebbe una vera interpolazione spline cubica
    const size_t n = controlPoints_.size() - 1;

    // Trova il segmento appropriato
    const size_t i = std::min(static_cast<size_t>(parameter * n), n - 1);

    // Parametro locale all'interno del segmento
    const double t = parameter * n - i;

    // Interpolazione cubica tra due punti
    double t2 = t * t;
    double t3 = t2 * t;

    // Coefficienti di Hermite
    double h00 = 2 * t3 - 3 * t2 + 1;
    double h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2;
    double h11 = t3 - t2;

    // Calcola tangenti (in un'implementazione completa, queste sarebbero calcolate in base alla continuità)
    Eigen::Vector3d tangent0, tangent1;
    if (i == 0)
    {
        tangent0 = (controlPoints_[1] - controlPoints_[0]) * 0.5;
    }
    else
    {
        tangent0 = (controlPoints_[i + 1] - controlPoints_[i - 1]) * 0.5;
    }

    if (i == n - 1)
    {
        tangent1 = (controlPoints_[n] - controlPoints_[n - 1]) * 0.5;
    }
    else
    {
        tangent1 = (controlPoints_[i + 2] - controlPoints_[i]) * 0.5;
    }

    // Interpolazione di Hermite
    return h00 * controlPoints_[i] + h10 * tangent0 + h01 * controlPoints_[i + 1] + h11 * tangent1;
}

Eigen::Vector3d GeometricPath::evaluateBSplinePath(double parameter) const
{
    if (controlPoints_.size() < splineDegree_ + 1 ||
        knots_.size() < controlPoints_.size() + splineDegree_ + 1)
    {
        throw std::runtime_error("Percorso con B-spline non inizializzato correttamente");
    }

    // Implementazione semplificata di una B-spline
    // In un'implementazione completa, qui ci sarebbe l'algoritmo di De Boor

    // Mappa il parametro all'intervallo dei nodi
    double u = knots_.front() + parameter * (knots_.back() - knots_.front());

    // Trova l'indice dell'intervallo del nodo
    size_t k = splineDegree_;
    for (; k < knots_.size() - 1; ++k)
    {
        if (u <= knots_[k + 1])
        {
            break;
        }
    }

    // Calcola i coefficienti base della B-spline (Cox-de Boor)
    std::vector<Eigen::Vector3d> d(splineDegree_ + 1);
    for (int i = 0; i <= splineDegree_; ++i)
    {
        d[i] = controlPoints_[k - splineDegree_ + i];
    }

    // Applica l'algoritmo di De Boor
    for (int r = 1; r <= splineDegree_; ++r)
    {
        for (int j = splineDegree_; j >= r; --j)
        {
            double alpha = (u - knots_[k - splineDegree_ + j]) /
                           (knots_[k + j - r + 1] - knots_[k - splineDegree_ + j]);
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
        }
    }

    return d[splineDegree_];
}

Eigen::Vector3d GeometricPath::evaluateLinearPathDerivative(double parameter, int order) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Percorso lineare non inizializzato correttamente");
    }

    if (order == 1)
    {
        // Prima derivata: velocità costante
        return controlPoints_[1] - controlPoints_[0];
    }
    else if (order > 1)
    {
        // Derivate di ordine superiore sono zero per un percorso lineare
        return Eigen::Vector3d::Zero();
    }

    throw std::invalid_argument("Ordine della derivata deve essere positivo");
}

Eigen::Vector3d GeometricPath::evaluateCircularPathDerivative(double parameter, int order) const
{
    if (!circleCenter_ || !circleNormal_ || !circleRadius_ || !arcAngle_ ||
        controlPoints_.size() < 3)
    {
        throw std::runtime_error("Percorso circolare non inizializzato correttamente");
    }

    // Calcola l'angolo corrente
    double angle = (*arcAngle_) * parameter;

    // Ottieni i vettori base
    const Eigen::Vector3d &radialDir = controlPoints_[0];
    const Eigen::Vector3d &tangentDir = controlPoints_[1];

    if (order == 1)
    {
        // Prima derivata: tangente alla traiettoria circolare
        double angleVelocity = *arcAngle_; // Velocità angolare
        return (*circleRadius_) * angleVelocity *
               (-radialDir * std::sin(angle) + tangentDir * std::cos(angle));
    }
    else if (order == 2)
    {
        // Seconda derivata: accelerazione centripeta
        double angleVelocity = *arcAngle_; // Velocità angolare
        return -(*circleRadius_) * angleVelocity * angleVelocity *
               (radialDir * std::cos(angle) + tangentDir * std::sin(angle));
    }
    else if (order > 2)
    {
        // Derivate di ordine superiore seguono un pattern simile
        if (order % 2 == 1)
        {
            // Ordine dispari
            double angleVelocity = std::pow(*arcAngle_, order);
            return (*circleRadius_) * angleVelocity *
                   (-radialDir * std::sin(angle) + tangentDir * std::cos(angle)) *
                   ((order % 4 == 1) ? 1.0 : -1.0);
        }
        else
        {
            // Ordine pari
            double angleVelocity = std::pow(*arcAngle_, order);
            return -(*circleRadius_) * angleVelocity *
                   (radialDir * std::cos(angle) + tangentDir * std::sin(angle)) *
                   ((order % 4 == 0) ? 1.0 : -1.0);
        }
    }

    throw std::invalid_argument("Ordine della derivata deve essere positivo");
}

Eigen::Vector3d GeometricPath::evaluateCubicSplinePathDerivative(double parameter, int order) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Percorso con spline cubica non inizializzato correttamente");
    }

    const size_t n = controlPoints_.size() - 1;

    // Trova il segmento appropriato
    const size_t i = std::min(static_cast<size_t>(parameter * n), n - 1);

    // Parametro locale all'interno del segmento
    const double t = parameter * n - i;

    if (order == 1)
    {
        // Prima derivata della spline cubica
        double t2 = t * t;

        // Derivate dei coefficienti di Hermite
        double h00_prime = 6 * t2 - 6 * t;
        double h10_prime = 3 * t2 - 4 * t + 1;
        double h01_prime = -6 * t2 + 6 * t;
        double h11_prime = 3 * t2 - 2 * t;

        // Calcola tangenti
        Eigen::Vector3d tangent0, tangent1;
        if (i == 0)
        {
            tangent0 = (controlPoints_[1] - controlPoints_[0]) * 0.5;
        }
        else
        {
            tangent0 = (controlPoints_[i + 1] - controlPoints_[i - 1]) * 0.5;
        }

        if (i == n - 1)
        {
            tangent1 = (controlPoints_[n] - controlPoints_[n - 1]) * 0.5;
        }
        else
        {
            tangent1 = (controlPoints_[i + 2] - controlPoints_[i]) * 0.5;
        }

        // Moltiplica per n per tenere conto della parametrizzazione
        return n * (h00_prime * controlPoints_[i] + h10_prime * tangent0 +
                    h01_prime * controlPoints_[i + 1] + h11_prime * tangent1);
    }
    else if (order == 2)
    {
        // Seconda derivata della spline cubica

        // Derivate seconde dei coefficienti di Hermite
        double h00_second = 12 * t - 6;
        double h10_second = 6 * t - 4;
        double h01_second = -12 * t + 6;
        double h11_second = 6 * t - 2;

        // Calcola tangenti
        Eigen::Vector3d tangent0, tangent1;
        if (i == 0)
        {
            tangent0 = (controlPoints_[1] - controlPoints_[0]) * 0.5;
        }
        else
        {
            tangent0 = (controlPoints_[i + 1] - controlPoints_[i - 1]) * 0.5;
        }

        if (i == n - 1)
        {
            tangent1 = (controlPoints_[n] - controlPoints_[n - 1]) * 0.5;
        }
        else
        {
            tangent1 = (controlPoints_[i + 2] - controlPoints_[i]) * 0.5;
        }

        // Moltiplica per n^2 per tenere conto della parametrizzazione
        return n * n *
               (h00_second * controlPoints_[i] + h10_second * tangent0 +
                h01_second * controlPoints_[i + 1] + h11_second * tangent1);
    }
    else if (order == 3)
    {
        // Terza derivata della spline cubica (costante all'interno di ogni segmento)

        // Derivate terze dei coefficienti di Hermite
        double h00_third = 12;
        double h10_third = 6;
        double h01_third = -12;
        double h11_third = 6;

        // Calcola tangenti
        Eigen::Vector3d tangent0, tangent1;
        if (i == 0)
        {
            tangent0 = (controlPoints_[1] - controlPoints_[0]) * 0.5;
        }
        else
        {
            tangent0 = (controlPoints_[i + 1] - controlPoints_[i - 1]) * 0.5;
        }

        if (i == n - 1)
        {
            tangent1 = (controlPoints_[n] - controlPoints_[n - 1]) * 0.5;
        }
        else
        {
            tangent1 = (controlPoints_[i + 2] - controlPoints_[i]) * 0.5;
        }

        // Moltiplica per n^3 per tenere conto della parametrizzazione
        return n * n * n *
               (h00_third * controlPoints_[i] + h10_third * tangent0 +
                h01_third * controlPoints_[i + 1] + h11_third * tangent1);
    }
    else if (order > 3)
    {
        // Derivate di ordine superiore a 3 sono zero per una spline cubica
        return Eigen::Vector3d::Zero();
    }

    throw std::invalid_argument("Ordine della derivata deve essere positivo");
}

Eigen::Vector3d GeometricPath::evaluateBSplinePathDerivative(double parameter, int order) const
{
    if (controlPoints_.size() < splineDegree_ + 1 ||
        knots_.size() < controlPoints_.size() + splineDegree_ + 1)
    {
        throw std::runtime_error("Percorso con B-spline non inizializzato correttamente");
    }

    // Implementazione semplificata della derivata di una B-spline
    // In un'implementazione completa, qui ci sarebbe l'algoritmo di de Boor per le derivate

    // Per semplicità, utilizziamo una differenza finita per le derivate
    const double h = 1e-6; // Piccolo incremento per la differenza finita

    if (order == 1)
    {
        // Prima derivata (differenza in avanti)
        Eigen::Vector3d pos1 = evaluateBSplinePath(parameter);
        Eigen::Vector3d pos2 = evaluateBSplinePath(std::min(parameter + h, 1.0));
        return (pos2 - pos1) / h;
    }
    else if (order == 2)
    {
        // Seconda derivata (differenza centrale)
        Eigen::Vector3d pos1 = evaluateBSplinePath(std::max(parameter - h, 0.0));
        Eigen::Vector3d pos2 = evaluateBSplinePath(parameter);
        Eigen::Vector3d pos3 = evaluateBSplinePath(std::min(parameter + h, 1.0));
        return (pos3 - 2 * pos2 + pos1) / (h * h);
    }
    else if (order == 3)
    {
        // Terza derivata (differenza in avanti della seconda derivata)
        Eigen::Vector3d d2_1 = evaluateBSplinePathDerivative(parameter, 2);
        Eigen::Vector3d d2_2 = evaluateBSplinePathDerivative(std::min(parameter + h, 1.0), 2);
        return (d2_2 - d2_1) / h;
    }
    else if (order > 3)
    {
        // Per ordini più elevati, applicare ricorsivamente
        Eigen::Vector3d d_lower_1 = evaluateBSplinePathDerivative(parameter, order - 1);
        Eigen::Vector3d d_lower_2 =
            evaluateBSplinePathDerivative(std::min(parameter + h, 1.0), order - 1);
        return (d_lower_2 - d_lower_1) / h;
    }

    throw std::invalid_argument("Ordine della derivata deve essere positivo");
}


//
// Implementazione della classe MotionLaw
//

MotionLaw::MotionLaw(MotionLawType type) : motionType_(type)
{
    // Inizializza la legge di moto in base al tipo
    switch (motionType_)
    {
    case MotionLawType::CUBIC_POLYNOMIAL:
        initCubicPolynomial();
        break;
    case MotionLawType::QUINTIC_POLYNOMIAL:
        initQuinticPolynomial();
        break;
    case MotionLawType::TRAPEZOIDAL:
        initTrapezoidal();
        break;
    // Gli altri tipi richiedono parametri aggiuntivi e verranno inizializzati separatamente
    default:
        break;
    }
}

MotionLaw::MotionLaw(const MotionLaw &other)
    : motionType_(other.motionType_), duration_(other.duration_), maxVelocity_(other.maxVelocity_),
      maxAcceleration_(other.maxAcceleration_), maxJerk_(other.maxJerk_),
      initialVelocity_(other.initialVelocity_), finalVelocity_(other.finalVelocity_),
      initialAcceleration_(other.initialAcceleration_),
      finalAcceleration_(other.finalAcceleration_), accelTime_(other.accelTime_),
      decelTime_(other.decelTime_), constVelTime_(other.constVelTime_),
      coefficients_(other.coefficients_), knotVector_(other.knotVector_),
      controlPoints_(other.controlPoints_), splineDegree_(other.splineDegree_)
{
}

MotionLaw::MotionLaw(MotionLaw &&other) noexcept
    : motionType_(std::move(other.motionType_)), duration_(std::move(other.duration_)),
      maxVelocity_(std::move(other.maxVelocity_)),
      maxAcceleration_(std::move(other.maxAcceleration_)), maxJerk_(std::move(other.maxJerk_)),
      initialVelocity_(std::move(other.initialVelocity_)),
      finalVelocity_(std::move(other.finalVelocity_)),
      initialAcceleration_(std::move(other.initialAcceleration_)),
      finalAcceleration_(std::move(other.finalAcceleration_)),
      accelTime_(std::move(other.accelTime_)), decelTime_(std::move(other.decelTime_)),
      constVelTime_(std::move(other.constVelTime_)), coefficients_(std::move(other.coefficients_)),
      knotVector_(std::move(other.knotVector_)), controlPoints_(std::move(other.controlPoints_)),
      splineDegree_(std::move(other.splineDegree_))
{
}

MotionLaw &MotionLaw::operator=(const MotionLaw &other)
{
    if (this != &other)
    {
        motionType_ = other.motionType_;
        duration_ = other.duration_;
        maxVelocity_ = other.maxVelocity_;
        maxAcceleration_ = other.maxAcceleration_;
        maxJerk_ = other.maxJerk_;
        initialVelocity_ = other.initialVelocity_;
        finalVelocity_ = other.finalVelocity_;
        initialAcceleration_ = other.initialAcceleration_;
        finalAcceleration_ = other.finalAcceleration_;
        accelTime_ = other.accelTime_;
        decelTime_ = other.decelTime_;
        constVelTime_ = other.constVelTime_;
        coefficients_ = other.coefficients_;
        knotVector_ = other.knotVector_;
        controlPoints_ = other.controlPoints_;
        splineDegree_ = other.splineDegree_;
    }
    return *this;
}

MotionLaw &MotionLaw::operator=(MotionLaw &&other) noexcept
{
    if (this != &other)
    {
        motionType_ = std::move(other.motionType_);
        duration_ = std::move(other.duration_);
        maxVelocity_ = std::move(other.maxVelocity_);
        maxAcceleration_ = std::move(other.maxAcceleration_);
        maxJerk_ = std::move(other.maxJerk_);
        initialVelocity_ = std::move(other.initialVelocity_);
        finalVelocity_ = std::move(other.finalVelocity_);
        initialAcceleration_ = std::move(other.initialAcceleration_);
        finalAcceleration_ = std::move(other.finalAcceleration_);
        accelTime_ = std::move(other.accelTime_);
        decelTime_ = std::move(other.decelTime_);
        constVelTime_ = std::move(other.constVelTime_);
        coefficients_ = std::move(other.coefficients_);
        knotVector_ = std::move(other.knotVector_);
        controlPoints_ = std::move(other.controlPoints_);
        splineDegree_ = std::move(other.splineDegree_);
    }
    return *this;
}

void MotionLaw::setBoundaryConditions(double initialVelocity,
                                      double finalVelocity,
                                      double initialAcceleration,
                                      double finalAcceleration)
{
    initialVelocity_ = initialVelocity;
    finalVelocity_ = finalVelocity;
    initialAcceleration_ = initialAcceleration;
    finalAcceleration_ = finalAcceleration;

    // Reinizializza la legge di moto con le nuove condizioni al contorno
    switch (motionType_)
    {
    case MotionLawType::CUBIC_POLYNOMIAL:
        initCubicPolynomial();
        break;
    case MotionLawType::QUINTIC_POLYNOMIAL:
        initQuinticPolynomial();
        break;
    case MotionLawType::TRAPEZOIDAL:
        initTrapezoidal();
        break;
    default:
        break;
    }
}

double MotionLaw::evaluateParameter(double time) const
{
    // Assicura che il tempo sia nell'intervallo [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalizza il tempo
    double normalizedTime = time / duration_;

    switch (motionType_)
    {
    case MotionLawType::CUBIC_POLYNOMIAL:
        return evaluateCubicPolynomial(normalizedTime);
    case MotionLawType::QUINTIC_POLYNOMIAL:
        return evaluateQuinticPolynomial(normalizedTime);
    case MotionLawType::TRAPEZOIDAL:
        return evaluateTrapezoidal(normalizedTime);
    case MotionLawType::CUBIC_SPLINE:
        return evaluateCubicSpline(normalizedTime);
    case MotionLawType::B_SPLINE:
        return evaluateBSpline(normalizedTime);
    default:
        throw std::runtime_error("Tipo di legge di moto non supportato");
    }
}

double MotionLaw::evaluateFirstDerivative(double time) const
{
    // Assicura che il tempo sia nell'intervallo [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalizza il tempo
    double normalizedTime = time / duration_;

    double derivative = 0.0;

    switch (motionType_)
    {
    case MotionLawType::CUBIC_POLYNOMIAL:
        derivative = evaluateCubicPolynomialDerivative(normalizedTime, 1);
        break;
    case MotionLawType::QUINTIC_POLYNOMIAL:
        derivative = evaluateQuinticPolynomialDerivative(normalizedTime, 1);
        break;
    case MotionLawType::TRAPEZOIDAL:
        derivative = evaluateTrapezoidalDerivative(normalizedTime, 1);
        break;
    case MotionLawType::CUBIC_SPLINE:
        derivative = evaluateCubicSplineDerivative(normalizedTime, 1);
        break;
    case MotionLawType::B_SPLINE:
        derivative = evaluateBSplineDerivative(normalizedTime, 1);
        break;
    default:
        throw std::runtime_error("Tipo di legge di moto non supportato");
    }

    // Scala per tenere conto della durata
    return derivative / duration_;
}

double MotionLaw::evaluateSecondDerivative(double time) const
{
    // Assicura che il tempo sia nell'intervallo [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalizza il tempo
    double normalizedTime = time / duration_;

    double derivative = 0.0;

    switch (motionType_)
    {
    case MotionLawType::CUBIC_POLYNOMIAL:
        derivative = evaluateCubicPolynomialDerivative(normalizedTime, 2);
        break;
    case MotionLawType::QUINTIC_POLYNOMIAL:
        derivative = evaluateQuinticPolynomialDerivative(normalizedTime, 2);
        break;
    case MotionLawType::TRAPEZOIDAL:
        derivative = evaluateTrapezoidalDerivative(normalizedTime, 2);
        break;
    case MotionLawType::CUBIC_SPLINE:
        derivative = evaluateCubicSplineDerivative(normalizedTime, 2);
        break;
    case MotionLawType::B_SPLINE:
        derivative = evaluateBSplineDerivative(normalizedTime, 2);
        break;
    default:
        throw std::runtime_error("Tipo di legge di moto non supportato");
    }

    // Scala per tenere conto della durata
    return derivative / (duration_ * duration_);
}

double MotionLaw::evaluateThirdDerivative(double time) const
{
    // Assicura che il tempo sia nell'intervallo [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalizza il tempo
    double normalizedTime = time / duration_;

    double derivative = 0.0;

    switch (motionType_)
    {
    case MotionLawType::CUBIC_POLYNOMIAL:
        derivative = evaluateCubicPolynomialDerivative(normalizedTime, 3);
        break;
    case MotionLawType::QUINTIC_POLYNOMIAL:
        derivative = evaluateQuinticPolynomialDerivative(normalizedTime, 3);
        break;
    case MotionLawType::TRAPEZOIDAL:
        derivative = evaluateTrapezoidalDerivative(normalizedTime, 3);
        break;
    case MotionLawType::CUBIC_SPLINE:
        derivative = evaluateCubicSplineDerivative(normalizedTime, 3);
        break;
    case MotionLawType::B_SPLINE:
        derivative = evaluateBSplineDerivative(normalizedTime, 3);
        break;
    default:
        throw std::runtime_error("Tipo di legge di moto non supportato");
    }

    // Scala per tenere conto della durata
    return derivative / (duration_ * duration_ * duration_);
}

MotionLaw MotionLaw::createCubicPolynomial(double duration,
                                           double initialVelocity,
                                           double finalVelocity)
{
    MotionLaw motionLaw(MotionLawType::CUBIC_POLYNOMIAL);
    motionLaw.duration_ = duration;
    motionLaw.initialVelocity_ = initialVelocity;
    motionLaw.finalVelocity_ = finalVelocity;
    motionLaw.initCubicPolynomial();
    return motionLaw;
}

MotionLaw MotionLaw::createQuinticPolynomial(double duration,
                                             double initialVelocity,
                                             double finalVelocity,
                                             double initialAcceleration,
                                             double finalAcceleration)
{
    MotionLaw motionLaw(MotionLawType::QUINTIC_POLYNOMIAL);
    motionLaw.duration_ = duration;
    motionLaw.initialVelocity_ = initialVelocity;
    motionLaw.finalVelocity_ = finalVelocity;
    motionLaw.initialAcceleration_ = initialAcceleration;
    motionLaw.finalAcceleration_ = finalAcceleration;
    motionLaw.initQuinticPolynomial();
    return motionLaw;
}

MotionLaw MotionLaw::createTrapezoidal(double duration, double maxVelocity, double maxAcceleration)
{
    MotionLaw motionLaw(MotionLawType::TRAPEZOIDAL);
    motionLaw.duration_ = duration;
    motionLaw.maxVelocity_ = maxVelocity;
    motionLaw.maxAcceleration_ = maxAcceleration;
    motionLaw.initTrapezoidal();
    return motionLaw;
}

double MotionLaw::evaluateCubicPolynomial(double time) const
{
    if (coefficients_.size() != 4)
    {
        throw std::runtime_error(
            "Coefficienti del polinomio cubico non inizializzati correttamente");
    }

    double t2 = time * time;
    double t3 = t2 * time;

    return coefficients_[0] + coefficients_[1] * time + coefficients_[2] * t2 +
           coefficients_[3] * t3;
}

double MotionLaw::evaluateQuinticPolynomial(double time) const
{
    if (coefficients_.size() != 6)
    {
        throw std::runtime_error(
            "Coefficienti del polinomio quintico non inizializzati correttamente");
    }

    double t2 = time * time;
    double t3 = t2 * time;
    double t4 = t3 * time;
    double t5 = t4 * time;

    return coefficients_[0] + coefficients_[1] * time + coefficients_[2] * t2 +
           coefficients_[3] * t3 + coefficients_[4] * t4 + coefficients_[5] * t5;
}

double MotionLaw::evaluateTrapezoidal(double time) const
{
    if (!maxVelocity_ || !maxAcceleration_ || accelTime_ == 0.0)
    {
        throw std::runtime_error("Legge di moto trapezoidale non inizializzata correttamente");
    }

    // Fase di accelerazione
    if (time < accelTime_)
    {
        return 0.5 * (*maxAcceleration_) * time * time;
    }
    // Fase a velocità costante
    else if (time < accelTime_ + constVelTime_)
    {
        return 0.5 * (*maxAcceleration_) * accelTime_ * accelTime_ +
               (*maxVelocity_) * (time - accelTime_);
    }
    // Fase di decelerazione
    else
    {
        double t = time - (accelTime_ + constVelTime_);
        return 0.5 * (*maxAcceleration_) * accelTime_ * accelTime_ +
               (*maxVelocity_) * constVelTime_ + (*maxVelocity_) * t -
               0.5 * (*maxAcceleration_) * t * t;
    }
}

double MotionLaw::evaluateCubicSpline(double time) const
{
    // Implementazione semplificata
    // In un'implementazione completa, qui ci sarebbe una vera spline cubica

    // Usiamo il polinomio cubico come approssimazione
    return evaluateCubicPolynomial(time);
}

double MotionLaw::evaluateBSpline(double time) const
{
    // Implementazione semplificata
    // In un'implementazione completa, qui ci sarebbe una vera B-spline

    // Usiamo il polinomio quintico come approssimazione
    return evaluateQuinticPolynomial(time);
}

double MotionLaw::evaluateCubicPolynomialDerivative(double time, int order) const
{
    if (coefficients_.size() != 4)
    {
        throw std::runtime_error(
            "Coefficienti del polinomio cubico non inizializzati correttamente");
    }

    if (order == 1)
    {
        double t2 = time * time;
        return coefficients_[1] + 2 * coefficients_[2] * time + 3 * coefficients_[3] * t2;
    }
    else if (order == 2)
    {
        return 2 * coefficients_[2] + 6 * coefficients_[3] * time;
    }
    else if (order == 3)
    {
        return 6 * coefficients_[3];
    }
    else if (order > 3)
    {
        return 0.0; // Derivate di ordine superiore a 3 sono zero per un polinomio cubico
    }

    throw std::invalid_argument("Ordine della derivata deve essere positivo");
}

double MotionLaw::evaluateQuinticPolynomialDerivative(double time, int order) const
{
    if (coefficients_.size() != 6)
    {
        throw std::runtime_error(
            "Coefficienti del polinomio quintico non inizializzati correttamente");
    }

    if (order == 1)
    {
        double t2 = time * time;
        double t3 = t2 * time;
        double t4 = t3 * time;
        return coefficients_[1] + 2 * coefficients_[2] * time + 3 * coefficients_[3] * t2 +
               4 * coefficients_[4] * t3 + 5 * coefficients_[5] * t4;
    }
    else if (order == 2)
    {
        double t2 = time * time;
        double t3 = t2 * time;
        return 2 * coefficients_[2] + 6 * coefficients_[3] * time + 12 * coefficients_[4] * t2 +
               20 * coefficients_[5] * t3;
    }
    else if (order == 3)
    {
        double t2 = time * time;
        return 6 * coefficients_[3] + 24 * coefficients_[4] * time + 60 * coefficients_[5] * t2;
    }
    else if (order == 4)
    {
        return 24 * coefficients_[4] + 120 * coefficients_[5] * time;
    }
    else if (order == 5)
    {
        return 120 * coefficients_[5];
    }
    else if (order > 5)
    {
        return 0.0; // Derivate di ordine superiore a 5 sono zero per un polinomio quintico
    }

    throw std::invalid_argument("Ordine della derivata deve essere positivo");
}

double MotionLaw::evaluateTrapezoidalDerivative(double time, int order) const
{
    if (!maxVelocity_ || !maxAcceleration_ || accelTime_ == 0.0)
    {
        throw std::runtime_error("Legge di moto trapezoidale non inizializzata correttamente");
    }

    if (order == 1)
    {
        // Fase di accelerazione
        if (time < accelTime_)
        {
            return (*maxAcceleration_) * time;
        }
        // Fase a velocità costante
        else if (time < accelTime_ + constVelTime_)
        {
            return (*maxVelocity_);
        }
        // Fase di decelerazione
        else
        {
            double t = time - (accelTime_ + constVelTime_);
            return (*maxVelocity_) - (*maxAcceleration_) * t;
        }
    }
    else if (order == 2)
    {
        // Fase di accelerazione
        if (time < accelTime_)
        {
            return (*maxAcceleration_);
        }
        // Fase a velocità costante
        else if (time < accelTime_ + constVelTime_)
        {
            return 0.0;
        }
        // Fase di decelerazione
        else
        {
            return -(*maxAcceleration_);
        }
    }
    else if (order > 2)
    {
        // Le derivate di ordine superiore non sono definite nei punti di transizione
        // Qui assumiamo che siano zero, ma in un'implementazione più rigorosa
        // si potrebbero usare le distribuzioni di Dirac per rappresentare i salti
        return 0.0;
    }

    throw std::invalid_argument("Ordine della derivata deve essere positivo");
}

double MotionLaw::evaluateCubicSplineDerivative(double time, int order) const
{
    // Implementazione semplificata
    // Usiamo il polinomio cubico come approssimazione
    return evaluateCubicPolynomialDerivative(time, order);
}

double MotionLaw::evaluateBSplineDerivative(double time, int order) const
{
    // Implementazione semplificata
    // Usiamo il polinomio quintico come approssimazione
    return evaluateQuinticPolynomialDerivative(time, order);
}

void MotionLaw::initCubicPolynomial()
{
    // Calcola i coefficienti del polinomio cubico a(t) = a0 + a1*t + a2*t^2 + a3*t^3
    // con condizioni al contorno a(0) = 0, a(1) = 1, a'(0) = v0, a'(1) = v1

    double v0 = initialVelocity_ * duration_; // Velocità normalizzata
    double v1 = finalVelocity_ * duration_;   // Velocità normalizzata

    coefficients_.resize(4);
    coefficients_[0] = 0.0;                 // a0 = a(0) = 0
    coefficients_[1] = v0;                  // a1 = a'(0) = v0
    coefficients_[2] = 3.0 - 2.0 * v0 - v1; // a2
    coefficients_[3] = v0 + v1 - 2.0;       // a3
}

void MotionLaw::initQuinticPolynomial()
{
    // Calcola i coefficienti del polinomio quintico
    // a(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // con condizioni al contorno a(0) = 0, a(1) = 1, a'(0) = v0, a'(1) = v1, a''(0) = a0, a''(1) = a1

    double v0 = initialVelocity_ * duration_;                 // Velocità normalizzata
    double v1 = finalVelocity_ * duration_;                   // Velocità normalizzata
    double a0 = initialAcceleration_ * duration_ * duration_; // Accelerazione normalizzata
    double a1 = finalAcceleration_ * duration_ * duration_;   // Accelerazione normalizzata

    coefficients_.resize(6);
    coefficients_[0] = 0.0;      // a0 = a(0) = 0
    coefficients_[1] = v0;       // a1 = a'(0) = v0
    coefficients_[2] = a0 / 2.0; // a2 = a''(0) / 2 = a0 / 2
    coefficients_[3] = 10.0 - 6.0 * v0 - 3.0 * a0 / 2.0 - 4.0 * v1 + a1 / 2.0; // a3
    coefficients_[4] = -15.0 + 8.0 * v0 + 3.0 * a0 / 2.0 + 7.0 * v1 - a1;      // a4
    coefficients_[5] = 6.0 - 3.0 * v0 - a0 / 2.0 - 3.0 * v1 + a1 / 2.0;        // a5
}

void MotionLaw::initTrapezoidal()
{
    if (!maxVelocity_ || !maxAcceleration_)
    {
        throw std::runtime_error("Velocità massima e accelerazione massima devono essere definite "
                                 "per la legge trapezoidale");
    }

    // Calcola i tempi di accelerazione e decelerazione
    double t_accel = (*maxVelocity_) / (*maxAcceleration_);

    // Verifica se il profilo è triangolare (non c'è fase a velocità costante)
    if (2.0 * t_accel > duration_)
    {
        // Profilo triangolare
        t_accel = duration_ / 2.0;
        // Aggiorna la velocità massima
        *maxVelocity_ = (*maxAcceleration_) * t_accel;
    }

    accelTime_ = t_accel;
    decelTime_ = t_accel;
    constVelTime_ = duration_ - accelTime_ - decelTime_;
}


//
// Implementazione della classe Trajectory
//

Trajectory::Trajectory(const std::string &name)
    : name_(name), geometricPath_(GeometricPathType::LINEAR),
      motionLaw_(MotionLawType::CUBIC_POLYNOMIAL)
{
}

Trajectory::Trajectory(const Trajectory &other)
    : name_(other.name_), waypoints_(other.waypoints_), duration_(other.duration_),
      geometricPath_(other.geometricPath_), motionLaw_(other.motionLaw_),
      orientationInterpolationType_(other.orientationInterpolationType_),
      maxVelocity_(other.maxVelocity_), maxAcceleration_(other.maxAcceleration_),
      maxJerk_(other.maxJerk_), trajectoryGenerated_(other.trajectoryGenerated_),
      segments_(other.segments_)
{
}

Trajectory::Trajectory(Trajectory &&other) noexcept
    : name_(std::move(other.name_)), waypoints_(std::move(other.waypoints_)),
      duration_(std::move(other.duration_)), geometricPath_(std::move(other.geometricPath_)),
      motionLaw_(std::move(other.motionLaw_)),
      orientationInterpolationType_(std::move(other.orientationInterpolationType_)),
      maxVelocity_(std::move(other.maxVelocity_)),
      maxAcceleration_(std::move(other.maxAcceleration_)), maxJerk_(std::move(other.maxJerk_)),
      trajectoryGenerated_(std::move(other.trajectoryGenerated_)),
      segments_(std::move(other.segments_))
{
}

Trajectory &Trajectory::operator=(const Trajectory &other)
{
    if (this != &other)
    {
        name_ = other.name_;
        waypoints_ = other.waypoints_;
        duration_ = other.duration_;
        geometricPath_ = other.geometricPath_;
        motionLaw_ = other.motionLaw_;
        orientationInterpolationType_ = other.orientationInterpolationType_;
        maxVelocity_ = other.maxVelocity_;
        maxAcceleration_ = other.maxAcceleration_;
        maxJerk_ = other.maxJerk_;
        trajectoryGenerated_ = other.trajectoryGenerated_;
        segments_ = other.segments_;
    }
    return *this;
}

Trajectory &Trajectory::operator=(Trajectory &&other) noexcept
{
    if (this != &other)
    {
        name_ = std::move(other.name_);
        waypoints_ = std::move(other.waypoints_);
        duration_ = std::move(other.duration_);
        geometricPath_ = std::move(other.geometricPath_);
        motionLaw_ = std::move(other.motionLaw_);
        orientationInterpolationType_ = std::move(other.orientationInterpolationType_);
        maxVelocity_ = std::move(other.maxVelocity_);
        maxAcceleration_ = std::move(other.maxAcceleration_);
        maxJerk_ = std::move(other.maxJerk_);
        trajectoryGenerated_ = std::move(other.trajectoryGenerated_);
        segments_ = std::move(other.segments_);
    }
    return *this;
}

void Trajectory::addWaypoint(const Waypoint &waypoint)
{
    waypoints_.push_back(waypoint);
    trajectoryGenerated_ = false; // Invalida la traiettoria generata
}

Waypoint &Trajectory::getWaypoint(size_t index)
{
    if (index >= waypoints_.size())
    {
        throw std::out_of_range("Indice del waypoint fuori range");
    }
    return waypoints_[index];
}

const Waypoint &Trajectory::getWaypoint(size_t index) const
{
    if (index >= waypoints_.size())
    {
        throw std::out_of_range("Indice del waypoint fuori range");
    }
    return waypoints_[index];
}

void Trajectory::setGeometricPathType(GeometricPathType type)
{
    geometricPath_.setPathType(type);
    trajectoryGenerated_ = false; // Invalida la traiettoria generata
}

void Trajectory::configureCircularPath(const Eigen::Vector3d &center,
                                       const Eigen::Vector3d &normal,
                                       double radius,
                                       double arcAngle)
{
    geometricPath_.setPathType(GeometricPathType::CIRCULAR);
    geometricPath_.setCircleCenter(center);
    geometricPath_.setCircleNormal(normal);
    geometricPath_.setCircleRadius(radius);
    geometricPath_.setArcAngle(arcAngle);
    trajectoryGenerated_ = false; // Invalida la traiettoria generata
}

void Trajectory::setMotionLawType(MotionLawType type)
{
    motionLaw_.setMotionType(type);
    trajectoryGenerated_ = false; // Invalida la traiettoria generata
}

void Trajectory::configureMotionLaw(double duration,
                                    double maxVelocity,
                                    double maxAcceleration,
                                    double maxJerk)
{
    motionLaw_.setDuration(duration);
    motionLaw_.setMaxVelocity(maxVelocity);
    motionLaw_.setMaxAcceleration(maxAcceleration);
    motionLaw_.setMaxJerk(maxJerk);
    duration_ = duration;
    trajectoryGenerated_ = false; // Invalida la traiettoria generata
}

bool Trajectory::generateTrajectory()
{
    if (waypoints_.size() < 2)
    {
        std::cerr << "Errore: Sono necessari almeno 2 waypoints per generare una traiettoria"
                  << std::endl;
        return false;
    }

    // Calcola la durata se non è già impostata
    if (duration_ <= 0.0)
    {
        // Assegna durata in base alla distanza e alle velocità
        double totalLength = 0.0;
        for (size_t i = 1; i < waypoints_.size(); ++i)
        {
            totalLength += (waypoints_[i].getPosition() - waypoints_[i - 1].getPosition()).norm();
        }

        // Stima la durata (assumendo velocità media di 1.0 m/s)
        duration_ = totalLength;
        motionLaw_.setDuration(duration_);
    }

    // Calcola i timestamp per ogni waypoint se non già impostati
    double currentTime = 0.0;

    for (size_t i = 0; i < waypoints_.size(); ++i)
    {
        if (!waypoints_[i].getTimeStamp())
        {
            if (i == 0)
            {
                // Primo waypoint ha timestamp 0
                waypoints_[i].setTimeStamp(0.0);
            }
            else if (i == waypoints_.size() - 1)
            {
                // Ultimo waypoint ha timestamp uguale alla durata totale
                waypoints_[i].setTimeStamp(duration_);
            }
            else
            {
                // Waypoint intermedi hanno timestamp equidistanti
                waypoints_[i].setTimeStamp(duration_ * static_cast<double>(i) /
                                           (waypoints_.size() - 1));
            }
        }
    }

    // Crea segmenti tra ogni coppia di waypoints
    segments_.clear();
    for (size_t i = 0; i < waypoints_.size() - 1; ++i)
    {
        TrajectorySegment segment;
        segment.startWaypointIndex = i;
        segment.endWaypointIndex = i + 1;
        segment.startTime = *waypoints_[i].getTimeStamp();
        segment.endTime = *waypoints_[i + 1].getTimeStamp();

        // Crea percorso geometrico per il segmento
        Eigen::Vector3d startPos = waypoints_[i].getPosition();
        Eigen::Vector3d endPos = waypoints_[i + 1].getPosition();

        if (geometricPath_.getPathType() == GeometricPathType::LINEAR)
        {
            segment.path = GeometricPath::createLinePath(startPos, endPos);
        }
        else if (geometricPath_.getPathType() == GeometricPathType::CIRCULAR)
        {
            // Per un percorso circolare, usa la configurazione globale
            segment.path = geometricPath_;
        }
        else if (geometricPath_.getPathType() == GeometricPathType::SPLINE_CUBIC)
        {
            // Per una spline cubica, usa i waypoints come punti di controllo
            std::vector<Eigen::Vector3d> controlPoints;
            for (size_t j = i; j <= i + 1; ++j)
            {
                controlPoints.push_back(waypoints_[j].getPosition());
            }
            segment.path = GeometricPath::createCubicSplinePath(controlPoints);
        }
        else if (geometricPath_.getPathType() == GeometricPathType::SPLINE_BSPLINE)
        {
            // Per una B-spline, usa i waypoints come punti di controllo
            std::vector<Eigen::Vector3d> controlPoints;
            for (size_t j = i; j <= i + 1; ++j)
            {
                controlPoints.push_back(waypoints_[j].getPosition());
            }
            segment.path = GeometricPath::createBSplinePath(controlPoints, 3);
        }
        else
        {
            // Tipo di percorso non supportato, usa lineare come fallback
            segment.path = GeometricPath::createLinePath(startPos, endPos);
        }

        // Crea legge di moto per il segmento
        double segmentDuration = segment.endTime - segment.startTime;

        if (motionLaw_.getMotionType() == MotionLawType::CUBIC_POLYNOMIAL)
        {
            double initialVel = 0.0;
            double finalVel = 0.0;

            // Usa le velocità dai waypoints se disponibili
            if (waypoints_[i].getLinearVelocity() && waypoints_[i + 1].getLinearVelocity())
            {
                initialVel = waypoints_[i].getLinearVelocity()->norm();
                finalVel = waypoints_[i + 1].getLinearVelocity()->norm();
            }

            segment.motion =
                MotionLaw::createCubicPolynomial(segmentDuration, initialVel, finalVel);
        }
        else if (motionLaw_.getMotionType() == MotionLawType::QUINTIC_POLYNOMIAL)
        {
            double initialVel = 0.0;
            double finalVel = 0.0;
            double initialAcc = 0.0;
            double finalAcc = 0.0;

            // Usa le velocità e accelerazioni dai waypoints se disponibili
            if (waypoints_[i].getLinearVelocity() && waypoints_[i + 1].getLinearVelocity())
            {
                initialVel = waypoints_[i].getLinearVelocity()->norm();
                finalVel = waypoints_[i + 1].getLinearVelocity()->norm();
            }

            if (waypoints_[i].getLinearAcceleration() && waypoints_[i + 1].getLinearAcceleration())
            {
                initialAcc = waypoints_[i].getLinearAcceleration()->norm();
                finalAcc = waypoints_[i + 1].getLinearAcceleration()->norm();
            }

            segment.motion = MotionLaw::createQuinticPolynomial(segmentDuration,
                                                                initialVel,
                                                                finalVel,
                                                                initialAcc,
                                                                finalAcc);
        }
        else if (motionLaw_.getMotionType() == MotionLawType::TRAPEZOIDAL)
        {
            double maxVel = 1.0; // Valore predefinito
            double maxAcc = 2.0; // Valore predefinito

            if (maxVelocity_)
            {
                maxVel = maxVelocity_->norm();
            }

            if (maxAcceleration_)
            {
                maxAcc = maxAcceleration_->norm();
            }

            segment.motion = MotionLaw::createTrapezoidal(segmentDuration, maxVel, maxAcc);
        }
        else
        {
            // Tipo di legge di moto non supportato, usa cubica come fallback
            segment.motion = MotionLaw::createCubicPolynomial(segmentDuration, 0.0, 0.0);
        }

        segments_.push_back(segment);
    }

    trajectoryGenerated_ = true;
    return true;
}

Waypoint Trajectory::evaluateAtTime(double time) const
{
    if (!trajectoryGenerated_)
    {
        throw std::runtime_error(
            "La traiettoria non è stata generata. Chiamare prima generateTrajectory()");
    }

    // Limita il tempo all'intervallo [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Trova il segmento appropriato
    size_t segmentIndex = findSegmentIndex(time);
    const TrajectorySegment &segment = segments_[segmentIndex];

    // Normalizza il tempo all'interno del segmento
    double normalizedTime = (time - segment.startTime) / (segment.endTime - segment.startTime);

    // Calcola il parametro della legge di moto
    double motionParameter = segment.motion.evaluateParameter(time - segment.startTime);

    // Calcola la posizione lungo il percorso
    Eigen::Vector3d position = segment.path.evaluatePosition(motionParameter);

    // Interpola l'orientamento
    Eigen::Quaterniond startOrientation = waypoints_[segment.startWaypointIndex].getOrientation();
    Eigen::Quaterniond endOrientation = waypoints_[segment.endWaypointIndex].getOrientation();

    Eigen::Quaterniond orientation;

    if (orientationInterpolationType_ == OrientationInterpolationType::SLERP)
    {
        orientation = interpolateSLERP(startOrientation, endOrientation, motionParameter);
    }
    else // SQUAD
    {
        // Per una vera SQUAD, dovremmo calcolare i quaternioni di controllo
        // Qui utilizziamo una doppia SLERP come approssimazione
        orientation = interpolateSLERP(startOrientation, endOrientation, motionParameter);
    }

    // Crea il waypoint risultante
    Waypoint result(position, orientation);

    // Calcola velocità, accelerazione e jerk se necessario
    double motionVelocity = segment.motion.evaluateFirstDerivative(time - segment.startTime);
    double motionAcceleration = segment.motion.evaluateSecondDerivative(time - segment.startTime);
    double motionJerk = segment.motion.evaluateThirdDerivative(time - segment.startTime);

    // Calcola le derivate del percorso
    Eigen::Vector3d pathVelocity = segment.path.evaluateDerivative(motionParameter, 1);
    Eigen::Vector3d pathAcceleration = segment.path.evaluateDerivative(motionParameter, 2);

    // Imposta velocità e accelerazione lineari
    result.setLinearVelocity(pathVelocity * motionVelocity);
    result.setLinearAcceleration(pathVelocity * motionAcceleration +
                                 pathAcceleration * motionVelocity * motionVelocity);
    result.setLinearJerk(Eigen::Vector3d(pathVelocity * motionJerk));

    // Imposta il timestamp
    result.setTimeStamp(time);

    return result;
}

std::vector<Waypoint> Trajectory::sampleTrajectory(double samplingFrequency) const
{
    if (!trajectoryGenerated_)
    {
        throw std::runtime_error(
            "La traiettoria non è stata generata. Chiamare prima generateTrajectory()");
    }

    // Calcola il numero di campioni
    double dt = 1.0 / samplingFrequency;
    size_t numSamples = static_cast<size_t>(std::ceil(duration_ * samplingFrequency)) + 1;

    // Campiona la traiettoria
    std::vector<Waypoint> samples;
    samples.reserve(numSamples);

    for (size_t i = 0; i < numSamples; ++i)
    {
        double time = i * dt;
        if (time > duration_)
        {
            time = duration_; // Assicura che l'ultimo campione sia esattamente alla fine
        }

        samples.push_back(evaluateAtTime(time));
    }

    return samples;
}

Trajectory Trajectory::concatenate(const Trajectory &other, double blendTime) const
{
    if (!trajectoryGenerated_ || !other.trajectoryGenerated_)
    {
        throw std::runtime_error(
            "Le traiettorie devono essere generate prima della concatenazione");
    }

    Trajectory result(name_ + "_" + other.name_);

    // Copia tutti i waypoints da questa traiettoria
    for (const auto &wp : waypoints_)
    {
        result.addWaypoint(wp);
    }

    // Calcola il tempo in cui inizia la seconda traiettoria
    double startTimeSecond = duration_;

    // Aggiungi waypoints dalla seconda traiettoria con timestamp aggiustati
    for (size_t i = 0; i < other.waypoints_.size(); ++i)
    {
        Waypoint wp = other.waypoints_[i];

        if (wp.getTimeStamp())
        {
            wp.setTimeStamp(*wp.getTimeStamp() + startTimeSecond);
        }

        // Salta il primo waypoint della seconda traiettoria se il tempo di fusione è zero
        if (i > 0 || blendTime == 0.0)
        {
            result.addWaypoint(wp);
        }
    }

    // Imposta durata totale
    result.setDuration(duration_ + other.duration_);

    // Genera la traiettoria
    result.setGeometricPathType(geometricPath_.getPathType());
    result.setMotionLawType(motionLaw_.getMotionType());
    result.setOrientationInterpolationType(orientationInterpolationType_);

    if (maxVelocity_)
    {
        result.setMaxVelocity(*maxVelocity_);
    }

    if (maxAcceleration_)
    {
        result.setMaxAcceleration(*maxAcceleration_);
    }

    if (maxJerk_)
    {
        result.setMaxJerk(*maxJerk_);
    }

    result.generateTrajectory();

    return result;
}

bool Trajectory::checkKinematicConstraints() const
{
    if (!trajectoryGenerated_)
    {
        throw std::runtime_error(
            "La traiettoria non è stata generata. Chiamare prima generateTrajectory()");
    }

    if (!maxVelocity_ && !maxAcceleration_ && !maxJerk_)
    {
        // Nessun vincolo da verificare
        return true;
    }

    // Campiona la traiettoria per verificare i vincoli
    const double dt = 0.01; // 100 Hz
    size_t numSamples = static_cast<size_t>(std::ceil(duration_ / dt)) + 1;

    for (size_t i = 0; i < numSamples; ++i)
    {
        double time = i * dt;
        if (time > duration_)
        {
            time = duration_;
        }

        Waypoint wp = evaluateAtTime(time);

        // Verifica i vincoli di velocità
        if (maxVelocity_ && wp.getLinearVelocity())
        {
            double vx = std::abs(wp.getLinearVelocity()->x());
            double vy = std::abs(wp.getLinearVelocity()->y());
            double vz = std::abs(wp.getLinearVelocity()->z());

            if (vx > maxVelocity_->x() || vy > maxVelocity_->y() || vz > maxVelocity_->z())
            {
                std::cerr << "Vincolo di velocità violato al tempo t = " << time << " s"
                          << std::endl;
                return false;
            }
        }

        // Verifica i vincoli di accelerazione
        if (maxAcceleration_ && wp.getLinearAcceleration())
        {
            double ax = std::abs(wp.getLinearAcceleration()->x());
            double ay = std::abs(wp.getLinearAcceleration()->y());
            double az = std::abs(wp.getLinearAcceleration()->z());

            if (ax > maxAcceleration_->x() || ay > maxAcceleration_->y() ||
                az > maxAcceleration_->z())
            {
                std::cerr << "Vincolo di accelerazione violato al tempo t = " << time << " s"
                          << std::endl;
                return false;
            }
        }

        // Verifica i vincoli di jerk
        if (maxJerk_ && wp.getLinearJerk())
        {
            double jx = std::abs(wp.getLinearJerk()->x());
            double jy = std::abs(wp.getLinearJerk()->y());
            double jz = std::abs(wp.getLinearJerk()->z());

            if (jx > maxJerk_->x() || jy > maxJerk_->y() || jz > maxJerk_->z())
            {
                std::cerr << "Vincolo di jerk violato al tempo t = " << time << " s" << std::endl;
                return false;
            }
        }
    }

    return true;
}

std::ostream &operator<<(std::ostream &os, const Trajectory &trajectory)
{
    os << "Traiettoria: " << trajectory.name_ << std::endl;
    os << "Durata: " << trajectory.duration_ << " s" << std::endl;
    os << "Numero di waypoints: " << trajectory.waypoints_.size() << std::endl;

    for (size_t i = 0; i < trajectory.waypoints_.size(); ++i)
    {
        os << "Waypoint " << i << ":" << std::endl;
        os << trajectory.waypoints_[i] << std::endl;
    }

    return os;
}

size_t Trajectory::findSegmentIndex(double time) const
{
    // Cerca il segmento che contiene il tempo specificato
    for (size_t i = 0; i < segments_.size(); ++i)
    {
        if (time >= segments_[i].startTime && time <= segments_[i].endTime)
        {
            return i;
        }
    }

    // Se il tempo è oltre la durata, restituisci l'ultimo segmento
    if (time >= duration_ && !segments_.empty())
    {
        return segments_.size() - 1;
    }

    throw std::runtime_error("Impossibile trovare un segmento per il tempo specificato");
}

Eigen::Quaterniond Trajectory::interpolateSLERP(const Eigen::Quaterniond &q1,
                                                const Eigen::Quaterniond &q2,
                                                double t) const
{
    // Usa l'implementazione di Eigen per SLERP
    return q1.slerp(t, q2);
}

Eigen::Quaterniond Trajectory::interpolateSQUAD(const Eigen::Quaterniond &q1,
                                                const Eigen::Quaterniond &q2,
                                                const Eigen::Quaterniond &a,
                                                const Eigen::Quaterniond &b,
                                                double t) const
{
    // Implementazione di SQUAD (Spherical Cubic Interpolation)
    Eigen::Quaterniond slerp1 = interpolateSLERP(q1, q2, t);
    Eigen::Quaterniond slerp2 = interpolateSLERP(a, b, t);
    return interpolateSLERP(slerp1, slerp2, 2.0 * t * (1.0 - t));
}

} // namespace MoveG

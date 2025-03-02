/**
 * @file geometric_path.cpp
 * @brief Implementation of the GeometricPath class
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#include "geometric_path.h"

namespace MoveG
{

// Default Constructor
GeometricPath::GeometricPath(GeometricPathType type) : pathType_(type)
{
}

// Copy Constructor
GeometricPath::GeometricPath(const GeometricPath &other)
    : pathType_(other.pathType_), circleCenter_(other.circleCenter_),
      circleNormal_(other.circleNormal_), circleRadius_(other.circleRadius_),
      arcAngle_(other.arcAngle_), controlPoints_(other.controlPoints_), knots_(other.knots_),
      splineDegree_(other.splineDegree_)
{
}

// Move Constructor
GeometricPath::GeometricPath(GeometricPath &&other) noexcept
    : pathType_(std::move(other.pathType_)), circleCenter_(std::move(other.circleCenter_)),
      circleNormal_(std::move(other.circleNormal_)), circleRadius_(std::move(other.circleRadius_)),
      arcAngle_(std::move(other.arcAngle_)), controlPoints_(std::move(other.controlPoints_)),
      knots_(std::move(other.knots_)), splineDegree_(std::move(other.splineDegree_))
{
}

// Copy Assignment Operator
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

// Move Assignment Operator
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

// Evaluate position on the path
Eigen::Vector3d GeometricPath::evaluatePosition(double parameter) const
{
    // Limit parameter to [0, 1] range
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
        throw std::runtime_error("Unsupported geometric path type");
    }
}

// Evaluate derivative of position
Eigen::Vector3d GeometricPath::evaluateDerivative(double parameter, int order) const
{
    // Limit parameter to [0, 1] range
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
        throw std::runtime_error("Unsupported geometric path type");
    }
}

// Calculate path length
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
        // Numerical calculation of length using sampling
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

// Create a linear path
GeometricPath GeometricPath::createLinePath(const Eigen::Vector3d &start,
                                            const Eigen::Vector3d &end)
{
    GeometricPath path(GeometricPathType::LINEAR);
    path.controlPoints_ = {start, end};
    return path;
}

// Create a circular path
GeometricPath GeometricPath::createCircularPath(const Eigen::Vector3d &center,
                                                const Eigen::Vector3d &normal,
                                                double radius,
                                                const Eigen::Vector3d &startPoint,
                                                double arcAngle)
{
    GeometricPath path(GeometricPathType::CIRCULAR);
    path.circleCenter_ = center;
    path.circleNormal_ = normal.normalized(); // Ensure normal is normalized
    path.circleRadius_ = radius;
    path.arcAngle_ = arcAngle;

    // Calculate the starting point relative to center
    Eigen::Vector3d relStart = startPoint - center;

    // Calculate radial and tangential directions
    Eigen::Vector3d radialDir = relStart.normalized();
    Eigen::Vector3d tangentDir = (*path.circleNormal_).cross(radialDir).normalized();

    // Store base vectors for calculating points on the arc
    path.controlPoints_ = {radialDir, tangentDir, *path.circleNormal_};

    return path;
}

// Create a cubic spline path
GeometricPath GeometricPath::createCubicSplinePath(
    const std::vector<Eigen::Vector3d> &controlPoints)
{
    if (controlPoints.size() < 2)
    {
        throw std::invalid_argument("At least 2 control points are required for a cubic spline");
    }

    GeometricPath path(GeometricPathType::SPLINE_CUBIC);
    path.controlPoints_ = controlPoints;

    return path;
}

// Create a B-spline path
GeometricPath GeometricPath::createBSplinePath(const std::vector<Eigen::Vector3d> &controlPoints,
                                               int degree,
                                               const std::vector<double> &knots)
{
    if (controlPoints.size() < degree + 1)
    {
        throw std::invalid_argument(
            "At least (degree + 1) control points are required for a B-spline");
    }

    GeometricPath path(GeometricPathType::SPLINE_BSPLINE);
    path.controlPoints_ = controlPoints;
    path.splineDegree_ = degree;

    // If knots are not provided, generate a uniform sequence
    if (knots.empty())
    {
        int numKnots = controlPoints.size() + degree + 1;
        path.knots_.resize(numKnots);

        // First 'degree' knots are 0
        for (int i = 0; i <= degree; ++i)
        {
            path.knots_[i] = 0.0;
        }

        // Internal knots are evenly distributed
        for (int i = 1; i < controlPoints.size() - degree; ++i)
        {
            path.knots_[i + degree] = static_cast<double>(i) / (controlPoints.size() - degree);
        }

        // Last 'degree' knots are 1
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

// Evaluate linear path
Eigen::Vector3d GeometricPath::evaluateLinearPath(double parameter) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Linear path not properly initialized");
    }

    return controlPoints_[0] + parameter * (controlPoints_[1] - controlPoints_[0]);
}

// Evaluate circular path
Eigen::Vector3d GeometricPath::evaluateCircularPath(double parameter) const
{
    if (!circleCenter_ || !circleNormal_ || !circleRadius_ || !arcAngle_ ||
        controlPoints_.size() < 3)
    {
        throw std::runtime_error("Circular path not properly initialized");
    }

    // Calculate the current angle
    double angle = (*arcAngle_) * parameter;

    // Get the base vectors previously stored
    const Eigen::Vector3d &radialDir = controlPoints_[0];
    const Eigen::Vector3d &tangentDir = controlPoints_[1];

    // Calculate the point on the arc
    Eigen::Vector3d position = *circleCenter_ + (*circleRadius_) * (radialDir * std::cos(angle) +
                                                                    tangentDir * std::sin(angle));

    return position;
}

// Evaluate cubic spline path
Eigen::Vector3d GeometricPath::evaluateCubicSplinePath(double parameter) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Cubic spline path not properly initialized");
    }

    // Basic cubic spline implementation (simplified)
    // In a complete implementation, this would use a proper cubic spline interpolation
    const size_t n = controlPoints_.size() - 1;

    // Find the appropriate segment
    const size_t i = std::min(static_cast<size_t>(parameter * n), n - 1);

    // Local parameter within the segment
    const double t = parameter * n - i;

    // Cubic interpolation between two points
    double t2 = t * t;
    double t3 = t2 * t;

    // Hermite coefficients
    double h00 = 2 * t3 - 3 * t2 + 1;
    double h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2;
    double h11 = t3 - t2;

    // Calculate tangents (in a complete implementation, these would be calculated based on continuity)
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

    // Hermite interpolation
    return h00 * controlPoints_[i] + h10 * tangent0 + h01 * controlPoints_[i + 1] + h11 * tangent1;
}

// Evaluate B-spline path
Eigen::Vector3d GeometricPath::evaluateBSplinePath(double parameter) const
{
    if (controlPoints_.size() < splineDegree_ + 1 ||
        knots_.size() < controlPoints_.size() + splineDegree_ + 1)
    {
        throw std::runtime_error("B-spline path not properly initialized");
    }

    // Simplified B-spline implementation
    // In a complete implementation, this would use the De Boor algorithm

    // Map parameter to knot range
    double u = knots_.front() + parameter * (knots_.back() - knots_.front());

    // Find the knot span index
    size_t k = splineDegree_;
    for (; k < knots_.size() - 1; ++k)
    {
        if (u <= knots_[k + 1])
        {
            break;
        }
    }

    // Calculate the basis functions (Cox-de Boor recursion)
    std::vector<Eigen::Vector3d> d(splineDegree_ + 1);
    for (int i = 0; i <= splineDegree_; ++i)
    {
        d[i] = controlPoints_[k - splineDegree_ + i];
    }

    // Apply the De Boor algorithm
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

// Evaluate linear path derivative
Eigen::Vector3d GeometricPath::evaluateLinearPathDerivative(double parameter, int order) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Linear path not properly initialized");
    }

    if (order == 1)
    {
        // First derivative: constant velocity
        return controlPoints_[1] - controlPoints_[0];
    }
    else if (order > 1)
    {
        // Higher-order derivatives are zero for a linear path
        return Eigen::Vector3d::Zero();
    }

    throw std::invalid_argument("Derivative order must be positive");
}

// Evaluate circular path derivative
Eigen::Vector3d GeometricPath::evaluateCircularPathDerivative(double parameter, int order) const
{
    if (!circleCenter_ || !circleNormal_ || !circleRadius_ || !arcAngle_ ||
        controlPoints_.size() < 3)
    {
        throw std::runtime_error("Circular path not properly initialized");
    }

    // Calculate the current angle
    double angle = (*arcAngle_) * parameter;

    // Get the base vectors previously stored
    const Eigen::Vector3d &radialDir = controlPoints_[0];
    const Eigen::Vector3d &tangentDir = controlPoints_[1];

    if (order == 1)
    {
        // First derivative: tangent to the circular trajectory
        double angleVelocity = *arcAngle_; // Angular velocity
        return (*circleRadius_) * angleVelocity *
               (-radialDir * std::sin(angle) + tangentDir * std::cos(angle));
    }
    else if (order == 2)
    {
        // Second derivative: centripetal acceleration
        double angleVelocity = *arcAngle_; // Angular velocity
        return -(*circleRadius_) * angleVelocity * angleVelocity *
               (radialDir * std::cos(angle) + tangentDir * std::sin(angle));
    }
    else if (order > 2)
    {
        // Higher-order derivatives follow a similar pattern
        if (order % 2 == 1)
        {
            // Odd order
            double angleVelocity = std::pow(*arcAngle_, order);
            return (*circleRadius_) * angleVelocity *
                   (-radialDir * std::sin(angle) + tangentDir * std::cos(angle)) *
                   ((order % 4 == 1) ? 1.0 : -1.0);
        }
        else
        {
            // Even order
            double angleVelocity = std::pow(*arcAngle_, order);
            return -(*circleRadius_) * angleVelocity *
                   (radialDir * std::cos(angle) + tangentDir * std::sin(angle)) *
                   ((order % 4 == 0) ? 1.0 : -1.0);
        }
    }

    throw std::invalid_argument("Derivative order must be positive");
}

// Evaluate cubic spline path derivative
Eigen::Vector3d GeometricPath::evaluateCubicSplinePathDerivative(double parameter, int order) const
{
    if (controlPoints_.size() < 2)
    {
        throw std::runtime_error("Cubic spline path not properly initialized");
    }

    const size_t n = controlPoints_.size() - 1;

    // Find the appropriate segment
    const size_t i = std::min(static_cast<size_t>(parameter * n), n - 1);

    // Local parameter within the segment
    const double t = parameter * n - i;

    if (order == 1)
    {
        // First derivative of cubic spline
        double t2 = t * t;

        // Derivatives of Hermite coefficients
        double h00_prime = 6 * t2 - 6 * t;
        double h10_prime = 3 * t2 - 4 * t + 1;
        double h01_prime = -6 * t2 + 6 * t;
        double h11_prime = 3 * t2 - 2 * t;

        // Calculate tangents
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

        // Multiply by n to account for parametrization
        return n * (h00_prime * controlPoints_[i] + h10_prime * tangent0 +
                    h01_prime * controlPoints_[i + 1] + h11_prime * tangent1);
    }
    else if (order == 2)
    {
        // Second derivative of cubic spline

        // Second derivatives of Hermite coefficients
        double h00_second = 12 * t - 6;
        double h10_second = 6 * t - 4;
        double h01_second = -12 * t + 6;
        double h11_second = 6 * t - 2;

        // Calculate tangents
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

        // Multiply by n^2 to account for parametrization
        return n * n *
               (h00_second * controlPoints_[i] + h10_second * tangent0 +
                h01_second * controlPoints_[i + 1] + h11_second * tangent1);
    }
    else if (order == 3)
    {
        // Third derivative of cubic spline (constant within each segment)

        // Third derivatives of Hermite coefficients
        double h00_third = 12;
        double h10_third = 6;
        double h01_third = -12;
        double h11_third = 6;

        // Calculate tangents
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

        // Multiply by n^3 to account for parametrization
        return n * n * n *
               (h00_third * controlPoints_[i] + h10_third * tangent0 +
                h01_third * controlPoints_[i + 1] + h11_third * tangent1);
    }
    else if (order > 3)
    {
        // Higher-order derivatives are zero for a cubic spline
        return Eigen::Vector3d::Zero();
    }

    throw std::invalid_argument("Derivative order must be positive");
}

// Evaluate B-spline path derivative
Eigen::Vector3d GeometricPath::evaluateBSplinePathDerivative(double parameter, int order) const
{
    if (controlPoints_.size() < splineDegree_ + 1 ||
        knots_.size() < controlPoints_.size() + splineDegree_ + 1)
    {
        throw std::runtime_error("B-spline path not properly initialized");
    }

    // Simplified implementation of B-spline derivative
    // In a complete implementation, we would use the de Boor algorithm for derivatives

    // For simplicity, use a finite difference for derivatives
    const double h = 1e-6; // Small increment for finite difference

    if (order == 1)
    {
        // First derivative (forward difference)
        Eigen::Vector3d pos1 = evaluateBSplinePath(parameter);
        Eigen::Vector3d pos2 = evaluateBSplinePath(std::min(parameter + h, 1.0));
        return (pos2 - pos1) / h;
    }
    else if (order == 2)
    {
        // Second derivative (central difference)
        Eigen::Vector3d pos1 = evaluateBSplinePath(std::max(parameter - h, 0.0));
        Eigen::Vector3d pos2 = evaluateBSplinePath(parameter);
        Eigen::Vector3d pos3 = evaluateBSplinePath(std::min(parameter + h, 1.0));
        return (pos3 - 2 * pos2 + pos1) / (h * h);
    }
    else if (order == 3)
    {
        // Third derivative (forward difference of second derivative)
        Eigen::Vector3d d2_1 = evaluateBSplinePathDerivative(parameter, 2);
        Eigen::Vector3d d2_2 = evaluateBSplinePathDerivative(std::min(parameter + h, 1.0), 2);
        return (d2_2 - d2_1) / h;
    }
    else if (order > 3)
    {
        // For higher orders, apply recursively
        Eigen::Vector3d d_lower_1 = evaluateBSplinePathDerivative(parameter, order - 1);
        Eigen::Vector3d d_lower_2 =
            evaluateBSplinePathDerivative(std::min(parameter + h, 1.0), order - 1);
        return (d_lower_2 - d_lower_1) / h;
    }

    throw std::invalid_argument("Derivative order must be positive");
}

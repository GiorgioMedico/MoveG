/**
 * @file motion_law.cpp
 * @brief Implementation of the MotionLaw class
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#include "motion_law.h"

namespace MoveG
{

// Default Constructor
MotionLaw::MotionLaw(MotionLawType type) : motionType_(type)
{
    // Initialize the motion law based on the type
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
    // Other types require additional parameters and will be initialized separately
    default:
        break;
    }
}

// Copy Constructor
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

// Move Constructor
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

// Copy Assignment Operator
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

// Move Assignment Operator
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

// Set boundary conditions
void MotionLaw::setBoundaryConditions(double initialVelocity,
                                      double finalVelocity,
                                      double initialAcceleration,
                                      double finalAcceleration)
{
    initialVelocity_ = initialVelocity;
    finalVelocity_ = finalVelocity;
    initialAcceleration_ = initialAcceleration;
    finalAcceleration_ = finalAcceleration;

    // Reinitialize the motion law with the new boundary conditions
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

// Evaluate parameter
double MotionLaw::evaluateParameter(double time) const
{
    // Ensure time is in [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalize time
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
        throw std::runtime_error("Unsupported motion law type");
    }
}

// Evaluate first derivative
double MotionLaw::evaluateFirstDerivative(double time) const
{
    // Ensure time is in [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalize time
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
        throw std::runtime_error("Unsupported motion law type");
    }

    // Scale to account for duration
    return derivative / duration_;
}

// Evaluate second derivative
double MotionLaw::evaluateSecondDerivative(double time) const
{
    // Ensure time is in [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalize time
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
        throw std::runtime_error("Unsupported motion law type");
    }

    // Scale to account for duration
    return derivative / (duration_ * duration_);
}

// Evaluate third derivative
double MotionLaw::evaluateThirdDerivative(double time) const
{
    // Ensure time is in [0, duration_]
    time = std::max(0.0, std::min(time, duration_));

    // Normalize time
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
        throw std::runtime_error("Unsupported motion law type");
    }

    // Scale to account for duration
    return derivative / (duration_ * duration_ * duration_);
}

// Create cubic polynomial motion law
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

// Create quintic polynomial motion law
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

// Create trapezoidal motion law
MotionLaw MotionLaw::createTrapezoidal(double duration, double maxVelocity, double maxAcceleration)
{
    MotionLaw motionLaw(MotionLawType::TRAPEZOIDAL);
    motionLaw.duration_ = duration;
    motionLaw.maxVelocity_ = maxVelocity;
    motionLaw.maxAcceleration_ = maxAcceleration;
    motionLaw.initTrapezoidal();
    return motionLaw;
}

// Evaluate cubic polynomial
double MotionLaw::evaluateCubicPolynomial(double time) const
{
    if (coefficients_.size() != 4)
    {
        throw std::runtime_error("Cubic polynomial coefficients not properly initialized");
    }

    double t2 = time * time;
    double t3 = t2 * time;

    return coefficients_[0] + coefficients_[1] * time + coefficients_[2] * t2 +
           coefficients_[3] * t3;
}

// Evaluate quintic polynomial
double MotionLaw::evaluateQuinticPolynomial(double time) const
{
    if (coefficients_.size() != 6)
    {
        throw std::runtime_error("Quintic polynomial coefficients not properly initialized");
    }

    double t2 = time * time;
    double t3 = t2 * time;
    double t4 = t3 * time;
    double t5 = t4 * time;

    return coefficients_[0] + coefficients_[1] * time + coefficients_[2] * t2 +
           coefficients_[3] * t3 + coefficients_[4] * t4 + coefficients_[5] * t5;
}

// Evaluate trapezoidal
double MotionLaw::evaluateTrapezoidal(double time) const
{
    if (!maxVelocity_ || !maxAcceleration_ || accelTime_ == 0.0)
    {
        throw std::runtime_error("Trapezoidal motion law not properly initialized");
    }

    // Acceleration phase
    if (time < accelTime_)
    {
        return 0.5 * (*maxAcceleration_) * time * time;
    }
    // Constant velocity phase
    else if (time < accelTime_ + constVelTime_)
    {
        return 0.5 * (*maxAcceleration_) * accelTime_ * accelTime_ +
               (*maxVelocity_) * (time - accelTime_);
    }
    // Deceleration phase
    else
    {
        double t = time - (accelTime_ + constVelTime_);
        return 0.5 * (*maxAcceleration_) * accelTime_ * accelTime_ +
               (*maxVelocity_) * constVelTime_ + (*maxVelocity_) * t -
               0.5 * (*maxAcceleration_) * t * t;
    }
}

// Evaluate cubic spline
double MotionLaw::evaluateCubicSpline(double time) const
{
    // Simplified implementation
    // In a complete implementation, this would use a proper cubic spline

    // Use cubic polynomial as an approximation
    return evaluateCubicPolynomial(time);
}

// Evaluate B-spline
double MotionLaw::evaluateBSpline(double time) const
{
    // Simplified implementation
    // In a complete implementation, this would use a proper B-spline

    // Use quintic polynomial as an approximation
    return evaluateQuinticPolynomial(time);
}

// Evaluate cubic polynomial derivative
double MotionLaw::evaluateCubicPolynomialDerivative(double time, int order) const
{
    if (coefficients_.size() != 4)
    {
        throw std::runtime_error("Cubic polynomial coefficients not properly initialized");
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
        return 0.0; // Higher-order derivatives are zero for a cubic polynomial
    }

    throw std::invalid_argument("Derivative order must be positive");
}

// Evaluate quintic polynomial derivative
double MotionLaw::evaluateQuinticPolynomialDerivative(double time, int order) const
{
    if (coefficients_.size() != 6)
    {
        throw std::runtime_error("Quintic polynomial coefficients not properly initialized");
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
        return 0.0; // Higher-order derivatives are zero for a quintic polynomial
    }

    throw std::invalid_argument("Derivative order must be positive");
}

// Evaluate trapezoidal derivative
double MotionLaw::evaluateTrapezoidalDerivative(double time, int order) const
{
    if (!maxVelocity_ || !maxAcceleration_ || accelTime_ == 0.0)
    {
        throw std::runtime_error("Trapezoidal motion law not properly initialized");
    }

    if (order == 1)
    {
        // Acceleration phase
        if (time < accelTime_)
        {
            return (*maxAcceleration_) * time;
        }
        // Constant velocity phase
        else if (time < accelTime_ + constVelTime_)
        {
            return (*maxVelocity_);
        }
        // Deceleration phase
        else
        {
            double t = time - (accelTime_ + constVelTime_);
            return (*maxVelocity_) - (*maxAcceleration_) * t;
        }
    }
    else if (order == 2)
    {
        // Acceleration phase
        if (time < accelTime_)
        {
            return (*maxAcceleration_);
        }
        // Constant velocity phase
        else if (time < accelTime_ + constVelTime_)
        {
            return 0.0;
        }
        // Deceleration phase
        else
        {
            return -(*maxAcceleration_);
        }
    }
    else if (order > 2)
    {
        // Higher-order derivatives are not defined at transition points
        // Here we assume they are zero, but in a more rigorous implementation
        // we could use Dirac delta distributions to represent the jumps
        return 0.0;
    }

    throw std::invalid_argument("Derivative order must be positive");
}

// Evaluate cubic spline derivative
double MotionLaw::evaluateCubicSplineDerivative(double time, int order) const
{
    // Simplified implementation
    // Use cubic polynomial as an approximation
    return evaluateCubicPolynomialDerivative(time, order);
}

// Evaluate B-spline derivative
double MotionLaw::evaluateBSplineDerivative(double time, int order) const
{
    // Simplified implementation
    // Use quintic polynomial as an approximation
    return evaluateQuinticPolynomialDerivative(time, order);
}

// Initialize cubic polynomial
void MotionLaw::initCubicPolynomial()
{
    // Calculate cubic polynomial coefficients a(t) = a0 + a1*t + a2*t^2 + a3*t^3
    // with boundary conditions a(0) = 0, a(1) = 1, a'(0) = v0, a'(1) = v1

    double v0 = initialVelocity_ * duration_; // Normalized velocity
    double v1 = finalVelocity_ * duration_;   // Normalized velocity

    coefficients_.resize(4);
    coefficients_[0] = 0.0;                 // a0 = a(0) = 0
    coefficients_[1] = v0;                  // a1 = a'(0) = v0
    coefficients_[2] = 3.0 - 2.0 * v0 - v1; // a2
    coefficients_[3] = v0 + v1 - 2.0;       // a3
}

// Initialize quintic polynomial
void MotionLaw::initQuinticPolynomial()
{
    // Calculate quintic polynomial coefficients
    // a(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    // with boundary conditions a(0) = 0, a(1) = 1, a'(0) = v0, a'(1) = v1, a''(0) = a0, a''(1) = a1

    double v0 = initialVelocity_ * duration_;                 // Normalized velocity
    double v1 = finalVelocity_ * duration_;                   // Normalized velocity
    double a0 = initialAcceleration_ * duration_ * duration_; // Normalized acceleration
    double a1 = finalAcceleration_ * duration_ * duration_;   // Normalized acceleration

    coefficients_.resize(6);
    coefficients_[0] = 0.0;      // a0 = a(0) = 0
    coefficients_[1] = v0;       // a1 = a'(0) = v0
    coefficients_[2] = a0 / 2.0; // a2 = a''(0) / 2 = a0 / 2
    coefficients_[3] = 10.0 - 6.0 * v0 - 3.0 * a0 / 2.0 - 4.0 * v1 + a1 / 2.0; // a3
    coefficients_[4] = -15.0 + 8.0 * v0 + 3.0 * a0 / 2.0 + 7.0 * v1 - a1;      // a4
    coefficients_[5] = 6.0 - 3.0 * v0 - a0 / 2.0 - 3.0 * v1 + a1 / 2.0;        // a5
}

// Initialize trapezoidal
void MotionLaw::initTrapezoidal()
{
    if (!maxVelocity_ || !maxAcceleration_)
    {
        throw std::runtime_error("Maximum velocity and acceleration must be defined "
                                 "for trapezoidal law");
    }

    // Calculate acceleration and deceleration times
    double t_accel = (*maxVelocity_) / (*maxAcceleration_);

    // Check if the profile is triangular (no constant velocity phase)
    if (2.0 * t_accel > duration_)
    {
        // Triangular profile
        t_accel = duration_ / 2.0;
        // Update maximum velocity
        *maxVelocity_ = (*maxAcceleration_) * t_accel;
    }

    accelTime_ = t_accel;
    decelTime_ = t_accel;
    constVelTime_ = duration_ - accelTime_ - decelTime_;
}

} // namespace MoveG

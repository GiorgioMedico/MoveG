/**
 * @file motion_law.h
 * @brief Class for representing motion laws in trajectories
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#pragma once

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace MoveG
{

/**
   * @brief Enumeration of supported motion law types
   */
enum class MotionLawType
{
    CUBIC_POLYNOMIAL,   ///< 3rd order polynomial
    QUINTIC_POLYNOMIAL, ///< 5th order polynomial
    TRAPEZOIDAL,        ///< Trapezoidal velocity profile
    CUBIC_SPLINE,       ///< Cubic spline
    B_SPLINE            ///< B-spline
};

/**
   * @class MotionLaw
   * @brief Class representing a motion law
   *
   * This class defines how velocity varies along the path,
   * implementing different motion laws such as polynomial, trapezoidal, and spline-based.
   */
class MotionLaw
{
public:
    /**
       * @brief Default constructor
       * @param type Motion law type
       */
    explicit MotionLaw(MotionLawType type = MotionLawType::CUBIC_POLYNOMIAL);

    /**
       * @brief Copy constructor
       * @param other MotionLaw to copy
       */
    MotionLaw(const MotionLaw &other);

    /**
       * @brief Move constructor
       * @param other MotionLaw to move
       */
    MotionLaw(MotionLaw &&other) noexcept;

    /**
       * @brief Copy assignment operator
       * @param other MotionLaw to copy
       * @return Reference to this MotionLaw
       */
    MotionLaw &operator=(const MotionLaw &other);

    /**
       * @brief Move assignment operator
       * @param other MotionLaw to move
       * @return Reference to this MotionLaw
       */
    MotionLaw &operator=(MotionLaw &&other) noexcept;

    /**
       * @brief Virtual destructor
       */
    virtual ~MotionLaw() = default;

    /**
       * @brief Set the motion law type
       * @param type Motion law type
       */
    void setMotionType(MotionLawType type)
    {
        motionType_ = type;
    }

    /**
       * @brief Get the motion law type
       * @return Motion law type
       */
    MotionLawType getMotionType() const
    {
        return motionType_;
    }

    /**
       * @brief Set the motion law duration
       * @param duration Duration in seconds
       */
    void setDuration(double duration)
    {
        duration_ = duration;
    }

    /**
       * @brief Get the motion law duration
       * @return Duration in seconds
       */
    double getDuration() const
    {
        return duration_;
    }

    /**
       * @brief Set the maximum velocity
       * @param velocity Maximum velocity
       */
    void setMaxVelocity(double velocity)
    {
        maxVelocity_ = velocity;
    }

    /**
       * @brief Set the maximum acceleration
       * @param acceleration Maximum acceleration
       */
    void setMaxAcceleration(double acceleration)
    {
        maxAcceleration_ = acceleration;
    }

    /**
       * @brief Set the maximum jerk
       * @param jerk Maximum jerk
       */
    void setMaxJerk(double jerk)
    {
        maxJerk_ = jerk;
    }

    /**
       * @brief Set the boundary conditions for the motion law
       * @param initialVelocity Initial velocity
       * @param finalVelocity Final velocity
       * @param initialAcceleration Initial acceleration
       * @param finalAcceleration Final acceleration
       */
    void setBoundaryConditions(double initialVelocity = 0.0,
                               double finalVelocity = 0.0,
                               double initialAcceleration = 0.0,
                               double finalAcceleration = 0.0);

    /**
       * @brief Calculate the normalized parameter for a given time
       * @param time Time in seconds
       * @return Normalized parameter (0.0 - 1.0)
       */
    double evaluateParameter(double time) const;

    /**
       * @brief Calculate the first derivative of the parameter (velocity)
       * @param time Time in seconds
       * @return First derivative of the parameter
       */
    double evaluateFirstDerivative(double time) const;

    /**
       * @brief Calculate the second derivative of the parameter (acceleration)
       * @param time Time in seconds
       * @return Second derivative of the parameter
       */
    double evaluateSecondDerivative(double time) const;

    /**
       * @brief Calculate the third derivative of the parameter (jerk)
       * @param time Time in seconds
       * @return Third derivative of the parameter
       */
    double evaluateThirdDerivative(double time) const;

    /**
       * @brief Create a cubic polynomial motion law
       * @param duration Duration in seconds
       * @param initialVelocity Initial velocity
       * @param finalVelocity Final velocity
       * @return Motion law
       */
    static MotionLaw createCubicPolynomial(double duration,
                                           double initialVelocity = 0.0,
                                           double finalVelocity = 0.0);

    /**
       * @brief Create a quintic polynomial motion law
       * @param duration Duration in seconds
       * @param initialVelocity Initial velocity
       * @param finalVelocity Final velocity
       * @param initialAcceleration Initial acceleration
       * @param finalAcceleration Final acceleration
       * @return Motion law
       */
    static MotionLaw createQuinticPolynomial(double duration,
                                             double initialVelocity = 0.0,
                                             double finalVelocity = 0.0,
                                             double initialAcceleration = 0.0,
                                             double finalAcceleration = 0.0);

    /**
       * @brief Create a trapezoidal motion law
       * @param duration Duration in seconds
       * @param maxVelocity Maximum velocity
       * @param maxAcceleration Maximum acceleration
       * @return Motion law
       */
    static MotionLaw createTrapezoidal(double duration, double maxVelocity, double maxAcceleration);

private:
    MotionLawType motionType_; ///< Motion law type
    double duration_ = 1.0;    ///< Duration in seconds

    // Kinematic limits
    std::optional<double> maxVelocity_;     ///< Maximum velocity
    std::optional<double> maxAcceleration_; ///< Maximum acceleration
    std::optional<double> maxJerk_;         ///< Maximum jerk

    // Boundary conditions
    double initialVelocity_ = 0.0;     ///< Initial velocity
    double finalVelocity_ = 0.0;       ///< Final velocity
    double initialAcceleration_ = 0.0; ///< Initial acceleration
    double finalAcceleration_ = 0.0;   ///< Final acceleration

    // Parameters for trapezoidal motion law
    double accelTime_ = 0.0;    ///< Acceleration time
    double decelTime_ = 0.0;    ///< Deceleration time
    double constVelTime_ = 0.0; ///< Constant velocity time

    // Polynomial coefficients
    std::vector<double> coefficients_; ///< Polynomial coefficients

    // B-spline parameters
    std::vector<double> knotVector_;    ///< Knot vector for B-spline
    std::vector<double> controlPoints_; ///< Control points for B-spline
    int splineDegree_ = 3;              ///< Spline degree

    // Evaluation functions
    double evaluateCubicPolynomial(double time) const;
    double evaluateQuinticPolynomial(double time) const;
    double evaluateTrapezoidal(double time) const;
    double evaluateCubicSpline(double time) const;
    double evaluateBSpline(double time) const;

    // Derivative functions
    double evaluateCubicPolynomialDerivative(double time, int order) const;
    double evaluateQuinticPolynomialDerivative(double time, int order) const;
    double evaluateTrapezoidalDerivative(double time, int order) const;
    double evaluateCubicSplineDerivative(double time, int order) const;
    double evaluateBSplineDerivative(double time, int order) const;

    // Coefficient initialization
    void initCubicPolynomial();
    void initQuinticPolynomial();
    void initTrapezoidal();
};

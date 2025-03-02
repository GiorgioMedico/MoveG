/**
 * @file geometric_path.h
 * @brief Class for representing geometric paths in trajectories
 *
 * @author Giorgio Medico
 * @date 01/03/2025
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace MoveG
{

/**
   * @brief Enumeration of supported geometric path types
   */
enum class GeometricPathType
{
    LINEAR,         ///< Linear path between waypoints
    CIRCULAR,       ///< Circular path or arc
    SPLINE_CUBIC,   ///< Cubic spline interpolation
    SPLINE_QUINTIC, ///< Quintic spline interpolation
    SPLINE_BSPLINE  ///< B-spline interpolation
};

/**
   * @class GeometricPath
   * @brief Class representing a geometric path
   *
   * This class defines the geometry of the path between waypoints,
   * supporting linear, circular, and spline-based paths.
   */
class GeometricPath
{
public:
    /**
       * @brief Default constructor
       * @param type Geometric path type
       */
    explicit GeometricPath(GeometricPathType type = GeometricPathType::LINEAR);

    /**
       * @brief Copy constructor
       * @param other GeometricPath to copy
       */
    GeometricPath(const GeometricPath &other);

    /**
       * @brief Move constructor
       * @param other GeometricPath to move
       */
    GeometricPath(GeometricPath &&other) noexcept;

    /**
       * @brief Copy assignment operator
       * @param other GeometricPath to copy
       * @return Reference to this GeometricPath
       */
    GeometricPath &operator=(const GeometricPath &other);

    /**
       * @brief Move assignment operator
       * @param other GeometricPath to move
       * @return Reference to this GeometricPath
       */
    GeometricPath &operator=(GeometricPath &&other) noexcept;

    /**
       * @brief Virtual destructor
       */
    virtual ~GeometricPath() = default;

    /**
       * @brief Set the path type
       * @param type Geometric path type
       */
    void setPathType(GeometricPathType type)
    {
        pathType_ = type;
    }

    /**
       * @brief Get the path type
       * @return Geometric path type
       */
    GeometricPathType getPathType() const
    {
        return pathType_;
    }

    /**
       * @brief Set the circle center (for circular paths)
       * @param center Circle center
       */
    void setCircleCenter(const Eigen::Vector3d &center)
    {
        circleCenter_ = center;
    }

    /**
       * @brief Set the circle plane normal vector (for circular paths)
       * @param normal Vector normal to the circle plane
       */
    void setCircleNormal(const Eigen::Vector3d &normal)
    {
        circleNormal_ = normal;
    }

    /**
       * @brief Set the circle radius (for circular paths)
       * @param radius Circle radius
       */
    void setCircleRadius(double radius)
    {
        circleRadius_ = radius;
    }

    /**
       * @brief Set the arc angle (for circular paths)
       * @param angle Arc angle in radians
       */
    void setArcAngle(double angle)
    {
        arcAngle_ = angle;
    }

    /**
       * @brief Set the control points (for spline-based paths)
       * @param points Vector of control points
       */
    void setControlPoints(const std::vector<Eigen::Vector3d> &points)
    {
        controlPoints_ = points;
    }

    /**
       * @brief Set the knots (for B-spline)
       * @param knots Vector of knots
       */
    void setKnots(const std::vector<double> &knots)
    {
        knots_ = knots;
    }

    /**
       * @brief Set the spline degree
       * @param degree Spline degree
       */
    void setSplineDegree(int degree)
    {
        splineDegree_ = degree;
    }

    /**
       * @brief Get a position on the path
       * @param parameter Normalized parameter (0.0 - 1.0)
       * @return 3D position
       */
    Eigen::Vector3d evaluatePosition(double parameter) const;

    /**
       * @brief Get a derivative of the position on the path
       * @param parameter Normalized parameter (0.0 - 1.0)
       * @param order Derivative order (1 = velocity, 2 = acceleration)
       * @return Derivative of the position
       */
    Eigen::Vector3d evaluateDerivative(double parameter, int order = 1) const;

    /**
       * @brief Path length
       * @return Length in meters
       */
    double getLength() const;

    /**
       * @brief Create a linear path between two points
       * @param start Start point
       * @param end End point
       * @return Geometric path
       */
    static GeometricPath createLinePath(const Eigen::Vector3d &start, const Eigen::Vector3d &end);

    /**
       * @brief Create a circular path
       * @param center Circle center
       * @param normal Normal to the circle plane
       * @param radius Circle radius
       * @param startPoint Start point
       * @param arcAngle Arc angle in radians
       * @return Geometric path
       */
    static GeometricPath createCircularPath(const Eigen::Vector3d &center,
                                            const Eigen::Vector3d &normal,
                                            double radius,
                                            const Eigen::Vector3d &startPoint,
                                            double arcAngle);

    /**
       * @brief Create a cubic spline path
       * @param controlPoints Control points
       * @return Geometric path
       */
    static GeometricPath createCubicSplinePath(const std::vector<Eigen::Vector3d> &controlPoints);

    /**
       * @brief Create a B-spline path
       * @param controlPoints Control points
       * @param degree Spline degree
       * @param knots Knots (optional)
       * @return Geometric path
       */
    static GeometricPath createBSplinePath(const std::vector<Eigen::Vector3d> &controlPoints,
                                           int degree = 3,
                                           const std::vector<double> &knots = {});

private:
    GeometricPathType pathType_; ///< Geometric path type

    // Parameters for circular paths
    std::optional<Eigen::Vector3d> circleCenter_; ///< Circle center
    std::optional<Eigen::Vector3d> circleNormal_; ///< Normal to the circle plane
    std::optional<double> circleRadius_;          ///< Circle radius
    std::optional<double> arcAngle_;              ///< Arc angle

    // Parameters for spline
    std::vector<Eigen::Vector3d> controlPoints_; ///< Control points
    std::vector<double> knots_;                  ///< Knots for B-spline
    int splineDegree_ = 3;                       ///< Spline degree

    // Evaluation functions
    Eigen::Vector3d evaluateLinearPath(double parameter) const;
    Eigen::Vector3d evaluateCircularPath(double parameter) const;
    Eigen::Vector3d evaluateCubicSplinePath(double parameter) const;
    Eigen::Vector3d evaluateBSplinePath(double parameter) const;

    // Derivative functions
    Eigen::Vector3d evaluateLinearPathDerivative(double parameter, int order) const;
    Eigen::Vector3d evaluateCircularPathDerivative(double parameter, int order) const;
    Eigen::Vector3d evaluateCubicSplinePathDerivative(double parameter, int order) const;
    Eigen::Vector3d evaluateBSplinePathDerivative(double parameter, int order) const;
};

} // namespace MoveG

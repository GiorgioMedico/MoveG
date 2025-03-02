/**
 * @file geometric_path.h
 * @brief Class for representing geometric paths in trajectories
 *
 * @author Giorgio Medico
 * @date 02/03/2025
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
    LINEAR,     ///< Linear path between waypoints
    CIRCULAR,   ///< Circular path or arc
    POLYNOMIAL, ///< polynomial interpolation of order 3 or 5
    BEZIER,
    B_SPLINE ///< B-spline interpolation
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


private:
    GeometricPathType pathType_; ///< Geometric path type
};

} // namespace MoveG

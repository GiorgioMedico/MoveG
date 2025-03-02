/**
 * @file MoveG_Lib.h
 * @brief Main include file for the MoveG library
 *
 * This file includes all the necessary headers for using the MoveG library,
 * providing a single inclusion point for applications.
 *
 * @author Giorgio Medico
 * @date 28/02/2025
 */

#pragma once

// Version information
#include "config.hpp"

// Core rotation and pose components
#include "pose/pose_lib.h"
#include "pose/rotation_lib.h"

// Trajectory components
#include "trajectory/geometric_path.h"
#include "trajectory/motion_law.h"
#include "trajectory/trajectory.h"
#include "trajectory/waypoint.h"

/**
  * @namespace MoveG
  * @brief Namespace containing all classes and functions for the MoveG library
  *
  * The MoveG library provides tools for representing and manipulating 3D rotations,
  * poses, and trajectories. It is designed for robotics and computer vision applications,
  * providing a robust foundation for spatial transformations.
  */
namespace MoveG
{
/**
      * @brief Get the library version
      * @return String containing the library version
      */
inline std::string getVersion()
{
    return std::string(project_version);
}

/**
      * @brief Get the library name
      * @return String containing the library name
      */
inline std::string getName()
{
    return std::string(project_name);
}

/**
      * @brief Get the library version major number
      * @return Major version number
      */
inline int getVersionMajor()
{
    return project_version_major;
}

/**
      * @brief Get the library version minor number
      * @return Minor version number
      */
inline int getVersionMinor()
{
    return project_version_minor;
}

/**
      * @brief Get the library version patch number
      * @return Patch version number
      */
inline int getVersionPatch()
{
    return project_version_patch;
}
} // namespace MoveG

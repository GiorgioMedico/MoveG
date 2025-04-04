#include "config.hpp"
#include "fmt/core.h" // For fmt version information
#include <Eigen/Core> // For Eigen version information
#include <iostream>
#include <string>

int main(int /*argc*/, char ** /*argv*/) // comment to avoid warning
{
    // Print the version of the project
    std::cout << fmt::format("Project Name: {}\n", project_name);
    std::cout << fmt::format("Version: {}\n", project_version);

    // Print fmt library version
    std::cout << fmt::format("fmt version: {}.{}.{}\n",
                             FMT_VERSION / 10000,       // Major version
                             (FMT_VERSION / 100) % 100, // Minor version
                             FMT_VERSION % 100);        // Patch version

    // Print Eigen library version
    std::cout << fmt::format("Eigen version: {}.{}.{}\n",
                             EIGEN_WORLD_VERSION,  // Major version
                             EIGEN_MAJOR_VERSION,  // Minor version
                             EIGEN_MINOR_VERSION); // Patch version

    return 0;
}

#include "config.hpp"
#include "fmt/core.h"
#include <Eigen/Core>
#include <iostream>
#include <string>

int main(int /*argc*/, char ** /*argv*/) // comment to avoid warning
{
    fmt::print("Project Name: {}\n", project_name);
    fmt::print("Version: {}\n", project_version);
    fmt::print("fmt version: {}.{}.{}\n",
               FMT_VERSION / 10000,
               (FMT_VERSION / 100) % 100,
               FMT_VERSION % 100);
    fmt::print("Eigen version: {}.{}.{}\n",
               EIGEN_WORLD_VERSION,
               EIGEN_MAJOR_VERSION,
               EIGEN_MINOR_VERSION);

    return 0;
}


#include "fmt/format.h"
#include <iostream>
#include <string>
#include <vector>

#include "config.hpp"
#include "rotation_lib.h"


int main(int /*argc*/, char ** /*argv*/) // comment to avoid warning
{
    // Print the version of the project
    std::cout << "Project Name: " << project_name;
    std::cout << "\nVersion: " << project_version;

    return 0;
}

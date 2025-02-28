# MoveG

![C++](https://img.shields.io/badge/C%2B%2B-20-blue)
[![Ubuntu CI Test](https://github.com/GiorgioMedico/MoveG/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/GiorgioMedico/MoveG/actions/workflows/ubuntu.yml)
![Code Coverage](https://img.shields.io/badge/Code%20Coverage-95%25-success?style=flat)
[![Documentation](https://github.com/GiorgioMedico/MoveG/actions/workflows/documentation.yml/badge.svg)](https://github.com/GiorgioMedico/MoveG/actions/workflows/documentation.yml)
[![pre-commit](https://github.com/GiorgioMedico/MoveG/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/GiorgioMedico/MoveG/actions/workflows/pre-commit.yml)

A modern C++ library for 3D rotations, poses, and trajectories with comprehensive tooling. MoveG is designed for robotics and computer vision applications, providing a robust foundation for spatial transformations with strong type safety and error checking.

## Features

- Robust 3D transformation handling (quaternions, rotation matrices, Euler angles)
- Coordinate frame transformations with error checking
- Modern C++20 implementation
- Comprehensive unit tests with Catch2
- Automated code quality checks
- Memory safety with sanitizers
- High-performance optimizations (LTO, parallel builds)

## Key Components

### Rotation Library
Provides representation and operations for 3D rotations using quaternions, rotation matrices, Euler angles, and axis-angle representations. Supports conversion between formats, composition of rotations, and angular velocity transformations.

### Pose Library
Builds on the rotation library to represent 6-DOF poses (position and orientation). Supports transformation composition, inverse operations, and coordinate frame conversions with proper error handling.

### Trajectory Library (Coming Soon)
Will provide capabilities for motion planning and trajectory generation.

## Development Tools

- **[CPM](https://github.com/cpm-cmake/CPM.cmake)**: Package manager for dependency handling
- **[Catch2](https://github.com/catchorg/Catch2)**: Unit testing framework
- **[fmt](https://github.com/fmtlib/fmt)**: Modern string formatting library
- **[Eigen](https://eigen.tuxfamily.org)**: Linear algebra library
- **Clang Tools**: Format and static analysis
- **Sanitizers**: Address, undefined behavior, and leak detection
- **[Doxygen](https://doxygen.nl/)**: Documentation generation
- **[pre-commit](https://pre-commit.com/)**: Git hooks for code quality checks

## Build Commands

```bash
# Clone the repository with submodules
git clone --recursive https://github.com/GiorgioMedico/MoveG.git
cd MoveG

# Create build directory
mkdir build && cd build

# Configure and build (Release mode)
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release

# Run the main executable
cd app
./main

# Run all tests
cd ..
ctest -C Release

# Generate documentation
cmake --build . --target docs

# Run code coverage analysis
cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=On ..
cmake --build . --target coverage

# Code formatting
cmake --build . --target run_clang_format
cmake --build . --target run_cmake_format
```

## CMake Options

- `-DENABLE_TESTING=ON/OFF`: Enable/disable unit tests (default: ON)
- `-DENABLE_COVERAGE=ON/OFF`: Enable code coverage analysis (default: ON)
- `-DENABLE_CLANG_TIDY=ON/OFF`: Enable static analysis (default: ON)
- `-DENABLE_SANITIZE_ADDR=ON/OFF`: Enable address sanitizer (default: ON)
- `-DENABLE_SANITIZE_UNDEF=ON/OFF`: Enable undefined behavior sanitizer (default: ON)
- `-DENABLE_SANITIZE_LEAK=ON/OFF`: Enable memory leak sanitizer (default: ON)
- `-DENABLE_LTO=ON/OFF`: Enable link-time optimization (default: OFF, auto-ON for Release)
- `-DENABLE_WARNINGS_AS_ERRORS=ON/OFF`: Treat warnings as errors (default: OFF)
- `-DUSE_CPM=ON/OFF`: Use CPM for dependency management (default: ON)

## IDE Integration

### Visual Studio Code
This project includes optimized configuration for VS Code:
- Press `Alt+S` to delete CMake cache and reconfigure
- Use the "CMake: Build" task for building
- Use the "CMake: Run Tests" button for running tests
- Use built-in tasks for documentation and code coverage

## Project Structure

- `src/MoveG_Lib/` - Core library implementation
  - `rotation_lib.{h,cpp}` - 3D rotation management
  - `pose_lib.{h,cpp}` - Pose (position + orientation) handling
  - `traj_lib.{h,cpp}` - Trajectory planning (in development)
- `app/` - Example applications
  - `pose_example.cpp` - Demonstration of pose library features
  - `rotation_example.cpp` - Demonstration of rotation library features
- `tests/` - Unit tests with Catch2
  - `test_Pose.cpp` - Comprehensive tests for pose functionality
  - `test_Rotation.cpp` - Comprehensive tests for rotation functionality
- `cmake/` - Build system modules
  - `CodeCoverage.cmake` - Code coverage configuration
  - `Sanitizer.cmake` - Memory and behavior sanitizers
  - `LTO.cmake` - Link-time optimization
- `docs/` - Documentation
- `tools/` - Development helper scripts

## Dependencies

MoveG uses Eigen3 as its primary dependency for linear algebra and geometry operations. The required version is automatically installed via CPM during the build process.

## Software Requirements

- CMake 3.21+
- C++20 compatible compiler (GCC 9+, Clang 9+, MSVC 2019+)
- Git (for cloning and development)
- Doxygen (for documentation generation)
- Python 3 (for helper scripts)

## License

Apache License 2.0

## Contributing

Contributions are welcome. Please format your code with clang-format and ensure all tests pass before submitting pull requests.

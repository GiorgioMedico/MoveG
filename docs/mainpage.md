/**
 * @mainpage MoveG Library Documentation
 *
 * # MoveG Library
 *
 * MoveG is a modern C++ library for 3D rotations, poses, and trajectories designed specifically for robotics and computer vision applications. The library provides a comprehensive foundation for spatial transformations with robust error checking and type safety mechanisms.
 *
 * ## Core Features
 *
 * MoveG offers several key capabilities essential for robotics programming:
 *
 * - Robust representation and manipulation of 3D rotations using quaternions, rotation matrices, and Euler angles
 * - Comprehensive pose management (position and orientation) with coordinate frame transformations
 * - High-performance implementation leveraging C++20 features
 * - Extensive error checking and validation to prevent common geometric errors
 * - Thoroughly tested with comprehensive unit tests to ensure reliability
 *
 * ## Library Components
 *
 * The library is organized into several primary components:
 *
 * ### Rotation Library
 *
 * The rotation library provides facilities for representing and manipulating 3D rotations using various mathematical formalisms. It includes:
 *
 * - Conversion between quaternions, rotation matrices, Euler angles, and axis-angle representations
 * - Robust composition of rotations with proper normalization
 * - Angular velocity transformations and differential rotation operations
 *
 * To explore the rotation library API, see the @ref MoveG::Rotation class documentation.
 *
 * ### Pose Library
 *
 * The pose library builds on the rotation library to represent complete 6-DOF transformations:
 *
 * - Combined position and orientation representation
 * - Pose composition and inversion operations
 * - Local-to-global and global-to-local coordinate transformations
 * - Distance metrics for comparing poses
 *
 * To explore the pose library API, see the @ref MoveG::Pose class documentation.
 *
 * ## Getting Started
 *
 * ### Prerequisites
 *
 * MoveG requires:
 * - A C++20 compatible compiler (GCC 9+, Clang 9+, MSVC 2019+)
 * - CMake 3.21 or higher
 * - Eigen3 (automatically installed via CPM during the build process)
 *
 * ### Integration
 *
 * MoveG can be integrated into your project using CMake:
 *
 * ```cmake
 * # Add MoveG as a subdirectory
 * add_subdirectory(MoveG)
 *
 * # Link against the library
 * target_link_libraries(your_target PRIVATE MoveG_Lib)
 * ```
 *
 * ## Examples
 *
 * The library includes practical examples demonstrating the core functionality:
 *
 * - Basic rotation operations: @ref rotation_example.cpp
 * - Working with poses: @ref pose_example.cpp
 * - Complete application integration: @ref main.cpp
 *
 * ## Development and Contribution
 *
 * MoveG follows modern C++ development practices with comprehensive tooling:
 *
 * - Automated testing with Catch2
 * - Code quality enforcement with clang-format and clang-tidy
 * - Memory safety verification using sanitizers
 * - Performance optimization with Link Time Optimization (LTO)
 *
 * For detailed information about contributing to the project, please refer to the guidelines in the README.
 */

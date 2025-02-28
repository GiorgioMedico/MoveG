cmake_minimum_required(VERSION 3.22)

# Enable parallel builds
include(ProcessorCount)
ProcessorCount(CPU_CORES)
if(CPU_CORES EQUAL 0)
  set(CPU_CORES 8)
endif()
set(CMAKE_BUILD_PARALLEL_LEVEL ${CPU_CORES})

# Enable ccache for faster rebuilds
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
endif()

project(
    "MoveG"
    VERSION 1.0.0
    LANGUAGES C CXX)

# Global CMake variables
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add optimization flags based on build type
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
    # Enable LTO by default for Release builds
    set(ENABLE_LTO ON CACHE BOOL "Enable LTO in Release mode" FORCE)
endif()

# Options
option(USE_CPM "Whether to use CPM." ON)

option(ENABLE_WARNINGS "Enable to add warnings to a target." ON)
option(ENABLE_WARNINGS_AS_ERRORS "Enable to treat warnings as errors." OFF)

option(ENABLE_TESTING "Enable a Unit Testing build." ON)
option(ENABLE_COVERAGE "Enable a Code Coverage build." ON)

# Enforce dependency: ENABLE_COVERAGE requires ENABLE_TESTING
if(ENABLE_COVERAGE AND NOT ENABLE_TESTING)
    message(WARNING "ENABLE_COVERAGE=ON requires ENABLE_TESTING=ON, enabling testing")
    set(ENABLE_TESTING ON CACHE BOOL "Enable a Unit Testing build." FORCE)
endif()

option(ENABLE_CLANG_TIDY "Enable to add clang tidy." ON)

option(ENABLE_SANITIZE_ADDR "Enable address sanitize." ON)
option(ENABLE_SANITIZE_UNDEF "Enable undefined sanitize." ON)
option(ENABLE_SANITIZE_LEAK "Enable leak sanitize (Gcc/Clang only)." ON)
option(ENABLE_SANITIZE_THREAD "Enable thread sanitize (Gcc/Clang only)." OFF)

option(ENABLE_CLANG_FORMAT "Enable to add clang-format." ON)
option(ENABLE_CMAKE_FORMAT "Enable to add cmake-format." ON)

option(ENABLE_LTO "Enable to add Link Time Optimization." OFF)

# Project/Library Names
set(EXECUTABLE_NAME "main")
set(LIBRARY_NAME "MoveG_Lib")

# CREATE INTERFACE LIBRARIES FOR PROJECT OPTIONS
# Base options without warnings - for all targets
add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_20)

# Library-specific options with warnings - only for our library code
add_library(project_options_with_warnings INTERFACE)
target_link_libraries(project_options_with_warnings INTERFACE project_options)

# CMAKE MODULES
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(ConfigSafeGuards)
include(Docs)
include(Tools)

if(ENABLE_WARNINGS)
    include(Warnings)
    # Only add warnings to project_options_with_warnings
    if(WARNING_FLAGS)
        message(STATUS "Warning flags: ${WARNING_FLAGS}")
        target_compile_options(project_options_with_warnings INTERFACE ${WARNING_FLAGS})

        if(ENABLE_WARNINGS_AS_ERRORS AND WARNINGS_AS_ERRORS_FLAGS)
            message(STATUS "Warnings as errors flags: ${WARNINGS_AS_ERRORS_FLAGS}")
            target_compile_options(project_options_with_warnings INTERFACE ${WARNINGS_AS_ERRORS_FLAGS})
        endif()
    else()
        message(WARNING "No warning flags were set for the current compiler")
    endif()
endif()


add_cmake_format_target()
add_clang_format_target()

if(ENABLE_SANITIZE_ADDR
   OR ENABLE_SANITIZE_UNDEF
   OR ENABLE_SANITIZE_LEAK
   OR ENABLE_SANITIZE_THREAD)
    include(Sanitizer)
    add_sanitizer_flags()
    target_compile_options(project_options INTERFACE ${SANITIZER_FLAGS})
    target_link_options(project_options INTERFACE ${SANITIZER_FLAGS})
endif()

if(ENABLE_TESTING AND ENABLE_COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()

if(ENABLE_LTO)
    include(LTO)
    # Apply LTO settings to project_options
    target_compile_options(project_options INTERFACE ${LTO_FLAGS})
    target_link_options(project_options INTERFACE ${LTO_LINK_FLAGS})
endif()

# Use CPM for package management
if(USE_CPM)
    message(STATUS "Using CPM")
    # Set the CPM source cache directory
    set(CPM_SOURCE_CACHE
        "${CMAKE_SOURCE_DIR}/.cpmcache"
        CACHE STRING "CPM source cache directory")

    include(CPM)
    cpmaddpackage("gh:fmtlib/fmt#9.1.0")
    cpmaddpackage(
        NAME
        Eigen
        GIT_REPOSITORY
        https://gitlab.com/libeigen/eigen.git
        GIT_TAG
        3.4.0
        EXCLUDE_FROM_ALL
        YES
        SYSTEM
        YES
        OPTIONS
        "EIGEN_BUILD_DOC OFF" # Disabilita la generazione della documentazione
        "EIGEN_BUILD_TESTS OFF" # Disabilita i test di Eigen
        "EIGEN_BUILD_PKGCONFIG OFF" # Disabilita la generazione del file pkg-config
        "BUILD_TESTING OFF" # Disabilita la generazione dei test
    )
    if(ENABLE_TESTING)
        cpmaddpackage("gh:catchorg/Catch2#v3.7.1")
        if(TARGET Catch2)
            list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
        endif()
    endif()
endif()

# Add precompiled headers to speed up builds
add_library(project_precompiled_headers INTERFACE)
target_precompile_headers(project_precompiled_headers INTERFACE
    <vector>
    <string>
    <memory>
    <fmt/format.h>
    <Eigen/Dense>
)

# SUBDIRECTORIES
add_subdirectory(configured)
add_subdirectory(src)
add_subdirectory(app)
if(ENABLE_TESTING)
    include(CTest) # include CTest module from default CMake modules
    enable_testing()
    add_subdirectory(tests)
endif()

# INSTALL TARGETS

install(
    TARGETS ${EXECUTABLE_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

install(
    TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# Display build configuration summary
message(STATUS "========== Build Configuration Summary ==========")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Testing enabled: ${ENABLE_TESTING}")
message(STATUS "Coverage enabled: ${ENABLE_COVERAGE}")
message(STATUS "LTO enabled: ${ENABLE_LTO}")
message(STATUS "Warnings enabled: ${ENABLE_WARNINGS} (only for library code)")
message(STATUS "Sanitizers: ADDR=${ENABLE_SANITIZE_ADDR}, UNDEF=${ENABLE_SANITIZE_UNDEF}, LEAK=${ENABLE_SANITIZE_LEAK}")
message(STATUS "================================================")

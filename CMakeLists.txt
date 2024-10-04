cmake_minimum_required(VERSION 3.22)

project(
    "MoveG"
    VERSION 1.0.0
    LANGUAGES C CXX)

# Global CMake variables are set here
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Set the build type
# set(CMAKE_BUILD_TYPE "Debug")

# Options
option(USE_CPM "Whether to use CPM." ON)

option(ENABLE_WARNINGS "Enable to add warnings to a target." ON)
option(ENABLE_WARNINGS_AS_ERRORS "Enable to treat warnings as errors." OFF)

option(ENABLE_TESTING "Enable a Unit Testing build." ON)
option(ENABLE_COVERAGE "Enable a Code Coverage build." ON)

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

# CMAKE MODULES
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(ConfigSafeGuards)
include(Docs)
include(Tools)

if(ENABLE_WARNINGS)
    include(Warnings)
endif()

add_cmake_format_target()
add_clang_format_target()

if(ENABLE_SANITIZE_ADDR
   OR ENABLE_SANITIZE_UNDEF
   OR ENABLE_SANITIZE_LEAK
   OR ENABLE_SANITIZE_THREAD)
    include(Sanitizer)
    add_sanitizer_flags()
endif()

if(ENABLE_TESTING AND ENABLE_COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()

if(ENABLE_LTO)
    include(LTO)
endif()

# EXTERNAL LIBRARIES

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
        "BUILD_TESTING OFF")
    if(ENABLE_TESTING)
        cpmaddpackage("gh:catchorg/Catch2#v3.5.3")
        list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras
        )# append path to CMake modules path
    endif()
endif()

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
    LIBRARY DESTINATION lib)

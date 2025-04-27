include(Catch)

# Unit Test for Rotation
add_executable("UnitTestRotation" "test_Rotation.cpp")
target_link_libraries(
    "UnitTestRotation"
    PRIVATE ${LIBRARY_NAME}
            fmt
            Eigen
            Catch2::Catch2WithMain
            ${WARNING_FLAGS}
            ${WARNINGS_AS_ERRORS_FLAGS})

# Unit Test for Pose
add_executable("UnitTestPose" "test_Pose.cpp")
target_link_libraries(
    "UnitTestPose"
    PRIVATE ${LIBRARY_NAME}
            fmt
            Eigen
            Catch2::Catch2WithMain
            ${WARNING_FLAGS}
            ${WARNINGS_AS_ERRORS_FLAGS})

# Unit Test for Tridiagonal Solver
add_executable("UnitTestTridiagonalSolver" "test_TridiagonalSolver.cpp")
target_link_libraries(
    "UnitTestTridiagonalSolver"
    PRIVATE ${LIBRARY_NAME}
            fmt
            Eigen
            Catch2::Catch2WithMain
            ${WARNING_FLAGS}
            ${WARNINGS_AS_ERRORS_FLAGS})

# Unit Test for fmt formatter
add_executable("UnitTestFormatter" "test_fmt_Formatter.cpp")
target_link_libraries(
    "UnitTestFormatter"
    PRIVATE ${LIBRARY_NAME}
            fmt
            Eigen
            Catch2::Catch2WithMain
            ${WARNING_FLAGS}
            ${WARNINGS_AS_ERRORS_FLAGS})

# Gives to CTest the ability to see the tests that are in the executable
catch_discover_tests("UnitTestRotation")
catch_discover_tests("UnitTestPose")
catch_discover_tests("UnitTestTridiagonalSolver")
catch_discover_tests("UnitTestFormatter")

if(ENABLE_COVERAGE)
    set(COVERAGE_MAIN "coverage")
    set(COVERAGE_EXCLUDES
        "${PROJECT_SOURCE_DIR}/app/*"
        "${PROJECT_SOURCE_DIR}/cmake/*"
        "${PROJECT_SOURCE_DIR}/docs/*"
        "${PROJECT_SOURCE_DIR}/.cpmcache/*"
        "${PROJECT_SOURCE_DIR}/tests/*"
        "${PROJECT_SOURCE_DIR}/build/*"
        "/usr/include/*")
    set(COVERAGE_EXTRA_FLAGS)
    set(COVERAGE_DEPENDENCIES
        "UnitTestRotation"
        "UnitTestPose"
        "UnitTestTridiagonalSolver"
        "UnitTestFormatter")

    setup_target_for_coverage_gcovr_html(
        NAME
        ${COVERAGE_MAIN}
        EXECUTABLE
        ctest
        DEPENDENCIES
        ${COVERAGE_DEPENDENCIES}
        BASE_DIRECTORY
        ${CMAKE_SOURCE_DIR})
endif()

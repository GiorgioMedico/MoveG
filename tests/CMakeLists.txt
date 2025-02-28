include(Catch)

# Unit Test for Rotation
add_executable("UnitTestRotation" "test_Rotation.cpp")
target_link_libraries("UnitTestRotation"
    PRIVATE
        ${LIBRARY_NAME}
        Catch2::Catch2WithMain
        project_options  # Base options without warnings, or use project_options_with_warnings
)

# Unit Test for Pose
add_executable("UnitTestPose" "test_Pose.cpp")
target_link_libraries("UnitTestPose"
    PRIVATE
        ${LIBRARY_NAME}
        Catch2::Catch2WithMain
        project_options  # Base options without warnings, or use project_options_with_warnings
)

# Gives to CTest the ability to see the tests that are in the executable
catch_discover_tests("UnitTestRotation")
catch_discover_tests("UnitTestPose")

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
    set(COVERAGE_DEPENDENCIES "UnitTestRotation" "UnitTestPose")

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

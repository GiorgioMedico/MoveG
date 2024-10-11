include(Catch)

add_executable("UnitTestRotation" "test_Rotation.cpp")
target_link_libraries("UnitTestRotation" PUBLIC ${LIBRARY_NAME})
target_link_libraries("UnitTestRotation" PRIVATE Catch2::Catch2WithMain)

add_executable("UnitTestPose" "test_Pose.cpp")
target_link_libraries("UnitTestPose" PUBLIC ${LIBRARY_NAME})
target_link_libraries("UnitTestPose" PRIVATE Catch2::Catch2WithMain)

add_executable("Example" "pose_example.cpp")
target_link_libraries("Example" PUBLIC ${LIBRARY_NAME})
target_link_libraries("Example" PRIVATE Catch2::Catch2WithMain)

if(${ENABLE_WARNINGS})

    target_set_warnings(
        TARGET
        "Example"
        ENABLE
        ${ENABLE_WARNINGS}
        AS_ERRORS
        ${ENABLE_WARNINGS_AS_ERRORS})

    target_set_warnings(
        TARGET
        "UnitTestRotation"
        ENABLE
        ${ENABLE_WARNINGS}
        AS_ERRORS
        ${ENABLE_WARNINGS_AS_ERRORS})

    target_set_warnings(
        TARGET
        "UnitTestPose"
        ENABLE
        ${ENABLE_WARNINGS}
        AS_ERRORS
        ${ENABLE_WARNINGS_AS_ERRORS})
endif()

# Gives to CTest the ability to see the tests that are in the executable
# catch_discover_tests("UnitTestFoo")

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

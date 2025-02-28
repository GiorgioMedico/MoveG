set(APP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

add_executable(${EXECUTABLE_NAME} ${APP_SOURCES})

add_executable(rotation "${CMAKE_CURRENT_SOURCE_DIR}/rotation_example.cpp")
add_executable(pose "${CMAKE_CURRENT_SOURCE_DIR}/pose_example.cpp")


target_link_libraries(
    ${EXECUTABLE_NAME}
    PRIVATE
        ${LIBRARY_NAME}
        fmt::fmt
        Eigen3::Eigen
        project_options_with_warnings  # Include warnings
        project_precompiled_headers
)

target_link_libraries(
    rotation
    PRIVATE
        ${LIBRARY_NAME}
        fmt::fmt
        Eigen3::Eigen
        project_options_with_warnings  # Include warnings
        project_precompiled_headers
)
target_link_libraries(
    pose
    PRIVATE
        ${LIBRARY_NAME}
        fmt::fmt
        Eigen3::Eigen
        project_options_with_warnings  # Include warnings
        project_precompiled_headers
)

if(${ENABLE_LTO})
    target_enable_lto(
        TARGET
        ${EXECUTABLE_NAME}
        ENABLE
        ON)
endif()

if(${ENABLE_CLANG_TIDY})
    add_clang_tidy_to_target(${EXECUTABLE_NAME})
endif()

set(APP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

add_executable(${EXECUTABLE_NAME} ${APP_SOURCES})

add_executable(rotation "${CMAKE_CURRENT_SOURCE_DIR}/rotation_example.cpp")
add_executable(pose "${CMAKE_CURRENT_SOURCE_DIR}/pose_example.cpp")

target_link_libraries(
    ${EXECUTABLE_NAME}
    PRIVATE ${LIBRARY_NAME}
            fmt
            Eigen
            ${WARNING_FLAGS}
            ${WARNINGS_AS_ERRORS_FLAGS})

target_link_libraries(
    rotation
    PRIVATE ${LIBRARY_NAME}
            fmt
            Eigen
            ${WARNING_FLAGS}
            ${WARNINGS_AS_ERRORS_FLAGS})
target_link_libraries(
    pose
    PRIVATE ${LIBRARY_NAME}
            fmt
            Eigen
            ${WARNING_FLAGS}
            ${WARNINGS_AS_ERRORS_FLAGS})

if(${ENABLE_LTO})
    target_enable_lto(
        TARGET
        ${EXECUTABLE_NAME}
        rotation
        pose
        ENABLE
        ON)
endif()

if(${ENABLE_CLANG_TIDY})
    add_clang_tidy_to_target(${EXECUTABLE_NAME})
    add_clang_tidy_to_target(rotation)
    add_clang_tidy_to_target(pose)
endif()

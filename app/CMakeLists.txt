set(APP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

add_executable(${EXECUTABLE_NAME} ${APP_SOURCES})


target_link_libraries(
    ${EXECUTABLE_NAME}
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

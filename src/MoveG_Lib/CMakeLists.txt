set(LIBRARY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/rotation_lib.cpp"
                    "${CMAKE_CURRENT_SOURCE_DIR}/pose_lib.cpp")

set(LIBRARY_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/rotation_lib.h"
                    "${CMAKE_CURRENT_SOURCE_DIR}/pose_lib.h")

set(LIBRARY_INCLUDES "./" "${CMAKE_BINARY_DIR}/configured_files/include")

add_library(${LIBRARY_NAME} STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_include_directories(${LIBRARY_NAME} PUBLIC ${LIBRARY_INCLUDES})

# Link to project_options_with_warnings instead of using target_set_warnings
# This applies all the warnings to your library code only
target_link_libraries(${LIBRARY_NAME}
    PRIVATE
        fmt::fmt
        Eigen3::Eigen
        project_options_with_warnings  # Add warnings only to your library
        project_precompiled_headers    # Also link precompiled headers
)

if(${ENABLE_LTO})
    target_enable_lto(
        TARGET
        ${LIBRARY_NAME}
        ENABLE
        ON)
endif()

if(${ENABLE_CLANG_TIDY})
    add_clang_tidy_to_target(${LIBRARY_NAME})
endif()

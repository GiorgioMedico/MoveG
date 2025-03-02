# Define all source files by component
set(ROTATION_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/rotation/rotation_lib.cpp"
)

set(POSE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/pose/pose_lib.cpp"
)

set(TRAJECTORY_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/trajectory.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/waypoint.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/geometric_path.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/motion_law.cpp"
)

# Combine all sources
set(LIBRARY_SOURCES
    ${ROTATION_SOURCES}
    ${POSE_SOURCES}
    ${TRAJECTORY_SOURCES}
)

# Define header files by component
set(ROTATION_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/rotation/rotation_lib.h"
)

set(POSE_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/pose/pose_lib.h"
)

set(TRAJECTORY_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/trajectory.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/waypoint.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/geometric_path.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/trajectory/motion_law.h"
)

# Combine all headers
set(LIBRARY_HEADERS
    ${ROTATION_HEADERS}
    ${POSE_HEADERS}
    ${TRAJECTORY_HEADERS}
)

set(LIBRARY_INCLUDES "./" "${CMAKE_BINARY_DIR}/configured_files/include")

add_library(${LIBRARY_NAME} STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_include_directories(${LIBRARY_NAME} PUBLIC ${LIBRARY_INCLUDES})

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

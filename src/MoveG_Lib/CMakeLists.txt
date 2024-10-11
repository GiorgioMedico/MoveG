set(LIBRARY_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/rotation_lib.cpp"
                    "${CMAKE_CURRENT_SOURCE_DIR}/pose_lib.cpp")

set(LIBRARY_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/rotation_lib.h"
                    "${CMAKE_CURRENT_SOURCE_DIR}/pose_lib.h")
                    
set(LIBRARY_INCLUDES "./" "${CMAKE_BINARY_DIR}/configured_files/include")

add_library(${LIBRARY_NAME} STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_include_directories(${LIBRARY_NAME} PUBLIC ${LIBRARY_INCLUDES})
target_link_libraries(${LIBRARY_NAME} PRIVATE fmt::fmt Eigen3::Eigen)

if(${ENABLE_WARNINGS})
    target_set_warnings(
        TARGET
        ${LIBRARY_NAME}
        ENABLE
        ${ENABLE_WARNINGS}
        AS_ERRORS
        ${ENABLE_WARNINGS_AS_ERRORS})
endif()

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

find_package(Doxygen)

if(DOXYGEN_FOUND)
    # Define source files properly using file(GLOB...)
    file(GLOB SOURCE_FILES
        "${CMAKE_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/*.h"
        "${CMAKE_SOURCE_DIR}/src/*.hpp"
    )

    file(GLOB EXAMPLE_FILES
        "${CMAKE_SOURCE_DIR}/app/*_example.cpp"
        "${CMAKE_SOURCE_DIR}/app/main.cpp"
    )

    # Set up documentation target without file dependencies in the command
    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/docs/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/docs
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )

    # Add message to indicate successful configuration
    message(STATUS "Doxygen found: Documentation target 'docs' configured")
else()
    message(STATUS "Doxygen not found: Documentation generation disabled")
endif()

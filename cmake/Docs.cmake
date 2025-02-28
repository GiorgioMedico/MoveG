find_package(Doxygen)

if(DOXYGEN_FOUND)
    # Copy example files to docs directory for reference
    file(GLOB EXAMPLE_FILES "${CMAKE_SOURCE_DIR}/app/*_example.cpp")
    file(COPY ${EXAMPLE_FILES} DESTINATION "${CMAKE_SOURCE_DIR}/docs/examples")

    # Configure Doxyfile with correct paths
    configure_file(${CMAKE_SOURCE_DIR}/docs/Doxyfile ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)

    # Add custom command to generate documentation
    add_custom_command(
        OUTPUT ${CMAKE_BINARY_DIR}/html/index.html
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS ${CMAKE_SOURCE_DIR}/src/*.cpp ${CMAKE_SOURCE_DIR}/src/*.h ${EXAMPLE_FILES}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )

    # Add documentation target
    add_custom_target(docs DEPENDS ${CMAKE_BINARY_DIR}/html/index.html)
endif()

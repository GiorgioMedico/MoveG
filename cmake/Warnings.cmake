# Warnings.cmake
# Definisce warning flags da usare per i target CMake

# Imposta le warning flags per i vari compilatori
function(set_warning_flags)
    set(MSVC_WARNINGS
        # Baseline
        /W4 # Baseline reasonable warnings
        /permissive- # standards conformance mode for MSVC compiler
        # C and C++ Warnings
        /w14242 # conversion from 'type1' to 'type1', possible loss of data
        /w14254 # 'operator': conversion from 't1:field_bits' to 't2:field_bits'
        /w14287 # unsigned/negative constant mismatch
        /w14296 # expression is always 'boolean_value'
        /w14311 # pointer truncation from 'type1' to 'type2'
        /w44062 # enumerator in a switch of enum 'enumeration' is not handled
        /w44242 # conversion from 'type1' to 'type2', possible loss of data
        /w14826 # Conversion from 'type1' to 'type_2' is sign-extended
        /w14905 # wide string literal cast to 'LPSTR'
        /w14906 # string literal cast to 'LPWSTR'
        # C++ Only
        /w14263 # function does not override any base class virtual function
        /w14265 # class has virtual functions, but destructor is not virtual
        /w14640 # Enable warning on thread un-safe static member initialization
        /w14928 # more than one implicitly user-defined conversion
        /we4289 # nonstandard extension used: 'variable'
    )

    set(CLANG_WARNINGS
        # Baseline
        -Wall
        -Wextra # reasonable and standard
        -Wshadow # if a variable declaration shadows one from a parent context
        -Wpedantic # warn if non-standard is used
        # C and C++ Warnings
        -Wunused # warn on anything being unused
        -Wformat=2 # warn on security issues around functions that format output
        -Wcast-align # warn for potential performance problem casts
        -Wconversion # warn on type conversions that may lose data
        -Wsign-conversion # warn on sign conversions
        -Wnull-dereference # warn if a null dereference is detected
        -Wdouble-promotion # warn if float is implicit promoted to double
        # C++ Warnings
        -Wnon-virtual-dtor # if a class with virtual func has a non-virtual dest
        -Wold-style-cast # warn for c-style casts
        -Woverloaded-virtual # if you overload (not override) a virtual function
        -Weffc++ # violations from Scott Meyers' Effective C++
    )

    set(GCC_WARNINGS
        ${CLANG_WARNINGS}
        -Wduplicated-cond # warn if if / else chain has duplicated conditions
        -Wduplicated-branches # warn if if / else branches have duplicated code
        -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
    )

    # Esporta le variabili come variabili globali
    set(MSVC_WARNINGS ${MSVC_WARNINGS} PARENT_SCOPE)
    set(CLANG_WARNINGS ${CLANG_WARNINGS} PARENT_SCOPE)
    set(GCC_WARNINGS ${GCC_WARNINGS} PARENT_SCOPE)
endfunction()

# Imposta le warning flags appropriate per il compilatore corrente
function(configure_warning_flags)
    # Configura warning flags in base al compilatore
    if(MSVC)
        set(WARNING_FLAGS ${MSVC_WARNINGS} PARENT_SCOPE)
        set(WARNINGS_AS_ERRORS_FLAGS /WX PARENT_SCOPE)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        set(WARNING_FLAGS ${CLANG_WARNINGS} PARENT_SCOPE)
        set(WARNINGS_AS_ERRORS_FLAGS -Werror PARENT_SCOPE)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(WARNING_FLAGS ${GCC_WARNINGS} PARENT_SCOPE)
        set(WARNINGS_AS_ERRORS_FLAGS -Werror PARENT_SCOPE)
    else()
        message(WARNING "Unknown compiler '${CMAKE_CXX_COMPILER_ID}': no warning flags set")
        set(WARNING_FLAGS "" PARENT_SCOPE)
        set(WARNINGS_AS_ERRORS_FLAGS "" PARENT_SCOPE)
    endif()
endfunction()

# Funzione per applicare warnings a un target
function(target_set_warnings)
    set(oneValueArgs TARGET ENABLE AS_ERRORS)
    cmake_parse_arguments(
        TARGET_SET_WARNINGS
        ""
        "${oneValueArgs}"
        ""
        ${ARGN})

    # Prima chiamiamo le funzioni per impostare le warning flags
    set_warning_flags()

    if(TARGET_SET_WARNINGS_ENABLE)
        # Seleziona i warnings in base al compilatore
        if(MSVC)
            target_compile_options(${TARGET_SET_WARNINGS_TARGET} PRIVATE ${MSVC_WARNINGS})
            if(TARGET_SET_WARNINGS_AS_ERRORS)
                target_compile_options(${TARGET_SET_WARNINGS_TARGET} PRIVATE /WX)
            endif()
        elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            target_compile_options(${TARGET_SET_WARNINGS_TARGET} PRIVATE ${CLANG_WARNINGS})
            if(TARGET_SET_WARNINGS_AS_ERRORS)
                target_compile_options(${TARGET_SET_WARNINGS_TARGET} PRIVATE -Werror)
            endif()
        elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            target_compile_options(${TARGET_SET_WARNINGS_TARGET} PRIVATE ${GCC_WARNINGS})
            if(TARGET_SET_WARNINGS_AS_ERRORS)
                target_compile_options(${TARGET_SET_WARNINGS_TARGET} PRIVATE -Werror)
            endif()
        else()
            message(WARNING "Unknown compiler '${CMAKE_CXX_COMPILER_ID}': no warning flags set for ${TARGET_SET_WARNINGS_TARGET}")
        endif()
    endif()
endfunction()

# Configurazione iniziale per impostare le variabili globali WARNING_FLAGS e WARNINGS_AS_ERRORS_FLAGS
set_warning_flags()
configure_warning_flags()

# # Stampa le warning flags configurate per debug
# message(STATUS "Warning flags: ${WARNING_FLAGS}")
# message(STATUS "Warnings as errors flags: ${WARNINGS_AS_ERRORS_FLAGS}")

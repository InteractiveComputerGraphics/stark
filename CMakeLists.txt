cmake_minimum_required(VERSION 3.18...3.31)

project(stark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Required for shared libraries / Python extension modules.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Default to Release build if not specified.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "MinSizeRel" "RelWithDebInfo"
    )
endif()

# Suppress warnings about ignored attributes when using SIMD types as template arguments.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wno-ignored-attributes)
endif()

option(STARK_BUILD_TESTS "Build STARK tests" ON)
option(STARK_BUILD_PYTHON_BINDINGS "Build STARK Python bindings" ON)

# STARK-specific Python override.
set(STARK_PYTHON_EXECUTABLE "NOT_SET" CACHE FILEPATH "Python executable used to build pystark")

# Build core library and examples.
add_subdirectory(stark)
add_subdirectory(examples)

if(STARK_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(STARK_BUILD_PYTHON_BINDINGS)
    # Prefer active virtualenv / Conda environment over system Python.
    set(Python_FIND_VIRTUALENV FIRST)
    set(Python_FIND_STRATEGY LOCATION)
    set(Python_FIND_IMPLEMENTATIONS CPython)

    # If the user provided the STARK-specific variable, forward it to CMake's FindPython.
    if(STARK_PYTHON_EXECUTABLE)
        if(NOT EXISTS "${STARK_PYTHON_EXECUTABLE}")
            message(FATAL_ERROR
                "STARK_PYTHON_EXECUTABLE does not exist:\n"
                "  ${STARK_PYTHON_EXECUTABLE}\n"
            )
        endif()

        set(Python_EXECUTABLE "${STARK_PYTHON_EXECUTABLE}"
            CACHE FILEPATH "Python executable used by FindPython" FORCE
        )
    endif()

    find_package(Python 3.8 QUIET COMPONENTS Interpreter Development.Module)

    if(NOT Python_FOUND)
        if(Python_VERSION)
            set(_stark_py_hint
                "Found Python ${Python_VERSION}, but the required Development.Module component "
                "was not found. This usually means the selected Python has no development "
                "headers/artifacts available to CMake."
            )
        else()
            set(_stark_py_hint
                "No suitable Python 3.8+ interpreter with development module support was found."
            )
        endif()

        message(FATAL_ERROR
            "STARK: Python bindings cannot be built.\n"
            "${_stark_py_hint}\n"
            "Recommended fix — point STARK at the exact Python environment:\n"
            "  cmake -S . -B build \\\n"
            "    -DSTARK_BUILD_PYTHON_BINDINGS=ON \\\n"
            "    -DSTARK_PYTHON_EXECUTABLE=$HOME/miniforge3/envs/ENVNAME/bin/python\n"
            "Alternative — disable Python bindings:\n"
            "  cmake -S . -B build -DSTARK_BUILD_PYTHON_BINDINGS=OFF\n"
            "Current hints:\n"
            "  STARK_PYTHON_EXECUTABLE = ${STARK_PYTHON_EXECUTABLE}\n"
            "  Python_EXECUTABLE       = ${Python_EXECUTABLE}\n"
        )
    endif()

    if(NOT TARGET Python::Module)
        message(FATAL_ERROR
            "STARK: Python was found, but CMake did not create Python::Module.\n"
            "This should not happen when Development.Module is found correctly.\n"
            "Python executable: ${Python_EXECUTABLE}\n"
            "Python version:    ${Python_VERSION}\n"
        )
    endif()

    message(STATUS "STARK: Python executable: ${Python_EXECUTABLE}")
    message(STATUS "STARK: Python version:    ${Python_VERSION}")
    message(STATUS "STARK: Python include:    ${Python_INCLUDE_DIRS}")

    add_subdirectory(pystark/cpp)
endif()
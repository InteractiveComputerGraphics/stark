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

option(STARK_BUILD_EXAMPLES "Build STARK examples" ON)
option(STARK_BUILD_TESTS "Build STARK tests" OFF)
option(STARK_BUILD_PYTHON_BINDINGS "Build STARK Python bindings" OFF)

# STARK-specific Python override.
set(STARK_PYTHON_EXECUTABLE "NOT_SET" CACHE FILEPATH "Python executable used to build pystark")

add_subdirectory(stark)

if(STARK_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(STARK_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(STARK_BUILD_PYTHON_BINDINGS)
    # Prefer active virtualenv / Conda Python over system Python.
    set(Python_FIND_VIRTUALENV FIRST)
    set(Python_FIND_STRATEGY LOCATION)
    set(Python_FIND_IMPLEMENTATIONS CPython)

    # Optional explicit Python override.
    if(STARK_PYTHON_EXECUTABLE)
        set(Python_EXECUTABLE "${STARK_PYTHON_EXECUTABLE}"
            CACHE FILEPATH "Python executable used by FindPython" FORCE
        )
    endif()

    find_package(Python 3.8 QUIET COMPONENTS Interpreter Development.Module)

    if(NOT Python_FOUND)
        message(FATAL_ERROR
            "STARK: Python bindings cannot be built.\n"
            "CMake could not find a Python 3.8+ interpreter with the required "
            "Development.Module component.\n"
            "Common fixes:\n"
            "  1. Disable Python bindings:\n"
            "       cmake -S . -B build -DSTARK_BUILD_PYTHON_BINDINGS=OFF\n"
            "  2. Point STARK to the correct Python environment:\n"
            "       cmake -S . -B build \\\n"
            "         -DSTARK_BUILD_PYTHON_BINDINGS=ON \\\n"
            "         -DSTARK_PYTHON_EXECUTABLE=$HOME/miniforge3/envs/ENVNAME/bin/python\n"
            "Current Python hints:\n"
            "  STARK_PYTHON_EXECUTABLE = ${STARK_PYTHON_EXECUTABLE}\n"
            "  Python_EXECUTABLE       = ${Python_EXECUTABLE}\n"
            "  Python_VERSION          = ${Python_VERSION}\n"
        )
    endif()

    message(STATUS "STARK: Python executable: ${Python_EXECUTABLE}")
    message(STATUS "STARK: Python version:    ${Python_VERSION}")

    add_subdirectory(pystark/cpp)
endif()
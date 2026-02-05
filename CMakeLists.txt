cmake_minimum_required(VERSION 3.15...3.27)
project(stark)
set(CMAKE_CXX_STANDARD 17)

# Suppress warnings about ignored attributes when using SIMD types as template arguments
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wno-ignored-attributes)
endif()

add_subdirectory(stark)
add_subdirectory(examples)
add_subdirectory(tests)

# option(STARK_BUILD_PYTHON_BINDINGS "Build Python Bindings" ON)
# if(STARK_BUILD_PYTHON_BINDINGS)
# 	add_subdirectory(pystark/cpp)
# endif()

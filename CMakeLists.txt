cmake_minimum_required(VERSION 3.15...3.27)
project(stark)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(stark)
add_subdirectory(examples)
add_subdirectory(tests)

option(STARK_BUILD_PYTHON_BINDINGS "Build Python Bindings" ON)
if(STARK_BUILD_PYTHON_BINDINGS)
	add_subdirectory(pystark/cpp)
endif()

set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../stark/extern/Eigen)
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
set (CMAKE_CXX_FLAGS "-DEIGEN_MPL2_ONLY")

# Enable SIMD and suppress compiler warnings
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	add_compile_options("-march=native")
    add_compile_options("-Wno-ignored-attributes")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/arch:AVX")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	add_compile_options("-march=native")
endif()

# Dependencies
add_subdirectory(extern/Catch2)

# Source files
set(SOURCE_FILES
    rb_constraints.cpp
	)

# Create executable    
add_executable(stark_tests ${SOURCE_FILES})

# Include/link dependencies
## Stark
target_link_libraries(stark_tests PRIVATE stark)
target_include_directories(stark_tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../stark/include)

## Eigen
target_include_directories(stark_tests PUBLIC ${EIGEN3_INCLUDE_DIR})

## CMake
target_link_libraries(stark_tests PRIVATE Catch2::Catch2WithMain)

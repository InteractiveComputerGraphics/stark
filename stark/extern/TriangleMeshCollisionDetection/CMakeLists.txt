cmake_minimum_required(VERSION 3.9)

set(TMCD_ENABLE_AVX ON CACHE BOOL "Flag to enable AVX for TriangleMeshCollisionDetection")

# Enable SIMD and suppress compiler warnings
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	add_compile_options("-march=native")
    add_compile_options("-Wno-ignored-attributes")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/arch:AVX")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	add_compile_options("-march=native")
endif()

# Add files
set(SOURCE_FILES
	src/types.h
	src/info_structs.h
	src/helpers.h
	src/Vec3.h
	src/shuffle_lut.h
	src/RecursiveBuffer.h
	src/alignment_allocator.h
	src/ipc_toolkit_geometry_functions.h
	src/ipc_toolkit_geometry_functions.cpp
	src/Meshes.h
	src/Meshes.cpp
	src/AABBs.h
	src/AABBs.cpp
	src/Octree.h
	src/Octree.cpp
	src/BroadPhaseBase.h
	src/BroadPhaseBase.cpp
	src/BroadPhaseET.h
	src/BroadPhaseET.cpp
	src/BroadPhasePTEEBase.h
	src/BroadPhasePTEEBase.cpp
	src/BroadPhasePTEE.h
	src/BroadPhasePTEE.cpp
	src/BroadPhasePTEECCD.h
	src/BroadPhasePTEECCD.cpp
	src/IntersectionResults.h
	src/IntersectionDetection.h
	src/IntersectionDetection.cpp
	src/ProximityResults.h
	src/ProximityDetection.h
	src/ProximityDetection.cpp
	#src/TriangleMeshCollisionDetection.h
	#src/TriangleMeshCollisionDetection.cpp
)

add_library(TriangleMeshCollisionDetection ${SOURCE_FILES})

if(${TMCD_ENABLE_AVX})
	target_compile_definitions(TriangleMeshCollisionDetection PUBLIC TMCD_ENABLE_AVX)
endif()

# External dependencies
## OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
	target_link_libraries(TriangleMeshCollisionDetection OpenMP::OpenMP_CXX)
endif()

source_group("TriangleMeshCollisionDetection" FILES ${SOURCE_FILES})

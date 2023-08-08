# External dependencies
## OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
	src/IntersectionDetection.h
	src/IntersectionDetection.cpp
	src/ProximityDetection.h
	src/ProximityDetection.cpp
	#src/TriangleMeshCollisionDetection.h
	#src/TriangleMeshCollisionDetection.cpp
)

add_library(TriangleMeshCollisionDetection ${SOURCE_FILES})

source_group("TriangleMeshCollisionDetection" FILES ${SOURCE_FILES})

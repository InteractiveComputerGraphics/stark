cmake_minimum_required(VERSION 3.9)

# Eigen path (also used in projects below)
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/Eigen)
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
add_compile_options("-DEIGEN_MPL2_ONLY")

# Parallel compilation in MSVC
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/MP")
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

# Dependencies
add_subdirectory(extern/symx)
add_subdirectory(extern/TriangleMeshCollisionDetection)
add_subdirectory(extern/fmt)

# Source files
set(SOLVER_FILES
	include/stark
	src/solver/Settings.cpp
	src/solver/Settings.h
	src/solver/Callbacks.h
	src/solver/AdaptiveParameter.cpp
	src/solver/AdaptiveParameter.h
	src/solver/NewtonsMethod.cpp
	src/solver/NewtonsMethod.h
	src/solver/Energy.h
	src/solver/Stark.cpp
	src/solver/Stark.h
	)

set(MODELS_FILES
	src/models/Simulation.h
	src/models/Simulation.cpp
	src/models/distances.h
	src/models/distances.cpp
	src/models/friction_geometry.h
	src/models/friction_geometry.cpp
	src/models/time_integration.h
	src/models/time_integration.cpp
	src/models/rigidbody_transformations.h
	src/models/rigidbody_transformations.cpp
	src/models/TriangleMeshContacts.h
	src/models/TriangleMeshFriction.h
	src/models/Cloth.h
	src/models/Cloth.cpp
	src/models/DeformableSolids.h
	src/models/DeformableSolids.cpp
	src/models/RigidBodies.h
	src/models/RigidBodies.cpp
	src/models/RigidBodyContacts.h
	src/models/RigidBodyFriction.h
	src/models/Interactions.h
	src/models/Interactions.cpp
	src/models/RB_Deformable_Contacts.h
	src/models/RB_Deformable_Friction.h
	src/models/deformables/interval_types.h
	src/models/deformables/PointDynamics.h
	src/models/deformables/PointDynamics.cpp
	src/models/deformables/Deformables.h
	src/models/deformables/Deformables.cpp
	src/models/deformables/EnergyPointLumpedMassAndDamping.h
	src/models/deformables/EnergyPointLumpedMassAndDamping.cpp
	)

set(UTILS_FILES
	src/utils/Logger.cpp
	src/utils/Logger.h
	src/utils/Console.cpp
	src/utils/Console.h
	src/utils/mesh_utils.h
	src/utils/mesh_utils.cpp
	src/utils/mesh_generators.cpp
	src/utils/mesh_generators.h
	src/utils/inertia_tensors.cpp
	src/utils/inertia_tensors.h
	src/utils/unordered_array_set_and_map.h
	src/utils/MultiMesh.h
	src/utils/MultiMeshEdges.h
	src/utils/MultiMeshEdges.cpp
	src/utils/MeshWithDynamics.h
	src/utils/linear_system.cpp
	src/utils/linear_system.h
	src/utils/MultiMesh/IntervalVector.h
	src/utils/MultiMesh/IntervalConnectivity.h
	)

# Create lireary
add_library(stark ${SOLVER_FILES} ${MODELS_FILES} ${UTILS_FILES})

# Include/link dependencies
## OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(stark OpenMP::OpenMP_CXX)
endif()

## SymX
target_link_libraries(stark symx)
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/symx/include)

## TriangleMeshCollisionDetection
target_link_libraries(stark TriangleMeshCollisionDetection)
target_include_directories(stark PUBLIC extern/TriangleMeshCollisionDetection/include)

## Conjugate Gradient
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/symx/extern)

## vtkio
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/vtkio/include)

## fmt
target_link_libraries(stark fmt::fmt)
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/fmt/include)

## JanBenderUtilities
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/JanBenderUtilities/include)

## Eigen
target_include_directories(stark PUBLIC ${EIGEN3_INCLUDE_DIR})

## par_shapes
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/par_shapes/include)

# Group files
source_group("solver" FILES ${SOLVER_FILES})
source_group("models" FILES ${MODELS_FILES})
source_group("utils" FILES ${UTILS_FILES})
cmake_minimum_required(VERSION 3.9)

# Option to disable AVX for ARM-based devices
set(STARK_ENABLE_AVX ON CACHE BOOL "Flag to enable AVX for Stark and its dependencies")

# If AVX is disabled for stark, disable it for all dependencies that use AVX
if(NOT ${STARK_ENABLE_AVX})
	set(SYMX_ENABLE_AVX OFF CACHE BOOL "Flag to enable AVX for symx" FORCE)
	set(TMCD_ENABLE_AVX OFF CACHE BOOL "Flag to enable AVX for tmcd" FORCE)
endif()

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
set(CORE_FILES
	src/core/Callbacks.h
	src/core/Console.cpp
	src/core/Console.h
	src/core/include_ui.h
	src/core/Logger.cpp
	src/core/Logger.h
	src/core/NewtonsMethod.cpp
	src/core/NewtonsMethod.h
	src/core/Settings.cpp
	src/core/Settings.h
	src/core/EventDrivenScript.cpp
	src/core/EventDrivenScript.h
	src/core/Stark.cpp
	src/core/Stark.h
	)

set(MODELS_DEFORMABLES_FILES
	src/models/deformables/point/EnergyLumpedInertia.cpp
	src/models/deformables/point/EnergyLumpedInertia.h
	src/models/deformables/point/EnergyPrescribedPositions.cpp
	src/models/deformables/point/EnergyPrescribedPositions.h
	
	src/models/deformables/line/EnergySegmentStrain.cpp
	src/models/deformables/line/EnergySegmentStrain.h

	src/models/deformables/surface/EnergyTriangleStrain.cpp
	src/models/deformables/surface/EnergyTriangleStrain.h
	src/models/deformables/surface/EnergyDiscreteShells.cpp
	src/models/deformables/surface/EnergyDiscreteShells.h

	src/models/deformables/volume/EnergyTetStrain.cpp
	src/models/deformables/volume/EnergyTetStrain.h

	src/models/deformables/deformable_tools.cpp
	src/models/deformables/deformable_tools.h
	src/models/deformables/deformables_energies_include.h
	src/models/deformables/PointDynamics.cpp
	src/models/deformables/PointDynamics.h
	src/models/deformables/PointSetHandler.cpp
	src/models/deformables/PointSetHandler.h
	src/models/deformables/Deformables.cpp
	src/models/deformables/Deformables.h
	src/models/deformables/DeformablesMeshOutput.cpp
	src/models/deformables/DeformablesMeshOutput.h
)

set(MODELS_RIGIDBODIES_FILES
	src/models/rigidbodies/EnergyRigidBodyConstraints.cpp
	src/models/rigidbodies/EnergyRigidBodyConstraints.h
	src/models/rigidbodies/EnergyRigidBodyInertia.cpp
	src/models/rigidbodies/EnergyRigidBodyInertia.h
	src/models/rigidbodies/inertia_tensors.cpp
	src/models/rigidbodies/inertia_tensors.h
	src/models/rigidbodies/rigid_body_constraint_utils.h
	src/models/rigidbodies/RigidBodies.cpp
	src/models/rigidbodies/RigidBodies.h
	src/models/rigidbodies/RigidBodyConstraints.h
	src/models/rigidbodies/RigidBodyDynamics.cpp
	src/models/rigidbodies/RigidBodyDynamics.h
	src/models/rigidbodies/RigidBodyHandler.cpp
	src/models/rigidbodies/RigidBodyHandler.h
	src/models/rigidbodies/rigidbody_constraints_ui.h
	src/models/rigidbodies/rigidbody_transformations.cpp
	src/models/rigidbodies/rigidbody_transformations.h
	src/models/rigidbodies/RigidBodiesMeshOutput.cpp
	src/models/rigidbodies/RigidBodiesMeshOutput.h
)

set(MODELS_INTERACTIONS_FILES
	src/models/interactions/contact_and_friction_data.h
	src/models/interactions/friction_geometry.cpp
	src/models/interactions/friction_geometry.h
	src/models/interactions/EnergyFrictionalContact.cpp
	src/models/interactions/EnergyFrictionalContact.h
	src/models/interactions/EnergyAttachments.cpp
	src/models/interactions/EnergyAttachments.h
	src/models/interactions/Interactions.cpp
	src/models/interactions/Interactions.h
)

set(MODELS_PRESETS_FILES
	src/models/presets/Presets.cpp
	src/models/presets/Presets.h
	src/models/presets/DeformablesPresets.cpp
	src/models/presets/DeformablesPresets.h
	src/models/presets/deformables_preset_types.cpp
	src/models/presets/deformables_preset_types.h
	src/models/presets/RigidBodyPresets.cpp
	src/models/presets/RigidBodyPresets.h
)

set(MODELS_FILES
	${MODELS_DEFORMABLES_FILES}
	${MODELS_RIGIDBODIES_FILES}
	${MODELS_INTERACTIONS_FILES}
	${MODELS_PRESETS_FILES}

	src/models/include.h
	src/models/types.h
	src/models/distances.cpp
	src/models/distances.h
	src/models/IntervalVector.h
	src/models/MeshOutputGroups.cpp
	src/models/MeshOutputGroups.h
	src/models/Simulation.cpp
	src/models/Simulation.h
	src/models/time_integration.cpp
	src/models/time_integration.h
	)

set(UTILS_FILES
	src/utils/Mesh.h
	src/utils/mesh_utils.h
	src/utils/mesh_utils.cpp
	src/utils/mesh_generators.cpp
	src/utils/mesh_generators.h
	src/utils/unordered_array_set_and_map.h
	src/utils/blends.cpp
	src/utils/blends.h
	src/utils/include.h
	)

set(ALL_FILES
	include/stark
	${CORE_FILES} 
	${MODELS_FILES} 
	${UTILS_FILES}
)

# Create lireary
add_library(stark ${ALL_FILES})

# Include/link dependencies
## OpenMP
find_package(OpenMP REQUIRED)
target_link_libraries(stark OpenMP::OpenMP_CXX)

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

## TriangleMeshDistance
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/TriangleMeshDistance/include)

## fmt
target_link_libraries(stark fmt::fmt)
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/fmt/include)

## Eigen
target_include_directories(stark PUBLIC ${EIGEN3_INCLUDE_DIR})

## par_shapes
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/par_shapes/include)

## tinyobjloader
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/tinyobjloader/include)

# Group files
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ALL_FILES})

# Conditionally increase the stack size
## Stark comes with complex energy functions and may require a larger stack size
set(STACK_SIZE "10480000" CACHE STRING "Linker stack size in bytes")

target_link_options(stark INTERFACE
  "$<$<CXX_COMPILER_ID:MSVC>:/STACK:${STACK_SIZE}>"
  "$<$<CXX_COMPILER_ID:GNU>:-Wl,-z,stack-size=${STACK_SIZE}>"
  "$<$<CXX_COMPILER_ID:Clang>:-Wl,-z,--stack_size=${STACK_SIZE}>"
  "$<$<CXX_COMPILER_ID:AppleClang>:-Wl,-stack_size,${STACK_SIZE}>"
)

# Add /EHsc for exception handling (avoids warnings from vtkio)
if(MSVC)
    target_compile_options(stark INTERFACE /EHsc)
endif()

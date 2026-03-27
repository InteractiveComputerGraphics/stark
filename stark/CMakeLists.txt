cmake_minimum_required(VERSION 3.10)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

# Option to disable AVX for ARM-based devices
set(STARK_ENABLE_AVX ON CACHE BOOL "Flag to enable AVX for Stark and its dependencies")

# If AVX is disabled for stark, disable it for all dependencies that use AVX
if(NOT ${STARK_ENABLE_AVX})
	set(SYMX_ENABLE_AVX OFF CACHE BOOL "Flag to enable AVX for symx" FORCE)
	set(TMCD_ENABLE_AVX OFF CACHE BOOL "Flag to enable AVX for tmcd" FORCE)
endif()


# Parallel compilation in MSVC
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/MP")
endif()

# Enable SIMD and suppress compiler warnings
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	add_compile_options("-march=native")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/arch:AVX")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	add_compile_options("-march=native")
endif()

# Dependencies
set(SYMX_EIGEN_PUBLIC ON)
set(SYMX_FMT_PUBLIC   ON)
set(SYMX_HESS_STORAGE_FLOAT "float" CACHE STRING "Storage type for Hessian matrices (float or double)" FORCE)
add_subdirectory(extern/symx)
add_subdirectory(extern/TriangleMeshCollisionDetection)

# Source files
set(CORE_FILES
	src/core/Callbacks.h
	src/core/include_ui.h
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

## vtkio
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/vtkio/include)

## TriangleMeshDistance
target_include_directories(stark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/TriangleMeshDistance/include)

## fmt (provided by symx transitively)
## Eigen (provided by symx transitively)

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

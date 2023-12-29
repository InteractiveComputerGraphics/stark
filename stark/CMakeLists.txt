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
set(CORE_FILES
	src/core/AdaptiveParameter.cpp
	src/core/AdaptiveParameter.h
	src/core/Callbacks.h
	src/core/Console.cpp
	src/core/Console.h
	src/core/include_ui_core.h
	src/core/linear_system.cpp
	src/core/linear_system.h
	src/core/Logger.cpp
	src/core/Logger.h
	src/core/NewtonsMethod.cpp
	src/core/NewtonsMethod.h
	src/core/Settings.cpp
	src/core/Settings.h
	src/core/Stark.cpp
	src/core/Stark.h
	)

set(MODELS_DEFORMABLES_FILES
	src/models/deformables/EnergyEdgeStrain.cpp
	src/models/deformables/EnergyEdgeStrain.h
	src/models/deformables/EnergyPointInertia.cpp
	src/models/deformables/EnergyPointInertia.h
	src/models/deformables/EnergyPointPrescribedPositions.cpp
	src/models/deformables/EnergyPointPrescribedPositions.h
	src/models/deformables/EnergyTetStrain.cpp
	src/models/deformables/EnergyTetStrain.h
	src/models/deformables/EnergyTriangleBendingBergou06.cpp
	src/models/deformables/EnergyTriangleBendingBergou06.h
	src/models/deformables/EnergyTriangleBendingGrinspun03.cpp
	src/models/deformables/EnergyTriangleBendingGrinspun03.h
	src/models/deformables/EnergyTriangleStrain.cpp
	src/models/deformables/EnergyTriangleStrain.h
	src/models/deformables/DeformableHandler.cpp
	src/models/deformables/DeformableHandler.h
	src/models/deformables/DeformableSurfaceHandler.cpp
	src/models/deformables/DeformableSurfaceHandler.h
	src/models/deformables/Id.cpp
	src/models/deformables/Id.h
	src/models/deformables/PointDynamics.cpp
	src/models/deformables/PointDynamics.h
	src/models/deformables/PrescribedPointGroup.cpp
	src/models/deformables/PrescribedPointGroup.h
	src/models/deformables/PrescribedPointGroupWithTransformation.cpp
	src/models/deformables/PrescribedPointGroupWithTransformation.h
	src/models/deformables/DeformableSolidsLines.cpp
	src/models/deformables/DeformableSolidsLines.h
	src/models/deformables/DeformableSolidsSurfaces.cpp
	src/models/deformables/DeformableSolidsSurfaces.h
	src/models/deformables/DeformableSolidsVolumes.cpp
	src/models/deformables/DeformableSolidsVolumes.h
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
	src/models/rigidbodies/RigidBodiesInternal.cpp
	src/models/rigidbodies/RigidBodiesInternal.h
	src/models/rigidbodies/RigidBodyConstraints.h
	src/models/rigidbodies/RigidBodyDynamics.cpp
	src/models/rigidbodies/RigidBodyDynamics.h
	src/models/rigidbodies/RigidBodyHandler.cpp
	src/models/rigidbodies/RigidBodyHandler.h
	src/models/rigidbodies/rigidbody_constraints_ui.h
	src/models/rigidbodies/rigidbody_transformations.cpp
	src/models/rigidbodies/rigidbody_transformations.h
)

set(MODELS_INTERACTIONS_FILES
	src/models/deformables/contact_and_friction_data.h
	src/models/interactions/EnergyFrictionalContact.cpp
	src/models/interactions/EnergyFrictionalContact.h
	src/models/interactions/friction_geometry.cpp
	src/models/interactions/friction_geometry.h
)

set(MODELS_FILES
	${MODELS_DEFORMABLES_FILES}
	${MODELS_RIGIDBODIES_FILES}
	#${MODELS_INTERACTIONS_FILES}

	src/models/distances.cpp
	src/models/distances.h
	src/models/include_ui_models.h
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
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${ALL_FILES})

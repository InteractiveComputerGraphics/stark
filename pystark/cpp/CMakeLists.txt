cmake_minimum_required(VERSION 3.15...3.27)

# Find Python
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

# Optimized release build by default
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Fetch nanobind
include(FetchContent)
FetchContent_Declare(
  nanobind
  GIT_REPOSITORY      https://github.com/wjakob/nanobind.git
  GIT_TAG             80a30c8efb093b14f0e744bc7f6a9ef34beb3f7f # v1.9.2 release 
)
FetchContent_MakeAvailable(nanobind)
FetchContent_GetProperties(nanobind)

# Add nanobind to the project
add_subdirectory(${nanobind_SOURCE_DIR} ${CMAKE_BINARY_DIR}/nanobind)

# Enable SIMD and suppress compiler warnings
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	add_compile_options("-march=native")
    add_compile_options("-Wno-ignored-attributes")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/arch:AVX")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	add_compile_options("-march=native")
endif()

# Source files
set(CORE_FILES
	core/pystark_core.h
	core/pystark_Logger.cpp
	core/pystark_Console.cpp
	core/pystark_Settings.cpp
	)

set(UTILS_FILES
	utils/pystark_utils.cpp
	utils/pystark_utils.h
	)

set(MODELS_FILES
	# General
	models/pystark_models.h
	models/pystark_Simulation.cpp

	# Deformables
	models/deformables/pystark_PointDynamics.cpp
	models/deformables/pystark_PointSetHandler.cpp
	models/deformables/pystark_Deformables.cpp
	models/deformables/pystark_DeformablesMeshOutput.cpp

	models/deformables/point/pystark_EnergyLumpedInertia.cpp
	models/deformables/point/pystark_EnergyPrescribedPositions.cpp
	models/deformables/line/pystark_EnergySegmentStrain.cpp
	models/deformables/surface/pystark_EnergyTriangleStrain.cpp
	models/deformables/surface/pystark_EnergyDiscreteShells.cpp
	models/deformables/volume/pystark_EnergyTetStrain.cpp

	# Rigid bodies
	models/rigidbodies/pystark_RigidBodyDynamics.cpp
	models/rigidbodies/pystark_RigidBodies.cpp
	models/rigidbodies/pystark_RigidBodyHandler.cpp
	models/rigidbodies/pystark_RigidBodiesMeshOutput.cpp
	models/rigidbodies/pystark_rigidbody_constraints_ui.cpp

	# Interactions
	models/interactions/pystark_Interactions.cpp
	models/interactions/pystark_EnergyFrictionalContact.cpp
	models/interactions/pystark_EnergyAttachments.cpp

	# Presets
	models/presets/pystark_Presets.cpp
	models/presets/pystark_DeformablesPresets.cpp
	models/presets/pystark_deformables_preset_types.cpp
	models/presets/pystark_RigidBodyPresets.cpp
	)

set(PYSTARK_FILES
	${CORE_FILES}
	${MODELS_FILES}
	${UTILS_FILES}

	nanobind_stark_include_all.h
	pystark_main.cpp
	)

# Create the pystark module
nanobind_add_module(pystark ${PYSTARK_FILES})

# Link the pystark module to the stark library
target_link_libraries(pystark PUBLIC stark)
target_include_directories(pystark PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../stark/include)

# Group files
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${PYSTARK_FILES})

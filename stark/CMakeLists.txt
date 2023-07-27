# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Eigen (also used in projects below)
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/extern/Eigen)
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
endif()
set (CMAKE_CXX_FLAGS "-DEIGEN_MPL2_ONLY")

# Parallel compilation in MSVC
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/MP")
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
	src/solver/Stark.cpp
	src/solver/Stark.h
	)

set(MODELS_FILES
	src/models/distances.h
	src/models/time_integration.h
	src/models/time_integration.cpp
	src/models/Cloth.h
	src/models/Cloth.cpp
	src/models/Simulation.h
	src/models/Simulation.cpp
	)

set(UTILS_FILES
	src/utils/Logger.cpp
	src/utils/Logger.h
	src/utils/Console.cpp
	src/utils/Console.h
	src/utils/mesh_utils.h
	src/utils/mesh_utils.cpp
	src/utils/unordered_array_set_and_map.h
	src/utils/MultiMesh.h
	src/utils/MeshWithDynamics.h
	src/utils/linear_system.cpp
	src/utils/linear_system.h
	)

# Create lireary
add_library(stark ${SOLVER_FILES} ${MODELS_FILES} ${UTILS_FILES})

# Include/link dependencies
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

# Group files
source_group("solver" FILES ${SOLVER_FILES})
source_group("models" FILES ${MODELS_FILES})
source_group("utils" FILES ${UTILS_FILES})
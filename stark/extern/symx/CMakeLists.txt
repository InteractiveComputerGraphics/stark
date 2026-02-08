cmake_minimum_required(VERSION 3.10)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

set(SYMX_ENABLE_AVX2 ON CACHE BOOL "Flag to enable AVX2 for symx")
set(BSM_ENABLE_AVX2 ON CACHE BOOL "Flag to enable AVX2 for BlockedSparseMatrix")
set(SYMX_COMPILER_PATH "AUTO" CACHE STRING "Path to the compiler to use for runtime compilation. Set to AUTO to auto-detect.")
set(SYMX_HESS_STORAGE_FLOAT "float" CACHE STRING "Storage type for Hessian matrices (float or double)")

# Enable SIMD
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	add_compile_options("-march=native")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/arch:AVX")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	add_compile_options("-march=native")
endif()

# Pass the compiler path to the code
add_compile_definitions(SYMX_COMPILER_PATH="${SYMX_COMPILER_PATH}")

# Parallel compilation in MSVC
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/MP")
endif()

# Suppress deprecation warnings for MSVC
if (MSVC)
    add_compile_definitions(
        _SILENCE_ALL_MS_EXT_DEPRECATION_WARNINGS
    )
endif()


# Dependencies
add_subdirectory(extern/fmt)

# Files
set(SOURCE_FILES
	src/macros.h

	src/symbol/Matrix.cpp
	src/symbol/Matrix.h
	src/symbol/Scalar.cpp
	src/symbol/Scalar.h
	src/symbol/diff.cpp
	src/symbol/diff.h
	src/symbol/Expr.h
	src/symbol/Expressions.cpp
	src/symbol/Expressions.h
	src/symbol/Workspace.cpp
	src/symbol/Workspace.h
	src/symbol/Vector.cpp
	src/symbol/Vector.h
	src/symbol/utils.cpp
	src/symbol/utils.h
	src/symbol/unordered_array_map.h
	
	src/compile/Compilation.cpp
	src/compile/Compilation.h
	src/compile/static_methods.cpp
	src/compile/static_methods.h
	src/compile/compiler_utils.cpp
	src/compile/compiler_utils.h
	src/compile/Compiled.cpp
	src/compile/Compiled.h
	src/compile/FixedBranchSequence.cpp
	src/compile/FixedBranchSequence.h
	src/compile/Sequence.cpp
	src/compile/Sequence.h
	src/compile/AlignmentAllocator.h
	src/compile/type_utils.h
	src/compile/CompiledInLoop.h
	src/compile/CompiledInLoop_run.h
	src/compile/Element.cpp
	src/compile/Element.h
	src/compile/LabelledConnectivity.h
	src/compile/coloring.cpp
	src/compile/coloring.h
	src/compile/fetch_blocks.h
	src/compile/simd_shuffles.h
	src/compile/data_maps.h
	src/compile/MappedWorkspace.h
	src/compile/View.h
	
	
	src/solver/DeferredParallelTasks.h
	src/solver/Context.h
	src/solver/Context.cpp
	src/solver/Potential.h
	src/solver/Potential.cpp
	src/solver/GlobalPotential.h
	src/solver/GlobalPotential.cpp
	src/solver/NewtonsMethod.h
	src/solver/NewtonsMethod.cpp
	src/solver/gnuplot.h
	src/solver/gnuplot.cpp
	src/solver/solver_utils.h
	src/solver/fem_integrators.h
	src/solver/fem_integrators.cpp
	src/solver/mesh_generation.h
	src/solver/mesh_generation.cpp
	src/solver/mesh_ordering.h
	src/solver/mesh_ordering.cpp
	
	src/solver/second_order/Assembly.cpp
	src/solver/second_order/Assembly.h
	src/solver/second_order/project_to_PD.cpp
	src/solver/second_order/project_to_PD.h
	src/solver/second_order/chunked_vector.h
	src/solver/second_order/ElementHessians.cpp
	src/solver/second_order/ElementHessians.h
	src/solver/second_order/SecondOrderCompiledPotential.h
	src/solver/second_order/SecondOrderCompiledPotential.cpp
	src/solver/second_order/SecondOrderCompiledGlobal.h
	src/solver/second_order/SecondOrderCompiledGlobal.cpp
	)

# Add library
add_library(symx ${SOURCE_FILES})

if(${SYMX_ENABLE_AVX2})
	target_compile_definitions(symx PUBLIC SYMX_ENABLE_AVX2)
	target_compile_definitions(symx PUBLIC BSM_ENABLE_AVX2)
endif()

# Hessian storage type (float or double)
target_compile_definitions(symx PUBLIC SYMX_HESS_STORAGE_FLOAT=${SYMX_HESS_STORAGE_FLOAT})
message(STATUS "SymX: Hessian storage type set to: ${SYMX_HESS_STORAGE_FLOAT}")

# Codegen directory configuration
# Default to the project's build directory unless user overrides in cache.
set(SYMX_CODEGEN_DIR "CMAKE_BUILD_DIR" CACHE STRING "Directory for codegen output. Set to a custom path to override.")
if ("${SYMX_CODEGEN_DIR}" STREQUAL "CMAKE_BUILD_DIR" OR "${SYMX_CODEGEN_DIR}" STREQUAL "\"CMAKE_BUILD_DIR\"" OR "${SYMX_CODEGEN_DIR}" STREQUAL "" OR "${SYMX_CODEGEN_DIR}" STREQUAL "\"\"")
	set(SYMX_CODEGEN_DIR "${CMAKE_BINARY_DIR}/codegen")
endif()
target_compile_definitions(symx PUBLIC SYMX_CODEGEN_DIR="${SYMX_CODEGEN_DIR}")

# Print codegen dir
message(STATUS "SymX: Codegen directory set to: ${SYMX_CODEGEN_DIR}")


# Include/link dependencies
## OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(symx OpenMP::OpenMP_CXX)
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	## libdl (shared object loading)
	target_link_libraries(symx dl)
endif()

## Eigen
target_include_directories(symx SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/Eigen)

## BlockedSparseMatrix
target_include_directories(symx SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/BlockedSparseMatrix/include)

## picoSHA2
target_include_directories(symx SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/picoSHA2/include)

## fmt
target_link_libraries(symx fmt::fmt)
target_include_directories(symx SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/fmt/include)

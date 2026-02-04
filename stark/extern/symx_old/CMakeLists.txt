cmake_minimum_required(VERSION 3.9)

set(SYMX_ENABLE_AVX ON CACHE BOOL "Flag to enable AVX for symx")
set(BSM_ENABLE_AVX ON CACHE BOOL "Flag to enable AVX for BlockedSparseMatrix")

# Enable SIMD and suppress compiler warnings
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	add_compile_options("-march=native")
    add_compile_options("-Wno-ignored-attributes")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/arch:AVX")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	add_compile_options("-march=native")
endif()

# Parallel compilation in MSVC
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
	add_compile_options("/MP")
endif()

# Files
set(SOURCE_FILES
	src/AlignmentAllocator.h
	src/Assembly.cpp
	src/Assembly.h
	src/Compilation.cpp
	src/Compilation.h
	src/Compiled.cpp
	src/Compiled.h
	src/CompiledInLoop.h
	src/diff.cpp
	src/diff.h
	src/Element.cpp
	src/Element.h
	src/Energy.cpp
	src/Energy.h
	src/EvalSequence.cpp
	src/EvalSequence.h
	src/Expr.h
	src/Expressions.cpp
	src/Expressions.h
	src/FixedBranchSequence.cpp
	src/FixedBranchSequence.h
	src/GlobalEnergy.cpp
	src/GlobalEnergy.h
	src/hashing.h
	src/LabelledConnectivity.h
	src/lambdas.h
	src/Matrix.cpp
	src/Matrix.h
	src/project_to_PD.h
	src/Scalar.cpp
	src/Scalar.h
	src/Sequence.cpp
	src/Sequence.h
	src/simd_utils.h
	src/SymbolicWorkSpace.cpp
	src/SymbolicWorkSpace.h
	src/utils.cpp
	src/utils.h
	src/Vector.cpp
	src/Vector.h
	)

# Add library
add_library(symx ${SOURCE_FILES})

if(${SYMX_ENABLE_AVX})
	target_compile_definitions(symx PUBLIC SYMX_ENABLE_AVX)
	target_compile_definitions(symx PUBLIC BSM_ENABLE_AVX)
endif()

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
target_include_directories(symx PUBLIC ${EIGEN3_INCLUDE_DIR})

## BlockedSparseMatrix
target_include_directories(symx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/BlockedSparseMatrix/include)

## picoSHA2
target_include_directories(symx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/picoSHA2/include)

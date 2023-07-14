# OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Enable SIMD instructions and suppress related warnings
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
	src/CompiledDerivativesLoop.h
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
	src/lambdas.h
	src/Matrix.cpp
	src/Matrix.h
	src/project_to_PD.h
	src/Quaternion.h
	src/Quaternion.cpp
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
target_include_directories(symx PUBLIC extern)
target_include_directories(symx PUBLIC ${EIGEN3_INCLUDE_DIR})
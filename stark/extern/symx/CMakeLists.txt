cmake_minimum_required(VERSION 3.15)

# SymX options
set(SYMX_ENABLE_AVX2 ON CACHE BOOL "Flag to enable AVX2 for symx")
set(SYMX_COMPILER_PATH "AUTO" CACHE STRING "Path to the compiler to use for runtime compilation. Set to AUTO to auto-detect.")
set(SYMX_HESS_STORAGE_FLOAT "float" CACHE STRING "Storage type for Hessian matrices (float or double)")

# CMake option to make fmt and Eigen PUBLIC to use in parent projects
if(NOT DEFINED SYMX_FMT_PUBLIC)
    set(SYMX_FMT_PUBLIC OFF)
endif()
if(NOT DEFINED SYMX_EIGEN_PUBLIC)
    set(SYMX_EIGEN_PUBLIC OFF)
endif()


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
	src/solver/Logger.h
	src/solver/Logger.cpp
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

# C++17 is required; propagate to consumers
target_compile_features(symx PUBLIC cxx_std_17)

# OpenMP
if(APPLE)
	set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH};/opt/homebrew/opt/libomp)
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(symx PUBLIC OpenMP::OpenMP_CXX)
endif()

# SIMD: when AVX2 is enabled its intrinsics appear in public headers
if(${SYMX_ENABLE_AVX2})
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(symx PUBLIC -mavx2 -mfma -Wno-ignored-attributes)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_compile_options(symx PUBLIC -mavx2 -mfma)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
        target_compile_options(symx PUBLIC /arch:AVX2)
    endif()
endif()

# MSVC-specific build options (private — only affects symx compilation)
if(MSVC)
    target_compile_options(symx PRIVATE /MP)
    target_compile_definitions(symx PRIVATE _SILENCE_ALL_MS_EXT_DEPRECATION_WARNINGS)
endif()

# Enable/disable the use of AVX2 instructions
if(${SYMX_ENABLE_AVX2})
    target_compile_definitions(symx PUBLIC SYMX_ENABLE_AVX2 BSM_ENABLE_AVX2)
endif()

# Hessian storage type (float or double)
target_compile_definitions(symx PUBLIC SYMX_HESS_STORAGE_FLOAT=${SYMX_HESS_STORAGE_FLOAT})
message(STATUS "SymX: Hessian storage type set to: ${SYMX_HESS_STORAGE_FLOAT}")

# Compiler path for runtime JIT compilation
target_compile_definitions(symx PUBLIC SYMX_COMPILER_PATH="${SYMX_COMPILER_PATH}")

# Codegen directory: defaults to <build>/codegen, can be overridden via cache.
set(SYMX_CODEGEN_DIR "" CACHE STRING "Directory for codegen output. Leave empty to use <build>/codegen.")
if(SYMX_CODEGEN_DIR STREQUAL "")
    set(SYMX_CODEGEN_DIR "${CMAKE_BINARY_DIR}/codegen")
endif()
target_compile_definitions(symx PUBLIC SYMX_CODEGEN_DIR="${SYMX_CODEGEN_DIR}")
message(STATUS "SymX: Codegen directory set to: ${SYMX_CODEGEN_DIR}")

# Public include directory
target_include_directories(symx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Dependencies
## libdl for shared-object loading (Linux/GCC)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(symx PRIVATE dl)
endif()

## Eigen — header-only, types appear in the public API.
if(SYMX_EIGEN_PUBLIC)
    set(_eigen_vis PUBLIC)
else()
    set(_eigen_vis PRIVATE)
endif()

# Parse the bundled Eigen version once so we can print it in the status message.
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/extern/Eigen/Eigen/src/Core/util/Macros.h"
    _eigen_ver_lines REGEX "^#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION ")
foreach(_line ${_eigen_ver_lines})
    if(_line MATCHES "EIGEN_WORLD_VERSION ([0-9]+)")
        set(_ew ${CMAKE_MATCH_1})
    elseif(_line MATCHES "EIGEN_MAJOR_VERSION ([0-9]+)")
        set(_ema ${CMAKE_MATCH_1})
    elseif(_line MATCHES "EIGEN_MINOR_VERSION ([0-9]+)")
        set(_emi ${CMAKE_MATCH_1})
    endif()
endforeach()

# Source selection: pre-existing target → system installation → bundled copy.
if(TARGET Eigen3::Eigen)
    message(STATUS "SymX: Eigen3 — using pre-existing Eigen3::Eigen target")
    target_link_libraries(symx ${_eigen_vis} Eigen3::Eigen)
else()
    find_package(Eigen3 QUIET NO_CMAKE_PACKAGE_REGISTRY)
    if(Eigen3_FOUND)
        message(STATUS "SymX: Eigen3 — found system installation (v${Eigen3_VERSION})")
        target_link_libraries(symx ${_eigen_vis} Eigen3::Eigen)
    else()
        unset(Eigen3_DIR CACHE)  # prevent NOTFOUND entry from polluting the CMake cache
        message(STATUS "SymX: Eigen3 — using copy bundled with SymX (v${_ew}.${_ema}.${_emi})")
        target_include_directories(symx SYSTEM ${_eigen_vis}
            ${CMAKE_CURRENT_SOURCE_DIR}/extern/Eigen)
    endif()
endif()

## fmt — compiled library; used only inside .cpp files, not in public headers.
# Parse the bundled fmt version once so we can print it in the status message.
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/extern/fmt/include/fmt/core.h"
    _fmt_ver_str REGEX "^#define FMT_VERSION ")
if(_fmt_ver_str MATCHES "FMT_VERSION ([0-9]+)")
    math(EXPR _fmaj "${CMAKE_MATCH_1} / 10000")
    math(EXPR _fmin "(${CMAKE_MATCH_1} % 10000) / 100")
    math(EXPR _fpat "${CMAKE_MATCH_1} % 100")
else()
    set(_fmaj "?")
    set(_fmin "?")
    set(_fpat "?")
endif()

# Source selection: pre-existing target → system installation → bundled copy.
if(TARGET fmt::fmt)
    message(STATUS "SymX: fmt — using pre-existing fmt::fmt target")
else()
    find_package(fmt QUIET)
    if(fmt_FOUND)
        message(STATUS "SymX: fmt — found system installation (v${fmt_VERSION})")
    else()
        unset(fmt_DIR CACHE)  # prevent NOTFOUND entry from polluting the CMake cache
        message(STATUS "SymX: fmt — using copy bundled with SymX (v${_fmaj}.${_fmin}.${_fpat})")
        add_subdirectory(extern/fmt)
    endif()
endif()

target_link_libraries(symx PRIVATE fmt::fmt)
if(SYMX_FMT_PUBLIC)
    target_include_directories(symx SYSTEM PUBLIC
        $<TARGET_PROPERTY:fmt::fmt,INTERFACE_INCLUDE_DIRECTORIES>)
endif()

## BlockedSparseMatrix — header-only, types appear in the public API (NewtonsMethod.h).
target_include_directories(symx SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/BlockedSparseMatrix/include)

## picoSHA2 — header-only, type appears in the public API (Scalar.h::_get_checksum).
target_include_directories(symx SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/picoSHA2/include)


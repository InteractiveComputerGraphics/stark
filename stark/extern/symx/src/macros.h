#pragma once
#include <iostream>
#include <string>
#include <filesystem>
#include <cmath>

// ---------- compiler portability ----------
#if defined(_MSC_VER)
  #define SYMX_FORCE_INLINE __forceinline
  #define SYMX_RESTRICT __restrict
  #define SYMX_LIKELY(x)   (x)
  #define SYMX_UNLIKELY(x) (x)
  #define SYMX_ASSUME(x)   __assume(x)
#else
  #define SYMX_FORCE_INLINE __attribute__((always_inline)) inline
  #define SYMX_RESTRICT __restrict__
  #define SYMX_LIKELY(x)   (__builtin_expect(!!(x), 1))
  #define SYMX_UNLIKELY(x) (__builtin_expect(!!(x), 0))
  #define SYMX_ASSUME(x)   do { if(!(x)) __builtin_unreachable(); } while(0)
#endif

// Vectorize-hint (ignore assumed loop-carried deps)
#if defined(__INTEL_COMPILER) || defined(__INTEL_LLVM_COMPILER)
  #define SYMX_IVDEP _Pragma("ivdep")
#elif defined(_MSC_VER)
  // MSVC: available as a loop pragma
  #define SYMX_IVDEP _Pragma("loop(ivdep)")
#elif defined(__clang__)
  // Clang: use loop hints
  #define SYMX_IVDEP _Pragma("clang loop vectorize(enable)")
#elif defined(__GNUC__)
  // GCC: native ivdep
  #define SYMX_IVDEP _Pragma("GCC ivdep")
#else
  #define SYMX_IVDEP /* no-op */
#endif


#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <stdexcept>

// Pull intrinsics only if AVX2 is available at compile time.
#if defined(SYMX_ENABLE_AVX2)
  #include <immintrin.h>
  #define SYMX_ENABLE_AVX2_SHUFFLES
#endif


// ---------- small utilities ----------
namespace symx::detail
{
  SYMX_FORCE_INLINE int32_t round_up(int32_t x, int32_t m) 
  {
    return (x + (m - 1)) / m * m;
  }
  SYMX_FORCE_INLINE int32_t ceil_div(int32_t x, int32_t m)
  {
    if (x%m == 0) {
      return x/m;
    }
    else {
      return x/m + 1;
    }
  }

  SYMX_FORCE_INLINE void cast_inplace_f32_to_f64(void* SYMX_RESTRICT base, std::size_t n)
  {
    float*  SYMX_RESTRICT f = reinterpret_cast<float*>(base);
    double* SYMX_RESTRICT d = reinterpret_cast<double*>(base);
    for (std::size_t i = n; i-- > 0; ) d[i] = (double)f[i];
  }

  SYMX_FORCE_INLINE void cast_inplace_f64_to_f32(void* SYMX_RESTRICT base, std::size_t n)
  {
    double* SYMX_RESTRICT d = reinterpret_cast<double*>(base);
    float*  SYMX_RESTRICT f = reinterpret_cast<float*>(base);
    for (std::size_t i = 0; i < n; ++i) f[i] = (float)d[i];
  }

  template<typename T>
  SYMX_FORCE_INLINE int32_t find_nan(T* input, int32_t n)
  {
    for (int32_t i = 0; i < n; ++i) {
      if (std::isnan(input[i])) {
        return i;  // Return index of first NaN found
      }
    }
    return -1;  // Return -1 if no NaN found
  }
}

// Codegen directory helper
namespace symx
{
  inline std::string get_codegen_dir()
  {
    // Turn preprocessor token into a string literal reliably
    constexpr std::string_view codegen_dir = SYMX_CODEGEN_DIR;
    const std::string path = std::string(codegen_dir);

    // Ensure directory exists. Try to create. Print error if invalid.
    if (!std::filesystem::exists(path)) {
      try {
        std::filesystem::create_directory(path);
      } 
      catch (const std::exception& e) {
        std::cout << "symx error: Failed to create directory '" << path << "': " << e.what() << std::endl;
        exit(-1);
      }
    }
    
    return path;
  }
}

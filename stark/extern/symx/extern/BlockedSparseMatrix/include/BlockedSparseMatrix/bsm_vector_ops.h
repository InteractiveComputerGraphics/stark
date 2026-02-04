#pragma once
/**
 * @file bsm_vector_ops.h
 * @brief Low-level vector operations for BSM solvers.
 * 
 * These are micro-operations optimized for auto-vectorization with OpenMP SIMD.
 * All functions assume aligned pointers and use restrict qualifiers.
 */

#include <omp.h>
#include <cstring>
#include <cmath>

namespace bsm
{
namespace detail
{
    // ============================================================================
    // Platform-specific macros
    // ============================================================================
#ifdef _MSC_VER
    #define BSM_RESTRICT __restrict
    #define BSM_ASSUME_ALIGNED(ptr, align) (ptr)
    #define BSM_USE_ATOMIC_REDUCTION 1
#else
    #define BSM_RESTRICT __restrict__
    #define BSM_ASSUME_ALIGNED(ptr, align) static_cast<decltype(ptr)>(__builtin_assume_aligned(ptr, align))
    #define BSM_USE_ATOMIC_REDUCTION 0
#endif

    // ============================================================================
    // Constants
    // ============================================================================
    
    // Threshold below which OpenMP parallel overhead exceeds benefits
    static constexpr int N_MIN_FOR_PARALLEL = 10000;
    
    // Alignment for SIMD (64 bytes = AVX-512 / cache line)
    static constexpr size_t SIMD_ALIGN = 64;

    // ============================================================================
    // Vector operations
    // All pointers assumed to be SIMD_ALIGN-byte aligned and non-overlapping
    // ============================================================================

    /**
     * @brief z = a*x + b*y  (axpby - scaled sum of two vectors)
     */
    template<typename T>
    inline void vec_axpby(
        T* BSM_RESTRICT z,
        T a, const T* BSM_RESTRICT x,
        T b, const T* BSM_RESTRICT y,
        int n, int nthreads)
    {
        z = BSM_ASSUME_ALIGNED(z, SIMD_ALIGN);
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        y = BSM_ASSUME_ALIGNED(y, SIMD_ALIGN);
        
        #pragma omp parallel for simd schedule(static) num_threads(nthreads) if(n >= N_MIN_FOR_PARALLEL)
        for (int i = 0; i < n; i++) {
            z[i] = a * x[i] + b * y[i];
        }
    }

    /**
     * @brief x += a*y  (axpy in-place)
     */
    template<typename T>
    inline void vec_axpy_inplace(
        T* BSM_RESTRICT x,
        T a, const T* BSM_RESTRICT y,
        int n, int nthreads)
    {
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        y = BSM_ASSUME_ALIGNED(y, SIMD_ALIGN);
        
        #pragma omp parallel for simd schedule(static) num_threads(nthreads) if(n >= N_MIN_FOR_PARALLEL)
        for (int i = 0; i < n; i++) {
            x[i] += a * y[i];
        }
    }

    /**
     * @brief z = x (copy)
     */
    template<typename T>
    inline void vec_copy(
        T* BSM_RESTRICT z,
        const T* BSM_RESTRICT x,
        int n, int nthreads)
    {
        z = BSM_ASSUME_ALIGNED(z, SIMD_ALIGN);
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        
        #pragma omp parallel for simd schedule(static) num_threads(nthreads) if(n >= N_MIN_FOR_PARALLEL)
        for (int i = 0; i < n; i++) {
            z[i] = x[i];
        }
    }

    /**
     * @brief r = x - y (difference)
     */
    template<typename T>
    inline void vec_sub(
        T* BSM_RESTRICT r,
        const T* BSM_RESTRICT x,
        const T* BSM_RESTRICT y,
        int n, int nthreads)
    {
        r = BSM_ASSUME_ALIGNED(r, SIMD_ALIGN);
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        y = BSM_ASSUME_ALIGNED(y, SIMD_ALIGN);
        
        #pragma omp parallel for simd schedule(static) num_threads(nthreads) if(n >= N_MIN_FOR_PARALLEL)
        for (int i = 0; i < n; i++) {
            r[i] = x[i] - y[i];
        }
    }

    /**
     * @brief Fused: x += a*p, r -= a*q  (used in CG iteration)
     */
    template<typename T>
    inline void vec_fused_xr_update(
        T* BSM_RESTRICT x, T* BSM_RESTRICT r,
        T a,
        const T* BSM_RESTRICT p, const T* BSM_RESTRICT q,
        int n, int nthreads)
    {
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        r = BSM_ASSUME_ALIGNED(r, SIMD_ALIGN);
        p = BSM_ASSUME_ALIGNED(p, SIMD_ALIGN);
        q = BSM_ASSUME_ALIGNED(q, SIMD_ALIGN);
        
        #pragma omp parallel for simd schedule(static) num_threads(nthreads) if(n >= N_MIN_FOR_PARALLEL)
        for (int i = 0; i < n; i++) {
            x[i] += a * p[i];
            r[i] -= a * q[i];
        }
    }

    /**
     * @brief dot product: sum(x[i] * y[i])
     */
    template<typename T>
    inline T vec_dot(
        const T* BSM_RESTRICT x,
        const T* BSM_RESTRICT y,
        int n, int nthreads)
    {
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        y = BSM_ASSUME_ALIGNED(y, SIMD_ALIGN);
        
        T sum = T(0);
        
#if BSM_USE_ATOMIC_REDUCTION
        if (n >= N_MIN_FOR_PARALLEL) {
            #pragma omp parallel num_threads(nthreads)
            {
                T local_sum = T(0);
                #pragma omp for schedule(static)
                for (int i = 0; i < n; i++) {
                    local_sum += x[i] * y[i];
                }
                #pragma omp atomic
                sum += local_sum;
            }
        } else {
            for (int i = 0; i < n; i++) {
                sum += x[i] * y[i];
            }
        }
#else
        #pragma omp parallel for simd schedule(static) num_threads(nthreads) reduction(+:sum) if(n >= N_MIN_FOR_PARALLEL)
        for (int i = 0; i < n; i++) {
            sum += x[i] * y[i];
        }
#endif
        return sum;
    }

    /**
     * @brief squared norm: sum(x[i]^2)
     */
    template<typename T>
    inline T vec_norm_sq(
        const T* BSM_RESTRICT x,
        int n, int nthreads)
    {
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        
        T sum = T(0);
        
#if BSM_USE_ATOMIC_REDUCTION
        if (n >= N_MIN_FOR_PARALLEL) {
            #pragma omp parallel num_threads(nthreads)
            {
                T local_sum = T(0);
                #pragma omp for schedule(static)
                for (int i = 0; i < n; i++) {
                    local_sum += x[i] * x[i];
                }
                #pragma omp atomic
                sum += local_sum;
            }
        } else {
            for (int i = 0; i < n; i++) {
                sum += x[i] * x[i];
            }
        }
#else
        #pragma omp parallel for simd schedule(static) num_threads(nthreads) reduction(+:sum) if(n >= N_MIN_FOR_PARALLEL)
        for (int i = 0; i < n; i++) {
            sum += x[i] * x[i];
        }
#endif
        return sum;
    }

} // namespace detail
} // namespace bsm

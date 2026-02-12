#pragma once
/**
 * @file solve_pcg.h
 * @brief Monolithic Preconditioned Conjugate Gradient solver for BlockedSparseMatrix.
 * 
 * This solver is designed to eliminate interface overhead by keeping all data
 * in aligned buffers throughout the solve. It should be used instead of the
 * std::function-based CG solver in ConjugateGradientMethod.h for performance.
 */

#include "bsm_vector_ops.h"
#include "AlignmentAllocator.h"
#include <cstring>
#include <cmath>

namespace bsm
{
    /**
     * @brief Return struct for PCG solve
     */
    struct PCGInfo
    {
        bool converged = false;
        int n_iterations = 1;
        double error = -1.0;
        bool found_indefiniteness = false;
    };

    /**
     * @brief Context for PCG solver to reuse allocated buffers across solves.
     * 
     * This struct holds all the work vectors needed by the PCG solver.
     * By reusing this context across Newton iterations, we avoid repeated
     * memory allocations when the problem size remains constant.
     */
    struct PCGContext
    {
        bsm::avx::avector<double, bsm::detail::SIMD_ALIGN> x_buf;
        bsm::avx::avector<double, bsm::detail::SIMD_ALIGN> b_buf;
        bsm::avx::avector<double, bsm::detail::SIMD_ALIGN> r_buf;
        bsm::avx::avector<double, bsm::detail::SIMD_ALIGN> z_buf;
        bsm::avx::avector<double, bsm::detail::SIMD_ALIGN> p_buf;
        bsm::avx::avector<double, bsm::detail::SIMD_ALIGN> Ap_buf;

        /**
         * @brief Ensure all buffers have at least the required capacity.
         * @param n Required size for each buffer
         */
        void resize(int n)
        {
            if (static_cast<int>(x_buf.size()) < n) {
                x_buf.resize(n);
                b_buf.resize(n);
                r_buf.resize(n);
                z_buf.resize(n);
                p_buf.resize(n);
                Ap_buf.resize(n);
            }
        }
    };

    /**
     * @brief Monolithic Preconditioned Conjugate Gradient solver with reusable context.
     * 
     * This overload reuses buffers from a PCGContext to avoid repeated allocations
     * when solving multiple linear systems of the same size (e.g., across Newton iterations).
     * 
     * @tparam BSM_TYPE BlockedSparseMatrix type
     * 
     * @param bsm Reference to the BlockedSparseMatrix (must have preconditioning prepared)
     * @param x_out Output solution vector (will be overwritten)
     * @param b_in Input right-hand side vector
     * @param n Size of the vectors
     * @param abs_tol Absolute tolerance for convergence
     * @param rel_tol Relative tolerance for convergence  
     * @param max_iter Maximum number of iterations
     * @param n_threads Number of threads (must be > 0, no -1 default handling here)
     * @param stop_on_indef Stop if matrix appears indefinite
     * @param ctx PCGContext with pre-allocated buffers (will be resized if needed)
     * @return PCGInfo struct with convergence info
     */
    template<typename BSM_TYPE>
    inline PCGInfo solve_pcg(
        BSM_TYPE& bsm,
        double* x_out,
        const double* b_in,
        int n,
        double abs_tol,
        double rel_tol,
        int max_iter,
        int n_threads,
        bool stop_on_indef,
        PCGContext& ctx)
    {
        using namespace detail;
        
        PCGInfo info;
        
        // ===================================================================
        // Step 1: Ensure buffers are large enough and copy inputs
        // ===================================================================
        ctx.resize(n);
        
        double* BSM_RESTRICT x = ctx.x_buf.data();
        double* BSM_RESTRICT b = ctx.b_buf.data();
        
        std::memcpy(x, x_out, n * sizeof(double));
        std::memcpy(b, b_in, n * sizeof(double));
        
        x = BSM_ASSUME_ALIGNED(x, SIMD_ALIGN);
        b = BSM_ASSUME_ALIGNED(b, SIMD_ALIGN);

        // ===================================================================
        // Step 2: Compute ||b||^2 for relative tolerance
        // ===================================================================
        const double b_norm_sq = vec_norm_sq(b, n, n_threads);
        
        // Handle zero RHS
        if (b_norm_sq < abs_tol * abs_tol) {
            std::memset(x_out, 0, n * sizeof(double));
            info.converged = true;
            info.n_iterations = 0;
            info.error = 0;
            return info;
        }

        // ===================================================================
        // Step 3: Get work vector pointers (already allocated in context)
        // ===================================================================
        double* BSM_RESTRICT r  = BSM_ASSUME_ALIGNED(ctx.r_buf.data(), SIMD_ALIGN);
        double* BSM_RESTRICT z  = BSM_ASSUME_ALIGNED(ctx.z_buf.data(), SIMD_ALIGN);
        double* BSM_RESTRICT p  = BSM_ASSUME_ALIGNED(ctx.p_buf.data(), SIMD_ALIGN);
        double* BSM_RESTRICT Ap = BSM_ASSUME_ALIGNED(ctx.Ap_buf.data(), SIMD_ALIGN);

        // ===================================================================
        // Step 4: r = b - A*x  (initial residual)
        // ===================================================================
        bsm._spmxv(Ap, x, n_threads);
        vec_sub(r, b, Ap, n, n_threads);

        // ===================================================================
        // Step 5: Check initial residual
        // ===================================================================
        double r_norm_sq = vec_norm_sq(r, n, n_threads);
        double error = std::sqrt(r_norm_sq / b_norm_sq);
        const double error_0 = error;
        
        if (error < abs_tol) {
            std::memcpy(x_out, x, n * sizeof(double));
            info.converged = true;
            info.n_iterations = 0;
            info.error = error;
            return info;
        }

        // ===================================================================
        // Step 6: z = M^-1 * r (apply preconditioner)
        // ===================================================================
        bsm.apply_preconditioning(z, r, n_threads);

        // ===================================================================
        // Step 7: p = z
        // ===================================================================
        vec_copy(p, z, n, n_threads);

        // ===================================================================
        // Step 8: rz = r.z (cached for beta computation)
        // ===================================================================
        double rz = vec_dot(r, z, n, n_threads);

        // ===================================================================
        // Step 9: Main PCG iteration loop
        // ===================================================================
        for (int it = 1; it <= max_iter; it++) {
            
            // Ap = A*p
            bsm._spmxv(Ap, p, n_threads);

            // pAp = p.Ap
            const double pAp = vec_dot(p, Ap, n, n_threads);

            // Check for indefiniteness
            if (pAp <= 0.0) {
                info.found_indefiniteness = true;
                if (stop_on_indef) {
                    std::memcpy(x_out, x, n * sizeof(double));
                    info.converged = false;
                    info.n_iterations = it;
                    info.error = error;
                    return info;
                }
            }

            // alpha = rz / pAp
            const double alpha = rz / pAp;

            // x += alpha*p,  r -= alpha*Ap (fused)
            vec_fused_xr_update(x, r, alpha, p, Ap, n, n_threads);

            // Check convergence
            r_norm_sq = vec_norm_sq(r, n, n_threads);
            error = std::sqrt(r_norm_sq / b_norm_sq);

            if (error < abs_tol || error / error_0 < rel_tol) {
                std::memcpy(x_out, x, n * sizeof(double));
                info.converged = true;
                info.n_iterations = it;
                info.error = error;
                return info;
            }

            // z = M^-1 * r
            bsm.apply_preconditioning(z, r, n_threads);

            // beta = r_new.z_new / r_old.z_old
            const double rz_old = rz;
            rz = vec_dot(r, z, n, n_threads);
            const double beta = rz / rz_old;

            // p = z + beta*p
            vec_axpby(p, 1.0, z, beta, p, n, n_threads);
        }

        // ===================================================================
        // Step 10: Max iterations reached
        // ===================================================================
        std::memcpy(x_out, x, n * sizeof(double));
        info.converged = false;
        info.n_iterations = max_iter;
        info.error = error;
        return info;
    }

} // namespace bsm

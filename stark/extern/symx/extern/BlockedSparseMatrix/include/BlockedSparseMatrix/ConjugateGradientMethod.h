#pragma once
#include <iostream>
#include <string>
#include <functional>
#ifdef BSM_ENABLE_AVX
#include <immintrin.h>
#endif

#include "AlignmentAllocator.h"


namespace cg
{
    static int N_ITEMS_FOR_PARALLELISM = 4000;

    /**
    * Struct for additional conjugate gradient solving informations.
    */
	struct Info
	{
        bool converged = false; /**< Success of the solving */
        int n_iterations = -1; /**< Number of used iterations. Always lower than specified maximum number of iterations */
        double error = -1.0; /**< Remaing error of the solution calculated as: sqrt(r.T * r / (b.T * b)) with r is residual and b is the given vector */
        std::string error_msg = ""; /**< Error message if solving was not successful */

        #ifdef BENCHMARK_ENABLED
        std::array<double, 9> timing;
        #endif
    };

    /**
    * Implementation of vector addition and scaling.
    *
    * @tparam FLOAT Float type to use. Currentl only double supported.
    *
    * @param solution Solution vector. Must point to size values.
    * @param alpha Scaling of vector a.
    * @param a Augend vector. Must point to size values.
    * @param beta Scaling of vector b.
    * @param b Addend vector Must point to size values.
    * @param size Number of elements in solution, a and b.
    * @param n_threads Number of threads to use. Default is system optimized.
    */
    template<typename FLOAT = double>
    inline void vector_add(FLOAT* solution, const FLOAT alpha, const FLOAT* a, const FLOAT beta, const FLOAT* b, const int size, const int n_threads)
    {
        const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() : n_threads;
#ifdef BSM_ENABLE_AVX
        __m256d* s_avx = reinterpret_cast<__m256d*>(solution);
        const __m256d* a_avx = reinterpret_cast<const __m256d*>(a);
        const __m256d* b_avx = reinterpret_cast<const __m256d*>(b);
        const int n_avx = size / 4;

        __m256d alpha_avx = _mm256_set1_pd(alpha);
        __m256d beta_avx = _mm256_set1_pd(beta);
#endif
        if (alpha == static_cast<FLOAT>(1.0) && solution == a) {
#ifdef BSM_ENABLE_AVX
            #pragma omp parallel for schedule(static) num_threads(n_threads_) if(size > N_ITEMS_FOR_PARALLELISM)
            for (int i = 0; i < n_avx; i++) {
                s_avx[i] = _mm256_fmadd_pd(beta_avx, b_avx[i], s_avx[i]);
            }

            for (int i = 4 * (size / 4); i < size; i++) {
                solution[i] += beta * b[i];
            }
#else
            #pragma omp parallel for schedule(static) num_threads(n_threads_) if(size > N_ITEMS_FOR_PARALLELISM)
            for (int i = 0; i < size; i++) {
                solution[i] += beta * b[i];
            }
#endif
        }
        else {
#ifdef BSM_ENABLE_AVX
            #pragma omp parallel for schedule(static) num_threads(n_threads_) if(size > N_ITEMS_FOR_PARALLELISM)
            for (int i = 0; i < n_avx; i++) {
                s_avx[i] = _mm256_add_pd(_mm256_mul_pd(alpha_avx, a_avx[i]), _mm256_mul_pd(beta_avx, b_avx[i]));
            }

            for (int i = 4 * (size / 4); i < size; i++) {
                solution[i] = alpha * a[i] + beta * b[i];
            }
#else
            #pragma omp parallel for schedule(static) num_threads(n_threads_) if(size > N_ITEMS_FOR_PARALLELISM)
            for (int i = 0.0; i < size; i++) {
                solution[i] = alpha * a[i] + beta * b[i];
            }
#endif
        }
    }

    /**
    * Implementation of vector dot product.
    *
    * @tparam FLOAT Float type to use. Default: double.
    *
    * @param a First vector. Must point to size values.
    * @param b Second vector Must point to size values.
    * @param size Number of elements in a and b.
    * @param n_threads Number of threads to use. Default is system optimized.
    * 
    * @return Dot product of a and b.
    */
    template<typename FLOAT = double>
    inline FLOAT vector_dot(const FLOAT* a, const FLOAT* b, const int size, const int n_threads)
    {
        FLOAT sum = static_cast<FLOAT>(0.0);
#ifdef BSM_ENABLE_AVX
        const __m256d* a_avx = reinterpret_cast<const __m256d*>(a);
        const __m256d* b_avx = reinterpret_cast<const __m256d*>(b);
        const int n_avx = size / 4;
#else
        const int n_avx = size;
#endif

        if (size > N_ITEMS_FOR_PARALLELISM) {
            const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() : n_threads;
            #pragma omp parallel num_threads(n_threads_)
            {
                const int thread_id = omp_get_thread_num();
                const int chunksize = n_avx / n_threads_;
                int begin = 0;
                int end = n_avx;
                if (thread_id != 0) {
                    begin = thread_id * chunksize;
                }
                if (thread_id != n_threads_ - 1) {
                    end = (thread_id + 1) * chunksize;
                }

#ifdef BSM_ENABLE_AVX
                __m256d loc_sum = _mm256_set1_pd(0.0);
                for (int i = begin; i < end; i++) {
                    loc_sum = _mm256_fmadd_pd(a_avx[i], b_avx[i], loc_sum);
                }
                const double* s = reinterpret_cast<double*>(&loc_sum);
                const double loc_sum_scalar = s[0] + s[1] + s[2] + s[3];
#else
                FLOAT loc_sum = 0.0;
                for (int i = begin; i < end; i++) {
                    loc_sum += a[i] * b[i];
                }
                const double loc_sum_scalar = loc_sum;
#endif

                #pragma omp atomic
                sum += loc_sum_scalar;
            }
        }
        else {
#ifdef BSM_ENABLE_AVX
            __m256d loc_sum = _mm256_set1_pd(0.0);
            for (int i = 0; i < n_avx; i++) {
                loc_sum = _mm256_fmadd_pd(a_avx[i], b_avx[i], loc_sum);
            }
            const double* s = reinterpret_cast<double*>(&loc_sum);
            const double loc_sum_scalar = s[0] + s[1] + s[2] + s[3];
#else
            FLOAT loc_sum = 0.0;
            for (int i = 0; i < n_avx; i++) {
                loc_sum += a[i] * b[i];
            }
            const double loc_sum_scalar = loc_sum;
#endif
            sum += loc_sum_scalar;
        }

#ifdef BSM_ENABLE_AVX
        for (int i = 4 * (size / 4); i < size; i++) {
            sum += a[i] * b[i];
        }
#endif

        return sum;
    }

    template<typename FLOAT = double>
    std::function<void(FLOAT* z, const FLOAT* r, const int size)> no_preconditioner = [](FLOAT* z, const FLOAT* r, const int size) { for (int i = 0; i < size; i++) { z[i] = r[i]; } };
    
    /**
    * Implementation of the Preconditioned Conjugated Gradient method
    * as described here
    * https://en.wikipedia.org/wiki/Conjugate_gradient_method#The_preconditioned_conjugate_gradient_method
    * to solve a linear system of the type A * x = b.
    * 
    * @tparam FLOAT Float type to use. Default: double.
    * 
    * @param x Solution vector. Its contents will be used as warmstart.
    * @param b Right hand side.
    * @param size Number of elements in x and b.
    * @param error_tolerance Error for the stopping criteria defined as error = sqrt( ((A*x - b).T * (A*x - b)) / (b.T * b) ). Must be 1 or less.
    * @param max_iterations Maximum iterations to stop the iterative solver.
    * @param matrix_vector_product Function that applies b = A*x
    * @param preconditioner Function that applies z = M_inv * r, where M_inv is the preconditioning matrix. Default: no_preconditioning.
    * @param n_threads Number of threads to use in the vector dot products and additions. If -1, use all available. Default: -1.
    */
    template<typename FLOAT=double>
    Info solve(
        FLOAT* x,
        const FLOAT* b,
        const int size, 
        const FLOAT error_tolerance,
        const int max_iterations, 
        const std::function<void(FLOAT* b, const FLOAT* x, const int size)> matrix_vector_product, 
        const std::function<void(FLOAT* z, const FLOAT* r, const int size)> preconditioner = no_preconditioner<FLOAT>,
        const int n_threads = -1)
    {
        const int n_threads_ = (n_threads == -1) ? omp_get_num_procs() : n_threads;
        Info info;
        if (error_tolerance > 1.0)
        {
            std::cout << "Solving error: Only tolerances less or equal to 1 supported." << std::endl;
            exit(-1);
        }
        if (error_tolerance <= 0.0)
        {
            std::cout << "Solving error: Only tolerances larger than 0 are supported." << std::endl;
            exit(-1);
        }
        
        /* --------------  Precomputation  -------------- */
        FLOAT b_norm_squared = vector_dot(b, b, size, n_threads_);

        /* --------------  Special Case where b is near zero  -------------- */
        if (b_norm_squared < error_tolerance * error_tolerance)
        {
            for (int i = 0; i < size; i++)
            {
                x[i] = 0;
            }

            info.converged = true;
            info.n_iterations = 0;
            info.error = 0;
            return info;
        }

        /* --------------  Allocations  -------------- */
        bsm::avx::avector<FLOAT, 32> r_vector(size);
        FLOAT* r = r_vector.data();
        bsm::avx::avector<FLOAT, 32> z_vector(size);
        FLOAT* z = z_vector.data();
        bsm::avx::avector<FLOAT, 32> p_vector(size);
        FLOAT* p = p_vector.data();
        bsm::avx::avector<FLOAT, 32> Ap_vector(size);
        FLOAT* Ap = Ap_vector.data();

        /* --------------  Initialziation  -------------- */
        // TODO: Branch the initialization If x (initial guess) == zero

        // A*x
        matrix_vector_product(Ap, x, size);

        // r = b - A*x
        vector_add<FLOAT>(r, 1.0, b, -1.0, Ap, size, n_threads_);

        // z = M_inv * r
        preconditioner(z, r, size);

        // p = z
        vector_add<FLOAT>(p, 1.0, z, 0.0, z, size, n_threads_);

        /* --------------  Main loop  -------------- */
        // error = sqrt(r.T * r / (b.T * b))
        FLOAT error = std::sqrt(vector_dot<FLOAT>(r, r, size, n_threads_) / b_norm_squared);
        if (error < error_tolerance) {
            info.converged = true;
            info.n_iterations = 0;
            info.error = error;
            return info;
        }

        for (int it = 1; it < max_iterations; it++) {

            #ifdef BENCHMARK_ENABLED
            double t;
            #endif

            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            matrix_vector_product(Ap, p, size);
            #ifdef BENCHMARK_ENABLED
            info.timing[0] += omp_get_wtime() - t;
            #endif

            // alpha = dot(r, z) / (p.T * A * p)
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            const FLOAT alpha = vector_dot(r, z, size, n_threads_) / vector_dot(p, Ap, size, n_threads_);
            #ifdef BENCHMARK_ENABLED
            info.timing[1] += omp_get_wtime() - t;
            #endif

            // x = x + alpha*p
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            vector_add<FLOAT>(x, 1.0, x, alpha, p, size, n_threads_);
            #ifdef BENCHMARK_ENABLED
            info.timing[2] += omp_get_wtime() - t;
            #endif

            // Need this later
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            const FLOAT r_0_dot_z_0 = vector_dot(r, z, size, n_threads_);
            #ifdef BENCHMARK_ENABLED
            info.timing[3] += omp_get_wtime() - t;
            #endif

            // r = r - alpha * A * p
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            vector_add<FLOAT>(r, 1.0, r, -alpha, Ap, size, n_threads_);
            #ifdef BENCHMARK_ENABLED
            info.timing[4] += omp_get_wtime() - t;
            #endif

            // error = sqrt(r.T * r / (b.T * b))
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            error = std::sqrt(vector_dot<FLOAT>(r, r, size, n_threads_) / b_norm_squared);
            #ifdef BENCHMARK_ENABLED
            info.timing[5] += omp_get_wtime() - t;
            #endif
            if (error < error_tolerance) {
                info.converged = true;
                info.n_iterations = it;
                info.error = error;
                return info;
            }

            // z = M_inv * r
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            preconditioner(z, r, size);
            #ifdef BENCHMARK_ENABLED
            info.timing[6] += omp_get_wtime() - t;
            #endif

            // beta = (r_1.T * z_1) / (r_0.T * z_0)
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            const FLOAT beta = vector_dot<FLOAT>(r, z, size, n_threads_) / r_0_dot_z_0;
            #ifdef BENCHMARK_ENABLED
            info.timing[7] += omp_get_wtime() - t;
            #endif

            // p = z + beta * p
            #ifdef BENCHMARK_ENABLED
            t = omp_get_wtime();
            #endif
            vector_add<FLOAT>(p, 1.0, z, beta, p, size, n_threads_);
            #ifdef BENCHMARK_ENABLED
            info.timing[8] += omp_get_wtime() - t;
            #endif
        }

        // If execution reaches this point, it failed converging
        info.converged = false;
        info.error = error;
        info.error_msg = "Maximum number of iterations";
        info.n_iterations = max_iterations;
        return info;
    }
}

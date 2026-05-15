#pragma once
#include <iostream>

#include "../macros.h"

namespace symx::detail
{
    // ---- 8x8 float transpose (AVX2), with scalar fallback ----
    SYMX_FORCE_INLINE void transpose8x8_ps_kernel(
        const float* SYMX_RESTRICT r0, const float* SYMX_RESTRICT r1,
        const float* SYMX_RESTRICT r2, const float* SYMX_RESTRICT r3,
        const float* SYMX_RESTRICT r4, const float* SYMX_RESTRICT r5,
        const float* SYMX_RESTRICT r6, const float* SYMX_RESTRICT r7,
        float* SYMX_RESTRICT c0, float* SYMX_RESTRICT c1, float* SYMX_RESTRICT c2, float* SYMX_RESTRICT c3,
        float* SYMX_RESTRICT c4, float* SYMX_RESTRICT c5, float* SYMX_RESTRICT c6, float* SYMX_RESTRICT c7)
    {
    #if defined(SYMX_ENABLE_AVX2) && defined(SYMX_ENABLE_AVX2_SHUFFLES)
        __m256 R0=_mm256_loadu_ps(r0), R1=_mm256_loadu_ps(r1);
        __m256 R2=_mm256_loadu_ps(r2), R3=_mm256_loadu_ps(r3);
        __m256 R4=_mm256_loadu_ps(r4), R5=_mm256_loadu_ps(r5);
        __m256 R6=_mm256_loadu_ps(r6), R7=_mm256_loadu_ps(r7);

        __m256 T0=_mm256_unpacklo_ps(R0,R1), T1=_mm256_unpackhi_ps(R0,R1);
        __m256 T2=_mm256_unpacklo_ps(R2,R3), T3=_mm256_unpackhi_ps(R2,R3);
        __m256 T4=_mm256_unpacklo_ps(R4,R5), T5=_mm256_unpackhi_ps(R4,R5);
        __m256 T6=_mm256_unpacklo_ps(R6,R7), T7=_mm256_unpackhi_ps(R6,R7);

        __m256 U0=_mm256_shuffle_ps(T0,T2,0x44), U1=_mm256_shuffle_ps(T0,T2,0xEE);
        __m256 U2=_mm256_shuffle_ps(T1,T3,0x44), U3=_mm256_shuffle_ps(T1,T3,0xEE);
        __m256 U4=_mm256_shuffle_ps(T4,T6,0x44), U5=_mm256_shuffle_ps(T4,T6,0xEE);
        __m256 U6=_mm256_shuffle_ps(T5,T7,0x44), U7=_mm256_shuffle_ps(T5,T7,0xEE);

        __m256 C0=_mm256_permute2f128_ps(U0,U4,0x20);
        __m256 C1=_mm256_permute2f128_ps(U1,U5,0x20);
        __m256 C2=_mm256_permute2f128_ps(U2,U6,0x20);
        __m256 C3=_mm256_permute2f128_ps(U3,U7,0x20);
        __m256 C4=_mm256_permute2f128_ps(U0,U4,0x31);
        __m256 C5=_mm256_permute2f128_ps(U1,U5,0x31);
        __m256 C6=_mm256_permute2f128_ps(U2,U6,0x31);
        __m256 C7=_mm256_permute2f128_ps(U3,U7,0x31);

        _mm256_storeu_ps(c0,C0); _mm256_storeu_ps(c1,C1);
        _mm256_storeu_ps(c2,C2); _mm256_storeu_ps(c3,C3);
        _mm256_storeu_ps(c4,C4); _mm256_storeu_ps(c5,C5);
        _mm256_storeu_ps(c6,C6); _mm256_storeu_ps(c7,C7);
    #else
        for (int j=0;j<8;++j) {
            c0[j]=r0[j]; c1[j]=r1[j]; c2[j]=r2[j]; c3[j]=r3[j];
            c4[j]=r4[j]; c5[j]=r5[j]; c6[j]=r6[j]; c7[j]=r7[j];
        }
    #endif
    }

    // ---- 4x4 double transpose (AVX2), with scalar fallback ----
    SYMX_FORCE_INLINE void transpose4x4_pd_kernel(
        const double* SYMX_RESTRICT r0,
        const double* SYMX_RESTRICT r1,
        const double* SYMX_RESTRICT r2,
        const double* SYMX_RESTRICT r3,
        double* SYMX_RESTRICT c0,
        double* SYMX_RESTRICT c1,
        double* SYMX_RESTRICT c2,
        double* SYMX_RESTRICT c3)
    {
    #if defined(SYMX_ENABLE_AVX2) && defined(SYMX_ENABLE_AVX2_SHUFFLES)

        // Load rows
        __m256d a = _mm256_loadu_pd(r0); // [a0 a1 a2 a3]
        __m256d b = _mm256_loadu_pd(r1); // [b0 b1 b2 b3]
        __m256d c = _mm256_loadu_pd(r2); // [c0 c1 c2 c3]
        __m256d d = _mm256_loadu_pd(r3); // [d0 d1 d2 d3]

        // Interleave within lanes
        __m256d ab_lo = _mm256_unpacklo_pd(a, b); // a0 b0 a2 b2
        __m256d ab_hi = _mm256_unpackhi_pd(a, b); // a1 b1 a3 b3
        __m256d cd_lo = _mm256_unpacklo_pd(c, d); // c0 d0 c2 d2
        __m256d cd_hi = _mm256_unpackhi_pd(c, d); // c1 d1 c3 d3

        // Stitch across 128b lanes
        __m256d r0v = _mm256_permute2f128_pd(ab_lo, cd_lo, 0x20); // a0 b0 c0 d0
        __m256d r1v = _mm256_permute2f128_pd(ab_hi, cd_hi, 0x20); // a1 b1 c1 d1
        __m256d r2v = _mm256_permute2f128_pd(ab_lo, cd_lo, 0x31); // a2 b2 c2 d2
        __m256d r3v = _mm256_permute2f128_pd(ab_hi, cd_hi, 0x31); // a3 b3 c3 d3

        _mm256_storeu_pd(c0, r0v);
        _mm256_storeu_pd(c1, r1v);
        _mm256_storeu_pd(c2, r2v);
        _mm256_storeu_pd(c3, r3v);
    #else
        // Scalar fallback (kept tiny; you said you don't need SSE)
        c0[0]=r0[0]; c0[1]=r1[0]; c0[2]=r2[0]; c0[3]=r3[0];
        c1[0]=r0[1]; c1[1]=r1[1]; c1[2]=r2[1]; c1[3]=r3[1];
        c2[0]=r0[2]; c2[1]=r1[2]; c2[2]=r2[2]; c2[3]=r3[2];
        c3[0]=r0[3]; c3[1]=r1[3]; c3[2]=r2[3]; c3[3]=r3[3];
    #endif
    }
    // ========================= AoS <-> SoA wrappers ============================
    //
    // Layouts:
    //  - Float (B=8): AoS = [e0:E][e1:E]...[e7:E], SoA = [c0:8][c1:8]...[cE-1:8]
    //  - Double (B=4): AoS = [e0:E][e1:E][e2:E][e3:E], SoA = [c0:4]...[cE-1:4]
    // We tile E by 8 (float) / 4 (double). Caller ensures E is padded accordingly.

    // Float, B=8
    SYMX_FORCE_INLINE void aos_to_soa_f32(
        const float* SYMX_RESTRICT aos,
        float* SYMX_RESTRICT soa,
        std::size_t E)
    {
        SYMX_ASSUME((E & 7u) == 0);
        const float* SYMX_RESTRICT e0=aos+0*E, * SYMX_RESTRICT e1=aos+1*E;
        const float* SYMX_RESTRICT e2=aos+2*E, * SYMX_RESTRICT e3=aos+3*E;
        const float* SYMX_RESTRICT e4=aos+4*E, * SYMX_RESTRICT e5=aos+5*E;
        const float* SYMX_RESTRICT e6=aos+6*E, * SYMX_RESTRICT e7=aos+7*E;

        for (std::size_t k=0;k<E;k+=8) {
            float* SYMX_RESTRICT c0=soa+(k+0)*8, * SYMX_RESTRICT c1=soa+(k+1)*8;
            float* SYMX_RESTRICT c2=soa+(k+2)*8, * SYMX_RESTRICT c3=soa+(k+3)*8;
            float* SYMX_RESTRICT c4=soa+(k+4)*8, * SYMX_RESTRICT c5=soa+(k+5)*8;
            float* SYMX_RESTRICT c6=soa+(k+6)*8, * SYMX_RESTRICT c7=soa+(k+7)*8;

            transpose8x8_ps_kernel(e0+k,e1+k,e2+k,e3+k,e4+k,e5+k,e6+k,e7+k,
                                c0,c1,c2,c3,c4,c5,c6,c7);
        }
    }

    SYMX_FORCE_INLINE void soa_to_aos_f32(
        const float* SYMX_RESTRICT soa,
        float* SYMX_RESTRICT aos,
        std::size_t E)
    {
        SYMX_ASSUME((E & 7u) == 0);
        float* SYMX_RESTRICT e0=aos+0*E, * SYMX_RESTRICT e1=aos+1*E;
        float* SYMX_RESTRICT e2=aos+2*E, * SYMX_RESTRICT e3=aos+3*E;
        float* SYMX_RESTRICT e4=aos+4*E, * SYMX_RESTRICT e5=aos+5*E;
        float* SYMX_RESTRICT e6=aos+6*E, * SYMX_RESTRICT e7=aos+7*E;

        for (std::size_t k=0;k<E;k+=8) {
            const float* SYMX_RESTRICT c0=soa+(k+0)*8, * SYMX_RESTRICT c1=soa+(k+1)*8;
            const float* SYMX_RESTRICT c2=soa+(k+2)*8, * SYMX_RESTRICT c3=soa+(k+3)*8;
            const float* SYMX_RESTRICT c4=soa+(k+4)*8, * SYMX_RESTRICT c5=soa+(k+5)*8;
            const float* SYMX_RESTRICT c6=soa+(k+6)*8, * SYMX_RESTRICT c7=soa+(k+7)*8;

            transpose8x8_ps_kernel(c0,c1,c2,c3,c4,c5,c6,c7,
                                e0+k,e1+k,e2+k,e3+k,e4+k,e5+k,e6+k,e7+k);
        }
    }


    // Double, B=4
    SYMX_FORCE_INLINE void aos_to_soa_f64(
        const double* SYMX_RESTRICT aos,
        double* SYMX_RESTRICT soa,
        std::size_t E)
    {
        SYMX_ASSUME((E & 3u) == 0); // multiple of 4

        const double* SYMX_RESTRICT e0 = aos + 0*E;
        const double* SYMX_RESTRICT e1 = aos + 1*E;
        const double* SYMX_RESTRICT e2 = aos + 2*E;
        const double* SYMX_RESTRICT e3 = aos + 3*E;

        for (std::size_t k = 0; k < E; k += 4) {
            double* SYMX_RESTRICT c0 = soa + (k+0)*4;
            double* SYMX_RESTRICT c1 = soa + (k+1)*4;
            double* SYMX_RESTRICT c2 = soa + (k+2)*4;
            double* SYMX_RESTRICT c3 = soa + (k+3)*4;
            transpose4x4_pd_kernel(e0 + k, e1 + k, e2 + k, e3 + k,
                                c0, c1, c2, c3);
        }
    }

    SYMX_FORCE_INLINE void soa_to_aos_f64(
        const double* SYMX_RESTRICT soa,
        double* SYMX_RESTRICT aos,
        std::size_t E)
    {
        SYMX_ASSUME((E & 3u) == 0); // multiple of 4

        double* SYMX_RESTRICT e0 = aos + 0*E;
        double* SYMX_RESTRICT e1 = aos + 1*E;
        double* SYMX_RESTRICT e2 = aos + 2*E;
        double* SYMX_RESTRICT e3 = aos + 3*E;

        for (std::size_t k = 0; k < E; k += 4) {
            const double* SYMX_RESTRICT c0 = soa + (k+0)*4;
            const double* SYMX_RESTRICT c1 = soa + (k+1)*4;
            const double* SYMX_RESTRICT c2 = soa + (k+2)*4;
            const double* SYMX_RESTRICT c3 = soa + (k+3)*4;
            transpose4x4_pd_kernel(c0, c1, c2, c3,
                                e0 + k, e1 + k, e2 + k, e3 + k);
        }
    }

    // Scalar only
    // ================= f64, B = 4 =================
    SYMX_FORCE_INLINE void aos_to_soa_f64_scalar(
        const double* SYMX_RESTRICT aos,
        double* SYMX_RESTRICT soa,
        std::size_t E)
    {
        SYMX_ASSUME((E & 3u) == 0);
        const double* SYMX_RESTRICT e0 = aos + 0*E;
        const double* SYMX_RESTRICT e1 = aos + 1*E;
        const double* SYMX_RESTRICT e2 = aos + 2*E;
        const double* SYMX_RESTRICT e3 = aos + 3*E;

        // Write destination contiguously: [c_k:4]
        for (std::size_t k = 0; k < E; ++k) {
            double* SYMX_RESTRICT ck = soa + k*4;
            const double a = e0[k];
            const double b = e1[k];
            const double c = e2[k];
            const double d = e3[k];
            ck[0] = a; ck[1] = b; ck[2] = c; ck[3] = d;
        }
    }

    SYMX_FORCE_INLINE void soa_to_aos_f64_scalar(
        const double* SYMX_RESTRICT soa,
        double* SYMX_RESTRICT aos,
        std::size_t E)
    {
        SYMX_ASSUME((E & 3u) == 0);
        double* SYMX_RESTRICT e0 = aos + 0*E;
        double* SYMX_RESTRICT e1 = aos + 1*E;
        double* SYMX_RESTRICT e2 = aos + 2*E;
        double* SYMX_RESTRICT e3 = aos + 3*E;

        for (std::size_t k = 0; k < E; ++k) {
            const double* SYMX_RESTRICT ck = soa + k*4;
            const double a = ck[0], b = ck[1], c = ck[2], d = ck[3];
            e0[k] = a; e1[k] = b; e2[k] = c; e3[k] = d;
        }
    }

    // ================= f32, B = 8 =================
    SYMX_FORCE_INLINE void aos_to_soa_f32_scalar(
        const float* SYMX_RESTRICT aos,
        float* SYMX_RESTRICT soa,
        std::size_t E)
    {
        SYMX_ASSUME((E & 7u) == 0);
        const float* SYMX_RESTRICT e0 = aos + 0*E;
        const float* SYMX_RESTRICT e1 = aos + 1*E;
        const float* SYMX_RESTRICT e2 = aos + 2*E;
        const float* SYMX_RESTRICT e3 = aos + 3*E;
        const float* SYMX_RESTRICT e4 = aos + 4*E;
        const float* SYMX_RESTRICT e5 = aos + 5*E;
        const float* SYMX_RESTRICT e6 = aos + 6*E;
        const float* SYMX_RESTRICT e7 = aos + 7*E;

        for (std::size_t k = 0; k < E; ++k) {
            float* SYMX_RESTRICT ck = soa + k*8;
            const float a=e0[k], b=e1[k], c=e2[k], d=e3[k];
            const float e=e4[k], f=e5[k], g=e6[k], h=e7[k];
            ck[0]=a; ck[1]=b; ck[2]=c; ck[3]=d;
            ck[4]=e; ck[5]=f; ck[6]=g; ck[7]=h;
        }
    }

    SYMX_FORCE_INLINE void soa_to_aos_f32_scalar(
        const float* SYMX_RESTRICT soa,
        float* SYMX_RESTRICT aos,
        std::size_t E)
    {
        SYMX_ASSUME((E & 7u) == 0);
        float* SYMX_RESTRICT e0 = aos + 0*E;
        float* SYMX_RESTRICT e1 = aos + 1*E;
        float* SYMX_RESTRICT e2 = aos + 2*E;
        float* SYMX_RESTRICT e3 = aos + 3*E;
        float* SYMX_RESTRICT e4 = aos + 4*E;
        float* SYMX_RESTRICT e5 = aos + 5*E;
        float* SYMX_RESTRICT e6 = aos + 6*E;
        float* SYMX_RESTRICT e7 = aos + 7*E;

        for (std::size_t k = 0; k < E; ++k) {
            const float* SYMX_RESTRICT ck = soa + k*8;
            const float a=ck[0], b=ck[1], c=ck[2], d=ck[3];
            const float e=ck[4], f=ck[5], g=ck[6], h=ck[7];
            e0[k]=a; e1[k]=b; e2[k]=c; e3[k]=d;
            e4[k]=e; e5[k]=f; e6[k]=g; e7[k]=h;
        }
    }
}

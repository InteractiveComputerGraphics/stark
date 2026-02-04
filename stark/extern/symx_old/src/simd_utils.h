#pragma once
#include <type_traits>
#ifdef SYMX_ENABLE_AVX
#include <immintrin.h>
#endif


namespace symx
{
#ifdef SYMX_ENABLE_AVX
	// SIMD types
	using SIMD2d  = __m128d;
	using SIMD4f  = __m128;
	using SIMD4d  = __m256d;
	using SIMD8f  = __m256;
	using SIMD8d  = __m512d;
	using SIMD16f = __m512;
#endif

	// Underlying type of SIMD types
	template<typename T>
	struct SIMD_to_Underlying {
		//static_assert(false, "Type not supported in SIMD_to_Underlying.");
		using Underlying = void;
	};

	template<> struct SIMD_to_Underlying<double> { using Underlying = double; };
	template<> struct SIMD_to_Underlying<float> { using Underlying = float; };
#ifdef SYMX_ENABLE_AVX
	template<> struct SIMD_to_Underlying<__m128d> { using Underlying = double; };
	template<> struct SIMD_to_Underlying<__m128> { using Underlying = float; };
	template<> struct SIMD_to_Underlying<__m256d> { using Underlying = double; };
	template<> struct SIMD_to_Underlying<__m256> { using Underlying = float; };
	template<> struct SIMD_to_Underlying<__m512d> { using Underlying = double; };
	template<> struct SIMD_to_Underlying<__m512> { using Underlying = float; };
#endif

	template<typename T>
	using UNDERLYING_TYPE = typename SIMD_to_Underlying<T>::Underlying;


	// SIMD number of items per line
	template<typename FLOAT>
	constexpr int get_n_items_in_simd()
	{
		if constexpr (std::is_same_v<FLOAT, float> || std::is_same_v<FLOAT, double>) {
			return 1;
		}
#ifdef SYMX_ENABLE_AVX
		else if constexpr (std::is_same_v<FLOAT, __m128d>) {
			return 2;
		}
		else if constexpr (std::is_same_v<FLOAT, __m128> || std::is_same_v<FLOAT, __m256d>) {
			return 4;
		}
		else if constexpr (std::is_same_v<FLOAT, __m256> || std::is_same_v<FLOAT, __m512d>) {
			return 8;
		}
		else if constexpr (std::is_same_v<FLOAT, __m512>) {
			return 16;
		}
#endif
	}

	template<typename FLOAT>
	std::string get_float_type_as_string()
	{
		if constexpr (std::is_same_v<FLOAT, float>) {
			return "float";
		}
		else if constexpr (std::is_same_v<FLOAT, double>) {
			return "double";
		}
#ifdef SYMX_ENABLE_AVX
		else if constexpr (std::is_same_v<FLOAT, SIMD2d>) {
			return "SIMD2d";
		}
		else if constexpr (std::is_same_v<FLOAT, SIMD4f>) {
			return "SIMD4f";
		}
		else if constexpr (std::is_same_v<FLOAT, SIMD4d>) {
			return "SIMD4d";
		}
		else if constexpr (std::is_same_v<FLOAT, SIMD8f>) {
			return "SIMD8f";
		}
		else if constexpr (std::is_same_v<FLOAT, SIMD8d>) {
			return "SIMD8d";
		}
		else if constexpr (std::is_same_v<FLOAT, SIMD16f>) {
			return "SIMD16f";
		}
#endif
	}
}

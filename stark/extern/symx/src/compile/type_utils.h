#pragma once
#include <type_traits>
#ifdef SYMX_ENABLE_AVX2
#include <immintrin.h>
#endif

#include <string>
#include <iostream>


namespace symx
{
#ifdef SYMX_ENABLE_AVX2
	// SIMD types
	using SIMD4d  = __m256d;
	using SIMD8f  = __m256;
#endif

	// Underlying type of SIMD types
	template<typename T>
	struct SIMD_to_Underlying {
		//static_assert(false, "Type not supported in SIMD_to_Underlying.");
		using Underlying = void;
	};

	template<> struct SIMD_to_Underlying<double> { using Underlying = double; };
	template<> struct SIMD_to_Underlying<float> { using Underlying = float; };
#ifdef SYMX_ENABLE_AVX2
	template<> struct SIMD_to_Underlying<__m256d> { using Underlying = double; };
	template<> struct SIMD_to_Underlying<__m256> { using Underlying = float; };
#endif

	template<typename T>
	using UNDERLYING_TYPE = typename SIMD_to_Underlying<T>::Underlying;

	enum class FloatType
	{
		Double, Float
#ifdef SYMX_ENABLE_AVX2
		, SIMD4d, SIMD8f
#endif
	};

	inline FloatType get_underlying_type(const FloatType& float_type)
	{
		switch (float_type) {
			case FloatType::Double: return FloatType::Double;
			case FloatType::Float: return FloatType::Float;
#ifdef SYMX_ENABLE_AVX2
			case FloatType::SIMD4d: return FloatType::Double;
			case FloatType::SIMD8f: return FloatType::Float;
#endif
		}
		return FloatType::Double; // Default fallback
	}

	// SIMD number of items per line
	template<typename FLOAT>
	constexpr int get_n_items_in_simd()
	{
		if constexpr (std::is_same_v<FLOAT, float> || std::is_same_v<FLOAT, double>) {
			return 1;
		}
#ifdef SYMX_ENABLE_AVX2
		else if constexpr (std::is_same_v<FLOAT, __m256d>) {
			return 4;
		}
		else if constexpr (std::is_same_v<FLOAT, __m256>) {
			return 8;
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
#ifdef SYMX_ENABLE_AVX2
		else if constexpr (std::is_same_v<FLOAT, SIMD4d>) {
			return "SIMD4d";
		}
		else if constexpr (std::is_same_v<FLOAT, SIMD8f>) {
			return "SIMD8f";
		}
#endif
	}

	inline std::string get_float_type_as_string(const FloatType& float_type)
	{
		switch (float_type)
		{
		case symx::FloatType::Double:
			return "double";
		case symx::FloatType::Float:
			return "float";
#ifdef SYMX_ENABLE_AVX2
		case symx::FloatType::SIMD4d:
			return "__m256d";
		case symx::FloatType::SIMD8f:
			return "__m256";
#endif
		default:
			std::cout << "symx error: Float type not supported." << std::endl;
			exit(-1);
		}
	}

	template<typename FLOAT>
	inline symx::FloatType get_float_type_as_enum()
	{
		if constexpr (std::is_same_v<FLOAT, double>) {
			return FloatType::Double;
		}
		else if constexpr (std::is_same_v<FLOAT, float>) {
			return FloatType::Float;
		}
#ifdef SYMX_ENABLE_AVX2
		else if constexpr (std::is_same_v<FLOAT, __m256d>) {
			return FloatType::SIMD4d;
		}
		else if constexpr (std::is_same_v<FLOAT, __m256>) {
			return FloatType::SIMD8f;
		}
#endif
		else {
			std::cout << "symx error: Float type not supported." << std::endl;
			exit(-1);
		}
	}

}

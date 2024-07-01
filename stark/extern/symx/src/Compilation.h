#pragma once
#include <fstream>
#include <sstream>
#ifdef SYMX_ENABLE_AVX
#include <immintrin.h>
#endif

#include "Sequence.h"

// Compilation includes
#ifdef _MSC_VER
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif


namespace symx
{
	extern std::string compiler_command;
	enum class OpType
	{
		Double, Float
#ifdef SYMX_ENABLE_AVX
		, SIMD2d, SIMD4f, SIMD4d, SIMD8f, SIMD8d, SIMD16f
#endif
	};
	class Compilation
	{
	private:
		template<typename FLOAT>
		using fptr = void(*)(FLOAT*, FLOAT*);

	public:
#ifdef _MSC_VER
		HINSTANCE lib = nullptr;
#else
		void* lib = nullptr;
#endif
		/* Fields */
		void* compiled_f = nullptr;
		int n_inputs = -1;
		int n_outputs = -1;
		OpType compiled_type = OpType::Double;
		bool was_cached = false;
		double runtime_codegen = 0.0;
		double runtime_compilation = 0.0;

		/* Methods */
		Compilation() = default;
		~Compilation();
		Compilation(const Compilation&) = delete; // Copying makes unclear who owns this->lib

		bool load_if_cached(std::string name, std::string folder, std::string id, OpType op_type);
		void compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", OpType op_type = OpType::Double, bool suppress_compiler_output = true);
		void try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", OpType op_type = OpType::Double, bool suppress_compiler_output = true);
		bool is_valid() const;

		template<typename FLOAT>
		bool load_if_cached(std::string name, std::string folder, std::string id);
		template<typename FLOAT>
		void compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", bool suppress_compiler_output = true);
		template<typename FLOAT>
		void try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", bool suppress_compiler_output = true);

		template<typename FLOAT>
		fptr<FLOAT> get_f();

	private:
		std::string _get_op_type_string(OpType op_type);
		void _write_shared_object_code(Sequence& eval, std::string name, std::string folder, std::string checksum, OpType op_type);
		void _add_instructions_scalar(std::string& code, Sequence& eval, std::string type);
		void _add_instructions_simd(std::string& code, Sequence& eval, std::string type);
		void _add_core_simd_functions(std::string& code, OpType op_type);
	};

	template<typename FLOAT>
	inline symx::OpType get_type_as_enum()
	{
		if constexpr (std::is_same_v<FLOAT, double>) {
			return OpType::Double;
		}
		else if constexpr (std::is_same_v<FLOAT, float>) {
			return OpType::Float;
		}
#ifdef SYMX_ENABLE_AVX
		else if constexpr (std::is_same_v<FLOAT, __m128d>) {
			return OpType::SIMD2d;
		}
		else if constexpr (std::is_same_v<FLOAT, __m128>) {
			return OpType::SIMD4f;
		}
		else if constexpr (std::is_same_v<FLOAT, __m256d>) {
			return OpType::SIMD4d;
		}
		else if constexpr (std::is_same_v<FLOAT, __m256>) {
			return OpType::SIMD8f;
		}
		else if constexpr (std::is_same_v<FLOAT, __m512d>) {
			return OpType::SIMD8d;
		}
		else if constexpr (std::is_same_v<FLOAT, __m512>) {
			return OpType::SIMD16f;
		}
#endif
		else {
			std::cout << "symx error: Compilation type not supported." << std::endl;
			exit(-1);
		}
	}

	template<typename FLOAT>
	inline bool Compilation::load_if_cached(std::string name, std::string folder, std::string id)
	{
		return this->load_if_cached(name, folder, id, get_type_as_enum<FLOAT>());
	}
	template<typename FLOAT>
	inline void Compilation::compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, bool suppress_compiler_output)
	{
		this->compile(expr, name, folder, id, get_type_as_enum<FLOAT>(), suppress_compiler_output);
	}
	template<typename FLOAT>
	inline void Compilation::try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, bool suppress_compiler_output)
	{
		this->try_load_otherwise_compile(expr, name, folder, id, get_type_as_enum<FLOAT>(), suppress_compiler_output);
	}
	template<typename FLOAT>
	inline symx::Compilation::fptr<FLOAT> Compilation::get_f()
	{
		return reinterpret_cast<fptr<FLOAT>>(this->compiled_f);
	}
}

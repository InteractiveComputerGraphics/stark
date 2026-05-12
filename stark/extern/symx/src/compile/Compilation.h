#pragma once
#ifdef SYMX_ENABLE_AVX2
#include <immintrin.h>
#endif

#include "type_utils.h"
#include "Sequence.h"
#include "static_methods.h"
#include "compiler_utils.h"

// Compilation includes
#ifdef _MSC_VER
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif


namespace symx
{
	class Compilation
	{
	private:
		/* Definitions */
		template<typename FLOAT>
		using fptr = void(*)(FLOAT*, FLOAT*);

		/* Fields */
		#ifdef _MSC_VER
			HINSTANCE lib = nullptr;
		#else
			void* lib = nullptr;
		#endif
		void* compiled_f = nullptr;
		int n_inputs = -1;
		int n_outputs = -1;
		FloatType compiled_type = FloatType::Double;
		bool cached = false;
		double runtime_codegen = 0.0;
		double runtime_compilation = 0.0;
		int n_bytes_symbols = 0;
		bool override_load = false; // Verifies and loads freshly compiled code, ignoring global force load state
		
	public:
		/* Definitions */
		struct Info {
			int n_inputs = -1;
			int n_outputs = -1;
			FloatType compiled_type = FloatType::Double;
			bool was_cached = false;
			double runtime_codegen = 0.0;
			double runtime_compilation = 0.0;
			int n_bytes_symbols = 0;

			void print() const;
		};

		/* Methods */
		Compilation() = default;
		~Compilation();
		Compilation(const Compilation&) = delete; // Copying makes unclear who owns this->lib

		bool load_if_cached(const std::string& name, const std::string& folder, const std::string& cache_id, FloatType float_type);
		void compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id = "", FloatType float_type = FloatType::Double);
		void try_load_otherwise_compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id = "", FloatType float_type = FloatType::Double);
		bool is_valid() const;

		int get_n_inputs() const;
		int get_n_outputs() const;
		FloatType get_compiled_type() const;
		bool was_cached() const;
		Info get_info() const;

		template<typename FLOAT>
		bool load_if_cached(const std::string& name, const std::string& folder, const std::string& cache_id);
		template<typename FLOAT>
		void compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id = "");
		template<typename FLOAT>
		void try_load_otherwise_compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id = "");

		template<typename FLOAT>
		fptr<FLOAT> get_f();

	private:
		void _write_shared_object_code(Sequence& eval, const std::string& name, const std::string& folder, const std::string& cache_id, FloatType float_type);
		void _add_instructions_scalar(std::string& code, Sequence& eval, const std::string& type);
		void _add_instructions_simd(std::string& code, Sequence& eval, const std::string& type);
		void _add_core_simd_functions(std::string& code, FloatType float_type);
		

	};

	template<typename FLOAT>
	inline bool Compilation::load_if_cached(const std::string& name, const std::string& folder, const std::string& cache_id)
	{
		return this->load_if_cached(name, folder, cache_id, get_float_type_as_enum<FLOAT>());
	}
	template<typename FLOAT>
	inline void Compilation::compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id)
	{
		this->compile(expr, name, folder, cache_id, get_float_type_as_enum<FLOAT>());
	}
	template<typename FLOAT>
	inline void Compilation::try_load_otherwise_compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id)
	{
		this->try_load_otherwise_compile(expr, name, folder, cache_id, get_float_type_as_enum<FLOAT>());
	}
	template<typename FLOAT>
	inline symx::Compilation::fptr<FLOAT> Compilation::get_f()
	{
		return reinterpret_cast<fptr<FLOAT>>(this->compiled_f);
	}
}

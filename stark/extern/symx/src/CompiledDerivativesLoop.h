#pragma once
#include "CompiledInLoop.h"
#include "diff.h"


namespace symx
{
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT = INPUT_FLOAT, typename OUTPUT_FLOAT = INPUT_FLOAT>
	class CompiledDerivativesLoop
	{
	public:
		using CompiledInLoop_ = CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>;

		/* Fields */
		CompiledInLoop_ E;
		CompiledInLoop_ dE;
		CompiledInLoop_ hE;
		int n_variables = -1;
		int block_size = -1;
		int n_blocks = -1;
		bool was_cached = false;
		double runtime_differentiation = 0.0;
		double runtime_codegen = 0.0;
		double runtime_compilation = 0.0;

		/* Methods */
		CompiledDerivativesLoop() = default;
		~CompiledDerivativesLoop() = default;
		CompiledDerivativesLoop(const CompiledDerivativesLoop&) = delete; // Copying makes unclear who owns the DLL
		CompiledDerivativesLoop(std::string name, std::string folder, Scalar expr, std::vector<Scalar>& variables, const int block_size, const bool suppress_compiler_output = true);
		void init(std::string name, std::string folder, Scalar expr, std::vector<Scalar>& variables, const int block_size, const bool force_compilation = false, const bool suppress_compiler_output = false);

		// Set connectivity
		Element set_connectivity(std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element);
		Element set_connectivity(const std::vector<int32_t>& arr, const int32_t n_items_per_element);
		template<std::size_t N>
		Element set_connectivity(const std::vector<std::array<int32_t, N>>& arr);

		// Update connectivity
		void update_connectivity(std::function<const int32_t* ()> data, std::function<int32_t()> n_elements);
		void update_connectivity(const std::vector<int32_t>& arr);
		template<std::size_t N>
		void update_connectivity(const std::vector<std::array<int32_t, N>>& arr);

		// Set fixed Scalar
		void set_scalar(const Scalar& scalar, std::function<const INPUT_FLOAT* ()> data);
		void set_scalar(const Scalar& scalar, const INPUT_FLOAT& value);

		// Set fixed Vector
		void set_vector(const Vector& vector, std::function<const INPUT_FLOAT* ()> data);
		template<typename ARRAY>
		void set_vector(const Vector& vector, ARRAY& arr);

		// Set Scalars
		void set_scalars(const std::vector<Scalar>& scalars, const std::vector<Index>& indices, std::function<const INPUT_FLOAT* ()> data, std::function<int32_t()> size);
		template<typename ARRAY>
		void set_scalars(const std::vector<Scalar>& scalars, const std::vector<Index>& indices, ARRAY& arr);

		// Set Vectors
		void set_vectors(const std::vector<Vector>& vectors, const std::vector<Index>& indices, std::function<const INPUT_FLOAT* ()> data, std::function<int32_t()> size);
		template<typename ARRAY>
		void set_vectors(const std::vector<Vector>& vectors, const std::vector<Index>& indices, ARRAY& arr);

		// Set Matrices
		void set_matrices(const std::vector<Matrix>& matrices, const std::vector<Index>& indices, std::function<const INPUT_FLOAT* ()> data, std::function<int32_t()> size);
		template<typename ARRAY>
		void set_matrices(const std::vector<Matrix>& matrices, const std::vector<Index>& indices, ARRAY& arr);

		// Set summation vector
		void set_summation_vector(const Vector vector, const std::vector<double>& summation_data, const int summation_stride);

		/*
			CB -> std::function<void(const int32_t* connectivity, const OUTPUT_FLOAT* solution)>
		*/
		template<typename CB>
		void run_E(const int n_threads, CB callback, const bool check_for_NaNs = false);

		/*
			CB -> std::function<void(const int32_t* connectivity, const OUTPUT_FLOAT* solution)>
		*/
		template<typename CB>
		void run_E_grad(const int n_threads, CB callback, const bool check_for_NaNs = false);

		/*
			CB -> std::function<void(const int32_t* connectivity, const OUTPUT_FLOAT* solution)>
		*/
		template<typename CB>
		void run_E_grad_hess(const int n_threads, CB callback, const bool check_for_NaNs = false);
	};


	// ===================================== DEFINITIONS =====================================
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::CompiledDerivativesLoop(std::string name, std::string folder, Scalar expr, std::vector<Scalar>& variables, const int block_size, const bool suppress_compiler_output)
	{
		this->init(name, folder, expr, variables, block_size, suppress_compiler_output);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::init(std::string name, std::string folder, Scalar expr, std::vector<Scalar>& variables, const int block_size, const bool force_compilation, const bool suppress_compiler_output)
	{
		this->block_size = block_size;
		this->n_variables = (int)variables.size();
		this->n_blocks = this->n_variables / this->block_size;

		std::string variables_names;
		for (Scalar& scalar : variables) {
			if (scalar.is_symbol()) {
				variables_names += scalar.get_name();
			}
			else {
				std::cout << "symx error: cannot take derivatives wrt non-symbols. Scalar passed: " << scalar.get_name() << std::endl;
				exit(-1);
			}
		}

		const std::string type_str = get_float_type_as_string<COMPILED_FLOAT>();
		const std::string id_string = expr.get_checksum() + variables_names;

		bool was_everything_cached = false;
		if (!force_compilation) {
			const bool was_E_cached = this->E.load_if_cached(name + "_E_" + type_str, folder, id_string);
			if (was_E_cached) {
				const bool was_dE_cached = this->dE.load_if_cached(name + "_dE_" + type_str, folder, id_string);
				const bool was_hE_cached = this->hE.load_if_cached(name + "_hE_" + type_str, folder, id_string);
				was_everything_cached = was_E_cached && was_dE_cached && was_hE_cached;
			}
		}

		if (!was_everything_cached) {
			
			const double t0 = omp_get_wtime();
			std::vector<Scalar> e = { expr };
			std::vector<Scalar> e_grad = symx::value_gradient(expr, variables);
			std::vector<Scalar> e_grad_hess = symx::value_gradient_hessian(expr, variables);
			const double t1 = omp_get_wtime();
			this->runtime_differentiation = t1 - t0;

			this->E.compile(e, name + "_E_" + type_str, folder, id_string, suppress_compiler_output);
			this->dE.compile(e_grad, name + "_dE_" + type_str, folder, id_string, suppress_compiler_output);
			this->hE.compile(e_grad_hess, name + "_hE_" + type_str, folder, id_string, suppress_compiler_output);
			this->runtime_codegen = this->E.compilation.runtime_codegen + this->dE.compilation.runtime_codegen + this->hE.compilation.runtime_codegen;
			this->runtime_compilation = this->E.compilation.runtime_compilation + this->dE.compilation.runtime_compilation + this->hE.compilation.runtime_compilation;
		}

		this->was_cached = was_everything_cached;
	}
	
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline Element CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_connectivity(std::function<const int32_t*()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element)
	{
		this->E.set_connectivity(data, n_elements, n_items_per_element);
		this->dE.set_connectivity(data, n_elements, n_items_per_element);
		return this->hE.set_connectivity(data, n_elements, n_items_per_element);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline Element CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_connectivity(const std::vector<int32_t>& arr, const int32_t n_items_per_element)
	{
		this->E.set_connectivity(arr, n_items_per_element);
		this->dE.set_connectivity(arr, n_items_per_element);
		return this->hE.set_connectivity(arr, n_items_per_element);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<std::size_t N>
	inline Element CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_connectivity(const std::vector<std::array<int32_t, N>>& arr)
	{
		this->E.set_connectivity(arr);
		this->dE.set_connectivity(arr);
		return this->hE.set_connectivity(arr);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::update_connectivity(std::function<const int32_t*()> data, std::function<int32_t()> n_elements)
	{
		this->E.update_connectivity(data, n_elements);
		this->dE.update_connectivity(data, n_elements);
		this->hE.update_connectivity(data, n_elements);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::update_connectivity(const std::vector<int32_t>& arr)
	{
		this->E.update_connectivity(arr);
		this->dE.update_connectivity(arr);
		this->hE.update_connectivity(arr);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<std::size_t N>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::update_connectivity(const std::vector<std::array<int32_t, N>>& arr)
	{
		this->E.update_connectivity(arr);
		this->dE.update_connectivity(arr);
		this->hE.update_connectivity(arr);
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalar(const Scalar& scalar, std::function<const INPUT_FLOAT* ()> data)
	{
		this->E.set_scalar(scalar, data);
		this->dE.set_scalar(scalar, data);
		this->hE.set_scalar(scalar, data);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalar(const Scalar& scalar, const INPUT_FLOAT& value)
	{
		this->E.set_scalar(scalar, value);
		this->dE.set_scalar(scalar, value);
		this->hE.set_scalar(scalar, value);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vector(const Vector& vector, std::function<const INPUT_FLOAT* ()> data)
	{
		this->E.set_vector(vector, data);
		this->dE.set_vector(vector, data);
		this->hE.set_vector(vector, data);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vector(const Vector& vector, ARRAY& arr)
	{
		this->E.set_vector(vector, arr);
		this->dE.set_vector(vector, arr);
		this->hE.set_vector(vector, arr);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalars(const std::vector<Scalar>& scalars, const std::vector<Index>& indices, std::function<const INPUT_FLOAT* ()> data, std::function<int32_t()> size)
	{
		this->E.set_scalars(scalars, indices, data, size);
		this->dE.set_scalars(scalars, indices, data, size);
		this->hE.set_scalars(scalars, indices, data, size);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalars(const std::vector<Scalar>& scalars, const std::vector<Index>& indices, ARRAY& arr)
	{
		this->E.set_scalars(scalars, indices, arr);
		this->dE.set_scalars(scalars, indices, arr);
		this->hE.set_scalars(scalars, indices, arr);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vectors(const std::vector<Vector>& vectors, const std::vector<Index>& indices, std::function<const INPUT_FLOAT* ()> data, std::function<int32_t()> size)
	{
		this->E.set_vectors(vectors, indices, data, size);
		this->dE.set_vectors(vectors, indices, data, size);
		this->hE.set_vectors(vectors, indices, data, size);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vectors(const std::vector<Vector>& vectors, const std::vector<Index>& indices, ARRAY& arr)
	{
		this->E.set_vectors(vectors, indices, arr);
		this->dE.set_vectors(vectors, indices, arr);
		this->hE.set_vectors(vectors, indices, arr);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_matrices(const std::vector<Matrix>& matrices, const std::vector<Index>& indices, std::function<const INPUT_FLOAT* ()> data, std::function<int32_t()> size)
	{
		this->E.set_matrices(matrices, indices, data, size);
		this->dE.set_matrices(matrices, indices, data, size);
		this->hE.set_matrices(matrices, indices, data, size);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_matrices(const std::vector<Matrix>& matrices, const std::vector<Index>& indices, ARRAY& arr)
	{
		this->E.set_matrices(matrices, indices, arr);
		this->dE.set_matrices(matrices, indices, arr);
		this->hE.set_matrices(matrices, indices, arr);
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_summation_vector(const Vector vector, const std::vector<double>& summation_data, const int summation_stride)
	{
		this->E.set_summation_vector(vector, summation_data, summation_stride);
		this->dE.set_summation_vector(vector, summation_data, summation_stride);
		this->hE.set_summation_vector(vector, summation_data, summation_stride);
	}
	
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename CB>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run_E(const int n_threads, CB callback, const bool check_for_NaNs)
	{
		this->E.run(n_threads, callback, check_for_NaNs);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename CB>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run_E_grad(const int n_threads, CB callback, const bool check_for_NaNs)
	{
		this->dE.run(n_threads, callback, check_for_NaNs);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename CB>
	inline void CompiledDerivativesLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run_E_grad_hess(const int n_threads, CB callback, const bool check_for_NaNs)
	{
		this->hE.run(n_threads, callback, check_for_NaNs);
	}
}
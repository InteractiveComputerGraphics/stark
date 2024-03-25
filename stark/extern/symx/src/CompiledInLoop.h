#pragma once
#include <functional>
#include <array>

#include "BlockedSparseMatrix/ParallelNumber.h"
#include <omp.h>

#include "lambdas.h"
#include "Element.h"
#include "Compilation.h"
#include "simd_utils.h"
#include "AlignmentAllocator.h"


namespace symx
{
	/*
		Represents loop executions of compiled functions.
		The main loop needs a connectivity and pointers and stride data to arrays.

		This class also handles the case of summations per entry in the connectivity 
		array where the data changes across summation iterations is fixed. The typical
		use case is FEM integrators where the compiled function evaluates one integration 
		point and the summation just calls that function for a set of integration points 
		and weights.

	*/
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT = INPUT_FLOAT, typename OUTPUT_FLOAT = INPUT_FLOAT>
	class CompiledInLoop
	{
	private:
		struct SymbolArrayMap
		{
			int32_t connectivity_index = -1;
			int32_t stride = -1;
			int32_t offset = -1;  // Coordinate for vectors
			int32_t current_size = -1;  // for debug
			const INPUT_FLOAT* array_begin = nullptr;
			std::function<const INPUT_FLOAT*()> data = nullptr;
			std::function<int32_t()> size = nullptr;  // array size for debug purposes
		};

	public:
		/* Fields */
		// Compilation and evaluation
		Compilation compilation;
		std::vector<SymbolArrayMap> symbol_array_map;
		std::vector<symx::avx::avector<COMPILED_FLOAT, 64>> thread_input;
		std::vector<symx::avx::avector<COMPILED_FLOAT, 64>> thread_output;
		std::vector<symx::avx::avector<COMPILED_FLOAT, 64>> thread_summation_buffer;
		std::vector<std::vector<OUTPUT_FLOAT>> thread_solution_buffer;
		std::function<void(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity, const OUTPUT_FLOAT* solution)> evaluation_callback = nullptr;

		// Connectivity
		std::function<const int32_t*()> connectivity_data;
		std::function<int32_t()> connectivity_n_elements;
		int32_t connectivity_stride = -1;

		// Summation
		std::vector<int> summation_symbols_idx;
		std::vector<int> non_summation_symbols_idx;
		std::vector<double> summation_data;
		int summation_stride = -1;
		int summation_iterations = -1;

		// Conditional evaluation
		std::function<bool(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity)> conditional_evaluation_callback = nullptr;
		std::vector<uint8_t> active_elements_bool;
		std::vector<int32_t> active_elements;

		// Misc
		bool check_for_NaNs = false;
		std::string name = "";


		/* Methods */
		CompiledInLoop() = default;
		~CompiledInLoop() = default;
		CompiledInLoop(const CompiledInLoop&) = delete; // Copying makes unclear who owns the DLL
		CompiledInLoop(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", bool suppress_compiler_output = true);
		bool load_if_cached(std::string name, std::string folder, std::string id);
		void compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", bool suppress_compiler_output = true);
		void try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", bool suppress_compiler_output = true);
		void set_check_for_NaNs(const bool check);
		void set_evaluation_callback(std::function<void(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity, const OUTPUT_FLOAT* solution)> callback);
		void set_conditional_evaluation_callback(std::function<bool(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity)> callback);
		bool is_valid() const;
		bool was_cached() const;
		int get_n_elements() const;
		int get_connectivity_stride() const;
		const int32_t* get_connectivity_data() const;
		const int32_t* get_connectivity_elem(const int32_t i) const;

		// Set connectivity
		Element set_connectivity(std::function<const int32_t*()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element);
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

		// Run
		void run(
			const int32_t n_threads,
			std::function<void(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity, OUTPUT_FLOAT* solution)> callback,
			std::function<bool(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity)> conditional_evaluation
		);
		void run(
			const int32_t n_threads,
			std::function<void(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity, OUTPUT_FLOAT* solution)> callback
		);
		void run(const int32_t n_threads);

	private:
		void _init();
		void _resize_symbol_array_map(const Scalar& scalar);
	};



	// ===================================== DEFINITIONS =====================================
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::_init()
	{
		static_assert((std::is_same_v<INPUT_FLOAT, float> || std::is_same_v<INPUT_FLOAT, double>), "INPUT_FLOAT must be {float, double}");
		static_assert((
			std::is_same_v<COMPILED_FLOAT, float> ||
			std::is_same_v<COMPILED_FLOAT, double>
#ifdef SYMX_ENABLE_AVX
			||
			std::is_same_v<COMPILED_FLOAT, __m128d> ||
			std::is_same_v<COMPILED_FLOAT, __m128> ||
			std::is_same_v<COMPILED_FLOAT, __m256d> ||
			std::is_same_v<COMPILED_FLOAT, __m256> ||
			std::is_same_v<COMPILED_FLOAT, __m512d> ||
			std::is_same_v<COMPILED_FLOAT, __m512>
#endif
			),
			"COMPILED_FLOAT must be {float, double, __m128d, __m128, __m256d, __m256, __m512d, __m512}");

		this->symbol_array_map.resize(this->compilation.n_inputs);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::_resize_symbol_array_map(const Scalar& scalar)
	{
		this->symbol_array_map.resize(scalar.get_expression_graph()->get_n_symbols());
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::CompiledInLoop(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, bool suppress_compiler_output)
	{
		this->compile(expr, name, folder, id, suppress_compiler_output);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, bool suppress_compiler_output)
	{
		this->name = name;
		this->compilation.template compile<COMPILED_FLOAT>(expr, name, folder, id, suppress_compiler_output);
		this->_init();
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, bool suppress_compiler_output)
	{
		if (!this->load_if_cached(name, folder, id)) {
			this->compile(expr, name, folder, id, suppress_compiler_output);
		}
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline bool CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::load_if_cached(std::string name, std::string folder, std::string id)
	{
		const bool success = this->compilation.template load_if_cached<COMPILED_FLOAT>(name, folder, id);
		if (success) {
			this->_init();
			this->name = name;
		}
		return success;
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline bool CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::is_valid() const
	{
		return this->compilation.is_valid();
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline bool CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::was_cached() const
	{
		return this->compilation.was_cached;
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_check_for_NaNs(const bool check)
	{
		this->check_for_NaNs = check;
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_evaluation_callback(std::function<void(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity, const OUTPUT_FLOAT* solution)> callback)
	{
		this->evaluation_callback = callback;
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_conditional_evaluation_callback(std::function<bool(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity)> callback)
	{
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline int CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::get_n_elements() const
	{
		return this->connectivity_n_elements();
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline int CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::get_connectivity_stride() const
	{
		return this->connectivity_stride;
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline const int32_t* CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::get_connectivity_data() const
	{
		return this->connectivity_data();
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline const int32_t* CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::get_connectivity_elem(const int32_t i) const
	{
		return this->connectivity_data() + this->connectivity_stride*i;
	}

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline Element CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_connectivity(std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element)
	{
		this->connectivity_data = data;
		this->connectivity_n_elements = n_elements;
		this->connectivity_stride = n_items_per_element;
		return Element(n_items_per_element);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline Element CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_connectivity(const std::vector<int32_t>& arr, const int32_t n_items_per_element)
	{
		return this->set_connectivity(l2data_int(arr), l2n_elements_int(arr, n_items_per_element), n_items_per_element);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<std::size_t N>
	inline Element CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_connectivity(const std::vector<std::array<int32_t, N>>& arr)
	{
		return this->set_connectivity(l2data_int(arr), l2n_elements_int(arr), N);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::update_connectivity(std::function<const int32_t* ()> data, std::function<int32_t()> n_elements)
	{
		this->connectivity_data = data;
		this->connectivity_n_elements = n_elements;
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::update_connectivity(const std::vector<int32_t>& arr)
	{
		this->update_connectivity(l2data_int(arr), l2n_elements_int(arr, this->connectivity_stride));
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<std::size_t N>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::update_connectivity(const std::vector<std::array<int32_t, N>>& arr)
	{
		this->update_connectivity(l2data_int(arr), l2n_elements_int(arr));
	}
	

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalar(const Scalar& scalar, const INPUT_FLOAT& value)
	{
		this->set_scalar(scalar, l2data_double(value));
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalar(const Scalar& symbol, std::function<const INPUT_FLOAT*()> data)
	{
		this->_resize_symbol_array_map(symbol);

		SymbolArrayMap map;
		map.data = data;
		map.connectivity_index = 0;
		map.stride = 0;
		map.offset = 0;
		this->symbol_array_map[symbol.get_symbol_idx()] = map;
		this->non_summation_symbols_idx.push_back(symbol.get_symbol_idx());
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalars(const std::vector<Scalar>& scalars, const std::vector<Index>& indices, std::function<const INPUT_FLOAT*()> data, std::function<int32_t()> size)
	{
		if (scalars.size() != indices.size()) {
			std::cout << "symx error: CompiledInLoop.set_scalars() got mistmatched sized scalars and indices vectors for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}
		this->_resize_symbol_array_map(scalars[0]);
		const int32_t n = (int)indices.size();

		for (size_t i = 0; i < n; i++){
			const Scalar& scalar = scalars[i];
			const Index& idx = indices[i];

			SymbolArrayMap map;
			map.data = data;
			map.size = size;
			map.connectivity_index = idx.idx;
			map.stride = 1;
			map.offset = 0;
			this->symbol_array_map[scalar.get_symbol_idx()] = map;
			this->non_summation_symbols_idx.push_back(scalar.get_symbol_idx());
		}
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_scalars(const std::vector<Scalar>& scalars, const std::vector<Index>& indices, ARRAY& arr)
	{
		this->set_scalars(scalars, indices, l2data_double(arr), l2count_double(arr));
	}
	

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vector(const Vector& vector, std::function<const INPUT_FLOAT*()> data)
	{
		this->_resize_symbol_array_map(vector[0]);

		for (int i = 0; i < vector.size(); i++) {
			SymbolArrayMap map;
			map.data = data;
			map.connectivity_index = 0;
			map.stride = 0;
			map.offset = i;
			this->symbol_array_map[vector[i].get_symbol_idx()] = map;
			this->non_summation_symbols_idx.push_back(vector[i].get_symbol_idx());
		}
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vector(const Vector& vector, ARRAY& arr)
	{
		this->set_vector(vector, l2data_double(arr));
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vectors(const std::vector<Vector>& vectors, const std::vector<Index>& indices, std::function<const INPUT_FLOAT*()> data, std::function<int32_t()> size)
	{
		if (vectors.size() != indices.size()) {
			std::cout << "symx error: CompiledInLoop.set_vectors() got mistmatched sized vectors and indices vectors for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}

		this->_resize_symbol_array_map(vectors[0][0]);
		const int32_t n = (int)indices.size();

		for (size_t i = 0; i < n; i++){
			const Vector& vector = vectors[i];
			const Index& idx = indices[i];
			for (int j = 0; j < vector.size(); j++) {
				SymbolArrayMap map;
				map.data = data;
				map.size = size;
				map.connectivity_index = idx.idx;
				map.stride = vector.size();
				map.offset = j;
				this->symbol_array_map[vector[j].get_symbol_idx()] = map;
				this->non_summation_symbols_idx.push_back(vector[j].get_symbol_idx());
			}
		}
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_vectors(const std::vector<Vector>& vectors, const std::vector<Index>& indices, ARRAY& arr)
	{
		this->set_vectors(vectors, indices, l2data_double(arr), l2count_double(arr));
	}
	

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_matrices(const std::vector<Matrix>& matrices, const std::vector<Index>& indices, std::function<const INPUT_FLOAT*()> data, std::function<int32_t()> size)
	{
		if (matrices.size() != indices.size()) {
			std::cout << "symx error: CompiledInLoop.set_matrices() got mistmatched sized vectors and indices vectors for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}

		const int32_t n = (int)indices.size();
		std::vector<Vector> values;
		for (size_t i = 0; i < n; i++) {
			values.push_back(Vector(matrices[i].values()));  // Set the values of the matrix as a vector
		}
		this->set_vectors(values, indices, data, size);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	template<typename ARRAY>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_matrices(const std::vector<Matrix>& matrices, const std::vector<Index>& indices, ARRAY& arr)
	{
		this->set_matrices(matrices, indices, l2data_double(arr), l2count_double(arr));
	}
	

	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::set_summation_vector(const Vector vector, const std::vector<double>& summation_data, const int summation_stride)
	{
		if (this->summation_stride != -1) {
			std::cout << "symx error: CompiledInLoop.set_summation_vector(): Summation vector already set for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}

		if ((int)vector.size() != summation_stride) {
			std::cout << "symx error: CompiledInLoop.set_summation_vector() got mistmatched sized vectors and data stride for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}

		this->summation_stride = summation_stride;
		this->summation_data = summation_data;
		this->summation_iterations = (int)this->summation_data.size() / this->summation_stride;

		if ((int)this->summation_data.size() != this->summation_iterations*this->summation_stride) {
			std::cout << "symx error: CompiledInLoop.set_summation_vector() got non consistent sized vectors and data stride for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}

		this->_resize_symbol_array_map(vector[0]);
		for (int j = 0; j < summation_stride; j++) {
			SymbolArrayMap map;
			map.data = l2data_double(this->summation_data);
			map.size = l2count_double(this->summation_data);
			map.connectivity_index = -1;
			map.stride = summation_stride;
			map.offset = j;
			this->symbol_array_map[vector[j].get_symbol_idx()] = map;
			this->summation_symbols_idx.push_back(vector[j].get_symbol_idx());
		}
	}


	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run(
		const int32_t n_threads, 
		std::function<void(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity, OUTPUT_FLOAT* solution)> callback,
		std::function<bool(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity)> conditional_evaluation)
	{
		using UNDERLYING_FLOAT = UNDERLYING_TYPE<COMPILED_FLOAT>;
		constexpr int N_SIMD = get_n_items_in_simd<COMPILED_FLOAT>();

		auto symbol_number_as_str = [&](const int i) { return std::to_string(i) + "/" + std::to_string((int)this->symbol_array_map.size() - 1); };

		// Update connectivity, pointers and sizes
		const int n_symbols = (int)this->symbol_array_map.size();
		const int32_t n_elements = this->connectivity_n_elements();
		if (n_elements == 0) { return; } // Early exit
		const int32_t* connectivity_data = this->connectivity_data();
		for (int symbol_i = 0; symbol_i < n_symbols; symbol_i++) {
			SymbolArrayMap& map = this->symbol_array_map[symbol_i];
			if (map.data == nullptr) {
				std::cout << "symx error: CompiledInLoop::run() symbol " << symbol_number_as_str(symbol_i) << " without pointer lambda for instance with name \"" + this->name + "\"" << std::endl;
				exit(-1);
			}
			map.array_begin = map.data();

			if (map.stride == 0) {  // Fixed (non-indexed) symbol
				map.current_size = map.offset + 1;
			}
			else {  // Indexed symbol
				if (map.size == nullptr) {
					std::cout << "symx error: CompiledInLoop::run() symbol " << symbol_number_as_str(symbol_i) << " without size lambda for instance with name \"" + this->name + "\"" << std::endl;
					exit(-1);
				}
				map.current_size = map.size();
			}

			if (map.connectivity_index >= this->connectivity_stride) {
				std::cout << "symx error: CompiledInLoop::run() symbol " << symbol_number_as_str(symbol_i) << " with a connectivity index larger than the connectivity stride for instance with name \"" + this->name + "\"" << std::endl;
				exit(-1);
			}
		}

		if (n_threads < 0) {
			std::cout << "symx error: invalid n_threads in CompiledInLoop.run() for instance with name \"" + this->name + "\"" << std::endl;
			exit(-1);
		}

		// Resize
		this->thread_solution_buffer.resize(n_threads);
		for (std::vector<OUTPUT_FLOAT>& solution_buffer : this->thread_solution_buffer) {
			solution_buffer.resize(this->compilation.n_outputs);
		}
		this->thread_input.resize(n_threads);
		for (symx::avx::avector<COMPILED_FLOAT, 64>&input : this->thread_input) {
			input.resize(this->compilation.n_inputs);
		}
		this->thread_output.resize(n_threads);
		for (symx::avx::avector<COMPILED_FLOAT, 64>&output : this->thread_output) {
			output.resize(this->compilation.n_outputs);
		}
		if (this->summation_data.size() > 0) {
			this->thread_summation_buffer.resize(n_threads);
			for (symx::avx::avector<COMPILED_FLOAT, 64>&output : this->thread_summation_buffer) {
				output.resize(this->compilation.n_outputs);
			}
		}

		// Get the compiled function
		void(*f)(COMPILED_FLOAT*, COMPILED_FLOAT*) = this->compilation.template get_f<COMPILED_FLOAT>();
		const int n_inputs = this->compilation.n_inputs;
		const int n_outputs = this->compilation.n_outputs;

		// Conditional evaluation
		bool has_conditional_evaluation = conditional_evaluation != nullptr;
		if (has_conditional_evaluation) {
			this->active_elements_bool.resize(n_elements);

			#pragma omp parallel for schedule(static) num_threads(n_threads)
			for (int i = 0; i < n_elements; i++) {
				const int32_t* loc_conn = connectivity_data + i * this->connectivity_stride;
				this->active_elements_bool[i] = conditional_evaluation(i, omp_get_thread_num(), loc_conn);
			}

			this->active_elements.clear();
			for (int i = 0; i < n_elements; i++) {
				if (this->active_elements_bool[i]) {
					this->active_elements.push_back(i);
				}
			}
		}

		// Connectivity loop
		const int n_active_elements = has_conditional_evaluation ? (int)this->active_elements.size() : n_elements;
		const int total_avx_lines = (n_active_elements % N_SIMD == 0) ? n_active_elements / N_SIMD : n_active_elements / N_SIMD + 1;
		
		#pragma omp parallel for schedule(static) num_threads(n_threads) if(total_avx_lines > 2*n_threads)
		for (int avx_i = 0; avx_i < total_avx_lines; avx_i++) {
			const int thread_id = omp_get_thread_num();
			COMPILED_FLOAT* f_input = this->thread_input[thread_id].data();
			COMPILED_FLOAT* f_output = this->thread_output[thread_id].data();

			// SIMD range
			const int begin_i = N_SIMD * avx_i;
			const int end_i = std::min(N_SIMD * (avx_i + 1), n_active_elements);
			const int active_simd_elements = end_i - begin_i;

			// Gather element indices
			std::array<int32_t, N_SIMD> element_indices;
			if (has_conditional_evaluation) {
				for (int i = 0; i < active_simd_elements; i++) {
					element_indices[i] = this->active_elements[begin_i + i];
				}
			}
			else {
				for (int i = 0; i < active_simd_elements; i++) {
					element_indices[i] = begin_i + i;
				}
			}

			// Gather input
			UNDERLYING_FLOAT* input_scalar = reinterpret_cast<UNDERLYING_FLOAT*>(f_input);
			for (int32_t i = 0; i < active_simd_elements; i++) {
				const int32_t* loc_conn = this->get_connectivity_elem(element_indices[i]);
				for (const int symbol_i : this->non_summation_symbols_idx) {
					SymbolArrayMap& map = this->symbol_array_map[symbol_i];
					const int32_t loc_idx = loc_conn[map.connectivity_index];
					const int32_t value_idx = loc_idx * map.stride + map.offset;

					if (value_idx >= map.current_size) {
						std::cout << "symx error: index out of bounds for symbol " << symbol_number_as_str(symbol_i) << " for instance with name \"" + this->name + "\"" << std::endl;
						exit(-1);
					}

					input_scalar[N_SIMD * symbol_i + i] = static_cast<UNDERLYING_FLOAT>(map.array_begin[value_idx]);

					if (this->check_for_NaNs) {
						if (std::isnan(input_scalar[N_SIMD * symbol_i + i])) {
							std::cout << "symx error: NaN found in CompiledInLoop::run() in the input " << symbol_number_as_str(symbol_i) << " for instance with name \"" + this->name + "\"" << std::endl;
							exit(-1);
						}
					}
				}
			}

			// Run
			//// No summation
			if (this->summation_data.size() == 0) {
				f(f_input, f_output);
			}

			//// Summation
			else {
				COMPILED_FLOAT* buffer = this->thread_summation_buffer[thread_id].data();
				UNDERLYING_FLOAT* buffer_scalar = reinterpret_cast<UNDERLYING_FLOAT*>(buffer);
				UNDERLYING_FLOAT* output_scalar = reinterpret_cast<UNDERLYING_FLOAT*>(f_output);

				for (int i = 0; i < n_outputs * N_SIMD; i++) {
					output_scalar[i] = static_cast<UNDERLYING_FLOAT>(0.0);
				}

				for (int sum_it = 0; sum_it < this->summation_iterations; sum_it++) {

					// Update summation data
					for (int32_t i = 0; i < active_simd_elements; i++) {
						for (const int symbol_i : this->summation_symbols_idx) {
							SymbolArrayMap& map = this->symbol_array_map[symbol_i];
							input_scalar[N_SIMD * symbol_i + i] = static_cast<UNDERLYING_FLOAT>(map.array_begin[map.stride * sum_it + map.offset]);

							if (this->check_for_NaNs) {
								if (std::isnan(input_scalar[N_SIMD * symbol_i + i])) {
									std::cout << "symx error: NaN found in CompiledInLoop::run() in the summation input " << symbol_number_as_str(symbol_i) << " for instance with name \"" + this->name + "\"" << std::endl;
									exit(-1);
								}
							}
						}
					}

					// Run compiled function
					f(f_input, buffer);

					// Add contribution to the summation results
					for (int i = 0; i < n_outputs * N_SIMD; i++) {
						output_scalar[i] += buffer_scalar[i];
					}
				}
			}

			// Scatter output
			if constexpr (N_SIMD == 1 && std::is_same_v<UNDERLYING_FLOAT, OUTPUT_FLOAT>) {
				if (this->check_for_NaNs) {
					for (int32_t i = 0; i < n_outputs; i++) {
						if (std::isnan(f_output[i])) {
							std::cout << "symx error: NaN found in CompiledInLoop::run() in the outputs for instance with name \"" + this->name + "\"" << std::endl;
							std::cout << "Inputs: ";
							for (int32_t i = 0; i < n_inputs; i++) {
								std::cout << std::to_string(f_input[i]) << ", ";
							}
							std::cout << std::endl;
							exit(-1);
						}
					}
				}

				const int32_t* loc_conn = this->get_connectivity_elem(element_indices[0]);
				callback(element_indices[0], thread_id, loc_conn, f_output);
			}
			else {
				std::vector<OUTPUT_FLOAT>& solution_buffer = this->thread_solution_buffer[thread_id];
				const UNDERLYING_FLOAT* output_scalar = reinterpret_cast<UNDERLYING_FLOAT*>(f_output);
				for (int32_t i = 0; i < active_simd_elements; i++) {
					for (int32_t sol_i = 0; sol_i < n_outputs; sol_i++) {
						solution_buffer[sol_i] = static_cast<OUTPUT_FLOAT>(output_scalar[N_SIMD * sol_i + i]);

						if (this->check_for_NaNs) {
							if (std::isnan(solution_buffer[sol_i])) {
								std::cout << "symx error: NaN found in the outputs of CompiledInLoop::run() for instance with name \"" + this->name + "\"" << std::endl;
								std::cout << "Compile with scalar COMPILED_FLOAT to see the input." << std::endl;
								exit(-1);
							}
						}
					}
					const int32_t* loc_conn = this->get_connectivity_elem(element_indices[i]);
					callback(element_indices[i], thread_id, loc_conn, solution_buffer.data());
				}
			}
		}
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run(
		const int32_t n_threads, 
		std::function<void(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity, OUTPUT_FLOAT* solution)> callback)
	{
		this->run(n_threads, callback, this->conditional_evaluation_callback);
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run(const int32_t n_threads)
	{
		this->run(this->n_threads, this->evaluation_callback, this->conditional_evaluation_callback);
	}
}


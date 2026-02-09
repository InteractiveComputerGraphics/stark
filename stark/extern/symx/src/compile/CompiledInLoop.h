#pragma once
#include <functional>
#include <array>
#include <unordered_map>
#include <string>
#include <fmt/format.h>

#include <omp.h>

#include "../symbol/utils.h"
#include "View.h"
#include "MappedWorkspace.h"
#include "Compilation.h"
#include "type_utils.h"
#include "AlignmentAllocator.h"
#include "coloring.h"
#include "simd_shuffles.h"
#include "fetch_blocks.h"


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
		/* Definitions */
		using UNDERLYING_FLOAT = UNDERLYING_TYPE<COMPILED_FLOAT>;
		using spCIL = std::shared_ptr<CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>>;
		struct BlockFetch
		{
			const char* array_begin = nullptr;  // Beginning of the array
			int32_t connectivity_index = 0;  // Entry in the connectivity array the symbol is associated with
			int32_t stride_in_bytes = 0;  // Stride in the array of this symbol
			int32_t mult = 1;  // Multiplier for fixed values (no arrays)
			int32_t input_byte_offset = 0; // Offset in the input buffer to the compiled function
		};

		/* Fields */
		Compilation compilation;
		spMWS<INPUT_FLOAT> mws;
		
		// Thread buffers - using unique_ptr to ensure each buffer is independently heap-allocated,
		// preventing false sharing of cache lines between threads when buffers are accessed/resized.
		std::vector<std::unique_ptr<simd::avector<char, 64>>> thread_buf;  // [thread_id][buffer_idx]
		
		std::vector<std::vector<int32_t>> thread_elem_buffer;
		std::vector<BlockFetch> block_fetches;

		// Conditional evaluation
		std::vector<uint8_t> empty_condition;
		std::vector<int32_t> active_element_indices;

		// Coloring
		std::vector<std::vector<int>> color_bins;
		std::vector<int32_t> coloring_use_indices;
		bool use_coloring = false;
		
		// Misc
		std::string name = "";

	public:
		/* Definitions */
		struct Info {
			int n_inputs = -1;
			int n_outputs = -1;
			int n_elements = -1;
			int connectivity_stride = -1;

			FloatType input_float = FloatType::Double;
			FloatType compiled_type = FloatType::Double;
			FloatType output_float = FloatType::Double;

			bool was_cached = false;
			double runtime_codegen = 0.0;
			double runtime_compilation = 0.0;
			int n_bytes_symbols = 0;
		};

		/* Methods */
		CompiledInLoop() = default;
		~CompiledInLoop() = default;
		CompiledInLoop(const CompiledInLoop&) = delete; // Copying makes unclear who owns the DLL
		CompiledInLoop(const spMWS<INPUT_FLOAT>& mws, const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id = "");
		static spCIL create(const spMWS<INPUT_FLOAT>& mws, const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id = "");
		
		/*
		* This can load having a MappedWorkspace context which does not contain the output symbols.
		* In general, only the input symbols are needed for evaluation. Output symbols are not required, it's just an array of data that the user should know what to do with.
		* A prominent use case for `load_if_cached` is loading a complex compiled object that exists and matches the cached_id without churning the math.
		* For instance, we can safely load the Hessian compiled object from a cache related to the energy form and dofs, without needing to differentiate it.
		*/
		bool load_if_cached(const spMWS<INPUT_FLOAT>& mws, std::string name, std::string folder, std::string cache_id);
		void compile(const spMWS<INPUT_FLOAT>& mws, const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id = "");
		void try_load_otherwise_compile(const spMWS<INPUT_FLOAT>& mws, const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id = "");
		
		bool is_valid() const;
		bool was_cached() const;
		// Info get_info() const;
		
		/*
		* @param conn_indices: indices in the connectivity array to use for coloring. E.g. tet4 conn = { enum, v0, v1, v2, v4 } -> conn_indices = {1,2,3,4}
		* Warning: This is a dangerous stateful option.
		* If coloring is enabled, the evaluation will happen on the colors.
		* If the connectivity changes, but update_coloring is not called, the coloring will be stale and the evaluation will be incorrect.
		*/
		void enable_coloring(const std::vector<int>& conn_indices);
		void disable_coloring();
		void update_coloring();

		// Run
		void run(
			int32_t n_threads,
			std::function<void(View<OUTPUT_FLOAT> solution, int32_t element_idx, int32_t thread_id, View<const int32_t> connectivity)> callback
		);
		void run(
			int32_t n_threads,
			std::function<void(View<OUTPUT_FLOAT> solution, int32_t element_idx, int32_t thread_id, View<const int32_t> connectivity)> callback,
			const std::vector<uint8_t>& is_element_active
		);

	private:
		void _check_types();
	};



	// ===================================== DEFINITIONS =====================================
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::_check_types()
	{
		static_assert((std::is_same_v<INPUT_FLOAT, float> || std::is_same_v<INPUT_FLOAT, double>), "INPUT_FLOAT must be {float, double}");
		static_assert((
			std::is_same_v<COMPILED_FLOAT, float> ||
			std::is_same_v<COMPILED_FLOAT, double>
#ifdef SYMX_ENABLE_AVX2
			||
			std::is_same_v<COMPILED_FLOAT, __m128d> ||
			std::is_same_v<COMPILED_FLOAT, __m128> ||
			std::is_same_v<COMPILED_FLOAT, __m256d> ||
			std::is_same_v<COMPILED_FLOAT, __m256> ||
			std::is_same_v<COMPILED_FLOAT, __m512d> ||
			std::is_same_v<COMPILED_FLOAT, __m512>
#endif
			),
			"COMPILED_FLOAT must be {float, double, __m128d, __m128, __m256d, __m256}");
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::CompiledInLoop(const spMWS<INPUT_FLOAT>& mws, const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id)
	{
		this->try_load_otherwise_compile(mws, expr, name, folder, cache_id);
	}
    template <typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
    inline std::shared_ptr<CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>> CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::create(const spMWS<INPUT_FLOAT> &mws, const std::vector<Scalar> &expr, std::string name, std::string folder, std::string cache_id)
    {
       return std::make_shared<CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>>(mws, expr, name, folder, cache_id);
    }
    template <typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
    inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::compile(const spMWS<INPUT_FLOAT> &mws, const std::vector<Scalar> &expr, std::string name, std::string folder, std::string cache_id)
    {
		this->mws = mws;
		this->name = name;
		this->compilation.template compile<COMPILED_FLOAT>(expr, name, folder, cache_id);
		this->_check_types();
	}
	template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::try_load_otherwise_compile(const spMWS<INPUT_FLOAT>& mws, const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id)
	{
		if (cache_id == "") {
			cache_id = get_checksum(expr);
		}
		if (!this->load_if_cached(mws, name, folder, cache_id)) {
			this->compile(mws, expr, name, folder, cache_id);
		}
	}
    template <typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
    inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::enable_coloring(const std::vector<int> &conn_indices)
    {
		this->use_coloring = true;
		this->coloring_use_indices = conn_indices;
		this->update_coloring();
    }
    template <typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
    inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::disable_coloring()
    {
		this->use_coloring = false;
		this->color_bins.clear();
    }
    template <typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
    inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::update_coloring()
    {
		compute_color_graph(
			this->color_bins,
			this->mws->conn.data(),
			this->mws->conn.n_elements(),
			this->mws->conn.stride,
			this->coloring_use_indices
		);
    }
    template <typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
    inline bool CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::load_if_cached(const spMWS<INPUT_FLOAT>& mws, std::string name, std::string folder, std::string cache_id)
    {
		const bool success = this->compilation.template load_if_cached<COMPILED_FLOAT>(name, folder, cache_id);
		if (success) {
			this->_check_types();
			this->mws = mws;
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
		return this->compilation.was_cached();
	}
	// template<typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
	// inline typename CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::Info CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::get_info() const
	// {
	// 	Compilation::Info compilation_info = this->compilation.get_info();
	// 	Info info;
	// 	info.n_inputs = compilation_info.n_inputs;
	// 	info.n_outputs = compilation_info.n_outputs;
	// 	info.n_elements = this->get_n_elements();
	// 	info.connectivity_stride = this->get_connectivity_stride();
	// 	info.input_float = get_float_type_as_enum<INPUT_FLOAT>();
	// 	info.compiled_type = this->compilation.get_compiled_type();
	// 	info.output_float = get_float_type_as_enum<OUTPUT_FLOAT>();
	// 	info.was_cached = this->was_cached();
	// 	info.runtime_codegen = compilation_info.runtime_codegen;
	// 	info.runtime_compilation = compilation_info.runtime_compilation;
	// 	info.n_bytes_symbols = compilation_info.n_bytes_symbols;
	// 	return info;
	// }

    template <typename INPUT_FLOAT, typename COMPILED_FLOAT, typename OUTPUT_FLOAT>
    inline void CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>::run(int32_t n_threads, std::function<void(View<OUTPUT_FLOAT> solution, int32_t element_idx, int32_t thread_id, View<const int32_t> connectivity)> callback)
    {
		this->run(n_threads, callback, this->empty_condition); // No conditional evaluation
    }

    template<typename INPUT_FLOAT, typename COMPILED_FLOAT = INPUT_FLOAT, typename OUTPUT_FLOAT = INPUT_FLOAT>
	using spCIL = std::shared_ptr<CompiledInLoop<INPUT_FLOAT, COMPILED_FLOAT, OUTPUT_FLOAT>>;
}

#include "CompiledInLoop_run.h"

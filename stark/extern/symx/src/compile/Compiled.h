#pragma once
#include <type_traits>
#include <vector>
#include <string>

#include "View.h"
#include "Compilation.h"
#include "AlignmentAllocator.h"

namespace symx
{

	/*
		Provides a handle and typed interface to compiled functions.
		
		This is the entry-level class for SymX compiled expressions, providing:
		- Thread-safe evaluation with per-thread buffers
		- Simple scalar, vector, and matrix input handling
		- Caching support for fast reload
		- Clean, focused API for direct function evaluation
		
		Unlike CompiledInLoop, this class doesn't handle connectivity, loops, 
		or complex data layouts - making it perfect for straightforward 
		mathematical function evaluation.
	*/
	template<typename FLOAT>
	class Compiled
	{
	private:
		Compilation compilation;
		int n_threads = 1;  // Number of threads configured

	public:

		/* Fields */
		std::vector<simd::avector<FLOAT, 64>> thread_input;   // Per-thread input buffers
		std::vector<simd::avector<FLOAT, 64>> thread_output;  // Per-thread output buffers
		std::string name = "";

		/* Methods */
		Compiled() = default;
		~Compiled() = default;
		Compiled(const Compiled&) = delete;  // Copying makes unclear who owns the DLL
		Compiled(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id = "");
		
		// Compilation management
		void compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id = "");
		bool load_if_cached(std::string name, std::string folder, std::string cache_id);
		void try_load_otherwise_compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string cache_id = "");
		
		// Status queries
		bool is_valid();
		bool was_cached() const;
		Compilation::Info get_compilation_info() const;
		
		// Thread management
		void set_n_threads(int n_threads);
		int get_n_threads() const;
		
		// Input
		void set(const Scalar& scalar, const FLOAT& v);
		void set(const Scalar& scalar, const FLOAT& v, int thread_id);

		template<typename IndexableVector>
		void set(const Vector& vector, const IndexableVector& v);
		template<typename IndexableVector>
		void set(const Vector& vector, const IndexableVector& v, int thread_id);

		void set(const Matrix& matrix, const FLOAT* v);
		void set(const Matrix& matrix, const FLOAT* v, int thread_id);
		template<typename RowMajorParenthesisIndexableMatrix>
		typename std::enable_if<!std::is_pointer<RowMajorParenthesisIndexableMatrix>::value, void>::type
		set(const Matrix& matrix, const RowMajorParenthesisIndexableMatrix& m);
		template<typename RowMajorParenthesisIndexableMatrix>
		typename std::enable_if<!std::is_pointer<RowMajorParenthesisIndexableMatrix>::value, void>::type
		set(const Matrix& matrix, const RowMajorParenthesisIndexableMatrix& m, int thread_id);
		
		// Evaluation
		View<FLOAT> run();
		View<FLOAT> run(int thread_id);
		
		// Information
		int get_n_inputs();
		int get_n_outputs();
		std::string get_name() const;

		// Raw buffer access for advanced users
		FLOAT* get_input_buffer(int thread_id = 0);
		FLOAT* get_output_buffer(int thread_id = 0);
		const FLOAT* get_input_buffer(int thread_id = 0) const;
		const FLOAT* get_output_buffer(int thread_id = 0) const;

	private:
		void _resize_buffers();
		void _resize_buffers(int thread_id);
	};
    
	
	
	// Definitions ==================================================================================
	template <typename FLOAT>
    template <typename IndexableVector>
    inline void Compiled<FLOAT>::set(const Vector &vector, const IndexableVector &v)
    {
		this->set(vector, v, 0);
    }
    template <typename FLOAT>
    template <typename IndexableVector>
    inline void Compiled<FLOAT>::set(const Vector &vector, const IndexableVector &v, int thread_id)
    {
		for (int i = 0; i < vector.size(); i++) {
			this->set(vector[i], v[i], thread_id);
		}
		
    }
    template <typename FLOAT>
    template <typename RowMajorParenthesisIndexableMatrix>
    inline typename std::enable_if<!std::is_pointer<RowMajorParenthesisIndexableMatrix>::value, void>::type
    Compiled<FLOAT>::set(const Matrix &matrix, const RowMajorParenthesisIndexableMatrix &m)
    {
		this->set(matrix, m, 0);
    }
    template <typename FLOAT>
    template <typename RowMajorParenthesisIndexableMatrix>
    inline typename std::enable_if<!std::is_pointer<RowMajorParenthesisIndexableMatrix>::value, void>::type
    Compiled<FLOAT>::set(const Matrix &matrix, const RowMajorParenthesisIndexableMatrix &m, int thread_id)
    {
		const int nrows = matrix.rows();
		const int ncols = matrix.cols();
		for (int i = 0; i < nrows; i++) {
			for (int j = 0; j < ncols; j++) {
				this->set(matrix(i, j), m(i, j), thread_id);
			}
		}
    }
}

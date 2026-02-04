#pragma once
#include <type_traits>

#include "Compilation.h"
#include "AlignmentAllocator.h"

namespace symx
{

	/*
		Provides a handle and typed interface to compiled functions.
	*/
	template<typename FLOAT>
	class Compiled
	{
	public:

		/* Fields */
		Compilation compilation;
		symx::avx::avector<FLOAT, 64> input;
		symx::avx::avector<FLOAT, 64> output;

		/* Methods */
		Compiled() = default;
		~Compiled() = default;
		Compiled(const Compiled&) = delete;  // Copying makes unclear who owns the DLL
		Compiled(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", bool suppress_compiler_output = true);
		void compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id = "", bool suppress_compiler_output = true);
		bool load_if_cached(std::string name, std::string folder, std::string id);
		bool is_valid();
		bool was_cached();

		void set(const Scalar& symbol, const FLOAT* v);
		void set(const Vector& vector, const FLOAT* v);
		void set_voidptr(const Scalar& symbol, const void* v);

		FLOAT* run();

		int get_n_inputs();
		int get_n_outputs();

	private:
		void _resize_buffers();
	};
}



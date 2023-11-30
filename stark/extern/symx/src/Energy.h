#pragma once
#include <iostream>

#include <Eigen/Sparse>

#include "Assembly.h"
#include "SymbolicWorkSpace.h"
#include "CompiledDerivativesLoop.h"


namespace symx
{
	struct DoF { int idx; };

	class Energy
	{
	private:
		struct DoFBlockGlobalIndex
		{
			int dof_set;  // nth set of degrees of freedom
			int conn_idx;  // nth entry in the local element connectivity
		};

	public:
		/* Fields */
		int32_t n_items_per_element = -1;
		CompiledDerivativesLoop<double, double, double> compiled_derivatives;
		CompiledDerivativesLoop<double, double, double> compiled_derivatives_d;  // For energies with branches
		std::vector<DoFBlockGlobalIndex> dof_block_global_index;
		std::vector<int> dof_dof_array_idx;  // which dof_array is assigned to each dof
		std::string name;
		int n_dofs = -1;
		std::string working_directory;
		bool has_branches = false;
		bool was_cached = false;
		double runtime_differentiation = 0.0;
		double runtime_codegen = 0.0;
		double runtime_compilation = 0.0;
		uint64_t n_bytes_symbols = 0;
		bool project_to_PD = false;
		bool is_active = true;

		// Symbols
		SymbolicWorkSpace sws;
		//std::vector<std::pair<std::pair<Vector, Index>, std::function<const double*()>>> block_symbols_data_pairs;
		std::vector<Scalar> dof_symbols;
		std::unique_ptr<Scalar> expr = nullptr;
		int name_conter = 0;

		// Conditional evaluation
		CompiledInLoop<double, double, double> compiled_condition;
		bool has_condition = false;
		std::unique_ptr<Scalar> cond = nullptr;
		std::vector<uint8_t> positive_conditions;
		std::vector<int32_t> connectivity_after_condition;


		/* Methods */
		Energy(const Energy&) = delete; // Copying makes unclear who owns the DLL
		Energy(std::string name, std::string working_directory, std::function<const int32_t*()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element);
		Energy(std::string name, std::string working_directory, std::vector<int32_t>& arr, const int32_t n_items_per_element);
		template<std::size_t N>
		Energy(std::string name, std::string working_directory, std::vector<std::array<int32_t, N>>& arr);

		void set(const Scalar& expr);
		// When the condition is positive, the energy becomes active
		void set_with_condition(const Scalar& expr, const Scalar& cond);
		bool is_expression_set() const;
		void deferred_init(std::vector<std::function<double* ()>> dof_arrays, const bool force_compilation, const bool suppress_compiler_output);
		void activate(const bool activate);
	
		// Make fixed Scalar
		Scalar make_scalar(std::function<const double* ()> data, const std::string name = "");
		Scalar make_scalar(const double& scalar, const std::string name = "");

		// Make fixed Vector
		Vector make_vector(std::function<const double* ()> data, const int32_t stride, const std::string name = "");
		template<typename STATIC_VECTOR>
		Vector make_vector(STATIC_VECTOR& arr, const std::string name = "");

		// Make Scalar
		Scalar make_scalar(std::function<const double* ()> data, std::function<int32_t()> size, const Index& idx, const std::string name = "");
		template<typename DYNAMIC_VECTOR>
		Scalar make_scalar(const DYNAMIC_VECTOR& arr, const Index& idx, const std::string name = "");

		// Make Vector
		Vector make_vector(std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const Index& idx, const std::string name = "");
		template<typename STATIC_VECTOR>
		Vector make_vector(std::vector<STATIC_VECTOR>& arr, const Index& idx, const std::string name = "");
		template<typename DYNAMIC_VECTOR>
		Vector make_vector(DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string name = "");

		// Make DoF Vector
		Vector make_dof_vector(const DoF& dof, std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const Index& idx, const std::string name = "");
		template<typename STATIC_VECTOR>
		Vector make_dof_vector(const DoF& dof, std::vector<STATIC_VECTOR>& arr, const Index& idx, const std::string name = "");
		template<typename DYNAMIC_VECTOR>
		Vector make_dof_vector(const DoF& dof, DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string name = "");

		// Make Vectors
		std::vector<Vector> make_vectors(std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const std::vector<Index>& indices, const std::string name = "");
		template<typename STATIC_VECTOR>
		std::vector<Vector> make_vectors(std::vector<STATIC_VECTOR>& arr, const std::vector<Index>& indices, const std::string name = "");
		template<typename STATIC_VECTOR>
		std::vector<Vector> make_vectors(std::vector<STATIC_VECTOR>& arr, const Element& element, const std::string name = "");
		template<typename DYNAMIC_VECTOR>
		std::vector<Vector> make_vectors(DYNAMIC_VECTOR& arr, const int32_t stride, const std::vector<Index>& indices, const std::string name = "");

		// Make DoF Vectors
		std::vector<Vector> make_dof_vectors(const DoF& dof, std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const std::vector<Index>& indices, const std::string name = "");
		template<typename STATIC_VECTOR>
		std::vector<Vector> make_dof_vectors(const DoF& dof, std::vector<STATIC_VECTOR>& arr, const std::vector<Index>& indices, const std::string name = "");
		template<typename STATIC_VECTOR>
		std::vector<Vector> make_dof_vectors(const DoF& dof, std::vector<STATIC_VECTOR>& arr, const Element& element, const std::string name = "");
		template<typename DYNAMIC_VECTOR>
		std::vector<Vector> make_dof_vectors(const DoF& dof, DYNAMIC_VECTOR& arr, const int32_t stride, const std::vector<Index>& indices, const std::string name = "");

		// Make Matrix
		Matrix make_matrix(std::function<const double* ()> data, std::function<int32_t()> size, const std::array<int, 2> shape, const Index& idx, const std::string name = "");
		template<typename STATIC_VECTOR>
		Matrix make_matrix(std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const Index& idx, const std::string name = "");
		template<typename DYNAMIC_VECTOR>
		Matrix make_matrix(DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const Index& idx, const std::string name = "");

		// Make summation vector
		template<std::size_t STRIDE>
		Vector make_summation_vector(const std::vector<std::array<double, STRIDE>>& iteration_vectors);

		// Make constants
		Scalar make_zero();
		Scalar make_one();
		Vector make_zero_vector(const int32_t size);
		Matrix make_zero_matrix(const std::array<int32_t, 2> shape);
		Matrix make_identity_matrix(const int32_t size);

		// Evaluations
		void evaluate_E(Assembly& assembly, const bool runtime_NaN_check = false);
		void evaluate_E_grad(Assembly& assembly, const bool runtime_NaN_check = false);
		void evaluate_E_grad_hess(Assembly& assembly, const bool runtime_NaN_check = false);

	private:
		void _update_connectivity_conditionally(const int n_threads);
		std::string _get_symbol_name(std::string user_name);
		void _init(std::string name, std::string working_directory, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element);
	};





	// =========================================================================================================================================================
	template<std::size_t N>
	inline Energy::Energy(std::string name, std::string working_directory, std::vector<std::array<int32_t, N>>& arr)
	{
		this->_init(name, working_directory, l2data_int(arr), l2n_elements_int(arr), N);
	}
	template<typename STATIC_VECTOR>
	inline Vector Energy::make_vector(STATIC_VECTOR& arr, const std::string name)
	{
		return this->make_vector(l2data_double(arr), sizeof(STATIC_VECTOR)/sizeof(double), name);
	}
	template<typename DYNAMIC_VECTOR>
	inline Scalar Energy::make_scalar(const DYNAMIC_VECTOR& arr, const Index& idx, const std::string name)
	{
		return this->make_scalar(l2data_double(arr), l2count_double(arr), idx, name);
	}
	template<typename STATIC_VECTOR>
	inline Vector Energy::make_vector(std::vector<STATIC_VECTOR>& arr, const Index& idx, const std::string name)
	{
		return this->make_vector(l2data_double(arr), l2count_double(arr), sizeof(STATIC_VECTOR)/sizeof(double), idx, name);
	}
	template<typename DYNAMIC_VECTOR>
	inline Vector Energy::make_vector(DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string name)
	{
		return this->make_vector(l2data_double(arr), l2count_double(arr), stride, idx, name);
	}
	template<typename STATIC_VECTOR>
	inline Vector Energy::make_dof_vector(const DoF& dof, std::vector<STATIC_VECTOR>& arr, const Index& idx, const std::string name)
	{
		return this->make_dof_vector(dof, l2data_double(arr), l2count_double(arr), sizeof(STATIC_VECTOR)/sizeof(double), idx, name);
	}
	template<typename DYNAMIC_VECTOR>
	inline Vector Energy::make_dof_vector(const DoF& dof, DYNAMIC_VECTOR& arr, const int32_t stride, const Index& idx, const std::string name)
	{
		return this->make_dof_vector(dof, l2data_double(arr), l2count_double(arr), stride, idx, name);
	}
	template<typename STATIC_VECTOR>
	inline std::vector<Vector> Energy::make_vectors(std::vector<STATIC_VECTOR>& arr, const std::vector<Index>& indices, const std::string name)
	{
		return this->make_vectors(l2data_double(arr), l2count_double(arr), sizeof(STATIC_VECTOR)/sizeof(double), indices, name);
	}
	template<typename STATIC_VECTOR>
	inline std::vector<Vector> Energy::make_vectors(std::vector<STATIC_VECTOR>& arr, const Element& element, const std::string name)
	{
		return this->make_vectors(arr, element.all(), name);
	}
	template<typename DYNAMIC_VECTOR>
	inline std::vector<Vector> Energy::make_vectors(DYNAMIC_VECTOR& arr, const int32_t stride, const std::vector<Index>& indices, const std::string name)
	{
		return this->make_vectors(l2data_double(arr), l2count_double(arr), stride, indices, name);
	}
	template<typename STATIC_VECTOR>
	inline std::vector<Vector> Energy::make_dof_vectors(const DoF& dof, std::vector<STATIC_VECTOR>& arr, const std::vector<Index>& indices, const std::string name)
	{
		return this->make_dof_vectors(dof, l2data_double(arr), l2count_double(arr), sizeof(STATIC_VECTOR)/sizeof(double), indices, name);
	}
	template<typename STATIC_VECTOR>
	inline std::vector<Vector> Energy::make_dof_vectors(const DoF& dof, std::vector<STATIC_VECTOR>& arr, const Element& element, const std::string name)
	{
		return this->make_dof_vectors(dof, arr, element.all(), name);
	}
	template<typename DYNAMIC_VECTOR>
	inline std::vector<Vector> Energy::make_dof_vectors(const DoF& dof, DYNAMIC_VECTOR& arr, const int32_t stride, const std::vector<Index>& indices, const std::string name)
	{
		return this->make_dof_vectors(dof, l2data_double(arr), l2count_double(arr), stride, indices, name);
	}
	template<typename STATIC_VECTOR>
	inline Matrix Energy::make_matrix(std::vector<STATIC_VECTOR>& arr, const std::array<int, 2> shape, const Index& idx, const std::string name)
	{
		return this->make_matrix(l2data_double(arr), l2count_double(arr), shape, idx, name);
	}
	template<typename DYNAMIC_VECTOR>
	inline Matrix Energy::make_matrix(DYNAMIC_VECTOR& arr, const std::array<int, 2> shape, const Index& idx, const std::string name)
	{
		return this->make_matrix(l2data_double(arr), l2count_double(arr), shape, idx, name);
	}
	template<std::size_t STRIDE>
	inline Vector Energy::make_summation_vector(const std::vector<std::array<double, STRIDE>>& iteration_vectors)
	{
		// Flatten the data
		std::vector<double> summation_data;
		for (int i = 0; i < (int)iteration_vectors.size(); i++) {
			for (int j = 0; j < (int)STRIDE; j++) {
				summation_data.push_back(iteration_vectors[i][j]);
			}
		}

		// Declare a Symbol vector so they can be used in the symbol engine
		symx::Vector summation = this->sws.make_vector("summation", (int)STRIDE);

		// Set the summation symbols with the data for the loop
		this->compiled_derivatives.set_summation_vector(summation, summation_data, (int)STRIDE);
		this->compiled_condition.set_summation_vector(summation, summation_data, (int)STRIDE);

		return summation;
	}
}
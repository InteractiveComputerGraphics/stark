#pragma once
#include <iostream>

#include <Eigen/Sparse>

#include "Assembly.h"
#include "SymbolicWorkSpace.h"
#include "CompiledInLoop.h"
#include "diff.h"

namespace symx
{
	/**
	* Simple representation of a set of degrees of freedom.
	*/
	struct DoF 
	{ 
		int idx; 
	};

	/**
	* Identifies an entry in the element connectivity with a DoF set.
	*/
	struct DoFInConn
	{
		int dof_set;  // nth set of degrees of freedom
		int conn_idx;  // nth entry in the local element connectivity
	};
	
	/**
	* Element handler for easy data access.
	*/
	class ElementInfo
	{
	private:
		int32_t conn_stride;
		const int32_t* conn;
		int32_t idx;
		const std::vector<DoFInConn>& dof_in_conn;
	public:
		ElementInfo(const int32_t conn_stride, const int32_t* conn, const int32_t idx, const std::vector<DoFInConn>& dof_in_conn)
			: conn_stride(conn_stride), conn(conn), idx(idx), dof_in_conn(dof_in_conn) {}
		inline int32_t get_stride() const { return conn_stride; }
		inline int32_t get_idx() const { return idx; }
		inline int32_t get_n_dof_nodes() const { return (int32_t)dof_in_conn.size(); }
		inline int32_t get_entry(const int32_t i) const { return conn[i]; }
		inline int32_t get_dof_node_set(const int32_t i) const { return dof_in_conn[i].dof_set; }
		inline int32_t get_dof_node_idx(const int32_t dof_i) const { return dof_in_conn[dof_i].conn_idx; }
		inline int32_t get_dof_node(const int32_t dof_i) const { return get_entry(get_dof_node_idx(dof_i)); }
	};

	class Energy
	{
	private:

	public:
		/* Fields */
		// Compiled functions
		CompiledInLoop<double> E;   // Energy
		CompiledInLoop<double> dE;  // Enegy and gradient
		CompiledInLoop<double> hE;  // Energy, gradient and hessian
		CompiledInLoop<double> C;   // Condition

		// Parameters
		std::string name;
		std::string working_directory;
		int n_dofs = -1;
		bool project_to_PD = false;
		bool is_active = true;
		bool check_for_duplicate_dofs = true;

		// Bookkeeping
		std::vector<DoFInConn> dof_in_conn;
		bool was_cached = false;

		// Performance metrics
		double runtime_differentiation = 0.0;
		double runtime_codegen = 0.0;
		double runtime_compilation = 0.0;
		uint64_t n_bytes_symbols = 0;

		// Conditional evaluation
		std::unique_ptr<Scalar> cond = nullptr;
		std::vector<uint8_t> has_element_positive_condition;
		std::vector<uint8_t> user_defined_element_condition;

		// Symbols
		SymbolicWorkSpace sws;
		std::vector<Scalar> dof_symbols;
		std::unique_ptr<Scalar> expr = nullptr;
		int name_conter = 0;


		/* Methods */
		Energy(const Energy&) = delete; // Copying makes unclear who owns the DLL
		Energy(std::string name, std::string working_directory, std::function<const int32_t*()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element);
		Energy(std::string name, std::string working_directory, std::vector<int32_t>& arr, const int32_t n_items_per_element);
		template<std::size_t N>
		Energy(std::string name, std::string working_directory, std::vector<std::array<int32_t, N>>& arr);
		void deferred_init(const bool force_compilation, const bool force_load, const bool suppress_compiler_output);

		void set(const Scalar& expr);
		void set_with_condition(const Scalar& expr, const Scalar& cond);
		void set_project_to_PD(const bool project_to_PD);
		void set_cse_mode(CSE mode = CSE::Safe);
		bool is_expression_set() const;
		void activate(const bool activate);
		void disable_check_for_duplicate_dofs();
		void set_check_for_NaNs(const bool check);
		int get_n_elements() const;
		int get_connectivity_stride() const;
		const int32_t* get_connectivity_elem(const int32_t element_idx) const;
		bool has_symbolic_element_condition() const;
		bool has_user_defined_element_condition() const;
		ElementInfo get_element_info(const int32_t element_idx) const;
		void disable_user_element_condition();
		void enable_user_element_condition();
		void set_user_element_condition(const int32_t element_idx, const bool activate);
	
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

		template<std::size_t STRIDE>
		Scalar add_for_each(const std::vector<std::array<double, STRIDE>>& iteration_vectors, std::function<Scalar(Vector& vec)> f);

		// Make constants
		Scalar make_zero();
		Scalar make_one();
		Vector make_zero_vector(const int32_t size);
		Matrix make_zero_matrix(const std::array<int32_t, 2> shape);
		Matrix make_identity_matrix(const int32_t size);

		// Evaluations
		void evaluate_E(Assembly& assembly);
		void evaluate_E_grad(Assembly& assembly);
		void evaluate_E_grad_hess(Assembly& assembly);
		void add_topology(Assembly& assembly, std::vector<std::vector<int32_t>>& topology);

	private:
		const std::vector<uint8_t>& _evaluate_element_conditions(const int n_threads);
		std::function<bool(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity)> _get_updated_conditional_evaluation(const int n_threads);
		void _for_each_compiled(const std::function<void(CompiledInLoop<double>&)> f);
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
		this->_for_each_compiled([&](CompiledInLoop<double>& compiled) {
			compiled.set_summation_vector(summation, summation_data, (int)STRIDE);
		});

		return summation;
	}
	template<std::size_t STRIDE>
	inline Scalar Energy::add_for_each(const std::vector<std::array<double, STRIDE>>& iteration_vectors, std::function<Scalar(Vector& vec)> f)
	{
		Vector summation_vector = this->make_summation_vector(iteration_vectors);
		return f(summation_vector);
	}
}

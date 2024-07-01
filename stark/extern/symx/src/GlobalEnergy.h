#pragma once
#include <vector>
#include <string>
#include <algorithm>
#include <functional>

#include "LabelledConnectivity.h"
#include "Element.h"
#include "Energy.h"
#include "Assembly.h"


namespace symx
{
	class GlobalEnergy
	{
	public:
		/* Types */
		struct CompilationOptions
		{
			int n_threads = -1;
			bool force_compilation = false;
			bool force_load = false;
			bool suppress_compiler_output = true;
			std::function<void(const std::string&)> msg_callback = nullptr;
		};

		constexpr static std::size_t BLOCK_SIZE = 3;

		/* Fields */
		std::vector<std::unique_ptr<Energy>> energies;
		std::vector<std::function<double*()>> dof_data;
		std::vector<std::function<int32_t()>> dof_ndofs;
		std::vector<std::string> dof_labels;

		std::string working_directory = "NO_PATH_SET_IN_SimulationWorkSpace";
		bool runtime_NaN_check = true;
		int n_threads = -1;
		bool is_initialized = false;
		CSE cse_mode = CSE::Safe;

		double runtime_differentiation = 0.0;
		double runtime_codegen = 0.0;
		double runtime_compilation = 0.0;
		uint64_t symbols_bytes = 0;

		// External contributions
		std::vector<std::function<void(Assembly&)>> external_E;
		std::vector<std::function<void(Assembly&)>> external_E_grad;
		std::vector<std::function<void(Assembly&)>> external_E_grad_hess;

		// Assembly
		Assembly assembly;

		// Topology
		std::vector<std::vector<int32_t>> topology;

		/* Methods */
		GlobalEnergy() = default;
		~GlobalEnergy() = default;
		GlobalEnergy(const GlobalEnergy&) = delete;
		
		void add_energy(std::string name, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element, std::function<void(Energy&, Element&)> energy, const std::vector<std::string>& labels = std::vector<std::string>());
		void add_energy(std::string name, const std::vector<int32_t>& arr, const int32_t n_items_per_element, std::function<void(Energy&, Element&)> energy);
		template<std::size_t N>
		void add_energy(std::string name, const std::vector<std::array<int32_t, N>>& arr, std::function<void(Energy&, Element&)> energy);
		template<std::size_t N>
		void add_energy(std::string name, const LabelledConnectivity<N>& conn, std::function<void(Energy&, Element&)> energy);
		void add_external_contributions(std::function<void(Assembly&)> E, std::function<void(Assembly&)> E_grad, std::function<void(Assembly&)> E_grad_hess);

		DoF add_dof_array(std::function<double*()> data, std::function<int32_t()> ndofs, std::string label = "");
		template<typename STATIC_VECTOR>  // std::vector<std::array<double, 3>>, std::vector<Eigen::Vector3d>...
		DoF add_dof_array(std::vector<STATIC_VECTOR>& arr, std::string label = "");
		template<typename DYNAMIC_VECTOR>  // std::vector<double>, Eigen::VectorXd...
		DoF add_dof_array(DYNAMIC_VECTOR& arr, std::string label = "");

		void compile(std::string working_directory);
		void compile(std::string working_directory, CompilationOptions options);
		Assembled evaluate_E();
		Assembled evaluate_E_grad();
		Assembled evaluate_E_grad_hess();
		const std::vector<std::vector<int32_t>>& compute_topology();  // WARNING: Ignores external contributions
		void get_dofs(double* u) const;
		void apply_dof_increment(const double* du);
		void set_dofs(const double* u);
		void set_project_to_PD(const bool activate);
		void set_cse_mode(CSE mode = CSE::Safe);
		void set_check_for_NaNs(const bool activate);
		void test_derivatives_with_FD(const double h = 1e-7);
		void clear_user_conditional_evaluation();
		void activate_elements_from_dofs(const std::vector<uint8_t>& activation, const int32_t n_threads);

		int get_total_n_dofs() const;
		int32_t get_n_dof_sets() const;
		std::vector<int32_t> get_dofs_offsets() const;

	private:
		void _exit_if_not_initialized() const;
	};


	template<std::size_t N>
	inline void GlobalEnergy::add_energy(std::string name, const LabelledConnectivity<N>& conn, std::function<void(Energy&, Element&)> energy)
	{
		std::vector<std::string> labels;
		for (const std::string& s : conn.labels) {
			labels.push_back(s);
		}
		this->add_energy(name, l2data_int(conn.data), l2n_elements_int(conn.data), N, energy, labels);
	}
	template<std::size_t N>
	inline void GlobalEnergy::add_energy(std::string name, const std::vector<std::array<int32_t, N>>& arr, std::function<void(Energy&, Element&)> energy)
	{
		this->add_energy(name, l2data_int(arr), l2n_elements_int(arr), N, energy);
	}
	template<typename STATIC_VECTOR>
	inline DoF GlobalEnergy::add_dof_array(std::vector<STATIC_VECTOR>& arr, std::string label)
	{
		return this->add_dof_array(l2data_double_mut(arr), l2count_double(arr), label);
	}
	template<typename DYNAMIC_VECTOR>
	inline DoF GlobalEnergy::add_dof_array(DYNAMIC_VECTOR& arr, std::string label)
	{
		return this->add_dof_array(l2data_double_mut(arr), l2count_double(arr), label);
	}
}

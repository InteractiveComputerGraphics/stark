#include "GlobalEnergy.h"

std::string two_columns(const std::string& str1, const std::string& str2, size_t distance) 
{
	std::string result = str1;

	if (result.length() < distance) {
		result.append(distance - result.length(), '.');
	}
	else {
		result.replace(distance, result.length() - distance, result.length() - distance, '.');
	}

	return result + str2;
}// -------------------------------------------------------------------

void symx::GlobalEnergy::add_external_contributions(std::function<void(Assembly&)> E, std::function<void(Assembly&)> E_grad, std::function<void(Assembly&)> E_grad_hess)
{
	this->external_E.push_back(E);
	this->external_E_grad.push_back(E_grad);
	this->external_E_grad_hess.push_back(E_grad_hess);
}
symx::DoF symx::GlobalEnergy::add_dof_array(std::function<double* ()> data, std::function<int32_t()> ndofs, std::string label)
{
	this->dof_data.push_back(data);
	this->dof_ndofs.push_back(ndofs);
	this->dof_labels.push_back(label);
	return { (int)this->dof_data.size() - 1 };
}
void symx::GlobalEnergy::add_energy(std::string name, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element, std::function<void(Energy&, Element&)> energy, const std::vector<std::string>& labels)
{
	for (auto& energy : this->energies) {
		if (energy->name == name) {
			std::cout << "symx error: cannot make_energy with already existing name \"" + name + "\"" << std::endl;
			exit(-1);
		}
	}
	std::unique_ptr<Energy> energy_ptr = std::make_unique<Energy>(name, this->working_directory, data, n_elements, n_items_per_element);
	energy_ptr->set_cse_mode(this->cse_mode);
	this->energies.push_back(std::move(energy_ptr));
	this->energies.back()->working_directory = this->working_directory;
	Element element(n_items_per_element);
	if (labels.size() > 0) {
		element.set_labels(labels);
	}
	energy(*this->energies.back(), element);

	if (!this->energies.back()->is_expression_set()) {
		std::cout << "symx::GlobalEnergy::add_energy() error: Energy not set with name \"" + name + "\"" << std::endl;
		exit(-1);
	}
}
void symx::GlobalEnergy::add_energy(std::string name, const std::vector<int32_t>& arr, const int32_t n_items_per_element, std::function<void(Energy&, Element&)> energy)
{
	this->add_energy(name, l2data_int(arr), l2n_elements_int(arr, n_items_per_element), n_items_per_element, energy);
}
int32_t symx::GlobalEnergy::get_total_n_dofs() const
{
	int32_t sum = 0;
	for (std::function<int32_t()> ndofs : this->dof_ndofs) {
		sum += ndofs();
	}
	return sum;
}
int symx::GlobalEnergy::get_n_dof_sets() const
{
	return (int)this->dof_ndofs.size();
}
std::vector<int32_t> symx::GlobalEnergy::get_dofs_offsets() const
{
	const int n_sets = this->get_n_dof_sets();
	std::vector<int32_t> offsets(n_sets + 1);
	offsets[0] = 0;
	for (int32_t set = 0; set < n_sets; set++) {
		offsets[set + 1] = offsets[set] + this->dof_ndofs[set]();
	}
	return offsets;
}
void symx::GlobalEnergy::_exit_if_not_initialized() const
{
	if (!this->is_initialized) {
		std::cout << "symx::GlobalEnergy error: Need to call GlobalEnergy.compile() before using it." << std::endl;
		exit(-1);
	}
}
void symx::GlobalEnergy::compile(std::string working_directory)
{
	this->compile(working_directory, CompilationOptions());
}
void symx::GlobalEnergy::compile(std::string working_directory, CompilationOptions options)
{
	if (this->get_n_dof_sets() == 0) {
		std::cout << "SymX GlobalEnergy.compile() error: No sets of degrees of freedom declared." << std::endl;
		exit(-1);
	}

	auto msg = [&](const std::string& s) {
		if (options.msg_callback) {
			options.msg_callback(s);
		}
	};

	this->working_directory = working_directory;
	this->n_threads = n_threads;
	if (this->n_threads == -1) {
		this->n_threads = omp_get_max_threads();
	}
	this->n_threads = options.suppress_compiler_output ? this->n_threads : 1;

	this->runtime_differentiation = 0.0;
	this->runtime_codegen = 0.0;
	this->runtime_compilation = 0.0;
	this->symbols_bytes = 0;
	msg("SymX energies:\n");

	const double t0 = omp_get_wtime();
	#pragma omp parallel for num_threads(this->n_threads) schedule(dynamic)
	for (int i = 0; i < (int)this->energies.size(); i++) {
		this->energies[i]->working_directory = this->working_directory;

		const double t0 = omp_get_wtime();
		this->energies[i]->deferred_init(options.force_compilation, options.force_load, options.suppress_compiler_output);
		const double t1 = omp_get_wtime();
		
		#pragma omp atomic
		this->runtime_differentiation += this->energies[i]->runtime_differentiation;

		#pragma omp atomic
		this->runtime_codegen += this->energies[i]->runtime_codegen;

		#pragma omp atomic
		this->runtime_compilation += this->energies[i]->runtime_compilation;

		#pragma omp atomic
		this->symbols_bytes += this->energies[i]->n_bytes_symbols;

		#pragma omp critical
		{
			if (this->energies[i]->was_cached) {
				msg("\t " + two_columns(this->energies[i]->name, "loaded. (" + std::to_string(t1 - t0) + " s)\n", 60));
			}
			else {
				msg("\t " + two_columns(this->energies[i]->name, "compiled. (" + std::to_string(t1 - t0) + " s)\n", 60));
			}
		}
	}
	const double t1 = omp_get_wtime();

	std::string slow_compile_note;
	#ifdef _MSC_VER
	slow_compile_note = "(Note: MSVC takes a lot of time just to load!)";
	#endif

	msg("SymX stats\n");
	msg("\t Total time: " + std::to_string(t1 - t0) + " s.\n");
	msg("\t Differentiation (acc): " + std::to_string(this->runtime_differentiation) + " s.\n");
	msg("\t Code generation (acc): " + std::to_string(this->runtime_codegen) + " s.\n");
	msg("\t Compilation (acc): " + std::to_string(this->runtime_compilation) + " s. " + slow_compile_note + "\n");
	msg("\t Symbol peak memory: " + std::to_string(this->symbols_bytes/1024) + " KB.\n");
	this->is_initialized = true;
}
symx::Assembled symx::GlobalEnergy::evaluate_E()
{
	this->_exit_if_not_initialized();
	this->assembly.reset(this->get_dofs_offsets(), this->n_threads, false, false);
	for (auto& energy : this->energies) {
		energy->evaluate_E(this->assembly);
	}
	for (auto& external_E : this->external_E) {
		external_E(this->assembly);
	}
	this->assembly.E.end();
	return Assembled(this->assembly);
}
symx::Assembled symx::GlobalEnergy::evaluate_E_grad()
{
	this->_exit_if_not_initialized();
	this->assembly.reset(this->get_dofs_offsets(), this->n_threads, false, true);
	for (auto& energy : this->energies) {
		energy->evaluate_E_grad(this->assembly);
	}
	for (auto& external_E_grad : this->external_E_grad) {
		external_E_grad(this->assembly);
	}
	this->assembly.E.end();
	this->assembly.grad.end();
	return Assembled(this->assembly);
}
symx::Assembled symx::GlobalEnergy::evaluate_E_grad_hess()
{
	this->_exit_if_not_initialized();
	this->assembly.reset(this->get_dofs_offsets(), this->n_threads, true, true);
	for (auto& energy : this->energies) {
		energy->evaluate_E_grad_hess(this->assembly);
	}
	for (auto& external_E_grad_hess : this->external_E_grad_hess) {
		external_E_grad_hess(this->assembly);
	}
	this->assembly.E.end();
	this->assembly.grad.end();
	this->assembly.hess.end_insertion(this->n_threads);
	return Assembled(this->assembly);
}
const std::vector<std::vector<int32_t>>& symx::GlobalEnergy::compute_topology()
{
	this->_exit_if_not_initialized();

	// Clear topology
	this->topology.resize(this->get_total_n_dofs());
	for (auto& t : this->topology) {
		t.clear();
	}

	// Compute topology
	this->assembly.reset(this->get_dofs_offsets(), this->n_threads, false, false);
	for (auto& energy : this->energies) {
		energy->add_topology(this->assembly, this->topology);
	}

	return this->topology;
}
void symx::GlobalEnergy::get_dofs(double* u) const
{
	const int n_sets = this->get_n_dof_sets();
	const std::vector<int32_t> offsets = this->get_dofs_offsets();

	for (int set = 0; set < n_sets; set++) {
		const int32_t begin = offsets[set];
		const int32_t end = offsets[set + 1];
		const int32_t length = end - begin;
		if (length > 0) {
			const double* data = this->dof_data[set]();
			memcpy(u + begin, data, length * sizeof(double));
		}
	}
}
void symx::GlobalEnergy::apply_dof_increment(const double* du)
{
	const int n_sets = this->get_n_dof_sets();
	const std::vector<int32_t> offsets = this->get_dofs_offsets();

	for (int set = 0; set < n_sets; set++) {
		const int32_t begin = offsets[set];
		const int32_t end = offsets[set + 1];
		const int32_t length = end - begin;
		
		if (length > 0) {
			double* data = this->dof_data[set]();
			for (int32_t i = 0; i < length; i++) {
				data[i] += du[begin + i];
			}
		}
	}
}
void symx::GlobalEnergy::set_dofs(const double* u)
{
	const int n_sets = this->get_n_dof_sets();
	const std::vector<int32_t> offsets = this->get_dofs_offsets();

	for (int set = 0; set < n_sets; set++) {
		const int32_t begin = offsets[set];
		const int32_t end = offsets[set + 1];
		const int32_t length = end - begin;

		if (length > 0) {
			double* data = this->dof_data[set]();
			memcpy(data, u + begin, length * sizeof(double));
		}
	}
}
void symx::GlobalEnergy::set_project_to_PD(const bool activate)
{
	for (auto& energy : this->energies) {
		energy->set_project_to_PD(activate);
	}
}
void symx::GlobalEnergy::set_cse_mode(CSE mode)
{
	this->cse_mode = mode;
}
void symx::GlobalEnergy::set_check_for_NaNs(const bool activate)
{
	for (auto& energy : this->energies) {
		energy->set_check_for_NaNs(activate);
	}
}

void symx::GlobalEnergy::test_derivatives_with_FD(const double h)
{
	// Get the current dofs
	const int n_dofs = this->get_total_n_dofs();
	Eigen::VectorXd u = Eigen::VectorXd::Zero(n_dofs);
	this->get_dofs(u.data());

	// Evaluate
	Assembled assembled = this->evaluate_E_grad_hess();
	const double E = *assembled.E;
	const Eigen::VectorXd grad = *assembled.grad;

	std::vector<Eigen::Triplet<double>> triplets;
	assembled.hess->to_triplets(triplets);
	Eigen::SparseMatrix<double> hess_sparse(n_dofs, n_dofs);
	hess_sparse.setFromTriplets(triplets.begin(), triplets.end());
	const Eigen::MatrixXd hess = hess_sparse.toDense();

	// Compute the FD gradient and Hessian
	Eigen::VectorXd grad_FD(n_dofs);
	Eigen::MatrixXd hess_FD(n_dofs, n_dofs);
	for (int i = 0; i < n_dofs; i++) {
		u[i] += h;
		this->set_dofs(u.data());
		Assembled A = this->evaluate_E_grad();
		const double E_plus = *A.E;
		const Eigen::VectorXd dE_plus = *A.grad;

		u[i] -= 2.0 * h;
		
		this->set_dofs(u.data());
		A = this->evaluate_E_grad();
		const double E_minus = *A.E;
		const Eigen::VectorXd dE_minus = *A.grad;
		u[i] += h;

		grad_FD[i] = (E_plus - E_minus) / (2.0 * h);
		hess_FD.row(i) = (dE_plus - dE_minus) / (2.0 * h);
	}

	// Compare
	double max_diff_grad = (grad - grad_FD).cwiseAbs().maxCoeff();
	double max_diff_hess = (hess - hess_FD).cwiseAbs().maxCoeff();
	std::cout << "symx::GlobalEnergy::test_derivatives_with_FD(h = " << h << ") max residual (GRAD, HESS): ( " << max_diff_grad << ", " << max_diff_hess << ")" << std::endl;
}

void symx::GlobalEnergy::clear_user_conditional_evaluation()
{
	for (auto& energy : this->energies) {
		energy->disable_user_element_condition();
	}
}

void symx::GlobalEnergy::activate_elements_from_dofs(const std::vector<uint8_t>& activation, const int32_t n_threads)
{
	const std::vector<int32_t> offsets = this->get_dofs_offsets();
	if (activation.size() != offsets.back()) {
		std::cout << "symx::GlobalEnergy::activate_elements() error: Size mismatch." << std::endl;
		exit(-1);
	}

	for (auto& energy : this->energies) {
		energy->enable_user_element_condition();
		const int n_elements = energy->get_n_elements();
		#pragma omp parallel for num_threads(n_threads) schedule(static)
		for (int32_t elem_i = 0; elem_i < n_elements; elem_i++) {
			const ElementInfo element = energy->get_element_info(elem_i);
			const int n_dof_nodes = element.get_n_dof_nodes();

			bool is_element_active = false;
			for (int32_t dof_i = 0; dof_i < n_dof_nodes; dof_i++) {
				const int32_t dof_set = element.get_dof_node_set(dof_i);
				const int32_t dof_node_local = element.get_dof_node(dof_i);
				const int32_t dof_global_begin = offsets[dof_set] + GlobalEnergy::BLOCK_SIZE*dof_node_local;

				for (int32_t i = 0; i < GlobalEnergy::BLOCK_SIZE; i++) {
					if (activation[dof_global_begin + i]) {
						is_element_active = true;
						break;
					}
				}

				if (is_element_active) {
					break;
				}
			}
			energy->set_user_element_condition(elem_i, is_element_active);
		}
	}
}

#include "GlobalEnergy.h"

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
void symx::GlobalEnergy::add_energy(std::string name, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element, std::function<void(Energy&, Element&)> energy)
{
	for (auto& energy : this->energies) {
		if (energy->name == name) {
			std::cout << "symx error: cannot make_energy with already existing name \"" + name + "\"" << std::endl;
			exit(-1);
		}
	}
	std::unique_ptr<Energy> energy_ptr = std::make_unique<Energy>(name, this->working_directory, data, n_elements, n_items_per_element);
	this->energies.push_back(std::move(energy_ptr));
	this->energies.back()->working_directory = this->working_directory;
	Element element(n_items_per_element);
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
std::string symx::GlobalEnergy::compile(std::string working_directory, const int n_threads, bool suppress_compiler_output)
{
	std::string output;
	if (this->get_n_dof_sets() == 0) {
		std::cout << "SymX GlobalEnergy.compile() error: No sets of degrees of freedom declared." << std::endl;
		exit(-1);
	}

	this->working_directory = working_directory;
	this->suppress_compiler_output = suppress_compiler_output;
	this->n_threads = n_threads;
	if (this->n_threads == -1) {
		this->n_threads = omp_get_max_threads();
	}
	this->n_threads = this->suppress_compiler_output ? this->n_threads : 1;

	this->runtime_differentiation = 0.0;
	this->runtime_codegen = 0.0;
	this->runtime_compilation = 0.0;
	this->symbols_bytes = 0;
	output += "SymX energies:\n";

	#pragma omp parallel for num_threads(this->n_threads)
	for (int i = 0; i < (int)this->energies.size(); i++) {
		this->energies[i]->working_directory = this->working_directory;

		const double t0 = omp_get_wtime();
		this->energies[i]->deferred_init(this->dof_data, this->suppress_compiler_output);
		const double t1 = omp_get_wtime();
		
		#pragma omp atomic
		this->runtime_differentiation += this->energies[i]->runtime_differentiation;

		#pragma omp atomic
		this->runtime_codegen += this->energies[i]->runtime_codegen;

		#pragma omp atomic
		this->runtime_compilation += this->energies[i]->runtime_compilation;

		#pragma omp atomic
		this->symbols_bytes += this->energies[i]->n_bytes_symbols;

		if (this->energies[i]->was_cached) {
			output += "\t " + this->energies[i]->name + "... loaded. (" + std::to_string(t1 - t0) + " s)\n";
		}
		else {
			output += "\t " + this->energies[i]->name + "... compiled. (" + std::to_string(t1 - t0) + " s)\n";
		}
	}
	std::string slow_compile_note;
	#ifdef _MSC_VER
	slow_compile_note = "(Note: MSVC takes a lot of time just to load!)";
	#endif

	output += "SymX stats\n";
	output += "\t Differentiation (acc): " + std::to_string(this->runtime_differentiation) + " s.\n";
	output += "\t Code generation (acc): " + std::to_string(this->runtime_codegen) + " s.\n";
	output += "\t Compilation (acc): " + std::to_string(this->runtime_compilation) + " s. " + slow_compile_note + "\n";
	output += "\t Symbol peak memory: " + std::to_string(this->symbols_bytes/1024) + " KB.\n";
	this->is_initialized = true;
	return output;
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
void symx::GlobalEnergy::get_dofs(double* u) const
{
	const int n_sets = this->get_n_dof_sets();
	const std::vector<int32_t> offsets = this->get_dofs_offsets();

	for (int set = 0; set < n_sets; set++) {
		const double* data = this->dof_data[set]();
		const int32_t begin = offsets[set];
		const int32_t end = offsets[set + 1];
		const int32_t length = end - begin;
		memcpy(u + begin, data, length*sizeof(double));
	}
}
void symx::GlobalEnergy::apply_dof_increment(const double* du)
{
	const int n_sets = this->get_n_dof_sets();
	const std::vector<int32_t> offsets = this->get_dofs_offsets();

	for (int set = 0; set < n_sets; set++) {
		double* data = this->dof_data[set]();
		const int32_t begin = offsets[set];
		const int32_t end = offsets[set + 1];
		const int32_t length = end - begin;

		for (int32_t i = 0; i < length; i++) {
			data[i] += du[begin + i];
		}
	}
}
void symx::GlobalEnergy::set_dofs(const double* u)
{
	const int n_sets = this->get_n_dof_sets();
	const std::vector<int32_t> offsets = this->get_dofs_offsets();

	for (int set = 0; set < n_sets; set++) {
		double* data = this->dof_data[set]();
		const int32_t begin = offsets[set];
		const int32_t end = offsets[set + 1];
		const int32_t length = end - begin;
		memcpy(data, u + begin, length * sizeof(double));
	}
}
void symx::GlobalEnergy::set_project_to_PD(const bool activate)
{
	for (auto& energy : this->energies) {
		energy->project_to_PD = activate;
	}
}

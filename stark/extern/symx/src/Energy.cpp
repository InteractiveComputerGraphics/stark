#include "Energy.h"

#include "project_to_PD.h"

symx::Energy::Energy(std::string name, std::string working_directory, std::function<const int32_t*()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element)
{
	this->_init(name, working_directory, data, n_elements, n_items_per_element);
}
symx::Energy::Energy(std::string name, std::string working_directory, std::vector<int32_t>& arr, const int32_t n_items_per_element)
{
	this->_init(name, working_directory, l2data_int(arr), l2n_elements_int(arr, n_items_per_element), n_items_per_element);
}
void symx::Energy::set(const Scalar& expr)
{
	this->expr = std::make_unique<Scalar>(expr);
}
void symx::Energy::set_with_condition(const Scalar& expr, const Scalar& cond)
{
	this->expr = std::make_unique<Scalar>(expr);
	this->cond = std::make_unique<Scalar>(cond);
	this->compiled_derivatives.update_connectivity(l2data_int(this->connectivity_after_condition), l2n_elements_int(this->connectivity_after_condition, this->n_items_per_element));
	this->compiled_derivatives_d.update_connectivity(l2data_int(this->connectivity_after_condition), l2n_elements_int(this->connectivity_after_condition, this->n_items_per_element));
	this->has_condition = true;
}
void symx::Energy::deferred_init(std::vector<std::function<double* ()>> dof_arrays, const bool force_compilation, const bool suppress_compiler_output)
{
	if (this->expr == nullptr) {
		std::cout << "symx::Energy error: Expression not set in energy " << this->name << std::endl;
		exit(-1);
	}
	if (this->dof_symbols.size() == 0) {
		std::cout << "symx::Energy error: Energy does not have degrees of freedom (DoF) " << this->name << std::endl;
		exit(-1);
	}

	this->has_branches = this->expr->has_branch();
	if (this->has_branches) {
		this->compiled_derivatives_d.init(name, this->working_directory, *this->expr.get(), this->dof_symbols, /*TODO:*/ 3, force_compilation, suppress_compiler_output);
		this->was_cached = this->compiled_derivatives_d.was_cached;
		this->runtime_codegen = this->compiled_derivatives_d.runtime_codegen;
		this->runtime_compilation = this->compiled_derivatives_d.runtime_compilation;
		this->runtime_differentiation = this->compiled_derivatives_d.runtime_differentiation;
	}
	else {
		this->compiled_derivatives.init(name, this->working_directory, *this->expr.get(), this->dof_symbols, /*TODO:*/ 3, force_compilation, suppress_compiler_output);
		this->was_cached = this->compiled_derivatives.was_cached;
		this->runtime_codegen = this->compiled_derivatives.runtime_codegen;
		this->runtime_compilation = this->compiled_derivatives.runtime_compilation;
		this->runtime_differentiation = this->compiled_derivatives.runtime_differentiation;
	}
	this->n_dofs = (int)this->dof_symbols.size();

	if (this->has_condition) {

		if (force_compilation) {
			this->compiled_condition.compile({ *this->cond }, this->name + "_cond", this->working_directory, this->cond->get_checksum(), suppress_compiler_output);
		}
		else {
			this->compiled_condition.try_load_otherwise_compile({ *this->cond }, this->name + "_cond", this->working_directory, this->cond->get_checksum(), suppress_compiler_output);
		}

		this->runtime_codegen += this->compiled_condition.compilation.runtime_codegen;
		this->runtime_compilation += this->compiled_condition.compilation.runtime_compilation;
	}

	// Clear symbolic allocations
	this->n_bytes_symbols = this->sws.expressions.expressions.size() * sizeof(symx::Expr);
	this->dof_symbols.clear();
	this->sws = SymbolicWorkSpace();
	this->expr = nullptr;
}
void symx::Energy::activate(const bool activate)
{
	this->is_active = activate;
}
void symx::Energy::disable_check_for_duplicate_dofs()
{
	this->check_for_duplicate_dofs = false;
}
bool symx::Energy::is_expression_set() const
{
	return this->expr != nullptr;
}

symx::Scalar symx::Energy::make_scalar(std::function<const double*()> data, const std::string name)
{
	Scalar scalar = this->sws.make_scalar(this->_get_symbol_name(name));
	this->compiled_derivatives.set_scalar(scalar, data);
	this->compiled_derivatives_d.set_scalar(scalar, data);
	this->compiled_condition.set_scalar(scalar, data);
	return scalar;
}
symx::Scalar symx::Energy::make_scalar(const double& scalar, const std::string name)
{
	return this->make_scalar(l2data_double(scalar), name);
}
symx::Scalar symx::Energy::make_scalar(std::function<const double*()> data, std::function<int32_t()> size, const Index& idx, const std::string name)
{
	Scalar scalar = this->sws.make_scalar(this->_get_symbol_name(name));
	this->compiled_derivatives.set_scalars({ scalar }, { idx }, data, size);
	this->compiled_derivatives_d.set_scalars({ scalar }, { idx }, data, size);
	this->compiled_condition.set_scalars({ scalar }, { idx }, data, size);
	return scalar;
}
symx::Vector symx::Energy::make_vector(std::function<const double* ()> data, const int32_t stride, const std::string name)
{
	Vector vector = this->sws.make_vector(this->_get_symbol_name(name), stride);
	this->compiled_derivatives.set_vector(vector, data);
	this->compiled_derivatives_d.set_vector(vector, data);
	this->compiled_condition.set_vector(vector, data);
	return vector;
}
symx::Vector symx::Energy::make_vector(std::function<const double*()> data, std::function<int32_t()> size, const int32_t stride, const Index& idx, const std::string name)
{
	Vector vector = this->sws.make_vector(this->_get_symbol_name(name), stride);
	this->compiled_derivatives.set_vectors({ vector }, { idx }, data, size);
	this->compiled_derivatives_d.set_vectors({ vector }, { idx }, data, size);
	this->compiled_condition.set_vectors({ vector }, { idx }, data, size);
	return vector;
}
symx::Vector symx::Energy::make_dof_vector(const DoF& dof, std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const Index& idx, const std::string name)
{
	// Check that this dof has not been created yet
	if (this->check_for_duplicate_dofs) {
		for (const auto& dof_block : this->dof_block_global_index) {
			if (dof_block.dof_set == dof.idx && dof_block.conn_idx == idx.idx) {
				std::cout << "symx error: Energy::make_dof_vector() tried to create a symbol for a DoF that already exists for energy " + this->name << std::endl;
				exit(-1);
			}
		}
	}

	// Create the vector and declare the DoF
	symx::Vector vector = this->make_vector(data, size, stride, idx, name);
	assert(vector.size() == 3);
	this->dof_block_global_index.push_back({ dof.idx, idx.idx });
	for (int i = 0; i < vector.size(); i++) {
		this->dof_symbols.push_back(vector[i]);
	}
	return vector;
}
std::vector<symx::Vector> symx::Energy::make_vectors(std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const std::vector<Index>& indices, const std::string name)
{
	const std::string name_ = this->_get_symbol_name(name);
	std::vector<Vector> vectors;
	for (size_t i = 0; i < indices.size(); i++) {
		vectors.push_back(this->make_vector(data, size, stride, indices[i], name_ + std::to_string(i)));
	}
	return vectors;
}
std::vector<symx::Vector> symx::Energy::make_dof_vectors(const DoF& dof, std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const std::vector<Index>& indices, const std::string name)
{
	const std::string name_ = this->_get_symbol_name(name);
	std::vector<Vector> vectors;
	for (size_t i = 0; i < indices.size(); i++) {
		vectors.push_back(this->make_dof_vector(dof, data, size, stride, indices[i], name_ + std::to_string(i)));
	}
	return vectors;
}
symx::Matrix symx::Energy::make_matrix(std::function<const double* ()> data, std::function<int32_t()> size, const std::array<int, 2> shape, const Index& idx, const std::string name)
{
	Matrix matrix = this->sws.make_matrix(this->_get_symbol_name(name), shape);
	this->compiled_derivatives.set_matrices({ matrix }, { idx }, data, size);
	this->compiled_derivatives_d.set_matrices({ matrix }, { idx }, data, size);
	this->compiled_condition.set_matrices({ matrix }, { idx }, data, size);
	return matrix;
}

symx::Scalar symx::Energy::make_zero()
{
	return this->sws.get_zero();
}
symx::Scalar symx::Energy::make_one()
{
	return this->sws.get_one();
}
symx::Vector symx::Energy::make_zero_vector(const int32_t size)
{
	return this->sws.get_zero_vector(size);
}
symx::Matrix symx::Energy::make_zero_matrix(const std::array<int32_t, 2> shape)
{
	return this->sws.get_zero_matrix(shape);
}
symx::Matrix symx::Energy::make_identity_matrix(const int32_t size)
{
	return this->sws.get_identity_matrix(size);
}



void symx::Energy::evaluate_E(Assembly& assembly, const bool runtime_NaN_check)
{
	if (!this->is_active) { return; }
	if (this->has_condition) {
		this->_update_connectivity_conditionally(assembly.n_threads);
	}

	auto assembly_f = [&assembly](const int32_t iteration, const int32_t thread_id, const int32_t* conn, const double* sol) 
	{
		assembly.E.get(thread_id) += sol[0];
	};

	if (this->has_branches) {
		this->compiled_derivatives_d.run_E(assembly.n_threads, assembly_f, runtime_NaN_check);
		assembly.compiled_runtime = this->compiled_derivatives_d.E.thread_compiled_timing.end();
	}
	else {
		this->compiled_derivatives.run_E(assembly.n_threads, assembly_f, runtime_NaN_check);
		assembly.compiled_runtime = this->compiled_derivatives.E.thread_compiled_timing.end();
	}
}
void symx::Energy::evaluate_E_grad(Assembly& assembly, const bool runtime_NaN_check)
{
	if (!this->is_active) { return; }
	if (this->has_condition) {
		this->_update_connectivity_conditionally(assembly.n_threads);
	}

	auto assembly_f = [&](const int32_t iteration, const int32_t thread_id, const int32_t* conn, const double* sol)
	{
		assembly.E.get(thread_id) += sol[0];

		Eigen::VectorXd& grad = assembly.grad.get(thread_id);
		const int n_blocks = this->n_dofs / assembly.BLOCK_SIZE;
		const double* grad_loc = sol + 1;
		for (int block_i = 0; block_i < n_blocks; block_i++) {
			DoFBlockGlobalIndex& map = this->dof_block_global_index[block_i];
			const int begin_glob = assembly.dof_set_offsets[map.dof_set] + conn[map.conn_idx] * assembly.BLOCK_SIZE;

			const int begin_loc = assembly.BLOCK_SIZE * block_i;
			for (int i = 0; i < assembly.BLOCK_SIZE; i++) {
				grad[begin_glob + i] += grad_loc[begin_loc + i];
			}
		}
	};

	if (this->has_branches) {
		this->compiled_derivatives_d.run_E_grad(assembly.n_threads, assembly_f, runtime_NaN_check);
		assembly.compiled_runtime = this->compiled_derivatives_d.dE.thread_compiled_timing.end();
	}
	else {
		this->compiled_derivatives.run_E_grad(assembly.n_threads, assembly_f, runtime_NaN_check);
		assembly.compiled_runtime = this->compiled_derivatives.dE.thread_compiled_timing.end();
	}
}
void symx::Energy::evaluate_E_grad_hess(Assembly& assembly, const bool runtime_NaN_check)
{
	if (!this->is_active) { return; }
	if (this->has_condition) {
		this->_update_connectivity_conditionally(assembly.n_threads);
	}

	auto assembly_f = [&](const int32_t iteration, const int32_t thread_id, const int32_t* conn, double* sol)
	{
		assembly.E.get(thread_id) += sol[0];

		Eigen::VectorXd& grad = assembly.grad.get(thread_id);
		const int n_blocks = this->n_dofs / assembly.BLOCK_SIZE;
		double* grad_loc = sol + 1;
		for (int block_i = 0; block_i < n_blocks; block_i++) {
			DoFBlockGlobalIndex& map = this->dof_block_global_index[block_i];
			const int begin_glob = assembly.dof_set_offsets[map.dof_set] + conn[map.conn_idx] * assembly.BLOCK_SIZE;

			const int begin_loc = assembly.BLOCK_SIZE * block_i;
			for (int i = 0; i < assembly.BLOCK_SIZE; i++) {
				grad[begin_glob + i] += grad_loc[begin_loc + i];
			}
		}

		double* hess_loc = grad_loc + this->n_dofs;
		if (this->project_to_PD) {
			project_to_PD_from_pointer(hess_loc, this->n_dofs, /*debug_print_lowest = */ false);
		}

		for (int block_i = 0; block_i < n_blocks; block_i++) {
			DoFBlockGlobalIndex& map = this->dof_block_global_index[block_i];
			const int begin_glob_i = assembly.dof_set_offsets[map.dof_set] + conn[map.conn_idx] * assembly.BLOCK_SIZE;

			const int begin_loc_i = assembly.BLOCK_SIZE * block_i;
			for (int block_j = 0; block_j < n_blocks; block_j++) {
				DoFBlockGlobalIndex& map = this->dof_block_global_index[block_j];
				const int begin_glob_j = assembly.dof_set_offsets[map.dof_set] + conn[map.conn_idx] * assembly.BLOCK_SIZE;

				const int begin_loc_j = assembly.BLOCK_SIZE * block_j;
				std::array<double, Assembly::BLOCK_SIZE* Assembly::BLOCK_SIZE> data;
				for (int i = 0; i < assembly.BLOCK_SIZE; i++) {
					for (int j = 0; j < assembly.BLOCK_SIZE; j++) {
						data[assembly.BLOCK_SIZE * i + j] = hess_loc[(begin_loc_i + i) * this->n_dofs + (begin_loc_j + j)];
					}
				}
				assembly.hess.add_block_from_ptr<double, bsm::Ordering::RowMajor>(begin_glob_i, begin_glob_j, data.data());
			}
		}
	};

	if (this->has_branches) {
		this->compiled_derivatives_d.run_E_grad_hess(assembly.n_threads, assembly_f, runtime_NaN_check);
		assembly.compiled_runtime += this->compiled_derivatives_d.hE.thread_compiled_timing.end();
	}
	else {
		this->compiled_derivatives.run_E_grad_hess(assembly.n_threads, assembly_f, runtime_NaN_check);
		assembly.compiled_runtime += this->compiled_derivatives.hE.thread_compiled_timing.end();
	}
}

void symx::Energy::_update_connectivity_conditionally(const int n_threads)
{
	this->connectivity_after_condition.clear();
	const int n_original_elements = this->compiled_condition.connectivity_n_elements();
	if (n_original_elements == 0) { return; }
	this->positive_conditions.resize(n_original_elements);
	this->compiled_condition.run(n_threads,
		[this](const int32_t iteration, const int32_t thread_id, const int32_t* conn, const double* sol)
		{
			this->positive_conditions[iteration] = static_cast<uint8_t>(sol[0] > 0);
		}
	);

	const int32_t* connectivity_data = this->compiled_condition.connectivity_data();
	for (int i = 0; i < n_original_elements; i++) {
		if (this->positive_conditions[i]) {
			for (int j = 0; j < n_items_per_element; j++) {
				this->connectivity_after_condition.push_back(connectivity_data[i*n_items_per_element + j]);
			}
		}
	}
}
std::string symx::Energy::_get_symbol_name(std::string user_name)
{
	if (user_name == "") {
		this->name_conter++;
		return "_v" + std::to_string(this->name_conter - 1);
	}
	else {
		return user_name;
	}
}
void symx::Energy::_init(std::string name, std::string working_directory, std::function<const int32_t* ()> data, std::function<int32_t()> n_elements, const int32_t n_items_per_element)
{
	this->name = name;
	this->working_directory = working_directory;
	this->n_items_per_element = n_items_per_element;
	this->compiled_derivatives.set_connectivity(data, n_elements, n_items_per_element);
	this->compiled_derivatives_d.set_connectivity(data, n_elements, n_items_per_element);
	this->compiled_condition.set_connectivity(data, n_elements, n_items_per_element);
}

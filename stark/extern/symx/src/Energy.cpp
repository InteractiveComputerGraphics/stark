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
}
void symx::Energy::set_project_to_PD(const bool project_to_PD)
{
	this->project_to_PD = project_to_PD;
}
void symx::Energy::deferred_init(const bool force_compilation, const bool force_load, const bool suppress_compiler_output)
{
	// Check that everything is set
	if (this->expr == nullptr) {
		std::cout << "symx::Energy error: Expression not set in energy " << this->name << std::endl;
		exit(-1);
	}
	if (this->dof_symbols.size() == 0) {
		std::cout << "symx::Energy error: Energy does not have degrees of freedom (DoF) " << this->name << std::endl;
		exit(-1);
	}

	// Shorthands
	constexpr int BLOCK_SIZE = 3;
	using COMPILED_FLOAT = double;
	this->n_dofs = (int)this->dof_symbols.size();

	// Expresion Compilation
	//// Generate unique ID
	std::string id_string;
	if (force_load) {
		id_string = "FORCE_LOAD";
	}
	else {
		id_string = this->expr->get_checksum(); // Expression id
		id_string += (this->has_symbolic_element_condition()) ? this->cond->get_checksum() : ""; // Condition id
		id_string += get_float_type_as_string<COMPILED_FLOAT>(); // Float type

		// All symbols
		for (const std::string& s : this->sws.get_expression_graph().get_symbol_names()) {
			id_string += s;
		}
	}

	//// Try to load
	this->was_cached = false;
	if (!force_compilation) {
		if (this->E.load_if_cached(this->name + "_E", this->working_directory, id_string)) {
			if (this->dE.load_if_cached(this->name + "_dE", this->working_directory, id_string)) {
				if (this->hE.load_if_cached(this->name + "_hE", this->working_directory, id_string)) {
					this->was_cached = true;
				}
			}
		}
	}

	//// Compile
	if (!this->was_cached) {

		// Differentiation
		const double t0 = omp_get_wtime();
		std::vector<Scalar> e = { *this->expr };
		std::vector<Scalar> e_grad = symx::value_gradient(*this->expr, this->dof_symbols);
		std::vector<Scalar> e_grad_hess = symx::value_gradient_hessian(*this->expr, this->dof_symbols);
		const double t1 = omp_get_wtime();
		this->runtime_differentiation = t1 - t0;

		// Compilation
		this->E.compile(e, this->name + "_E", this->working_directory, id_string, suppress_compiler_output);
		this->dE.compile(e_grad, this->name + "_dE", this->working_directory, id_string, suppress_compiler_output);
		this->hE.compile(e_grad_hess, this->name + "_hE", this->working_directory, id_string, suppress_compiler_output);
		this->runtime_codegen = this->E.compilation.runtime_codegen + this->dE.compilation.runtime_codegen + hE.compilation.runtime_codegen;
		this->runtime_compilation = this->E.compilation.runtime_compilation + this->dE.compilation.runtime_compilation + hE.compilation.runtime_compilation;
	}

	// Condition Compilation
	if (this->has_symbolic_element_condition()) {

		// Compilation
		if (force_compilation) {
			std::string id = this->cond->get_checksum() + id_string;
			this->C.compile({ *this->cond }, this->name + "_cond", this->working_directory, id, suppress_compiler_output);
		}
		else {
			std::string id = (force_load) ? "FORCE_LOAD" : this->cond->get_checksum() + id_string;
			this->C.try_load_otherwise_compile({ *this->cond }, this->name + "_cond", this->working_directory, id, suppress_compiler_output);
		}
		this->runtime_codegen += this->C.compilation.runtime_codegen;
		this->runtime_compilation += this->C.compilation.runtime_compilation;
	}

	// Clear symbolic allocations
	this->n_bytes_symbols = this->sws.get_expression_graph().size() * sizeof(symx::Expr);
	this->dof_symbols.clear();
	this->expr = nullptr;
	this->cond = nullptr;
}
void symx::Energy::activate(const bool activate)
{
	this->is_active = activate;
}
void symx::Energy::disable_check_for_duplicate_dofs()
{
	this->check_for_duplicate_dofs = false;
}
void symx::Energy::set_cse_mode(CSE mode)
{
	this->sws.set_cse_mode(mode);
}
void symx::Energy::set_check_for_NaNs(const bool check)
{
	this->_for_each_compiled([&](CompiledInLoop<double>& compiled) { compiled.set_check_for_NaNs(check); });
}
int symx::Energy::get_n_elements() const
{
	return this->E.connectivity_n_elements();
}
int symx::Energy::get_connectivity_stride() const
{
	return this->E.get_connectivity_stride();
}
void symx::Energy::set_user_element_condition(const int32_t element_idx, const bool activate)
{
	assert(element_idx >= 0 && element_idx < this->get_n_elements() && element_idx < this->user_defined_element_condition.size());
	this->user_defined_element_condition[element_idx] = static_cast<uint8_t>(activate);
}
const int32_t* symx::Energy::get_connectivity_elem(const int32_t element_idx) const
{
	assert(element_idx >= 0 && element_idx < this->get_n_elements());
	return this->E.get_connectivity_elem(element_idx);
}
bool symx::Energy::has_symbolic_element_condition() const
{
	return this->C.is_valid();
}
bool symx::Energy::has_user_defined_element_condition() const
{
	return this->user_defined_element_condition.size() > 0;
}
bool symx::Energy::is_expression_set() const
{
	return this->expr != nullptr;
}
symx::ElementInfo symx::Energy::get_element_info(const int32_t element_idx) const
{
	if (element_idx < 0 || element_idx >= this->get_n_elements()) {
		std::cout << "symx error: Energy::get_element_info(" + std::to_string(element_idx) + ") element index out of range for energy " + this->name + " with number of elements " + std::to_string(this->get_n_elements()) << std::endl;
		exit(-1);
	}

	return ElementInfo(this->get_connectivity_stride(), this->get_connectivity_elem(element_idx), element_idx, this->dof_in_conn);
}

symx::Scalar symx::Energy::make_scalar(std::function<const double*()> data, const std::string name)
{
	Scalar scalar = this->sws.make_scalar(this->_get_symbol_name(name));
	this->_for_each_compiled([&](CompiledInLoop<double>& compiled) { compiled.set_scalar(scalar, data); });
	return scalar;
}
symx::Scalar symx::Energy::make_scalar(const double& scalar, const std::string name)
{
	return this->make_scalar(l2data_double(scalar), name);
}
symx::Scalar symx::Energy::make_scalar(std::function<const double*()> data, std::function<int32_t()> size, const Index& idx, const std::string name)
{
	Scalar scalar = this->sws.make_scalar(this->_get_symbol_name(name));
	this->_for_each_compiled([&](CompiledInLoop<double>& compiled) { compiled.set_scalars({ scalar }, { idx }, data, size); });
	return scalar;
}
symx::Vector symx::Energy::make_vector(std::function<const double* ()> data, const int32_t stride, const std::string name)
{
	Vector vector = this->sws.make_vector(this->_get_symbol_name(name), stride);
	this->_for_each_compiled([&](CompiledInLoop<double>& compiled) { compiled.set_vector(vector, data); });
	return vector;
}
symx::Vector symx::Energy::make_vector(std::function<const double*()> data, std::function<int32_t()> size, const int32_t stride, const Index& idx, const std::string name)
{
	Vector vector = this->sws.make_vector(this->_get_symbol_name(name), stride);
	this->_for_each_compiled([&](CompiledInLoop<double>& compiled) { compiled.set_vectors({ vector }, { idx }, data, size); });
	return vector;
}
symx::Vector symx::Energy::make_dof_vector(const DoF& dof, std::function<const double* ()> data, std::function<int32_t()> size, const int32_t stride, const Index& idx, const std::string name)
{
	// Check that this dof has not been created yet
	if (this->check_for_duplicate_dofs) {
		for (const auto& dof_block : this->dof_in_conn) {
			if (dof_block.dof_set == dof.idx && dof_block.conn_idx == idx.idx) {
				std::cout << "symx error: Energy::make_dof_vector() tried to create a symbol for a DoF that already exists for energy " + this->name << std::endl;
				exit(-1);
			}
		}
	}

	// Create the vector and declare the DoF
	symx::Vector vector = this->make_vector(data, size, stride, idx, name);
	assert(vector.size() == 3);
	this->dof_in_conn.push_back({ dof.idx, idx.idx });
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
void symx::Energy::disable_user_element_condition()
{
	this->user_defined_element_condition.resize(0);
}
void symx::Energy::enable_user_element_condition()
{
	this->user_defined_element_condition.resize(this->get_n_elements());
	std::fill(this->user_defined_element_condition.begin(), this->user_defined_element_condition.end(), static_cast<uint8_t>(true));
}
symx::Matrix symx::Energy::make_matrix(std::function<const double* ()> data, std::function<int32_t()> size, const std::array<int, 2> shape, const Index& idx, const std::string name)
{
	Matrix matrix = this->sws.make_matrix(this->_get_symbol_name(name), shape);
	this->_for_each_compiled([&](CompiledInLoop<double>& compiled) { compiled.set_matrices({ matrix }, { idx }, data, size); });
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



void symx::Energy::evaluate_E(Assembly& assembly)
{
	if (!this->is_active) { return; }

	this->E.run(
		assembly.n_threads, 
		[&assembly](const int32_t element_idx, const int32_t thread_id, const int32_t* conn, const double* sol)
		{
			assembly.E.get(thread_id) += sol[0];
		},
		this->_get_updated_conditional_evaluation(assembly.n_threads)
	);
}
void symx::Energy::evaluate_E_grad(Assembly& assembly)
{
	if (!this->is_active) { return; }

	this->dE.run(
		assembly.n_threads, 
		[this, &assembly](const int32_t element_idx, const int32_t thread_id, const int32_t* conn, const double* sol)
		{
			assembly.E.get(thread_id) += sol[0];

			Eigen::VectorXd& grad = assembly.grad.get(thread_id);
			const int n_blocks = this->n_dofs / assembly.BLOCK_SIZE;
			const double* grad_loc = sol + 1;
			for (int block_i = 0; block_i < n_blocks; block_i++) {
				const DoFInConn& map = this->dof_in_conn[block_i];
				const int begin_glob = assembly.dof_set_offsets[map.dof_set] + conn[map.conn_idx] * assembly.BLOCK_SIZE;

				const int begin_loc = assembly.BLOCK_SIZE * block_i;
				for (int i = 0; i < assembly.BLOCK_SIZE; i++) {
					grad[begin_glob + i] += grad_loc[begin_loc + i];
				}
			}
		},
		this->_get_updated_conditional_evaluation(assembly.n_threads)
	);
}
void symx::Energy::evaluate_E_grad_hess(Assembly& assembly)
{
	if (!this->is_active) { return; }

	this->hE.run(
		assembly.n_threads,
		[this, &assembly](const int32_t element_idx, const int32_t thread_id, const int32_t* conn, double* sol)
		{
			assembly.E.get(thread_id) += sol[0];

			Eigen::VectorXd& grad = assembly.grad.get(thread_id);
			const int n_blocks = this->n_dofs / assembly.BLOCK_SIZE;
			double* grad_loc = sol + 1;
			for (int block_i = 0; block_i < n_blocks; block_i++) {
				const DoFInConn& map = this->dof_in_conn[block_i];
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
				const DoFInConn& map = this->dof_in_conn[block_i];
				const int begin_glob_i = assembly.dof_set_offsets[map.dof_set] + conn[map.conn_idx] * assembly.BLOCK_SIZE;

				const int begin_loc_i = assembly.BLOCK_SIZE * block_i;
				for (int block_j = 0; block_j < n_blocks; block_j++) {
					const DoFInConn& map = this->dof_in_conn[block_j];
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
		},
		this->_get_updated_conditional_evaluation(assembly.n_threads)
	);
}

void symx::Energy::add_topology(Assembly& assembly, std::vector<std::vector<int32_t>>& topology)
{
	if (!this->is_active) { return; }
	
	this->E.run(
		/* n_threads = */ 1,
		[this, &assembly, &topology](const int32_t element_idx, const int32_t thread_id, const int32_t* conn, const double* sol)
		{
			auto append_if_not_present = [](std::vector<int32_t>& vec, int32_t val)
				{
					if (std::find(vec.begin(), vec.end(), val) == vec.end()) {
						vec.push_back(val);
					}
				};

			const int n_blocks = this->n_dofs / assembly.BLOCK_SIZE;
			for (int block_i = 0; block_i < n_blocks; block_i++) {
				const DoFInConn& map_i = this->dof_in_conn[block_i];
				const int begin_glob_i = assembly.dof_set_offsets[map_i.dof_set] + conn[map_i.conn_idx] * assembly.BLOCK_SIZE;
				const int glob_node_i = begin_glob_i / assembly.BLOCK_SIZE;

				for (int block_j = 0; block_j < n_blocks; block_j++) {
					const DoFInConn& map_j = this->dof_in_conn[block_j];
					const int begin_glob_j = assembly.dof_set_offsets[map_j.dof_set] + conn[map_j.conn_idx] * assembly.BLOCK_SIZE;
					const int glob_node_j = begin_glob_j / assembly.BLOCK_SIZE;

					append_if_not_present(topology[glob_node_i], glob_node_j);
					append_if_not_present(topology[glob_node_j], glob_node_i);
				}
			}
		},
		this->_get_updated_conditional_evaluation(assembly.n_threads)
	);
}
const std::vector<uint8_t>& symx::Energy::_evaluate_element_conditions(const int n_threads)
{
	this->has_element_positive_condition.resize(this->get_n_elements());
	this->C.run(n_threads,
		[this](const int32_t element_idx, const int32_t thread_id, const int32_t* conn, const double* sol)
		{
			this->has_element_positive_condition[element_idx] = static_cast<uint8_t>(sol[0] > 0);
		}
	);
	return this->has_element_positive_condition;
}
std::function<bool(const int32_t element_idx, const int32_t thread_id, const int32_t* connectivity)> symx::Energy::_get_updated_conditional_evaluation(const int n_threads)
{
	if (!this->has_symbolic_element_condition() && !this->has_user_defined_element_condition()) {
		return nullptr;
	}

	else {
		// User defined condition
		if (this->has_user_defined_element_condition()) {
			this->has_element_positive_condition = this->user_defined_element_condition;
		}
		else {
			this->has_element_positive_condition.resize(this->get_n_elements());
		}

		// Symbolic condition
		if (this->has_symbolic_element_condition()) {
			this->C.run(n_threads,
				[this](const int32_t element_idx, const int32_t thread_id, const int32_t* conn, const double* sol)
				{
					this->has_element_positive_condition[element_idx] = static_cast<uint8_t>(sol[0] > 0);
				}
			);
		}

		// Return a lambda that reads the pre-evaluated conditions
		auto conditional_evaluation = [this](const int32_t element_idx, const int32_t thread_id, const int32_t* conn)
			{
				return static_cast<bool>(this->has_element_positive_condition[element_idx]);
			};

		return conditional_evaluation;
	}
}
void symx::Energy::_for_each_compiled(const std::function<void(CompiledInLoop<double>&)> f)
{
	f(this->E);
	f(this->dE);
	f(this->hE);
	f(this->C);
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
	this->_for_each_compiled([&](CompiledInLoop<double>& compiled) { compiled.set_connectivity(data, n_elements, n_items_per_element); });
}

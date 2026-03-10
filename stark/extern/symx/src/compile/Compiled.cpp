#include "Compiled.h"
#include <stdexcept>
#include <iostream>

using namespace symx;

template<typename FLOAT>
Compiled<FLOAT>::Compiled(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id)
{
	this->try_load_otherwise_compile(expr, name, folder, cache_id);
}

template<typename FLOAT>
void Compiled<FLOAT>::compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id)
{
	this->name = name;
	this->compilation.template compile<FLOAT>(expr, name, folder, cache_id);
	this->_resize_buffers();
}

template<typename FLOAT>
bool Compiled<FLOAT>::load_if_cached(const std::string& name, const std::string& folder, const std::string& cache_id)
{
	bool success = this->compilation.template load_if_cached<FLOAT>(name, folder, cache_id);
	if (success) {
		this->name = name;
		this->_resize_buffers();
	}
	return success;
}

template<typename FLOAT>
void Compiled<FLOAT>::try_load_otherwise_compile(const std::vector<Scalar>& expr, const std::string& name, const std::string& folder, const std::string& cache_id)
{
	this->name = name;
	this->compilation.template try_load_otherwise_compile<FLOAT>(expr, name, folder, cache_id);
	this->_resize_buffers();
}

template<typename FLOAT>
bool Compiled<FLOAT>::is_valid() const
{
	return this->compilation.is_valid();
}

template <typename FLOAT>
Compilation::Info Compiled<FLOAT>::get_compilation_info() const
{
    return this->compilation.get_info();
}

template<typename FLOAT>
bool Compiled<FLOAT>::was_cached() const
{
	return this->compilation.was_cached();
}

template<typename FLOAT>
void Compiled<FLOAT>::set_n_threads(int32_t n_threads)
{
	assert(n_threads > 0 && "Number of threads must be positive");
	
	this->n_threads = n_threads;
	this->thread_input.resize(n_threads);
	this->thread_output.resize(n_threads);
	
	// Resize all thread buffers
	for (int32_t i = 0; i < n_threads; i++) {
		this->_resize_buffers(i);
	}
}

template<typename FLOAT>
int32_t Compiled<FLOAT>::get_n_threads() const
{
	return this->n_threads;
}

template <typename FLOAT>
void symx::Compiled<FLOAT>::set(const Scalar &scalar, const FLOAT &v)
{
	this->set(scalar, v, 0);
}

template <typename FLOAT>
void symx::Compiled<FLOAT>::set(const Scalar &scalar, const FLOAT &v, int32_t thread_id)
{
	assert(thread_id >= 0 && thread_id < this->n_threads && "Thread ID is out of range");
	this->thread_input[thread_id][scalar.expr.a] = v;
}

template <typename FLOAT>
void symx::Compiled<FLOAT>::set(const Matrix &matrix, const FLOAT *v)
{
	this->set(matrix, v, 0);
}

template <typename FLOAT>
void symx::Compiled<FLOAT>::set(const Matrix &matrix, const FLOAT *v, int32_t thread_id)
{
	const int32_t nrows = matrix.rows();
	const int32_t ncols = matrix.cols();
	for (int32_t i = 0; i < nrows; i++) {
		for (int32_t j = 0; j < ncols; j++) {
			this->set(matrix(i, j), v[i*ncols + j], thread_id);
		}
	}
}

template<typename FLOAT>
View<FLOAT> Compiled<FLOAT>::run()
{
	return this->run(0);
}

template<typename FLOAT>
View<FLOAT> Compiled<FLOAT>::run(int32_t thread_id)
{
	assert(thread_id >= 0 && thread_id < this->n_threads && "Thread ID is out of range");

	if (!this->is_valid()) {
		throw std::runtime_error("Cannot run invalid compilation for '" + this->name + "'");
	}
	
	// Check for NaN in the input
	if (get_check_mode_ON()) {
		const FloatType underlying_type = get_underlying_type(this->compilation.get_compiled_type());
		if (underlying_type == FloatType::Double) {
			const double* input = reinterpret_cast<const double*>(this->thread_input[thread_id].data());
			const int32_t size = (int32_t)(this->thread_input[thread_id].size()*get_n_items_in_simd<FLOAT>());

			for (int32_t i = 0; i < size; i++) {
				if (std::isnan(input[i])) {
					throw std::runtime_error("NaN detected in input #" + std::to_string(i) + " of (compiled function '" + this->name + "')");
				}
			}
		}
		else if (underlying_type == FloatType::Float) {
			const float* input = reinterpret_cast<const float*>(this->thread_input[thread_id].data());
			const int32_t size = (int32_t)(this->thread_input[thread_id].size()*get_n_items_in_simd<FLOAT>());

			for (int32_t i = 0; i < size; i++) {
				if (std::isnan(input[i])) {
					throw std::runtime_error("NaN detected in input #" + std::to_string(i) + " of (compiled function '" + this->name + "')");
				}
			}
		}
	}

	// Run
	this->compilation.template get_f<FLOAT>()(this->thread_input[thread_id].data(), this->thread_output[thread_id].data());
	
	// Check for NaN in the output
	if (get_check_mode_ON()) {
		const FloatType underlying_type = get_underlying_type(this->compilation.get_compiled_type());
		if (underlying_type == FloatType::Double) {
			const double* output = reinterpret_cast<const double*>(this->thread_output[thread_id].data());
			const int32_t size = (int32_t)(this->thread_output[thread_id].size()*get_n_items_in_simd<FLOAT>());

			for (int32_t i = 0; i < size; i++) {
				if (std::isnan(output[i])) {
					throw std::runtime_error("NaN detected in output #" + std::to_string(i) + " of (compiled function '" + this->name + "')");
				}
			}
		}
		else if (underlying_type == FloatType::Float) {
			const float* output = reinterpret_cast<const float*>(this->thread_output[thread_id].data());
			const int32_t size = (int32_t)(this->thread_output[thread_id].size()*get_n_items_in_simd<FLOAT>());

			for (int32_t i = 0; i < size; i++) {
				if (std::isnan(output[i])) {
					throw std::runtime_error("NaN detected in output #" + std::to_string(i) + " of (compiled function '" + this->name + "')");
				}
			}
		}
	}

	return View<FLOAT>(this->thread_output[thread_id].data(), (int32_t)this->thread_output[thread_id].size());
}

template<typename FLOAT>
int32_t Compiled<FLOAT>::get_n_inputs() const
{
	return this->compilation.get_n_inputs();
}

template<typename FLOAT>
int32_t Compiled<FLOAT>::get_n_outputs() const
{
	return this->compilation.get_n_outputs();
}

template<typename FLOAT>
std::string Compiled<FLOAT>::get_name() const
{
	return this->name;
}

template<typename FLOAT>
void Compiled<FLOAT>::_resize_buffers()
{
	// Initialize with at least 1 thread
	if (this->n_threads == 0) {
		this->n_threads = 1;
	}
	
	this->thread_input.resize(this->n_threads);
	this->thread_output.resize(this->n_threads);
	
	for (int32_t i = 0; i < this->n_threads; i++) {
		this->_resize_buffers(i);
	}
}

template<typename FLOAT>
void Compiled<FLOAT>::_resize_buffers(int32_t thread_id)
{
	if (thread_id >= 0 && thread_id < this->n_threads) {
		this->thread_input[thread_id].resize(this->compilation.get_n_inputs());
		this->thread_output[thread_id].resize(this->compilation.get_n_outputs());
	}
}

template<typename FLOAT>
FLOAT* Compiled<FLOAT>::get_input_buffer(int32_t thread_id)
{
	assert(thread_id >= 0 && thread_id < this->n_threads && "Thread ID is out of range");
	return this->thread_input[thread_id].data();
}

template<typename FLOAT>
FLOAT* Compiled<FLOAT>::get_output_buffer(int32_t thread_id)
{
	assert(thread_id >= 0 && thread_id < this->n_threads && "Thread ID is out of range");
	return this->thread_output[thread_id].data();
}

template<typename FLOAT>
const FLOAT* Compiled<FLOAT>::get_input_buffer(int32_t thread_id) const
{
	assert(thread_id >= 0 && thread_id < this->n_threads && "Thread ID is out of range");
	return this->thread_input[thread_id].data();
}

template<typename FLOAT>
const FLOAT* Compiled<FLOAT>::get_output_buffer(int32_t thread_id) const
{
	assert(thread_id >= 0 && thread_id < this->n_threads && "Thread ID is out of range");
	return this->thread_output[thread_id].data();
}


// Explicit template instantiation
template class symx::Compiled<double>;
template class symx::Compiled<float>;
#ifdef SYMX_ENABLE_AVX2
template class symx::Compiled<__m256d>;
template class symx::Compiled<__m256>;
#endif

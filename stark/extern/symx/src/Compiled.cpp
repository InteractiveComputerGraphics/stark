#include "Compiled.h"


template<typename FLOAT>
symx::Compiled<FLOAT>::Compiled(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, bool suppress_compiler_output)
{
	this->compile(expr, name, folder, id, suppress_compiler_output);
}

template<typename FLOAT>
void symx::Compiled<FLOAT>::compile(const std::vector<Scalar>& expr, std::string name, std::string folder, std::string id, bool suppress_compiler_output)
{
	this->compilation.template compile<FLOAT>(expr, name, folder, id, suppress_compiler_output);
	this->_resize_buffers();
}

template<typename FLOAT>
bool symx::Compiled<FLOAT>::load_if_cached(std::string name, std::string folder, std::string id)
{
	bool success = this->compilation.template load_if_cached<FLOAT>(name, folder, id);
	if (success) {
		this->_resize_buffers();
	}
	return success;
}

template<typename FLOAT>
bool symx::Compiled<FLOAT>::is_valid()
{
	return this->compilation.is_valid();
}

template<typename FLOAT>
bool symx::Compiled<FLOAT>::was_cached()
{
	return this->compilation.was_cached;
}

template<typename FLOAT>
void symx::Compiled<FLOAT>::set(const Scalar& symbol, const FLOAT* v)
{
	assert(symbol.expr.a < this->input.size() && "symx error: symx::Sequence.set() symbol idx higher than buffer size.");
	this->input[symbol.expr.a] = v[0];
}

template<typename FLOAT>
void symx::Compiled<FLOAT>::set(const Vector& vector, const FLOAT* v)
{
	for (int i = 0; i < vector.size(); i++) {
		this->set(vector[i], v + i);
	}
}

template<typename FLOAT>
void symx::Compiled<FLOAT>::set_voidptr(const Scalar& symbol, const void* v)
{
	assert(symbol.expr.a < this->input.size() && "symx error: symx::Sequence.set() symbol idx higher than buffer size.");
	memcpy(&this->input[symbol.expr.a], v, sizeof(FLOAT));
}

template<typename FLOAT>
FLOAT* symx::Compiled<FLOAT>::run()
{
	this->compilation.template get_f<FLOAT>()(this->input.data(), this->output.data());
	return this->output.data();
}

template<typename FLOAT>
int symx::Compiled<FLOAT>::get_n_inputs()
{
	return this->compilation.n_inputs;
}

template<typename FLOAT>
int symx::Compiled<FLOAT>::get_n_outputs()
{
	return this->compilation.n_outputs;
}

template<typename FLOAT>
void symx::Compiled<FLOAT>::_resize_buffers()
{
	this->input.resize(this->compilation.n_inputs);
	this->output.resize(this->compilation.n_outputs);
}


// Explicit template instantiation
template class symx::Compiled<double>;
template class symx::Compiled<float>;
#ifdef SYMX_ENABLE_AVX
template class symx::Compiled<__m128d>;
template class symx::Compiled<__m128>;
template class symx::Compiled<__m256d>;
template class symx::Compiled<__m256>;
template class symx::Compiled<__m512d>;
template class symx::Compiled<__m512>;
#endif

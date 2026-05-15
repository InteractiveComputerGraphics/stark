#include "Workspace.h"

using namespace symx;

Workspace::Workspace()
{
}

Scalar Workspace::make_scalar()
{
	return Scalar(this->expressions.declare_symbol(), &this->expressions);
}

Scalar Workspace::get_scalar(const int32_t symbol_id)
{
	return Scalar(this->expressions.get_symbol_location(symbol_id), &this->expressions);
}

Vector Workspace::make_vector(const int32_t size)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < size; i++) {
		values.push_back(this->make_scalar());
	}
	return Vector(values);
}

Matrix Workspace::make_matrix(const std::array<int32_t, 2> shape)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < shape[0]; i++) {
		for (int32_t j = 0; j < shape[1]; j++) {
			values.push_back(this->make_scalar());
		}
	}
	return Matrix(values, shape);
}

std::vector<Scalar> Workspace::make_scalars(const int32_t n)
{
	std::vector<Scalar> scalars;
	for (int i = 0; i < n; i++) {
		scalars.push_back(this->make_scalar());
	}
	return scalars;
}

std::vector<Vector> Workspace::make_vectors(const int32_t size, const int32_t n)
{
	std::vector<Vector> vectors;
	for (int i = 0; i < n; i++) {
		vectors.push_back(this->make_vector(size));
	}
	return vectors;
}

std::vector<Matrix> Workspace::make_matrices(const std::array<int32_t, 2> shape, const int32_t n)
{
    std::vector<Matrix> matrices;
    for (int i = 0; i < n; i++) {
        matrices.push_back(this->make_matrix(shape));
    }
    return matrices;
}

Scalar Workspace::get_zero()
{
	return Scalar(this->expressions.get_zero_idx(), &this->expressions);
}

Scalar Workspace::get_one()
{
	return Scalar(this->expressions.get_one_idx(), &this->expressions);
}

Vector Workspace::get_zero_vector(const int32_t size)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < size; i++) {
		values.push_back(this->get_zero());
	}
	return Vector(values);
}

Matrix Workspace::get_zero_matrix(const std::array<int32_t, 2> shape)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < shape[0]; i++) {
		for (int32_t j = 0; j < shape[1]; j++) {
			values.push_back(this->get_zero());
		}
	}
	return Matrix(values, shape);
}

Matrix Workspace::get_identity_matrix(const int32_t size)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < size; i++) {
		for (int32_t j = 0; j < size; j++) {
			if (i == j) {
				values.push_back(this->get_one());
			}
			else {
				values.push_back(this->get_zero());
			}
		}
	}
	return Matrix(values, { size, size });
}

int32_t symx::Workspace::get_n_symbols() const
{
    return this->expressions.get_n_symbols();
}
std::string symx::Workspace::get_checksum() const
{
    return this->expressions.get_checksum();
}
const Expressions &Workspace::get_expression_graph() const
{
	return this->expressions;
}


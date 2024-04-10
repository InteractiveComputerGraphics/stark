#include "SymbolicWorkSpace.h"

symx::SymbolicWorkSpace::SymbolicWorkSpace()
{
}

void symx::SymbolicWorkSpace::set_cse_mode(CSE mode)
{
	this->expressions.set_cse_mode(mode);
}

symx::Scalar symx::SymbolicWorkSpace::make_scalar(const std::string label)
{
	return Scalar(this->expressions.declare_symbol(label), &this->expressions);
}

symx::Vector symx::SymbolicWorkSpace::make_vector(const std::string label, const int32_t size)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < size; i++) {
		values.push_back(this->make_scalar(label + "(" + std::to_string(i) + ")"));
	}
	return Vector(values);
}

symx::Matrix symx::SymbolicWorkSpace::make_matrix(const std::string label, const std::array<int32_t, 2> shape)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < shape[0]; i++) {
		for (int32_t j = 0; j < shape[1]; j++) {
			values.push_back(this->make_scalar(label + "(" + std::to_string(i) + ", " + std::to_string(j) + ")"));
		}
	}
	return Matrix(values, shape);
}

std::vector<symx::Scalar> symx::SymbolicWorkSpace::make_scalars(const std::string label, const int32_t n)
{
	std::vector<Scalar> scalars;
	for (int i = 0; i < n; i++) {
		scalars.push_back(this->make_scalar(label + std::to_string(i)));
	}
	return scalars;
}

std::vector<symx::Vector> symx::SymbolicWorkSpace::make_vectors(const std::string label, const int32_t size, const int32_t n)
{
	std::vector<Vector> vectors;
	for (int i = 0; i < n; i++) {
		vectors.push_back(this->make_vector(label + std::to_string(i), size));
	}
	return vectors;
}

symx::Scalar symx::SymbolicWorkSpace::make_branch(const Scalar& condition, const Scalar& positive_branch, const Scalar& negative_branch)
{
	return Scalar(this->expressions.add_branch(condition.expr_id, positive_branch.expr_id, negative_branch.expr_id), &this->expressions);
}

symx::Scalar symx::SymbolicWorkSpace::get_zero()
{
	return Scalar(this->expressions.get_zero_idx(), &this->expressions);
}

symx::Scalar symx::SymbolicWorkSpace::get_one()
{
	return Scalar(this->expressions.get_one_idx(), &this->expressions);
}

symx::Vector symx::SymbolicWorkSpace::get_zero_vector(const int32_t size)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < size; i++) {
		values.push_back(this->get_zero());
	}
	return Vector(values);
}

symx::Matrix symx::SymbolicWorkSpace::get_zero_matrix(const std::array<int32_t, 2> shape)
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < shape[0]; i++) {
		for (int32_t j = 0; j < shape[1]; j++) {
			values.push_back(this->get_zero());
		}
	}
	return Matrix(values, shape);
}

symx::Matrix symx::SymbolicWorkSpace::get_identity_matrix(const int32_t size)
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

const symx::Expressions& symx::SymbolicWorkSpace::get_expression_graph() const
{
	return this->expressions;
}

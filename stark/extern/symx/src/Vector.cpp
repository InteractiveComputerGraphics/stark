#include "Vector.h"

int32_t symx::Vector::size() const
{
	return std::max(this->nrows, this->ncols);
}
const symx::Scalar& symx::Vector::operator[](const int32_t& i) const
{
	assert(i < this->size() && "symx error: symx::Vector index out of bounds.");
	return this->vals[i];
}
symx::Scalar& symx::Vector::operator[](const int32_t& i)
{
	assert(i < this->size() && "symx error: symx::Vector index out of bounds.");
	return this->vals[i];
}
const symx::Scalar& symx::Vector::operator()(const int32_t& i) const
{
	assert(i < this->size() && "symx error: symx::Vector index out of bounds.");
	return this->vals[i];
}
symx::Scalar& symx::Vector::operator()(const int32_t& i)
{
	assert(i < this->size() && "symx error: symx::Vector index out of bounds.");
	return this->vals[i];
}
void symx::Vector::set_value(const double* val)
{
	for (int i = 0; i < this->size(); i++) {
		this->vals[i].set_value(val[i]);
	}
}
symx::Scalar* symx::Vector::data()
{
	return this->vals.data();
}
symx::Vector::Vector(const std::vector<Scalar>& values)
	: vals(values)
{
	this->ncols = 1;
	this->nrows = (int32_t)values.size();
}
symx::Vector symx::Vector::transpose() const
{
	Vector other = (*this);
	other.nrows = this->ncols;
	other.ncols = this->nrows;
	return other;
}
symx::Scalar symx::Vector::dot(const Vector& other) const
{
	const int32_t n = this->size();
	assert(n == other.size() && "symx error: Dot product between symx::Vector of different sizes");

	Scalar sum = (*this)[0] * other[0];
	for (int i = 1; i < n; i++) {
		sum += (*this)[i] * other[i];
	}

	return sum;
}
symx::Scalar symx::Vector::cross2(const Vector& other) const
{
	const int32_t size = this->size();
	assert(size == other.size() && "symx error: Cross product between symx::Vector of different sizes");
	assert(size == 2 && "symx error: symx::Vector::cross3 only supported for Vector of size 2");

	const Vector& u = (*this);
	const Vector& v = other;
	return u[0]*v[1] - u[1]*v[0];
}
symx::Vector symx::Vector::cross3(const Vector& other) const
{
	const int32_t size = this->size();
	assert(size == other.size() && "symx error: Cross product between symx::Vector of different sizes");
	assert(size == 3 && "symx error: symx::Vector::cross3 only supported for Vector of size 3");

	const Vector& u = (*this);
	const Vector& v = other;
	std::vector<Scalar> values = { u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0] };
	return Vector(values);
}
symx::Scalar symx::Vector::squared_norm() const
{
	Scalar sum = this->vals[0].powN(2);
	for (int32_t i = 1; i < this->size(); i++) {
		sum += this->vals[i].powN(2);
	}
	return sum;
}
symx::Scalar symx::Vector::norm() const
{
	return this->squared_norm().sqrt();
}
symx::Vector symx::Vector::normalized() const
{
	Scalar norm = this->norm();
	return (*this)/norm;
}
void symx::Vector::normalize()
{
	(*this) = this->normalized();
}
symx::Vector symx::Vector::operator+(const Vector& other) const
{
	const int32_t size = this->size();
	assert(size == other.size() && "symx error: symx::Vector::operator+ mixed symx::Vector of different sizes");

	std::vector<Scalar> values;
	for (int32_t i = 0; i < size; i++) {
		values.push_back((*this)[i] + other[i]);
	}
	return Vector(values);
}
symx::Vector symx::Vector::operator-(const Vector& other) const
{
	const int32_t size = this->size();
	assert(size == other.size() && "symx error: symx::Vector::operator- mixed symx::Vector of different sizes");

	std::vector<Scalar> values;
	for (int32_t i = 0; i < size; i++) {
		values.push_back((*this)[i] - other[i]);
	}
	return Vector(values);
}
symx::Vector symx::Vector::operator*(double val) const
{
	return (*this) * this->vals[0].make_constant(val);
}
symx::Vector symx::Vector::operator*(const Scalar& scalar) const
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < this->size(); i++) {
		values.push_back((*this)[i] * scalar);
	}
	return Vector(values);
}
symx::Scalar symx::Vector::operator*(const Vector& other) const
{
	assert(this->ncols == other.nrows && "symx error: symx::Vector::operator* ncols does not match nrows");
	return this->dot(other);
}
symx::Vector symx::Vector::operator/(double val) const
{
	return (*this) / this->vals[0].make_constant(val);
}
symx::Vector symx::Vector::operator/(const Scalar& scalar) const
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < this->size(); i++) {
		values.push_back((*this)[i] / scalar);
	}
	return Vector(values);
}
void symx::Vector::operator+=(const Vector& other)
{
	(*this) = (*this) + other;
}
void symx::Vector::operator-=(const Vector& other)
{
	(*this) = (*this) - other;
}
void symx::Vector::operator*=(double val)
{
	(*this) = (*this) * val;
}
void symx::Vector::operator*=(const Scalar& scalar)
{
	(*this) = (*this) * scalar;
}
void symx::Vector::operator/=(double val)
{
	(*this) = (*this) / val;
}
void symx::Vector::operator/=(const Scalar& scalar)
{
	(*this) = (*this) / scalar;
}
symx::Vector symx::Vector::cwise_add(double val) const
{
	return this->cwise_add(this->vals[0].make_constant(val));
}
symx::Vector symx::Vector::cwise_add(const Scalar& scalar) const
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < this->size(); i++) {
		values.push_back((*this)[i] + scalar);
	}
	return Vector(values);
}
symx::Vector symx::Vector::cwise_sub(double val) const
{
	return this->cwise_sub(this->vals[0].make_constant(val));
}
symx::Vector symx::Vector::cwise_sub(const Scalar& scalar) const
{
	std::vector<Scalar> values;
	for (int32_t i = 0; i < this->size(); i++) {
		values.push_back((*this)[i] - scalar);
	}
	return Vector(values);
}
symx::Vector symx::operator*(double val, const Vector& vec)
{
	return vec * val;
}
symx::Vector symx::operator*(const Scalar& scalar, const Vector& vec)
{
	return vec * scalar;
}
symx::Vector symx::operator-(const Vector& vec)
{
	return (-1.0) * vec;
}

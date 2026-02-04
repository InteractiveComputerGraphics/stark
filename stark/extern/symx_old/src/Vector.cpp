#include "Vector.h"

symx::Vector::Vector(const std::vector<Scalar>& values)
	: vals(values)
{
	this->n = (int32_t)values.size();
}
symx::Vector symx::Vector::zero(const int32_t size, const Scalar& seed)
{
	std::vector<Scalar> vals;
	for (int32_t i = 0; i < size; i++) {
		vals.push_back(seed.get_zero());
	}
	return Vector(vals);
}
int32_t symx::Vector::size() const
{
	return this->n;
}
std::vector<symx::Scalar>& symx::Vector::values()
{
	return this->vals;
}
const std::vector<symx::Scalar>& symx::Vector::values() const
{
	return this->vals;
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
symx::Vector symx::Vector::segment(const int32_t begin, const int32_t n) const
{
	assert(begin >= 0 && begin + n <= this->n);

	std::vector<Scalar> values;
	for (int i = 0; i < n; i++) {
		values.push_back((*this)[begin + i]);
	}
	return Vector(values);
}
void symx::Vector::set_segment(const int32_t begin, const int32_t n, const Vector& other)
{
	assert(begin >= 0 && begin + n <= this->n);

	for (int i = 0; i < n; i++) {
		(*this)[begin + i] = other[i];
	}
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

symx::Scalar symx::dot(const Vector& a, const Vector& b)
{
	return a.dot(b);
}

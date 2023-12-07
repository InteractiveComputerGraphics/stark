#pragma once
#include <vector>
#include <cassert>

#include "Expr.h"
#include "Scalar.h"

namespace symx
{
	class Vector
	{
	public:
		/* Fields */
		int32_t nrows = -1;
		int32_t ncols = -1;
		std::vector<Scalar> vals;

	public:
		/* Methods */
		Vector(const std::vector<Scalar>& values);

		// Operations
		Vector slice(const int32_t begin, const int32_t end) const;
		Vector transpose() const;
		Scalar dot(const Vector& other) const;
		Scalar cross2(const Vector& other) const;
		Vector cross3(const Vector& other) const;
		Scalar squared_norm() const;
		Scalar norm() const;
		Vector normalized() const;
		void normalize();

		Vector operator+(const Vector& other) const;
		Vector operator-(const Vector& other) const;
		Vector operator*(double val) const;
		Vector operator*(const Scalar& scalar) const;
		Scalar operator*(const Vector& other) const;
		Vector operator/(double val) const;
		Vector operator/(const Scalar& scalar) const;
		void operator+=(const Vector& other);
		void operator-=(const Vector& other);
		void operator*=(double val);
		void operator*=(const Scalar& scalar);
		void operator/=(double val);
		void operator/=(const Scalar& scalar);
		Vector cwise_add(double val) const;
		Vector cwise_add(const Scalar& scalar) const;
		Vector cwise_sub(double val) const;
		Vector cwise_sub(const Scalar& scalar) const;

		// Functionalities
		int32_t size() const;
		const Scalar& operator[](const int32_t& i) const;
		Scalar& operator[](const int32_t& i);
		const Scalar& operator()(const int32_t& i) const;
		Scalar& operator()(const int32_t& i);
		void set_value(const double* val);
		Scalar* data();
	};

	Vector operator*(double val, const Vector& vec);
	Vector operator*(const Scalar& scalar, const Vector& vec);
	Vector operator-(const Vector& vec);
}
#pragma once
#include <vector>
#include <cassert>

#include "Expr.h"
#include "Scalar.h"

namespace symx
{
	class Vector
	{
	private:
		/* Fields */
		int32_t n = -1;
		std::vector<Scalar> vals;

	public:
		/* Methods */
		Vector(const std::vector<Scalar>& values);
		static Vector zero(const int32_t size, const Scalar& seed);

		// Getters
		int32_t size() const;
		std::vector<Scalar>& values();
		const std::vector<Scalar>& values() const;

		// Indexing
		const Scalar& operator[](const int32_t& i) const;
		Scalar& operator[](const int32_t& i);
		const Scalar& operator()(const int32_t& i) const;
		Scalar& operator()(const int32_t& i);
		Vector segment(const int32_t begin, const int32_t n) const;
		void set_segment(const int32_t begin, const int32_t n, const Vector& other);

		// Operations
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
		Vector operator/(double val) const;
		Vector operator/(const Scalar& scalar) const;
		void operator+=(const Vector& other);
		void operator-=(const Vector& other);
		void operator*=(double val);
		void operator*=(const Scalar& scalar);
		void operator/=(double val);
		void operator/=(const Scalar& scalar);
	};

	Vector operator*(double val, const Vector& vec);
	Vector operator*(const Scalar& scalar, const Vector& vec);
	Vector operator-(const Vector& vec);

	Scalar dot(const Vector& a, const Vector& b);
}

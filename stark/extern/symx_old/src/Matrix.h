#pragma once
#include <vector>
#include <cassert>

#include "Expr.h"
#include "Scalar.h"
#include "Vector.h"

namespace symx
{
	class Matrix
	{
	private:
		/* Fields */
		int32_t nrows = -1;
		int32_t ncols = -1;
		std::vector<Scalar> vals;

	public:
		/* Methods */
		// values must be in row major order
		Matrix(const std::vector<Scalar>& values, const std::array<int32_t, 2>& shape);
		static Matrix zero(const std::array<int32_t, 2> shape, const Scalar& seed);
		static Matrix identity(const int32_t shape, const Scalar& seed);

		// Getters
		int32_t rows() const;
		int32_t cols() const;
		std::array<int32_t, 2> shape() const;
		std::vector<Scalar>& values();
		const std::vector<Scalar>& values() const;

		// Indexing
		Scalar& operator()(const int32_t& i, const int32_t& j);
		const Scalar& operator()(const int32_t& i, const int32_t& j) const;
		Matrix block(const int32_t begin_row, const int32_t begin_col, const int32_t n_rows, const int32_t n_cols) const;
		Vector row(const int32_t idx) const;
		Vector col(const int32_t idx) const;
		void set_block(const int32_t begin_row, const int32_t begin_col, const int32_t n_rows, const int32_t n_cols, const Matrix& m);
		void set_row(const int32_t idx, const Vector& v);
		void set_col(const int32_t idx, const Vector& v);

		// Operations
		Matrix transpose() const;
		Scalar det() const;
		Matrix inv() const;
		Scalar trace() const;
		Scalar frobenius_norm_sq() const;
		Vector singular_values_2x2() const;
		Matrix dot(const Matrix& other) const;
		Vector dot(const Vector& vec) const;

		Matrix operator+(const Matrix& other) const;
		Matrix operator-(const Matrix& other) const;
		Matrix operator*(double val) const;
		Matrix operator*(const Scalar& scalar) const;
		Vector operator*(const Vector& vec) const;
		Matrix operator*(const Matrix& other) const;
		Matrix operator/(double val) const;
		Matrix operator/(const Scalar& scalar) const;
		void operator+=(const Matrix& other);
		void operator-=(const Matrix& other);
		void operator*=(double val);
		void operator*=(const Scalar& scalar);
		void operator/=(double val);
		void operator/=(const Scalar& scalar);

	private:
		bool _is_squared_and_of_size(const int32_t n) const;
		void _assert_consistent_size(const Matrix& other) const;
		void _assert_consistent_size(const Vector& vector) const;
	};

	Matrix operator*(double val, const Matrix& m);
	Matrix operator*(const Scalar& scalar, const Matrix& m);
	Matrix operator-(const Matrix& m);
	Vector operator*(const Vector& vec, const Matrix& m);

	Matrix outer(const Vector& a, const Vector& b);
}
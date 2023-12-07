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
	public:
		enum class Layout
		{
			NonSymmetric,
			SymmetricFullMatrix,
			SymmetricLowerTriangular,
			SymmetricUpperTriangular
		};
	public:
		/* Fields */
		int32_t nrows = -1;
		int32_t ncols = -1;
		std::vector<Scalar> vals; // If symmetric, stored as LowerTriangular
		bool is_symmetric = false;

	public:
		/* Methods */
		Matrix(const std::vector<Scalar>& values, const std::array<int32_t, 2> shape, const Layout symmetry = Layout::NonSymmetric);

		// Operations
		Matrix transpose() const;
		Scalar det() const;
		Matrix inv() const;
		Scalar trace() const;
		Scalar frobenius_norm_sq() const;
		Vector singular_values_2x2() const;
		Matrix block(const int32_t begin_row, const int32_t begin_col, const int32_t n_rows, const int32_t n_cols) const;
		Vector row(const int32_t idx) const;
		Vector col(const int32_t idx) const;
		void set_block(const int32_t begin_row, const int32_t begin_col, const int32_t n_rows, const int32_t n_cols, const Matrix& m);
		void set_row(const int32_t idx, const Vector& v);
		void set_col(const int32_t idx, const Vector& v);

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
		Matrix cwise_add(double val) const;
		Matrix cwise_add(const Scalar& scalar) const;
		Matrix cwise_sub(double val) const;
		Matrix cwise_sub(const Scalar& scalar) const;

		// Functionalities
		int32_t rows() const;
		int32_t cols() const;
		const Scalar& get_value(const int i) const;
		Scalar& get_value(const int i);
		Matrix get_zero(const std::array<int32_t, 2> shape) const;
		Matrix get_identity(const int32_t shape) const;
		const Scalar& operator()(const int32_t& i, const int32_t& j) const;
		Scalar& operator()(const int32_t& i, const int32_t& j);
		Scalar* data();
		Matrix as_symmetric() const;
		void set_value(const double* val);

		static Matrix zero(const std::array<int32_t, 2> shape, const Scalar& seed);
		static Matrix identity(const int32_t shape, const Scalar& seed);

	private:
		bool _is_squared_and_of_size(const int32_t n) const;
		void _assert_consistent_size(const Matrix& other) const;
		void _assert_consistent_size(const Vector& vector) const;
	};

	Matrix operator*(double val, const Matrix& m);
	Matrix operator*(const Scalar& scalar, const Matrix& m);
	Matrix operator-(const Matrix& m);
	Vector operator*(const Vector& vec, const Matrix& m);
}
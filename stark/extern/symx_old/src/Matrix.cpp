#include "Matrix.h"

symx::Matrix::Matrix(const std::vector<Scalar>& values, const std::array<int32_t, 2>& shape)
	: vals(values), nrows(shape[0]), ncols(shape[1])
{
	assert(nrows*ncols != 0 && "symx error: Cannot declare a zero sized matrix.");
	assert(nrows*ncols == (int32_t)values.size() && "symx error: nrows and ncols do not match in symx::Matrix init.");
	this->vals = values;
}
symx::Matrix symx::Matrix::zero(const std::array<int32_t, 2> shape, const Scalar& seed)
{
	std::vector<Scalar> vals;
	for (int32_t i = 0; i < shape[0]; i++) {
		for (int32_t j = 0; j < shape[1]; j++) {
			vals.push_back(seed.get_zero());
		}
	}
	return Matrix(vals, shape);
}
symx::Matrix symx::Matrix::identity(const int32_t shape, const Scalar& seed)
{
	std::vector<Scalar> vals;
	for (int32_t i = 0; i < shape; i++) {
		for (int32_t j = 0; j < shape; j++) {
			if (i == j) {
				vals.push_back(seed.get_one());
			}
			else {
				vals.push_back(seed.get_zero());
			}
		}
	}
	return Matrix(vals, { shape, shape });
}

int32_t symx::Matrix::rows() const
{
	return this->nrows;
}
int32_t symx::Matrix::cols() const
{
	return this->ncols;
}
std::array<int32_t, 2> symx::Matrix::shape() const
{
	return { this->nrows, this->ncols };
}
std::vector<symx::Scalar>& symx::Matrix::values()
{
	return this->vals;
}
const std::vector<symx::Scalar>& symx::Matrix::values() const
{
	return this->vals;
}

const symx::Scalar& symx::Matrix::operator()(const int32_t& i, const int32_t& j) const
{
	return this->vals[i*this->ncols + j];
}
symx::Scalar& symx::Matrix::operator()(const int32_t& i, const int32_t& j)
{
	return this->vals[i*this->ncols + j];
}
symx::Matrix symx::Matrix::block(const int32_t begin_row, const int32_t begin_col, const int32_t n_rows, const int32_t n_cols) const
{
	assert(begin_row >= 0 && begin_row + n_rows <= this->rows());
	assert(begin_col >= 0 && begin_col + n_cols <= this->cols());

	std::vector<Scalar> values;
	for (int i = 0; i < n_rows; i++) {
		for (int j = 0; j < n_cols; j++) {
			values.push_back((*this)(begin_row + i, begin_col + j));
		}
	}
	return Matrix(values, { n_rows, n_cols });
}
symx::Vector symx::Matrix::row(const int32_t idx) const
{
	return Vector(this->block(idx, 0, 1, this->ncols).vals);
}
symx::Vector symx::Matrix::col(const int32_t idx) const
{
	return Vector(this->block(0, idx, this->nrows, 1).vals);
}
void symx::Matrix::set_block(const int32_t begin_row, const int32_t begin_col, const int32_t n_rows, const int32_t n_cols, const Matrix& m)
{
	assert(begin_row >= 0 && begin_row + n_rows <= this->rows());
	assert(begin_col >= 0 && begin_col + n_cols <= this->cols());

	for (int i = 0; i < n_rows; i++) {
		for (int j = 0; j < n_cols; j++) {
			(*this)(begin_row + i, begin_col + j) = m(i, j);
		}
	}
}
void symx::Matrix::set_row(const int32_t idx, const Vector& v)
{
	this->set_block(idx, 0, 1, this->ncols, Matrix(v.values(), { 1, this->ncols }));
}
void symx::Matrix::set_col(const int32_t idx, const Vector& v)
{
	this->set_block(0, idx, this->nrows, 1, Matrix(v.values(), { this->nrows, 1 }));
}

symx::Matrix symx::Matrix::transpose() const
{
	const Matrix& m = (*this);
	Matrix other = m;
	other.nrows = this->ncols;
	other.ncols = this->nrows;

	for (int i = 0; i < this->nrows; i++) {
		for (int j = 0; j < this->ncols; j++) {
			other(j, i) = m(i, j);
		}
	}

	return other;
}
symx::Scalar symx::Matrix::trace() const
{
	const Matrix& m = (*this);
	Scalar tr = m(0, 0);
	for (int i = 1; i < this->nrows; i++) {
		tr += m(i, i);
	}
	return tr;
}
symx::Scalar symx::Matrix::frobenius_norm_sq() const
{
	const Matrix& m = (*this);
	Scalar res = m(0, 0).get_zero();
	for (int i = 0; i < this->nrows; i++) {
		for (int j = 0; j < this->ncols; j++) {
			res += m(i, j).powN(2);
		}
	}
	return res;
}
symx::Vector symx::Matrix::singular_values_2x2() const
{
	/*
		https://scicomp.stackexchange.com/questions/8899/robust-algorithm-for-2-times-2-svd
		paper: Consider the Lowly 2x2 Matrix
		Returns the largest singular value first

		warning: This works only when det(m) > 0
	*/

	const Matrix& m = (*this);
	assert(m.rows() == 2 && m.cols() == 2);

	const Scalar E = 0.5*(m(0, 0) + m(1, 1));
	const Scalar F = 0.5*(m(0, 0) - m(1, 1));
	const Scalar G = 0.5*(m(1, 0) + m(0, 1));
	const Scalar H = 0.5*(m(1, 0) - m(0, 1));

	const Scalar Q = sqrt(E.powN(2) + H.powN(2));
	const Scalar R = sqrt(F.powN(2) + G.powN(2));

	const Scalar s0 = Q + R;
	const Scalar s1 = Q - R;

	return Vector({ s0, s1 });
}
symx::Matrix symx::Matrix::dot(const Matrix& other) const
{
	return (*this) * other;
}
symx::Vector symx::Matrix::dot(const Vector& vec) const
{
	return (*this) * vec;
}
symx::Scalar symx::Matrix::det() const
{
	assert(this->nrows == this->ncols && "symx error: cannot use symx::Matrix::div() on non-square matrices.");
	const Matrix& m = (*this);
	if (this->nrows == 1) {
		return this->vals[0];
	} 
	else if (this->nrows == 2) {
		return m(0, 0)*m(1, 1) - m(0, 1)*m(1, 0);
	}
	else if (this->nrows == 3) {
		return m(0, 0)*m(1, 1)*m(2, 2) - m(0, 0)*m(1, 2)*m(2, 1) - m(0, 1)*m(1, 0)*m(2, 2) + m(0, 1)*m(1, 2)*m(2, 0) + m(0, 2)*m(1, 0)*m(2, 1) - m(0, 2)*m(1, 1)*m(2, 0);
	}
	else if (this->nrows == 4) {
		Scalar tmp0 = m(0, 0)*m(1, 1);
		Scalar tmp1 = m(2, 2)*m(3, 3);
		Scalar tmp2 = m(0, 0)*m(1, 2);
		Scalar tmp3 = m(2, 3)*m(3, 1);
		Scalar tmp4 = m(0, 0)*m(1, 3);
		Scalar tmp5 = m(2, 1)*m(3, 2);
		Scalar tmp6 = m(2, 3)*m(3, 2);
		Scalar tmp7 = m(0, 1)*m(1, 0);
		Scalar tmp8 = m(0, 1)*m(1, 2);
		Scalar tmp9 = m(2, 0)*m(3, 3);
		Scalar tmp10 = m(0, 1)*m(1, 3);
		Scalar tmp11 = m(2, 2)*m(3, 0);
		Scalar tmp12 = m(2, 1)*m(3, 3);
		Scalar tmp13 = m(0, 2)*m(1, 0);
		Scalar tmp14 = m(2, 3)*m(3, 0);
		Scalar tmp15 = m(0, 2)*m(1, 1);
		Scalar tmp16 = m(0, 2)*m(1, 3);
		Scalar tmp17 = m(2, 0)*m(3, 1);
		Scalar tmp18 = m(2, 2)*m(3, 1);
		Scalar tmp19 = m(0, 3)*m(1, 0);
		Scalar tmp20 = m(2, 0)*m(3, 2);
		Scalar tmp21 = m(0, 3)*m(1, 1);
		Scalar tmp22 = m(2, 1)*m(3, 0);
		Scalar tmp23 = m(0, 3)*m(1, 2);
		return tmp0*tmp1 - tmp0*tmp6 - tmp1*tmp7 + tmp10*tmp11 - tmp10*tmp20 - tmp11*tmp21 + tmp12*tmp13 - tmp12*tmp2 - tmp13*tmp3 + tmp14*tmp15 - tmp14*tmp8 - tmp15*tmp9 + tmp16*tmp17 - tmp16*tmp22 - tmp17*tmp23 + tmp18*tmp19 - tmp18*tmp4 - tmp19*tmp5 + tmp2*tmp3 + tmp20*tmp21 + tmp22*tmp23 + tmp4*tmp5 + tmp6*tmp7 + tmp8*tmp9;
	}
	else {
		std::cout << "symx error: Matrix determinant larger than 4x4 is not supported." << std::endl;
		exit(-1);
	}
}
symx::Matrix symx::Matrix::inv() const
{
	assert(this->nrows == this->ncols && "symx error: cannot use symx::Matrix::inv() on non-square matrices.");
	Matrix m_inv = Matrix::zero({ this->nrows, this->ncols }, this->vals[0]);
	const Matrix& m = (*this);

	if (this->nrows == 1) {
		m_inv(0, 0) = 1.0/this->vals[0];
	}
	else if (this->nrows == 2) {
		Scalar tmp0 = 1.0/(m(0, 0)*m(1, 1) - m(0, 1)*m(1, 0));
		m_inv(0, 0) = m(1, 1)*tmp0;
		m_inv(1, 1) = m(0, 0)*tmp0;
		m_inv(0, 1) = -m(0, 1)*tmp0;
		m_inv(1, 0) = -m(1, 0)*tmp0;
	}
	else if (this->nrows == 3) {
		Scalar tmp0 = m(1, 1)*m(2, 2);
		Scalar tmp1 = m(1, 2)*m(2, 1);
		Scalar tmp2 = m(0, 1)*m(1, 2);
		Scalar tmp3 = m(0, 2)*m(2, 1);
		Scalar tmp4 = m(0, 1)*m(2, 2);
		Scalar tmp5 = m(0, 2)*m(1, 1);
		Scalar tmp6 = 1.0/(m(0, 0)*tmp0 - m(0, 0)*tmp1 + m(1, 0)*tmp3 - m(1, 0)*tmp4 + m(2, 0)*tmp2 - m(2, 0)*tmp5);
		m_inv(2, 2) = tmp6*(m(0, 0)*m(1, 1) - m(0, 1)*m(1, 0));
		m_inv(2, 1) = -tmp6*(m(0, 0)*m(2, 1) - m(0, 1)*m(2, 0));
		m_inv(2, 0) = tmp6*(m(1, 0)*m(2, 1) - m(1, 1)*m(2, 0));
		m_inv(1, 2) = -tmp6*(m(0, 0)*m(1, 2) - m(0, 2)*m(1, 0));
		m_inv(1, 1) = tmp6*(m(0, 0)*m(2, 2) - m(0, 2)*m(2, 0));
		m_inv(1, 0) = -tmp6*(m(1, 0)*m(2, 2) - m(1, 2)*m(2, 0));
		m_inv(0, 2) = tmp6*(tmp2 - tmp5);
		m_inv(0, 1) = -tmp6*(-tmp3 + tmp4);
		m_inv(0, 0) = tmp6*(tmp0 - tmp1);
	}
	else if (this->nrows == 4) {
		Scalar tmp0 = m(2, 2)*m(3, 3);
		Scalar tmp1 = m(1, 1)*tmp0;
		Scalar tmp2 = m(2, 3)*m(3, 1);
		Scalar tmp3 = m(1, 2)*tmp2;
		Scalar tmp4 = m(2, 1)*m(3, 2);
		Scalar tmp5 = m(1, 3)*tmp4;
		Scalar tmp6 = m(2, 3)*m(3, 2);
		Scalar tmp7 = m(1, 1)*tmp6;
		Scalar tmp8 = m(2, 1)*m(3, 3);
		Scalar tmp9 = m(1, 2)*tmp8;
		Scalar tmp10 = m(2, 2)*m(3, 1);
		Scalar tmp11 = m(1, 3)*tmp10;
		Scalar tmp12 = m(0, 1)*tmp6;
		Scalar tmp13 = m(0, 1)*m(1, 2);
		Scalar tmp14 = m(3, 3)*tmp13;
		Scalar tmp15 = m(0, 1)*m(1, 3);
		Scalar tmp16 = m(2, 2)*tmp15;
		Scalar tmp17 = m(0, 2)*tmp8;
		Scalar tmp18 = m(0, 2)*m(1, 1);
		Scalar tmp19 = m(2, 3)*tmp18;
		Scalar tmp20 = m(0, 2)*m(1, 3);
		Scalar tmp21 = m(3, 1)*tmp20;
		Scalar tmp22 = m(0, 3)*tmp10;
		Scalar tmp23 = m(0, 3)*m(1, 1);
		Scalar tmp24 = m(3, 2)*tmp23;
		Scalar tmp25 = m(0, 3)*m(1, 2);
		Scalar tmp26 = m(2, 1)*tmp25;
		Scalar tmp27 = m(0, 1)*tmp0;
		Scalar tmp28 = m(2, 3)*tmp13;
		Scalar tmp29 = m(3, 2)*tmp15;
		Scalar tmp30 = m(0, 2)*tmp2;
		Scalar tmp31 = m(3, 3)*tmp18;
		Scalar tmp32 = m(2, 1)*tmp20;
		Scalar tmp33 = m(0, 3)*tmp4;
		Scalar tmp34 = m(2, 2)*tmp23;
		Scalar tmp35 = m(3, 1)*tmp25;
		Scalar tmp36 = 1.0/(m(0, 0)*tmp1 - m(0, 0)*tmp11 + m(0, 0)*tmp3 + m(0, 0)*tmp5 - m(0, 0)*tmp7 - m(0, 0)*tmp9 + m(1, 0)*tmp12 + m(1, 0)*tmp17 + m(1, 0)*tmp22 - m(1, 0)*tmp27 - m(1, 0)*tmp30 - m(1, 0)*tmp33 + m(2, 0)*tmp14 + m(2, 0)*tmp21 + m(2, 0)*tmp24 - m(2, 0)*tmp29 - m(2, 0)*tmp31 - m(2, 0)*tmp35 + m(3, 0)*tmp16 + m(3, 0)*tmp19 + m(3, 0)*tmp26 - m(3, 0)*tmp28 - m(3, 0)*tmp32 - m(3, 0)*tmp34);
		m_inv(0, 3) = -tmp36*(-tmp16 - tmp19 - tmp26 + tmp28 + tmp32 + tmp34);
		m_inv(0, 2) = tmp36*(tmp14 + tmp21 + tmp24 - tmp29 - tmp31 - tmp35);
		m_inv(0, 1) = -tmp36*(-tmp12 - tmp17 - tmp22 + tmp27 + tmp30 + tmp33);
		m_inv(0, 0) = tmp36*(tmp1 - tmp11 + tmp3 + tmp5 - tmp7 - tmp9);
		Scalar tmp37 = m(2, 3)*m(3, 0);
		Scalar tmp38 = m(2, 0)*m(3, 2);
		Scalar tmp39 = m(2, 0)*m(3, 3);
		Scalar tmp40 = m(2, 2)*m(3, 0);
		m_inv(1, 1) = tmp36*(m(0, 0)*tmp0 - m(0, 0)*tmp6 + m(0, 2)*tmp37 - m(0, 2)*tmp39 + m(0, 3)*tmp38 - m(0, 3)*tmp40);
		m_inv(1, 0) = -tmp36*(m(1, 0)*tmp0 - m(1, 0)*tmp6 + m(1, 2)*tmp37 - m(1, 2)*tmp39 + m(1, 3)*tmp38 - m(1, 3)*tmp40);
		Scalar tmp41 = m(0, 0)*m(1, 2);
		Scalar tmp42 = m(0, 3)*m(1, 0);
		Scalar tmp43 = m(0, 0)*m(1, 3);
		Scalar tmp44 = m(0, 2)*m(1, 0);
		m_inv(1, 3) = tmp36*(m(2, 0)*tmp20 - m(2, 0)*tmp25 + m(2, 2)*tmp42 - m(2, 2)*tmp43 + m(2, 3)*tmp41 - m(2, 3)*tmp44);
		m_inv(1, 2) = -tmp36*(m(3, 0)*tmp20 - m(3, 0)*tmp25 + m(3, 2)*tmp42 - m(3, 2)*tmp43 + m(3, 3)*tmp41 - m(3, 3)*tmp44);
		Scalar tmp45 = m(2, 0)*m(3, 1);
		Scalar tmp46 = m(2, 1)*m(3, 0);
		m_inv(3, 1) = tmp36*(-m(0, 0)*tmp10 + m(0, 0)*tmp4 - m(0, 1)*tmp38 + m(0, 1)*tmp40 + m(0, 2)*tmp45 - m(0, 2)*tmp46);
		m_inv(3, 0) = -tmp36*(-m(1, 0)*tmp10 + m(1, 0)*tmp4 - m(1, 1)*tmp38 + m(1, 1)*tmp40 + m(1, 2)*tmp45 - m(1, 2)*tmp46);
		m_inv(2, 1) = -tmp36*(-m(0, 0)*tmp2 + m(0, 0)*tmp8 + m(0, 1)*tmp37 - m(0, 1)*tmp39 + m(0, 3)*tmp45 - m(0, 3)*tmp46);
		m_inv(2, 0) = tmp36*(-m(1, 0)*tmp2 + m(1, 0)*tmp8 + m(1, 1)*tmp37 - m(1, 1)*tmp39 + m(1, 3)*tmp45 - m(1, 3)*tmp46);
		Scalar tmp47 = m(0, 0)*m(1, 1);
		Scalar tmp48 = m(0, 1)*m(1, 0);
		m_inv(3, 3) = tmp36*(m(2, 0)*tmp13 - m(2, 0)*tmp18 - m(2, 1)*tmp41 + m(2, 1)*tmp44 + m(2, 2)*tmp47 - m(2, 2)*tmp48);
		m_inv(3, 2) = -tmp36*(m(3, 0)*tmp13 - m(3, 0)*tmp18 - m(3, 1)*tmp41 + m(3, 1)*tmp44 + m(3, 2)*tmp47 - m(3, 2)*tmp48);
		m_inv(2, 3) = -tmp36*(m(2, 0)*tmp15 - m(2, 0)*tmp23 + m(2, 1)*tmp42 - m(2, 1)*tmp43 + m(2, 3)*tmp47 - m(2, 3)*tmp48);
		m_inv(2, 2) = tmp36*(m(3, 0)*tmp15 - m(3, 0)*tmp23 + m(3, 1)*tmp42 - m(3, 1)*tmp43 + m(3, 3)*tmp47 - m(3, 3)*tmp48);
	}
	else {
		std::cout << "symx error: Matrix inverse larger than 4x4 is not supported." << std::endl;
		exit(-1);
	}

	return m_inv;
}

symx::Matrix symx::Matrix::operator+(const Matrix& other) const
{
	this->_assert_consistent_size(other);

	std::vector<Scalar> vals;
	for (int i = 0; i < this->nrows; i++) {
		for (int j = 0; j < this->ncols; j++) {
			vals.push_back((*this)(i, j) + other(i, j));
		}
	}
	return Matrix(vals, { this->nrows, this->ncols });
}
symx::Matrix symx::Matrix::operator-(const Matrix& other) const
{
	this->_assert_consistent_size(other);

	std::vector<Scalar> vals;
	for (int i = 0; i < this->nrows; i++) {
		for (int j = 0; j < this->ncols; j++) {
			vals.push_back((*this)(i, j) - other(i, j));
		}
	}
	return Matrix(vals, { this->nrows, this->ncols });
}
symx::Matrix symx::Matrix::operator*(double val) const
{
	return (*this)*this->vals[0].make_constant(val);
}
symx::Matrix symx::Matrix::operator*(const Scalar& scalar) const
{
	std::vector<Scalar> vals;
	for (int i = 0; i < (int)this->vals.size(); i++) {
		vals.push_back(this->vals[i]*scalar);
	}
	return Matrix(vals, { this->nrows, this->ncols });
}
symx::Vector symx::Matrix::operator*(const Vector& vec) const
{
	this->_assert_consistent_size(vec);
	const Matrix& m = (*this);
	std::vector<Scalar> vals;
	for (int i = 0; i < this->nrows; i++) {
		Scalar sum = m(i, 0)*vec[0];
		for (int j = 1; j < this->ncols; j++) {
			sum += m(i, j)*vec[j];
		}
		vals.push_back(sum);
	}
	return Vector(vals);
}
symx::Matrix symx::Matrix::operator*(const Matrix& other) const
{
	assert((this->nrows > 0 && this->ncols > 0) && "symx error: Empty Matrix.");
	assert((other.nrows > 0 && other.ncols > 0) && "symx error: Empty Matrix.");
	assert((this->ncols == other.nrows) && "symx error: Matrix sizes do not match.");

	const Matrix& m = (*this);
	std::vector<Scalar> vals;
	for (int i = 0; i < this->nrows; i++) {
		for (int j = 0; j < other.ncols; j++) {
			Scalar sum = m(i, 0)*other(0, j);
			for (int k = 1; k < this->ncols; k++) {
				sum += m(i, k)*other(k, j);
			}
			vals.push_back(sum);
		}
	}
	return Matrix(vals, { this->nrows, other.ncols });
}
symx::Matrix symx::Matrix::operator/(double val) const
{
	return (*this)/this->vals[0].make_constant(val);
}
symx::Matrix symx::Matrix::operator/(const Scalar& scalar) const
{
	std::vector<Scalar> vals;
	for (int i = 0; i < (int)this->vals.size(); i++) {
		vals.push_back(this->vals[i]/scalar);
	}
	return Matrix(vals, { this->nrows, this->ncols });
}
void symx::Matrix::operator+=(const Matrix& other)
{
	(*this) = (*this) + other;
}
void symx::Matrix::operator-=(const Matrix& other)
{
	(*this) = (*this) - other;
}
void symx::Matrix::operator*=(double val)
{
	(*this) = (*this)*val;
}
void symx::Matrix::operator*=(const Scalar& scalar)
{
	(*this) = (*this)*scalar;
}
void symx::Matrix::operator/=(double val)
{
	(*this) = (*this)/val;
}
void symx::Matrix::operator/=(const Scalar& scalar)
{
	(*this) = (*this)/scalar;
}

symx::Matrix symx::operator*(double val, const Matrix& m)
{
	return m*val;
}
symx::Matrix symx::operator*(const Scalar& scalar, const Matrix& m)
{
	return m*scalar;
}
symx::Matrix symx::operator-(const Matrix& m) 
{
	return (-1.0)*m;
}
symx::Vector symx::operator*(const Vector& vec, const Matrix& m)
{
	assert((m.cols() > 0 && vec.size() > 0) && "symx error: Empty Matrix or Vector.");
	assert((vec.size() == m.rows()) && "symx error: Matrix and vector sizes do not match.");

	std::vector<Scalar> vals;
	for (int j = 0; j < m.cols(); j++) {
		Scalar sum = vec[0]*m(0, j);
		for (int i = 1; i < m.rows(); i++) {
			sum += vec[i]*m(i, j);
		}
		vals.push_back(sum);
	}
	return Vector(vals);
}

symx::Matrix symx::outer(const Vector& a, const Vector& b)
{
	assert((a.size() == b.size()) && "symx error: Vectors not of equal size.");
	std::vector<Scalar> values;
	for (int i = 0; i < a.size(); i++) {
		for (int j = 0; j < b.size(); j++) {
			values.push_back(a[i] * b[j]);
		}
	}
	return Matrix(values, { a.size() , a.size() });
}

bool symx::Matrix::_is_squared_and_of_size(const int32_t n) const
{
	return (this->nrows == n && this->ncols == n);
}
void symx::Matrix::_assert_consistent_size(const Matrix& other) const
{
	assert((this->nrows > 0 && this->ncols > 0) && "symx error: Empty Matrix.");
	assert((this->nrows == other.nrows && this->ncols == other.ncols) && "symx error: Matrix sizes do not match.");
}
void symx::Matrix::_assert_consistent_size(const Vector& vector) const
{
	assert((this->cols() > 0 && vector.size() > 0) && "symx error: Empty Matrix or Vector.");
	assert((this->cols() == vector.size()) && "symx error: Matrix and vector sizes do not match.");
}
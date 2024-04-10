#include "Scalar.h"

// ------------------------------------------------------------------------------------------------------
symx::Scalar double_to_scalar(const symx::Scalar& seed, const double v)
{
	if (v == 0.0) {
		return seed.get_zero();
	}
	else if (v == 1.0) {
		return seed.get_one();
	}
	else {
		return v * seed.get_one();
	}
}
// ------------------------------------------------------------------------------------------------------

symx::Scalar& symx::Scalar::operator=(const Scalar& other)
{
	assert(this->expressions == other.expressions && "symx error: Cannot mix symbols from different workspaces");
	this->expr_id = other.expr_id;
	this->expr = other.expr;
	return *this;
}
symx::Scalar symx::Scalar::operator+(double val) const
{
	Scalar scalar(this->expressions->declare_constant_float(val), this->expressions);
	return (*this) + scalar;
}
symx::Scalar symx::Scalar::operator-(double val) const
{
	Scalar scalar(this->expressions->declare_constant_float(val), this->expressions);
	return (*this) - scalar;
}
symx::Scalar symx::Scalar::operator*(double val) const
{
	Scalar scalar(this->expressions->declare_constant_float(val), this->expressions);
	return (*this) * scalar;
}
symx::Scalar symx::Scalar::operator/(double val) const
{
	return (*this) * (1.0 / val);
}
void symx::Scalar::operator+=(double val)
{
	symx::Scalar sum = (*this) + val;
	(*this) = sum;
}
void symx::Scalar::operator-=(double val)
{
	symx::Scalar sum = (*this) - val;
	(*this) = sum;
}
void symx::Scalar::operator*=(double val)
{
	symx::Scalar sum = (*this) * val;
	(*this) = sum;
}
void symx::Scalar::operator/=(double val)
{
	symx::Scalar sum = (*this) * (1.0 / val);
	(*this) = sum;
}
symx::Scalar symx::Scalar::operator+(const Scalar& other) const
{
	const Scalar& u = (*this);
	const Scalar& v = other;
	if (u.is_zero()) {
		return v;
	}
	else if (v.is_zero()) {
		return u;
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Add, this->expr_id, other.expr_id), this->expressions);
	}
}
symx::Scalar symx::Scalar::operator-(const Scalar& other) const
{
	const Scalar& u = (*this);
	const Scalar& v = other;
	if (v.is_zero()) {
		return u;
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Sub, this->expr_id, other.expr_id), this->expressions);
	}
}
symx::Scalar symx::Scalar::operator*(const Scalar& other) const
{
	const Scalar& u = (*this);
	const Scalar& v = other;
	if (u.is_zero() || v.is_zero()) {
		return this->get_zero();
	}
	else if (u.is_one()) {
		return v;
	}
	else if (v.is_one()) {
		return u;
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Mul, this->expr_id, other.expr_id), this->expressions);
	}
}
symx::Scalar symx::Scalar::operator/(const Scalar& other) const
{
	const Scalar& u = (*this);
	const Scalar& v = other;

	if (v.is_zero()) {
		std::cout << "symx error: Division by zero found." << std::endl;
		exit(-1);
	}
	else if (v.is_one() && u.is_one()) {
		return this->get_one();
	}
	else if (v.is_one()) {
		return u;
	}
	else {
		return u * v.inv();
	}
}
void symx::Scalar::operator+=(const Scalar& other)
{
	symx::Scalar sum = (*this) + other;
	(*this) = sum;
}
void symx::Scalar::operator-=(const Scalar& other)
{
	symx::Scalar sum = (*this) - other;
	(*this) = sum;
}
void symx::Scalar::operator*=(const Scalar& other)
{
	symx::Scalar sum = (*this) * other;
	(*this) = sum;
}
void symx::Scalar::operator/=(const Scalar& other)
{
	symx::Scalar sum = (*this) / other;
	(*this) = sum;
}
symx::Scalar symx::Scalar::powN(const int32_t val) const
{
	const Scalar& u = (*this);
	if (val == 0) {
		return this->get_one();
	}
	else if (val == 1) {
		return u;
	}
	else if (val == -1) {
		return u.inv();
	}
	else if (val < 0) {
		const Scalar powN = Scalar(this->expressions->add_operation(ExprType::PowN, this->expr_id, -val), this->expressions);
		return powN.inv();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::PowN, this->expr_id, val), this->expressions);
	}
}
symx::Scalar symx::Scalar::powF(double val) const
{
	const int32_t constant_double_id = this->expressions->declare_constant_float(val);
	return Scalar(this->expressions->add_operation(ExprType::PowF, this->expr_id, constant_double_id), this->expressions);
}
symx::Scalar symx::Scalar::sqrt() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		return this->get_zero();
	}
	else if (u.is_one()) {
		return this->get_one();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Sqrt, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::log() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		std::cout << "symx error: log(0) found." << std::endl;
		exit(-1);
	}

	if (u.is_one()) {
		return this->get_one();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Ln, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::exp() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		return this->get_one();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Exp, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::sin() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		return this->get_zero();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Sin, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::cos() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		return this->get_one();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Cos, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::tan() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		return this->get_zero();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Tan, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::asin() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		return this->get_zero();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::ArcSin, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::acos() const
{
	const Scalar& u = (*this);
	if (u.is_one()) {
		return this->get_zero();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::ArcCos, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::atan() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		return this->get_zero();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::ArcTan, this->expr_id, -1), this->expressions);
	}
}
symx::Scalar symx::Scalar::print() const
{
	const Scalar& u = (*this);
	return Scalar(this->expressions->add_operation(ExprType::Print, this->expr_id, -1), this->expressions);
}
symx::Scalar symx::Scalar::inv() const
{
	const Scalar& u = (*this);
	if (u.is_zero()) {
		std::cout << "symx error: symbolic division by zero found." << std::endl;
		exit(-1);
	}
	else if (u.is_one()) {
		return this->get_one();
	}
	else {
		return Scalar(this->expressions->add_operation(ExprType::Reciprocal, this->expr_id, -1), this->expressions);
	}
}
bool symx::Scalar::is_zero() const
{
	return this->expr.type == ExprType::Zero;
}
bool symx::Scalar::is_one() const
{
	return this->expr.type == ExprType::One;
}
void symx::Scalar::set_value(const double val)
{
	this->expressions->set_value(this->expr_id, val);
}
double symx::Scalar::get_value() const
{
	return this->expressions->get_value(this->expr_id);
}
double symx::Scalar::eval()
{
	return this->expressions->eval(this->expr_id);
}
std::string symx::Scalar::get_checksum()
{
	picosha2::hash256_one_by_one hasher;
	hasher.init();
	this->get_checksum(hasher);
	hasher.finish();

	std::vector<unsigned char> out_hash(picosha2::k_digest_size);
	hasher.get_hash_bytes(out_hash.begin(), out_hash.end());
	std::string hex_str = picosha2::bytes_to_hex_string(out_hash.begin(), out_hash.end());
	return hex_str;
}
void symx::Scalar::get_checksum(picosha2::hash256_one_by_one& hasher)
{
	std::array<int32_t, 4> expr_ints = { static_cast<int32_t>(this->expr.type), this->expr.a, this->expr.b, this->expr.cond };
	std::array<char, sizeof(std::array<int32_t, 4>)>* expr_bytes = reinterpret_cast<std::array<char, sizeof(std::array<int32_t, 4>)>*>(&expr_ints);
	hasher.process(expr_bytes->begin(), expr_bytes->end());

	if (is_operation(this->expr.type) || this->expr.type == ExprType::Branch) {
		this->left().get_checksum(hasher);
		if (this->has_right()) {
			this->right().get_checksum(hasher);
		}
	}
	else if (this->expr.type == ExprType::Symbol) {
		const std::string label = this->get_name();
		std::vector<char> bytes(label.begin(), label.end());
		hasher.process(bytes.begin(), bytes.end());
	}
	else if (this->expr.type == ExprType::Zero || this->expr.type == ExprType::One || this->expr.type == ExprType::ConstantFloat) {
		double value = 0.0;
		switch (this->expr.type)
		{
			case ExprType::Zero:
				value = 0.0;
				break;
			case ExprType::One:
				value = 1.0;
				break;
			case ExprType::ConstantFloat:
				value = this->expr.unpack_double();
				break;
			default:
				break;
		}
		std::array<char, sizeof(double)>* bytes = reinterpret_cast<std::array<char, sizeof(double)>*>(&value);
		hasher.process(bytes->begin(), bytes->end());
	}
}
symx::Scalar symx::Scalar::left() const
{
	return Scalar(this->expr.a, this->expressions);
}
symx::Scalar symx::Scalar::right() const
{
	const int32_t right = this->expr.b;
	if (right == -1) {
		std::cout << "symx error: symx::Scalar::right() found no right argument." << std::endl;
		exit(-1);
	}
	return Scalar(this->expr.b, this->expressions);
}
bool symx::Scalar::has_right() const
{
	const int32_t right = this->expr.b;
	return right != -1;
}
bool symx::Scalar::has_not_right() const
{
	const int32_t right = this->expr.b;
	return right == -1;
}
bool symx::Scalar::has_branch() const
{
	if (this->expr.type == ExprType::Branch) {
		return true;
	}
	else {
		if (is_operation(this->expr.type)) {
			bool has_branch = this->left().has_branch();
			if (this->has_right()) {
				has_branch = has_branch || this->right().has_branch();
			}
			return has_branch;
		}
		else {
			return false;
		}
	} 
}
symx::Scalar symx::Scalar::get_zero() const
{
	return Scalar(this->expressions->get_zero_idx(), this->expressions);
}
symx::Scalar symx::Scalar::get_one() const
{
	return Scalar(this->expressions->get_one_idx(), this->expressions);
}
symx::Scalar symx::Scalar::get_condition() const
{
	if (this->expr.type != ExprType::Branch) {
		std::cout << "symx error: Cannot call Scalar::get_condition() for not ExprType::Branch type of Scalars." << std::endl;
		exit(-1);
	}
	return Scalar(this->expr.cond, this->expressions);
}
symx::Scalar symx::Scalar::make_constant(const double val) const
{
	return Scalar(this->expressions->declare_constant_float(val), this->expressions);
}
symx::Scalar symx::Scalar::make_branch(const Scalar& condition, const Scalar& positive_branch, const Scalar& negative_branch) const
{
	return Scalar(this->expressions->add_branch(condition.expr_id, positive_branch.expr_id, negative_branch.expr_id), this->expressions);
}
int32_t symx::Scalar::get_symbol_idx() const
{
	if (this->is_symbol()) {
		return this->expr.a;
	}
	else {
		std::cout << "symx error: Scalar::get_symbol_idx() called on a non-symbol expression." << std::endl;
		exit(-1);
	}
}
bool symx::Scalar::is_symbol() const
{
	return this->expr.type == ExprType::Symbol;
}
std::string symx::Scalar::get_name() const
{
	if (!this->is_symbol()) {
		std::cout << "symx error: Scalar::get_name() can only be used on non-derived symbols." << std::endl;
		exit(-1);
	}
	return this->expressions->symbols[this->expr.a];
}
const symx::Expressions* symx::Scalar::get_expression_graph() const
{
	return this->expressions;
}
symx::Scalar symx::operator+(double val, Scalar scalar)
{
	return scalar + val;
}
symx::Scalar symx::operator-(double val, Scalar scalar)
{
	Scalar val_ = scalar.make_constant(val);
	return val_ - scalar;
}
symx::Scalar symx::operator*(double val, Scalar scalar)
{
	return scalar * val;
}
symx::Scalar symx::operator/(double val, Scalar scalar)
{
	Scalar val_ = scalar.make_constant(val);
	return val_ / scalar;
}
symx::Scalar symx::operator-(const Scalar& scalar)
{
	return 0.0 - scalar;
}
symx::Scalar symx::powN(const Scalar& scalar, int32_t val)
{
	return scalar.powN(val);
}
symx::Scalar symx::powF(const Scalar& scalar, double val)
{
	return scalar.powF(val);
}
symx::Scalar symx::sqrt(const Scalar& scalar)
{
	return scalar.sqrt();
}
symx::Scalar symx::log(const Scalar& scalar)
{
	return scalar.log();
}
symx::Scalar symx::exp(const Scalar& scalar)
{
	return scalar.exp();
}
symx::Scalar symx::sin(const Scalar& scalar)
{
	return scalar.sin();
}
symx::Scalar symx::cos(const Scalar& scalar)
{
	return scalar.cos();
}
symx::Scalar symx::tan(const Scalar& scalar)
{
	return scalar.tan();
}
symx::Scalar symx::asin(const Scalar& scalar)
{
	return scalar.asin();
}
symx::Scalar symx::acos(const Scalar& scalar)
{
	return scalar.acos();
}
symx::Scalar symx::atan(const Scalar& scalar)
{
	return scalar.atan();
}

symx::Scalar symx::operator>(const Scalar& a, const Scalar& b)
{
	return a - b;
}
symx::Scalar symx::operator<(const Scalar& a, const Scalar& b)
{
	return b - a;
}
symx::Scalar symx::branch(const Scalar& cond, const Scalar& a, const Scalar& b)
{
	return a.make_branch(cond, a, b);
}
symx::Scalar symx::min(const Scalar& a, const Scalar& b)
{
	return branch(a > b, b, a);
}
symx::Scalar symx::max(const Scalar& a, const Scalar& b)
{
	return branch(a > b, a, b);
}
symx::Scalar symx::abs(const Scalar& a)
{
	return branch(a > 0.0, a, -a);
}
symx::Scalar symx::sign(const Scalar& a)
{
	return branch(a > 0.0, 1.0, -1.0);
}

symx::Scalar symx::branch(const Scalar& cond, const Scalar& positive_branch, const double negative_branch)
{
	return branch(cond, positive_branch, double_to_scalar(cond, negative_branch));
}
symx::Scalar symx::branch(const Scalar& cond, const double positive_branch, const Scalar& negative_branch)
{
	return branch(cond, double_to_scalar(cond, positive_branch), negative_branch);
}
symx::Scalar symx::branch(const Scalar& cond, const double positive_branch, const double negative_branch)
{
	return branch(cond, double_to_scalar(cond, positive_branch), double_to_scalar(cond, negative_branch));
}
symx::Scalar symx::operator>(const double& a, const Scalar& b)
{
	return double_to_scalar(b, a) > b;
}
symx::Scalar symx::operator>(const Scalar& a, const double& b)
{
	return a > double_to_scalar(a, b);
}
symx::Scalar symx::operator<(const double& a, const Scalar& b)
{
	return double_to_scalar(b, a) < b;
}
symx::Scalar symx::operator<(const Scalar& a, const double& b)
{
	return a < double_to_scalar(a, b);
}

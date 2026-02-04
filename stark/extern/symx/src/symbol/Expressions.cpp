#include "Expressions.h"

#include <limits>
#include <cmath>
#include <picoSHA2/picosha2.h>

// Helpers ========================================================================================
bool containsOnlyASCII(const std::string& string) {
	for (auto c : string) {
		if (static_cast<unsigned char>(c) > 127) {
			return false;
		}
	}
	return true;
}
// ================================================================================================

using namespace symx;

// CSE Mode
CSE symx::_cse_mode = CSE::Safe; // Default to false (no NaN checking)

void symx::set_cse_mode(CSE mode)
{
	_cse_mode = mode;
}

CSE symx::get_cse_mode()
{
	return _cse_mode;
}

Expressions::Expressions()
{
	// Zero and One are always at indices 0 and 1.
	this->_find_or_add(Expr(ExprType::Zero, -1, -1));
	this->_find_or_add(Expr(ExprType::One, -1, -1));
}
int32_t Expressions::declare_symbol(std::string label)
{
	if (!containsOnlyASCII(label)) {
		std::cout << "symx error: Symbols labels must be only ASCII." << std::endl;
		exit(-1);
	}

	// Create the symbol
	this->symbols.push_back(label);
	this->symbol_values_set.push_back(false);
	this->symbol_values.push_back(0.0);
	this->symbol_locations.push_back((int32_t)this->expressions.size());

	// Add it to the expression graph
	const int32_t symbol_id = (int32_t)this->symbols.size() - 1;
	const int32_t expr_id = (int32_t)this->expressions.size();
	this->expressions.push_back(Expr(ExprType::Symbol, symbol_id, -1));

	return expr_id;
}
int32_t Expressions::declare_constant_float(double constant)
{
	// Check if it is zero
	if (std::abs(constant) < 10.0 * std::numeric_limits<double>::epsilon()) {
		return this->get_zero_idx();
	}

	// Check if it is one
	else if (std::abs(constant - 1.0) < 10.0 * std::numeric_limits<double>::epsilon()) {
		return this->get_one_idx();
	}

	// Check if it is already declared
	else {
		// Create the Expr
		Expr expr(ExprType::ConstantFloat, 0, 0, 0);
		expr.pack_double(constant);
		return this->_find_or_add(expr);
	}
}
int32_t Expressions::add_operation(ExprType type, int32_t a, int32_t b)
{
	if (!is_operation(type)) {
		std::cout << "symx error: cannot use Expressions::add_operation() for non-operation types." << std::endl;
		exit(-1);
	}

	return this->_find_or_add(Expr(type, a, b));
}
int32_t Expressions::add_branch(int32_t cond_expr_id, int32_t positive_branch_expr_id, int32_t negative_branch_expr_id)
{
	int32_t expr_id = (int32_t)this->expressions.size();
	this->expressions.push_back(Expr(ExprType::Branch, positive_branch_expr_id, negative_branch_expr_id, cond_expr_id));
	return expr_id;
}
int32_t Expressions::size() const
{
	return (int32_t)this->expressions.size();
}
int32_t Expressions::get_n_symbols() const
{
	return (int32_t)this->symbols.size();
}
int32_t symx::Expressions::get_symbol_location(int32_t symbol_id) const
{
    return this->symbol_locations[symbol_id];
}
const std::vector<Expr> &Expressions::get_expressions() const
{
	return this->expressions;
}
const std::vector<std::string>& Expressions::get_symbol_names() const
{
	return this->symbols;
}
int32_t Expressions::get_zero_idx()
{
	return 0;
}
int32_t Expressions::get_one_idx()
{
	return 1;
}
void Expressions::set_value(int32_t expr_id, double val)
{
	const Expr& expr = this->expressions[expr_id];
	if (expr.type != ExprType::Symbol) {
		std::cout << "symx error: Can't set a non-symbol value." << std::endl;
		exit(-1);
	}
	const int symbol_id = expr.a;
	this->symbol_values[symbol_id] = val;
	this->symbol_values_set[symbol_id] = static_cast<uint8_t>(true);
}

double Expressions::get_value(int32_t expr_id) const
{
	const Expr& expr = this->expressions[expr_id];
	if (expr.type != ExprType::Symbol) {
		std::cout << "symx error: Can't get a value from a non-symbol." << std::endl;
		exit(-1);
	}
	if (!this->symbol_values_set[expr.a]) {
		std::cout << "symx error: Can't read a value not set. Missing symbol " << this->symbols[expr.a] << std::endl;
	}

	const int symbol_id = expr.a;
	return this->symbol_values[symbol_id];
}

double Expressions::eval(int32_t expr_id) const
{
	const Expr& expr = this->expressions[expr_id];
	switch (expr.type)
	{
	case ExprType::Zero:
		return 0.0; break;
	case ExprType::One:
		return 1.0; break;
	case ExprType::ConstantFloat:
		return expr.unpack_double(); break;
	case ExprType::Symbol:
		if (!this->symbol_values_set[expr.a]) {
			std::cout << "symx error: Cannot evaluate an expression if all symbols are not set.\nMissing symbol " << this->symbols[expr.a] << std::endl;
		}
		return this->symbol_values[expr.a]; break;
	case ExprType::Add:
		return this->eval(expr.a) + this->eval(expr.b); break;
	case ExprType::Sub:
		return this->eval(expr.a) - this->eval(expr.b); break;
	case ExprType::Mul:
		return this->eval(expr.a) * this->eval(expr.b); break;
	case ExprType::Reciprocal:
		return 1.0 / this->eval(expr.a); break;
	case ExprType::PowN:
		return std::pow(this->eval(expr.a), (int)expr.b); break;
	case ExprType::PowF:
		return std::pow(this->eval(expr.a), this->eval(expr.b)); break;
	case ExprType::Sqrt:
		return std::sqrt(this->eval(expr.a)); break;
	case ExprType::Ln:
	{
		double arg = this->eval(expr.a);
		if (arg <= 0.0) {
			return -std::numeric_limits<double>::infinity();
		}
		return std::log(arg);
	}
	break;
	case ExprType::Log10:
	{
		double arg = this->eval(expr.a);
		if (arg <= 0.0) {
			return -std::numeric_limits<double>::infinity();
		}
		return std::log10(arg);
	}
	break;
	case ExprType::Exp:
		return std::exp(this->eval(expr.a)); break;
	case ExprType::Sin:
		return std::sin(this->eval(expr.a)); break;
	case ExprType::Cos:
		return std::cos(this->eval(expr.a)); break;
	case ExprType::Tan:
		return std::tan(this->eval(expr.a)); break;
	case ExprType::ArcSin:
		return std::asin(this->eval(expr.a)); break;
	case ExprType::ArcCos:
		return std::acos(this->eval(expr.a)); break;
	case ExprType::ArcTan:
		return std::atan(this->eval(expr.a)); break;

	case ExprType::Branch:
		if (this->eval(expr.cond) >= 0) {
			return this->eval(expr.a);
		}
		else {
			return this->eval(expr.b);
		}
		break;

	default:
		std::cout << "symx error: Unhandled ExprType in Expressions.eval()" << std::endl;
		exit(-1);
		break;
	}
}
int64_t symx::Expressions::get_n_bytes() const
{
    return sizeof(Expr) * this->expressions.size();
}
std::string symx::Expressions::get_checksum(const std::string &in) const
{
	// Initialize hasher
	picosha2::hash256_one_by_one hasher;
    hasher.init();

	// Process input string
	hasher.process(in.begin(), in.end());

	// Process all expressions (flat)
	hasher.process(
		reinterpret_cast<const char*>(this->expressions.data()),
		reinterpret_cast<const char*>(this->expressions.data() + this->expressions.size())
	);

	// Finalize the hash
    hasher.finish();
    std::vector<unsigned char> out_hash(picosha2::k_digest_size);
    hasher.get_hash_bytes(out_hash.begin(), out_hash.end());
    std::string hex_str = picosha2::bytes_to_hex_string(out_hash.begin(), out_hash.end());

    return hex_str;
}
int32_t Expressions::_find_or_add(const Expr &expr)
{
	/*
		Based on the Common Subexpression Elimination strategy set, expressions are found (because they already existed) or added to the list.
	*/

	// No CSE
	if (get_cse_mode() == CSE::No) {
		int32_t expr_id = (int32_t)this->expressions.size();
		this->expressions.push_back(expr);
		return expr_id;
	}

	// Regardless of the CSE mode, check if the expression we are looking for is exactly the next one from the last found
	if (this->expressions.size() > this->last_cse_idx_checked + 1 && this->_are_same(expr, this->expressions[this->last_cse_idx_checked + 1])) {
		this->last_cse_idx_checked += 1;
		return this->last_cse_idx_checked;
	}

	// Pick a CSE strategy
	if (get_cse_mode() == CSE::Safe) {

		// Look for the expression in the hash map
		const std::array<int32_t, 4> key = { (int32_t)expr.type, expr.a, expr.b, expr.cond };
		auto it = this->cse_map.find(key);
		if (it == this->cse_map.end()) {
			// New expression
			int32_t expr_id = (int32_t)this->expressions.size();
			this->cse_map[key] = expr_id;
			this->expressions.push_back(expr);
			return expr_id;
		}
		else {
			// Already exists
			const int existing_idx = it->second;
			this->last_cse_idx_checked = existing_idx;
			return existing_idx;
		}
	}
	else {
		std::cout << "symx error: Unhandled CSE mode in Expressions._find_or_add()" << std::endl;
		exit(-1);
	}
}

bool Expressions::_are_same(const Expr& a, const Expr& b)
{
	return a.type == b.type && a.a == b.a && a.b == b.b && a.cond == b.cond;
}

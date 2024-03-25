#include "Expressions.h"

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

symx::Expressions::Expressions()
	:
	random_engine(std::random_device()()),
	random_distribution((uint64_t)std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint64_t>::max()) // Random hashes should not be too small -> [2^34, 2^64)
{
	// Zero and One are always at indices 0 and 1.
	this->_find_or_add(Expr(ExprType::Zero, -1, -1, this->random_distribution(this->random_engine)));
	this->_find_or_add(Expr(ExprType::One, -1, -1, this->random_distribution(this->random_engine)));
}
int32_t symx::Expressions::declare_symbol(std::string label)
{
	if (!containsOnlyASCII(label)) {
		std::cout << "symx error: Symbols labels must be only ASCII." << std::endl;
		exit(-1);
	}

	// Random hash
	uint64_t hash = 0;
	if (this->cse_mode == CSE::Greedy) {
		hash = this->random_distribution(this->random_engine);
	}

	// Create the symbol
	this->symbols.push_back(label);
	this->symbol_values_set.push_back(false);
	this->symbol_values.push_back(0.0);

	// Add it to the expression graph
	const int32_t symbol_id = (int32_t)this->symbols.size() - 1;
	const int32_t expr_id = (int32_t)this->expressions.size();
	this->expressions.push_back(Expr(ExprType::Symbol, symbol_id, -1, hash));

	return expr_id;
}
int32_t symx::Expressions::declare_constant_float(double constant)
{
	// Check it is zero
	if (std::abs(constant) < 10.0 * std::numeric_limits<double>::epsilon()) {
		return this->get_zero_idx();
	}

	// Check it is one
	else if (std::abs(constant - 1.0) < 10.0 * std::numeric_limits<double>::epsilon()) {
		return this->get_one_idx();
	}

	// Check if it is already declared
	else {
		// Create the Expr
		Expr expr(ExprType::ConstantFloat, 0, 0, 0);
		expr.pack_double(constant);
		if (this->cse_mode == CSE::Greedy) {
			expr.hash = symx::hash(expr.type, expr.a, expr.b);
		}
		return this->_find_or_add(expr);
	}
}
int32_t symx::Expressions::add_operation(ExprType type, int32_t a, int32_t b)
{
	if (!is_operation(type)) {
		std::cout << "symx error: cannot use Expressions::add_operation() for non-operation types." << std::endl;
		exit(-1);
	}

	uint64_t expr_hash = 0;
	if (this->cse_mode == CSE::Greedy) {
		switch (type) {
			// Algebraic hashes
		case ExprType::Add:
			expr_hash = this->expressions[a].hash + this->expressions[b].hash; break;
		case ExprType::Sub:
			expr_hash = this->expressions[a].hash - this->expressions[b].hash; break;
		case ExprType::Mul:
			expr_hash = this->expressions[a].hash * this->expressions[b].hash; break;

			// Expressions with two operands
		case ExprType::PowN:
			expr_hash = symx::hash(ExprType::PowN, this->expressions[a].hash, b); break;
		case ExprType::PowF:
			expr_hash = symx::hash(ExprType::PowF, this->expressions[a].hash, this->expressions[b].hash); break;

			// Expressions with one operand
		default:
			expr_hash = symx::hash(type, this->expressions[a].hash); break;
		}
	}

	return this->_find_or_add(Expr(type, a, b, expr_hash));
}
int32_t symx::Expressions::add_branch(int32_t cond_expr_id, int32_t positive_branch_expr_id, int32_t negative_branch_expr_id)
{
	int32_t expr_id = (int32_t)this->expressions.size();
	uint64_t expr_hash = 0;
	if (this->cse_mode == CSE::Greedy) {
		expr_hash = symx::hash(this->expressions[cond_expr_id].hash, this->expressions[positive_branch_expr_id].hash, this->expressions[negative_branch_expr_id].hash);
	}
	this->expressions.push_back(Expr(ExprType::Branch, positive_branch_expr_id, negative_branch_expr_id, expr_hash, cond_expr_id));
	return expr_id;
}
int32_t symx::Expressions::size() const
{
	return (int32_t)this->expressions.size();
}
int32_t symx::Expressions::get_n_symbols() const
{
	return (int32_t)this->symbols.size();
}
void symx::Expressions::set_cse_mode(CSE mode)
{
	if (this->size() != 2) {
		std::cout << "symx error: Cannot change the CSE mode after expressions have been added." << std::endl;
	}
	this->cse_mode = mode;
}
const std::vector<symx::Expr>& symx::Expressions::get_expressions() const
{
	return this->expressions;
}
const std::vector<std::string>& symx::Expressions::get_symbol_names() const
{
	return this->symbols;
}
int32_t symx::Expressions::get_zero_idx()
{
	return 0;
}
int32_t symx::Expressions::get_one_idx()
{
	return 1;
}
void symx::Expressions::set_value(int32_t expr_id, double val)
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

double symx::Expressions::get_value(int32_t expr_id) const
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

double symx::Expressions::eval(int32_t expr_id)
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
		return std::log(this->eval(expr.a)); break;
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
		return 0.0;
		break;
	}
}
int32_t symx::Expressions::_find_or_add(const Expr& expr)
{
	/*
		Based on the Common Subexpression Elimination strategy set, expressions are found (because they already existed) or added to the list.
	*/

	// No CSE
	if (this->cse_mode == CSE::No) {
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
	if (this->cse_mode == CSE::Safe) {

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
	else if (this->cse_mode == CSE::Greedy) {

		// Look for the expression in the hash map
		auto it = this->ahash_map.find(expr.hash);
		if (it == this->ahash_map.end()) {
			// New expression
			int32_t expr_id = (int32_t)this->expressions.size();
			this->expressions.push_back(expr);
			this->ahash_map[expr.hash] = expr_id;
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

bool symx::Expressions::_are_same(const Expr& a, const Expr& b)
{
	return a.type == b.type && a.a == b.a && a.b == b.b && a.cond == b.cond;
}

#include "Expressions.h"

bool containsOnlyASCII(const std::string& string) {
	for (auto c : string) {
		if (static_cast<unsigned char>(c) > 127) {
			return false;
		}
	}
	return true;
}

symx::Expressions::Expressions()
{
	/*
		Having the hashes of Zero and One being 0 and 1 respectively
		means that the algebraic hash equivalency will also work as
		simplification. When a number is multiplied by zero its hash
		will be multiplied by zero which will return the zero hash
		which in turn will fetch and return the expression zero.
	*/

	// Constants
	this->_find_or_add(Expr(ExprType::Zero, -1, -1, (uint64_t)0));
	this->_find_or_add(Expr(ExprType::One, -1, -1, (uint64_t)1));
}
int32_t symx::Expressions::declare_symbol(std::string label)
{
	if (!containsOnlyASCII(label)) {
		std::cout << "symx error: Symbols labels must be only ASCII." << std::endl;
		exit(-1);
	}

	// Random hash
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<uint64_t> distrib((uint64_t)std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint64_t>::max());
	uint64_t hash = distrib(gen);

	// Check the hash is not duplicated
	assert(this->ahash_map.find(hash) == this->ahash_map.end() && "Symbol hash already exist.");

	this->symbol_values_set.push_back(false);
	this->symbol_values.push_back(0.0);
	this->symbols.push_back(label);
	const int32_t symbol_id = (int32_t)this->symbols.size() - 1;
	const int32_t expr_id = (int32_t)this->expressions.size();
	this->expressions.push_back(Expr(ExprType::Symbol, symbol_id, -1, hash));
	return expr_id;
}
int32_t symx::Expressions::declare_constant_float(double constant)
{
	if (std::abs(constant) < 10.0*std::numeric_limits<double>::epsilon()) {
		return this->get_zero_idx();
	}
	else if (std::abs(constant - 1.0) < 10.0*std::numeric_limits<double>::epsilon()) {
		return this->get_one_idx();
	}
	else {
		int32_t low, high;
		pack_double(low, high, constant);
		return this->_find_or_add(Expr(ExprType::ConstantFloat, low, high, hash(ExprType::ConstantFloat, constant)));
	}
}
int32_t symx::Expressions::add_operation(ExprType type, int32_t a, int32_t b)
{
	if (!is_operation(type)) {
		std::cout << "symx error: cannot use Expressions::add_operation() for non-operation types." << std::endl;
		exit(-1);
	}

	uint64_t expr_hash;
	switch (type)
	{
	case ExprType::Add:
		expr_hash = this->expressions[a].hash + this->expressions[b].hash; break;
	case ExprType::Sub:
		expr_hash = this->expressions[a].hash - this->expressions[b].hash; break;
	case ExprType::Mul:
		expr_hash = this->expressions[a].hash * this->expressions[b].hash; break;
	case ExprType::PowN:
		expr_hash = hash(ExprType::PowN, this->expressions[a].hash, b); break;
	case ExprType::PowF:
		expr_hash = hash(ExprType::PowF, this->expressions[a].hash, this->expressions[b].hash); break;

	default:
		expr_hash = hash(type, this->expressions[a].hash); break;
	}

	return this->_find_or_add(Expr(type, a, b, expr_hash));
}
int32_t symx::Expressions::add_branch(int32_t cond_expr_id, int32_t positive_branch_expr_id, int32_t negative_branch_expr_id)
{
	int32_t expr_id = (int32_t)this->expressions.size();
	uint64_t expr_hash = hash(this->expressions[cond_expr_id].hash, this->expressions[positive_branch_expr_id].hash, this->expressions[negative_branch_expr_id].hash);
	this->expressions.push_back(Expr(ExprType::Branch, positive_branch_expr_id, negative_branch_expr_id, expr_hash, cond_expr_id));
	return expr_id;
}
int32_t symx::Expressions::size() const
{
	return (int32_t)this->expressions.size();
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
		std::cout << "symx error: Non-symbols fixed values not supported yet." << std::endl;
		exit(-1);
	}
	const int symbol_id = expr.a;
	this->symbol_values[symbol_id] = val;
	this->symbol_values_set[symbol_id] = static_cast<uint8_t>(true);
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
		return (double)unpack_double(expr.a, expr.b); break;
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
	if constexpr (Expressions::USE_EXPRESSION_COMPRESSION) {
		auto it = this->ahash_map.find(expr.hash);
		if (it != this->ahash_map.end()) {
			// Already exists
			return it->second;
		}
		else {
			// New expression
			int32_t expr_id = (int32_t)this->expressions.size();
			this->expressions.push_back(expr);
			this->ahash_map[expr.hash] = expr_id;
			return expr_id;
		}
	}
	else {
		int32_t expr_id = (int32_t)this->expressions.size();
		this->expressions.push_back(expr);
		return expr_id;
	}
}

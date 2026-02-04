#pragma once
#include <string>
#include <vector>

#include "Expr.h"
#include "unordered_array_map.h"

namespace symx
{
	// Common Subexpression Elimination strategies
	enum class CSE { No, Safe };
	extern CSE _cse_mode;
	void set_cse_mode(CSE mode = CSE::Safe);
	CSE get_cse_mode();

	/*
		Low level (internal) representation of symbolic expression graphs.
		Provides a way to declare symbols, constants, and operations, and to evaluate the resulting expression.
		It is interfaced via integers, which are used to refer to the expressions.

		Higher level abstractions are Scalar, Vector and Matrix.
	*/
	class Expressions
	{
		friend class Scalar;
	public:
		/* Methods */
		Expressions();
		int32_t declare_symbol(std::string label);
		int32_t declare_constant_float(double constant);
		int32_t add_operation(ExprType type, int32_t a, int32_t b);
		int32_t add_branch(int32_t cond_expr_id, int32_t positive_branch_expr_id, int32_t negative_branch_expr_id);
		int32_t get_zero_idx();
		int32_t get_one_idx();
		int32_t size() const;
		int32_t get_n_symbols() const;
		int32_t get_symbol_location(int32_t symbol_id) const;
		const std::vector<Expr>& get_expressions() const;
		const std::vector<std::string>& get_symbol_names() const;
		void set_value(int32_t expr_id, double val);
		double get_value(int32_t expr_id) const;
		double eval(int32_t expr_id) const;
		int64_t get_n_bytes() const;
		std::string get_checksum(const std::string& in = "") const;

	private:
		/* Fields */
		std::vector<Expr> expressions;  // Expression graph. Zero and One are always at indices 0 and 1.
		std::vector<std::string> symbols;
		std::vector<int32_t> symbol_locations;

		// Evaluation
		std::vector<double> symbol_values;
		std::vector<uint8_t> symbol_values_set;

		// CSE
		CSE cse_mode = CSE::Safe;
		unordered_array_map<int32_t, 4, uint32_t> cse_map; // common subexpression elimination map
		int last_cse_idx_checked = 0;

		// Differentiation Cache
		std::unordered_map<int64_t, Expr> diff_cache;

		/* Methods */
		int32_t _find_or_add(const Expr& expr);
		bool _are_same(const Expr& a, const Expr& b);
	};
}

#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <cassert>
#include <cstring>
#include <cmath>
#include <random>
#include <unordered_map>

#include "Expr.h"
#include "hashing.h"

namespace symx
{
	// Common Subexpression Elimination strategies
	enum class CSE { No, Safe, Greedy };

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
		void set_cse_mode(CSE mode = CSE::Safe);
		const std::vector<Expr>& get_expressions() const;
		const std::vector<std::string>& get_symbol_names() const;
		void set_value(int32_t expr_id, double val);
		double get_value(int32_t expr_id) const;
		double eval(int32_t expr_id);

	private:
		/* Fields */
		std::vector<Expr> expressions;  // Expression graph. Zero and One are always at indices 0 and 1.
		std::vector<std::string> symbols;

		// Evaluation
		std::vector<double> symbol_values;
		std::vector<uint8_t> symbol_values_set;

		// CSE
		CSE cse_mode = CSE::Safe;
		std::unordered_map<uint64_t, uint32_t> ahash_map;  // algebraic hash map [Herholtz et al 2022]
		unordered_array_map<int32_t, 4, uint32_t> cse_map; // common subexpression elimination map
		int last_cse_idx_checked = 0;

		//// Random engine for the CSE Greedy strategy
		std::mt19937 random_engine;
		std::uniform_int_distribution<uint64_t> random_distribution;

		/* Methods */
		int32_t _find_or_add(const Expr& expr);
		bool _are_same(const Expr& a, const Expr& b);
	};
}

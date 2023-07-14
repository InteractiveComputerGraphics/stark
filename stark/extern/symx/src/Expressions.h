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
	class Expressions
	{
	public:
		static constexpr bool USE_EXPRESSION_COMPRESSION = true;

		/* Fields */
		std::vector<std::string> symbols;
		std::vector<double> symbol_values;
		std::vector<uint8_t> symbol_values_set;
		std::vector<Expr> expressions;
		std::unordered_map<uint64_t, uint32_t> ahash_map;

		/* Methods */
		Expressions();
		int32_t declare_symbol(std::string label);
		int32_t declare_constant_float(double constant);
		int32_t add_operation(ExprType type, int32_t a, int32_t b);
		int32_t add_branch(int32_t cond_expr_id, int32_t positive_branch_expr_id, int32_t negative_branch_expr_id);
		int32_t size() const;
		void set_value(int32_t expr_id, double val);
		double eval(int32_t expr_id);
		int32_t get_zero_idx();
		int32_t get_one_idx();

	private:
		int32_t _find_or_add(const Expr& expr);
	};
}

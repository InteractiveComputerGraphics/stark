#pragma once
#include <iostream>
#include <vector>
#include <cassert>
#include <random>
#include <string>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "Scalar.h"
#include "Vector.h"
#include "Matrix.h"

namespace symx
{

	/*
	Representation:
		The mental data model (that will not be realized) is that all the indexing will happen
		in a single array having the following structure
			[{symbols}, {constants}, {tmp}]
		Output will be a set of indices to this array (for the corresponding tmp) values.

		The process is to traverse the graph from the outputs collecting operations bottom-up.
		Every time an expression is added to the operation lists, the index that holds its result
		is stored so, when it is used again, we can reuse the value and not create more operations.
	*/

	/*
	Branches:
		`FixedBranchSequence` generates a sequence of operations from the expression graph
		for fixed branch outcomes. `Sequence` will merge a collection of `FixedBranchSequence`
		into a single sequence.

		The generated sequence in `FixedBranchSequence` will explicitly contain only the first
		occurrence of each unique branch to make merging them easier. The reason for this is that
		when computing hessians, the number of branches will be N*M where N is the number of conditions
		in the original expression and M is the number of entries in the hessian matrix. However, we
		want to just branch once (as soon as we know the result of the condition) and then compute 
		different hessians on each branch without the need of a branch per entry.
	*/
	namespace core
	{
		struct Op
		{
			ExprType type;
			int32_t dst = -1;
			int32_t a = -1;
			int32_t b = -1;
			int32_t cond = -1;
			double constant = 0.0;  // In case the operation is a constant
			Op(const ExprType type, const int32_t dst, const int32_t a, const int32_t b, const double constant = 0.0, const int32_t cond = 0)
				: type(type), dst(dst), a(a), b(b), constant(constant), cond(cond) {};
			Op() = default;
			
			bool is_positive_branch() const
			{
				return this->a == 0;
			}
			bool is_negative_branch() const
			{
				return this->a == 1;
			}
			bool is_endif() const
			{
				return this->cond == -2;
			}

			static Op make_positive_branch(const int32_t cond_idx)
			{
				return Op(ExprType::Branch, -1, /* means positive branch*/ 0, -1, 0.0, cond_idx);
			}
			static Op make_negative_branch(const int32_t cond_idx)
			{
				return Op(ExprType::Branch, -1, /* means negative branch*/ 1, -1, 0.0, cond_idx);
			}
			static Op make_endif()
			{
				return Op(ExprType::Branch, -1, -1, -1, 0.0, -2);
			}

			static void print(const std::vector<Op>& ops);
			static std::vector<int> count_ops(std::vector<Op>& ops);
			static std::string count_ops_string(std::vector<Op>& ops, std::string pre_string = "");
		};

		class FixedBranchSequence
		{
		public:
			/* Fields */
			std::vector<Op> ops;
			std::vector<int> expr_evaluated_at;
			int n_inputs = -1;
			int n_outputs = -1;
			std::unordered_set<int32_t> found_branches;

			/* Methods */
			FixedBranchSequence(const std::vector<Scalar>& expr, const std::unordered_map<int, bool>& branch_combination = {});
			int get_n_inputs();
			int get_n_outputs();
			int get_n_variables();
			void print();

		private:
			bool _was_branch_found(const int32_t cond_idx);
			int32_t _gather_ops(const Scalar& scalar, const std::unordered_map<int, bool>& branch_combination);
		};
	}
}

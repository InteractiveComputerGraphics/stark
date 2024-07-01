#pragma once
#include <unordered_set>
#include <numeric>

#include "FixedBranchSequence.h"

namespace symx
{
	/*
		`Sequence` generates a sequence of expressions from an list of expressions.
		
	Branches:
		The challenge is to handle branches. Usually when computing hessians, the number 
		of branches will be N*M where N is the number of conditions	in the original 
		expression and M is the number of entries in the hessian matrix. However, we
		want to just branch once (as soon as we know the result of the condition) and then 
		compute different hessians on each branch without the need of a branch per entry.

		`Sequence` will identify the unique conditional branches in the expression tree
		and create a `FixedBranchSequence` per combination. Then, those will be merged
		into a single final sequence branching correspondingly.
	*/

	class Sequence
	{
	public:
		/* Fields */
		std::vector<core::Op> ops;
		int n_inputs = -1;
		int n_outputs = -1;

		/* Methods */
		Sequence(const std::vector<Scalar>& expr);
		int get_n_inputs() const;
		int get_n_outputs() const;
		std::string count_ops_string(std::string pre_string = "");

	private:
		std::vector<core::FixedBranchSequence> branch_sequences;
		void _add_ops_until_next_branch(const std::vector<int>& branches, int cursor);
	};
}
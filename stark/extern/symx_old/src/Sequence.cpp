#include "Sequence.h"

std::vector<std::vector<bool>> boolean_combinations(const int n)
{
	/*
		Combinations of the kind
			0, 0, 0,
			1, 0, 0,
			0, 1, 0,
			1, 1, 0,
			0, 0, 1,
			1, 0, 1,
			0, 1, 1,
			1, 1, 1,

		for 3 items, are the binary representation of the number 2^3.
	*/
	std::vector<std::vector<bool>> combinations;
	for (int i = 0; i < std::pow(2, n); i++) {
		std::vector<bool> row;
		for (int j = 0; j < n; j++) {
			uint32_t b = i;
			row.push_back((b << (31 - j) >> 31) == 1);
		}
		combinations.push_back(row);
	}
	return combinations;
}


symx::Sequence::Sequence(const std::vector<Scalar>& expr)
{
	/*
		Arguably, one could do this directly while building the sequences
		instead of build a sequence per branch and then merge. But that would
		complicate the code in the Sequence for a fixed branch combination
		(which is by far the most commmon case).

		I prefer to use post-process the output of that much simpler and well-tested
		functionality. This is not at all the bottleneck of any subprocess in the pipeline.
	*/

	const std::vector<Expr>& expressions = expr[0].get_expression_graph()->get_expressions();

	// Find unique branching points
	std::unordered_map<int32_t, Expr> unique_branch_conditions;
	for (const Expr& e : expressions) {
		if (e.type == ExprType::Branch) {
			unique_branch_conditions[e.cond] = e;
		}
	}
	std::vector<Expr> branches;
	for (auto& it : unique_branch_conditions) {
		branches.push_back(it.second);
	}
	const int n_branches = (int)branches.size();

	if (n_branches == 0) {
		this->branch_sequences.emplace_back(expr);
		this->ops.swap(this->branch_sequences[0].ops);
		this->n_inputs = this->branch_sequences[0].n_inputs;
		this->n_outputs = this->branch_sequences[0].n_outputs;
	}
	else {
		if (n_branches > 8) {
			std::cout << "symx error: Too unique many branches found in Sequence. Max allowed is 8 due to combinatorial explosion." << std::endl;
			exit(-1);
		}

		// Generate a sequence per unique branch combination
		std::unordered_map<int, bool> branch_combination_map;
		std::vector<std::vector<bool>> combinations = boolean_combinations(n_branches);
		for (const std::vector<bool>& combination : combinations) {
			for (int i = 0; i < n_branches; i++) {
				branch_combination_map[branches[i].cond] = combination[i];
			}
			this->branch_sequences.emplace_back(expr, branch_combination_map);
		}

		std::vector<int> iota(this->branch_sequences.size());
		std::iota(iota.begin(), iota.end(), 0);
		this->_add_ops_until_next_branch(iota, 0);
		this->n_inputs = this->branch_sequences[0].n_inputs;
		this->n_outputs = this->branch_sequences[0].n_outputs;
	}
}

int symx::Sequence::get_n_inputs() const
{
	return this->n_inputs;
}

int symx::Sequence::get_n_outputs() const
{
	return this->n_outputs;
}

std::string symx::Sequence::count_ops_string(std::string pre_string)
{
	return core::Op::count_ops_string(this->ops, pre_string);
}

void symx::Sequence::_add_ops_until_next_branch(const std::vector<int>& sequence_idxs, int cursor)
{
	auto& sequence = this->branch_sequences;
	const int n_sequences = (int)sequence_idxs.size();

	if (n_sequences == 1) {
		const int sequence_i = sequence_idxs[0];
		for (int i = cursor; i < (int)this->branch_sequences[sequence_i].ops.size(); i++) {

			if (sequence[sequence_i].ops[i].type == ExprType::Branch) {
				std::cout << "symx error: Single BranchSequences cannot contain more if statements." << std::endl;
				exit(-1);
			}

			this->ops.push_back(sequence[sequence_i].ops[i]);
		}
	}
	else {

		// All the sequence we have must have the same chain of expressions until one `if` is found in all sequence
		// or they are actually identical and we actually don't have more conditionals.
		// In any case, we can just append ops from the first branch until the next if or the end
		auto& sequence_0 = sequence[sequence_idxs[0]];
		const int sequence_0_size = (int)sequence_0.ops.size();
		bool found_a_branch = false;
		while (cursor < sequence_0_size) {

			if (sequence_0.ops[cursor].type == ExprType::Branch) {
				found_a_branch = true;
				break;
			}
			else {
				this->ops.push_back(sequence_0.ops[cursor]);
				cursor++;
			}
		}

		if (found_a_branch) {
			std::vector<int> positive_branch_idx;
			std::vector<int> negative_branch_idx;
			core::Op positive_if;
			core::Op else_if;
			for (int i = 0; i < n_sequences; i++) {
				const int sequence_i = sequence_idxs[i];

				if (sequence[sequence_i].ops[cursor].type != ExprType::Branch) {
					std::cout << "symx error: Sequence::_add_ops_until_next_branch() Branch sequences with different conditional topology found." << std::endl;
					exit(-1);
				}

				if (sequence[sequence_i].ops[cursor].is_positive_branch()) {
					positive_if = sequence[sequence_i].ops[cursor];
					positive_branch_idx.push_back(sequence_i);
				}
				else {
					else_if = sequence[sequence_i].ops[cursor];
					negative_branch_idx.push_back(sequence_i);
				}
			}
			cursor++;

			if (positive_branch_idx.size() == 0 || negative_branch_idx.size() == 0) {
				std::cout << "symx error: Sequence::_add_ops_until_next_branch() found an empty branch sequence." << std::endl;
				exit(-1);
			}


			// Add branch operation
			const int branch_idx= (int)this->ops.size();
			this->ops.push_back(positive_if);
			this->_add_ops_until_next_branch(positive_branch_idx, cursor);
			this->ops[branch_idx].b = (int)this->ops.size() + 1; // Beginning of the negative branch
			this->ops.push_back(else_if);
			this->_add_ops_until_next_branch(negative_branch_idx, cursor);
			this->ops.push_back(core::Op::make_endif());
		}
	}
}

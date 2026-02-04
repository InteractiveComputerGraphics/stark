#include "FixedBranchSequence.h"

void symx::core::Op::print(const std::vector<Op>& ops)
{
	for (auto& op : ops) {
		switch (op.type)
		{
		case ExprType::Symbol:
			std::cout << op.dst << " <- " << "out(" << op.a << ");" << std::endl; break;
		case ExprType::ConstantFloat:
			std::cout << op.dst << " <- " << "set(" << op.constant << ");" << std::endl; break;
		case ExprType::Add:
			std::cout << op.dst << " <- " << "add(" << op.a << ", " << op.b << ");" << std::endl; break;
		case ExprType::Sub:
			std::cout << op.dst << " <- " << "sub(" << op.a << ", " << op.b << ");" << std::endl; break;
		case ExprType::Mul:
			std::cout << op.dst << " <- " << "mul(" << op.a << ", " << op.b << ");" << std::endl; break;
		case ExprType::Reciprocal:
			std::cout << op.dst << " <- " << "inv(" << op.a << ");" << std::endl; break;
		case ExprType::PowN:
			std::cout << op.dst << " <- " << "powN(" << op.a << ", " << op.b << ");" << std::endl; break;
		case ExprType::PowF:
			std::cout << op.dst << " <- " << "powF(" << op.a << ", " << op.b << ");" << std::endl; break;
		case ExprType::Sqrt:
			std::cout << op.dst << " <- " << "sqrt(" << op.a << ");" << std::endl; break;
		case ExprType::Ln:
			std::cout << op.dst << " <- " << "log(" << op.a << ");" << std::endl; break;
		case ExprType::Exp:
			std::cout << op.dst << " <- " << "exp(" << op.a << ");" << std::endl; break;
		case ExprType::Sin:
			std::cout << op.dst << " <- " << "sin(" << op.a << ");" << std::endl; break;
		case ExprType::Cos:
			std::cout << op.dst << " <- " << "cos(" << op.a << ");" << std::endl; break;
		case ExprType::Tan:
			std::cout << op.dst << " <- " << "tan(" << op.a << ");" << std::endl; break;
		case ExprType::Branch:
			if (op.cond == -1) {
				std::cout << "}" << std::endl; break;
			}
			if (op.a == 0) {
				std::cout << "if (" << op.cond << " >= 0.0)\n{" << std::endl; break;
			}
			else if (op.a == 1) {
				std::cout << "}\nelse if (" << op.cond << " < 0.0)\n{" << std::endl; break;
			}

		default:
			std::cout << "symx error: Unhandled ExprType found in Op::print()." << std::endl;
			exit(-1);
		}
	}
}

std::vector<int> symx::core::Op::count_ops(std::vector<Op>& ops)
{
	std::vector<int> count(n_expr_types());
	for (const Op& op : ops) {
		count[static_cast<int>(op.type)]++;
	}
	return count;
}

std::string symx::core::Op::count_ops_string(std::vector<Op>& ops, std::string pre_string)
{
	std::string out;
	std::vector<int> count = Op::count_ops(ops);
	for (int i = 0; i < (int)count.size(); i++) {
		const ExprType type = static_cast<ExprType>(i);
		if (is_operation(type) && count[i] > 0) {
			const std::string label = get_label(type);
			out += pre_string + label + ": \t" + std::to_string(count[i]) + "\n";
		}
	}
	out += pre_string + "Total: \t" + std::to_string(ops.size()) + "\n";
	return out;
}



symx::core::FixedBranchSequence::FixedBranchSequence(const std::vector<Scalar>& expr, const std::unordered_map<int, bool>& branch_combination)
{
	// Expression graph size
	const int n_exprs_in_the_graph = expr[0].get_expression_graph()->size();
	const int n_symbols_in_the_graph = expr[0].get_expression_graph()->get_n_symbols();
	
	// Mapping of where each expression has been evaluated to. -1 if not evaluated yet.
	this->expr_evaluated_at.resize(n_exprs_in_the_graph, -1);

	// Run
	this->n_inputs = n_symbols_in_the_graph;
	this->n_outputs = (int)expr.size();
	this->ops.reserve(n_exprs_in_the_graph);
	for (int i = 0; i < (int)expr.size(); i++) {
		const int expr_sol_idx = this->_gather_ops(expr[i], branch_combination);
		this->ops.push_back(Op(ExprType::Symbol, i, expr_sol_idx, -1)); // We use ExprType::Symbol to indicate move solution to output
	}
}
int symx::core::FixedBranchSequence::get_n_inputs()
{
	return this->n_inputs;
}
int symx::core::FixedBranchSequence::get_n_outputs()
{
	return this->n_outputs;
}
int symx::core::FixedBranchSequence::get_n_variables()
{
	return this->ops.back().dst + 1;
}
void symx::core::FixedBranchSequence::print()
{
	core::Op::print(this->ops);
}
bool symx::core::FixedBranchSequence::_was_branch_found(const int32_t cond_idx)
{
	if (this->found_branches.find(cond_idx) == this->found_branches.end()) {
		this->found_branches.insert(cond_idx);
		return false;
	}
	else {
		return true;
	}
}
int32_t symx::core::FixedBranchSequence::_gather_ops(const Scalar& scalar, const std::unordered_map<int, bool>& branch_combination)
{
	// Expression already evaluated
	if (this->expr_evaluated_at[scalar.expr_id] != -1) {
		const int32_t at = this->expr_evaluated_at[scalar.expr_id];
		return at;
	}

	// New expression
	else {

		// Value
		if (!is_operation(scalar.expr.type)) {
			if (scalar.expr.type == ExprType::Symbol) {
				return scalar.expr.a;
			}
			else if (scalar.expr.type == ExprType::Zero) {
				const int32_t pos = (int32_t)this->ops.size() + this->n_inputs;
				this->ops.emplace_back(ExprType::ConstantFloat, pos, -1, -1, 0.0);
				this->expr_evaluated_at[scalar.expr_id] = pos;
				return pos;
			}
			else if (scalar.expr.type == ExprType::One) {
				const int32_t pos = (int32_t)this->ops.size() + this->n_inputs;
				this->ops.emplace_back(ExprType::ConstantFloat, pos, -1, -1, 1.0);
				this->expr_evaluated_at[scalar.expr_id] = pos;
				return pos;
			}
			else if (scalar.expr.type == ExprType::ConstantFloat) {
				const int32_t pos = (int32_t)this->ops.size() + this->n_inputs;
				this->ops.emplace_back(ExprType::ConstantFloat, pos, -1, -1, scalar.expr.unpack_double());
				this->expr_evaluated_at[scalar.expr_id] = pos;
				return pos;
			}
			else if (scalar.expr.type == ExprType::Branch) {
				// Deterministically traverse a specific branch combination
				// Note: Only the first occurrence of a branch is added to the ops
				const int32_t cond_idx = this->_gather_ops(scalar.get_condition(), branch_combination);
				const bool was_already_found = this->_was_branch_found(cond_idx);

				if (branch_combination.at(scalar.expr.cond) == true) {
					if (!was_already_found) {
						this->ops.push_back(Op::make_positive_branch(cond_idx));
					}
					return this->_gather_ops(scalar.left(), branch_combination);
				}
				else {
					if (!was_already_found) {
						this->ops.emplace_back(Op::make_negative_branch(cond_idx));
					}
					return this->_gather_ops(scalar.right(), branch_combination);
				}
				// End of the branch -> end of the program (we dont merge after branches)
			}
		}

		// Operation
		else {
			if (scalar.expr.type == ExprType::PowN) {
				const int32_t idx_left = this->_gather_ops(scalar.left(), branch_combination);
				const int32_t pos = (int32_t)this->ops.size() + this->n_inputs;
				this->ops.emplace_back(scalar.expr.type, pos, idx_left, scalar.expr.b);
				this->expr_evaluated_at[scalar.expr_id] = pos;
				return pos;
			}
			else if (scalar.expr.b == -1) {
				const int32_t idx_left = this->_gather_ops(scalar.left(), branch_combination);
				const int32_t pos = (int32_t)this->ops.size() + this->n_inputs;
				this->ops.emplace_back(scalar.expr.type, pos, idx_left, -1);
				this->expr_evaluated_at[scalar.expr_id] = pos;
				return pos;
			}
			else {
				const int32_t idx_left = this->_gather_ops(scalar.left(), branch_combination);
				const int32_t idx_right = this->_gather_ops(scalar.right(), branch_combination);
				const int32_t pos = (int32_t)this->ops.size() + this->n_inputs;
				this->ops.emplace_back(scalar.expr.type, pos, idx_left, idx_right);
				this->expr_evaluated_at[scalar.expr_id] = pos;
				return pos;
			}
		}


		std::cout << "symx error: Operation not handled in Sequence." << std::endl;
		exit(-1);

		return -1;
	}
}


#include "EvalSequence.h"

symx::EvalSequence::EvalSequence(const std::vector<Scalar>& expr)
	: seq(expr)
{
	this->output.resize(this->seq.get_n_outputs());
}

void symx::EvalSequence::set(const Scalar& symbol, const double& v)
{
	assert(symbol.expr.a < this->seq.get_n_inputs() && "symx error: symx::Sequence.set() symbol idx higher than buffer size.");
	this->buffer[symbol.expr.a] = v;
}

void symx::EvalSequence::set(const Vector& vector, const double* v)
{
	for (int i = 0; i < vector.size(); i++) {
		this->set(vector[i], v[i]);
	}
}

void symx::EvalSequence::set(const Matrix& matrix, const double* v)
{
	for (int i = 0; i < matrix.values().size(); i++) {
		this->set(matrix.values()[i], v[i]);
	}
}

double* symx::EvalSequence::run()
{
	for (int i = 0; i < this->seq.n_inputs; i++) {
		if (std::isnan(this->buffer[i])) {
			std::cout << "symx error: EvalSequence::run() found not set input symbols." << std::endl;
			exit(-1);
		}
	}

	const double EPS = 100.0 * std::numeric_limits<double>::epsilon();
	for (int i = 0; i < (int)this->seq.ops.size(); i++) {
		const auto& op = this->seq.ops[i];
		switch (op.type)
		{
		// Output
		case ExprType::Symbol:
			this->output[op.dst] = this->buffer[op.a]; break;
		
		// Branch
		case ExprType::Branch:
			if (op.is_endif() || op.is_negative_branch()) {
				i = (int)this->seq.ops.size();  // Found the end of the branch. End of the execution.
				break;
			}
			else if (this->buffer[op.cond] >= 0) {
				break; // Next operation is the beginning of the positive branch
			}
			else {
				i = op.b - 1;  // Jump to the negative branch
				break;
			}

		// Else
		case ExprType::ConstantFloat:
			this->buffer[op.dst] = op.constant; break;
		case ExprType::Add:
			this->buffer[op.dst] = this->buffer[op.a] + this->buffer[op.b]; break;
		case ExprType::Sub:
			this->buffer[op.dst] = this->buffer[op.a] - this->buffer[op.b]; break;
		case ExprType::Mul:
			this->buffer[op.dst] = this->buffer[op.a] * this->buffer[op.b]; break;
		case ExprType::Reciprocal:
			if (std::abs(this->buffer[op.a]) < EPS) {
				std::cout << "symx error: Division by zero found evaluating an expression." << std::endl;
				exit(-1);
			}
			this->buffer[op.dst] = 1.0 / this->buffer[op.a]; break;
		case ExprType::PowN:
			this->buffer[op.dst] = std::pow(this->buffer[op.a], op.b); break;
		case ExprType::PowF:
			this->buffer[op.dst] = std::pow(this->buffer[op.a], this->buffer[op.b]); break;
		case ExprType::Sqrt:
			if (this->buffer[op.a] < 0.0) {
				std::cout << "symx error: sqrt of a negative number found evaluating an expression." << std::endl;
				exit(-1);
			}
			this->buffer[op.dst] = std::sqrt(this->buffer[op.a]); break;
		case ExprType::Ln:
			if (this->buffer[op.a] < EPS) {
				std::cout << "symx error: log of zero or a negative number found evaluating an expression." << std::endl;
				exit(-1);
			}
			this->buffer[op.dst] = std::log(this->buffer[op.a]); break;
		case ExprType::Exp:
			this->buffer[op.dst] = std::exp(this->buffer[op.a]); break;
		case ExprType::Sin:
			this->buffer[op.dst] = std::sin(this->buffer[op.a]); break;
		case ExprType::Cos:
			this->buffer[op.dst] = std::cos(this->buffer[op.a]); break;
		case ExprType::Tan:
			this->buffer[op.dst] = std::tan(this->buffer[op.a]); break;
		case ExprType::ArcSin:
			this->buffer[op.dst] = std::asin(this->buffer[op.a]); break;
		case ExprType::ArcCos:
			this->buffer[op.dst] = std::acos(this->buffer[op.a]); break;
		case ExprType::ArcTan:
			this->buffer[op.dst] = std::atan(this->buffer[op.a]); break;
	
		default:
			break;
		}
	}
	return this->output.data();
}

#pragma once
#include <unordered_map>
#include "Sequence.h"

namespace symx
{
	class EvalSequence
	{
	public:
		/* Fields */
		Sequence seq;
		std::unordered_map<int, double> buffer;
		std::vector<double> output;

		/* Methods */
		EvalSequence(const std::vector<Scalar>& expr);
		void set(const Scalar &symbol, const double& v);
		void set(const Vector &vector, const double* v);
		void set(const Matrix&matrix, const double* v);
		double* run();
	};
}
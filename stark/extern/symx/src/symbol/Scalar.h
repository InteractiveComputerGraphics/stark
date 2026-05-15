#pragma once
#include <unordered_set>

#include <picoSHA2/picosha2.h>
#include "Expressions.h"


namespace symx
{
	/*
	* Symbolic scalar value. Interfaces manipulating the expression graph
	* with nice readable operator overloads.
	*/
	class Scalar
	{
	friend class Workspace;

	private:
		/* Methods */
		Scalar(const int32_t expr_id, Expressions* expressions)
			: expr_id(expr_id), expressions(expressions) 
		{
			this->expr = this->expressions->expressions[this->expr_id];
		};

	public:
		/* Fields */
		int32_t expr_id;
		Expr expr;
		Expressions* expressions = nullptr;

		/* Methods */
		Scalar& operator=(const Scalar& other);

		// Operations
		Scalar operator+(double val) const;
		Scalar operator-(double val) const;
		Scalar operator*(double val) const;
		Scalar operator/(double val) const;
		void operator+=(double val);
		void operator-=(double val);
		void operator*=(double val);
		void operator/=(double val);
		Scalar operator+(const Scalar& other) const;
		Scalar operator-(const Scalar& other) const;
		Scalar operator*(const Scalar& other) const;
		Scalar operator/(const Scalar& other) const;
		void operator+=(const Scalar& other);
		void operator-=(const Scalar& other);
		void operator*=(const Scalar& other);
		void operator/=(const Scalar& other);
		Scalar powN(int32_t val) const;
		Scalar powF(double val) const;
		Scalar inv() const;
		Scalar sqrt() const;
		Scalar log() const;  // Keep for backward compatibility (natural log)
		Scalar ln() const;   // Natural logarithm (explicit)
		Scalar log10() const; // Base-10 logarithm
		Scalar exp() const;
		Scalar sin() const;
		Scalar cos() const;
		Scalar tan() const;
		Scalar asin() const;
		Scalar acos() const;
		Scalar atan() const;
		Scalar print() const;

		// Functionalities
		bool is_zero() const;
		bool is_one() const;
		bool is_symbol() const;
		void set_value(const double val);
		double get_value() const;
		double eval() const;
		std::string get_checksum() const;  // Computes a reproducible hash from the section of the expression graph relevant to this Scalar

		// Expression tree traversal and inspection
		Scalar left() const;
		Scalar right() const;
		bool has_right() const;
		bool has_not_right() const;
		bool has_branch() const;          // True if this node is a Branch (conditional) expression
		int32_t get_symbol_idx() const;   // Index of this symbol in the Workspace symbol table (valid only when is_symbol() is true)
		const Expressions* get_expression_graph() const;

		// Get/Make
		Scalar get_zero() const;
		Scalar get_one() const;
		Scalar get_condition() const;     // Condition scalar of a Branch node
		Scalar make_constant(const double val) const;
		Scalar make_branch(const Scalar& condition, const Scalar& positive_branch, const Scalar& negative_branch) const;


	private:
		void _get_checksum(picosha2::hash256_one_by_one& hasher, std::unordered_set<int32_t>& visited) const;
	};

	// Operations
	Scalar operator+(double val, Scalar scalar);
	Scalar operator-(double val, Scalar scalar);
	Scalar operator*(double val, Scalar scalar);
	Scalar operator/(double val, Scalar scalar);
	Scalar operator-(const Scalar& scalar);
	Scalar powN(const Scalar& scalar, int32_t val);
	Scalar powF(const Scalar& scalar, double val);
	Scalar sqrt(const Scalar& scalar);
	Scalar log(const Scalar& scalar);  // Keep for backward compatibility (natural log)
	Scalar ln(const Scalar& scalar);   // Natural logarithm (explicit)
	Scalar log10(const Scalar& scalar); // Base-10 logarithm
	Scalar exp(const Scalar& scalar);
	Scalar sin(const Scalar& scalar);
	Scalar cos(const Scalar& scalar);
	Scalar tan(const Scalar& scalar);
	Scalar asin(const Scalar& scalar);
	Scalar acos(const Scalar& scalar);
	Scalar atan(const Scalar& scalar);

	// Conditionals
	Scalar branch(const Scalar& condition, const Scalar& positive_branch, const Scalar& negative_branch);
	Scalar operator>(const Scalar& a, const Scalar& b);
	Scalar operator<(const Scalar& a, const Scalar& b);
	Scalar min(const Scalar& a, const Scalar& b);
	Scalar max(const Scalar& a, const Scalar& b);
	Scalar abs(const Scalar& a);
	Scalar sign(const Scalar& a);

	// Convenient conditional alternatives
	Scalar branch(const Scalar& cond, const double positive_branch, const Scalar& negative_branch);
	Scalar branch(const Scalar& cond, const Scalar& positive_branch, const double negative_branch);
	Scalar branch(const Scalar& cond, const double positive_branch, const double negative_branch);
	Scalar operator>(const double& a, const Scalar& b);
	Scalar operator>(const Scalar& a, const double& b);
	Scalar operator<(const double& a, const Scalar& b);
	Scalar operator<(const Scalar& a, const double& b);
}

#pragma once
#include <vector>
#include <string>
#include <cstdint>
#include <array>
#include <cstring>
#include <cassert>

namespace symx
{
	constexpr uint8_t uint8_true = static_cast<uint8_t>(true);
	constexpr uint8_t uint8_false = static_cast<uint8_t>(false);

	enum class ExprType
	{
		// Identities
		Zero = 0,
		One = 1,
		Branch = 2,

		// Numbers
		ConstantFloat = 4,
		Symbol = 5,

		// Operations
		Add = 6,
		Sub = 7,
		Mul = 8,
		Reciprocal = 9,
		PowN = 10,
		PowF = 11,
		Sqrt = 12,
		Ln = 13,
		Exp = 14,
		Sin = 15,
		Cos = 16,
		Tan = 17,
		ArcSin = 18,
		ArcCos = 19,
		ArcTan = 20,
		Print = 21,

		EnumCount = 22 // This entry serves as a marker
	};
	constexpr static bool is_operation(const ExprType& type)
	{
		return static_cast<int32_t>(ExprType::Add) <= static_cast<int32_t>(type)
			&& static_cast<int32_t>(type) < static_cast<int32_t>(ExprType::EnumCount);
	}
	constexpr static int n_expr_types()
	{
		return static_cast<int32_t>(ExprType::EnumCount);
	}
	static std::vector<std::string> get_expr_type_labels()
	{
		std::vector<std::string> labels = { "Zero", "One", "Branch", "ConstantInteger", "ConstantFloat", "Symbol", "Add", "Sub", "Mul", "Inv", "PowN", "PowF", "Sqrt", "Log", "Exp", "Sin", "Cos", "Tan", "ArcSin", "ArcCos", "ArcTan", "Print" };
		assert(labels.size() == n_expr_types());
		return labels;
	}

	struct Expr
	{
		ExprType type;
		int32_t a = -1;
		int32_t b = -1;
		uint64_t hash = 0;
		int32_t cond = 0; // Used for ExprType::Branch
		Expr(const ExprType type, const int32_t a, const int32_t b, const uint64_t hash, const uint32_t cond = 0)
			: type(type), a(a), b(b), hash(hash), cond(cond) {};
		Expr() = default;
	};
	inline void pack_double(int32_t& a, int32_t& b, const double val)
	{
		memcpy(&a, &val, 4);
		memcpy(&b, (char*)(&val) + 4, 4);
	}
	inline double unpack_double(int32_t a, int32_t b)
	{
		double val;
		memcpy(&val, &a, 4);
		memcpy((char*)(&val) + 4, &b, 4);
		return val;
	}
}

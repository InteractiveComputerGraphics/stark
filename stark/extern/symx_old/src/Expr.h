#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <cstdint>
#include <array>
#include <cstring>
#include <cassert>

namespace symx
{
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
	constexpr static int n_expr_types()
	{
		return static_cast<int32_t>(ExprType::EnumCount);
	}
	constexpr static bool is_operation(const ExprType& type)
	{
		assert(static_cast<int32_t>(type) < static_cast<int32_t>(ExprType::EnumCount) && "Invalid ExprType");
		return static_cast<int32_t>(ExprType::Add) <= static_cast<int32_t>(type)
			&& static_cast<int32_t>(type) < static_cast<int32_t>(ExprType::EnumCount);
	}
	static std::string get_label(const ExprType& type)
	{
		switch (type)
		{
		case ExprType::Zero:
			return "Zero";
		case ExprType::One:
			return "One";
		case ExprType::Branch:
			return "Branch";
		case ExprType::ConstantFloat:
			return "ConstantFloat";
		case ExprType::Symbol:
			return "Symbol";
		case ExprType::Add:
			return "Add";
		case ExprType::Sub:
			return "Sub";
		case ExprType::Mul:
			return "Mul";
		case ExprType::Reciprocal:
			return "Reciprocal";
		case ExprType::PowN:
			return "PowN";
		case ExprType::PowF:
			return "PowF";
		case ExprType::Sqrt:
			return "Sqrt";
		case ExprType::Ln:
			return "Ln";
		case ExprType::Exp:
			return "Exp";
		case ExprType::Sin:
			return "Sin";
		case ExprType::Cos:
			return "Cos";
		case ExprType::Tan:
			return "Tan";
		case ExprType::ArcSin:
			return "ArcSin";
		case ExprType::ArcCos:
			return "ArcCos";
		case ExprType::ArcTan:
			return "ArcTan";
		case ExprType::Print:
			return "Print";
		default:
			std::cout << "symx error: Invalid ExprType: " << static_cast<int32_t>(type) << std::endl;
			exit(-1);
			break;
		}
	}
	struct Expr
	{
		ExprType type;
		int32_t a = -1;
		int32_t b = -1;
		uint64_t hash = 0;
		int32_t cond = 0; // Used for ExprType::Branch
		Expr(const ExprType type, const int32_t a, const int32_t b, const uint64_t hash = 0, const uint32_t cond = 0)
			: type(type), a(a), b(b), hash(hash), cond(cond) {};
		Expr() = default;

		inline bool operator==(const Expr& other) const
		{
			return 
				this->type == other.type && 
				this->a == other.a && 
				this->b == other.b && 
				this->cond == other.cond;
		}
		inline bool operator!=(const Expr& other) const
		{
			return !(*this == other);
		}
		inline bool is_zero() const
		{
			return this->type == ExprType::Zero;
		}
		inline bool is_one() const
		{
			return this->type == ExprType::One;
		}
		inline bool is_operation() const
		{
			return symx::is_operation(this->type);
		}
		inline std::string get_expr_label() const
		{
			return get_label(this->type);
		}
		inline void pack_double(const double val)
		{
			memcpy(&this->a, &val, 4);
			memcpy(&this->b, (char*)(&val) + 4, 4);
		}
		inline double unpack_double() const
		{
			double val;
			memcpy(&val, &this->a, 4);
			memcpy((char*)(&val) + 4, &this->b, 4);
			return val;
		}
	};
}

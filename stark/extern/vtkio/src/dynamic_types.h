#pragma once
#include <array>
#include <string>
#include <type_traits>

namespace dynamic_types
{
	// Tables =====================================================================================
	enum class DataType
	{
		Char = 0,
		Int16 = 1,
		Int32 = 2,
		Int64 = 3,
		Uint16 = 4,
		Uint32 = 5,
		Uint64 = 6,
		Float32 = 7,
		Float64 = 8,
		Undefined = 9
	};
	constexpr std::array<size_t, 10> DATA_TYPE_SIZE = { 1, 2, 4, 8, 2, 4, 8, 4, 8, 0 };

	// Functions =====================================================================================
	constexpr size_t getDataTypeSize(const DataType data_type)
	{
		return DATA_TYPE_SIZE[static_cast<size_t>(data_type)];
	}
	template<typename T>
	constexpr bool isNumberTypeSupported()
	{
		return  std::is_same_v<T, int16_t> ||
			std::is_same_v<T, int32_t> ||
			std::is_same_v<T, int64_t> ||
			std::is_same_v<T, uint16_t> ||
			std::is_same_v<T, uint32_t> ||
			std::is_same_v<T, uint64_t> ||
			std::is_same_v<T, float> ||
			std::is_same_v<T, double>;
	}
	template<typename T>
	constexpr bool isTypeSupported()
	{
		return std::is_same<T, char>::value || isNumberTypeSupported<T>();
	}
	template<typename T>
	constexpr void throwIfNumberTypeNotSupported()
	{
		if constexpr (!isNumberTypeSupported<T>()) {
			static_assert(!std::is_same<T, T>::value, "Number type not supported.");
		}
	}
	template<typename T>
	constexpr void throwIfTypeNotSupported()
	{
		if constexpr (!isTypeSupported<T>()) {
			static_assert(!std::is_same<T, T>::value, "Data type not supported.");
		}
	}
	template<typename T>
	constexpr DataType getDataEnumType()
	{
		throwIfTypeNotSupported<T>();

		if		constexpr (std::is_same_v<T, char>) { return DataType::Char; }
		else if constexpr (std::is_same_v<T, int16_t>) { return DataType::Int16; }
		else if constexpr (std::is_same_v<T, int32_t>) { return DataType::Int32; }
		else if constexpr (std::is_same_v<T, int64_t>) { return DataType::Int64; }
		else if constexpr (std::is_same_v<T, uint16_t>) { return DataType::Uint16; }
		else if constexpr (std::is_same_v<T, uint32_t>) { return DataType::Uint32; }
		else if constexpr (std::is_same_v<T, uint64_t>) { return DataType::Uint64; }
		else if constexpr (std::is_same_v<T, float>) { return DataType::Float32; }
		else if constexpr (std::is_same_v<T, double>) { return DataType::Float64; }
		else { } // This line is never reached
	}
	inline std::string getDataTypeString(DataType data_type)
	{
		switch (data_type)
		{
		case dynamic_types::DataType::Char: return "char"; break;
		case dynamic_types::DataType::Int16: return "int16_t"; break;
		case dynamic_types::DataType::Int32: return "int32_t"; break;
		case dynamic_types::DataType::Int64: return "int64_t"; break;
		case dynamic_types::DataType::Uint16: return "uint16_t"; break;
		case dynamic_types::DataType::Uint32: return "uint32_t"; break;
		case dynamic_types::DataType::Uint64: return "uint64_t"; break;
		case dynamic_types::DataType::Float32: return "float"; break;
		case dynamic_types::DataType::Float64: return "double"; break;
		case dynamic_types::DataType::Undefined: return "undefined"; break;
		default:
			return " "; // This is never reached
		}
	}
	template<typename T>
	inline std::string getDataTypeString()
	{
		return getDataTypeString(getDataEnumType<T>());
	}

	// Non-trivial iterators =====================================================================================
	template<typename T>
	struct Nested2Iterator
	{
		using container_type = typename std::iterator_traits<T>::value_type;
		using nested_value_type = typename std::iterator_traits<typename container_type::iterator>::value_type;
	};
}
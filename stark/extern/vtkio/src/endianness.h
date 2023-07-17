#pragma once
#include <vector>
#include <string>   // memcpy
#include <cstdint>  // integer types
#include <utility>  // std::swap

namespace endianness
{
	// Tables =====================================================================================
	enum class ByteOrder
	{
		Big,
		Little,
		Native
	};

	// Functions =====================================================================================
	inline ByteOrder getNativeEndianness(void)
	{
		union {
			uint32_t i;
			char c[4];
		} foo = { 0x01020304 };

		if (foo.c[0] == 1) {
			return ByteOrder::Big;
		}
		else {
			return ByteOrder::Little;
		}
	}
	inline bool sameEndianness(const ByteOrder byte_order_a, const ByteOrder byte_order_b)
	{
		const ByteOrder a = (byte_order_a == ByteOrder::Native) ? getNativeEndianness() : byte_order_a;
		const ByteOrder b = (byte_order_b == ByteOrder::Native) ? getNativeEndianness() : byte_order_b;
		return a == b;
	}
	template<size_t N_BYTES_PER_ELEMENT>
	inline void swapByteArrayInplace(char* arr, const size_t n)
	{
		constexpr unsigned int HALF_N = N_BYTES_PER_ELEMENT / 2;
		for (size_t e = 0; e < n; e++) {
			const size_t offset_0 = e * N_BYTES_PER_ELEMENT;
			const size_t offset_1 = (e + 1) * N_BYTES_PER_ELEMENT;
			for (int w = 0; w < HALF_N; w++) {
				std::swap(arr[offset_0 + w], arr[offset_1 - 1 - w]);
			}
		}
	}
	template<size_t N_BYTES_PER_ELEMENT>
	inline void swapByteArrayInplace(std::vector<char> &arr)
	{
		const size_t n_items = arr.size() / N_BYTES_PER_ELEMENT;
		swapByteArrayInplace<N_BYTES_PER_ELEMENT>(arr.data(), n_items);
	}
	template<class T>
	inline void swapBytesInplace(T* arr, const int n)
	{
		swapBytesInplace<sizeof(T)>(reinterpret_cast<char*>(arr), n);
	}
	template<class T>
	inline void swapBytesInplace(std::vector<T> &arr)
	{
		swapBytesInplace(arr.data(), arr.size());
	}
	template<class T>
	inline std::vector<T> swap_bytes(const std::vector<T> &arr)
	{
		std::vector<T> out = arr;
		swapBytesInplace(out.data(), out.size());
		return out;
	}
	/* Warning: Assumes the input is in native endianness */
	template<typename T_in, typename T_out>
	inline void castBytesInPlace(std::vector<char>& arr)
	{
		if constexpr (std::is_same<T_in, T_out>::value) {
			return;
		}

		const size_t size_in = sizeof(T_in);
		const size_t size_out = sizeof(T_out);
		const size_t n_char_in = arr.size();
		const size_t n_items = n_char_in / size_in;
		const size_t n_char_out = n_items * size_out;

		T_in* cast_in;
		T_out* cast_out;
		if constexpr (size_in >= size_out) {
			cast_in = reinterpret_cast<T_in*>(arr.data());
			cast_out = reinterpret_cast<T_out*>(arr.data());
			for (size_t i = 0; i < n_items; i++) {
				cast_out[i] = static_cast<T_out>(cast_in[i]);
			}
		}
		else {
			arr.resize(n_char_out);
			cast_in = reinterpret_cast<T_in*>(arr.data() + n_char_out - n_char_in);
			cast_out = reinterpret_cast<T_out*>(arr.data());
			for (size_t i = 0; i < n_items; i++) {
				cast_out[n_items - 1 - i] = static_cast<T_out>(cast_in[i]);
			}
		}

		arr.resize(n_char_out);
	}
}
#pragma once
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <cstring>
#include <iterator>

#include "dynamic_types.h"
#include "endianness.h"
#include "stream_io.h"

namespace bytebuffer
{
	using namespace dynamic_types;
	using namespace endianness;

	// Declarations =====================================================================================
	class ByteBuffer
	{
	protected:
		DataType type = DataType::Undefined;
		ByteOrder byte_order = ByteOrder::Native;
		std::vector<char> data;

	public:
		ByteBuffer() {};
		~ByteBuffer() {};

		// Set data
		template<class Indexable>
		void setFromIndexable(const Indexable& indexable, const ByteOrder byte_order = ByteOrder::Native);
		template<class Indexable>
		void setFromIndexable(const Indexable& indexable, const size_t begin, const size_t end, const ByteOrder byte_order = ByteOrder::Native);

		template<class TwiceIndexable>
		void setFromTwiceIndexable(const TwiceIndexable& twice_indexable, const ByteOrder byte_order = ByteOrder::Native);
		template<class TwiceIndexable>
		void setFromTwiceIndexable(const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const ByteOrder byte_order = ByteOrder::Native);

		void setFromBinaryStream(std::ifstream& input, const DataType data_type, const size_t n_bytes, const ByteOrder byte_order = ByteOrder::Native);
		void setFromASCIIStream(std::ifstream& input, const DataType data_type, const size_t n_items, const char sep);

		// Get
		template<class Indexable>
		void getToIndexable(Indexable& indexable, const ByteOrder byte_order = ByteOrder::Native);
		template<class Indexable>
		void getToIndexable(Indexable& indexable, const size_t begin, const size_t end, const ByteOrder byte_order = ByteOrder::Native);

		template<class TwiceIndexable>
		void getToTwiceIndexable(TwiceIndexable& twice_indexable, const ByteOrder byte_order = ByteOrder::Native);
		template<class TwiceIndexable>
		void getToTwiceIndexable(TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const ByteOrder byte_order = ByteOrder::Native);

		void toStreamBinary(std::ofstream& output, const ByteOrder byte_order = ByteOrder::Native);
		void toStreamASCII(std::ofstream& output, const char sep = ' ');

		template<typename T>
		T* getDataPtr();
		char* getDataPtrChar();


		// Important functionalities
		void swap();
		void swap(const ByteOrder byte_order);
		template<typename T>
		void castInplace();
		template<typename T>
		ByteBuffer castCopy();

		// Other functions
		DataType getDataType();
		ByteOrder getByteOrder();
		size_t getNumberOfBytes();
		size_t getNumberOfItems();
		template<typename T>
		bool isType();
		bool isType(DataType data_type);
		template<typename T>
		void throwIfNotType();
		void clear();

	private:
		void setByteOrder(const ByteOrder byte_order);
		void setDataType(const DataType data_type);
	};



	// Definitions =====================================================================================
	inline void ByteBuffer::setFromBinaryStream(std::ifstream& input, const DataType data_type, const size_t n_bytes, const ByteOrder byte_order)
	{
		this->setDataType(data_type);
		this->setByteOrder(byte_order);
		this->data.clear();
		this->data.resize(n_bytes);
		auto& success = input.read(this->data.data(), n_bytes);
		if (!success) {
			std::cout << "ByteBuffer error: Could not read from file." << std::endl; exit(-1);
		}
	}
	inline void ByteBuffer::setFromASCIIStream(std::ifstream& input, const DataType data_type, const size_t n_items, const char sep)
	{
		this->setDataType(data_type);
		this->setByteOrder(ByteOrder::Native);
		this->data.clear();
		const size_t n_bytes = n_items * getDataTypeSize(data_type);
		this->data.resize(n_bytes);
		switch (this->type)
		{
		case DataType::Int16: memcpy(this->data.data(), stream_io::readNumbersASCII<int16_t>(input, n_items, sep).data(), n_bytes); break;
		case DataType::Int32: memcpy(this->data.data(), stream_io::readNumbersASCII<int32_t>(input, n_items, sep).data(), n_bytes); break;
		case DataType::Int64: memcpy(this->data.data(), stream_io::readNumbersASCII<int64_t>(input, n_items, sep).data(), n_bytes); break;
		case DataType::Uint16: memcpy(this->data.data(), stream_io::readNumbersASCII<uint16_t>(input, n_items, sep).data(), n_bytes); break;
		case DataType::Uint32: memcpy(this->data.data(), stream_io::readNumbersASCII<uint32_t>(input, n_items, sep).data(), n_bytes); break;
		case DataType::Uint64: memcpy(this->data.data(), stream_io::readNumbersASCII<uint64_t>(input, n_items, sep).data(), n_bytes); break;
		case DataType::Float32: memcpy(this->data.data(), stream_io::readNumbersASCII<float>(input, n_items, sep).data(), n_bytes); break;
		case DataType::Float64: memcpy(this->data.data(), stream_io::readNumbersASCII<double>(input, n_items, sep).data(), n_bytes); break;
		default:
			std::cout << "ByteBuffer error: Invalid cast type used." << std::endl; exit(-1);
			break;
		}
	}
	inline void ByteBuffer::toStreamBinary(std::ofstream& output, const ByteOrder byte_order)
	{
		this->swap(byte_order);
		output.write(this->data.data(), this->data.size());
	}
	inline void ByteBuffer::toStreamASCII(std::ofstream& output, const char sep)
	{
		const size_t n_elements = this->getNumberOfItems();
		this->swap(ByteOrder::Native);
		switch (this->type)
		{
		case DataType::Int16: { int16_t* begin = this->getDataPtr<int16_t>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		case DataType::Int32: { int32_t* begin = this->getDataPtr<int32_t>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		case DataType::Int64: { int64_t* begin = this->getDataPtr<int64_t>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		case DataType::Uint16: { uint16_t* begin = this->getDataPtr<uint16_t>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		case DataType::Uint32: { uint32_t* begin = this->getDataPtr<uint32_t>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		case DataType::Uint64: { uint64_t* begin = this->getDataPtr<uint64_t>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		case DataType::Float32: { float* begin = this->getDataPtr<float>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		case DataType::Float64: { double* begin = this->getDataPtr<double>(); stream_io::writeNumbersASCII(output, begin, begin + n_elements); } break;
		default:
			std::cout << "ByteBuffer error: Invalid cast type used." << std::endl; exit(-1);
			break;
		}
	}
	inline void bytebuffer::ByteBuffer::swap()
	{
		if (byte_order == ByteOrder::Native) {
			this->byte_order = getNativeEndianness();
		}
		this->byte_order = (this->byte_order == ByteOrder::Big) ? ByteOrder::Little : ByteOrder::Big;
		
		const size_t n_bytes_per_element = getDataTypeSize(this->type);
		switch (n_bytes_per_element) {
		case 1: break;
		case 2: swapByteArrayInplace<1>(this->data); break;
		case 4: swapByteArrayInplace<4>(this->data); break;
		case 8: swapByteArrayInplace<8>(this->data); break;
		default:
			throw std::invalid_argument("Cannot swap this n bytes per element: " + std::to_string(n_bytes_per_element));
			break;
		}
	}
	inline void ByteBuffer::swap(const ByteOrder byte_order)
	{
		if (!sameEndianness(this->byte_order, byte_order)) {
			this->swap();
		}
	}
	inline size_t ByteBuffer::getNumberOfBytes()
	{
		return this->data.size();
	}
	inline size_t bytebuffer::ByteBuffer::getNumberOfItems()
	{
		return this->data.size() / getDataTypeSize(this->type);
	}
	inline DataType ByteBuffer::getDataType()
	{
		return this->type;
	}
	inline ByteOrder ByteBuffer::getByteOrder()
	{
		return this->byte_order;
	}
	inline char* ByteBuffer::getDataPtrChar()
	{
		return this->data.data();
	}
	inline void ByteBuffer::setByteOrder(const ByteOrder byte_order)
	{
		if (byte_order == ByteOrder::Native) {
			this->byte_order = getNativeEndianness();
		}
		else {
			this->byte_order = byte_order;
		}
	}
	inline void ByteBuffer::setDataType(const DataType data_type)
	{
		if (data_type == DataType::Undefined) {
			std::cout << "ByteBuffer error: Invalid cast data type." << std::endl; exit(-1);
		}
		else {
			this->type = data_type;
		}
	}
	template<class Indexable>
	inline void ByteBuffer::setFromIndexable(const Indexable& indexable, const ByteOrder byte_order)
	{
		this->setFromIndexable(indexable, 0, indexable.size(), byte_order);
	}
	template<class Indexable>
	inline void ByteBuffer::setFromIndexable(const Indexable& indexable, const size_t begin, const size_t end, const ByteOrder byte_order)
	{
		if (indexable.size() == 0) { return; }

		using T = typename std::remove_const<typename std::remove_reference<decltype(indexable[0])>::type>::type;
		throwIfNumberTypeNotSupported<T>();

		this->setByteOrder(byte_order);
		this->setDataType(dynamic_types::getDataEnumType<T>());
		const auto n_items = end - begin;
		this->data.clear();
		this->data.resize(n_items * sizeof(T));

		T* casted_buffer = reinterpret_cast<T*>(this->data.data());
		for (size_t i = begin; i < end; i++) {
			*casted_buffer = T(indexable[i]);
			casted_buffer++;
		}
	}
	template<class TwiceIndexable>
	inline void ByteBuffer::setFromTwiceIndexable(const TwiceIndexable& twice_indexable, const ByteOrder byte_order)
	{
		this->setFromTwiceIndexable(twice_indexable, 0, twice_indexable.end(), byte_order);
	}
	template<class TwiceIndexable>
	inline void ByteBuffer::setFromTwiceIndexable(const TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const ByteOrder byte_order)
	{
		if (twice_indexable.size() == 0) { return; }
		using T = typename std::remove_const<typename std::remove_reference<decltype(twice_indexable[0][0])>::type>::type;
		throwIfNumberTypeNotSupported<T>();

		this->setByteOrder(byte_order);
		this->setDataType(dynamic_types::getDataEnumType<T>());
		this->data.clear();
		const size_t outer_size = end - begin;
		const size_t inner_size = twice_indexable[0].size();
		const size_t n_items = outer_size * inner_size;
		this->data.resize(n_items * sizeof(T));

		T* casted_buffer = reinterpret_cast<T*>(this->data.data());
		for (size_t i = begin; i < end; i++) {
			for (size_t j = 0; j < inner_size; j++) {
				*casted_buffer = T(twice_indexable[i][j]);
				casted_buffer++;
			}
		}
	}
	template<class Indexable>
	inline void ByteBuffer::getToIndexable(Indexable& indexable, const ByteOrder byte_order)
	{
		indexable.resize(this->getNumberOfItems());
		this->getToIndexable(indexable, 0, indexable.size(), byte_order);
	}
	template<class Indexable>
	inline void ByteBuffer::getToIndexable(Indexable& indexable, const size_t begin, const size_t end, const ByteOrder byte_order)
	{
		using T = typename std::remove_reference<decltype(indexable[0])>::type;
		this->throwIfNotType<T>();
		const size_t n = end - begin;
		if (n < this->getNumberOfItems()) {
			std::cout << "ByteBuffer error: Destination storage is too small to copy the buffer data." << std::endl; exit(-1);
		}
		this->swap(byte_order);

		T* casted_buffer = reinterpret_cast<T*>(this->data.data());
		for (size_t i = begin; i < end; i++) {
			indexable[i] = *casted_buffer;
			casted_buffer++;
		}
	}
	template<class TwiceIndexable>
	inline void ByteBuffer::getToTwiceIndexable(TwiceIndexable& twice_indexable, const ByteOrder byte_order)
	{
		const size_t n = this->getNumberOfItems();
		const size_t inner_size = twice_indexable[0].size();
		const size_t outer_size = n / inner_size;
		twice_indexable.resize(outer_size);
		this->getToTwiceIndexable(twice_indexable, 0, twice_indexable.size(), byte_order);
	}
	template<class TwiceIndexable>
	inline void ByteBuffer::getToTwiceIndexable(TwiceIndexable& twice_indexable, const size_t begin, const size_t end, const ByteOrder byte_order)
	{
		if (twice_indexable.size() == 0) { return; }
		if (twice_indexable[begin].size() == 0) { std::cout << "ByteBuffer error: Inner container of TwiceIndexable has size 0. Probably due to being dynamically sized. Consider using a statically sized inner container instead (such as std::array)." << std::endl; exit(-1); }
		using T = typename std::remove_reference<decltype(twice_indexable[0][0])>::type;
		throwIfNumberTypeNotSupported<T>();

		const size_t n = this->getNumberOfItems();
		const size_t inner_size = twice_indexable[0].size();
		const size_t outer_size = n / inner_size;
		if (this->getNumberOfItems() != inner_size * outer_size) { std::cout << "ByteBuffer error: ByteBuffer::getToTwiceIndexable error. Dimensions mismatch." << std::endl; exit(-1); }

		this->swap(byte_order);
		T* buffer_it = this->getDataPtr<T>();
		T* buffer_end = buffer_it + this->getNumberOfItems();

		for (size_t i = begin; i < end; i++) {
			for (size_t j = 0; j < inner_size; j++) {
				twice_indexable[i][j] = T(*buffer_it);
				buffer_it++;
			}
		}
	}
	template<typename T>
	inline bool ByteBuffer::isType()
	{
		return dynamic_types::getDataEnumType<T>() == this->type;
	}
	inline bool ByteBuffer::isType(DataType data_type)
	{
		return this->type == data_type;
	}
	inline void ByteBuffer::clear()
	{
		DataType type = DataType::Undefined;
		ByteOrder byte_order = ByteOrder::Native;
		this->data.clear();
	}
	template<typename T>
	inline void ByteBuffer::throwIfNotType()
	{
		throwIfNumberTypeNotSupported<T>();
		if (!this->isType<T>()) {
			throw std::invalid_argument("Buffer of type " + dynamic_types::getDataTypeString(this->type) + " is not of type " + dynamic_types::getDataTypeString<T>());
		}
	}
	template<typename T>
	inline void ByteBuffer::castInplace()
	{
		throwIfNumberTypeNotSupported<T>();
		if (this->isType<T>()) {
			return;
		}
		else {
			this->swap(ByteOrder::Native);
			switch (this->type)
			{
			case DataType::Int16: castBytesInPlace<int16_t, T>(this->data); break;
			case DataType::Int32: castBytesInPlace<int32_t, T>(this->data); break;
			case DataType::Int64: castBytesInPlace<int64_t, T>(this->data); break;
			case DataType::Uint16: castBytesInPlace<uint16_t, T>(this->data); break;
			case DataType::Uint32: castBytesInPlace<uint32_t, T>(this->data); break;
			case DataType::Uint64: castBytesInPlace<uint64_t, T>(this->data); break;
			case DataType::Float32: castBytesInPlace<float, T>(this->data); break;
			case DataType::Float64: castBytesInPlace<double, T>(this->data); break;
			default:
				std::cout << "ByteBuffer error: Invalid cast type used." << std::endl; exit(-1);
				break;
			}
			this->setDataType(dynamic_types::getDataEnumType<T>());
		}
	}
	template<typename T>
	inline ByteBuffer ByteBuffer::castCopy()
	{
		ByteBuffer copy;
		copy.castInplace<T>();
		return copy;
	}
	template<typename T>
	inline T* ByteBuffer::getDataPtr()
	{
		this->throwIfNotType<T>();
		return reinterpret_cast<T*>(this->data.data());
	}
}

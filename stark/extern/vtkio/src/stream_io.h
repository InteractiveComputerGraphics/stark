#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "dynamic_types.h"

namespace stream_io
{
	template<typename T>
	inline std::vector<T> readNumbersASCII(std::ifstream& input, const size_t n, const char sep = ' ')
	{
		dynamic_types::throwIfNumberTypeNotSupported<T>();

		std::vector<T> solution(n);
		std::string line_buffer;
		std::string word_buffer;

		size_t number_count = 0;
		while (number_count != n) {
			std::getline(input, line_buffer);
			std::istringstream line_stream(line_buffer);
			while (std::getline(line_stream, word_buffer, sep)) {

				if		constexpr (std::is_same_v<T, int16_t>)  { solution[number_count] = (int16_t)std::stoi(word_buffer); }
				else if constexpr (std::is_same_v<T, int32_t>)  { solution[number_count] = std::stoi(word_buffer); }
				else if constexpr (std::is_same_v<T, int64_t>)  { solution[number_count] = std::stoll(word_buffer); }
				else if constexpr (std::is_same_v<T, uint16_t>) { solution[number_count] = (uint16_t)std::stoi(word_buffer); }
				else if constexpr (std::is_same_v<T, uint32_t>) { solution[number_count] = std::stoul(word_buffer); }
				else if constexpr (std::is_same_v<T, uint64_t>) { solution[number_count] = std::stoull(word_buffer); }
				else if constexpr (std::is_same_v<T, float>)    { solution[number_count] = std::stof(word_buffer); }
				else if constexpr (std::is_same_v<T, double>)   { solution[number_count] = std::stod(word_buffer); }
				else { } // This line is never reached

				number_count++;
			}
		}

		return solution;
	}

	template<typename T>
	inline std::vector<T> readNumbersBinary(std::ifstream& input, const size_t n_items)
	{
		std::vector<T> solution(n_items);
		input.get(reinterpret_cast<char*>(solution.data()), n_items*sizeof(T));
		return solution;
	}
	template<typename InputIterator>
	inline void writeNumbersASCII(std::ofstream& output, InputIterator begin, InputIterator end, const char sep = ' ')
	{
		using T = typename std::iterator_traits<InputIterator>::value_type;
		dynamic_types::throwIfNumberTypeNotSupported<T>();

		const auto n = std::distance(begin, end);
		for (auto it = begin; it != end - 1; it++) {
			output << *it << sep;
		}
		output << *(end - 1);
	}
	template<typename T>
	inline void writeNumbersBynary(std::ofstream& output, const T* arr, const size_t n_elements)
	{
		output.write(reinterpret_cast<const char*>(arr), n_elements*sizeof(T));
	}
}
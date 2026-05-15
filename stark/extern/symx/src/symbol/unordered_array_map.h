#pragma once
#include <unordered_map>
#include <array>
#include <stdint.h>

namespace symx
{
    template<std::size_t N, typename INT_TYPE = int>
    struct ArrayHasher {
        std::size_t operator()(const std::array<INT_TYPE, N>& a) const {
            std::size_t h = 0;
            for (auto e : a) {
                h ^= std::hash<INT_TYPE>{}(e)+0x9e3779b9 + (h << 6) + (h >> 2);
            }
            return h;
        }
    };
    template<typename INT_TYPE, std::size_t N, typename VAL>
    using unordered_array_map = std::unordered_map<std::array<INT_TYPE, N>, VAL, ArrayHasher<N, INT_TYPE>>;
}
#pragma once
#include <cstdint>
#include <array>
#include <unordered_map>
#include <unordered_set>

namespace stark
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

    /**
        unordered_array_map<INT_TYPE, N, VAL> = std::unordered_map<std::array<INT_TYPE, N>, VAL>>
    */
    template<typename INT_TYPE, std::size_t N, typename VAL>
    using unordered_array_map = std::unordered_map<std::array<INT_TYPE, N>, VAL, ArrayHasher<N, INT_TYPE>>;
    template<typename INT_TYPE, std::size_t N>
    using unordered_array_set = std::unordered_set<std::array<INT_TYPE, N>, ArrayHasher<N, INT_TYPE>>;
}

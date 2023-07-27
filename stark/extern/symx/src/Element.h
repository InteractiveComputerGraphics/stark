#pragma once
#include <vector>
#include <string>
#include <cstdint>

namespace symx
{
    struct Index { int32_t idx; };

    struct Element
    {
        /* Fields */
        std::vector<Index> indices;

        /* Methods */
        Element(const int32_t& n_items_per_element);
        Index operator[](const int i) const;
        std::vector<Index> slice(const int begin, const int end) const;
        std::vector<Index> all() const;
    };
}
#pragma once
#include <vector>
#include <string>

namespace symx
{
    struct Index { int idx; };

    struct Element
    {
        Element(const int& n_items_per_element);
        Index operator[](const int i) const;
        Index operator[](const std::string label) const;
        std::vector<Index> slice(const int begin, const int end) const;
        std::vector<Index> all() const;
        void set_labels(const std::vector<std::string>& labels);

    private:
        int size = -1;
        std::vector<std::string> labels;
    };
}
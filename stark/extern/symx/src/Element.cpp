#include "Element.h"

#include <cassert>
#include <iostream>

symx::Element::Element(const int32_t& n_items_per_element)
	: size(n_items_per_element)
{
}
symx::Index symx::Element::operator[](const int i) const
{
	if (i < 0) {
		std::cout << "symx error: Element indices must be positive intergers." << std::endl;
		exit(-1);
	}
	if (i >= this->size) {
		std::cout << "symx error: Index " << i << " exceeds Element size " << this->size << std::endl;
		exit(-1);
	}
	return {i};
}
std::vector<symx::Index> symx::Element::slice(const int begin, const int end) const
{
	std::vector<Index> indices;
	for (int i = begin; i < end; i++) {
		indices.push_back((*this)[i]);
	}
	return indices;
}
std::vector<symx::Index> symx::Element::all() const
{
	return this->slice(0, this->size);
}

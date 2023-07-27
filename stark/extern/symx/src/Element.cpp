#include "Element.h"

#include <cassert>

symx::Element::Element(const int32_t& n_items_per_element)
{
	this->indices.resize(n_items_per_element);
	for (int32_t i = 0; i < n_items_per_element; i++) {
		this->indices[i].idx = i;
	}
}
symx::Index symx::Element::operator[](const int i) const
{
	return this->indices[i];
}
std::vector<symx::Index> symx::Element::slice(const int begin, const int end) const
{
	assert(begin >= 0);
	assert(end <= this->indices.size());

	std::vector<Index> indices;
	for (int i = begin; i < end; i++) {
		indices.push_back((*this)[i]);
	}
	return indices;
}
std::vector<symx::Index> symx::Element::all() const
{
	return this->slice(0, (int)this->indices.size());
}

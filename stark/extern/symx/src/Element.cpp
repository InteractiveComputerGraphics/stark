#include "Element.h"

#include <cassert>
#include <iostream>
#include <algorithm>

symx::Element::Element(const int& n_items_per_element)
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
symx::Index symx::Element::operator[](const std::string label) const
{
	auto it = std::find(this->labels.begin(), this->labels.end(), label);
	if (it == this->labels.end()) {
		std::cout << "symx error: Element index label " + label + " not found." << std::endl;
		exit(-1);
	}
	const int idx = (int)std::distance(this->labels.begin(), it);
	return (*this)[idx];
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

void symx::Element::set_labels(const std::vector<std::string>& labels)
{
	if (labels.size() != this->size) {
		std::cout << "symx Element::set_labels() error: Number of labels doesn't match the element size: {";
		for (int i = 0; i < (int)labels.size() - 1; i++) {
			std::cout << labels[i] << ", ";
		}
		std::cout << labels.back() << "}" << std::endl;
		exit(-1);
	}
	this->labels = labels;
}

#include "Id.h"

stark::models::Id::Id(const int idx)
	: idx(idx)
{
}

void stark::models::Id::set_local_idx(const std::string label, const int local_idx)
{
	this->local_indices[label] = local_idx;
}

int stark::models::Id::get_global_idx() const
{
	return this->idx;
}

int stark::models::Id::get_local_idx(const std::string label) const
{
	auto it = this->local_indices.find(label);
	if (it == this->local_indices.end()) {
		std::cout << "stark error: Id.get_local_idx(label) didn't find " + label << std::endl;
		exit(-1);
	}
	return it->second;
}

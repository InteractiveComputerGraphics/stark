#include "MeshOutputGroups.h"

void stark::MeshOutputGroups::add_to_group(const std::string label, const int body_id)
{
	this->groups[label].insert(body_id);
}

void stark::MeshOutputGroups::add_to_group(const std::string label, const std::vector<int>& ids)
{
	for (int id : ids) {
		this->add_to_group(label, ids);
	}
}

int stark::MeshOutputGroups::size() const
{
	return (int)this->groups.size();
}

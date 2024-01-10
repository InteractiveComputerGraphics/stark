#include "Interactions.h"

using namespace stark::models;

stark::models::Interactions::Interactions(core::Stark& stark, spEnergyFrictionalContact contact)
	: contact(contact)
{
	this->attachments = std::make_shared<EnergyAttachments>(stark, contact->dyn, contact->rb);
}

void stark::models::Interactions::set_friction(const int global_idx0, const int global_idx1, double coulombs_mu)
{
	this->contact->set_coulomb_friction_pair(global_idx0, global_idx1, coulombs_mu);
}

void stark::models::Interactions::disable_collision(const int global_idx0, const int global_idx1)
{
	this->contact->disable_collision(global_idx0, global_idx1);
}

void stark::models::Interactions::_attach_rb_deformable_by_distance(const RigidBodyHandler& rigidbody, const int deformable_idx, const double distance, const double stiffness)
{
	const auto& dist = rigidbody.get_collision_mesh_global_distance();
	const auto& x = this->contact->dyn->x1;
	std::vector<int> indices;
	for (int i = x.get_begin(deformable_idx); i < x.get_end(deformable_idx); i++) {
		if (dist.unsigned_distance(x[i]).distance < distance) {
			indices.push_back(i);
		}
	}
	this->attachments->add_rigidbody_deformable(rigidbody.index(), indices, stiffness);
}

void stark::models::Interactions::_attach_deformable_deformable_by_distance(const int deformable_idx0, const int deformable_idx1, const double distance, const double stiffness)
{
	const auto& x = this->contact->dyn->x1;

	std::vector<std::array<int, 2>> pairs;
	for (int i = x.get_begin(deformable_idx0); i < x.get_end(deformable_idx0); i++) {
		for (int j = x.get_begin(deformable_idx1); j < x.get_end(deformable_idx1); j++) {
			if ((x[i] - x[j]).norm() < distance) {
				pairs.push_back({ i, j });
			}
		}
	}
	this->attachments->add_deformable_deformable(pairs, stiffness);
}

#include "EnergyPointPrescribedPositions.h"

#include "../time_integration.h"


stark::models::EnergyPointPrescribedPositions::EnergyPointPrescribedPositions(Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	// Update BC in case the user has changed them
	stark.callbacks.before_time_step.push_back([&]() { this->_before_time_step(stark); });

	// Declare the energy
	stark.global_energy.add_energy("EnergyPointPrescribedPositions", this->conn,
		[&](symx::Energy& energy, symx::Element& node)
		{
			// Create symbols
			symx::Vector v1 = energy.make_dof_vector(this->dyn->dof, this->dyn->v1.data, node["point"]);
			symx::Vector x0 = energy.make_vector(this->dyn->x0.data, node["point"]);
			symx::Vector x1_prescribed = energy.make_vector(this->target_positions, node["idx"]);
			symx::Scalar k = energy.make_scalar(this->stiffness, node["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			symx::Vector x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = 0.5 * k * (x1 - x1_prescribed).squared_norm();
			energy.set(E);
		}
	);
}
std::shared_ptr<stark::models::PrescribedPointGroup> stark::models::EnergyPointPrescribedPositions::create_group(Id& id, const std::string label)
{
	if (id.get_physical_system() != PhysicalSystem::Deformables) {
		std::cout << "Stark error: EnergyPointPrescribedPositions only works with PhysicalSystem::Deformables." << std::endl;
		exit(-1);
	}
	auto bc = std::make_shared<PrescribedPointGroup>(id.get_global_idx(), label);
	this->bc_source.push_back(bc);
	return this->bc_source.back();
}
std::shared_ptr<stark::models::PrescribedPointGroupWithTransformation> stark::models::EnergyPointPrescribedPositions::create_group_with_transformation(Id& id, const std::string label)
{
	if (id.get_physical_system() != PhysicalSystem::Deformables) {
		std::cout << "Stark error: EnergyPointPrescribedPositions only works with PhysicalSystem::Deformables." << std::endl;
		exit(-1);
	}
	auto bc = std::make_shared<PrescribedPointGroupWithTransformation>(id.get_global_idx(), label);
	this->bc_transform_source.push_back(bc);
	return this->bc_transform_source.back();
}
void stark::models::EnergyPointPrescribedPositions::_before_time_step(Stark& stark)
{
	this->conn.clear();
	this->target_positions.clear();
	this->stiffness.clear();
	this->labels.clear();

	for (auto bc : this->bc_source) {
		const int group_idx = (int)this->stiffness.size();
		this->stiffness.push_back(bc->get_stiffness());
		this->labels.push_back(bc->get_label());

		const int obj = bc->get_obj_idx();
		for (int i = 0; i < bc->size(); i++) {
			const int loc_idx = bc->get_point_idx(i);
			const Eigen::Vector3d& target = bc->get_target_position(i);
			const int glob_idx = this->dyn->X.get_global_index(obj, loc_idx);
			this->conn.numbered_push_back({ glob_idx, group_idx });
			this->target_positions.push_back(target);
		}
	}

	for (auto bc : this->bc_transform_source) {
		const int group_idx = (int)this->stiffness.size();
		this->stiffness.push_back(bc->get_stiffness());
		this->labels.push_back(bc->get_label());

		const int obj = bc->get_obj_idx();
		for (int i = 0; i < bc->size(); i++) {
			const int loc_idx = bc->get_point_idx(i);
			const int glob_idx = this->dyn->X.get_global_index(obj, loc_idx);
			const Eigen::Vector3d& X = this->dyn->X[glob_idx];
			const Eigen::Vector3d& target = bc->get_transformed(X, stark.current_time);
			this->conn.numbered_push_back({ glob_idx, group_idx });
			this->target_positions.push_back(target);
		}
	}
}
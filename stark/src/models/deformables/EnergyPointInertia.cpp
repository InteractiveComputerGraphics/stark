#include "EnergyPointInertia.h"

#include "../../utils/mesh_utils.h"

stark::models::EnergyPointInertia::EnergyPointInertia(Stark& stark, const spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyPointInertia", this->conn,
		[&](symx::Energy& energy, symx::Element& node)
		{
			PointDynamics& dyn = *(this->dyn);

			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(dyn.dof, dyn.v1.data, node["glob"]);
			symx::Vector x0 = energy.make_vector(dyn.x0.data, node["glob"]);
			symx::Vector v0 = energy.make_vector(dyn.v0.data, node["glob"]);
			symx::Vector a = energy.make_vector(dyn.a.data, node["glob"]);
			symx::Scalar volume = energy.make_scalar(this->lumped_volume.data, node["idx"]);
			symx::Scalar density = energy.make_scalar(this->density, node["obj"]);
			symx::Scalar inertial_damping = energy.make_scalar(this->inertial_damping, node["obj"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);
			symx::Vector gravity = energy.make_vector(stark.settings.simulation.gravity);

			//// Set energy expression
			symx::Scalar mass = volume * density;
			symx::Vector x1 = x0 + dt * v1;
			symx::Vector xhat = x0 + dt * v0 + dt * dt * (a + gravity);
			symx::Vector dev = x1 - xhat;
			symx::Vector dev2 = x1 - x0;
			symx::Scalar E = 0.5 * mass * (dev.dot(dev) / (dt.powN(2)) + dev2.dot(dev2) * inertial_damping / dt);
			energy.set(E);
		}
	);
}

void stark::models::EnergyPointInertia::add(Id& id, const std::vector<double>& lumped_volume, const double density, const double inertial_damping, const std::string label)
{
	const int n = this->dyn->size(id);
	if (lumped_volume.size() != n) {
		std::cout << "Stark error: EnergyPointInertia::add() found `lumped_volume` with different size than the set nodes." << std::endl;
		exit(-1);
	}

	const int group = this->lumped_volume.append(lumped_volume);
	this->density.push_back(density);
	this->inertial_damping.push_back(inertial_damping);
	this->labels.push_back(label);

	for (int i = 0; i < n; i++) {
		this->dyn->X.get_global_index(id.get_global_idx(), i);
		this->conn.numbered_push_back({ i, group });
	}

	id.set_local_idx("EnergyPointInertia", group);
}
void stark::models::EnergyPointInertia::add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double density, const double inertial_damping, const std::string label)
{
	const int n = this->dyn->size(id);
	std::vector<double> lumped_volume(n, 0.0);
	for (int tri_i = 0; tri_i < (int)triangles.size(); tri_i++) {

		// Connectivity
		const std::array<int, 3>& conn = triangles[tri_i];
		const std::array<int, 3> conn_glob = this->dyn->X.get_global_indices(id.get_global_idx(), conn);

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d& C = this->dyn->X[conn_glob[2]];

		const double lumped = utils::triangle_area(A, B, C)/3.0;
		lumped_volume[conn_glob[0]] += lumped;
		lumped_volume[conn_glob[1]] += lumped;
		lumped_volume[conn_glob[2]] += lumped;
	}
	this->add(id, lumped_volume, density, inertial_damping, label);
}
void stark::models::EnergyPointInertia::add(Id& id, const std::vector<std::array<int, 4>>& tets, const double density, const double inertial_damping, const std::string label)
{
	const int n = this->dyn->size(id);
	std::vector<double> lumped_volume(n, 0.0);
	for (int tri_i = 0; tri_i < (int)tets.size(); tri_i++) {

		// Connectivity
		const std::array<int, 4>& conn = tets[tri_i];
		const std::array<int, 4> conn_glob = this->dyn->X.get_global_indices(id.get_global_idx(), conn);

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d& C = this->dyn->X[conn_glob[2]];
		const Eigen::Vector3d& D = this->dyn->X[conn_glob[3]];

		const double lumped = utils::unsigned_tetra_volume(A, B, C, D)/4.0;
		lumped_volume[conn_glob[0]] += lumped;
		lumped_volume[conn_glob[1]] += lumped;
		lumped_volume[conn_glob[2]] += lumped;
		lumped_volume[conn_glob[3]] += lumped;
	}
	this->add(id, lumped_volume, density, inertial_damping, label);
}
void stark::models::EnergyPointInertia::update(Id& id, const std::vector<double>& lumped_volume, const double density, const double inertial_damping)
{
	const int local_idx = id.get_local_idx("EnergyPointInertia");
	this->lumped_volume.update(local_idx, lumped_volume);
	this->density[local_idx] = density;
	this->inertial_damping[local_idx] = inertial_damping;
}

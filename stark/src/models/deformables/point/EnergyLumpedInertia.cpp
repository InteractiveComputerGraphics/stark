#include "EnergyLumpedInertia.h"

#include "../../../utils/include.h"

stark::EnergyLumpedInertia::EnergyLumpedInertia(stark::core::Stark& stark, const spPointDynamics dyn)
	: dyn(dyn)
{
	// Energy definition
	stark.global_energy.add_energy("EnergyLumpedInertia", this->conn,
		[&](symx::Energy& energy, symx::Element& node)
		{
			PointDynamics& dyn = *(this->dyn);

			//// Create symbols
			symx::Vector v1 = energy.make_dof_vector(dyn.dof, dyn.v1.data, node["glob"]);
			symx::Vector x0 = energy.make_vector(dyn.x0.data, node["glob"]);
			symx::Vector v0 = energy.make_vector(dyn.v0.data, node["glob"]);
			symx::Vector a = energy.make_vector(dyn.a.data, node["glob"]);
			symx::Vector f = energy.make_vector(dyn.f.data, node["glob"]);
			symx::Scalar volume = energy.make_scalar(this->lumped_volume, node["idx"]);
			symx::Scalar density = energy.make_scalar(this->density, node["group"]);
			symx::Scalar damping = energy.make_scalar(this->damping, node["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);
			symx::Vector gravity = energy.make_vector(stark.gravity);

			//// Set energy expression
			symx::Scalar mass = volume * density;
			symx::Vector x1 = x0 + dt * v1;
			symx::Vector xhat = x0 + dt * v0 + dt * dt * (a + gravity + f/mass);
			symx::Vector dev = x1 - xhat;
			symx::Vector dev2 = x1 - x0;
			symx::Scalar E = 0.5 * mass * (dev.dot(dev) / (dt.powN(2)) + dev2.dot(dev2) * damping / dt);
			energy.set(E);
		}
	);

	// Inverse mass for acceleration residual
	stark.callbacks.add_inv_mass_application(this->dyn->dof, 
		[&](double* begin, double* end)
		{
			const int n = (int)std::distance(begin, end);
			if (n != 3 * this->dyn->size()) {
				std::cout << "Stark error: EnergyLumpedInertia::inv_mass_application() found `begin` and `end` with different size than the set nodes." << std::endl;
				exit(-1);
			}

			// Apply inverse mass
			for (const std::array<int, 3>&conn : this->conn.data) {
				const int idx = conn[0];
				const int glob = conn[1];
				const int obj = conn[2];
				const double mass = this->density[obj] * this->lumped_volume[idx];
				const double mass_inv = 1.0 / mass;
				begin[3 * glob + 0] *= mass_inv;
				begin[3 * glob + 1] *= mass_inv;
				begin[3 * glob + 2] *= mass_inv;
			}
		}
	);
}

stark::EnergyLumpedInertia::Handler stark::EnergyLumpedInertia::add(const PointSetHandler& set, const std::vector<int>& points, const std::vector<double>& lumped_volume, const Params& params)
{
	set.exit_if_not_valid("EnergyLumpedInertia::add");
	const int group = (int)this->density.size();
	this->density.push_back(params.density);
	this->damping.push_back(params.damping);

	for (int i = 0; i < (int)points.size(); i++) {
		this->lumped_volume.push_back(lumped_volume[i]);
		const int glob_idx = set.get_global_index(points[i]);
		this->conn.numbered_push_back({ glob_idx, group });
	}

	return Handler(this, group);
}
stark::EnergyLumpedInertia::Handler stark::EnergyLumpedInertia::add(const PointSetHandler& set, const std::vector<double>& lumped_volume, const Params& params)
{
	set.exit_if_not_valid("EnergyLumpedInertia::add");
	const int n = set.size();
	if (lumped_volume.size() != n) {
		std::cout << "Error: EnergyLumpedInertia::add(): lumped_volume.size() != n" << std::endl;
		exit(-1);
	}

	std::vector<int> points;
	std::vector<double> nonzero_lumped_volume;
	points.reserve(n);
	nonzero_lumped_volume.reserve(n);
	for (int i = 0; i < n; i++) {
		if (lumped_volume[i] > 0.0) {
			points.push_back(i);
			nonzero_lumped_volume.push_back(lumped_volume[i]);
		}
	}

	return this->add(set, points, nonzero_lumped_volume, params);
}
stark::EnergyLumpedInertia::Handler stark::EnergyLumpedInertia::add(const PointSetHandler& set, const std::vector<std::array<int, 2>>& edges, const Params& params)
{
	set.exit_if_not_valid("EnergyLumpedInertia::add");
	std::vector<double> lumped_volume(set.size(), 0.0);
	for (int edge_i = 0; edge_i < (int)edges.size(); edge_i++) {

		// Connectivity
		const std::array<int, 2>& conn = edges[edge_i];
		const std::array<int, 2> conn_glob = set.get_global_indices(conn);

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];

		const double lumped = (A - B).norm() / 2.0;
		lumped_volume[conn[0]] += lumped;
		lumped_volume[conn[1]] += lumped;
	}

	return this->add(set, lumped_volume, params);
}
stark::EnergyLumpedInertia::Handler stark::EnergyLumpedInertia::add(const PointSetHandler& set, const std::vector<std::array<int, 3>>& triangles, const Params& params)
{
	set.exit_if_not_valid("EnergyLumpedInertia::add");
	std::vector<double> lumped_volume(set.size(), 0.0);
	for (int tri_i = 0; tri_i < (int)triangles.size(); tri_i++) {

		// Connectivity
		const std::array<int, 3>& conn = triangles[tri_i];
		const std::array<int, 3> conn_glob = set.get_global_indices(conn);

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d& C = this->dyn->X[conn_glob[2]];

		const double lumped = triangle_area(A, B, C) / 3.0;
		lumped_volume[conn[0]] += lumped;
		lumped_volume[conn[1]] += lumped;
		lumped_volume[conn[2]] += lumped;
	}

	return this->add(set, lumped_volume, params);
}
stark::EnergyLumpedInertia::Handler stark::EnergyLumpedInertia::add(const PointSetHandler& set, const std::vector<std::array<int, 4>>& tets, const Params& params)
{
	set.exit_if_not_valid("EnergyLumpedInertia::add");
	std::vector<double> lumped_volume(set.size(), 0.0);
	for (int tet_i = 0; tet_i < (int)tets.size(); tet_i++) {

		// Connectivity
		const std::array<int, 4>& conn = tets[tet_i];
		const std::array<int, 4> conn_glob = set.get_global_indices(conn);

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d& C = this->dyn->X[conn_glob[2]];
		const Eigen::Vector3d& D = this->dyn->X[conn_glob[3]];

		const double lumped = unsigned_tetra_volume(A, B, C, D) / 4.0;
		lumped_volume[conn[0]] += lumped;
		lumped_volume[conn[1]] += lumped;
		lumped_volume[conn[2]] += lumped;
		lumped_volume[conn[3]] += lumped;

	}

	return this->add(set, lumped_volume, params);
}

stark::EnergyLumpedInertia::Params stark::EnergyLumpedInertia::get_params(const Handler& handler) const
{
	handler.exit_if_not_valid("EnergyLumpedInertia::get_params");
	const int group = handler.get_idx();

	Params params;
	params.density = this->density[group];
	params.damping = this->damping[group];
	return params;
}
void stark::EnergyLumpedInertia::set_params(const Handler& handler, const Params& params)
{
	handler.exit_if_not_valid("EnergyLumpedInertia::set_params");
	const int group = handler.get_idx();
	this->density[group] = params.density;
	this->damping[group] = params.damping;
}

double stark::EnergyLumpedInertia::get_mass(const Handler& handler) const
{
	handler.exit_if_not_valid("EnergyLumpedInertia::get_mass");
	const int group = handler.get_idx();
	auto group_idx = this->conn.get_label_idx("group");
	double mass = 0.0;
	for (int idx = 0; idx < (int)this->conn.data.size(); idx++) {
		if (this->conn.data[idx][group_idx] == group) {
			mass += this->density[group] * this->lumped_volume[idx];
		}
	}
	return mass;
}

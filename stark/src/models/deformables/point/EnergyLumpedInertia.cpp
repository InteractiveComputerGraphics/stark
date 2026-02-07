#include "EnergyLumpedInertia.h"

#include "../../../utils/include.h"

using namespace symx;

stark::EnergyLumpedInertia::EnergyLumpedInertia(stark::core::Stark& stark, const spPointDynamics dyn)
	: dyn(dyn)
{
	// Energy definition
	stark.global_potential->add_potential("EnergyLumpedInertia", this->conn,
		[&](MappedWorkspace<double>& mws, Element& node)
		{
			//// Create symbols
			Vector v1 = mws.make_vector(this->dyn->v1.data, node["glob"]);
			Vector x0 = mws.make_vector(this->dyn->x0.data, node["glob"]);
			Vector v0 = mws.make_vector(this->dyn->v0.data, node["glob"]);
			Vector a = mws.make_vector(this->dyn->a.data, node["glob"]);
			Vector f = mws.make_vector(this->dyn->f.data, node["glob"]);
			Scalar volume = mws.make_scalar(this->lumped_volume, node["idx"]);
			Scalar density = mws.make_scalar(this->density, node["group"]);
			Scalar damping = mws.make_scalar(this->damping, node["group"]);
			Scalar is_quasistatic = mws.make_scalar(this->is_quasistatic, node["group"]);
			Scalar dt = mws.make_scalar(stark.dt);
			Vector gravity = mws.make_vector(stark.gravity);

			//// Set energy expression
			Scalar mass = volume * density;
			Vector x1 = x0 + dt * v1;
			Vector xhat = x0 + dt * v0 + dt * dt * (a + gravity + f/mass);
			Vector dev = x1 - xhat;
			Vector dev2 = x1 - x0;
			Scalar E = 0.5 * mass * (dev.dot(dev) / (dt.powN(2)) + dev2.dot(dev2) * damping / dt);
			return branch(is_quasistatic, 0.0, E);
		}
	);
}

stark::EnergyLumpedInertia::Handler stark::EnergyLumpedInertia::add(const PointSetHandler& set, const std::vector<int>& points, const std::vector<double>& lumped_volume, const Params& params)
{
	set.exit_if_not_valid("EnergyLumpedInertia::add");
	const int group = (int)this->density.size();
	this->density.push_back(params.density);
	this->damping.push_back(params.damping);
	if (params.quasistatic) {
		this->is_quasistatic.push_back(1.0);
	}
	else {
		this->is_quasistatic.push_back(0.0);
	}

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
	this->is_quasistatic[group] = (double)params.quasistatic;
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

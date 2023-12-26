#include "EnergyTriangleBendingGrinspun03.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"

//constexpr double EPSILON = 1e-8;
constexpr double EPSILON = 1e-12;

stark::models::EnergyTriangleBendingGrinspun03::EnergyTriangleBendingGrinspun03(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyTriangleBendingGrinspun03", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> internal_edge = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, internal_edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, internal_edge);

			symx::Scalar rest_angle = energy.make_scalar(this->rest_angle, conn["idx"]);
			symx::Scalar rest_edge_length = energy.make_scalar(this->rest_edge_length, conn["idx"]);
			symx::Scalar rest_height = energy.make_scalar(this->rest_height, conn["idx"]);

			symx::Scalar stiffness = energy.make_scalar(this->stiffness, conn["group"]);
			//symx::Scalar damping = energy.make_scalar(this->damping, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = dt.get_zero();
			{
				auto e0 = x1[1] - x1[0];
				auto e1 = x1[2] - x1[0];
				auto e2 = x1[3] - x1[0];

				auto n0 = e0.cross3(e1);
				auto n1 = -e0.cross3(e2);

				auto dihedral_angle_rad = ((1.0 - EPSILON) * n0.normalized().dot(n1.normalized())).acos();
				auto dihedral_angle_comp_rad = utils::PI - dihedral_angle_rad;
				auto angle_delta = dihedral_angle_comp_rad - rest_angle;

				E += stiffness * (angle_delta * angle_delta) * (rest_edge_length / rest_height);
			}

			energy.set(E);
		}
	);
}
void stark::models::EnergyTriangleBendingGrinspun03::add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double stiffness)
{
	const int group = (int)this->stiffness.size();

	this->stiffness.push_back(stiffness);

	// Find internal_angles (dihedral) connectivity
	std::vector<std::array<int, 4>> internal_angles;
	utils::find_internal_angles(internal_angles, triangles, this->dyn->X.get_set_size(id.get_global_idx()));

	// Initialize structures
	for (int internal_angle_i = 0; internal_angle_i < (int)internal_angles.size(); internal_angle_i++) {

		// Connectivity
		const std::array<int, 4>& conn = internal_angles[internal_angle_i];
		const std::array<int, 4> conn_glob = this->dyn->X.get_global_indices(id.get_global_idx(), conn);
		this->conn.numbered_push_back({ group, conn_glob[0], conn_glob[1], conn_glob[2], conn_glob[3] });

		// Fetch coordinates and compute edges
		const Eigen::Vector3d e0 = this->dyn->X[conn_glob[1]] - this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d e1 = this->dyn->X[conn_glob[2]] - this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d e2 = this->dyn->X[conn_glob[3]] - this->dyn->X[conn_glob[0]];

		// Edge length
		const double edge_length = e0.norm();
		this->rest_edge_length.push_back(edge_length);

		const Eigen::Vector3d n0 = e0.cross(e1);
		const Eigen::Vector3d n1 = -e0.cross(e2);

		const double dihedral_angle_rad = std::acos((1.0 - EPSILON) * n0.normalized().dot(n1.normalized()));
		const double dihedral_angle_comp_rad = utils::PI - dihedral_angle_rad;
		this->rest_angle.push_back(dihedral_angle_comp_rad);

		// Heights
		const double A0 = 0.5 * n0.norm();
		const double A1 = 0.5 * n1.norm();

		const double h1 = 2.0 * A0 / edge_length;
		const double h2 = 2.0 * A1 / edge_length;

		const double height_factor = 1.0/6.0 * (h1 + h2);
		this->rest_height.push_back(height_factor);
	}

	id.set_local_idx("EnergyTriangleBendingGrinspun03", group);
}
void stark::models::EnergyTriangleBendingGrinspun03::set_parameters(Id& id, const double stiffness)
{
	this->stiffness[id.get_local_idx("EnergyTriangleBendingGrinspun03")] = stiffness;
}

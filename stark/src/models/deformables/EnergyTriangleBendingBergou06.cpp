#include "EnergyTriangleBendingBergou06.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"

stark::models::EnergyTriangleBendingBergou06::EnergyTriangleBendingBergou06(spPointDynamics dyn)
	: dyn(dyn)
{
}

void stark::models::EnergyTriangleBendingBergou06::declare(Stark& stark)
{
	stark.global_energy.add_energy("EnergyTriangleBergou06", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> internal_edge = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, internal_edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, internal_edge);
			symx::Matrix Q = energy.make_matrix(this->Q_matrix, { 4, 4 }, conn["idx"]);
			symx::Scalar stiffness = energy.make_scalar(this->stiffness, conn["group"]);
			symx::Scalar damping = energy.make_scalar(this->damping, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Energy
			symx::Scalar E = dt.get_zero();
			for (int i = 0; i < 3; i++) {
				symx::Vector x = symx::Vector({ x1[0][i], x1[1][i], x1[2][i], x1[3][i] });
				symx::Vector v = symx::Vector({ v1[0][i], v1[1][i], v1[2][i], v1[3][i] });
				E += stiffness * (x.transpose() * Q * x) + 0.5 * damping * dt * (v.transpose() * Q * v);
			}
			energy.set(E);
		}
	);
}
int stark::models::EnergyTriangleBendingBergou06::add(const int obj_idx, const std::vector<std::array<int, 3>>& triangles, const double stiffness, const double damping, const double cutoff_angle_deg, const std::string label)
{
	const int group = this->labels.size();

	this->stiffness.push_back(stiffness);
	this->damping.push_back(damping);
	this->cutoff_angle_deg.push_back(cutoff_angle_deg);
	this->labels.push_back(label);

	// Find internal_angles (dihedral) connectivity
	std::vector<std::array<int, 4>> internal_angles;
	utils::find_internal_angles(internal_angles, triangles, this->dyn->X.get_set_size(obj_idx));

	// Definitions
	auto cotTheta = [](const Eigen::Vector3d& v, const Eigen::Vector3d& w) {
		const double cosTheta = v.dot(w);
		const double sinTheta = (v.cross(w)).norm();
		return (cosTheta / sinTheta);
	};
	const double cutoff_angle_rad = utils::deg2rad(cutoff_angle_deg);

	// Initialize structures
	for (int internal_angle_i = 0; internal_angle_i < (int)internal_angles.size(); internal_angle_i++) {

		// Connectivity
		const std::array<int, 4>& conn = internal_angles[internal_angle_i];
		const std::array<int, 4> conn_glob = this->dyn->X.get_global_indices(obj_idx, conn);
		this->conn.numbered_push_back({ group, conn_glob[0], conn_glob[1], conn_glob[2], conn_glob[3] });

		// Fetch coordinates and compute edges
		const Eigen::Vector3d e0 = this->dyn->X[conn_glob[1]] - this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d e1 = this->dyn->X[conn_glob[2]] - this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d e2 = this->dyn->X[conn_glob[3]] - this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d e3 = this->dyn->X[conn_glob[2]] - this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d e4 = this->dyn->X[conn_glob[3]] - this->dyn->X[conn_glob[1]];

		// Precompute Q matrix
		const double c01 = cotTheta(e0, e1);
		const double c02 = cotTheta(e0, e2);
		const double c03 = cotTheta(-e0, e3);
		const double c04 = cotTheta(-e0, e4);

		const Eigen::Vector3d n0 = e0.cross(e1);
		const Eigen::Vector3d n1 = -e0.cross(e2);

		const double angle_rad = std::acos(n0.normalized().dot(n1.normalized()));
		if (angle_rad > cutoff_angle_rad) {
			continue;
		}

		const double A0 = 0.5 * n0.norm();
		const double A1 = 0.5 * n1.norm();

		const double coef = -3.0 / (A0 + A1) * 0.5;
		const Eigen::Vector4d K = Eigen::Vector4d(c03 + c04, c01 + c02, -c01 - c03, -c02 - c04);
		const Eigen::Matrix4d Q = -coef * K * K.transpose();
		this->Q_matrix.push_back({
			Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3),
			Q(1, 0), Q(1, 1), Q(1, 2), Q(1, 3),
			Q(2, 0), Q(2, 1), Q(2, 2), Q(2, 3),
			Q(3, 0), Q(3, 1), Q(3, 2), Q(3, 3)
		});
	}

	return group;
}
void stark::models::EnergyTriangleBendingBergou06::set_parameters(const int id, const double stiffness, const double damping)
{
	this->stiffness[id] = stiffness;
	this->damping[id] = damping;
}

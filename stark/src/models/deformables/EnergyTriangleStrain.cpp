#include "EnergyTriangleStrain.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"


stark::models::EnergyTriangleStrain::EnergyTriangleStrain(Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyTriangleStrain", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> triangle = conn.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, triangle);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, triangle);
			symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 2, 2 }, conn["idx"]);
			symx::Scalar rest_area = energy.make_scalar(this->triangle_area_rest, conn["idx"]);
			symx::Scalar E = energy.make_scalar(this->young_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			symx::Matrix Dx_32 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			symx::Matrix F_32 = Dx_32 * DXinv;  // 3x2
			symx::Matrix C = F_32.transpose() * F_32;

			// Stable Neo-Hookean strain energy
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (E * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			symx::Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			symx::Scalar J = area / rest_area;
			symx::Scalar Ic = C.trace();
			symx::Scalar logJ = symx::log(J);
			symx::Scalar energy_density = 0.5 * mu * (Ic - 3.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);
			symx::Scalar Energy = area * energy_density;
			energy.set(Energy);
		}
	);
}
void stark::models::EnergyTriangleStrain::add(Id& id, const std::vector<std::array<int, 3>>& triangles, const double young_modulus, const double poisson_ratio, const std::string label)
{
	const int group = (int)this->labels.size();

	this->young_modulus.push_back(young_modulus);
	this->poisson_ratio.push_back(poisson_ratio);
	this->labels.push_back(label);

	// Initialize structures
	for (int tri_i = 0; tri_i < (int)triangles.size(); tri_i++) {

		// Connectivity
		const std::array<int, 3>& conn = triangles[tri_i];
		const std::array<int, 3> conn_glob = this->dyn->X.get_global_indices(id.get_global_idx(), conn);
		this->conn.numbered_push_back({ group, conn_glob[0], conn_glob[1], conn_glob[2] });

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d& C = this->dyn->X[conn_glob[2]];

		// Area
		this->triangle_area_rest.push_back(utils::triangle_area(A, B, C));

		// DXinv
		//// Projection matrix
		const Eigen::Vector3d u = (B - A).normalized();
		const Eigen::Vector3d n = u.cross(C - A);
		const Eigen::Vector3d v = u.cross(n).normalized();
		Eigen::Matrix<double, 2, 3> P;
		P.row(0) = u;
		P.row(1) = v;

		//// Projection
		const Eigen::Vector2d A_ = P*A;
		const Eigen::Vector2d B_ = P*B;
		const Eigen::Vector2d C_ = P*C;

		//// DX
		Eigen::Matrix2d DX;
		DX.col(0) = B_ - A_;
		DX.col(1) = C_ - A_;
		Eigen::Matrix2d DXinv = DX.inverse();
		this->DXinv.push_back({DXinv(0, 0), DXinv(0, 1), DXinv(1, 0), DXinv(1, 1)});
	}

	id.set_local_idx("EnergyTriangleStrain", group);
}

void stark::models::EnergyTriangleStrain::set_parameters(Id& id, const double young_modulus, const double poisson_ratio)
{
	this->young_modulus[id.get_local_idx("EnergyTriangleStrain")] = young_modulus;
	this->poisson_ratio[id.get_local_idx("EnergyTriangleStrain")] = poisson_ratio;
}

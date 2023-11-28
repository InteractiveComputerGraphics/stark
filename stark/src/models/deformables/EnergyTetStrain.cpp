#include "EnergyTetStrain.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"


stark::models::EnergyTetStrain::EnergyTetStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyTetStrain", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> tet = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, tet);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, tet);
			symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 3, 3 }, conn["idx"]);
			symx::Scalar tet_rest_volume = energy.make_scalar(this->tet_volume_rest, conn["idx"]);
			symx::Scalar E = energy.make_scalar(this->young_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			symx::Matrix Dx = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0], x1[3] - x1[0] }), { 3, 3 }).transpose();
			symx::Matrix F = Dx * DXinv;

			// [Smith et al. 2022] Stable Neo-Hookean Flesh Simulation
			// Eq. 49 from [Smith et al. 2022]
			symx::Scalar mu = E / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (E * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu)); // 3D
			symx::Scalar mu_ = 4.0 / 3.0 * mu;
			symx::Scalar lambda_ = lambda + 5.0 / 6.0 * mu;
			symx::Scalar detF = F.det();
			symx::Scalar Ic = F.frobenius_norm_sq();
			symx::Scalar alpha = 1.0 + mu_ / lambda_ - mu_ / (4.0 * lambda_);
			symx::Scalar energy_density = 0.5 * mu_ * (Ic - 3.0) + 0.5 * lambda_ * (detF - alpha).powN(2) - 0.5 * mu_ * symx::log(Ic + 1.0);
			symx::Scalar Energy = energy_density * tet_rest_volume;
			energy.set(Energy);
		}
	);
}
void stark::models::EnergyTetStrain::add(Id& id, const std::vector<std::array<int, 4>>& tets, const double young_modulus, const double poisson_ratio, const std::string label)
{
	const int group = (int)this->labels.size();

	this->young_modulus.push_back(young_modulus);
	this->poisson_ratio.push_back(poisson_ratio);
	this->labels.push_back(label);

	// Initialize structures
	for (int tet_i = 0; tet_i < (int)tets.size(); tet_i++) {

		// Connectivity
		const std::array<int, 4>& conn = tets[tet_i];
		const std::array<int, 4> conn_glob = this->dyn->X.get_global_indices(id.get_global_idx(), conn);
		this->conn.numbered_push_back({ group, conn_glob[0], conn_glob[1], conn_glob[2], conn_glob[3] });

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];
		const Eigen::Vector3d& C = this->dyn->X[conn_glob[2]];
		const Eigen::Vector3d& D = this->dyn->X[conn_glob[3]];

		// Volume
		this->tet_volume_rest.push_back(utils::unsigned_tetra_volume(A, B, C, D));

		// DXinv
		Eigen::Matrix3d DX;
		DX.col(0) = B - A;
		DX.col(1) = C - A;
		DX.col(2) = D - A;
		Eigen::Matrix3d DXinv = DX.inverse();
		this->DXinv.push_back({
			DXinv(0, 0), DXinv(0, 1), DXinv(0, 2), 
			DXinv(1, 0), DXinv(1, 1), DXinv(1, 2),
			DXinv(2, 0), DXinv(2, 1), DXinv(2, 2)
			}
		);
	}

	id.set_local_idx("EnergyTetStrain", group);
}

void stark::models::EnergyTetStrain::set_parameters(const int id, const double young_modulus, const double poisson_ratio)
{
	this->young_modulus[id] = young_modulus;
	this->poisson_ratio[id] = poisson_ratio;
}

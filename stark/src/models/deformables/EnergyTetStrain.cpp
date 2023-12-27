#include "EnergyTetStrain.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"


symx::Scalar largest_eigenvalue_sym_3x3(const symx::Matrix& A)
{
	// https://hal.science/hal-01501221/document
	const symx::Scalar& a = A(0, 0);
	const symx::Scalar& b = A(1, 1);
	const symx::Scalar& c = A(2, 2);
	const symx::Scalar& d = A(1, 0);
	const symx::Scalar& e = A(2, 1);
	const symx::Scalar& f = A(2, 0);

	const symx::Scalar x1 = a.powN(2) + b.powN(2) + c.powN(2) - a*b - a*c - b*c + 3*(d.powN(2) + e.powN(2) + f.powN(2));
	const symx::Scalar x2 = - (2.0*a - b - c)*(2.0*b - a - c)*(2.0*c - a - b) +
        + 9.0*((2.0*c - a - b)*d.powN(2) + (2.0*b - a - c)*f.powN(2) + (2.0*a - b - c)*e.powN(2)) +
        - 54.0*d*e*f;
	const symx::Scalar sqrt_arg = 4.0*x1.powN(3) - x2.powN(2);
	symx::Scalar phi = symx::atan(symx::sqrt(sqrt_arg) / x2);
	phi += symx::branch(x2 > 0.0, 0.0, stark::utils::PI);
	return (a + b + c + 2.0*symx::sqrt(x1)*symx::cos((phi - stark::utils::PI)/3.0))/3.0;
}

stark::models::EnergyTetStrain::EnergyTetStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyTetStrain_D_SL", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> tet = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, tet);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, tet);
			symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 3, 3 }, conn["idx"]);
			symx::Scalar tet_rest_volume = energy.make_scalar(this->tet_volume_rest, conn["idx"]);
			symx::Scalar e = energy.make_scalar(this->young_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poisson_ratio, conn["group"]);
			symx::Scalar strain_limit = energy.make_scalar(this->strain_limit, conn["group"]);
			symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->strain_limiting_stiffness, conn["group"]);
			symx::Scalar damping = energy.make_scalar(this->damping, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Kinematics
			symx::Matrix Dx1 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0], x1[3] - x1[0] }), { 3, 3 }).transpose();
			symx::Matrix F1 = Dx1 * DXinv;
			symx::Matrix E1 = 0.5*(F1.transpose() * F1 - energy.make_identity_matrix(3));

			symx::Matrix Dx0 = symx::Matrix(symx::gather({ x0[1] - x0[0], x0[2] - x0[0], x0[3] - x0[0] }), { 3, 3 }).transpose();
			symx::Matrix F0 = Dx0 * DXinv;
			symx::Matrix E0 = 0.5*(F0.transpose() * F0 - energy.make_identity_matrix(3));

			symx::Matrix dE_dt = (E1 - E0) / dt;

			// Hardening for strain limiting
			symx::Scalar s1 = largest_eigenvalue_sym_3x3(E1);
			symx::Scalar dl = s1 - (strain_limit + 1.0);
			e += symx::branch(dl > 0.0, strain_limiting_stiffness * dl.powN(3) / 3.0, 0.0);

			// [Smith et al. 2022] Stable Neo-Hookean Flesh Simulation
			// Eq. 49 from [Smith et al. 2022]
			symx::Scalar mu = e / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu)); // 3D
			symx::Scalar mu_ = 4.0 / 3.0 * mu;
			symx::Scalar lambda_ = lambda + 5.0 / 6.0 * mu;
			symx::Scalar detF = F1.det();
			symx::Scalar Ic = F1.frobenius_norm_sq();
			symx::Scalar alpha = 1.0 + mu_ / lambda_ - mu_ / (4.0 * lambda_);
			symx::Scalar elastic_energy_density = 0.5 * mu_ * (Ic - 3.0) + 0.5 * lambda_ * (detF - alpha).powN(2) - 0.5 * mu_ * symx::log(Ic + 1.0);

			// Damping
			symx::Scalar damping_energy_density = 0.5 * damping * dE_dt.frobenius_norm_sq();

			// Total
			symx::Scalar Energy = tet_rest_volume * (elastic_energy_density + damping_energy_density);
			energy.set(Energy);
		}
	);
	//stark.global_energy.add_energy("EnergyTetStrain", this->conn,
	//	[&](symx::Energy& energy, symx::Element& conn)
	//	{
	//		// Unpack connectivity
	//		std::vector<symx::Index> tet = conn.slice(2, 6);

	//		// Create symbols
	//		std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, tet);
	//		std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, tet);
	//		symx::Matrix DXinv = energy.make_matrix(this->DXinv, { 3, 3 }, conn["idx"]);
	//		symx::Scalar tet_rest_volume = energy.make_scalar(this->tet_volume_rest, conn["idx"]);
	//		symx::Scalar e = energy.make_scalar(this->young_modulus, conn["group"]);
	//		symx::Scalar nu = energy.make_scalar(this->poisson_ratio, conn["group"]);
	//		symx::Scalar strain_limit = energy.make_scalar(this->strain_limit, conn["group"]);
	//		symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->strain_limiting_stiffness, conn["group"]);
	//		symx::Scalar damping = energy.make_scalar(this->damping, conn["group"]);
	//		symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

	//		// Time integration
	//		std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

	//		// Kinematics
	//		symx::Matrix Dx1 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0], x1[3] - x1[0] }), { 3, 3 }).transpose();
	//		symx::Matrix F1 = Dx1 * DXinv;
	//		symx::Matrix E1 = 0.5*(F1.transpose() * F1 - energy.make_identity_matrix(3));

	//		// [Smith et al. 2022] Stable Neo-Hookean Flesh Simulation
	//		// Eq. 49 from [Smith et al. 2022]
	//		symx::Scalar mu = e / (2.0 * (1.0 + nu));
	//		symx::Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu)); // 3D
	//		symx::Scalar mu_ = 4.0 / 3.0 * mu;
	//		symx::Scalar lambda_ = lambda + 5.0 / 6.0 * mu;
	//		symx::Scalar detF = F1.det();
	//		symx::Scalar Ic = F1.frobenius_norm_sq();
	//		symx::Scalar alpha = 1.0 + mu_ / lambda_ - mu_ / (4.0 * lambda_);
	//		symx::Scalar elastic_energy_density = 0.5 * mu_ * (Ic - 3.0) + 0.5 * lambda_ * (detF - alpha).powN(2) - 0.5 * mu_ * symx::log(Ic + 1.0);

	//		// Total
	//		symx::Scalar Energy = tet_rest_volume * elastic_energy_density;
	//		energy.set(Energy);
	//	}
	//);
}
void stark::models::EnergyTetStrain::add(Id& id, const std::vector<std::array<int, 4>>& tets, const double young_modulus, const double poisson_ratio, const double damping, const double strain_limit, const double strain_limiting_stiffness, const std::string label)
{
	if (strain_limit < std::numeric_limits<double>::epsilon()) {
		std::cout << "stark error: EnergyTetStrain strain_limit must be larger than epsilon." << std::endl;
		exit(-1);
	}

	const int group = (int)this->labels.size();

	this->young_modulus.push_back(young_modulus);
	this->poisson_ratio.push_back(poisson_ratio);
	this->strain_limit.push_back(strain_limit);
	this->strain_limiting_stiffness.push_back(strain_limiting_stiffness);
	this->damping.push_back(damping);
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

#include "EnergyTetStrain.h"

#include "../deformable_tools.h"
#include "../../time_integration.h"
#include "../../../utils/include.h"


stark::EnergyTetStrain::EnergyTetStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyTetStrain", this->conn_complete,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> tet = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, tet);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, tet);
			std::vector<symx::Vector> X = energy.make_vectors(this->dyn->X.data, tet);
			symx::Scalar scale = energy.make_scalar(this->scale, conn["group"]);
			symx::Scalar e = energy.make_scalar(this->youngs_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poissons_ratio, conn["group"]);
			symx::Scalar strain_limit = energy.make_scalar(this->strain_limit, conn["group"]);
			symx::Scalar strain_limit_stiffness = energy.make_scalar(this->strain_limit_stiffness, conn["group"]);
			symx::Scalar damping = energy.make_scalar(this->strain_damping, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<symx::Vector> Xs = { scale * X[0], scale * X[1], scale * X[2], scale * X[3] };

			// Kinematics
			symx::Matrix DX = tet_jacobian(Xs);
			symx::Matrix DXinv = DX.inv();
			symx::Matrix Dx1 = tet_jacobian(x1);
			symx::Matrix F1 = Dx1 * DXinv;
			symx::Matrix E1 = 0.5*(F1.transpose() * F1 - energy.make_identity_matrix(3));
			symx::Scalar tet_rest_volume = DX.det()/6.0;

			symx::Matrix Dx0 = symx::Matrix(symx::gather({ x0[1] - x0[0], x0[2] - x0[0], x0[3] - x0[0] }), { 3, 3 }).transpose();
			symx::Matrix F0 = Dx0 * DXinv;
			symx::Matrix E0 = 0.5*(F0.transpose() * F0 - energy.make_identity_matrix(3));

			symx::Matrix dE_dt = (E1 - E0) / dt;

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

			// Strain limiting
			std::array<symx::Scalar, 3> s = eigenvalues_sym_3x3(E1);
			symx::Scalar strain_limiting_energy_density = energy.make_zero();
			for (int i = 0; i < 3; i++) {
				symx::Scalar dl = s[i] - strain_limit;
				strain_limiting_energy_density += symx::branch(dl > 0.0, strain_limit_stiffness*dl.powN(3)/3.0, 0.0);
			}

			// Total
			symx::Scalar Energy = tet_rest_volume * (elastic_energy_density + damping_energy_density + strain_limiting_energy_density);
			energy.set(Energy);
		}
	);
	stark.global_energy.add_energy("EnergyTetStrain_Elasticity_Only", this->conn_elasticity_only,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> tet = conn.slice(2, 6);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, tet);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, tet);
			std::vector<symx::Vector> X = energy.make_vectors(this->dyn->X.data, tet);
			symx::Scalar scale = energy.make_scalar(this->scale, conn["group"]);
			symx::Scalar e = energy.make_scalar(this->youngs_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poissons_ratio, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<symx::Vector> Xs = { scale * X[0], scale * X[1], scale * X[2], scale * X[3] };

			// Kinematics
			symx::Matrix DX = symx::Matrix(symx::gather({ Xs[1] - Xs[0], Xs[2] - Xs[0] , Xs[3] - Xs[0] }), { 3, 3 }).transpose();
			symx::Matrix DXinv = DX.inv();
			symx::Matrix Dx1 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0], x1[3] - x1[0] }), { 3, 3 }).transpose();
			symx::Matrix F1 = Dx1 * DXinv;
			symx::Scalar tet_rest_volume = DX.det() / 6.0; // Specific for linear tet elements

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

			// Total
			symx::Scalar Energy = tet_rest_volume * elastic_energy_density;
			energy.set(Energy);
		}
	);
}

stark::EnergyTetStrain::Handler stark::EnergyTetStrain::add(const PointSetHandler& set, const std::vector<std::array<int, 4>>& tets, const Params& params)
{
	set.exit_if_not_valid("EnergyTetStrain::add");
	const int group = (int)this->youngs_modulus.size();

	this->elasticity_only.push_back(params.elasticity_only);
	this->scale.push_back(params.scale);
	this->youngs_modulus.push_back(params.youngs_modulus);
	this->poissons_ratio.push_back(params.poissons_ratio);
	this->strain_damping.push_back(params.damping);
	this->strain_limit.push_back(params.strain_limit);
	this->strain_limit_stiffness.push_back(params.strain_limit_stiffness);

	// Connectivity
	symx::LabelledConnectivity<6>* conn = params.elasticity_only == true ? &this->conn_elasticity_only : &this->conn_complete;
	for (int tet_i = 0; tet_i < (int)tets.size(); tet_i++) {
		const std::array<int, 4>& conn_loc = tets[tet_i];
		const std::array<int, 4> conn_glob = set.get_global_indices(conn_loc);
		conn->numbered_push_back({ group, conn_glob[0], conn_glob[1], conn_glob[2], conn_glob[3] });
	}

	return Handler(this, group);
}

stark::EnergyTetStrain::Params stark::EnergyTetStrain::get_params(const Handler& handler) const
{
	handler.exit_if_not_valid("EnergyTetStrain::get_params");

	const int group = handler.get_idx();

	Params params;
	params.elasticity_only = this->elasticity_only[group];
	params.scale = this->scale[group];
	params.youngs_modulus = this->youngs_modulus[group];
	params.poissons_ratio = this->poissons_ratio[group];
	params.damping = this->strain_damping[group];
	params.strain_limit = this->strain_limit[group];
	params.strain_limit_stiffness = this->strain_limit_stiffness[group];
	return params;
}

void stark::EnergyTetStrain::set_params(const Handler& handler, const Params& params)
{
	handler.exit_if_not_valid("EnergyTetStrain::set_params");

	const int group = handler.get_idx();
	if (this->elasticity_only[group] != params.elasticity_only) {
		std::cout << "Error: EnergyTetStrain::set_params(): elasticity_only cannot be changed" << std::endl;
		exit(-1);
	}

	this->scale[group] = params.scale;
	this->youngs_modulus[group] = params.youngs_modulus;
	this->poissons_ratio[group] = params.poissons_ratio;
	this->strain_damping[group] = params.damping;
	this->strain_limit[group] = params.strain_limit;
	this->strain_limit_stiffness[group] = params.strain_limit_stiffness;
}

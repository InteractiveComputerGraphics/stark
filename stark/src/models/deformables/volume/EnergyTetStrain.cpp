#include "EnergyTetStrain.h"

#include "../deformable_tools.h"
#include "../../time_integration.h"
#include "../../../utils/include.h"

using namespace symx;

stark::EnergyTetStrain::EnergyTetStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	// stark.global_potential->add_potential("EnergyTetStrain", this->conn_complete,
	// 	[&](MappedWorkspace<double>& mws, Element& conn)
	// 	{
	// 		// Unpack connectivity
	// 		std::vector<Index> tet = conn.slice(2, 6);

	// 		// Create symbols
	// 		std::vector<Vector> v1 = mws.make_vectors(this->dyn->v1.data, tet);
	// 		std::vector<Vector> x0 = mws.make_vectors(this->dyn->x0.data, tet);
	// 		std::vector<Vector> X = mws.make_vectors(this->dyn->X.data, tet);
	// 		Scalar scale = mws.make_scalar(this->scale, conn["group"]);
	// 		Scalar e = mws.make_scalar(this->youngs_modulus, conn["group"]);
	// 		Scalar nu = mws.make_scalar(this->poissons_ratio, conn["group"]);
	// 		Scalar strain_limit = mws.make_scalar(this->strain_limit, conn["group"]);
	// 		Scalar strain_limit_stiffness = mws.make_scalar(this->strain_limit_stiffness, conn["group"]);
	// 		Scalar damping = mws.make_scalar(this->strain_damping, conn["group"]);
	// 		Scalar dt = mws.make_scalar(stark.dt);

	// 		// Time integration
	// 		std::vector<Vector> x1 = time_integration(x0, v1, dt);

	// 		// Scaling
	// 		std::vector<Vector> Xs = { scale * X[0], scale * X[1], scale * X[2], scale * X[3] };

	// 		// Kinematics
	// 		Matrix DX = tet_jacobian(Xs);
	// 		Matrix DXinv = DX.inv();
	// 		Matrix Dx1 = tet_jacobian(x1);
	// 		Matrix F1 = Dx1 * DXinv;
	// 		Matrix E1 = 0.5*(F1.transpose() * F1 - mws.make_identity_matrix(3));
	// 		Scalar tet_rest_volume = DX.det()/6.0;

	// 		Matrix Dx0 = Matrix(collect_scalars({ x0[1] - x0[0], x0[2] - x0[0], x0[3] - x0[0] }), { 3, 3 }).transpose();
	// 		Matrix F0 = Dx0 * DXinv;
	// 		Matrix E0 = 0.5*(F0.transpose() * F0 - mws.make_identity_matrix(3));

	// 		Matrix dE_dt = (E1 - E0) / dt;

	// 		// [Smith et al. 2022] Stable Neo-Hookean Flesh Simulation
	// 		// Eq. 49 from [Smith et al. 2022]
	// 		Scalar mu = e / (2.0 * (1.0 + nu));
	// 		Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu)); // 3D
	// 		Scalar mu_ = 4.0 / 3.0 * mu;
	// 		Scalar lambda_ = lambda + 5.0 / 6.0 * mu;
	// 		Scalar detF = F1.det();
	// 		Scalar Ic = F1.frobenius_norm_sq();
	// 		Scalar alpha = 1.0 + mu_ / lambda_ - mu_ / (4.0 * lambda_);
	// 		Scalar elastic_energy_density = 0.5 * mu_ * (Ic - 3.0) + 0.5 * lambda_ * (detF - alpha).powN(2) - 0.5 * mu_ * log(Ic + 1.0);

	// 		// Damping
	// 		Scalar damping_energy_density = 0.5 * damping * dE_dt.frobenius_norm_sq();

	// 		// Strain limiting
	// 		std::array<Scalar, 3> s = eigenvalues_sym_3x3(E1);
	// 		Scalar strain_limiting_energy_density = mws.make_zero();
	// 		for (int i = 0; i < 3; i++) {
	// 			Scalar dl = s[i] - strain_limit;
	// 			strain_limiting_energy_density += branch(dl > 0.0, strain_limit_stiffness*dl.powN(3)/3.0, 0.0);
	// 		}

	// 		// Total
	// 		Scalar Energy = tet_rest_volume * (elastic_energy_density + damping_energy_density + strain_limiting_energy_density);
	// 		return Energy;
	// 	}
	// );
	stark.global_potential->add_potential("EnergyTetStrain_Elasticity_Only", this->conn_elasticity_only,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			// Unpack connectivity
			std::vector<Index> tet = conn.slice(2, 6);

			// Create symbols
			std::vector<Vector> v1 = mws.make_vectors(this->dyn->v1.data, tet);
			std::vector<Vector> x0 = mws.make_vectors(this->dyn->x0.data, tet);
			std::vector<Vector> X = mws.make_vectors(this->dyn->X.data, tet);
			Scalar scale = mws.make_scalar(this->scale, conn["group"]);
			Scalar e = mws.make_scalar(this->youngs_modulus, conn["group"]);
			Scalar nu = mws.make_scalar(this->poissons_ratio, conn["group"]);
			Scalar dt = mws.make_scalar(stark.dt);

			// Time integration
			std::vector<Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<Vector> Xs = { scale * X[0], scale * X[1], scale * X[2], scale * X[3] };

			// Kinematics
			Matrix DX = Matrix(collect_scalars({ Xs[1] - Xs[0], Xs[2] - Xs[0] , Xs[3] - Xs[0] }), { 3, 3 }).transpose();
			Matrix DXinv = DX.inv();
			Matrix Dx1 = Matrix(collect_scalars({ x1[1] - x1[0], x1[2] - x1[0], x1[3] - x1[0] }), { 3, 3 }).transpose();
			Matrix F1 = Dx1 * DXinv;
			Scalar tet_rest_volume = DX.det() / 6.0; // Specific for linear tet elements

			// [Smith et al. 2022] Stable Neo-Hookean Flesh Simulation
			// Eq. 49 from [Smith et al. 2022]
			Scalar mu = e / (2.0 * (1.0 + nu));
			Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - 2.0 * nu)); // 3D
			Scalar mu_ = 4.0 / 3.0 * mu;
			Scalar lambda_ = lambda + 5.0 / 6.0 * mu;
			Scalar detF = F1.det();
			Scalar Ic = F1.frobenius_norm_sq();
			Scalar alpha = 1.0 + mu_ / lambda_ - mu_ / (4.0 * lambda_);
			Scalar elastic_energy_density = 0.5 * mu_ * (Ic - 3.0) + 0.5 * lambda_ * (detF - alpha).powN(2) - 0.5 * mu_ * log(Ic + 1.0);

			// Total
			Scalar Energy = tet_rest_volume * elastic_energy_density;
			return Energy;
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
	LabelledConnectivity<6>* conn = params.elasticity_only == true ? &this->conn_elasticity_only : &this->conn_complete;
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

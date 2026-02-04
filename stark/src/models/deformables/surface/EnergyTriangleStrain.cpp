#include "EnergyTriangleStrain.h"

#include "../deformable_tools.h"
#include "../../time_integration.h"
#include "../../../utils/include.h"

using namespace symx;

stark::EnergyTriangleStrain::EnergyTriangleStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	// Energies
	stark.global_potential->add_potential("EnergyTriangleStrain", this->conn_complete,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			// Unpack connectivity
			std::vector<Index> triangle = conn.slice(2, 5);

			// Create symbols
			std::vector<Vector> v1 = mws.make_vectors(this->dyn->v1.data, triangle);
			std::vector<Vector> x0 = mws.make_vectors(this->dyn->x0.data, triangle);
			std::vector<Vector> X = mws.make_vectors(this->dyn->X.data, triangle);
			Scalar scale = mws.make_scalar(this->scale, conn["group"]);
			Scalar thickness = mws.make_scalar(this->thickness, conn["group"]);
			Scalar e = mws.make_scalar(this->youngs_modulus, conn["group"]);
			Scalar nu = mws.make_scalar(this->poissons_ratio, conn["group"]);
			Scalar strain_damping = mws.make_scalar(this->strain_damping, conn["group"]);
			Scalar strain_limit = mws.make_scalar(this->strain_limit, conn["group"]);
			Scalar strain_limit_stiffness = mws.make_scalar(this->strain_limit_stiffness, conn["group"]);
			Scalar inflation = mws.make_scalar(this->inflation, conn["group"]);
			Scalar dt = mws.make_scalar(stark.dt);

			// Time integration
			std::vector<Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<Vector> Xs = { scale * X[0], scale * X[1], scale * X[2] };

			// Kinematics
			Scalar rest_area = 0.5 * ((Xs[0] - Xs[2]).cross3(Xs[1] - Xs[2])).norm();
			Matrix DXinv = triangle_jacobian(Xs).inv();
			Matrix Dx1_32 = Matrix(gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			Matrix F1_32 = Dx1_32 * DXinv;  // 3x2
			Matrix C1 = F1_32.transpose() * F1_32;
			Matrix E1 = 0.5 * (C1 - energy.make_identity_matrix(2));

			Matrix Dx0_32 = Matrix(gather({ x0[1] - x0[0], x0[2] - x0[0] }), { 2, 3 }).transpose();
			Matrix F0_32 = Dx0_32 * DXinv;  // 3x2
			Matrix E0 = 0.5 * (F0_32.transpose() * F0_32 - energy.make_identity_matrix(2));

			Matrix dE_dt = (E1 - E0) / dt;

			// Neo-Hookean strain energy
			Scalar mu = e / (2.0 * (1.0 + nu));
			Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			Scalar J = area / rest_area;
			Scalar Ic = C1.trace();
			Scalar logJ = log(J);
			Scalar elastic_energy_density = 0.5 * mu * (Ic - 2.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);

			// Damping
			Scalar damping_energy_density = 0.5 * strain_damping * dE_dt.frobenius_norm_sq();

			// Strain limiting
			std::array<Scalar, 2> s = eigenvalues_sym_2x2(E1);
			Scalar strain_limiting_energy_density = mws.make_zero();
			for (int i = 0; i < 2; i++) {
				Scalar dl = s[i] - strain_limit;
				strain_limiting_energy_density += branch(dl > 0.0, strain_limit_stiffness*dl.powN(3)/3.0, 0.0);
			}

			// Inflate
			Vector n0 = -(x0[1] - x0[0]).cross3(x0[2] - x0[0]).normalized();
			Scalar inflation_energy_density = inflation * n0.dot(x1[0] + x1[1] + x1[2])/3.0;

			// Total
			Scalar Energy = thickness * rest_area * (elastic_energy_density + damping_energy_density + strain_limiting_energy_density + inflation_energy_density);
			return Energy;
		}
	);

	stark.global_potential->add_potential("EnergyTriangleStrain_Elasticity_Only", this->conn_elasticity_only,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			// Unpack connectivity
			std::vector<Index> triangle = conn.slice(2, 5);

			// Create symbols
			std::vector<Vector> v1 = mws.make_vectors(this->dyn->v1.data, triangle);
			std::vector<Vector> x0 = mws.make_vectors(this->dyn->x0.data, triangle);
			std::vector<Vector> X = mws.make_vectors(this->dyn->X.data, triangle);
			Scalar scale = mws.make_scalar(this->scale, conn["group"]);
			Scalar thickness = mws.make_scalar(this->thickness, conn["group"]);
			Scalar e = mws.make_scalar(this->youngs_modulus, conn["group"]);
			Scalar nu = mws.make_scalar(this->poissons_ratio, conn["group"]);
			Scalar inflation = mws.make_scalar(this->inflation, conn["group"]);
			Scalar dt = mws.make_scalar(stark.dt);

			// Time integration
			std::vector<Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<Vector> Xs = { scale * X[0], scale * X[1], scale * X[2] };

			// Kinematics
			Scalar rest_area = 0.5 * ((Xs[0] - Xs[2]).cross3(Xs[1] - Xs[2])).norm();
			Matrix DXinv = triangle_jacobian(Xs).inv();
			Matrix Dx1_32 = Matrix(gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			Matrix F1_32 = Dx1_32 * DXinv;  // 3x2
			Matrix C1 = F1_32.transpose() * F1_32;

			// Neo-Hookean strain energy
			Scalar mu = e / (2.0 * (1.0 + nu));
			Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			Scalar J = area / rest_area;
			Scalar Ic = C1.trace();
			Scalar logJ = log(J);
			Scalar elastic_energy_density = 0.5 * mu * (Ic - 2.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);

			// Inflate
			Vector n0 = -(x0[1] - x0[0]).cross3(x0[2] - x0[0]).normalized();
			Scalar inflation_energy_density = inflation * n0.dot(x1[0] + x1[1] + x1[2]) / 3.0;

			// Total
			Scalar Energy = thickness * rest_area * (elastic_energy_density + inflation_energy_density);
			return Energy;
		}
	);
}
stark::EnergyTriangleStrain::Handler stark::EnergyTriangleStrain::add(const PointSetHandler& set, const std::vector<std::array<int, 3>>& triangles, const Params& params)
{
	set.exit_if_not_valid("EnergyTriangleStrain::add");
	const int group = (int)this->youngs_modulus.size();

	this->elasticity_only.push_back(params.elasticity_only);
	this->scale.push_back(params.scale);
	this->thickness.push_back(params.thickness);
	this->youngs_modulus.push_back(params.youngs_modulus);
	this->poissons_ratio.push_back(params.poissons_ratio);
	this->strain_damping.push_back(params.damping);
	this->strain_limit.push_back(params.strain_limit);
	this->strain_limit_stiffness.push_back(params.strain_limit_stiffness);
	this->inflation.push_back(params.inflation);

	// Connectivity
	LabelledConnectivity<5>* conn = params.elasticity_only == true ? &this->conn_elasticity_only : &this->conn_complete;
	for (int tri_i = 0; tri_i < (int)triangles.size(); tri_i++) {
		const std::array<int, 3>& conn_loc = triangles[tri_i];
		const std::array<int, 3> conn_glob = set.get_global_indices(conn_loc);
		conn->numbered_push_back({ group, conn_glob[0], conn_glob[1], conn_glob[2] });
	}

	return Handler(this, group);
}
stark::EnergyTriangleStrain::Params stark::EnergyTriangleStrain::get_params(const Handler& handler) const
{
	handler.exit_if_not_valid("EnergyTriangleStrain::get_params");

	const int group = handler.get_idx();

	Params params;
	params.elasticity_only = this->elasticity_only[group];
	params.scale = this->scale[group];
	params.thickness = this->thickness[group];
	params.youngs_modulus = this->youngs_modulus[group];
	params.poissons_ratio = this->poissons_ratio[group];
	params.damping = this->strain_damping[group];
	params.strain_limit = this->strain_limit[group];
	params.strain_limit_stiffness = this->strain_limit_stiffness[group];
	params.inflation = this->inflation[group];
	return params;
}
void stark::EnergyTriangleStrain::set_params(const Handler& handler, const Params& params)
{
	handler.exit_if_not_valid("EnergyTriangleStrain::set_params");

	const int group = handler.get_idx();

	if (this->elasticity_only[group] != params.elasticity_only) {
		std::cout << "Error: EnergyTriangleStrain::set_params(): elasticity_only cannot be changed" << std::endl;
		exit(-1);
	}

	this->scale[group] = params.scale;
	this->thickness[group] = params.thickness;
	this->youngs_modulus[group] = params.youngs_modulus;
	this->poissons_ratio[group] = params.poissons_ratio;
	this->strain_damping[group] = params.damping;
	this->strain_limit[group] = params.strain_limit;
	this->strain_limit_stiffness[group] = params.strain_limit_stiffness;
	this->inflation[group] = params.inflation;
}

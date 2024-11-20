#include "EnergyTriangleStrain.h"

#include "../deformable_tools.h"
#include "../../time_integration.h"
#include "../../../utils/include.h"


stark::EnergyTriangleStrain::EnergyTriangleStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	// Energies
	stark.global_energy.add_energy("EnergyTriangleStrain", this->conn_complete,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> triangle = conn.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, triangle);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, triangle);
			std::vector<symx::Vector> X = energy.make_vectors(this->dyn->X.data, triangle);
			symx::Scalar scale = energy.make_scalar(this->scale, conn["group"]);
			symx::Scalar thickness = energy.make_scalar(this->thickness, conn["group"]);
			symx::Scalar e = energy.make_scalar(this->youngs_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poissons_ratio, conn["group"]);
			symx::Scalar strain_damping = energy.make_scalar(this->strain_damping, conn["group"]);
			symx::Scalar strain_limit = energy.make_scalar(this->strain_limit, conn["group"]);
			symx::Scalar strain_limit_stiffness = energy.make_scalar(this->strain_limit_stiffness, conn["group"]);
			symx::Scalar inflation = energy.make_scalar(this->inflation, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<symx::Vector> Xs = { scale * X[0], scale * X[1], scale * X[2] };

			// Kinematics
			symx::Scalar rest_area = 0.5 * ((Xs[0] - Xs[2]).cross3(Xs[1] - Xs[2])).norm();
			symx::Matrix DXinv = triangle_jacobian(Xs).inv();
			symx::Matrix Dx1_32 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			symx::Matrix F1_32 = Dx1_32 * DXinv;  // 3x2
			symx::Matrix C1 = F1_32.transpose() * F1_32;
			symx::Matrix E1 = 0.5 * (C1 - energy.make_identity_matrix(2));

			symx::Matrix Dx0_32 = symx::Matrix(symx::gather({ x0[1] - x0[0], x0[2] - x0[0] }), { 2, 3 }).transpose();
			symx::Matrix F0_32 = Dx0_32 * DXinv;  // 3x2
			symx::Matrix E0 = 0.5 * (F0_32.transpose() * F0_32 - energy.make_identity_matrix(2));

			symx::Matrix dE_dt = (E1 - E0) / dt;

			// Neo-Hookean strain energy
			symx::Scalar mu = e / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			symx::Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			symx::Scalar J = area / rest_area;
			symx::Scalar Ic = C1.trace();
			symx::Scalar logJ = symx::log(J);
			symx::Scalar elastic_energy_density = 0.5 * mu * (Ic - 2.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);

			// Damping
			symx::Scalar damping_energy_density = 0.5 * strain_damping * dE_dt.frobenius_norm_sq();

			// Strain limiting
			std::array<symx::Scalar, 2> s = eigenvalues_sym_2x2(E1);
			symx::Scalar strain_limiting_energy_density = energy.make_zero();
			for (int i = 0; i < 2; i++) {
				symx::Scalar dl = s[i] - strain_limit;
				strain_limiting_energy_density += symx::branch(dl > 0.0, strain_limit_stiffness*dl.powN(3)/3.0, 0.0);
			}

			// Inflate
			symx::Vector n0 = -(x0[1] - x0[0]).cross3(x0[2] - x0[0]).normalized();
			symx::Scalar inflation_energy_density = inflation * n0.dot(x1[0] + x1[1] + x1[2])/3.0;

			// Total
			symx::Scalar Energy = thickness * rest_area * (elastic_energy_density + damping_energy_density + strain_limiting_energy_density + inflation_energy_density);
			energy.set(Energy);
		}
	);

	stark.global_energy.add_energy("EnergyTriangleStrain_Elasticity_Only", this->conn_elasticity_only,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			// Unpack connectivity
			std::vector<symx::Index> triangle = conn.slice(2, 5);

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, triangle);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, triangle);
			std::vector<symx::Vector> X = energy.make_vectors(this->dyn->X.data, triangle);
			symx::Scalar scale = energy.make_scalar(this->scale, conn["group"]);
			symx::Scalar thickness = energy.make_scalar(this->thickness, conn["group"]);
			symx::Scalar e = energy.make_scalar(this->youngs_modulus, conn["group"]);
			symx::Scalar nu = energy.make_scalar(this->poissons_ratio, conn["group"]);
			symx::Scalar inflation = energy.make_scalar(this->inflation, conn["group"]);
			symx::Scalar dt = energy.make_scalar(stark.dt);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<symx::Vector> Xs = { scale * X[0], scale * X[1], scale * X[2] };

			// Kinematics
			symx::Scalar rest_area = 0.5 * ((Xs[0] - Xs[2]).cross3(Xs[1] - Xs[2])).norm();
			symx::Matrix DXinv = triangle_jacobian(Xs).inv();
			symx::Matrix Dx1_32 = symx::Matrix(symx::gather({ x1[1] - x1[0], x1[2] - x1[0] }), { 2, 3 }).transpose();
			symx::Matrix F1_32 = Dx1_32 * DXinv;  // 3x2
			symx::Matrix C1 = F1_32.transpose() * F1_32;

			// Neo-Hookean strain energy
			symx::Scalar mu = e / (2.0 * (1.0 + nu));
			symx::Scalar lambda = (e * nu) / ((1.0 + nu) * (1.0 - nu));  // 2D !!
			symx::Scalar area = 0.5 * ((x1[0] - x1[2]).cross3(x1[1] - x1[2])).norm();
			symx::Scalar J = area / rest_area;
			symx::Scalar Ic = C1.trace();
			symx::Scalar logJ = symx::log(J);
			symx::Scalar elastic_energy_density = 0.5 * mu * (Ic - 2.0) - mu * logJ + 0.5 * lambda * logJ.powN(2);

			// Inflate
			symx::Vector n0 = -(x0[1] - x0[0]).cross3(x0[2] - x0[0]).normalized();
			symx::Scalar inflation_energy_density = inflation * n0.dot(x1[0] + x1[1] + x1[2]) / 3.0;

			// Total
			symx::Scalar Energy = thickness * rest_area * (elastic_energy_density + inflation_energy_density);
			energy.set(Energy);
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
	symx::LabelledConnectivity<5>* conn = params.elasticity_only == true ? &this->conn_elasticity_only : &this->conn_complete;
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

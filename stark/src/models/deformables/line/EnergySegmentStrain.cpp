#include "EnergySegmentStrain.h"

#include "../../time_integration.h"
#include "../../../utils/include.h"

using namespace symx;

stark::EnergySegmentStrain::EnergySegmentStrain(stark::core::Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_potential->add_potential("EnergySegmentStrain", this->conn_complete,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			std::vector<Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<Vector> v1 = mws.make_vectors(this->dyn->v1.data, edge);
			std::vector<Vector> x0 = mws.make_vectors(this->dyn->x0.data, edge);
			std::vector<Vector> X = mws.make_vectors(this->dyn->X.data, edge);
			Scalar scale = mws.make_scalar(this->scale, conn["group"]);
			Scalar section_radius = mws.make_scalar(this->section_radius, conn["group"]);
			Scalar youngs_modulus = mws.make_scalar(this->youngs_modulus, conn["group"]);
			Scalar strain_damping = mws.make_scalar(this->strain_damping, conn["group"]);
			Scalar strain_limit = mws.make_scalar(this->strain_limit, conn["group"]);
			Scalar strain_limit_stiffness = mws.make_scalar(this->strain_limit_stiffness, conn["group"]);
			Scalar dt = mws.make_scalar(stark.dt);

			// Time integration
			std::vector<Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<Vector> Xs = { scale * X[0], scale * X[1] };

			// Strain
			Scalar section_area = M_PI * section_radius.powN(2);
			Scalar l_rest = (Xs[0] - Xs[1]).norm();
			Scalar l = (x1[0] - x1[1]).norm();
			Scalar e = (l - l_rest) / l_rest;
			Scalar volume = section_area * l_rest;
			Scalar E_s = volume * youngs_modulus * e.powN(2) / 2.0;

			// Strain limiting
			Scalar e_over_limit = e - strain_limit;
			Scalar E_sl_ = volume * strain_limit_stiffness * e_over_limit.powN(3) / 3.0;
			Scalar E_sl = branch(e_over_limit > 0.0, E_sl_, 0.0);

			// Strain damping
			Scalar l0 = (x0[1] - x0[0]).norm();
			Scalar e0 = (l0 - l_rest) / l_rest;
			Scalar E_d = dt * strain_damping * ((e - e0) / dt).powN(2) / 2.0;

			// Total
			Scalar E = E_s + E_sl + E_d;
			return E;
		}
	);
	stark.global_potential->add_potential("EnergySegmentStrain_Elasticity_Only", this->conn_elasticity_only,
		[&](MappedWorkspace<double>& mws, Element& conn)
		{
			std::vector<Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<Vector> v1 = mws.make_vectors(this->dyn->v1.data, edge);
			std::vector<Vector> x0 = mws.make_vectors(this->dyn->x0.data, edge);
			std::vector<Vector> X = mws.make_vectors(this->dyn->X.data, edge);
			Scalar scale = mws.make_scalar(this->scale, conn["group"]);
			Scalar section_radius = mws.make_scalar(this->section_radius, conn["group"]);
			Scalar youngs_modulus = mws.make_scalar(this->youngs_modulus, conn["group"]);
			Scalar dt = mws.make_scalar(stark.dt);

			// Time integration
			std::vector<Vector> x1 = time_integration(x0, v1, dt);

			// Scaling
			std::vector<Vector> Xs = { scale * X[0], scale * X[1] };

			// Strain
			Scalar section_area = M_PI * section_radius.powN(2);
			Scalar l_rest = (Xs[0] - Xs[1]).norm();
			Scalar l = (x1[0] - x1[1]).norm();
			Scalar e = (l - l_rest) / l_rest;
			Scalar volume = section_area * l_rest;
			Scalar E_s = volume * youngs_modulus * e.powN(2) / 2.0;

			// Total
			Scalar E = E_s;
			return E;
		}
	);
}
stark::EnergySegmentStrain::Handler stark::EnergySegmentStrain::add(const PointSetHandler& set, const std::vector<std::array<int, 2>>& segments, const Params& params)
{
	set.exit_if_not_valid("EnergySegmentStrain::add");
	const int group = (int)this->youngs_modulus.size();

	this->elasticity_only.push_back(params.elasticity_only);
	this->scale.push_back(params.scale);
	this->section_radius.push_back(params.section_radius);
	this->youngs_modulus.push_back(params.youngs_modulus);
	this->strain_damping.push_back(params.damping);
	this->strain_limit.push_back(params.strain_limit);
	this->strain_limit_stiffness.push_back(params.strain_limit_stiffness);

	// Connectivity
	LabelledConnectivity<4>* conn = params.elasticity_only == true ? &this->conn_elasticity_only : &this->conn_complete;
	for (int edge_i = 0; edge_i < (int)segments.size(); edge_i++) {
		const std::array<int, 2>& conn_loc = segments[edge_i];
		const std::array<int, 2> conn_glob = set.get_global_indices(conn_loc);
		conn->numbered_push_back({ group, conn_glob[0], conn_glob[1] });
	}

	return Handler(this, group);
}
stark::EnergySegmentStrain::Params stark::EnergySegmentStrain::get_params(const Handler& handler) const
{
	handler.exit_if_not_valid("EnergySegmentStrain::get_params");
	const int group = handler.get_idx();

	Params params;
	params.elasticity_only = this->elasticity_only[group];
	params.scale = this->scale[group];
	params.section_radius = this->section_radius[group];
	params.youngs_modulus = this->youngs_modulus[group];
	params.damping = this->strain_damping[group];
	params.strain_limit = this->strain_limit[group];
	params.strain_limit_stiffness = this->strain_limit_stiffness[group];
	return params;
}
void stark::EnergySegmentStrain::set_params(const Handler& handler, const Params& params)
{
	handler.exit_if_not_valid("EnergySegmentStrain::set_params");

	const int group = handler.get_idx();
	if (this->elasticity_only[group] != params.elasticity_only) {
		std::cout << "Error: EnergySegmentStrain::set_params(): elasticity_only cannot be changed" << std::endl;
		exit(-1);
	}

	this->scale[group] = params.scale;
	this->section_radius[group] = params.section_radius;
	this->youngs_modulus[group] = params.youngs_modulus;
	this->strain_damping[group] = params.damping;
	this->strain_limit[group] = params.strain_limit;
	this->strain_limit_stiffness[group] = params.strain_limit_stiffness;
}

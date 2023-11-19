#include "EnergyEdgeStrain.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"

stark::models::EnergyEdgeStrain::EnergyEdgeStrain(Stark& stark, spPointDynamics dyn)
	: dyn(dyn)
{
	stark.global_energy.add_energy("EnergyEdgeStrain", this->conn,
		[&](symx::Energy& energy, symx::Element& conn)
		{
			std::vector<symx::Index> edge = { conn["i"], conn["j"] };

			// Create symbols
			std::vector<symx::Vector> v1 = energy.make_dof_vectors(this->dyn->dof, this->dyn->v1.data, edge);
			std::vector<symx::Vector> x0 = energy.make_vectors(this->dyn->x0.data, edge);
			symx::Scalar young_modulus = energy.make_scalar(this->young_modulus, conn["group"]);
			symx::Scalar section_area = energy.make_scalar(this->section_area, conn["group"]);
			symx::Scalar strain_limit = energy.make_scalar(this->strain_limit, conn["group"]);
			symx::Scalar strain_limiting_stiffness = energy.make_scalar(this->strain_limiting_stiffness, conn["group"]);
			symx::Scalar strain_damping = energy.make_scalar(this->strain_damping, conn["group"]);
			symx::Scalar rest_length = energy.make_scalar(this->rest_length, conn["idx"]);
			symx::Scalar dt = energy.make_scalar(stark.settings.simulation.adaptive_time_step.value);

			// Time integration
			std::vector<symx::Vector> x1 = time_integration(x0, v1, dt);

			// Strain
			symx::Scalar volume = section_area * rest_length;
			symx::Scalar l = (x1[0] - x1[1]).norm();
			symx::Scalar e = (l - rest_length)/rest_length;
			symx::Scalar E_s = volume * young_modulus * e.powN(2)/2.0;

			// Strain limiting
			symx::Scalar e_over_limit = e - strain_limit;
			symx::Scalar E_sl_ = volume * strain_limiting_stiffness * e_over_limit.powN(3) / 3.0;
			symx::Scalar E_sl = symx::branch(e_over_limit > 0.0, E_sl_, 0.0);

			// Strain damping
			symx::Scalar l0 = (x0[1] - x0[0]).norm();
			symx::Scalar e0 = (l0 - rest_length)/rest_length;
			symx::Scalar E_d = dt * strain_damping * ((e - e0) / dt).powN(2) / 2.0;

			// Total
			symx::Scalar E = E_s + E_sl + E_d;
			energy.set(E);
		}
	);
}
void stark::models::EnergyEdgeStrain::add(Id& id, const std::vector<std::array<int, 2>>& edges, const double section_radius, const double young_modulus, const double strain_limit, const double strain_limiting_stiffness, const double strain_damping, const std::string label)
{
	const int group = (int)this->labels.size();

	this->section_area.push_back(utils::PI * std::pow(section_radius, 2));
	this->young_modulus.push_back(young_modulus);
	this->strain_limit.push_back(strain_limit);
	this->strain_limiting_stiffness.push_back(strain_limiting_stiffness);
	this->strain_damping.push_back(strain_damping);
	this->labels.push_back(label);

	// Initialize structures
	for (int edge_i = 0; edge_i < (int)edges.size(); edge_i++) {

		// Connectivity
		const std::array<int, 2>& conn = edges[edge_i];
		const std::array<int, 2> conn_glob = this->dyn->X.get_global_indices(id.get_global_idx(), conn);
		this->conn.numbered_push_back({ group, conn_glob[0], conn_glob[1] });

		// Fetch coordinates
		const Eigen::Vector3d& A = this->dyn->X[conn_glob[0]];
		const Eigen::Vector3d& B = this->dyn->X[conn_glob[1]];

		// Length
		this->rest_length.push_back((A - B).norm());
	}

	id.set_local_idx("EnergyEdgeStrain", group);
}
void stark::models::EnergyEdgeStrain::set_parameters(Id& id, const double young_modulus, const double strain_limit, const double strain_limiting_stiffness, const double strain_damping)
{
	const int local_idx = id.get_local_idx("EnergyEdgeStrain");
	this->young_modulus[local_idx] = young_modulus;
	this->strain_limit[local_idx] = strain_limit;
	this->strain_limiting_stiffness[local_idx] = strain_limiting_stiffness;
	this->strain_damping[local_idx] = strain_damping;
}

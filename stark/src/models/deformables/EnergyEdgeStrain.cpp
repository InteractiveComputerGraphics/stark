#include "EnergyEdgeStrain.h"

#include "../time_integration.h"
#include "../../utils/mesh_utils.h"

stark::models::EnergyEdgeStrain::EnergyEdgeStrain(stark::core::Stark& stark, spPointDynamics dyn)
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
void stark::models::EnergyEdgeStrain::add(Id& id, const std::vector<std::array<int, 2>>& edges, const double section_radius, const double young_modulus, const double strain_limit, const double strain_limit_stiffness, const double strain_damping)
{
	const int group = (int)this->young_modulus.size();

	this->section_area.push_back(utils::PI * std::pow(section_radius, 2));
	this->young_modulus.push_back(young_modulus);
	this->strain_limit.push_back(strain_limit);
	this->strain_limiting_stiffness.push_back(strain_limiting_stiffness);
	this->strain_damping.push_back(strain_damping);

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

void stark::models::EnergyEdgeStrain::set_radius(const Id& id, const double radius)
{
	this->section_area[this->get_index(id)] = utils::PI * std::pow(radius, 2);
}
void stark::models::EnergyEdgeStrain::set_young_modulus(const Id& id, const double young_modulus)
{
	this->young_modulus[this->get_index(id)] = young_modulus;
}
void stark::models::EnergyEdgeStrain::set_poisson_ratio(const Id& id, const double poisson_ratio)
{
	this->poisson_ratio[this->get_index(id)] = poisson_ratio;
}
void stark::models::EnergyEdgeStrain::set_strain_damping(const Id& id, const double strain_damping)
{
	this->strain_damping[this->get_index(id)] = strain_damping;
}
void stark::models::EnergyEdgeStrain::set_strain_limit(const Id& id, const double strain_limit)
{
	if (strain_limit < std::numeric_limits<double>::epsilon()) {
		std::cout << "stark error: strain_limit must be larger than epsilon." << std::endl;
		exit(-1);
	}

	this->strain_limit[this->get_index(id)] = strain_limit;
}
void stark::models::EnergyEdgeStrain::set_strain_limit_stiffness(const Id& id, const double strain_limit_stiffness)
{
	this->strain_limit_stiffness[this->get_index(id)] = strain_limit_stiffness;
}
double stark::models::EnergyEdgeStrain::get_radius(const Id& id)
{
	return std::sqrt(this->section_area[this->get_index(id)] / utils::PI);
}
double stark::models::EnergyEdgeStrain::get_young_modulus(const Id& id)
{
	return this->young_modulus[this->get_index(id)];
}
double stark::models::EnergyEdgeStrain::get_poisson_ratio(const Id& id)
{
	return this->poisson_ratio[this->get_index(id)];
}
double stark::models::EnergyEdgeStrain::get_strain_damping(const Id& id)
{
	return this->strain_damping[this->get_index(id)];
}
double stark::models::EnergyEdgeStrain::get_strain_limit(const Id& id)
{
	return this->strain_limit[this->get_index(id)];
}
double stark::models::EnergyEdgeStrain::get_strain_limit_stiffness(const Id& id)
{
	return this->strain_limit_stiffness[this->get_index(id)];
}
int stark::models::EnergyEdgeStrain::get_index(const Id& id) const
{
	return id.get_local_idx("EnergyEdgeStrain");
}

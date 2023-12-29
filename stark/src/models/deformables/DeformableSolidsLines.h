#pragma once

#include "../../core/Stark.h"
#include "../MeshOutputGroups.h"
#include "../IntervalVector.h"
#include "Id.h"
#include "PointDynamics.h"
#include "EnergyPointInertia.h"
#include "EnergyPointPrescribedPositions.h"
#include "EnergyEdgeStrain.h"
//#include "EnergyFrictionalContact.h"


namespace stark::models
{
	/* Definitions */
	struct MaterialLine
	{
		double line_density = 0.0;
		double section_radius = 0.0;
		double inertia_damping = 0.0;
		double strain_young_modulus = 0.0;
		double strain_damping = 0.0;
		double strain_limit = 0.0;
		double strain_limit_stiffness = 0.0;
		static MaterialLine sticky_goo();
	};

	class DeformableSolidsLines
	{
	public:

		/* Methods */
		DeformableSolidsLines(
			stark::core::Stark& stark,
			spPointDynamics dyn, 
			spEnergyPointInertia inertia,
			spEnergyPointPrescribedPositions prescribed_positions
			//spEnergyFrictionalContact contact
		);

		Id add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 2>>& edges, const MaterialLine& material);
		int get_index(const Id& id) const;
		int get_n_volumes() const;

	private:
		/* Fields */
		spPointDynamics dyn;
		spEnergyPointInertia inertia;
		spEnergyPointPrescribedPositions prescribed_positions;
		spEnergyEdgeStrain strain;
		//spEnergyFrictionalContact contact;
		std::vector<int> global_indices;

		// Output
		MeshOutputGroups output_groups;
		std::vector<std::vector<std::array<int, 2>>> input_edges;

		// Stark callbaks
		void _write_frame(stark::core::Stark& stark);
	};
}

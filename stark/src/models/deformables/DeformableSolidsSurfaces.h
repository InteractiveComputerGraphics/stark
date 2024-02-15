#pragma once

#include "../../core/Stark.h"
#include "../IntervalVector.h"
#include "../MeshOutputGroups.h"
#include "Id.h"
#include "PointDynamics.h"
#include "EnergyPointInertia.h"
#include "EnergyPointPrescribedPositions.h"
#include "EnergyTriangleStrain.h"
#include "EnergyTriangleBendingGrinspun03.h"
#include "../interactions/EnergyFrictionalContact.h"


namespace stark::models
{
	/* Definitions */
	struct MaterialSurface
	{
		double area_density = 0.0;
		double thickness = 0.0;
		double inertia_damping = 0.0;
		double strain_young_modulus = 0.0;
		double strain_poisson_ratio = 0.0;
		double strain_damping = 0.0;
		double strain_limit = 0.0;
		double strain_limit_stiffness = 0.0;
		double bending_stiffness = 0.0;
		double bending_damping = 0.0;
		static MaterialSurface towel();
	};

	class DeformableSolidsSurfaces
	{
	public:
		/* Methods */
		DeformableSolidsSurfaces(
			stark::core::Stark& stark,
			spPointDynamics dyn, 
			spEnergyPointInertia inertia,
			spEnergyPointPrescribedPositions prescribed_positions,
			spEnergyFrictionalContact contact
		);

		Id add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialSurface& material);
		int get_index(const Id& id) const;
		int get_n_surfaces() const;

		/* Fields */
		spPointDynamics dyn;
		spEnergyPointInertia inertia;
		spEnergyPointPrescribedPositions prescribed_positions;
		spEnergyTriangleStrain strain;
		spEnergyTriangleBendingGrinspun03 bending_grispun_03;
		spEnergyFrictionalContact contact;
		std::vector<int> global_indices;

		// Output
		MeshOutputGroups output_groups;
		std::vector<std::vector<std::array<int, 3>>> input_triangles;

		// Stark callbaks
		void _write_frame(stark::core::Stark& stark);
	};
}

#pragma once

#include "../../solver/Stark.h"
#include "MeshOutputGroups.h"
#include "Id.h"
#include "interval_types.h"
#include "PointDynamics.h"
#include "EnergyPointInertia.h"
#include "EnergyPointPrescribedPositions.h"
#include "EnergyTriangleStrain.h"
#include "EnergyTriangleBendingBergou06.h"
#include "EnergyEdgeStrain.h"
#include "EnergyFrictionalContact.h"


namespace stark::models
{
	/*
		This class is exposed to the user.
	*/
	class Shells
	{
	public:
		/* Definitions */
		struct Material
		{
			double density;
			double inertia_damping;
			double strain_young_modulus;
			double strain_poisson_ratio;
			double strain_damping;
			double strain_limit;
			double strain_limit_stiffness;
			double bending_stiffness;
			double bending_damping;
			double bending_cutoff_angle_deg;
			static Material cotton();
		};

		/* Methods */
		Shells(
			Stark& stark,
			spPointDynamics dyn, 
			spEnergyPointInertia inertia,
			spEnergyPointPrescribedPositions prescribed_positions,
			spEnergyTriangleStrain strain,
			spEnergyTriangleBendingBergou06 bending_bergou,
			spEnergyEdgeStrain edge_strain_limiting_and_damping,
			spEnergyFrictionalContact contact
		);

		Id add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const Material material);
		std::shared_ptr<PrescribedPointGroup> create_prescribed_positions_group(Id& id, const std::string label = "");
		std::shared_ptr<PrescribedPointGroupWithTransformation> create_prescribed_positions_group_with_transformation(Id& id, const std::string label = "");
		void add_to_group(const std::string label, Id& id);
		bool is_empty() const;
		int get_n_objects() const;

	private:
		/* Fields */
		spPointDynamics dyn;
		spEnergyPointInertia inertia;
		spEnergyPointPrescribedPositions prescribed_positions;
		spEnergyTriangleStrain strain;
		spEnergyTriangleBendingBergou06 bending_bergou;
		spEnergyEdgeStrain edge_strain_limiting_and_damping;
		spEnergyFrictionalContact contact;
		std::vector<int> global_indices;

		// Output
		MeshOutputGroups output_groups;  // local_indices
		std::vector<std::vector<std::array<int, 3>>> input_triangles;

		// Stark callbaks
		void _write_frame(Stark& stark);
	};
	using spShells = std::shared_ptr<Shells>;
}

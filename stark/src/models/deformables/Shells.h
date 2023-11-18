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
//#include "EnergyFrictionalContact.h"


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
			double bending_cutoff_angle_deg = std::numeric_limits<double>::max();
			static Material towel();
		};

		/* Methods */
		Shells(
			Stark& stark,
			spPointDynamics dyn, 
			spEnergyPointInertia inertia,
			spEnergyPointPrescribedPositions prescribed_positions,
			spEnergyTriangleStrain strain,
			spEnergyTriangleBendingBergou06 bending_bergou
			//spEnergyFrictionalContact contact
		);

		Id add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const Material material);
		std::shared_ptr<PrescribedPointGroup> create_prescribed_positions_group(Id& id, const std::string label = "");
		std::shared_ptr<PrescribedPointGroupWithTransformation> create_prescribed_positions_group_with_transformation(Id& id, const std::string label = "");
		void add_to_output_label(const std::string label, Id& id);
		bool is_empty() const;
		int get_n_objects() const;

	private:
		/* Fields */
		spPointDynamics dyn;
		spEnergyPointInertia inertia;
		spEnergyPointPrescribedPositions prescribed_positions;
		spEnergyTriangleStrain strain;
		spEnergyTriangleBendingBergou06 bending_bergou;
		//spEnergyFrictionalContact contact;
		std::vector<int> global_indices;

		// Output
		MeshOutputGroups output_groups;  // local_indices
		std::vector<std::vector<std::array<int, 3>>> input_triangles;

		// Stark callbaks
		void _write_frame(Stark& stark);
	};
	using spShells = std::shared_ptr<Shells>;
}

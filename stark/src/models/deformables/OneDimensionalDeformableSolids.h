#pragma once

#include "../../solver/Stark.h"
#include "MeshOutputGroups.h"
#include "Id.h"
#include "interval_types.h"
#include "PointDynamics.h"
#include "EnergyPointInertia.h"
#include "EnergyPointPrescribedPositions.h"
#include "EnergyEdgeStrain.h"
//#include "EnergyFrictionalContact.h"


namespace stark::models
{
	/* Definitions */
	struct OneDimensionalMaterial
	{
		double line_density = 0.0;
		double section_radius = 0.0;
		double inertia_damping = 0.0;
		double strain_young_modulus = 0.0;
		double strain_damping = 0.0;
		double strain_limit = 0.0;
		double strain_limit_stiffness = 0.0;
		static OneDimensionalMaterial sticky_goo();
	};

	/*
		This class is exposed to the user.
	*/
	class OneDimensionalDeformableSolids
	{
	public:

		/* Methods */
		OneDimensionalDeformableSolids(
			Stark& stark,
			spPointDynamics dyn, 
			spEnergyPointInertia inertia,
			spEnergyPointPrescribedPositions prescribed_positions
			//spEnergyFrictionalContact contact
		);

		Id add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 2>>& tets, const OneDimensionalMaterial& material);
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
		spEnergyEdgeStrain strain;
		//spEnergyFrictionalContact contact;
		std::vector<int> global_indices;

		// Output
		MeshOutputGroups output_groups;  // local_indices
		std::vector<std::vector<std::array<int, 2>>> input_edges;

		// Stark callbaks
		void _write_frame(Stark& stark);
	};
	using spOneDimensionalDeformableSolids = std::shared_ptr<OneDimensionalDeformableSolids>;
}

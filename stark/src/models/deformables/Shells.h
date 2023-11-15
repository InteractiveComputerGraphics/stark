#pragma once

#include "../../solver/Stark.h"
#include "../../solver/MeshWriter.h"
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
		enum class MaterialPresets { Cloth };

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
		MeshWriter mesh_writer;
		std::vector<std::vector<std::array<int, 3>>> input_triangles;
		std::vector<std::string, std::vector<int>> labeled_groups;  // {label: global_idx}

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

		Id add(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialPresets material = MaterialPresets::Cloth);
		bool is_empty() const;

	private:
		void _write_frame(Stark& stark);
	};
}

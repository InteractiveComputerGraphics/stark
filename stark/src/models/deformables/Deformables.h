#pragma once
#include <memory>

#include "DeformableSolidsLines.h"
#include "DeformableSolidsSurfaces.h"
#include "DeformableSolidsVolumes.h"
#include "../interactions/EnergyFrictionalContact.h"

#include "DeformableLineHandler.h"
#include "DeformableSurfaceHandler.h"
#include "DeformableVolumeHandler.h"


namespace stark::models
{
	class Deformables
	{
	public:
		/* Methods */
		Deformables(
			stark::core::Stark& stark,
			spPointDynamics dyn,
			spEnergyFrictionalContact contact
		);

		DeformableLineHandler add_line(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 2>>& segments, const MaterialLine& material);
		DeformableSurfaceHandler add_surface(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 3>>& triangles, const MaterialSurface& material);
		DeformableVolumeHandler add_volume(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int32_t, 4>>& tets, const MaterialVolume& material);

	private:
		std::shared_ptr<DeformableSolidsLines> lines;
		std::shared_ptr<DeformableSolidsSurfaces> surfaces;
		std::shared_ptr<DeformableSolidsVolumes> volumes;
	};
}

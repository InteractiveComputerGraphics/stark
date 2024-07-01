#pragma once
#include "../deformables/Deformables.h"
#include "../interactions/Interactions.h"

#include "deformables_preset_types.h"

namespace stark
{
	class DeformablesPresets
	{
	public:
		/* Methods */
		DeformablesPresets(core::Stark& stark, std::shared_ptr<Deformables> deformables, std::shared_ptr<Interactions> interactions);

		Line::Handler add_line(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& segments, const Line::Params& params);
		Line::VCH add_line_as_segments(const std::string& output_label, const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments, const Line::Params& params);

		Surface::Handler add_surface(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const Surface::Params& params);
		Surface::VCH add_surface_grid(const std::string& output_label, const Eigen::Vector2d& dim, const std::array<int, 2>& subdivisions, const Surface::Params& params);
		PrescribedSurface::Handler add_prescribed_surface(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const PrescribedSurface::Params& params);

		Volume::Handler add_volume(const std::string& output_label, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets, const Volume::Params& params);
		Volume::VCH add_volume_grid(const std::string& output_label, const Eigen::Vector3d& dim, const std::array<int, 3>& subdivisions, const Volume::Params& params);

	private:
		/* Fields */
		std::shared_ptr<Deformables> deformables;
		std::shared_ptr<Interactions> interactions;
	};
}

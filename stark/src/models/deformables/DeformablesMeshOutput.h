#pragma once
#include <vector>
#include <array>
#include <string>

#include <Eigen/Dense>

#include "../../core/Stark.h"
#include "../MeshOutputGroups.h"
#include "PointDynamics.h"
#include "PointSetHandler.h"


namespace stark
{
	class DeformablesMeshOutput
	{
	public:
		/* Methods */
		DeformablesMeshOutput(core::Stark& stark, spPointDynamics dyn);
		void add_point_set(const std::string& label, const PointSetHandler& set);
		void add_point_set(const std::string& label, const PointSetHandler& set, const std::vector<int>& conn);
		void add_segment_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 2>>& conn);
		void add_segment_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 2>>& conn, const std::vector<int>& point_set_map);
		void add_triangle_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 3>>& conn);
		void add_triangle_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 3>>& conn, const std::vector<int>& point_set_map);
		void add_tet_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 4>>& conn);
		void add_tet_mesh(const std::string& label, const PointSetHandler& set, const std::vector<std::array<int, 4>>& conn, const std::vector<int>& point_set_map);

	private:
		/* Fields */
		spPointDynamics dyn;

		// Custom set + connectivity struct
		template<std::size_t N>
		struct Mesh
		{
			PointSetHandler set;  // Node set for the deformable
			std::vector<std::array<int, N>> conn; // Local connectivity (e.g. the surface nodes, not the tet nodes)
			std::vector<int> point_set_map; // E.g. mapping from surface indices to tet indices
		};

		// Connectivities
		std::vector<Mesh<1>> points;
		std::vector<Mesh<2>> segments;
		std::vector<Mesh<3>> triangles;
		std::vector<Mesh<4>> tets;

		// Groups
		MeshOutputGroups point_groups;
		MeshOutputGroups segment_groups;
		MeshOutputGroups triangle_groups;
		MeshOutputGroups tet_groups;

		// Buffers
		std::vector<Eigen::Vector3d> vertices;
		std::vector<std::array<int, 1>> conn_points;
		std::vector<std::array<int, 2>> conn_segments;
		std::vector<std::array<int, 3>> conn_triangles;
		std::vector<std::array<int, 4>> conn_tets;

		/* Methods */
		void _write_frame(stark::core::Stark& stark);

		template<std::size_t N>
		void _write(stark::core::Stark& stark, const stark::MeshOutputGroups& groups, const std::vector<stark::DeformablesMeshOutput::Mesh<N>>& meshes, std::vector<std::array<int, N>>& conn_buffer);
	};
}

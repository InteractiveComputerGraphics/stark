#pragma once
#include <vector>
#include <array>
#include <string>

#include <Eigen/Dense>

#include "../../core/Stark.h"
#include "../MeshOutputGroups.h"
#include "RigidBodyDynamics.h"
#include "RigidBodyHandler.h"


namespace stark
{
	class RigidBodiesMeshOutput
	{
	public:
		/* Methods */
		RigidBodiesMeshOutput(core::Stark& stark, spRigidBodyDynamics rb);
		void add_point_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices);
		void add_segment_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& conn);
		void add_triangle_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& conn);
		void add_tet_mesh(const std::string& label, const RigidBodyHandler& rb, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& conn);

	private:
		/* Fields */
		spRigidBodyDynamics rb;

		// Custom set + connectivity struct
		template<std::size_t N>
		struct Mesh
		{
			RigidBodyHandler rb;
			std::vector<Eigen::Vector3d> vertices;
			std::vector<std::array<int, N>> conn;
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
		void _write_frame(core::Stark& stark);

		template<std::size_t N>
		void _write(core::Stark& stark, const MeshOutputGroups& groups, const std::vector<Mesh<N>>& meshes, std::vector<std::array<int, N>>& conn_buffer);
	};
}

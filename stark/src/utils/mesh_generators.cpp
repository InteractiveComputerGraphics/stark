#include "mesh_generators.h"
#include <limits>

#include <par_shapes/par_shapes.h>

#include "include.h"


stark::Mesh<3> as_mesh(par_shapes_mesh* pm)
{
	stark::Mesh<3> mesh;

	mesh.vertices.resize(pm->npoints);
	for (int i = 0; i < pm->npoints; i++) {
		mesh.vertices[i] = { pm->points[3*i], pm->points[3*i + 1], pm->points[3*i + 2] };
	}

	mesh.conn.resize(pm->ntriangles);
	for (int i = 0; i < pm->ntriangles; i++) {
		mesh.conn[i] = { pm->triangles[3*i], pm->triangles[3*i + 1], pm->triangles[3*i + 2] };
	}

	par_shapes_free_mesh(pm);
	return mesh;
}


stark::Mesh<3> stark::make_sphere(const double radius, const int subdivisions)
{
	Mesh m = as_mesh(par_shapes_create_subdivided_sphere(subdivisions));
	scale(m.vertices, radius);
	return m;
}
stark::Mesh<3> stark::make_box(const Eigen::Vector3d& size, const bool lid)
{
	Mesh<3> m = as_mesh(par_shapes_create_cube());
	if (!lid) {
		int cursor = 0;
		for (int i = 0; i < (int)m.conn.size(); i++) {
			const std::array<int, 3>& tri = m.conn[i];

			bool is_lid = true;
			for (int j = 0; j < 3; j++) {
				const Eigen::Vector3d& p = m.vertices[tri[j]];
				if (p[2] < 1.0 - std::numeric_limits<double>::epsilon()) {
					is_lid = false;
					break;
				}
			}

			if (!is_lid) {
				m.conn[cursor] = tri;
				cursor++;
			}
		}
		m.conn.resize(cursor);
	}
	move(m.vertices, {-0.5, -0.5, -0.5});
	scale(m.vertices, size);
	return m;
}
stark::Mesh<3> stark::make_box(const double size, const bool lid)
{
	return make_box({size, size, size}, lid);
}
stark::Mesh<3> stark::make_cylinder(const double radius, const double full_height, const int slices, const int stacks)
{
	const std::array<float, 3> c_t = {0, 0, 1};
	const std::array<float, 3> c_b = {0, 0, 0};
	const std::array<float, 3> n = {0, 0, 1};

	par_shapes_mesh* pc = par_shapes_create_cylinder(slices, stacks);
	par_shapes_mesh* pd_top = par_shapes_create_disk(1, slices, c_t.data(), n.data());
	par_shapes_mesh* pd_bottom = par_shapes_create_disk(1, slices, c_b.data(), n.data());
	par_shapes_merge_and_free(pc, pd_top);
	par_shapes_merge_and_free(pc, pd_bottom);
	pc = par_shapes_weld(pc, 10.0f*std::numeric_limits<float>::epsilon(), nullptr);

	Mesh m = as_mesh(pc);
	move(m.vertices, {0.0, 0.0, -0.5});
	scale(m.vertices, {radius, radius, full_height });
	return m;
}
stark::Mesh<3> stark::make_torus(const double outer_radius, const double inner_radius, const int slices, const int stacks)
{
	Mesh m = as_mesh(par_shapes_create_torus(slices, stacks, (float)(inner_radius/outer_radius)));
	auto [v, t] = clean_triangle_mesh(m.vertices, m.conn, /* merge_by_distance = */inner_radius/(double)slices);
	m.vertices = v;
	m.conn = t;
	scale(m.vertices, outer_radius);
	return m;
}
stark::Mesh<3> stark::make_knot(const double scale_, const double inner_radius, const int slices, const int stacks)
{
	Mesh m = as_mesh(par_shapes_create_trefoil_knot(slices, stacks, (float)(inner_radius/scale_)));
	scale(m.vertices, scale_);
	return m;
}

void stark::generate_triangle_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const Eigen::Vector2d& center, const Eigen::Vector2d& dimensions, const std::array<int, 2>& n_quads_per_dim, const double z)
{
	const Eigen::Vector2d bottom = center - 0.5 * dimensions;
	const Eigen::Vector2d top = center + 0.5 * dimensions;
	assert(bottom[0] <= top[0] && bottom[1] <= top[1]);

	// Precomputation
	const int nx = n_quads_per_dim[0] + 1;
	const int ny = n_quads_per_dim[1] + 1;
	const int n_points = nx * ny;
	const int n_quadrilaterals = n_quads_per_dim[0] * n_quads_per_dim[1];
	const int n_triangles = 2 * n_quadrilaterals;
	const double dx = (top[0] - bottom[0]) / (double)n_quads_per_dim[0];
	const double dy = (top[1] - bottom[1]) / (double)n_quads_per_dim[1];

	// Points
	/* Correspond to a quadrilateral following the pattern:
			[[0, 0],
			 [0, 1],
			 [1, 0],
			 [1, 1]]

		move y -> move x
	*/
	out_vertices.resize(n_points);
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			const int idx = ny * i + j;
			out_vertices[idx] = { bottom[0] + i * dx, bottom[1] + j * dy, z };
		}
	}

	// Connectivity
	/* Corresponds to a splited quadrilateral. Resulting triangles with
	   homogeneous normals
			[[0, 2, 3],
			 [0, 3, 1]]

		The relation between an element "coordinate" in the grid
		and its vertices global indices is:

			node_idx = ny*(ei + ni) + (ej + nj)

		where n_xyz is the number of nodes in the xyz dimension, e_ijk
		is the element "coordinate" in the ijk (grid) dimension and n_ijk
		is the local node "coordinate" within the hexahedron.
	*/
	out_connectivity.clear();
	out_connectivity.reserve(n_triangles);
	int tri_i = 0;
	for (int ei = 0; ei < n_quads_per_dim[0]; ei++) {
		for (int ej = 0; ej < n_quads_per_dim[1]; ej++) {
			int nodes_idx[] = { ny * (ei + 0) + (ej + 0),
							   ny * (ei + 0) + (ej + 1),
							   ny * (ei + 1) + (ej + 0),
							   ny * (ei + 1) + (ej + 1) };

			if (ei % 2 == ej % 2) {
				out_connectivity.push_back({ nodes_idx[0], nodes_idx[2], nodes_idx[3] });
				out_connectivity.push_back({ nodes_idx[0], nodes_idx[3], nodes_idx[1] });
			}
			else {
				out_connectivity.push_back({ nodes_idx[0], nodes_idx[2], nodes_idx[1] });
				out_connectivity.push_back({ nodes_idx[2], nodes_idx[3], nodes_idx[1] });
			}
		}
	}
}
stark::Mesh<3> stark::generate_triangle_grid(const Eigen::Vector2d& center, const Eigen::Vector2d& dimensions, const std::array<int, 2>& n_quads_per_dim, const double z)
{
	Mesh<3> mesh;
	generate_triangle_grid(mesh.vertices, mesh.conn, center, dimensions, n_quads_per_dim, z);
	return mesh;
}
void stark::generate_cylindrical_triangle_mesh(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, double radius, double height, const std::array<int, 2>& n_quads_per_dim)
{
	// Number of quads along the circumference:
	const int n_quads_circumference = n_quads_per_dim[0];
	// Number of quads along the height:
	const int n_quads_height = n_quads_per_dim[1];

	// Number of vertices in the angular direction (no duplicate for wrap)
	// and in the height direction (we need +1 to accommodate quads along height):
	const int nx = n_quads_circumference;           // around the circle
	const int ny = n_quads_height + 1;              // along the height

	// Total number of points:
	const int n_points = nx * ny;

	// Angular step for each segment around the circle:
	const double dtheta = 2.0 * M_PI / static_cast<double>(n_quads_circumference);
	// Step in height:
	const double dz = height / static_cast<double>(n_quads_height);

	// Generate vertices
	out_vertices.resize(n_points);
	for (int i = 0; i < nx; ++i)
	{
		// Current angle in [0, 2π)
		const double theta = i * dtheta;
		// Precompute cos/sin
		const double c = std::cos(theta);
		const double s = std::sin(theta);

		for (int j = 0; j < ny; ++j)
		{
			// z goes from 0 up to height
			const double z_coord = j * dz;
			// Global index for this vertex
			const int idx = i * ny + j;

			// Map from (theta, z) to Cartesian
			out_vertices[idx] = Eigen::Vector3d(radius * c, radius * s, z_coord);
		}
	}

	// Each "cell" is a quadrilateral split into two triangles.
	// We'll have:
	//   - n_quads_circumference * n_quads_height total quads
	//   - 2 * (n_quads_circumference * n_quads_height) triangles
	out_connectivity.clear();
	out_connectivity.reserve(2 * n_quads_circumference * n_quads_height);

	for (int i = 0; i < n_quads_circumference; ++i)
	{
		// Next in the angular direction (wrapped)
		const int ip1 = (i + 1) % nx;

		for (int j = 0; j < n_quads_height; ++j)
		{
			// The four corner indices of the quad:
			// node0 = (i, j),   node1 = (i, j+1)
			// node2 = (i+1, j), node3 = (i+1, j+1)
			const int node0 = i * ny + j;
			const int node1 = i * ny + (j + 1);
			const int node2 = ip1 * ny + j;
			const int node3 = ip1 * ny + (j + 1);

			// Optionally, alternate the diagonal based on parity
			if ((i + j) % 2 == 0)
			{
				// First triangle:  (node0, node2, node3)
				// Second triangle: (node0, node3, node1)
				out_connectivity.push_back({ node0, node2, node3 });
				out_connectivity.push_back({ node0, node3, node1 });
			}
			else
			{
				// First triangle:  (node0, node2, node1)
				// Second triangle: (node2, node3, node1)
				out_connectivity.push_back({ node0, node2, node1 });
				out_connectivity.push_back({ node2, node3, node1 });
			}
		}
	}
}

stark::Mesh<3> stark::generate_cylindrical_triangle_mesh(double radius, double height, const std::array<int, 2>& n_quads_per_dim)
{
	stark::Mesh<3> mesh;
	generate_cylindrical_triangle_mesh(mesh.vertices, mesh.conn, radius, height, n_quads_per_dim);
	return mesh;
}

void stark::generate_tet_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 4>>& out_tets, const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const std::array<int, 3>& n_quads_per_dim)
{
	const Eigen::Vector3d bottom = center - 0.5 * dimensions;
	const Eigen::Vector3d top = center + 0.5 * dimensions;
	assert(bottom[0] <= top[0] && bottom[1] <= top[1] && bottom[2] <= top[2]);

	// Precomputation
	const int nx = n_quads_per_dim[0] + 1;
	const int ny = n_quads_per_dim[1] + 1;
	const int nz = n_quads_per_dim[2] + 1;
	const int n_points = nx * ny * nz;
	const int n_hexas = n_quads_per_dim[0] * n_quads_per_dim[1] * n_quads_per_dim[2];
	const int n_tetras = 12 * n_hexas;
	const double dx = (top[0] - bottom[0]) / (double)n_quads_per_dim[0];
	const double dy = (top[1] - bottom[1]) / (double)n_quads_per_dim[1];
	const double dz = (top[2] - bottom[2]) / (double)n_quads_per_dim[2];

	const int nx_hex = n_quads_per_dim[0];
	const int ny_hex = n_quads_per_dim[1];
	const int nz_hex = n_quads_per_dim[2];

	const double dx_half = 0.5 * dx;
	const double dy_half = 0.5 * dy;
	const double dz_half = 0.5 * dz;

	// Points
	/* Correspond to a hexahedron following the pattern:
			[[0, 0, 0],
				[0, 0, 1],
				[0, 1, 0],
				[0, 1, 1],
				[1, 0, 0],
				[1, 0, 1],
				[1, 1, 0],
				[1, 1, 1]]

		move z -> move y -> move x
	*/
	out_vertices.resize(n_points + n_hexas);
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int k = 0; k < nz; k++) {
				out_vertices[nz * ny * i + nz * j + k] = { bottom[0] + i * dx, bottom[1] + j * dy, bottom[2] + k * dz };
			}
		}
	}

	// Add the hex center points
	const int center_offset = n_points;
	for (int i = 0; i < nx_hex; i++) {
		for (int j = 0; j < ny_hex; j++) {
			for (int k = 0; k < nz_hex; k++) {
				out_vertices[center_offset + nz_hex * ny_hex * i + nz_hex * j + k] = { bottom[0] + i * dx + dx_half, bottom[1] + j * dy + dy_half, bottom[2] + k * dz + dz_half };
			}
		}
	}

	out_tets.reserve(n_tetras);
	for (int ei = 0; ei < nx_hex; ei++) {
		for (int ej = 0; ej < ny_hex; ej++) {
			for (int ek = 0; ek < nz_hex; ek++) {
				int nodes_idx[] = { nz * ny * (ei + 0) + nz * (ej + 0) + (ek + 0),
									nz * ny * (ei + 0) + nz * (ej + 0) + (ek + 1),
									nz * ny * (ei + 0) + nz * (ej + 1) + (ek + 0),
									nz * ny * (ei + 0) + nz * (ej + 1) + (ek + 1),
									nz * ny * (ei + 1) + nz * (ej + 0) + (ek + 0),
									nz * ny * (ei + 1) + nz * (ej + 0) + (ek + 1),
									nz * ny * (ei + 1) + nz * (ej + 1) + (ek + 0),
									nz * ny * (ei + 1) + nz * (ej + 1) + (ek + 1),
									center_offset + nz_hex * ny_hex * ei + nz_hex * ej + ek };

				if (((ek % 2 == 0) && (ei % 2 == ej % 2)) || ((ek % 2 == 1) && (ei % 2 != ej % 2))) {
					// Front
					out_tets.push_back({ nodes_idx[0], nodes_idx[1], nodes_idx[4], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[1], nodes_idx[5], nodes_idx[4], nodes_idx[8] });
					// Left
					out_tets.push_back({ nodes_idx[0], nodes_idx[2], nodes_idx[1], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[1], nodes_idx[2], nodes_idx[3], nodes_idx[8] });
					// Bottom
					out_tets.push_back({ nodes_idx[0], nodes_idx[4], nodes_idx[6], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[0], nodes_idx[6], nodes_idx[2], nodes_idx[8] });

					// Back
					out_tets.push_back({ nodes_idx[3], nodes_idx[2], nodes_idx[7], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[2], nodes_idx[6], nodes_idx[7], nodes_idx[8] });
					// Right
					out_tets.push_back({ nodes_idx[4], nodes_idx[5], nodes_idx[7], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[4], nodes_idx[7], nodes_idx[6], nodes_idx[8] });
					// Top
					out_tets.push_back({ nodes_idx[1], nodes_idx[3], nodes_idx[5], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[3], nodes_idx[7], nodes_idx[5], nodes_idx[8] });
				}
				else {
					// Front
					out_tets.push_back({ nodes_idx[0], nodes_idx[1], nodes_idx[5], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[0], nodes_idx[5], nodes_idx[4], nodes_idx[8] });
					// Left
					out_tets.push_back({ nodes_idx[0], nodes_idx[3], nodes_idx[1], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[0], nodes_idx[2], nodes_idx[3], nodes_idx[8] });
					// Bottom
					out_tets.push_back({ nodes_idx[0], nodes_idx[4], nodes_idx[2], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[2], nodes_idx[4], nodes_idx[6], nodes_idx[8] });

					// Back
					out_tets.push_back({ nodes_idx[3], nodes_idx[2], nodes_idx[6], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[3], nodes_idx[6], nodes_idx[7], nodes_idx[8] });
					// Right
					out_tets.push_back({ nodes_idx[5], nodes_idx[7], nodes_idx[6], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[5], nodes_idx[6], nodes_idx[4], nodes_idx[8] });
					// Top
					out_tets.push_back({ nodes_idx[1], nodes_idx[3], nodes_idx[7], nodes_idx[8] });
					out_tets.push_back({ nodes_idx[1], nodes_idx[7], nodes_idx[5], nodes_idx[8] });
				}
			}
		}
	}
}
stark::Mesh<4> stark::generate_tet_grid(const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const std::array<int, 3>& n_quads_per_dim)
{
	Mesh<4> mesh;
	generate_tet_grid(mesh.vertices, mesh.conn, center, dimensions, n_quads_per_dim);
	return mesh;
}
void stark::generate_segment_line(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 2>>& out_connectivity, const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments)
{
	out_vertices.resize(n_segments + 1);
	out_connectivity.resize(n_segments);

	for (int i = 0; i <= n_segments; i++) {
		out_vertices[i] = begin + (end - begin) * (i / (double)n_segments);
	}
	for (int i = 0; i < n_segments; i++) {
		out_connectivity[i] = { i, i + 1 };
	}
}
stark::Mesh<2> stark::generate_segment_line(const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments)
{
	Mesh<2> mesh;
	generate_segment_line(mesh.vertices, mesh.conn, begin, end, n_segments);
	return mesh;
}

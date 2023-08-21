#include "mesh_utils.h"

#include <algorithm>
#include <random>

#include <JanBenderUtilities/OBJLoader.h>

// Tools
void push_back_if_not_present(std::vector<int>& v, const int value)
{
	if (std::find(v.begin(), v.end(), value) == v.end()) {
		v.push_back(value);
	}
}
double generate_random_double(double l, int seed) {
	// Create a random number generator
	std::random_device rd;
	std::mt19937 gen(seed);
	std::uniform_real_distribution<double> dist(-l, l);

	// Generate a random number between zero and l
	return dist(gen);
}

// ========================================================================================================

double stark::utils::deg2rad(const double deg)
{
	return 2.0 * PI * (deg / 360.0);
}

double stark::utils::rad2deg(const double rad)
{
	return rad * 180.0 / PI;
}

void stark::utils::load_obj(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_triangles, const std::string path)
{
	std::vector<std::array<float, 3>> vertices;
	std::vector<std::array<float, 3>> normals;
	std::vector<std::array<float, 2>> texture_coordinates;
	std::vector<JanBenderUtilities::OBJMeshFaceIndices> faces;
	JanBenderUtilities::OBJLoader::loadObj(path, &vertices, &faces, &normals, &texture_coordinates, { 1.0f, 1.0f, 1.0f });

	out_vertices.reserve(vertices.size());
	for (const std::array<float, 3> &point : vertices) {
		out_vertices.push_back({ (double)point[0], (double)point[1], (double)point[2] });
	}

	out_triangles.reserve(faces.size());
	for (const auto& face : faces) {
		out_triangles.push_back({ face.posIndices[0] - 1, face.posIndices[1] - 1, face.posIndices[2] - 1 });
	}
}

Eigen::Vector3d stark::utils::triangle_normal(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
	return (p0 - p2).cross(p1 - p2).normalized();
}
double stark::utils::triangle_area(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
	return 0.5 * (p0 - p2).cross(p1 - p2).norm();
}
double stark::utils::unsigned_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3)
{
	return (1.0 / 6.0) * ((p1 - p0).cross(p2 - p0)).dot(p3 - p0);
}
double stark::utils::signed_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3)
{
	return std::abs(unsigned_tetra_volume(p0, p1, p2, p3));
}

void stark::utils::find_edges(std::vector<std::array<int, 2>>& out_edges, const std::vector<std::array<int, 3>>& triangles, const int n_nodes)
{
	out_edges.reserve(3 * triangles.size());
	for (const std::array<int, 3>&triangle : triangles) {
		out_edges.push_back({ std::min(triangle[0], triangle[1]), std::max(triangle[0], triangle[1]) });
		out_edges.push_back({ std::min(triangle[1], triangle[2]), std::max(triangle[1], triangle[2]) });
		out_edges.push_back({ std::min(triangle[2], triangle[0]), std::max(triangle[2], triangle[0]) });
	}
	std::sort(out_edges.begin(), out_edges.end(), [&](const std::array<int, 2>& a, const std::array<int, 2>& b) { return a[0] * n_nodes + a[1] < b[0] * n_nodes + b[1]; });
	auto end_unique = std::unique(out_edges.begin(), out_edges.end());
	out_edges.erase(end_unique, out_edges.end());
}
void stark::utils::find_node_node_map_simplex(std::vector<std::vector<int>>& output, const int32_t* connectivity, const int32_t n_simplices, const int32_t n_nodes_per_simplex, const int32_t n_nodes)
{
	output.resize(n_nodes);
	for (int simplex_i = 0; simplex_i < n_simplices; simplex_i++) {
		const int begin = n_nodes_per_simplex * simplex_i;
		for (int i = 0; i < n_nodes_per_simplex; i++) {
			const int node_i = connectivity[begin + i];
			for (int j = i + 1; j < n_nodes_per_simplex; j++) {
				const int node_j = connectivity[begin + j];
				push_back_if_not_present(output[node_i], node_j);
				push_back_if_not_present(output[node_j], node_i);
			}
		}
	}
}
void stark::utils::find_internal_angles(std::vector<std::array<int, 4>>& internal_angles, const std::vector<std::array<int, 3>>& triangles, const int n_nodes)
{
	/*
		For each edge in the triangle mesh, find the 2 nodes that are common neighbors of both edge end-points.
	*/
	internal_angles.clear();
	if (triangles.size() == 0) {
		return;
	}

	std::vector<std::vector<int>> node_node_map;
	find_node_node_map_simplex(node_node_map, &triangles[0][0], (int)triangles.size(), 3, n_nodes);
	for (std::vector<int>& nodes : node_node_map) {
		std::sort(nodes.begin(), nodes.end());  // std::set_intersection assumes sorted
	}

	std::vector<std::array<int, 2>> edges;
	find_edges(edges, triangles, n_nodes);

	std::vector<int> buffer;
	internal_angles.reserve(edges.size());
	for (int edge_i = 0; edge_i < (int)edges.size(); edge_i++) {
		const std::array<int, 2>& edge = edges[edge_i];
		const std::vector<int>& neighs_i = node_node_map[edge[0]];
		const std::vector<int>& neighs_j = node_node_map[edge[1]];
		buffer.clear();
		std::set_intersection(neighs_i.begin(), neighs_i.end(), neighs_j.begin(), neighs_j.end(), std::back_inserter(buffer));
		if (buffer.size() == 2) {
			internal_angles.push_back({edge[0], edge[1], buffer[0], buffer[1]});
		}
		else if (buffer.size() > 2) {
			std::cout << "Stark error: triangle mesh has edges with more than two incident triangles." << std::endl;
			exit(-1);
		}
	}
}

void stark::utils::compute_node_normals(std::vector<Eigen::Vector3d>& output, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles)
{
	output.resize(vertices.size(), Eigen::Vector3d::Zero());
	for (const std::array<int, 3> &triangle : triangles) {
		const Eigen::Vector3d normal = triangle_normal(vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]);
		const double area = triangle_area(vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]);
		output[triangle[0]] += area*normal;
		output[triangle[1]] += area*normal;
		output[triangle[2]] += area*normal;
	}
	for (Eigen::Vector3d& normal : output) {
		normal.normalize();
	}
}
void stark::utils::generate_triangular_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const Eigen::Vector2d& bottom, const Eigen::Vector2d& top, const std::array<int, 2>& n_quads_per_dim, const bool randomize, const double z)
{
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

			if (randomize && i != 0 && i != (nx - 1) && j != 0 && j != (ny - 1)) {
				out_vertices[idx][0] += generate_random_double(dx*0.2, idx);
				out_vertices[idx][1] += generate_random_double(dy*0.2, idx);
			}
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
std::array<int, 2> stark::utils::generate_triangular_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const double x_length, const double y_length, const int n_short_side, const bool randomize, const double z)
{
	int nx = 0;
	int ny = 0;
	if (x_length < y_length) {
		nx = n_short_side;
		ny = (int)((y_length/x_length)*(double)nx);
	}
	else {
		ny = n_short_side;
		nx = (int)((y_length/x_length)*(double)ny);
	}
	generate_triangular_grid(out_vertices, out_connectivity, { -0.5*x_length, -0.5*y_length }, { 0.5*x_length, 0.5*y_length }, {nx, ny}, randomize, z);
	return {nx, ny};
}
void stark::utils::write_VTK(const std::string path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const bool generate_normals)
{
	vtkio::VTKFile vtk_file;

	if (vertices.size() == 0) {
		vtk_file.write_empty(path);
	}
	else {
		vtk_file.set_points_from_twice_indexable(vertices);
		vtk_file.set_cells_from_twice_indexable(triangles, vtkio::CellType::Triangle);
		if (generate_normals) {
			std::vector<Eigen::Vector3d> normals;
			compute_node_normals(normals, vertices, triangles);
			vtk_file.set_point_data_from_twice_indexable("normals", normals, vtkio::AttributeType::Vectors);
		}
		vtk_file.write(path);
	}
}

void stark::utils::move(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& translation)
{
	for (Eigen::Vector3d& point : points) {
		point += translation;
	}
}
void stark::utils::rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis)
{
	Eigen::Matrix3d R = Eigen::AngleAxis<double>(deg2rad(angle), axis.normalized()).toRotationMatrix();
	for (Eigen::Vector3d& point : points) {
		point = R * point;
	}
}
void stark::utils::rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot)
{
	move(points, -pivot);
	rotate_deg(points, angle, axis);
	move(points, pivot);
}
void stark::utils::scale(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& scale)
{
	for (Eigen::Vector3d& point : points) {
		point = scale.cwiseProduct(point);
	}
}
void stark::utils::scale(std::vector<Eigen::Vector3d>& points, const double s)
{
	scale(points, { s, s, s });
}
void stark::utils::mirror(std::vector<Eigen::Vector3d>& points, const int dim, const double pivot)
{
	for (Eigen::Vector3d& point : points) {
		const double dist = point[dim] - pivot;
		point[dim] = pivot - dist;
	}
}

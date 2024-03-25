#include "mesh_utils.h"

#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <random>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader.h"

#include "unordered_array_set_and_map.h"

// Tools
void push_back_if_not_present(std::vector<int>& v, const int value)
{
	if (std::find(v.begin(), v.end(), value) == v.end()) {
		v.push_back(value);
	}
}
double generate_random_double(double l, int seed) {
	// Create a random number generator
	std::mt19937 gen(seed);
	std::uniform_real_distribution<double> dist(-l, l);

	// Generate a random number between zero and l
	return dist(gen);
}
bool is_outward_facing(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& tetCenter) {
	Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0);
	Eigen::Vector3d faceCenter = (v0 + v1 + v2) / 3.0;
	Eigen::Vector3d toCenter = tetCenter - faceCenter;
	return normal.dot(toCenter) < 0; // outward if dot product is negative
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

std::vector<stark::utils::Mesh<3>> stark::utils::load_obj(const std::string& path)
{
    std::vector<stark::utils::Mesh<3>> tri_meshes;

    // Initialise tinyobjloader objects and read the file
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str());

    // Input checking
    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        exit(1);
    }

    // Global information
    const int total_n_vertices = (int)attrib.vertices.size() / 3;

    // Write the geometric information into individual triangular meshes
    // Loop over meshes
    for (int shape_i = 0; shape_i < shapes.size(); shape_i++) {

        // Initialize individual triangular mesh
        Mesh<3> tri_mesh;
        tri_mesh.conn.resize(shapes[shape_i].mesh.num_face_vertices.size());
        std::vector<bool> global_nodes_present(total_n_vertices, false);

        // Loop over triangles
        int index_offset = 0;
        for (int tri_i = 0; tri_i < shapes[shape_i].mesh.num_face_vertices.size(); tri_i++) {
            if (shapes[shape_i].mesh.num_face_vertices[tri_i] != 3) {
                std::cout << "learnSPH error: readTriMeshesFromObj can only read triangle meshes." << std::endl;
            }

            // Gather triangle global indices
            std::array<int, 3> triangle_global_indices;
            for (int vertex_i = 0; vertex_i < 3; vertex_i++) {
                tinyobj::index_t idx = shapes[shape_i].mesh.indices[(int)(3 * tri_i + vertex_i)];
                const int global_vertex_index = idx.vertex_index;
                triangle_global_indices[vertex_i] = global_vertex_index;
                global_nodes_present[global_vertex_index] = true;
            }
            tri_mesh.conn.push_back(triangle_global_indices);
        }

        // Reduce global indexes to local indexes
        std::vector<int> global_to_local_vertex_idx(total_n_vertices, -1);
        int local_vertices_count = 0;
        for (int global_vertex_i = 0; global_vertex_i < total_n_vertices; global_vertex_i++) {
            if (global_nodes_present[global_vertex_i]) {
                // Map global -> local
                global_to_local_vertex_idx[global_vertex_i] = local_vertices_count;
                local_vertices_count++;

                // Add vertex to the local mesh vertex vector
                tinyobj::real_t vx = attrib.vertices[(int)(3 * global_vertex_i + 0)];
                tinyobj::real_t vy = attrib.vertices[(int)(3 * global_vertex_i + 1)];
                tinyobj::real_t vz = attrib.vertices[(int)(3 * global_vertex_i + 2)];
                tri_mesh.vertices.push_back({ vx, vy, vz });
            }
        }

        // Change triangle indices
        for (int tri_i = 0; tri_i < tri_mesh.conn.size(); tri_i++) {
            for (int vertex_i = 0; vertex_i < 3; vertex_i++) {
                tri_mesh.conn[tri_i][vertex_i] = global_to_local_vertex_idx[tri_mesh.conn[tri_i][vertex_i]];
            }
        }

        tri_meshes.push_back(tri_mesh);
    }

	return tri_meshes;
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
	find_edges_from_simplices(edges, triangles, n_nodes);

	std::vector<int> buffer;
	internal_angles.reserve(edges.size());
	for (int edge_i = 0; edge_i < (int)edges.size(); edge_i++) {
		const std::array<int, 2>& edge = edges[edge_i];
		const std::vector<int>& neighs_i = node_node_map[edge[0]];
		const std::vector<int>& neighs_j = node_node_map[edge[1]];
		buffer.clear();
		std::set_intersection(neighs_i.begin(), neighs_i.end(), neighs_j.begin(), neighs_j.end(), std::back_inserter(buffer));
		if (buffer.size() == 2) {
			internal_angles.push_back({ edge[0], edge[1], buffer[0], buffer[1] });
		}
		else if (buffer.size() > 2) {
			std::cout << "Stark error: triangle mesh has edges with more than two incident triangles." << std::endl;
			exit(-1);
		}
	}
}
void stark::utils::find_perimeter_edges(std::vector<std::array<int, 2>>& perimeter_edges, const std::vector<std::array<int, 3>>& triangles)
{
	utils::unordered_array_map<int, 2, int> edge_count;
	for (const auto& triangle : triangles) {
		edge_count[{ std::min(triangle[0], triangle[1]), std::max(triangle[0], triangle[1])} ]++;
		edge_count[{ std::min(triangle[1], triangle[2]), std::max(triangle[1], triangle[2])} ]++;
		edge_count[{ std::min(triangle[2], triangle[0]), std::max(triangle[2], triangle[0])} ]++;
	}

	perimeter_edges.clear();
	for (const auto& [edge, count] : edge_count) {
		if (count == 1) { // Edge appears only once, so it's a perimeter edge
			perimeter_edges.push_back(edge);
		}
	}
}
void stark::utils::extract_surface(std::vector<std::array<int, 3>>& out_triangles, std::vector<int>& out_triangle_to_tet_node_map, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	out_triangles.clear();
	out_triangle_to_tet_node_map.clear();

	// Store the faces (with sorted connectivity) that occur only once and map them to their corresponding tet index
	utils::unordered_array_map<int, 3, int> unique_face_tet_map;

	for (int tet_i = 0; tet_i < (int)tets.size(); tet_i++) {
		const std::array<int, 4>& tet = tets[tet_i];
		std::array<std::array<int, 3>, 4> faces = { {
			{{tet[0], tet[1], tet[2]}},
			{{tet[0], tet[1], tet[3]}},
			{{tet[0], tet[2], tet[3]}},
			{{tet[1], tet[2], tet[3]}}
		} };

		for (auto& face : faces) {
			std::sort(face.begin(), face.end());

			// If face exist, remove it from the list. Otherwise add it with the tet index.
			auto it = unique_face_tet_map.find(face);
			if (it == unique_face_tet_map.end()) {
				unique_face_tet_map[face] = tet_i;
			}
			else {
				unique_face_tet_map.erase(face);
			}
		}
	}

	// New connectivity + Face winding
	std::vector<std::array<int, 3>> unique_triangles; 
	unique_triangles.reserve(unique_face_tet_map.size());
	for (const auto& it : unique_face_tet_map) {
		std::array<int, 3> face = it.first;

		// Face winding to point outwards from the tet
		const std::array<int, 4>& tet = tets[it.second];
		const Eigen::Vector3d center = (vertices[tet[0]] + vertices[tet[1]] + vertices[tet[2]] + vertices[tet[3]]) / 4.0;
		if (is_outward_facing(vertices[face[0]], vertices[face[1]], vertices[face[2]], center)) {
			std::swap(face[0], face[1]);
		}

		unique_triangles.push_back(face);
	}

	// Reduce connectivity to a full, smaller mesh
	reduce_connectivity(out_triangles, out_triangle_to_tet_node_map, unique_triangles, (int)vertices.size());
}
std::tuple<std::vector<std::array<int, 3>>, std::vector<int>> stark::utils::extract_surface(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	std::vector<std::array<int, 3>> out_triangles;
	std::vector<int> out_triangle_to_tet_node_map;
	extract_surface(out_triangles, out_triangle_to_tet_node_map, vertices, tets);
	return { out_triangles, out_triangle_to_tet_node_map };
}
void stark::utils::find_sharp_edges(std::vector<std::array<int, 2>>& out_edges, std::vector<int>& out_old_to_new_map, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, double angle_deg_threshold)
{
	std::vector<std::array<int, 2>> edges;
	std::vector<std::array<int, 4>> internal_angles;
	find_internal_angles(internal_angles, triangles, (int)vertices.size());

	const double cos_angle_rad_threshold = std::cos(deg2rad(angle_deg_threshold));
	edges.reserve(internal_angles.size());
	for (const std::array<int, 4>& angle : internal_angles) {
		const Eigen::Vector3d& p0 = vertices[angle[0]];
		const Eigen::Vector3d& p1 = vertices[angle[1]];
		const Eigen::Vector3d& p2 = vertices[angle[2]];
		const Eigen::Vector3d& p3 = vertices[angle[3]];

		const Eigen::Vector3d n0 = triangle_normal(p0, p1, p2);
		const Eigen::Vector3d n1 = triangle_normal(p1, p0, p3);
		if (n0.dot(n1) < cos_angle_rad_threshold) {
			edges.push_back({ angle[0], angle[1] });
		}
	}

	reduce_connectivity(out_edges, out_old_to_new_map, edges, (int)vertices.size());
}
std::tuple<std::vector<std::array<int, 2>>, std::vector<int>> stark::utils::find_sharp_edges(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, double angle_deg_threshold)
{
	std::vector<std::array<int, 2>> out_edges;
	std::vector<int> out_old_to_new_map;
	find_sharp_edges(out_edges, out_old_to_new_map, vertices, triangles, angle_deg_threshold);
	return { out_edges, out_old_to_new_map };
}




double stark::utils::total_volume(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	double volume = 0.0;
	for (const std::array<int, 4>&tet : tets) {
		volume += unsigned_tetra_volume(vertices[tet[0]], vertices[tet[1]], vertices[tet[2]], vertices[tet[3]]);
	}
	return volume;
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

void stark::utils::generate_triangle_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_connectivity, const Eigen::Vector2d& center, const Eigen::Vector2d& dimensions, const std::array<int, 2>& n_quads_per_dim, const bool randomize, const double z)
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
stark::utils::Mesh<3> stark::utils::generate_triangle_grid(const Eigen::Vector2d& center, const Eigen::Vector2d& dimensions, const std::array<int, 2>& n_quads_per_dim, const bool randomize, const double z)
{
	Mesh<3> mesh;
	generate_triangle_grid(mesh.vertices, mesh.conn, center, dimensions, n_quads_per_dim, randomize, z);
	return mesh;
}
void stark::utils::generate_tet_grid(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 4>>& out_tets, const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const std::array<int, 3>& n_quads_per_dim)
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
stark::utils::Mesh<4> stark::utils::generate_tet_grid(const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const std::array<int, 3>& n_quads_per_dim)
{
	Mesh<4> mesh;
	generate_tet_grid(mesh.vertices, mesh.conn, center, dimensions, n_quads_per_dim);
	return mesh;
}

void stark::utils::generate_segment_line(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 2>>& out_connectivity, const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments)
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

stark::utils::Mesh<2> stark::utils::generate_segment_line(const Eigen::Vector3d& begin, const Eigen::Vector3d& end, const int n_segments)
{
	Mesh<2> mesh;
	generate_segment_line(mesh.vertices, mesh.conn, begin, end, n_segments);
	return mesh;
}

void stark::utils::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	vtkio::VTKFile vtk_file;

	if (vertices.size() == 0) {
		vtk_file.write_empty(path);
	}
	else {
		vtk_file.set_points_from_twice_indexable(vertices);
		vtk_file.set_cells_from_twice_indexable(tets, vtkio::CellType::Tetra);
		vtk_file.write(path);
	}
}
void stark::utils::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const bool generate_normals)
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
void stark::utils::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& edges)
{
	vtkio::VTKFile vtk_file;

	if (vertices.size() == 0) {
		vtk_file.write_empty(path);
	}
	else {
		vtk_file.set_points_from_twice_indexable(vertices);
		vtk_file.set_cells_from_twice_indexable(edges, vtkio::CellType::Line);
		vtk_file.write(path);
	}
}
void stark::utils::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 1>>& points)
{
	vtkio::VTKFile vtk_file;

	if (vertices.size() == 0) {
		vtk_file.write_empty(path);
	}
	else {
		vtk_file.set_points_from_twice_indexable(vertices);
		vtk_file.set_cells_as_particles(vertices.size());
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

Eigen::Vector3d stark::utils::rotate_deg(const Eigen::Vector3d& point, const Eigen::Matrix3d& R, const Eigen::Vector3d& pivot)
{
	const Eigen::Vector3d p_shifted = point - pivot;
	const Eigen::Vector3d p_shifted_rotated = R*p_shifted;
	const Eigen::Vector3d p_rotated = p_shifted_rotated + pivot;
	return p_rotated;
}

#include "include.h"

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
bool is_outward_facing(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& tetCenter) {
	Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0);
	Eigen::Vector3d faceCenter = (v0 + v1 + v2) / 3.0;
	Eigen::Vector3d toCenter = tetCenter - faceCenter;
	return normal.dot(toCenter) < 0; // outward if dot product is negative
}
// ========================================================================================================

double stark::deg2rad(const double deg)
{
	return 2.0 * M_PI * (deg / 360.0);
}
double stark::rad2deg(const double rad)
{
	return rad * 180.0 / M_PI;
}

std::vector<stark::Mesh<3>> stark::load_obj(const std::string& path)
{
    std::vector<stark::Mesh<3>> tri_meshes;

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
void stark::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
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
void stark::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const bool generate_normals)
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
void stark::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 2>>& edges)
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
void stark::write_VTK(const std::string& path, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 1>>& points)
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

Eigen::Vector3d stark::triangle_normal(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
	return (p0 - p2).cross(p1 - p2).normalized();
}
double stark::triangle_area(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
	return 0.5 * (p0 - p2).cross(p1 - p2).norm();
}
double stark::unsigned_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3)
{
	return std::abs(signed_tetra_volume(p0, p1, p2, p3));
}
double stark::signed_tetra_volume(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3)
{
	return (1.0 / 6.0) * ((p1 - p0).cross(p2 - p0)).dot(p3 - p0);
}

void stark::find_node_node_map_simplex(std::vector<std::vector<int>>& output, const int32_t* connectivity, const int32_t n_simplices, const int32_t n_nodes_per_simplex, const int32_t n_nodes)
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
void stark::find_internal_angles(std::vector<std::array<int, 4>>& internal_angles, const std::vector<std::array<int, 3>>& triangles, const int n_nodes)
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
void stark::find_perimeter_edges(std::vector<std::array<int, 2>>& out_perimeter_edges, std::vector<int>& out_edge_to_triangle_node_map, const std::vector<std::array<int, 3>>& triangles, const int n_nodes)
{
	unordered_array_map<int, 2, int> edge_count;
	for (const auto& triangle : triangles) {
		edge_count[{ std::min(triangle[0], triangle[1]), std::max(triangle[0], triangle[1])} ]++;
		edge_count[{ std::min(triangle[1], triangle[2]), std::max(triangle[1], triangle[2])} ]++;
		edge_count[{ std::min(triangle[2], triangle[0]), std::max(triangle[2], triangle[0])} ]++;
	}

	std::vector<std::array<int, 2>> perimeter_edges;
	for (const auto& [edge, count] : edge_count) {
		if (count == 1) { // Edge appears only once, so it's a perimeter edge
			perimeter_edges.push_back(edge);
		}
	}

	reduce_connectivity(out_perimeter_edges, out_edge_to_triangle_node_map, perimeter_edges, n_nodes);
}
std::tuple<std::vector<std::array<int, 2>>, std::vector<int>> stark::find_perimeter_edges(const std::vector<std::array<int, 3>>& triangles, const int n_nodes)
{
	std::vector<std::array<int, 2>> edges;
	std::vector<int> edge_to_triangle_node_map;
	find_perimeter_edges(edges, edge_to_triangle_node_map, triangles, n_nodes);
	return std::make_tuple(edges, edge_to_triangle_node_map);
}
void stark::find_surface(std::vector<std::array<int, 3>>& out_triangles, std::vector<int>& out_triangle_to_tet_node_map, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	out_triangles.clear();
	out_triangle_to_tet_node_map.clear();

	// Store the faces (with sorted connectivity) that occur only once and map them to their corresponding tet index
	unordered_array_map<int, 3, int> unique_face_tet_map;

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
std::tuple<std::vector<std::array<int, 3>>, std::vector<int>> stark::find_surface(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	std::vector<std::array<int, 3>> out_triangles;
	std::vector<int> out_triangle_to_tet_node_map;
	find_surface(out_triangles, out_triangle_to_tet_node_map, vertices, tets);
	return { out_triangles, out_triangle_to_tet_node_map };
}
void stark::clean_triangle_mesh(std::vector<Eigen::Vector3d>& out_vertices, std::vector<std::array<int, 3>>& out_triangles, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double merge_by_distance)
{
	// Quantize all vertices to a grid with cell size equal to merge_by_distance
	if (merge_by_distance > 0.0) {

		// AABB
		Eigen::AlignedBox3d bbox;
		for (const Eigen::Vector3d& v : vertices) {
			bbox.extend(v);
		}

		// Check dimensions
		double max_n_cells_double = bbox.diagonal().maxCoeff() / merge_by_distance;
		if (max_n_cells_double > (double)std::numeric_limits<int>::max()) {
			std::cout << "Stark error: clean_triangle_mesh merge_by_distance is too small." << std::endl;
			exit(-1);
		}

		// Quantize
		unordered_array_map<int, 3, int> unique_vertices;  // {cell_ijk: new_vertex_idx}
		std::vector<int> old_to_new_map(vertices.size(), -1);
		for (size_t old_i = 0; old_i < vertices.size(); ++old_i) {

			const std::array<int, 3> ijk = { 
				(int)((vertices[old_i].x() - bbox.min().x())/merge_by_distance),
				(int)((vertices[old_i].y() - bbox.min().y())/merge_by_distance),
				(int)((vertices[old_i].z() - bbox.min().z())/merge_by_distance)
			};

			int new_idx = -1;
			auto it = unique_vertices.find(ijk);
			if (it == unique_vertices.end()) {
				new_idx = out_vertices.size();
				out_vertices.push_back(vertices[old_i]);
				unique_vertices[ijk] = new_idx;
			}
			else {
				new_idx = it->second;
			}
			old_to_new_map[old_i] = new_idx;
		}

		// Update triangles
		out_triangles = apply_map(triangles, old_to_new_map);
	}
	else {
		out_vertices = vertices;
		out_triangles = triangles;
	}

	// Remove duplicated and degenerated triangles
	unordered_array_map<int, 3, int> unique_triangles;  //  {sorted_idx_triangle: old_triangle_idx}  We need this to conserve winding
	for (int tri_i = 0; tri_i < (int)out_triangles.size(); tri_i++) {
		std::array<int, 3> tri = out_triangles[tri_i];
		if (tri[0] != tri[1] && tri[0] != tri[2] && tri[1] != tri[2]) {
			std::sort(tri.begin(), tri.end());
			unique_triangles[tri] = tri_i;
		}
	}
	const std::vector<std::array<int, 3>> prev_triangles = out_triangles;
	out_triangles.clear();
	for (const auto& unique_tri : unique_triangles) {
		out_triangles.push_back(prev_triangles[unique_tri.second]);
	}
}
std::tuple<std::vector<Eigen::Vector3d>, std::vector<std::array<int, 3>>> stark::clean_triangle_mesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, const double merge_by_distance)
{
	std::vector<Eigen::Vector3d> out_vertices;
	std::vector<std::array<int, 3>> out_triangles;
	clean_triangle_mesh(out_vertices, out_triangles, vertices, triangles, merge_by_distance);
	return { out_vertices, out_triangles };
}
void stark::find_sharp_edges(std::vector<std::array<int, 2>>& out_edges, std::vector<int>& out_old_to_new_map, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, double angle_deg_threshold)
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
std::tuple<std::vector<std::array<int, 2>>, std::vector<int>> stark::find_sharp_edges(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles, double angle_deg_threshold)
{
	std::vector<std::array<int, 2>> out_edges;
	std::vector<int> out_old_to_new_map;
	find_sharp_edges(out_edges, out_old_to_new_map, vertices, triangles, angle_deg_threshold);
	return { out_edges, out_old_to_new_map };
}


double stark::total_volume(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 4>>& tets)
{
	double volume = 0.0;
	for (const std::array<int, 4>&tet : tets) {
		volume += unsigned_tetra_volume(vertices[tet[0]], vertices[tet[1]], vertices[tet[2]], vertices[tet[3]]);
	}
	return volume;
}
void stark::compute_node_normals(std::vector<Eigen::Vector3d>& output, const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int, 3>>& triangles)
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


void stark::move(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& translation)
{
	for (Eigen::Vector3d& point : points) {
		point += translation;
	}
}
void stark::rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis)
{
	Eigen::Matrix3d R = Eigen::AngleAxis<double>(deg2rad(angle), axis.normalized()).toRotationMatrix();
	for (Eigen::Vector3d& point : points) {
		point = R * point;
	}
}
void stark::rotate_deg(std::vector<Eigen::Vector3d>& points, const double angle, const Eigen::Vector3d& axis, const Eigen::Vector3d& pivot)
{
	move(points, -pivot);
	rotate_deg(points, angle, axis);
	move(points, pivot);
}
void stark::scale(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& scale)
{
	for (Eigen::Vector3d& point : points) {
		point = scale.cwiseProduct(point);
	}
}
void stark::scale(std::vector<Eigen::Vector3d>& points, const double s)
{
	scale(points, { s, s, s });
}
void stark::mirror(std::vector<Eigen::Vector3d>& points, const int dim, const double pivot)
{
	for (Eigen::Vector3d& point : points) {
		const double dist = point[dim] - pivot;
		point[dim] = pivot - dist;
	}
}

Eigen::Vector3d stark::rotate_deg(const Eigen::Vector3d& point, const Eigen::Matrix3d& R, const Eigen::Vector3d& pivot)
{
	const Eigen::Vector3d p_shifted = point - pivot;
	const Eigen::Vector3d p_shifted_rotated = R*p_shifted;
	const Eigen::Vector3d p_rotated = p_shifted_rotated + pivot;
	return p_rotated;
}

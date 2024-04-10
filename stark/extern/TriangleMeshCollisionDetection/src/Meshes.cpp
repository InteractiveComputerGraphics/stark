#include "Meshes.h"

#include <algorithm>

// ==========================================================================================================
#include <cstdint>
#include <array>
#include <unordered_set>

template<std::size_t N, typename INT_TYPE = int>
struct ArrayHasher {
	std::size_t operator()(const std::array<INT_TYPE, N>& a) const {
		std::size_t h = 0;
		for (auto e : a) {
			h ^= std::hash<INT_TYPE>{}(e)+0x9e3779b9 + (h << 6) + (h >> 2);
		}
		return h;
	}
};
template<typename INT_TYPE, std::size_t N>
using unordered_array_set = std::unordered_set<std::array<INT_TYPE, N>, ArrayHasher<N, INT_TYPE>>;
// ==========================================================================================================



const std::array<int32_t, 2> tmcd::internals::Meshes::get_edge_connectivity(const int32_t set, const int32_t idx) const
{
	const int32_t* e = &this->set_edges[set][2 * idx];
	return { e[0], e[1] };
}

const std::array<int32_t, 3> tmcd::internals::Meshes::get_triangle_connectivity(const int32_t set, const int32_t idx) const
{
	const int32_t* t = &this->set_triangles[set][3 * idx];
	return { t[0], t[1], t[2] };
}

tmcd::Vec3d tmcd::internals::Meshes::get_coords(const int32_t set, const int32_t idx) const
{
	const double* v = &this->set_xm[set][3 * idx];
	return Vec3d(v[0], v[1], v[2]);
}

int32_t tmcd::internals::Meshes::get_total_n_points() const
{
	return (int32_t)this->points_offsets.back();
}

int32_t tmcd::internals::Meshes::get_total_n_edges() const
{
	return (int32_t)this->edges_offsets.back();
}

int32_t tmcd::internals::Meshes::get_total_n_triangles() const
{
	return (int32_t)this->triangles_offsets.back();
}

int32_t tmcd::internals::Meshes::get_n_meshes() const
{
	return (int32_t)this->set_triangles.size();
}

std::array<int32_t, 2> tmcd::internals::Meshes::get_vertices_range(const int32_t set) const
{
	return { this->points_offsets[set], this->points_offsets[set + 1] };
}
std::array<int32_t, 2> tmcd::internals::Meshes::get_triangles_range(const int32_t set) const
{
	return { this->triangles_offsets[set], this->triangles_offsets[set + 1] };
}
std::array<int32_t, 2> tmcd::internals::Meshes::get_edges_range(const int32_t set) const
{
	return { this->edges_offsets[set], this->edges_offsets[set + 1] };
}

tmcd::info::Meshes tmcd::internals::Meshes::get_info() const
{
	info::Meshes info;
	info.n_meshes = this->get_n_meshes();
	info.n_points = this->get_total_n_points();
	info.n_edges = this->get_total_n_edges();
	info.n_triangles = this->get_total_n_triangles();
	return info;
}

int32_t tmcd::internals::Meshes::add_mesh(const double* x0, const double* x1, const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges)
{
	// Check for mesh correctness: Unique non-degenerate elements
	//// Triangles
	unordered_array_set<int32_t, 3> unique_triangles;
	for (int i = 0; i < n_triangles; i++) {
		const int32_t* t = &triangles[3 * i];
		if (t[0] == t[1] || t[0] == t[2] || t[1] == t[2]) {
			std::cout << "TriangleMeshCollisionDetection error: degenerate triangle found in mesh." << std::endl;
			exit(-1);
		}
		std::array<int32_t, 3> t_sorted = { t[0], t[1], t[2] };
		std::sort(t_sorted.begin(), t_sorted.end());
		unique_triangles.insert({ t_sorted[0], t_sorted[1], t_sorted[2] });
	}
	if (unique_triangles.size() != n_triangles) {
		std::cout << "TriangleMeshCollisionDetection error: non-unique triangle found in mesh." << std::endl;
		exit(-1);
	}

	//// Edges
	unordered_array_set<int32_t, 2> unique_edges;
	for (int i = 0; i < n_edges; i++) {
		const int32_t* e = &edges[2 * i];
		if (e[0] == e[1]) {
			std::cout << "TriangleMeshCollisionDetection error: degenerate edge found in mesh." << std::endl;
			exit(-1);
		}
		std::array<int32_t, 2> e_sorted = { e[0], e[1] };
		std::sort(e_sorted.begin(), e_sorted.end());
		unique_edges.insert({ e_sorted[0], e_sorted[1] });
	}
	if (unique_edges.size() != n_edges) {
		std::cout << "TriangleMeshCollisionDetection error: non-unique edge found in mesh." << std::endl;
		exit(-1);
	}

	// Add
	const int mesh_id = this->get_n_meshes();

	if (mesh_id == 0) {
		this->triangles_offsets.push_back(0);
		this->edges_offsets.push_back(0);
		this->points_offsets.push_back(0);
	}

	this->set_triangles.push_back(triangles);
	this->n_triangles_in_set.push_back(n_triangles);
	this->triangles_offsets.push_back(this->triangles_offsets.back() + n_triangles);

	this->set_edges.push_back(edges);
	this->n_edges_in_set.push_back(n_edges);
	this->edges_offsets.push_back(this->edges_offsets.back() + n_edges);

	this->set_x0.push_back(x0);
	this->set_x1.push_back(x1);
	this->set_xm.push_back(xm);
	this->n_points_in_set.push_back(n_vertices);
	this->points_offsets.push_back(this->points_offsets.back() + n_vertices);

	return mesh_id;
}

void tmcd::internals::Meshes::clear()
{
	this->set_triangles.clear();
	this->n_triangles_in_set.clear();
	this->triangles_offsets.clear();
	this->set_edges.clear();
	this->n_edges_in_set.clear();
	this->edges_offsets.clear();
	this->set_x0.clear();
	this->set_x1.clear();
	this->set_xm.clear();
	this->n_points_in_set.clear();
	this->points_offsets.clear();
}

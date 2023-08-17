#include "Meshes.h"

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

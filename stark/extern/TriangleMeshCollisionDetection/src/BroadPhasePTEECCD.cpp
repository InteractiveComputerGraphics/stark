#include "BroadPhasePTEECCD.h"

int32_t tmcd::BroadPhasePTEECCD::add_mesh(const double* x0, const double* x1, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges)
{
	return this->meshes.add_mesh(x0, x1, nullptr, n_vertices, triangles, n_triangles, edges, n_edges);
}

const tmcd::BroadPhasePTEEResults& tmcd::BroadPhasePTEECCD::run(const BroadPhaseStrategy strat)
{
	return this->_run(/* is_ccd = */ false, /* enlargement = */ 0.0, strat);
}

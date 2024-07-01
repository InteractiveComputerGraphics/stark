#include "BroadPhasePTEE.h"

int32_t tmcd::BroadPhasePTEE::add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges)
{
	return this->meshes.add_mesh(nullptr, nullptr, xm, n_vertices, triangles, n_triangles, edges, n_edges);
}

const tmcd::BroadPhasePTEEResults& tmcd::BroadPhasePTEE::run(const double enlargement, const BroadPhaseStrategy strat)
{
	return this->_run(/* is_ccd = */ false, enlargement, strat);
}

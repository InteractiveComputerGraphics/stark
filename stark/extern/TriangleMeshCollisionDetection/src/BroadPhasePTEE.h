#pragma once
#include <vector>
#include <array>

#include "BroadPhasePTEEBase.h"


namespace tmcd
{
	class BroadPhasePTEE
		: public internals::BroadPhasePTEEBase
	{
	public:
		/* Methods */
		BroadPhasePTEE() = default;
		~BroadPhasePTEE() = default;

		int32_t add_mesh(const double* xm, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges);
		const BroadPhasePTEEResults& run(const double enlargement, const BroadPhaseStrategy strat = BroadPhaseStrategy::OctreeSIMD);
	};
}

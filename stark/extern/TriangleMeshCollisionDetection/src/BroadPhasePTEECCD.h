#pragma once
#include <vector>
#include <array>

#include "BroadPhasePTEEBase.h"


namespace tmcd
{
	class BroadPhasePTEECCD
		: public internals::BroadPhasePTEEBase
	{
	public:
		/* Methods */
		BroadPhasePTEECCD() = default;
		~BroadPhasePTEECCD() = default;

		int32_t add_mesh(const double* x0, const double* x1, const int32_t n_vertices, const int32_t* triangles, const int32_t n_triangles, const int32_t* edges, const int32_t n_edges);
		const BroadPhasePTEEResults& run(const BroadPhaseStrategy strat = BroadPhaseStrategy::OctreeSIMD);
	};
}
